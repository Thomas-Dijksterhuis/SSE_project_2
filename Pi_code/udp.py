import argparse
import datetime
import json
import queue
import socket
import struct
import threading
import traceback
import wave
from collections import deque
from pathlib import Path

import numpy as np
import psycopg2
import matplotlib.pyplot as plt

plot_initialized = False
plot_line = None
plot_fig = None
plot_ax = None



# ---------------------------------------------------------------------------
# Config
# ---------------------------------------------------------------------------

with open("PI_code/credentials.json") as f:
    credentials = json.load(f)

SAMPLE_RATE      = credentials["recording"]["sample_rate"]
SAVE_SECONDS     = credentials["recording"]["save_seconds"]
CHANNELS         = credentials["recording"]["channels"]
SAMPLE_WIDTH     = credentials["recording"]["sample_width_bytes"]

SAMPLES_PER_FILE = SAMPLE_RATE * SAVE_SECONDS
FILE_DURATION    = datetime.timedelta(seconds=SAVE_SECONDS)

RAW_SAVE_SECONDS     = 20
RAW_SAMPLES_PER_FILE = SAMPLE_RATE * RAW_SAVE_SECONDS

PLOT_SECONDS = 1

plotLeftBuffer = deque(maxlen=SAMPLE_RATE * PLOT_SECONDS)
plotRightBuffer = deque(maxlen=SAMPLE_RATE * PLOT_SECONDS)

plotLock = threading.Lock()

LIVE_BUFFER_MS            = 300
LIVE_BUFFER_SAMPLES       = max(1, int(SAMPLE_RATE * LIVE_BUFFER_MS / 1000))
LIVE_PLAYBACK_FRAME_MS    = 256
LIVE_PLAYBACK_FRAME_SAMPLES = max(1, int(SAMPLE_RATE * LIVE_PLAYBACK_FRAME_MS / 1000))
LIVE_MIXER_BUFFER         = max(256, min(1024, LIVE_PLAYBACK_FRAME_SAMPLES * 2))

RECORDINGS_DIR = Path(__file__).parent / "recordings"
RECORDINGS_DIR.mkdir(parents=True, exist_ok=True)

# NLMS
M = 64
MU = 0.25

# ---------------------------------------------------------------------------
# Per-device state
# ---------------------------------------------------------------------------

class DeviceState:
    """All mutable state scoped to a single device ID."""

    def __init__(self, m: int = M, mu: float = MU):
        self.lock              = threading.Lock()
        self.recorded_samples  = deque()
        self.file_start_ts     = None
        self.last_sequence     = None

        self.input_samples     = deque()
        self.desired_samples   = deque()
        self.input_file_start_ts = None

        # NLMS
        self.m = m
        self.mu = mu
        self.w = np.zeros(self.m)
        self.x = np.zeros(self.m)
        self.idx = 0

    def nlms_step(self, u: float, d: float) -> float:
        self.x[self.idx] = u
        self.idx = (self.idx + 1) % len(self.x)

        x = np.roll(self.x, -self.idx)

        y = np.dot(self.w, x)
        e = d - y

        norm = np.dot(x, x) + 1e-8
        self.w += (self.mu / norm) * e * x

        return e


_device_states: dict[str, DeviceState] = {}
_device_states_lock = threading.Lock()


def get_device_state(device_id: str) -> DeviceState:
    with _device_states_lock:
        if device_id not in _device_states:
            _device_states[device_id] = DeviceState(M, MU)
        return _device_states[device_id]

# ---------------------------------------------------------------------------
# Database worker
# ---------------------------------------------------------------------------

db_queue: queue.Queue = queue.Queue()

def plotWorker():
    plt.ion()

    fig, ax = plt.subplots(figsize=(14, 6))

    x = np.linspace(
        -PLOT_SECONDS,
        0,
        SAMPLE_RATE * PLOT_SECONDS
    )

    ax.set_xlabel("Time (s)")

    leftLine, = ax.plot(x, np.zeros_like(x), label="Peizo")
    rightLine, = ax.plot(x, np.zeros_like(x), label="Microphone")

    ax.set_title("Received Stereo Audio")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Amplitude")

    ax.set_xlim(-PLOT_SECONDS, 0)

    y_lim = 1.5/4
    # Bigger range
    ax.set_ylim(y_lim, -y_lim)

    ax.grid(True)
    ax.legend()

    while True:

        with plotLock:

            left = np.array(plotLeftBuffer)
            right = np.array(plotRightBuffer)

        if len(left):

            leftData = np.zeros(len(x))
            rightData = np.zeros(len(x))

            leftData[-len(left):] = left
            rightData[-len(right):] = right

            leftLine.set_ydata(leftData)
            rightLine.set_ydata(rightData)

        fig.canvas.draw_idle()
        fig.canvas.flush_events()

        plt.pause(0.05)

def _make_db_connection():
    db = credentials["database"]
    return psycopg2.connect(
        dbname   = db["db_name"],
        user     = db["user"],
        password = db["password"],
        host     = db["host"],
        port     = db["Port"],
    )


def _save_wav(conn, device_id: str, samples: list, start_ts: datetime.datetime, stop_ts: datetime.datetime):
    if not samples:
        return

    clipped = np.clip(np.asarray(samples, dtype=np.float64), -1.0, 1.0)
    pcm     = (clipped * 32767).astype(np.int16)

    date_dir = RECORDINGS_DIR / device_id / str(start_ts.date())
    date_dir.mkdir(parents=True, exist_ok=True)

    out_path = date_dir / f"{start_ts.strftime('%H%M%S')}_{stop_ts.strftime('%H%M%S')}.wav"

    with wave.open(str(out_path), "wb") as wf:
        wf.setnchannels(CHANNELS)
        wf.setsampwidth(SAMPLE_WIDTH)
        wf.setframerate(SAMPLE_RATE)
        wf.writeframes(pcm.tobytes())

    with conn.cursor() as cur:
        cur.execute(
            """
            INSERT INTO audio_recordings (device_id, start_time, stop_time, path)
            VALUES (%s, %s, %s, %s)
            """,
            (device_id, start_ts, stop_ts, str(out_path)),  # pass datetime objects directly
        )
    conn.commit()

    print(f"[{device_id}] Saved {len(samples)} samples → {out_path}")


def db_worker():
    """Single writer thread owns the DB connection — no locking needed."""
    conn = _make_db_connection()
    while True:
        device_id, samples, start_ts, stop_ts = db_queue.get()
        try:
            _save_wav(conn, device_id, samples, start_ts, stop_ts)
        except Exception as e:
            print("Save error:", e)
            traceback.print_exc()
            # Attempt reconnect on next item
            try:
                conn.close()
            except Exception:
                pass
            try:
                conn = _make_db_connection()
            except Exception as reconnect_err:
                print("DB reconnect failed:", reconnect_err)
        finally:
            db_queue.task_done()


threading.Thread(target=db_worker, daemon=True).start()


def save_wav(device_id: str, samples: list, start_ts: datetime.datetime, stop_ts: datetime.datetime):
    """Enqueue a save task (non-blocking, copies the sample list)."""
    db_queue.put((device_id, list(samples), start_ts, stop_ts))


def save_input_desired_wav(device_id: str, input_samples: list, desired_samples: list, start_ts: datetime.datetime, stop_ts: datetime.datetime):
    """Save the original left/right channels as separate mono input/desired WAV files."""
    if not input_samples and not desired_samples:
        return

    frame_count = min(len(input_samples), len(desired_samples))
    if frame_count <= 0:
        return

    input_pcm = (np.clip(np.asarray(input_samples[:frame_count], dtype=np.float64), -1.0, 1.0) * 32767).astype(np.int16)
    desired_pcm = (np.clip(np.asarray(desired_samples[:frame_count], dtype=np.float64), -1.0, 1.0) * 32767).astype(np.int16)

    date_dir = RECORDINGS_DIR / device_id / str(start_ts.date())
    date_dir.mkdir(parents=True, exist_ok=True)

    (date_dir / "input").mkdir(parents=True, exist_ok=True)
    (date_dir / "desired").mkdir(parents=True, exist_ok=True)

    input_path = date_dir / "input" / f"{start_ts.strftime('%H%M%S')}_{stop_ts.strftime('%H%M%S')}_input.wav"
    desired_path = date_dir / "desired" / f"{start_ts.strftime('%H%M%S')}_{stop_ts.strftime('%H%M%S')}_desired.wav"

    with wave.open(str(input_path), "wb") as wf:
        wf.setnchannels(1)
        wf.setsampwidth(SAMPLE_WIDTH)
        wf.setframerate(SAMPLE_RATE)
        wf.writeframes(input_pcm.tobytes())

    with wave.open(str(desired_path), "wb") as wf:
        wf.setnchannels(1)
        wf.setsampwidth(SAMPLE_WIDTH)
        wf.setframerate(SAMPLE_RATE)
        wf.writeframes(desired_pcm.tobytes())

    print(f"[{device_id}] Saved {frame_count} input/desired frames → {input_path} and {desired_path}")

# ---------------------------------------------------------------------------
# Live playback
# ---------------------------------------------------------------------------

livePlaybackEnabled = threading.Event()
saveInputDesiredEnabled = threading.Event()
livePlaybackQueue: queue.Queue = queue.Queue(maxsize=100)
livePendingSamples: deque      = deque()
livePendingLock                = threading.Lock()


def enqueueLiveSamples(samples: list):
    if not livePlaybackEnabled.is_set():
        return

    with livePendingLock:
        livePendingSamples.extend(samples)

        while len(livePendingSamples) >= LIVE_PLAYBACK_FRAME_SAMPLES:
            chunk = [livePendingSamples.popleft() for _ in range(LIVE_PLAYBACK_FRAME_SAMPLES)]
            try:
                livePlaybackQueue.put_nowait(chunk)
            except queue.Full:
                print("Live playback buffer full, dropping audio chunk")


def livePlaybackWorker():
    try:
        import pygame

        pygame.mixer.pre_init(
            frequency = SAMPLE_RATE,
            size      = -16,
            channels  = 1,
            buffer    = LIVE_MIXER_BUFFER,
        )
        pygame.mixer.init()
    except Exception as e:
        print("Live listen disabled:", e)
        return

    print(
        f"Live listen enabled — {LIVE_BUFFER_MS} ms start buffer "
        f"({LIVE_BUFFER_SAMPLES} samples), {LIVE_PLAYBACK_FRAME_MS} ms chunks"
    )

    mixer_channels = (pygame.mixer.get_init() or (None, None, 1))[2]
    liveChannel     = None
    primedSamples   = []
    primedCount     = 0
    primed          = False

    while True:
        chunk = livePlaybackQueue.get()
        try:
            pcm = np.clip(np.asarray(chunk, dtype=np.float64), -1.0, 1.0)
            pcm = (pcm * 32767).astype(np.int16)

            audio = pcm if mixer_channels == 1 else np.column_stack([pcm] * mixer_channels)

            if not primed:
                primedSamples.append(audio)
                primedCount += len(audio)

                if primedCount < LIVE_BUFFER_SAMPLES:
                    continue

                # We have enough — flush accumulated frames then mark primed
                audio  = np.concatenate(primedSamples, axis=0)
                primedSamples.clear()
                primedCount = 0
                primed = True

            sound = pygame.sndarray.make_sound(audio)

            if liveChannel is None or not liveChannel.get_busy():
                liveChannel = sound.play()
            else:
                liveChannel.queue(sound)
        except Exception as e:
            print("Live playback error:", e)
        finally:
            livePlaybackQueue.task_done()

# ---------------------------------------------------------------------------
# Packet handling
# ---------------------------------------------------------------------------

HEADER_SIZE = 46  # <16s 24s H I>


def _parse_timestamp(raw: bytes) -> datetime.datetime | None:
    text = raw.decode("utf-8", errors="ignore").rstrip("\x00").strip()
    if not text:
        return None
    for fmt in (None, "%Y-%m-%d %H:%M:%S.%f"):
        try:
            return datetime.datetime.fromisoformat(text) if fmt is None else datetime.datetime.strptime(text, fmt)
        except Exception:
            continue
    return None


def handle_packet(payload: bytes, message_time: datetime.datetime):
    if len(payload) < HEADER_SIZE:
        return

    device_id_raw, timestamp_raw, length, sequence = struct.unpack_from("<16s24sHI", payload, 0)

    device_id     = device_id_raw.decode("utf-8", errors="ignore").rstrip("\x00").strip() or "unknown"
    device_ts     = _parse_timestamp(timestamp_raw)
    effective_ts  = device_ts or message_time

    state = get_device_state(device_id)

    with state.lock:
        # --- Sleep / flush signal ---
        if length == 0:
            if state.recorded_samples:
                chunk       = list(state.recorded_samples)
                flush_start = state.file_start_ts
                state.recorded_samples.clear()
                state.file_start_ts = None
                save_wav(device_id, chunk, flush_start, effective_ts)
                print(f"[{device_id}] Sleep signal — flushed {len(chunk)} samples")

            if saveInputDesiredEnabled.is_set() and state.input_samples and state.desired_samples:
                input_chunk = list(state.input_samples)
                desired_chunk = list(state.desired_samples)
                input_flush_start = state.input_file_start_ts or effective_ts
                state.input_samples.clear()
                state.desired_samples.clear()
                state.input_file_start_ts = None
                save_input_desired_wav(device_id, input_chunk, desired_chunk, input_flush_start, effective_ts)
                print(f"[{device_id}] Sleep signal — flushed {min(len(input_chunk), len(desired_chunk))} input/desired frames")

            state.last_sequence = None
            return

        # --- Validate size ---
        expected_size = HEADER_SIZE + length
        if len(payload) != expected_size:
            print(f"[{device_id}] Bad packet size: got {len(payload)}, expected {expected_size}")
            return

        # --- Sequence check ---
        if state.last_sequence is not None:
            expected_seq = (state.last_sequence + 1) & 0xFFFF_FFFF
            if sequence != expected_seq:
                lost = (sequence - expected_seq) & 0xFFFF_FFFF
                print(f"[{device_id}] Packet loss: {lost} packet(s) missing")
        state.last_sequence = sequence

        # --- Decode samples ---
        raw_samples = np.frombuffer(payload[HEADER_SIZE:], dtype=np.int16).astype(np.float64)
        raw_samples /= 32768.0

        # Left = primary (desired), Right = reference/noise — verify against your wiring
        left  = raw_samples[0::2]
        right = raw_samples[1::2]

        with plotLock:
            plotLeftBuffer.extend(left)
            plotRightBuffer.extend(right)

        if saveInputDesiredEnabled.is_set():
            if state.input_file_start_ts is None:
                state.input_file_start_ts = effective_ts

            state.input_samples.extend(left.tolist())
            state.desired_samples.extend(right.tolist())

            while len(state.input_samples) >= RAW_SAMPLES_PER_FILE and len(state.desired_samples) >= RAW_SAMPLES_PER_FILE:
                input_chunk = [state.input_samples.popleft() for _ in range(RAW_SAMPLES_PER_FILE)]
                desired_chunk = [state.desired_samples.popleft() for _ in range(RAW_SAMPLES_PER_FILE)]
                save_input_desired_wav(device_id, input_chunk, desired_chunk, state.input_file_start_ts, effective_ts)
                state.input_file_start_ts = effective_ts

        cleaned_chunk = []
        for d, u in zip(left, right):
            # d = desired (left/primary), u = reference (right/noise)
            cleaned = state.nlms_step(u, d)
            state.recorded_samples.append(cleaned)
            cleaned_chunk.append(cleaned)

        enqueueLiveSamples(cleaned_chunk)

        if state.file_start_ts is None:
            state.file_start_ts = effective_ts

        # --- Flush complete files ---
        while len(state.recorded_samples) >= SAMPLES_PER_FILE:
            chunk = [state.recorded_samples.popleft() for _ in range(SAMPLES_PER_FILE)]
            save_wav(device_id, chunk, state.file_start_ts, effective_ts)
            state.file_start_ts = effective_ts

# ---------------------------------------------------------------------------
# UDP server
# ---------------------------------------------------------------------------

def udp_server():
    conn_cfg = credentials["connection"]
    host     = conn_cfg["ip_address"]
    port     = conn_cfg["port"]

    server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4096 * 4096)
    server.bind((host, port))

    print(f"UDP server listening on {host}:{port}")

    while True:
        try:
            payload, _addr = server.recvfrom(65535)
            handle_packet(payload, datetime.datetime.now())
        except Exception as e:
            print("UDP receive error:", e)
            traceback.print_exc()

# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="UDP audio receiver")
    parser.add_argument(
        "--listen-live",
        action="store_true",
        help="Play received audio with a 300 ms start buffer",
    )
    parser.add_argument(
        "--save-raw-stereo",
        action="store_true",
        help="Also save original left/right channels as 20 second input/desired WAV files",
    )
    parser.add_argument(
        "--plot",
        action="store_true",
        help="Show real-time plot of received audio",
    )
    args = parser.parse_args()

    if args.listen_live:
        livePlaybackEnabled.set()
        threading.Thread(target=livePlaybackWorker, daemon=True).start()

    if args.save_raw_stereo:
        saveInputDesiredEnabled.set()
        print(f"Input/desired save enabled — {RAW_SAVE_SECONDS} second files")

    threading.Thread(target=udp_server, daemon=True).start()

    if args.plot:
        plotWorker()
    else:
        while True:
            try:
                threading.Event().wait(1)
            except KeyboardInterrupt:
                print("Exiting...")
                break
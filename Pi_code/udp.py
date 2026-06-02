import struct
import numpy as np
from collections import deque
import wave
from pathlib import Path
import psycopg2
import datetime
import json
import socket
import threading
import queue
import traceback


recorded_samples = deque()

credentials = json.load(open('PI_code/credentials.json'))


conn = psycopg2.connect(
    dbname=credentials["database"]["db_name"],
    user=credentials["database"]["user"],
    password=credentials["database"]["password"],
    host=credentials["database"]["host"],
    port=credentials["database"]["Port"]
)

cur = conn.cursor()

SAMPLES_PER_FILE = (
    credentials["recording"]["sample_rate"]
    * credentials["recording"]["save_seconds"]
)

FILE_DURATION = datetime.timedelta(
    seconds=credentials["recording"]["save_seconds"]
)

RECORDINGS_DIR = Path(__file__).parent / "recordings"
RECORDINGS_DIR.mkdir(parents=True, exist_ok=True)

M = 64
MU = 0.01

file_start_timestamp = None

w = np.zeros(M)
x_hist = deque([0.0] * M, maxlen=M)

last_sequence = {}

db_queue = queue.Queue()


def _save_wav(device_id, samples, start_timestamp, stop_timestamp):

    if not samples:
        return

    clipped = np.clip(
        np.asarray(samples, dtype=np.float64),
        -1.0,
        1.0
    )

    pcm = (clipped * 32767).astype(np.int16)

    date_dir = (
        RECORDINGS_DIR
        / device_id
        / str(start_timestamp.date())
    )

    date_dir.mkdir(parents=True, exist_ok=True)

    start_label = start_timestamp.strftime("%H%M%S")
    stop_label  = stop_timestamp.strftime("%H%M%S")

    out_path = date_dir / f"{start_label}_{stop_label}.wav"

    with wave.open(str(out_path), "wb") as wf:
        wf.setnchannels(credentials["recording"]["channels"])
        wf.setsampwidth(credentials["recording"]["sample_width_bytes"])
        wf.setframerate(credentials["recording"]["sample_rate"])
        wf.writeframes(pcm.tobytes())

    print(f"[{device_id}] Saved {len(samples)} samples to {out_path}")


def save_wav(device_id, samples, start_timestamp, stop_timestamp):
    db_queue.put((device_id, list(samples), start_timestamp, stop_timestamp))


def db_worker():
    while True:
        device_id, samples, start_timestamp, stop_timestamp = db_queue.get()
        try:
            _save_wav(device_id, samples, start_timestamp, stop_timestamp)
        except Exception as e:
            print("Save error:", e)
        db_queue.task_done()


threading.Thread(target=db_worker, daemon=True).start()


def nlms_step(u, d):

    global w
    global x_hist

    x_hist.append(u)

    x = np.array(x_hist)

    y = np.dot(w, x)

    e = d - y

    norm = np.dot(x, x) + 1e-8

    w += (MU / norm) * e * x

    return e


def handle_packet(payload, message_time):

    global recorded_samples
    global file_start_timestamp
    global last_sequence

    header_size = 46

    if len(payload) < header_size:
        return

    device_id_raw, timestamp_raw, length, sequence = struct.unpack_from("<16s24sHI", payload, 0)

    # try to decode device-provided timestamp (fallback to None)
    device_timestamp = None
    try:
        timestamp_str = (
            timestamp_raw.decode("utf-8", errors="ignore").rstrip("\x00").strip()
        )
        if timestamp_str:
            try:
                # ISO 8601 like: 2026-06-02T12:34:56.789
                device_timestamp = datetime.datetime.fromisoformat(timestamp_str)
            except Exception:
                try:
                    # common fallback format: 'YYYY-MM-DD HH:MM:SS.sss'
                    device_timestamp = datetime.datetime.strptime(timestamp_str, "%Y-%m-%d %H:%M:%S.%f")
                except Exception:
                    device_timestamp = None
    except Exception:
        device_timestamp = None

    # Decode device_id first so it's available for both the stop and audio paths
    device_id = (
        device_id_raw
        .decode("utf-8", errors="ignore")
        .rstrip("\x00")
        .strip()
    ) or "unknown"

    # Stop packet — flush partial buffer and reset state
    if length == 0:
        if recorded_samples:
            chunk = list(recorded_samples)
            flush_start = file_start_timestamp  # capture before clearing
            recorded_samples.clear()
            file_start_timestamp = None
            # prefer device timestamp for stop time if available
            save_wav(device_id, chunk, flush_start, device_timestamp or message_time)
            print(f"[{device_id}] Sleep signal received, flushed {len(chunk)} samples")
        last_sequence.pop(device_id, None)
        return

    expected_size = header_size + length

    if len(payload) != expected_size:
        print(f"[{device_id}] Invalid packet size: got {len(payload)}, expected {expected_size}")
        return

    if device_id in last_sequence:
        expected = (last_sequence[device_id] + 1) & 0xFFFFFFFF
        if sequence != expected:
            delta = (sequence - expected) & 0xFFFFFFFF
            print(f"[{device_id}] Packet loss: lost={delta}")

    last_sequence[device_id] = sequence

    frame = payload[header_size:]

    samples = np.frombuffer(frame, dtype=np.int16).astype(np.float64)
    samples /= 32767.0

    left  = samples[0::2]
    right = samples[1::2]

    for d, u in zip(left, right):
        cleaned = nlms_step(d, u)
        recorded_samples.append(cleaned)

    # use device timestamp for file start when available, otherwise receive time
    if file_start_timestamp is None:
        file_start_timestamp = device_timestamp or message_time

    while len(recorded_samples) >= SAMPLES_PER_FILE:

        chunk = [recorded_samples.popleft() for _ in range(SAMPLES_PER_FILE)]

        chunk_start = file_start_timestamp
        chunk_stop = chunk_start + FILE_DURATION

        save_wav(device_id, chunk, chunk_start, chunk_stop)

        file_start_timestamp = message_time if recorded_samples else None


def udp_server(host="0.0.0.0", port=50000):

    server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    server.setsockopt(
        socket.SOL_SOCKET,
        socket.SO_RCVBUF,
        1024 * 1024  # 1 MB
    )

    server.bind((host, port))

    print(f"UDP server listening on {host}:{port}")

    while True:
        try:
            payload, addr = server.recvfrom(65535)
            handle_packet(payload, datetime.datetime.now())
        except Exception as e:
            print("UDP receive error:", e)
            traceback.print_exc()


if __name__ == "__main__":
    udp_server()
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

credentials = json.load(open('PI_code/credentials.json'))

db_conn = psycopg2.connect(
    dbname=credentials["database"]["db_name"],
    user=credentials["database"]["user"],
    password=credentials["database"]["password"],
    host=credentials["database"]["host"],
    port=credentials["database"]["Port"]
)

frameFormat = 16
framesizebytes = frameFormat // 8

SAMPLE_RATE = credentials["recording"]["sample_rate"]
CHANNELS = credentials["recording"].get("channels", 1)
SAVE_SECONDS = credentials["recording"]["save_seconds"]

SAMPLES_PER_FILE = SAMPLE_RATE * SAVE_SECONDS * CHANNELS
FILE_DURATION = datetime.timedelta(seconds=SAVE_SECONDS)

RECORDINGS_DIR = Path(__file__).parent / "recordings"
RECORDINGS_DIR.mkdir(parents=True, exist_ok=True)

M = 64
MU = 0.01

HEADER_SIZE = 18

db_lock = threading.Lock()

device_states = {}
device_states_lock = threading.Lock()


class DeviceState:
    def __init__(self):
        self.recorded_samples = []
        self.file_start_timestamp = None

        self.w = np.zeros(M)
        self.x_hist = np.zeros(M, dtype=np.float64)

        self.sample_count = 0
        self.sample_start = datetime.datetime.now()
        self.sample_window = deque(maxlen=5)

        self.lock = threading.Lock()


def get_device_state(device_id):
    with device_states_lock:
        if device_id not in device_states:
            device_states[device_id] = DeviceState()

        return device_states[device_id]


def nlms_step(state, u, d):
    state.x_hist[1:] = state.x_hist[:-1]
    state.x_hist[0] = u

    x = state.x_hist

    y = np.dot(state.w, x)

    e = d - y

    norm = np.dot(x, x) + 1e-8

    state.w += (MU / norm) * e * x

    return e


def insert_record(deviceid, start_timestamp, stop_timestamp, path):
    with db_lock:
        cur = db_conn.cursor()

        cur.execute(
            """
            INSERT INTO audiodata (
                deviceid,
                start_datetime,
                stop_datetime,
                recording_path
            )
            VALUES (%s, %s, %s, %s)
            """,
            (
                deviceid,
                start_timestamp.strftime("%Y%m%d_%H%M%S"),
                stop_timestamp.strftime("%Y%m%d_%H%M%S"),
                str(path)
            )
        )

        db_conn.commit()

        cur.close()


def save_wav(device_id, samples, start_timestamp, stop_timestamp):
    if not samples:
        return

    clipped = np.clip(
        np.asarray(samples, dtype=np.float64),
        -1.0,
        1.0
    )

    pcm = (clipped * 32767).astype(np.int16)

    date_dir = (
        RECORDINGS_DIR /
        device_id /
        start_timestamp.strftime("%Y-%m-%d")
    )

    date_dir.mkdir(parents=True, exist_ok=True)

    start_label = start_timestamp.strftime("%H%M%S")
    stop_label = stop_timestamp.strftime("%H%M%S")

    out_path = date_dir / f"{start_label}_{stop_label}.wav"

    with wave.open(str(out_path), "wb") as wf:
        wf.setnchannels(CHANNELS)
        wf.setsampwidth(
            credentials["recording"]["sample_width_bytes"]
        )
        wf.setframerate(SAMPLE_RATE)
        wf.writeframes(pcm.tobytes())

    insert_record(
        device_id,
        start_timestamp,
        stop_timestamp,
        out_path
    )


def flush_remaining(device_id):
    state = get_device_state(device_id)

    with state.lock:
        if not state.recorded_samples:
            return

        duration_seconds = (
            len(state.recorded_samples) /
            (SAMPLE_RATE * CHANNELS)
        )

        stop_timestamp = (
            state.file_start_timestamp +
            datetime.timedelta(seconds=duration_seconds)
        )

        save_wav(
            device_id,
            state.recorded_samples,
            state.file_start_timestamp,
            stop_timestamp
        )

        print(
            f"Flushed partial recording for {device_id}: "
            f"{len(state.recorded_samples)} samples"
        )

        state.recorded_samples.clear()
        state.file_start_timestamp = None

        state.w = np.zeros(M)
        state.x_hist = np.zeros(M, dtype=np.float64)


def update_sample_stats(state, device_id, samples_per_channel):
    now_ts = datetime.datetime.now()

    state.sample_count += samples_per_channel

    elapsed = (
        now_ts - state.sample_start
    ).total_seconds()

    if elapsed >= 5.0:
        rate = state.sample_count / elapsed

        state.sample_window.append(rate)

        avg = (
            sum(state.sample_window) /
            len(state.sample_window)
        )

        print(
            f"Samples/sec for {device_id}: "
            f"{avg:.1f} per channel"
        )

        state.sample_count = 0
        state.sample_start = now_ts


def handle_packet(payload, message_time):
    if len(payload) < HEADER_SIZE:
        return

    raw_device_id, length = struct.unpack_from(
        "<16sH",
        payload,
        0
    )

    device_id = (
        raw_device_id
        .decode("utf-8", errors="ignore")
        .replace("\x00", "")
        .strip()
    )

    if not device_id:
        device_id = "unknown"

    if length > 10_000_000:
        print(f"Rejected oversized packet from {device_id}")
        return

    frame = payload[HEADER_SIZE:HEADER_SIZE + length]

    state = get_device_state(device_id)

    with state.lock:
        samples_per_channel = (
            (len(frame) // framesizebytes) //
            max(1, CHANNELS)
        )

        update_sample_stats(
            state,
            device_id,
            samples_per_channel
        )

        samples = np.frombuffer(
            frame,
            dtype=f'<i{framesizebytes}'
        ).astype(np.float64)

        denom = float(2**(frameFormat - 1) - 1)

        if CHANNELS == 2:
            if len(samples) % 2 != 0:
                samples = samples[:-1]

            left = samples[0::2] / denom
            right = samples[1::2] / denom

            cleaned = []

            for d, u in zip(left, right):
                cleaned_sample = nlms_step(state, u, d)
                cleaned.append(cleaned_sample)

            state.recorded_samples.extend(cleaned)

        else:
            mono = samples / denom
            state.recorded_samples.extend(mono.tolist())

        if state.file_start_timestamp is None:
            state.file_start_timestamp = message_time

        while len(state.recorded_samples) >= SAMPLES_PER_FILE:
            chunk = state.recorded_samples[:SAMPLES_PER_FILE]

            chunk_start = state.file_start_timestamp

            file_stop_timestamp = (
                chunk_start + FILE_DURATION
            )

            save_wav(
                device_id,
                chunk,
                chunk_start,
                file_stop_timestamp
            )

            print(
                f"Saved {len(chunk)} samples for "
                f"{device_id} from "
                f"{chunk_start} to "
                f"{file_stop_timestamp}"
            )

            state.recorded_samples = (
                state.recorded_samples[SAMPLES_PER_FILE:]
            )

            if state.recorded_samples:
                state.file_start_timestamp = (
                    file_stop_timestamp
                )
            else:
                state.file_start_timestamp = None


def client_thread(conn, addr):
    print(f"Client connected: {addr}")

    conn.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)

    conn.settimeout(10)

    buffer = bytearray()

    connected_devices = set()

    try:
        while True:
            data = conn.recv(4096)

            if not data:
                break

            buffer.extend(data)

            while len(buffer) >= HEADER_SIZE:
                raw_device_id, length = struct.unpack_from(
                    "<16sH",
                    buffer,
                    0
                )

                device_id = (
                    raw_device_id
                    .decode("utf-8", errors="ignore")
                    .replace("\x00", "")
                    .strip()
                )

                if not device_id:
                    device_id = "unknown"

                total_len = HEADER_SIZE + length

                if total_len > 10_000_000:
                    print(
                        f"Oversized packet from {device_id}"
                    )
                    return

                if len(buffer) < total_len:
                    break

                packet = bytes(buffer[:total_len])

                del buffer[:total_len]

                connected_devices.add(device_id)

                handle_packet(
                    packet,
                    datetime.datetime.now()
                )

    except socket.timeout:
        print(f"Client timeout: {addr}")

    except Exception as e:
        print(f"Connection error {addr}: {e}")

    finally:
        for device_id in connected_devices:
            flush_remaining(device_id)

        conn.close()

        print(f"Client disconnected: {addr}")


def tcp_server(host="0.0.0.0", port=50000):
    server = socket.socket(
        socket.AF_INET,
        socket.SOCK_STREAM
    )

    server.setsockopt(
        socket.SOL_SOCKET,
        socket.SO_REUSEADDR,
        1
    )

    server.bind((host, port))

    server.listen(32)

    print(f"TCP server listening on {host}:{port}")

    while True:
        conn, addr = server.accept()

        thread = threading.Thread(
            target=client_thread,
            args=(conn, addr),
            daemon=True
        )

        thread.start()


if __name__ == "__main__":
    tcp_server()
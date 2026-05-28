import struct
import numpy as np
from collections import deque
import wave
import os
from pathlib import Path
import psycopg2
import datetime
import json
import socket

credentials = json.load(open('PI_code/credentials.json'))

conn = psycopg2.connect(
    dbname=credentials["database"]["db_name"],
    user=credentials["database"]["user"],
    password=credentials["database"]["password"],
    host=credentials["database"]["host"],
    port=credentials["database"]["Port"]
)

cur = conn.cursor()

SAMPLES_PER_FILE = credentials["recording"]["sample_rate"] * credentials["recording"]["save_seconds"] * credentials["recording"].get("channels", 1)
FILE_DURATION = datetime.timedelta(seconds=credentials["recording"]["save_seconds"])

RECORDINGS_DIR = Path(__file__).parent / "recordings"
RECORDINGS_DIR.mkdir(parents=True, exist_ok=True)

M = 64
MU = 0.01

recorded_samples = []
file_start_timestamp = None

# per-device stats for monitoring samples/sec
sample_stats = {}

w = np.zeros(M)
x_hist = deque([0.0] * M, maxlen=M)


def nlms_step(u, d):

    global w, x_hist

    x_hist.append(u)

    x = np.array(x_hist)

    y = np.dot(w, x)

    e = d - y

    norm = np.dot(x, x) + 1e-8

    w += (MU / norm) * e * x

    return e


def insert_record(deviceid, start_timestamp, stop_timestamp, path):
    cur.execute(
        """
        INSERT INTO audiodata (deviceid, start_datetime, stop_datetime, recording_path)
        VALUES (%s, %s, %s, %s)
        """,
        (
            deviceid,
            start_timestamp.strftime("%Y%m%d_%H%M%S"),
            stop_timestamp.strftime("%Y%m%d_%H%M%S"),
            str(path)
        )
    )
    conn.commit()


def save_wav(device_id, samples, start_timestamp, stop_timestamp):

    if not samples:
        return

    clipped = np.clip(np.asarray(samples, dtype=np.float64), -1.0, 1.0)
    pcm = (clipped * 32767).astype(np.int16)

    date_dir = RECORDINGS_DIR / device_id / start_timestamp.strftime("%Y-%m-%d")
    date_dir.mkdir(parents=True, exist_ok=True)

    start_label = start_timestamp.strftime("%H%M%S")
    stop_label = stop_timestamp.strftime("%H%M%S")

    out_path = date_dir / f"{start_label}_{stop_label}.wav"

    with wave.open(str(out_path), "wb") as wf:
        wf.setnchannels(credentials["recording"]["channels"])
        wf.setsampwidth(credentials["recording"]["sample_width_bytes"])
        wf.setframerate(credentials["recording"]["sample_rate"])
        wf.writeframes(pcm.tobytes())

    insert_record(device_id, start_timestamp, stop_timestamp, out_path)


def handle_packet(payload, message_time):
    global recorded_samples, file_start_timestamp

    header_size = 18 

    if len(payload) < header_size:
        return

    device_id, length = struct.unpack_from("<16sHI", payload, 0)
    raw_id = device_id.decode("utf-8", errors="ignore").replace("\x00", "").strip()
    if not raw_id:
        raw_id = "unknown"
    device_id = raw_id

    frame = payload[header_size:header_size + length]

    # if configured for stereo, treat incoming interleaved int32 samples as L,R
    channels = credentials["recording"].get("channels", 1)

    # update per-device samples/sec stats (count per channel)
    now_ts = datetime.datetime.now()
    stats = sample_stats.setdefault(device_id, {"count": 0, "start": now_ts})
    samples_per_channel = (len(frame) // 4) // max(1, channels)
    stats["count"] += samples_per_channel
    elapsed = (now_ts - stats["start"]).total_seconds()
    if elapsed >= 1.0:
        rate = stats["count"] / elapsed
        print(f"Samples/sec for {device_id}: {rate:.1f} per channel")
        stats["count"] = 0
        stats["start"] = now_ts
    if channels == 2:
        # interpret bytes as little-endian 32-bit signed ints
        samples_i32 = np.frombuffer(frame, dtype='<i4')
        if len(samples_i32) % 2 != 0:
            samples_i32 = samples_i32[:-1]
        # normalize to -1..1 using int32 max
        denom = float(2**31 - 1)
        float_samples = samples_i32.astype(np.float64) / denom
        # append interleaved floats to recorded_samples
        recorded_samples.extend(float_samples.tolist())
    else:
        samples = np.array(
            struct.unpack_from(f"<{len(frame)//4}i", frame),
            dtype=np.float64
        )

        left = samples[0::2]
        right = samples[1::2]

        for d, u in zip(left, right):
            cleaned = nlms_step(u, d)
            recorded_samples.append(cleaned)

    if file_start_timestamp is None:
        file_start_timestamp = message_time

    while len(recorded_samples) >= SAMPLES_PER_FILE:
        chunk = recorded_samples[:SAMPLES_PER_FILE]
        chunk_start_timestamp = file_start_timestamp
        file_stop_timestamp = file_start_timestamp + FILE_DURATION
        save_wav(device_id, chunk, chunk_start_timestamp, file_stop_timestamp)
        recorded_samples = recorded_samples[SAMPLES_PER_FILE:]

        if recorded_samples:
            file_start_timestamp = message_time
        else:
            file_start_timestamp = None

        print(f"Saved {len(chunk)} samples for device {device_id} from {chunk_start_timestamp} to {file_stop_timestamp}")


def tcp_server(host="0.0.0.0", port=50000):
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((host, port))
    server.listen(5)

    print(f"TCP server listening on {host}:{port}")

    while True:
        conn, addr = server.accept()
        print(f"Client connected: {addr}")

        buffer = b""

        try:
            while True:
                data = conn.recv(4096)
                if not data:
                    break

                buffer += data

                while len(buffer) >= 22:
                    device_id, length, sequence = struct.unpack_from("<16sHI", buffer, 0)
                    total_len = 22 + length

                    if len(buffer) < total_len:
                        break

                    packet = buffer[:total_len]
                    buffer = buffer[total_len:]

                    handle_packet(packet, datetime.datetime.now())

        except Exception as e:
            print("Connection error:", e)

        finally:
            conn.close()
            print("Client disconnected")


if __name__ == "__main__":
    tcp_server()
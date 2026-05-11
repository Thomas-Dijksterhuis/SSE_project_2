import struct
import numpy as np
import paho.mqtt.client as mqtt
from collections import deque
import wave
import os
from pathlib import Path
import psycopg2
import datetime
import json

credentials = json.load(open('credentials.json'))
Port = credentials["Port"]
db_name = credentials["db_name"]
user = credentials["user"]
password = credentials["password"]
host = credentials["host"]

conn = psycopg2.connect(
    dbname=db_name,
    user=user,
    password=password,
    host=host,
    port=Port
)


cur = conn.cursor()


BROKER_ADDRESS = "192.168.1.226"
TOPIC = "Readings"
SAMPLE_RATE = 10000
CHANNELS = 1
SAMPLE_WIDTH_BYTES = 2
SAVE_SECONDS = 20
SAMPLES_PER_FILE = SAMPLE_RATE * SAVE_SECONDS
RECORDINGS_DIR = Path(__file__).parent / "recordings"
RECORDINGS_DIR.mkdir(parents=True, exist_ok=True)

M = 64
MU = 0.01

recorded_samples = []
file_index = 0
file_start_timestamp = None

w = np.zeros(M)
x_hist = deque([0.0] * M, maxlen=M)


def insert_record(deviceid, start_timestamp, stop_timestamp, path):
    cur.execute(
        """
        INSERT INTO audiodata (deviceid, start_datetime, stop_datetime, recording_path)
        VALUES (%s, %s, %s, %s)
        """,
        (deviceid, start_timestamp.strftime("%Y%m%d_%H%M%S"), stop_timestamp.strftime("%Y%m%d_%H%M%S"), str(path))
    )
    conn.commit()


def nlms_step(u, d):
    global w, x_hist

    x_hist.append(u)
    x = np.array(x_hist)

    y = np.dot(w, x)
    e = d - y

    norm = np.dot(x, x) + 1e-8
    w += (MU / norm) * e * x

    return e


def save_wav(device_id, samples, start_timestamp, stop_timestamp):

    if not samples:
        return

    clipped = np.clip(np.asarray(samples, dtype=np.float64), -1.0, 1.0)
    pcm = (clipped * 32767).astype(np.int16)

    date_dir = start_timestamp.date()
    start_label = start_timestamp.strftime("%H%M%S")
    stop_label = stop_timestamp.strftime("%H%M%S")
    if not os.path.exists(RECORDINGS_DIR / f"{device_id}/{date_dir}"):
        os.makedirs(RECORDINGS_DIR / f"{device_id}/{date_dir}")
    out_path = RECORDINGS_DIR / f"{device_id}/{date_dir}/{start_label}_{stop_label}.wav"

    with wave.open(str(out_path), "wb") as wf:
        wf.setnchannels(CHANNELS)
        wf.setsampwidth(SAMPLE_WIDTH_BYTES)
        wf.setframerate(SAMPLE_RATE)
        wf.writeframes(pcm.tobytes())

    print(f"Saved NLMS error signal with {len(pcm)} samples to {out_path}")
    insert_record(device_id, start_timestamp, stop_timestamp, out_path)


def on_message(client, userdata, message):
    global recorded_samples, file_start_timestamp

    payload = message.payload

    header_size = 22
    if len(payload) < header_size:
        return

    device_id, length = struct.unpack_from('<16sH', payload, 0)
    device_id = device_id.decode('utf-8', errors='ignore').rstrip('\x00')

    frame = payload[header_size:header_size + length]
    samples = np.array(
        struct.unpack_from(f'<{len(frame)//4}i', frame),
        dtype=np.float64
    )

    left = samples[0::2]
    right = samples[1::2]

    for d, u in zip(left, right):
        cleaned = nlms_step(u, d)
        recorded_samples.append(cleaned)

    if file_start_timestamp is None:
        file_start_timestamp = datetime.datetime.now()

    while len(recorded_samples) >= SAMPLES_PER_FILE:
        chunk = recorded_samples[:SAMPLES_PER_FILE]
        file_stop_timestamp = file_start_timestamp + datetime.timedelta(seconds=SAVE_SECONDS)
        save_wav(device_id, chunk, file_start_timestamp, file_stop_timestamp)
        recorded_samples = recorded_samples[SAMPLES_PER_FILE:]

        if recorded_samples:
            file_start_timestamp += datetime.timedelta(seconds=SAVE_SECONDS)
        else:
            file_start_timestamp = None


client = mqtt.Client(
client_id="nlms_batch",
callback_api_version=mqtt.CallbackAPIVersion.VERSION2
)

client.on_message = on_message
client.connect(BROKER_ADDRESS)
client.subscribe(TOPIC)


try:
    client.loop_forever()
except KeyboardInterrupt:
    pass

client.disconnect()
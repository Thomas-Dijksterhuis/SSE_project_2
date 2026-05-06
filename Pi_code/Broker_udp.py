import socket
import struct
import os
import wave
from collections import defaultdict
from datetime import datetime
import time

UDP_IP = "0.0.0.0"
UDP_PORT = 50005

UPLOAD_DIR = "uploads"
os.makedirs(UPLOAD_DIR, exist_ok=True)

SAMPLE_RATE = 10000
CHANNELS = 1
SAMPLE_WIDTH = 2

sessions = defaultdict(dict)

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

print(f"UDP server listening on {UDP_IP}:{UDP_PORT}")

while True:
    packet = sock.recvfrom(4096)[0]

    if b"|" not in packet:
        continue

    device_id, payload = packet.split(b"|", 1)
    device_id = device_id.decode(errors="ignore")

    sessions[device_id] = payload

    now = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
    print(f"[{now}] Received data from {device_id}")
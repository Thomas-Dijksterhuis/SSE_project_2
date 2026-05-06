from flask import Flask, request
import os
import wave

app = Flask(__name__)

UPLOAD_DIR = "uploads"
os.makedirs(UPLOAD_DIR, exist_ok=True)

SAMPLE_RATE = 10000
CHANNELS = 1
SAMPLE_WIDTH = 2  # 16-bit PCM

sessions = {}

@app.route("/upload_chunk", methods=["POST"])
def upload_chunk():
    device_id = request.headers.get("Device-Id", "unknown")
    index = int(request.headers.get("Chunk-Index", 0))
    total_chunks = int(request.headers.get("Total-Chunks", 1))

    data = request.data

    if device_id not in sessions:
        sessions[device_id] = {}

    sessions[device_id][index] = data

    print(f"Received {device_id} chunk {index}/{total_chunks}")

    # if len(sessions[device_id]) == total_chunks:

    #     print(f"All chunks received for {device_id}, assembling...")

    #     audio_data = b''.join(
    #         sessions[device_id][i]
    #         for i in range(total_chunks)
    #     )

    #     output_path = os.path.join(UPLOAD_DIR, f"{device_id}.wav")

    #     with wave.open(output_path, 'wb') as wf:
    #         wf.setnchannels(CHANNELS)
    #         wf.setsampwidth(SAMPLE_WIDTH)
    #         wf.setframerate(SAMPLE_RATE)
    #         wf.writeframes(audio_data)

    #     print(f"Saved assembled audio to {output_path}")

    #     del sessions[device_id]

    return "OK", 200


if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000, threaded=True)
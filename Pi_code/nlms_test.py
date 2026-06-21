import wave
import numpy as np
import matplotlib.pyplot as plt

from udp import DeviceState
from scipy.optimize import minimize_scalar

M = 64


def read_wav_mono(path):
    with wave.open(path, "rb") as wf:
        n = wf.getnframes()
        data = wf.readframes(n)
        x = np.frombuffer(data, dtype=np.int16).astype(np.float64)
        x /= 32768.0
        return x

def evaluate(reference, target, desired, m, mu, warmup=None):
    device = DeviceState(m, mu)
    cleaned = np.zeros(len(desired))

    for i in range(len(desired)):
        cleaned[i] = device.nlms_step(reference[i], desired[i])

    warmup = warmup or 10 * m
    mse = np.mean((cleaned[warmup:] - target[warmup:]) ** 2)
    return mse, cleaned


def run(reference, target, desired, m=64, mu=0.1):
    n = min(len(reference), len(target), len(desired))
    mse, cleaned = evaluate(reference, target, desired, m, mu)

    t = np.arange(n)
    fig, ax = plt.subplots(4, 1, figsize=(14, 10), sharex=True)
    fig.suptitle(f"NLMS filter with MU={mu:.6f}. MSE={mse:.10e}")

    ax[0].plot(t, target[:n]);   ax[0].set_title("Original (clean) input")
    ax[1].plot(t, reference[:n]);  ax[1].set_title("Noise")
    ax[2].plot(t, desired[:n]);  ax[2].set_title("Desired = input + noise")
    ax[3].plot(t, cleaned[:n]);  ax[3].set_title("NLMS output")
    ax[3].sharey(ax[0])

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    input_sig = read_wav_mono("Test_recordings/test2/left.wav")
    noise_sig = read_wav_mono("Test_recordings/test2/right.wav")

    n = min(len(input_sig), len(noise_sig))
    input_sig, noise_sig = input_sig[:n], noise_sig[:n]
    desired = input_sig + noise_sig

    result = minimize_scalar(
        lambda mu: evaluate(noise_sig, input_sig, desired, M, mu)[0],
        bounds=(0.001, 0.35),
        method="bounded",
        options={"xatol": 1e-5}
    )
    print(f"Optimal mu = {result.x:.5f}, MSE = {result.fun:.10e}")

    run(noise_sig, input_sig, desired, m=M, mu=result.x)
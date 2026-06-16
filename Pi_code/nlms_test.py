import wave
import numpy as np
import matplotlib.pyplot as plt

from udp import DeviceState


def read_wav_mono(path):
    with wave.open(path, "rb") as wf:
        n = wf.getnframes()
        data = wf.readframes(n)
        x = np.frombuffer(data, dtype=np.int16).astype(np.float64)
        x /= 32768.0
        return x


def run(input, desired, speech, m=64, mu=0.1):
    u = input
    d = desired

    n = min(len(u), len(d), len(speech))

    device = DeviceState(m, mu)

    cleaned = np.zeros(n)

    for i in range(n):
        cleaned[i] = device.nlms_step(u[i], d[i])

    mse_before = np.mean((d[:n] - speech[:n])**2)
    mse_after = np.mean((cleaned - speech[:n])**2)

    print(f"MSE before: {mse_before:.6f}")
    print(f"MSE after : {mse_after:.6f}")

    t = np.arange(n)

    fig, ax = plt.subplots(4, 1, figsize=(14, 10), sharex=True)

    ax[0].plot(t, speech)
    ax[0].set_title("Original speech")

    ax[1].plot(t, u)
    ax[1].set_title("Reference noise")

    ax[2].plot(t, d)
    ax[2].set_title("Desired = speech + noise")

    ax[3].plot(t, cleaned)
    ax[3].set_title("NLMS output")

    ax[3].sharey(ax[0])  # link output scale to original speech scale

    plt.tight_layout()
    plt.show()

    plt.figure(figsize=(12, 4))
    plt.plot(device.w)
    plt.title("Learned filter coefficients")
    plt.grid()
    plt.show()


if __name__ == "__main__":
    speech = read_wav_mono("Test_recordings/test2/right.wav")

    reference = np.random.randn(len(speech))
    reference /= np.max(np.abs(reference))

    noise = 0.1 * np.roll(reference, 50)

    desired = speech + noise

    n = len(speech)

    Ms = [16, 32, 64, 128, 256]
    MUs = [0.05, 0.1, 0.2, 0.25, 0.3]

    best_error = np.inf
    best = None

    for m in Ms:
        for mu in MUs:

            device = DeviceState(m, mu)

            cleaned = np.zeros(n)

            for i in range(n):
                cleaned[i] = device.nlms_step(reference[i], desired[i])

            mse = np.mean((cleaned - speech) ** 2)

            print(f"M={m:4d} MU={mu:4.2f} MSE={mse:.6f}")

            if mse < best_error:
                best_error = mse
                best_error_signal = cleaned.copy()
                best = (m, mu)

    print()
    print("Best parameters:")
    print(f"M={best[0]} MU={best[1]} MSE={best_error:.6f}")

    run(reference, desired, speech, m=best[0], mu=best[1])



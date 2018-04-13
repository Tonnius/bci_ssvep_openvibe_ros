from scipy.signal import butter, lfilter

def butter_bandpass(lowcut, highcut, fs, order=5):
    nyq = 0.5 * fs
    low = lowcut / nyq
    high = highcut / nyq
    b, a = butter(order, [low, high], btype='band')
    return b, a


def butter_bandpass_filter(data, lowcut, highcut, fs, order=5):
    b, a = butter_bandpass(lowcut, highcut, fs, order=order)
    y = lfilter(b, a, data)
    return y


def run():
    import numpy as np
    import matplotlib.pyplot as plt
    from scipy.signal import freqz

    # Sample rate and desired cutoff frequencies (in Hz).
    fs = 250.0
    lowcut = 9.75
    highcut = 10.75

    # Plot the frequency response for a few different orders.
    plt.figure(1)
    plt.clf()
    proov1 = [8.57142857143, 8.57142857143*2, 8.57142857143*3, 7.5, 7.5*2, 7.5*3, 12, 24, 36, 10, 20,30] #10, 20 or 20, 40
    proov3 = [6, 12, 8.57142857143, 17.1428571429, 20, 40, 30, 60]
    proov2 = [7.5, 15, 10, 20, 12, 24, 30, 60]
    proov4 = [6.6666, 6.6666*2,  8.57142857143,  8.57142857143*2, 20, 40, 30, 60]
    proov5 = [7.5, 7.5*2, 7.5*3, 20, 40,80, 12, 24, 36, 30, 60, 90]
    proov6 = [10, 20, 30, 20, 40, 80, 12, 24, 36, 15, 30, 45]

    threshold = 0.25
    for fCent in proov1:
        b, a = butter_bandpass(fCent-threshold, fCent+threshold, fs, order=4)
        w, h = freqz(b, a, worN=2000)
        plt.plot((fs * 0.5 / np.pi) * w, abs(h), label="order = %d" % 4)

    plt.plot([0, 0.5 * fs], [np.sqrt(0.5), np.sqrt(0.5)],
             '--', label='sqrt(0.5)')
    plt.xlabel('Frequency (Hz)')
    plt.ylabel('Gain')
    plt.grid(True)
    plt.legend(loc='best')

    # Filter a noisy signal.
    T = 0.05
    nsamples = T * fs
    t = np.linspace(0, T, nsamples, endpoint=False)
    a = 0.02
    f0 = 10.0
    x = 0.1 * np.sin(2 * np.pi * 1.2 * np.sqrt(t))
    x += 0.01 * np.cos(2 * np.pi * 312 * t + 0.1)
    x += a * np.cos(2 * np.pi * f0 * t + .11)
    x += 0.03 * np.cos(2 * np.pi * 2000 * t)
    plt.figure(2)
    plt.clf()
    plt.plot(t, x, label='Noisy signal')

    y = butter_bandpass_filter(x, lowcut, highcut, fs, order=4)
    plt.plot(t, y, label='Filtered signal (%g Hz)' % f0)
    plt.xlabel('time (seconds)')
    plt.hlines([-a, a], 0, T, linestyles='--')
    plt.grid(True)
    plt.axis('tight')
    plt.legend(loc='upper left')

    plt.show()


run()
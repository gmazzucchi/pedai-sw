import numpy as np

x = [10, 20, 30, 40, 50, 60, 70, 80, 90, 100]
# y = np.fft.fft(x)
# y = np.fft.rfft(x)
y = np.abs(np.fft.fft(x))
print(y)


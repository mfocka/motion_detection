import numpy as np
import math

def generate_vibration(duration_s=60.0, fs=104.0, amp_deg=2.0, freq_hz=1.2):
    n = int(duration_s * fs)
    t = np.arange(n) / fs
    yaw = amp_deg * np.sin(2*math.pi*freq_hz*t)
    pitch = amp_deg * 0.5 * np.sin(2*math.pi*(freq_hz*1.3)*t)
    return t, yaw, pitch

if __name__ == "__main__":
    t, yaw, pitch = generate_vibration()
    print("%d samples" % len(t))


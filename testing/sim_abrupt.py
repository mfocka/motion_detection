import numpy as np

def generate_abrupt(duration_s=120.0, fs=104.0, step_time_s=10.0, yaw_step_deg=50.0, pitch_step_deg=0.0):
    n = int(duration_s * fs)
    t = np.arange(n) / fs
    yaw = np.zeros_like(t)
    pitch = np.zeros_like(t)
    idx = int(step_time_s * fs)
    yaw[idx:] = yaw_step_deg
    pitch[idx:] = pitch_step_deg
    return t, yaw, pitch

if __name__ == "__main__":
    t, yaw, pitch = generate_abrupt()
    print("%d samples" % len(t))


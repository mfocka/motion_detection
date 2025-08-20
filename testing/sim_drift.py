import numpy as np

def generate_drift(duration_s=1200.0, fs=104.0, yaw_rate_deg_per_min=0.5, pitch_rate_deg_per_min=0.2):
    n = int(duration_s * fs)
    t = np.arange(n) / fs
    yaw = (yaw_rate_deg_per_min/60.0) * t
    pitch = (pitch_rate_deg_per_min/60.0) * t
    return t, yaw, pitch

if __name__ == "__main__":
    t, yaw, pitch = generate_drift()
    print("%d samples" % len(t))


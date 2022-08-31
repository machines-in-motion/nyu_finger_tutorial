import numpy as np

def spline_interpolation(start, end, horizon, dt):
    duration = (horizon - 1) * dt
    diff = end - start

    a5 = 6/(duration ** 5)
    a4 = -15/(duration ** 4)
    a3 = 10/(duration ** 3)

    q = start * np.ones((horizon, len(diff)))
    dq = np.zeros((horizon, len(diff)))
    ddq = np.zeros((horizon, len(diff)))

    for n in range(horizon):
        t = n * dt
    
        s = a3 * t**3 + a4 * t**4 + a5 * t**5
        ds = 3 * a3 * t**2 + 4 * a4 * t**3 + 5 * a5 * t**4
        dds = 6 * a3 * t + 12 * a4 * t**2 + 20 * a5 * t**3
    
        q[n] += s * diff
        dq[n] = diff * ds
        ddq[n] = diff * dds
    
    return q, dq, ddq
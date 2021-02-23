import numpy as np
import time
import queue
import matplotlib.pyplot as plt

def velocity(pi, pf, ti, tf):
    return (pf - pi) / (tf - ti)

def acceleration(vi, vf, ti, tf):
    return (vf - vi) / (tf - ti)
def get_accelerations():
    points = queue.Queue()
    times = queue.Queue()
    accelerations = []

    for pt in range(pts_needed):
        points.put((float)(input())) # need to add error handling
        times.put(time.time())

    while (True):
        buffer = time.time()
        p1 = points.get()
        p2 = points.get()
        p3 = points.get()
        t1 = times.get()
        t2 = times.get()
        t3 = times.get()

        v1 = velocity(p1, p2, t1, t2)
        v2 = velocity(p2, p3, t2, t3)

        accelerations.append(acceleration(v1, v2, t1, t3))

        points.put(p2)
        points.put(p3)
        times.put(t2)
        times.put(t3)

        new_input = input()
        if (new_input == esc_keyword):
            break
        points.put((float)(new_input))
        times.put(time.time() - (buffer - t3))   
    
    return accelerations
    
    
pts_needed = 3
esc_keyword = "Done"
mean = 0
sigma = 1

a = get_accelerations()

noise = np.random.normal(mean, sigma, len(a))
noisy_a = a + noise

plt.plot(np.linspace(0, len(a), len(a)), a, "o")
plt.plot(np.linspace(0, len(a), len(a)), a)
plt.xlabel("Time")
plt.xlabel("Acceleration")
plt.title("Acceleration data")
plt.show()

plt.plot(np.linspace(0, len(noise), len(noise)), noise, "o")
plt.plot(np.linspace(0, len(noise), len(noise)), noise)
plt.xlabel("Time")
plt.xlabel("Noise")
plt.title("Gaussian Noise")
plt.show()

plt.plot(np.linspace(0, len(noisy_a), len(noisy_a)), noisy_a, "o")
plt.plot(np.linspace(0, len(noisy_a), len(noisy_a)), noisy_a)
plt.xlabel("Time")
plt.xlabel("Acceleration")
plt.title("Noisy Acceleration")
plt.show()


print("Acceleration: ", a, "\n")
print("Noise: ", noise, "\n")
print("Noisy acceleration: ", noisy_a, "\n")

#TODO: - change input to be from file (assume csv)
#      - change time to be based on provided frequency of data (10hz for accel, ~50hz for vel, ~100hz for pos)
#      - send noisy_a to a UART port
#      - integrate with gps position data generation program

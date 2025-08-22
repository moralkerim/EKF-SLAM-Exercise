import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np

# Dosyayı oku
data = {"Gaussian": [], "Actual": [], "Prediction": [], "Update": [], "Landmark": [], "Landmark EKF": []}

with open("poses.txt") as f:
    for line in f:
        parts = line.strip().split(",")
        label = parts[0]
        x, y, theta, t = map(float, parts[1:])
        data[label].append((x, y, theta, t))

# Maksimum zaman değeri bul
max_t = int(max(v[3] for values in data.values() for v in values))

# Grafik ayarları
fig, ax = plt.subplots(figsize=(6,6))
ax.set_xlim(min(v[0] for values in data.values() for v in values)-1,
            max(v[0] for values in data.values() for v in values)+1)
ax.set_ylim(min(v[1] for values in data.values() for v in values)-1,
            max(v[1] for values in data.values() for v in values)+1)
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_title("EKF Logging - Zamanla")
ax.grid(True)

scatters = {label: ax.scatter([], [], marker="o", label=label) for label in data}
texts = []  # artık temizlemiyoruz, her frame’de yenisi ekleniyor

def update(frame_t):
    for label, scatter in scatters.items():
        # t <= frame_t olan verileri al
        values = [(x,y,theta,t) for (x,y,theta,t) in data[label] if int(t) <= frame_t]
        if values:
            xs, ys, thetas, ts = zip(*values)
            scatter.set_offsets(np.column_stack((xs, ys)))

            # o frame’de eklenen noktaların hepsine zaman yaz
            for i, tt in enumerate(ts):
                if int(tt) == frame_t:
                    texts.append(ax.text(xs[i], ys[i], f"{tt:.0f}", fontsize=8, ha="right", va="bottom"))

    ax.legend()
    return list(scatters.values()) + texts

ani = animation.FuncAnimation(fig, update, frames=range(max_t+1), interval=1000, blit=False, repeat=False)

plt.show()

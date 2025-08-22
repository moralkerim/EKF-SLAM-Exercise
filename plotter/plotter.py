import matplotlib.pyplot as plt

# Dosyayı oku
data = {"Gaussian": [], "Actual": [], "Prediction": [], "Update": [], "Landmark": [], "Landmark EKF": []}

with open("poses.txt") as f:
    for line in f:
        parts = line.strip().split(",")
        label = parts[0]
        x, y, theta, t = map(float, parts[1:])
        data[label].append((x, y, theta, t))

# Grafiğe dök
plt.figure(figsize=(6,6))

for label, values in data.items():
    if not values:  # boşsa atla
        continue
    xs, ys, thetas, ts = zip(*values)
    plt.scatter(xs, ys, marker="o", label=label)
    
    # Zaman değerlerini noktaların yanına ekle
    for x, y, t in zip(xs, ys, ts):
        plt.text(x, y, f"{t:.1f}", fontsize=8, ha="right", va="bottom")

plt.legend()
plt.xlabel("X")
plt.ylabel("Y")
plt.title("EKF Logging")
plt.grid(True)
plt.show()

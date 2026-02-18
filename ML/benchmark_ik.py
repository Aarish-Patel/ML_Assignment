import numpy as np
import torch
import torch.nn as nn
import joblib
import time
import matplotlib.pyplot as plt

# =========================
# Robot parameters
# =========================

L1 = 0.3
L2 = 0.25
L3 = 0.15

# =========================
# FK function (ground truth)
# =========================

def fk(t1, t2, t3):

    x = (L2*np.cos(t2) + L3*np.cos(t2+t3))*np.cos(t1)
    y = (L2*np.cos(t2) + L3*np.cos(t2+t3))*np.sin(t1)
    z = L1 + L2*np.sin(t2) + L3*np.sin(t2+t3)

    return np.array([x,y,z])


# =========================
# Analytical IK
# =========================

def analytical_ik(x,y,z):

    t1 = np.arctan2(y,x)

    r = np.sqrt(x*x + y*y)
    z_offset = z - L1

    d = np.sqrt(r*r + z_offset*z_offset)

    if d > (L2+L3) or d < abs(L2-L3):
        return None

    cos_t3 = (d*d - L2*L2 - L3*L3)/(2*L2*L3)
    cos_t3 = np.clip(cos_t3,-1,1)

    t3 = np.arccos(cos_t3)

    alpha = np.arctan2(z_offset,r)
    beta = np.arctan2(L3*np.sin(t3), L2+L3*np.cos(t3))

    t2 = alpha - beta

    return np.array([t1,t2,t3])


# =========================
# Neural FK model
# =========================

class FKNet(nn.Module):

    def __init__(self):

        super().__init__()

        self.net = nn.Sequential(
            nn.Linear(3,256),
            nn.GELU(),
            nn.Linear(256,256),
            nn.GELU(),
            nn.Linear(256,256),
            nn.GELU(),
            nn.Linear(256,3)
        )

    def forward(self,x):
        return self.net(x)


device = torch.device("cuda")

model = FKNet().to(device)
model.load_state_dict(torch.load("fk_model.pth", map_location=device, weights_only=True))
model.eval()

scaler_angles = joblib.load("scaler_angles.save")
scaler_pos = joblib.load("scaler_pos.save")


# =========================
# Neural IK
# =========================

def neural_ik(target):

    target_scaled = scaler_pos.transform([target])
    target_tensor = torch.tensor(target_scaled, device=device, dtype=torch.float32)

    angles = torch.zeros((1,3), device=device, requires_grad=True)

    optimizer = torch.optim.Adam([angles], lr=0.01)

    for i in range(100):

        pred = model(angles)
        loss = torch.mean((pred-target_tensor)**2)

        optimizer.zero_grad()
        loss.backward()
        optimizer.step()

    angles_real = scaler_angles.inverse_transform(
        angles.detach().cpu().numpy()
    )[0]

    return angles_real


# =========================
# Benchmark settings
# =========================

N = 200

analytic_errors = []
neural_errors = []

analytic_times = []
neural_times = []

joint_errors = []

analytic_success = 0
neural_success = 0


# =========================
# Generate random targets
# =========================

targets = []

for i in range(N):

    t1 = np.random.uniform(-np.pi,np.pi)
    t2 = np.random.uniform(-np.pi/2,np.pi/2)
    t3 = np.random.uniform(-np.pi/2,np.pi/2)

    pos = fk(t1,t2,t3)
    targets.append(pos)


# =========================
# Run benchmark
# =========================

for target in targets:

    # analytical
    start = time.perf_counter()
    analytic = analytical_ik(*target)
    analytic_times.append(time.perf_counter()-start)

    if analytic is not None:

        analytic_success += 1
        err = np.linalg.norm(fk(*analytic)-target)
        analytic_errors.append(err)
    else:
        analytic_errors.append(np.nan)


    # neural
    start = time.perf_counter()
    neural = neural_ik(target)
    neural_times.append(time.perf_counter()-start)

    neural_success += 1

    err = np.linalg.norm(fk(*neural)-target)
    neural_errors.append(err)

    if analytic is not None:

        joint_errors.append(
            np.linalg.norm(neural-analytic)
        )


# =========================
# Convert to numpy
# =========================

analytic_errors = np.array(analytic_errors)
neural_errors = np.array(neural_errors)

analytic_times = np.array(analytic_times)
neural_times = np.array(neural_times)

joint_errors = np.array(joint_errors)


# =========================
# Print results
# =========================

print("\nRESULTS")

print("\nAccuracy:")
print("Analytic mean error:", np.nanmean(analytic_errors))
print("Neural mean error:", np.mean(neural_errors))

print("\nSpeed:")
print("Analytic mean time:", np.mean(analytic_times)*1e6,"us")
print("Neural mean time:", np.mean(neural_times)*1000,"ms")

print("\nSuccess rate:")
print("Analytic:", analytic_success/N)
print("Neural:", neural_success/N)


# =========================
# GRAPHS
# =========================

plt.figure(figsize=(15,10))

# error histogram
plt.subplot(231)
plt.hist(analytic_errors, bins=50, alpha=0.5, label="Analytical")
plt.hist(neural_errors, bins=50, alpha=0.5, label="Neural")
plt.title("Position Error Histogram")
plt.legend()

# speed histogram
plt.subplot(232)
plt.hist(analytic_times*1e6, bins=50, alpha=0.5, label="Analytical")
plt.hist(neural_times*1000, bins=50, alpha=0.5, label="Neural")
plt.title("Speed Histogram")
plt.legend()

# joint error
plt.subplot(233)
plt.hist(joint_errors, bins=50)
plt.title("Joint Angle Error")

# scatter error vs sample
plt.subplot(234)
plt.plot(analytic_errors, label="Analytical")
plt.plot(neural_errors, label="Neural")
plt.title("Error vs Sample")
plt.legend()

# success rate bar
plt.subplot(235)
plt.bar(["Analytical","Neural"],
        [analytic_success/N, neural_success/N])
plt.title("Success Rate")

# speed bar
plt.subplot(236)
plt.bar(["Analytical","Neural"],
        [np.mean(analytic_times)*1e6,
         np.mean(neural_times)*1000])
plt.title("Average Speed")

plt.tight_layout()
plt.savefig("ik_comparison.png", dpi=300)
plt.show()

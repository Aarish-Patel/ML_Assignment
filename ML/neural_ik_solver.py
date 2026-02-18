import torch
import torch.nn as nn
import numpy as np
import joblib
import os
import sys


# =====================
# Device setup
# =====================

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

print("Using device:", device)


# =====================
# FK Model definition
# MUST MATCH TRAINING EXACTLY
# =====================

class FKNet(nn.Module):

    def __init__(self):

        super().__init__()

        self.net = nn.Sequential(

            nn.Linear(3, 256),
            nn.GELU(),

            nn.Linear(256, 256),
            nn.GELU(),

            nn.Linear(256, 256),
            nn.GELU(),

            nn.Linear(256, 3)
        )

    def forward(self, x):

        return self.net(x)


# =====================
# Load trained FK model
# =====================
# Get directory where this script exists
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

MODEL_PATH = os.path.join(SCRIPT_DIR, "fk_model.pth")
SCALER_ANGLES_PATH = os.path.join(SCRIPT_DIR, "scaler_angles.save")
SCALER_POS_PATH = os.path.join(SCRIPT_DIR, "scaler_pos.save")
model = FKNet().to(device)

try:
    # Load model
    model.load_state_dict(
        torch.load(
            MODEL_PATH,
            map_location=device,
            weights_only=True
        )
    )

except:

    # fallback if running from ML folder
    model.load_state_dict(
        torch.load(
            "fk_model.pth",
            map_location=device,
            weights_only=True
        )
    )


model.eval()

print("FK model loaded")


# =====================
# Load scalers
# =====================

try:
    # Load scalers
    scaler_angles = joblib.load(SCALER_ANGLES_PATH)
    scaler_pos = joblib.load(SCALER_POS_PATH)

except:

    scaler_angles = joblib.load("scaler_angles.save")
    scaler_pos    = joblib.load("scaler_pos.save")


print("Scalers loaded")


# =====================
# Neural IK Solver
# =====================

def solve_ik(target_pos,
             iterations=800,
             learning_rate=0.01):

    print("\nTarget position:", target_pos)

    # Scale target
    target_scaled = scaler_pos.transform([target_pos])

    target_tensor = torch.tensor(
        target_scaled,
        device=device,
        dtype=torch.float32
    )


    # Optimize in scaled angle space
    angles_scaled = torch.zeros(
        (1,3),
        device=device,
        dtype=torch.float32,
        requires_grad=True
    )


    optimizer = torch.optim.Adam(
        [angles_scaled],
        lr=learning_rate
    )


    # Optimization loop
    for i in range(iterations):

        optimizer.zero_grad()

        pred_scaled = model(angles_scaled)

        loss = torch.mean(
            (pred_scaled - target_tensor)**2
        )

        loss.backward()

        optimizer.step()

        if i % 200 == 0:

            print(f"Iter {i} Loss {loss.item():.10f}")


    # Convert to real angles
    angles_real = scaler_angles.inverse_transform(
        angles_scaled.detach().cpu().numpy()
    )

    return angles_real[0]


# =====================
# Verify solution
# =====================

def verify_solution(angles, target):

    with torch.no_grad():

        angles_scaled = scaler_angles.transform([angles])

        angles_tensor = torch.tensor(
            angles_scaled,
            device=device,
            dtype=torch.float32
        )

        pred_scaled = model(angles_tensor)

        pred_pos = scaler_pos.inverse_transform(
            pred_scaled.cpu().numpy()
        )[0]


    error = np.linalg.norm(
        np.array(pred_pos) - np.array(target)
    )

    return pred_pos, error


# =====================
# Main entry (called from C++)
# =====================

if __name__ == "__main__":

    # Check target file exists

    if not os.path.exists("neural_target.txt"):

        print("ERROR: neural_target.txt not found")
        sys.exit(1)


    # Read target from C++

    target_position = np.loadtxt("neural_target.txt")


    # Solve IK

    solved_angles = solve_ik(
        target_position,
        iterations=800,
        learning_rate=0.01
    )


    # Verify

    predicted_pos, error = verify_solution(
        solved_angles,
        target_position
    )


    print("\nSolved angles:", solved_angles)

    print("Predicted position:", predicted_pos)

    print("Position error:", error, "meters")


    # Save for C++

    np.savetxt(
        "neural_solution.csv",
        solved_angles.reshape(1,3),
        delimiter=","
    )


    print("\nneural_solution.csv written successfully")

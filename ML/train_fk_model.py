import torch
import torch.nn as nn
import torch.optim as optim

import pandas as pd
import numpy as np

from sklearn.model_selection import train_test_split
from sklearn.preprocessing import StandardScaler

import joblib
import time

# =====================
# Model
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
# Training
# =====================

def main():

    torch.backends.cudnn.benchmark = True
    torch.backends.cuda.matmul.allow_tf32 = True

    device = torch.device("cuda")

    print("GPU:", torch.cuda.get_device_name(0))

    # Load dataset
    df = pd.read_csv("D:/Coding/ML Assignment/ML/dataset_ik.csv")

    angles = df[['t1','t2','t3']].values.astype(np.float32)
    pos = df[['x','y','z']].values.astype(np.float32)

    # Normalize inputs and outputs
    scaler_angles = StandardScaler()
    scaler_pos = StandardScaler()

    angles = scaler_angles.fit_transform(angles)
    pos = scaler_pos.fit_transform(pos)

    joblib.dump(scaler_angles, "scaler_angles.save")
    joblib.dump(scaler_pos, "scaler_pos.save")

    X_train, X_test, y_train, y_test = train_test_split(
        angles, pos, test_size=0.1
    )

    X_train = torch.tensor(X_train, device=device)
    y_train = torch.tensor(y_train, device=device)

    X_test = torch.tensor(X_test, device=device)
    y_test = torch.tensor(y_test, device=device)

    model = FKNet().to(device)

    optimizer = optim.AdamW(
        model.parameters(),
        lr=0.001,
        fused=True
    )

    loss_fn = nn.MSELoss()

    batch_size = 32768
    epochs = 30

    print("\nTraining FK model...")

    start = time.time()

    for epoch in range(epochs):

        perm = torch.randperm(X_train.size(0), device=device)

        total_loss = 0

        for i in range(0, X_train.size(0), batch_size):

            idx = perm[i:i+batch_size]

            pred = model(X_train[idx])

            loss = loss_fn(pred, y_train[idx])

            optimizer.zero_grad(set_to_none=True)
            loss.backward()
            optimizer.step()

            total_loss += loss.item()

        print(f"Epoch {epoch+1} Loss {total_loss:.6f}")

    torch.cuda.synchronize()

    print("Training time:", time.time()-start)

    torch.save(model.state_dict(), "fk_model.pth")

    print("FK model saved")


if __name__ == "__main__":
    main()

import torch
import torch.nn as nn
import joblib

# same model definition
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


device = torch.device("cpu")

model = FKNet()
model.load_state_dict(torch.load("fk_model.pth", weights_only=True))
model.eval()

dummy = torch.randn(1,3)

torch.onnx.export(
    model,
    dummy,
    "fk_model.onnx",
    input_names=["angles"],
    output_names=["position"],
    opset_version=17
)

print("Exported fk_model.onnx")

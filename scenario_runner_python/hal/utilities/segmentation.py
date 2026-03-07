"""Lightweight segmentation inference wrapper for TorchScript lane models.

Provides a simple synchronous API: load model, predict(frame) -> binary_mask.
Falls back gracefully when PyTorch is not installed.
"""
import cv2
import numpy as np

try:
    import torch
    TORCH_AVAILABLE = True
except Exception:
    TORCH_AVAILABLE = False


class SegmentationModel:
    def __init__(self, model_path, device=None, input_size=(320, 160), threshold=0.5):
        self.model_path = model_path
        self.input_size = input_size  # (W, H)
        self.threshold = threshold
        if device is None:
            self.device = 'cuda' if TORCH_AVAILABLE and torch.cuda.is_available() else 'cpu'
        else:
            self.device = device

        self.model = None
        if TORCH_AVAILABLE:
            try:
                self.model = torch.jit.load(model_path, map_location=self.device)
                self.model.eval()
            except Exception:
                # model failed to load
                self.model = None

    def preprocess(self, frame):
        # frame: BGR uint8
        w, h = self.input_size
        img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        img = cv2.resize(img, (w, h), interpolation=cv2.INTER_LINEAR)
        img = img.astype(np.float32) / 255.0
        # HWC -> CHW
        img = img.transpose(2, 0, 1)
        tensor = None
        if TORCH_AVAILABLE and self.model is not None:
            tensor = torch.from_numpy(img).unsqueeze(0).to(self.device)
        return tensor, (frame.shape[1], frame.shape[0])

    def postprocess(self, out, out_size):
        # out: torch tensor or numpy array; expected shape (1,1,H,W) or (1,H,W)
        if out is None:
            return None
        if TORCH_AVAILABLE and isinstance(out, torch.Tensor):
            arr = out.detach().cpu().numpy()
        else:
            arr = np.array(out)

        # squeeze to HxW
        arr = np.squeeze(arr)
        # normalize to 0..255
        arr = (arr - arr.min()) / max(1e-6, (arr.max() - arr.min()))
        arr = (arr * 255.0).astype('uint8')
        # resize to original frame size
        w, h = out_size
        mask = cv2.resize(arr, (w, h), interpolation=cv2.INTER_NEAREST)
        # threshold
        _, bin_mask = cv2.threshold(mask, int(self.threshold * 255), 255, cv2.THRESH_BINARY)
        return bin_mask

    def predict(self, frame):
        """Return binary mask (uint8) same size as input frame, or None on failure."""
        if not TORCH_AVAILABLE or self.model is None:
            return None
        tensor, out_size = self.preprocess(frame)
        if tensor is None:
            return None
        with torch.no_grad():
            try:
                out = self.model(tensor)
            except Exception:
                try:
                    # Some TorchScript models return dict
                    out = self.model.forward(tensor)
                except Exception:
                    return None

        # handle common output shapes
        if isinstance(out, (list, tuple)):
            out = out[0]

        return self.postprocess(out, out_size)


def create_segmentation_model(model_path, device=None, input_size=(320, 160), threshold=0.5):
    if not TORCH_AVAILABLE:
        return None
    try:
        return SegmentationModel(model_path, device=device, input_size=input_size, threshold=threshold)
    except Exception:
        return None

"""The NumPy-backed subset of Ultralytics Boxes used by the trackers."""

import numpy as np


class BoxesShim:
    """Expose tracker box arrays without importing the Ultralytics package."""

    def __init__(
        self,
        conf: np.ndarray,
        cls: np.ndarray,
        xywh: np.ndarray,
        xyxy: np.ndarray,
    ) -> None:
        self.conf = np.asarray(conf, dtype=np.float32)
        self.cls = np.asarray(cls, dtype=np.float32)
        self.xywh = np.asarray(xywh, dtype=np.float32)
        self.xyxy = np.asarray(xyxy, dtype=np.float32)
        count = len(self.conf)
        if self.conf.shape != (count,) or self.cls.shape != (count,):
            raise ValueError("conf and cls must be one-dimensional arrays")
        if self.xywh.shape != (count, 4) or self.xyxy.shape != (count, 4):
            raise ValueError("xywh and xyxy must have shape (N, 4)")

    def __len__(self) -> int:
        return len(self.conf)

    def __getitem__(self, mask: np.ndarray) -> "BoxesShim":
        mask = np.asarray(mask)
        if mask.dtype != np.bool_ or mask.shape != (len(self),):
            raise TypeError("BoxesShim indexing requires a boolean mask with shape (N,)")
        return BoxesShim(
            conf=self.conf[mask],
            cls=self.cls[mask],
            xywh=self.xywh[mask],
            xyxy=self.xyxy[mask],
        )

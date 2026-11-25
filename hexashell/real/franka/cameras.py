"""RealSense cameras through Intel's pyrealsense2: aligned colour and depth, one pipeline per serial."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .rig import CameraMount

#: Frames to wait for one aligned pair (ms)
FRAME_TIMEOUT_MS = 5000
#: Frames dropped after a start, while auto-exposure settles
SETTLE_FRAMES = 30


@dataclass
class Picture:
    rgb: np.ndarray
    depth: np.ndarray  # metres along the optical axis; NaN where the camera measured nothing
    k: np.ndarray
    timestamp: float


def depth_metres(raw: np.ndarray, scale_m: float) -> np.ndarray:
    """Metres from RealSense's z16 units; its 0 means no measurement."""
    depth = raw.astype(np.float32) * float(scale_m)
    depth[raw == 0] = np.nan
    return depth


class RealSenseCamera:
    def __init__(self, mount: CameraMount):
        import pyrealsense2 as rs

        self.rs = rs
        self.mount = mount
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_device(mount.serial)
        config.enable_stream(rs.stream.depth, mount.width, mount.height, rs.format.z16, mount.fps)
        config.enable_stream(rs.stream.color, mount.width, mount.height, rs.format.rgb8, mount.fps)
        profile = self.pipeline.start(config)
        self.scale_m = float(profile.get_device().first_depth_sensor().get_depth_scale())
        self.align = rs.align(rs.stream.color)
        self.filter = None
        if mount.temporal_filter_alpha is not None:
            self.filter = rs.temporal_filter()
            self.filter.set_option(rs.option.filter_smooth_alpha, float(mount.temporal_filter_alpha))
        for _ in range(SETTLE_FRAMES):
            self.pipeline.wait_for_frames(FRAME_TIMEOUT_MS)

    def capture(self) -> Picture:
        frames = self.align.process(self.pipeline.wait_for_frames(FRAME_TIMEOUT_MS))
        color, depth = frames.get_color_frame(), frames.get_depth_frame()
        if not color or not depth:
            raise RuntimeError(f"camera {self.mount.name}: no complete aligned frame")
        if self.filter is not None:
            depth = self.filter.process(depth)
        intrinsics = color.profile.as_video_stream_profile().intrinsics
        k = np.array([[intrinsics.fx, 0.0, intrinsics.ppx], [0.0, intrinsics.fy, intrinsics.ppy], [0.0, 0.0, 1.0]])
        return Picture(np.asanyarray(color.get_data()).copy(), depth_metres(np.asanyarray(depth.get_data()), self.scale_m),
                       k, color.get_timestamp() / 1000.0)

    def close(self) -> None:
        self.pipeline.stop()

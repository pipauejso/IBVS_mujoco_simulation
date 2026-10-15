"""Offscreen RGB rendering; MuJoCo owns and closes the GL context."""

import mujoco


class MujocoRenderer:
    def __init__(self, model, data, camera_name=None, resolution=(320, 240)):
        self.model, self.data = model, data
        self.camera_name = camera_name if camera_name is not None else -1
        self.resolution = resolution
        self.renderer = None

    def setup_offscreen_rendering(self):
        self.renderer = mujoco.Renderer(
            self.model, height=self.resolution[1], width=self.resolution[0])

    def render_image(self):
        if self.renderer is None:
            raise RuntimeError("Call setup_offscreen_rendering() first")
        self.renderer.update_scene(self.data, camera=self.camera_name)
        # Renderer already returns top-left-origin RGB.
        return self.renderer.render().copy()

    def is_window_open(self):
        return self.renderer is not None

    def cleanup(self):
        if self.renderer is not None:
            self.renderer.close()
            self.renderer = None

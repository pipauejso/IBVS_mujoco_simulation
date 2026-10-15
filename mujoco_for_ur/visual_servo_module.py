"""Four-point IBVS, using RGB detections and simulation-only metric depth."""

import cv2
import numpy as np


class VisualServoController:
    def __init__(self, model, data, camera_name, target_name, resolution,
                 lambda_gain=2.0, tolerance_px=0.2):
        self.model, self.data = model, data
        self.resolution = resolution
        self.lambda_gain = lambda_gain
        self.tolerance_px = tolerance_px
        self.camera_id = model.camera(camera_name).id
        self.target_body_id = model.body(target_name).id
        self.geom_id = int(model.body_geomadr[self.target_body_id])
        self.target_size = model.geom_size[self.geom_id].copy()
        self.fx, self.fy, self.cx, self.cy = self._calculate_camera_intrinsics()
        self.desired_points = self.desired_z = self.current_points = None
        self.error_rms_px = None
        self.max_joint_speed = 0.5

    def _calculate_camera_intrinsics(self):
        width, height = self.resolution
        focal = height / (2 * np.tan(np.deg2rad(
            self.model.cam_fovy[self.camera_id]) / 2))
        return float(focal), float(focal), (width - 1) / 2, (height - 1) / 2

    def _extract_features_from_image(self, image):
        hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
        mask = (cv2.inRange(hsv, (0, 70, 40), (10, 255, 255)) |
                cv2.inRange(hsv, (170, 70, 40), (180, 255, 255)))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None, False
        contour = max(contours, key=cv2.contourArea)
        if cv2.contourArea(contour) < 30:
            return None, False
        # An enclosing rectangle destroys perspective on a tilted target.
        polygon = cv2.approxPolyDP(contour, 0.02 * cv2.arcLength(contour, True), True)
        if len(polygon) != 4 or not cv2.isContourConvex(polygon):
            return None, False
        corners = polygon.reshape(4, 2).astype(np.float32)
        width, height = self.resolution
        if (np.any(corners[:, 0] < 5) or np.any(corners[:, 0] > width - 6) or
                np.any(corners[:, 1] < 5) or np.any(corners[:, 1] > height - 6)):
            return None, False
        term = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        corners = cv2.cornerSubPix(mask, corners.reshape(-1, 1, 2),
                                  (4, 4), (-1, -1), term).reshape(4, 2)
        centre = corners.mean(axis=0)
        order = np.argsort(np.arctan2(corners[:, 1] - centre[1],
                                     corners[:, 0] - centre[0]))
        corners = corners[order]
        if self.current_points is None:
            corners = np.roll(corners, -np.argmin(corners.sum(axis=1)), axis=0)
        else:
            # Preserve identities across the 45-degree sorting boundary.
            candidates = [np.roll(corners, i, axis=0) for i in range(4)]
            corners = min(candidates, key=lambda p: np.sum((p - self.current_points)**2))
        return corners.copy(), True

    def _get_depth_for_points(self, points):
        """Intersect each detected pixel ray with the visible target face.

        Depth is simulator geometry, NOT an RGB depth estimator. Ray
        intersection preserves point/depth correspondence for tilted targets.
        """
        rotation = self.data.cam_xmat[self.camera_id].reshape(3, 3)
        origin = self.data.cam_xpos[self.camera_id]
        geom_rotation = self.data.geom_xmat[self.geom_id].reshape(3, 3)
        normal = geom_rotation[:, 2]
        centre = self.data.geom_xpos[self.geom_id]
        side = 1 if np.dot(normal, origin - centre) >= 0 else -1
        face = centre + side * self.target_size[2] * normal
        optical_rays = np.column_stack(((points[:, 0] - self.cx) / self.fx,
                                        (points[:, 1] - self.cy) / self.fy,
                                        np.ones(4)))
        world_rays = optical_rays @ np.diag([1., -1., -1.]) @ rotation.T
        denominators = world_rays @ normal
        if np.any(np.abs(denominators) < 1e-8):
            return None
        depths = np.dot(face - origin, normal) / denominators
        if not np.all(np.isfinite(depths)) or np.any(depths <= 0.01):
            return None
        return depths

    def get_feature_points(self, image=None):
        if image is None or not isinstance(image, np.ndarray):
            self.error_rms_px = None
            return None, None
        points, success = self._extract_features_from_image(image)
        if not success:
            self.error_rms_px = None
            return None, None
        depths = self._get_depth_for_points(points)
        if depths is None:
            self.error_rms_px = None
            return None, None
        self.current_points = points
        self.error_rms_px = (None if self.desired_points is None else
                             float(np.sqrt(np.mean((points - self.desired_points)**2))))
        return points, depths

    def set_desired_features(self, points_star):
        if points_star is None:
            return False
        points = np.asarray(points_star, dtype=np.float64)
        if points.shape != (4, 2) or not np.all(np.isfinite(points)):
            return False
        depths = self._get_depth_for_points(points)
        if depths is None:
            return False
        self.desired_points = points.copy()
        self.desired_z = depths.copy()
        return True

    def calculate_interaction_matrix(self, points, Z):
        depths = np.asarray(Z, dtype=np.float64).reshape(4)
        if np.any(depths <= 0) or not np.all(np.isfinite(depths)):
            raise ValueError("Feature depths must be finite and positive")
        L = np.zeros((8, 6))
        for i, ((u, v), z) in enumerate(zip(points, depths)):
            x, y = (u - self.cx) / self.fx, (v - self.cy) / self.fy
            L[2*i:2*i+2] = [[-1/z, 0, x/z, x*y, -(1+x*x), y],
                              [0, -1/z, y/z, 1+y*y, -x*y, -x]]
        return L

    def process_visual_servo(self, robot_controller, current_joint_angles=None, image=None):
        points, depths = self.get_feature_points(image)
        # Always overwrite old actuator commands when features are lost.
        if points is None:
            return np.zeros(self.model.nv)
        if self.desired_points is None:
            self.set_desired_features(points)
            return np.zeros(self.model.nv)
        if self.error_rms_px < self.tolerance_px:
            return np.zeros(self.model.nv)
        # e = current - desired; sdot = L J qdot; qdot = -gain (LJ)^+ e.
        error = ((points - self.desired_points) / [self.fx, self.fy]).reshape(8)
        image_jacobian = (self.calculate_interaction_matrix(points, depths) @
                          robot_controller.get_jacobian(current_joint_angles))
        damping = 1e-4
        velocity = -self.lambda_gain * np.linalg.solve(
            image_jacobian.T @ image_jacobian + damping**2 * np.eye(self.model.nv),
            image_jacobian.T @ error)
        if not np.all(np.isfinite(velocity)):
            return np.zeros(self.model.nv)
        # Uniform scaling preserves the descent direction under saturation.
        velocity *= min(1., self.max_joint_speed / max(np.max(np.abs(velocity)), 1e-12))
        return velocity

    def draw_feature_points(self, image):
        result = image.copy()
        for points, colour in ((self.current_points, (0, 255, 0)),
                               (self.desired_points, (255, 255, 0))):
            if points is None:
                continue
            contour = np.int32(np.round(points))
            cv2.polylines(result, [contour], True, colour, 1)
            for i, (u, v) in enumerate(contour):
                cv2.circle(result, (int(u), int(v)), 3, colour, 1)
                cv2.putText(result, str(i+1), (int(u)+4, int(v)-4),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, colour, 1)
        return result

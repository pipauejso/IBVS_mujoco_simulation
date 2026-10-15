"""Interactive UR5e IBVS and deterministic rendered-image reproduction."""
import sys
sys.dont_write_bytecode = True

import argparse
from contextlib import ExitStack
import csv
import json
from pathlib import Path
import platform
import time

import cv2
import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

from rbt_module import RobotController
from mujoco_render_module import MujocoRenderer
from opencv_render_module import OpenCVRenderer
from keyboard_controller import KeyboardController
from visual_servo_module import VisualServoController

ROOT = Path(__file__).resolve().parent
SCENE = ROOT / "universal_robots_ur5e" / "scene.xml"


def load_simulation():
    model = mujoco.MjModel.from_xml_path(str(SCENE))
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, 0)
    data.qvel[:] = 0
    data.ctrl[:] = 0
    # Activation indices need not equal actuator indices.
    for aid in range(model.nu):
        adr = int(model.actuator_actadr[aid])
        if adr >= 0:
            jid = int(model.actuator_trnid[aid, 0])
            data.act[adr] = data.qpos[model.jnt_qposadr[jid]]
    # Settle the dynamic plant before capturing the desired image.
    mujoco.mj_step(model, data, nstep=100)
    mujoco.mj_forward(model, data)
    return model, data


def optical_pose(data, camera_id):
    rotation = data.cam_xmat[camera_id].reshape(3, 3) @ np.diag([1., -1., -1.])
    return rotation, data.cam_xpos[camera_id].copy()


def target_relative_pose(model, data, camera_id):
    rotation, position = optical_pose(data, camera_id)
    target = model.body("target").id
    return (rotation.T @ data.xmat[target].reshape(3, 3),
            rotation.T @ (data.xpos[target] - position))


def perturb_target(model, data, camera_id, scenario):
    target = model.body("target").id
    mocap = int(model.body_mocapid[target])
    rotation, _ = optical_pose(data, camera_id)
    if scenario in ("translation", "combined"):
        data.mocap_pos[mocap] += rotation @ np.array([0.025, -0.018, 0.])
    if scenario in ("depth", "combined"):
        data.mocap_pos[mocap] += rotation @ np.array([0., 0., 0.03])
    initial = Rotation.from_quat(data.mocap_quat[mocap][[1, 2, 3, 0]])
    if scenario == "rotation":
        change = Rotation.from_euler("z", 25, degrees=True)
    elif scenario == "tilt":
        change = Rotation.from_euler("xy", [12, -10], degrees=True)
    elif scenario == "combined":
        change = Rotation.from_euler("xyz", [8, -6, 18], degrees=True)
    else:
        change = Rotation.identity()
    data.mocap_quat[mocap] = (initial * change).as_quat()[[3, 0, 1, 2]]
    mujoco.mj_forward(model, data)


def save_rgb(path, image):
    if not cv2.imwrite(str(path), cv2.cvtColor(image, cv2.COLOR_RGB2BGR)):
        raise RuntimeError(f"Could not save image: {path}")


def save_results(output, records, model, data, controller, reference_pose, scenario, dt):
    columns = (["time_s", "error_rms_px", "detected"] +
               [f"q{i+1}_rad" for i in range(6)] +
               [f"qdot{i+1}_rad_s" for i in range(6)] +
               [f"command{i+1}_rad_s" for i in range(6)])
    with (output / "trajectory.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(columns)
        writer.writerows(records)
    current_rotation, current_position = target_relative_pose(
        model, data, controller.camera_id)
    rotation_error = Rotation.from_matrix(current_rotation @ reference_pose[0].T)
    final = controller.error_rms_px
    losses = sum(row[2] == 0 for row in records)
    summary = {
        "scenario": scenario,
        "python": platform.python_version(), "mujoco": mujoco.__version__,
        "numpy": np.__version__, "opencv": cv2.__version__,
        "control_period_s": dt, "physics_timestep_s": float(model.opt.timestep),
        "control_frames": len(records), "duration_sim_s": len(records) * dt,
        "initial_error_rms_px": records[0][1] if records else None,
        "final_error_rms_px": final,
        "feature_loss_frames": losses,
        "max_command_rad_s": max((max(abs(v) for v in row[-6:]) for row in records), default=0),
        "relative_position_error_m": float(np.linalg.norm(current_position - reference_pose[1])),
        "relative_orientation_error_deg": float(np.rad2deg(rotation_error.magnitude())),
        "residual_rotation_optical_z_deg": float(np.rad2deg(rotation_error.as_rotvec()[2])),
    }
    summary["passed"] = bool(final is not None and final < 1.0 and losses == 0
                             and summary["relative_orientation_error_deg"] < 3.0)
    (output / "summary.json").write_text(
        json.dumps(summary, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(2, 1, figsize=(8, 6), sharex=True)
    t = [r[0] for r in records]
    axes[0].plot(t, [r[1] for r in records])
    axes[0].axhline(1., color="gray", linestyle="--", linewidth=1)
    axes[0].set_ylabel("Feature RMS (px)")
    axes[0].set_title(f"Rendered-image IBVS: {scenario}")
    values = np.array([r[-6:] for r in records])
    if len(values):
        for i in range(6):
            axes[1].plot(t, values[:, i], label=f"Joint {i+1}")
    axes[1].set_ylabel("Command (rad/s)")
    axes[1].set_xlabel("Simulation time (s)")
    axes[1].legend(ncol=3, fontsize=8)
    for ax in axes:
        ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(output / "convergence.png", dpi=160)
    plt.close(fig)
    print(json.dumps(summary, indent=2))
    return summary


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--headless", action="store_true", help="Offscreen rendering, no GUI windows")
    parser.add_argument("--demo", action="store_true", help="Capture reference, perturb target, enable IBVS")
    parser.add_argument("--scenario", choices=["translation", "depth", "rotation", "tilt", "combined"],
                        default="translation")
    parser.add_argument("--steps", type=int, default=None, help="Control frames; demo default 600")
    parser.add_argument("--output", type=Path, help="Results directory, default <repo>/results/<scenario>")
    parser.add_argument("--record", action="store_true", help="Save annotated camera.mp4 (50 fps)")
    args = parser.parse_args(argv)
    if args.headless and not args.demo:
        parser.error("--headless requires --demo")
    if args.steps is not None and args.steps <= 0:
        parser.error("--steps must be positive")
    limit = args.steps if args.steps is not None else (600 if args.demo else None)
    run_name = args.scenario if args.demo else "interactive"
    output = (args.output or ROOT.parent / "results" / run_name).resolve()
    output.mkdir(parents=True, exist_ok=True)

    model, data = load_simulation()
    robot = RobotController(model, data)
    resolution = (320, 240)
    controller = VisualServoController(model, data, "end_effector_camera",
                                       "target", resolution)
    renderer = MujocoRenderer(model, data, "end_effector_camera", resolution)
    period_steps = max(1, round(0.02 / model.opt.timestep))
    dt = period_steps * float(model.opt.timestep)
    records = []
    enabled = args.demo
    video = None
    with ExitStack() as cleanup:
        renderer.setup_offscreen_rendering()
        cleanup.callback(renderer.cleanup)
        reference = renderer.render_image()
        points, _ = controller.get_feature_points(reference)
        if not controller.set_desired_features(points):
            raise RuntimeError("No complete target visible in reference image")
        reference_pose = target_relative_pose(model, data, controller.camera_id)
        save_rgb(output / "reference.png", controller.draw_feature_points(reference))
        if args.demo:
            perturb_target(model, data, controller.camera_id, args.scenario)
        keyboard = gui = viewer = None
        if not args.headless:
            import mujoco.viewer as mj_viewer
            # Create passive viewer BEFORE offscreen render calls in the loop.
            viewer = cleanup.enter_context(mj_viewer.launch_passive(model, data))
            gui = OpenCVRenderer("End-Effector Camera", resolution)
            cleanup.callback(gui.cleanup)
            keyboard = KeyboardController(model, data, "target", step_size=0.01)
            print("Reference captured. v: servo; t: new reference; s: screenshot; ESC: quit.")
        if args.record:
            video = cv2.VideoWriter(str(output / "camera.mp4"),
                                    cv2.VideoWriter_fourcc(*"mp4v"), 1 / dt, resolution)
            if not video.isOpened():
                raise RuntimeError("Could not open MP4 encoder")
            cleanup.callback(video.release)
        start_time = float(data.time)
        frame = 0
        while limit is None or frame < limit:
            wall_start = time.perf_counter()
            if viewer is not None and not viewer.is_running():
                break
            image = renderer.render_image()
            if enabled:
                command = controller.process_visual_servo(robot, image=image)
            else:
                controller.get_feature_points(image)
                command = np.zeros(model.nu)
            data.ctrl[:] = command
            error = controller.error_rms_px
            records.append([float(data.time) - start_time, error, int(error is not None),
                            *data.qpos[:6], *data.qvel[:6], *command])
            annotated = controller.draw_feature_points(image)
            if frame == 0:
                save_rgb(output / "initial.png", annotated)
            if video is not None:
                video.write(cv2.cvtColor(annotated, cv2.COLOR_RGB2BGR))
            if gui is not None:
                key = gui.show_image(annotated)
                if key == 27 or not gui.is_window_open():
                    break
                if key == ord("v"):
                    enabled = not enabled
                    data.ctrl[:] = 0
                    print(f"Servo {'enabled' if enabled else 'disabled'}")
                elif key == ord("t"):
                    if controller.set_desired_features(controller.current_points
                                                       if error is not None else None):
                        reference_pose = target_relative_pose(model, data, controller.camera_id)
                        print("Reference updated")
                elif key == ord("s"):
                    save_rgb(output / f"screenshot_{frame}.png", annotated)
                else:
                    keyboard.process_key_input(key)
            mujoco.mj_step(model, data, nstep=period_steps)
            # mj_step does not refresh every derived pose after integration.
            mujoco.mj_forward(model, data)
            frame += 1
            if frame % 100 == 0:
                print(f"frame={frame}, sim={frame*dt:.2f}s, RMS={error}")
            if viewer is not None:
                viewer.sync()
                remaining = dt - (time.perf_counter() - wall_start)
                if remaining > 0:
                    time.sleep(remaining)
        data.ctrl[:] = 0
        final_image = renderer.render_image()
        controller.get_feature_points(final_image)
        save_rgb(output / "final.png", controller.draw_feature_points(final_image))
    summary = save_results(output, records, model, data, controller,
                           reference_pose, run_name, dt)
    return 0 if not args.demo or summary["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())

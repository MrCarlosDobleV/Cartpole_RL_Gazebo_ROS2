import time
from collections import deque

import matplotlib.pyplot as plt
import rclpy

from cartpole_application.cartpole_ros_node import CartPoleROSNode


def main():
    if not rclpy.ok():
        rclpy.init()

    node = CartPoleROSNode()

    history = 400
    t_hist = deque(maxlen=history)
    pos_hist = deque(maxlen=history)
    vel_hist = deque(maxlen=history)
    theta_hist = deque(maxlen=history)
    theta_dot_hist = deque(maxlen=history)
    cmd_hist = deque(maxlen=history)

    cmd = 0.0
    cmd_step = 0.0
    last_switch = time.time()

    plt.ion()
    fig, axes = plt.subplots(4, 1, sharex=True, figsize=(9, 8))
    fig.suptitle("CartPole ROS Node Live Test")

    (line_cmd,) = axes[0].plot([], [], label="cmd_vel")
    axes[0].set_ylabel("cmd")
    axes[0].legend(loc="upper right")

    (line_pos,) = axes[1].plot([], [], label="x", color="tab:blue")
    axes[1].set_ylabel("position")
    axes[1].legend(loc="upper right")

    (line_vel,) = axes[2].plot([], [], label="x_dot", color="tab:orange")
    axes[2].set_ylabel("velocity")
    axes[2].legend(loc="upper right")

    (line_theta,) = axes[3].plot([], [], label="theta", color="tab:green")
    axes[3].set_ylabel("theta")
    axes[3].set_xlabel("time (s)")
    axes[3].legend(loc="upper right")

    start = time.time()
    step_idx = 0

    try:
        while plt.fignum_exists(fig.number):
            # alternate command every 2 seconds
            if time.time() - last_switch > 2.0:
                cmd_step = -cmd_step if cmd_step != 0.0 else 2.0
                cmd = cmd_step
                last_switch = time.time()

            node.apply_vel(cmd)
            rclpy.spin_once(node, timeout_sec=0.02)

            try:
                x, x_dot, theta, theta_dot = node.get_state()
            except RuntimeError:
                continue

            t = time.time() - start
            t_hist.append(t)
            pos_hist.append(x)
            vel_hist.append(x_dot)
            theta_hist.append(theta)
            theta_dot_hist.append(theta_dot)
            cmd_hist.append(cmd)

            line_cmd.set_data(t_hist, cmd_hist)
            line_pos.set_data(t_hist, pos_hist)
            line_vel.set_data(t_hist, vel_hist)
            line_theta.set_data(t_hist, theta_hist)

            if step_idx % 5 == 0:
                for ax in axes:
                    ax.relim()
                    ax.autoscale_view()

            step_idx += 1
            plt.pause(0.001)

    except KeyboardInterrupt:
        pass
    finally:
        node.apply_vel(0.0)
        node.destroy_node()
        # rclpy.shutdown()


if __name__ == "__main__":
    main()

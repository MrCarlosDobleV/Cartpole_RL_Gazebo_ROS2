import time
from collections import deque

import matplotlib.pyplot as plt
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class ControllerTest(Node):
    def __init__(self):
        super().__init__("cartpole_controller_test")
        self.state = None
        self.create_subscription(JointState, "/joint_states", self._joint_cb, 10)
        self.cmd_pub = self.create_publisher(
            Float64MultiArray,
            "/cart_velocity_controller/commands",
            10,
        )

    def _joint_cb(self, msg: JointState):
        self.state = msg

    def get_state(self):
        if self.state is None:
            return None
        try:
            i_cart = self.state.name.index("pole_hing")
        except ValueError:
            return None
        return self.state.position[i_cart], self.state.velocity[i_cart]

    def send_vel(self, vel: float):
        msg = Float64MultiArray()
        msg.data = [float(vel)]
        self.cmd_pub.publish(msg)


def main():
    rclpy.init()
    node = ControllerTest()

    history = 400
    t_hist = deque(maxlen=history)
    pos_hist = deque(maxlen=history)
    vel_hist = deque(maxlen=history)
    cmd_hist = deque(maxlen=history)

    cmd = 0.0
    cmd_step = 0.0
    last_switch = time.time()

    plt.ion()
    fig, axes = plt.subplots(3, 1, sharex=True, figsize=(9, 6))
    fig.suptitle("CartPole Controller Test")

    (line_cmd,) = axes[0].plot([], [], label="cmd_vel")
    axes[0].set_ylabel("cmd")
    axes[0].legend(loc="upper right")

    (line_pos,) = axes[1].plot([], [], label="x", color="tab:blue")
    axes[1].set_ylabel("position")
    axes[1].legend(loc="upper right")

    (line_vel,) = axes[2].plot([], [], label="x_dot", color="tab:orange")
    axes[2].set_ylabel("velocity")
    axes[2].set_xlabel("time (s)")
    axes[2].legend(loc="upper right")

    start = time.time()
    step_idx = 0

    try:
        while plt.fignum_exists(fig.number):
            # alternate command every 2 seconds
            if time.time() - last_switch > 2.0:
                cmd_step = -cmd_step if cmd_step != 0.0 else 2.0
                cmd = cmd_step
                last_switch = time.time()

            node.send_vel(cmd)
            rclpy.spin_once(node, timeout_sec=0.02)

            state = node.get_state()
            if state is None:
                continue
            x, x_dot = state

            t = time.time() - start
            t_hist.append(t)
            pos_hist.append(x)
            vel_hist.append(x_dot)
            cmd_hist.append(cmd)

            line_cmd.set_data(t_hist, cmd_hist)
            line_pos.set_data(t_hist, pos_hist)
            line_vel.set_data(t_hist, vel_hist)

            if step_idx % 5 == 0:
                for ax in axes:
                    ax.relim()
                    ax.autoscale_view()

            step_idx += 1
            plt.pause(0.001)

    except KeyboardInterrupt:
        pass
    finally:
        node.send_vel(0.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

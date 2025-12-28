import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64


class CartPoleROSNode(Node):

    def __init__(self):
        super().__init__("cartpole_env_node")

        self._joint_state = None

        # Subscribe to joint states
        self.create_subscription(
            JointState,
            "/joint_states",
            self._joint_cb,
            10
        )

        # Publish force command
        self.force_pub = self.create_publisher(
            Float64,
            "/cart_force",
            10
        )

    def _joint_cb(self, msg: JointState):
        self._joint_state = msg

    def get_state(self):
        if self._joint_state is None:
            raise RuntimeError("No joint state received yet")

        msg = self._joint_state

        try:
            i_cart = msg.name.index("cart_slide")
            i_pole = msg.name.index("pole_hinge")
        except ValueError:
            raise RuntimeError(f"Expected joints not found: {msg.name}")

        x = msg.position[i_cart]
        x_dot = msg.velocity[i_cart]
        theta = msg.position[i_pole]
        theta_dot = msg.velocity[i_pole]

        return x, x_dot, theta, theta_dot

    def apply_force(self, force: float):
        msg = Float64()
        msg.data = float(force)
        self.force_pub.publish(msg)

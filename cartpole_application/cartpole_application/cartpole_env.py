import gymnasium as gym
import numpy as np
import rclpy

from cartpole_application.cartpole_ros_node import CartPoleROSNode


class GazeboCartPoleEnv(gym.Env):
    metadata = {"render_modes": []}

    def __init__(self):
        super().__init__()

        # ---- RL parameters ----
        self.dt = 0.02
        self.max_force = 10.0
        self.max_steps = 1000
        self.step_count = 0
        self.cart_limit = 2.4

        # ---- Action space (continuous) ----
        self.action_space = gym.spaces.Box(
            low=-self.max_force,
            high=self.max_force,
            shape=(1,),
            dtype=np.float32,
        )

        # ---- Observation space (like Gym CartPole, no theta limit) ----
        self.observation_space = gym.spaces.Box(
            low=-np.inf,
            high=np.inf,
            shape=(4,),
            dtype=np.float32,
        )

        # ---- ROS ----
        if not rclpy.ok():
            rclpy.init(args=None)

        self.node = CartPoleROSNode()

        # Wait for first joint state
        self._wait_for_state()

    def _wait_for_state(self):
        while True:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            try:
                self.node.get_state()
                break
            except RuntimeError:
                pass

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)

        self.step_count = 0

        self.node.apply_force(0.0)

        # Let the system settle
        for _ in range(5):
            
            rclpy.spin_once(self.node, timeout_sec=0.05)

        obs = np.array(self.node.get_state(), dtype=np.float32)
        return obs, {}

    def step(self, action):
        self.step_count += 1

        force = float(np.clip(action[0], -self.max_force, self.max_force))
        self.node.apply_force(force)

        rclpy.spin_once(self.node, timeout_sec=self.dt)


        x, x_dot, theta, theta_dot = self.node.get_state()
        obs = np.array([x, x_dot, theta, theta_dot], dtype=np.float32)

        # ---- Reward (simple swing-up style) ----
        reward = -np.cos(theta)          # swing-up objective
        reward -= 0.01 * x**2            # keep cart centered
        reward -= 0.001 * force**2       # energy penalty


        # ---- Termination ----
        terminated = abs(x) > self.cart_limit
        truncated = self.step_count >= self.max_steps

        return obs, reward, terminated, truncated, {}

    def close(self):
        self.node.destroy_node()
        # rclpy.shutdown()

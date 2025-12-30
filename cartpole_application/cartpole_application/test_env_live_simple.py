import time
from collections import deque

import matplotlib.pyplot as plt
import numpy as np

from cartpole_application.cartpole_env import GazeboCartPoleEnv


def main():
    env = GazeboCartPoleEnv()

    history = 400
    t_hist = deque(maxlen=history)
    action_hist = deque(maxlen=history)
    reward_hist = deque(maxlen=history)
    theta_hist = deque(maxlen=history)

    action_state = {
        "vel": 0.0,
        "reset": False,
        "quit": False,
    }

    def on_key(event):
        key = event.key
        if key in ("left", "a"):
            action_state["vel"] = -abs(action_state["vel"] or 10.0)
        elif key in ("right", "d"):
            action_state["vel"] = abs(action_state["vel"] or 10.0)
        elif key in ("up", "w"):
            action_state["vel"] = min(action_state["vel"] + 10.0, env.max_vel)
        elif key in ("down", "s"):
            action_state["vel"] = max(action_state["vel"] - 10.0, -env.max_vel)
        elif key in (" ", "0"):
            action_state["vel"] = 0.0
        elif key in ("r",):
            action_state["reset"] = True
        elif key in ("q", "escape"):
            action_state["quit"] = True

    plt.ion()
    fig, axes = plt.subplots(3, 1, sharex=True, figsize=(9, 6))
    fig.canvas.mpl_connect("key_press_event", on_key)
    fig.suptitle("CartPole Live Debug (action, reward, pole angle)")

    (line_action,) = axes[0].plot([], [], label="action vel")
    axes[0].set_ylabel("action")
    axes[0].legend(loc="upper right")

    (line_reward,) = axes[1].plot([], [], label="reward", color="tab:green")
    axes[1].set_ylabel("reward")
    axes[1].legend(loc="upper right")

    (line_theta,) = axes[2].plot([], [], label="theta", color="tab:orange")
    axes[2].set_ylabel("theta")
    axes[2].set_xlabel("time (s)")
    axes[2].legend(loc="upper right")
    axes[2].set_ylim(-2 * np.pi, 2 * np.pi)
    theta_ticks = [-2 * np.pi, -np.pi, 0.0, np.pi, 2 * np.pi]
    theta_labels = ["-2π", "-π", "0", "π", "2π"]
    axes[2].set_yticks(theta_ticks)
    axes[2].set_yticklabels(theta_labels)

    obs, _ = env.reset()
    start = time.time()
    step_idx = 0

    try:
        while plt.fignum_exists(fig.number):
            if action_state["quit"]:
                break

            if action_state["reset"]:
                obs, _ = env.reset()
                action_state["reset"] = False

            action = np.array([action_state["vel"]], dtype=np.float32)
            obs, reward, terminated, truncated, _ = env.step(action)
            theta = float(obs[2])

            t = time.time() - start
            t_hist.append(t)
            action_hist.append(action_state["vel"])
            reward_hist.append(reward)
            theta_hist.append(theta)

            line_action.set_data(t_hist, action_hist)
            line_reward.set_data(t_hist, reward_hist)
            line_theta.set_data(t_hist, theta_hist)

            if step_idx % 5 == 0:
                for ax in axes:
                    ax.relim()
                    ax.autoscale_view()

            if terminated or truncated:
                obs, _ = env.reset()

            step_idx += 1
            plt.pause(0.001)

    except KeyboardInterrupt:
        pass
    finally:
        env.close()


if __name__ == "__main__":
    main()

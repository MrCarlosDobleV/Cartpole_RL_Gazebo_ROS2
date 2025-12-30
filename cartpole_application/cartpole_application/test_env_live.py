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
    obs_hists = [deque(maxlen=history) for _ in range(4)]

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
    fig, axes = plt.subplots(3, 1, sharex=True, figsize=(9, 7))
    fig.canvas.mpl_connect("key_press_event", on_key)
    fig.suptitle("CartPole Live Debug (keys: left/right, up/down, space=stop, r=reset, q=quit)")

    (line_action,) = axes[0].plot([], [], label="action vel")
    axes[0].set_ylabel("action")
    axes[0].legend(loc="upper right")

    (line_reward,) = axes[1].plot([], [], label="reward", color="tab:green")
    axes[1].set_ylabel("reward")
    axes[1].legend(loc="upper right")

    obs_labels = ["x", "x_dot", "theta", "theta_dot"]
    obs_lines = []
    for i, label in enumerate(obs_labels):
        (line,) = axes[2].plot([], [], label=label)
        obs_lines.append(line)
    axes[2].set_ylabel("obs")
    axes[2].set_xlabel("time (s)")
    axes[2].legend(loc="upper right", ncol=2)

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

            t = time.time() - start
            t_hist.append(t)
            action_hist.append(action_state["vel"])
            reward_hist.append(reward)
            for i in range(4):
                obs_hists[i].append(obs[i])

            line_action.set_data(t_hist, action_hist)
            line_reward.set_data(t_hist, reward_hist)
            for i, line in enumerate(obs_lines):
                line.set_data(t_hist, obs_hists[i])

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

import numpy as np
from cartpole_application.cartpole_env import GazeboCartPoleEnv

env = GazeboCartPoleEnv()

obs, _ = env.reset()
print("Initial obs:", obs)

for i in range(200):
    action = np.array([np.random.uniform(-10.0, 10.0)], dtype=np.float32)

    obs, reward, terminated, truncated, _ = env.step(action)

    print(f"{i:03d} | action={action[0]:+.2f} obs={obs} reward={reward:.3f}")

    if terminated or truncated:
        print("Episode ended")
        break

env.close()

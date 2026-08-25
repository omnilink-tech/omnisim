# Copyright 2026 OmniLink
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Smoke test: prove SB3+PPO+gymnasium work on this Python install.

Trains PPO on CartPole-v1 for 5k steps, asserts the reward improves
above random baseline.
"""

from __future__ import annotations

import sys

import gymnasium as gym
from stable_baselines3 import PPO


def main() -> int:
    env = gym.make("CartPole-v1")
    model = PPO("MlpPolicy", env, verbose=0, n_steps=256, batch_size=64)

    # Baseline: 5 random rollouts
    baseline = []
    for _ in range(5):
        obs, _ = env.reset()
        total = 0.0
        for _ in range(500):
            a = env.action_space.sample()
            obs, r, term, trunc, _ = env.step(a)
            total += r
            if term or trunc:
                break
        baseline.append(total)
    base_mean = sum(baseline) / len(baseline)
    print(f"random baseline mean reward: {base_mean:.1f}")

    model.learn(total_timesteps=5000)

    trained = []
    for _ in range(5):
        obs, _ = env.reset()
        total = 0.0
        for _ in range(500):
            a, _ = model.predict(obs, deterministic=True)
            obs, r, term, trunc, _ = env.step(a)
            total += r
            if term or trunc:
                break
        trained.append(total)
    trained_mean = sum(trained) / len(trained)
    print(f"trained policy mean reward:  {trained_mean:.1f}")

    if trained_mean > base_mean + 10:
        print("SMOKE TEST PASS (learning works)")
        return 0
    print("SMOKE TEST FAIL (no learning)")
    return 1


if __name__ == "__main__":
    sys.exit(main())


# 🦾 IsaacLab-SO_100: Cube Lifting with SO100 Robot Arm

This project uses NVIDIA Isaac Lab and `skrl` to train a 5-DOF robotic arm (SO100) to **grasp and lift a cube** in simulation.

---

## 🧠 Project Summary: Cube Lifting with SO100 Robot

You're training the SO100 robot arm to:
- Identify and reach a cube
- Grasp it using a binary gripper
- Lift it vertically without dropping
- Optimize control smoothness via penalties
- Stabilize over time using curriculum rewards

The environment is defined using Isaac Lab's configuration system, and the reinforcement learning algorithm is Proximal Policy Optimization (PPO) using `skrl`.

---

## 🚀 Features

- **Simulation with Isaac Sim 4.5**
- **Reinforcement Learning via PPO** using [`skrl`](https://skrl.readthedocs.io/)
- **Custom environment** with:
  - SO100 robotic arm
  - Binary gripper
  - DexCube object on a table
- Full training config and environment manager setup

---

## 📦 Installation

### 1. Clone the Repository

```bash
git clone https://github.com/YOUR_USERNAME/IsaacLab-SO_100.git
cd IsaacLab-SO_100
```

> Replace `YOUR_USERNAME` with your actual GitHub username if applicable.

---

### 2. Activate IsaacLab Conda Environment

Make sure you have installed Isaac Lab and its dependencies. Then activate your environment:

```bash
conda activate env_isaaclab
```

---

### 3. Install the SO_100 Package in Editable Mode

```bash
python -m pip install -e source/SO_100
```

Expected output:
```
Successfully built SO_100
Successfully installed SO_100-0.1.0
```

---

## 🧪 Run Environment Check

Verify that the SO100 environment is detected by Isaac Lab:

```bash
python scripts/list_envs.py
```

Expected output:
```
| S. No. | Task Name                   | Entry Point                     | Config                                                                 |
|--------|-----------------------------|----------------------------------|------------------------------------------------------------------------|
|   1    | Template-So-100-CubeLift-v0 | isaaclab.envs:ManagerBasedRLEnv | SO_100.tasks.manager_based.so_100.so_100_cube_lift_env_cfg:SO100CubeLiftEnvCfg |
```

---

## 🏋️ Train the Agent (Headless)

Run PPO training using `skrl`:

```bash
python scripts/skrl/train.py --task=Template-So-100-CubeLift-v0 --headless
```

This will:
- Use the custom PPO config in `skrl_ppo_cfg.yaml`
- Train the agent to lift the cube
- Save checkpoints and logs in `SO100_lift/`

---

## 📁 File Structure

```
IsaacLab-SO_100/
├── source/
│   └── SO_100/                  # Main Python package (editable install)
├── scripts/
│   ├── list_envs.py            # Environment discovery
│   └── skrl/train.py           # Training entry point
├── skrl_ppo_cfg.yaml           # RL configuration
```

---

## 📋 Requirements

- Windows 10/11 with NVIDIA GPU (RTX recommended)
- Isaac Sim 4.5
- Isaac Lab (from Omniverse or source)
- Conda Python environment with dependencies (Isaac Lab docs)
- `skrl` installed via pip:
  ```bash
  pip install skrl
  ```

---



## 🔧 **Project Summary: Cube Manipulation with SO100 Robot**

This project uses a simulated 5-DOF robotic arm (SO100) to learn how to **grasp and lift a cube** using reinforcement learning (RL), implemented with **Isaac Lab + NVIDIA Omniverse Isaac Sim + `skrl` PPO**.

---

## 🧱 Project Components Breakdown

### 1. **Environment (`SO100CubeLiftEnvCfg`)**

This config defines everything about the simulated world.

#### 🏗 Scene

- **Robot:** SO100 robotic arm
- **Object:** A cube (`dex_cube_instanceable.usd`)
- **Table:** Seattle Lab table
- **Markers:** Visual aids for robot end-effector and cube
- **EE Frame:** `{ENV}/Robot/Fixed_Gripper` (used for control and tracking)

#### 🧠 MDP Definition

Markov Decision Process is defined in `so_100_base_env_cfg.py`:

- **Observations:**
  - Joint positions & velocities
  - Object position (in robot frame)
  - Target pose (commanded)
  - Last actions

- **Actions:**
  - **Arm:** Joint position control for 5 joints
  - **Gripper:** Binary open/close action

- **Rewards:**
  - +25 for lifting the object above 2cm
  - +2 for reaching object
  - Small penalties for:
    - High joint velocity
    - High action rate (to smooth motion)

- **Terminations:**
  - Timeout (episode end)
  - Object falls below certain height

- **Events:**
  - Scene resets between episodes

- **Curriculum Learning:**
  - Gradually increases penalties on erratic actions to stabilize behavior

---

### 2. **MDP Components**

Defined using modular functions in `mdp`, these handle the physics-based logic for:

- Action processing
- Observations
- Rewards
- Termination conditions
- Command sampling

---

### 3. **Training Config (`skrl_ppo_cfg.yaml`)**

You're training using **Proximal Policy Optimization (PPO)** from the `skrl` library.

#### 🧠 Policy Network

- Input: State (observations)
- Hidden layers: [256, 128, 64]
- Output: Action
- Activation: `ELU`
- Gaussian policy with log_std constraints

#### 📊 Value Network

- Similar structure as the policy network
- Outputs scalar value estimates

#### 🧮 PPO Parameters

- 24 rollout steps
- 8 learning epochs per update
- 4 minibatches
- Discount factor: `0.99`
- GAE λ: `0.95`
- Learning rate: `1e-4` with KL-adaptive scheduler
- Clipping:
  - Policy: `0.2`
  - Value: `0.2`
- Entropy bonus: `0.001`
- Value loss scale: `2.0`

#### 🧠 Normalization

- `RunningStandardScaler` used for both state and value normalization

#### 💾 Logging

- Logs and checkpoints go to: `SO100_lift/`

#### ⏱ Trainer

- `SequentialTrainer`
- 36,000 timesteps


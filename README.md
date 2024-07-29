# tbai
Towards better athletic intelligence

## Installing tbai
```bash
# Install dependencies
sudo apt install libmpfr-dev

# Download project
mkdir -p <your-workspace>/src && cd <your-file> && catkin init && cd src
git clone git@github.com:asblab-research/tbai.git --recursive

# Install other dependencies using rosdep
cd .. && rosdep install --from-paths src --ignore-src -r -y && cd src/tbai

# !! Now install libtorch by following the installation guideline below
```

## Installing Additional Dependencies

### 1. Installing libtorch C++
There are two steps to installing `libtorch`. First, you need to download a suitable `libtorch` version.
Once the library is downloaded, it's necessary to create a symlink to it in the `dependencies` folder.
Here's how to do it:

#### 1.1 Getting libtorch download link
Get your download link from the [official PyTorch website](https://pytorch.org/). Note that opting for the `(cxx11 ABI)` version is paramount.
If you download the `(Pre-cxx11 ABI)` version, things won't work as necessary.


![image](https://github.com/lnotspotl/tbai/assets/82883398/183255fc-83c5-4bab-a48d-f70e5c7593d7)


#### 1.2 Downloading libtorch and creating a symlink
Now that you have your url, you can download the library, unzip it and create a symlink in the `dependencies` folder.
```bash
wget <your-url> # Latest URL (JUL-29-2024) - https://download.pytorch.org/libtorch/cpu/libtorch-cxx11-abi-shared-with-deps-2.4.0%2Bcpu.zip
unzip <downloaded-zip> -d <your-workspace>/src/tbai/dependencies  # can be `dependencies` folder of the package
ln -s <your-folder>/libtorch dependencies  # Only necessary if, in the previous step, you did not unzip in `dependencies`
```
Your `dependencies` folder should not look as follows:
<p align="center">
  <img src="https://github.com/lnotspotl/tbai/assets/82883398/657d8681-1abd-4dae-b4c2-15347ed542fd" />
</p>

### Installing aws-robomaker-hospital-world
```bash
cd <your-workspace>/src
git clone git@github.com:asblab-research/aws-robomaker-hospital-world.git
```
## Building the package
```bash
#Build aws-robomaker-hospital-world
cd <your-workspace> && catkin build aws_robomaker_hospital_world

# Build tbai
cd src/tbai
catkin config -DCMAKE_BUILD_TYPE=Release
bash ./tbai.bash --build  # This will only build the necessary packages

# Source
cd ../.. && source devel/setup.bash
```
## Usage
Currently only tbai_mpc_perceptive and tbai_rl_perceptive are modified to use aws-robomaker-hospital-world.

### tbai_mpc_perceptive
```bash
# Start ROS and relevant nodes
roslaunch tbai_mpc_perceptive simple_hospital_two_floor.launch #single floor world can be launched using simple_hospital.launch

# Change controllers (in a different terminal)
rostopic pub /anymal_d/change_controller std_msgs/String "data: 'STAND'"
rostopic pub /anymal_d/change_controller std_msgs/String "data: 'WBC'"
#rostopic pub /anymal_d/change_controller std_msgs/String "data: 'SIT'"

# In the MPC controller terminal (started with roslaunch) change gait to trot
```
### tbai_rl_perceptive
# Start ROS and relevant nodes
roslaunch tbai_rl_perceptive simple.launch gui:=true

# Change controllers
```bash
# Start ROS and relevant nodes
roslaunch tbai_mpc_perceptive simple_hospital_two_floor.launch #single floor world can be launched using simple_hospital.launch

# Change controllers (in a different terminal)
rostopic pub /anymal_d/change_controller std_msgs/String "data: 'RL'"
rostopic pub /anymal_d/change_controller std_msgs/String "data: 'BOB'"  # RL and BOB are the same controllers
rostopic pub /anymal_d/change_controller std_msgs/String "data: 'STAND'"
#rostopic pub /anymal_d/change_controller std_msgs/String "data: 'SIT'"
```

## Implemented controllers

```
📦tbai
 ┣ 📂tbai_static               # Static (high gain PD) controller
 ┣ 📂tbai_mpc_perceptive       # Perceptive NMPC controller [1]
 ┣ 📂tbai_mpc_blind            # Blind NMPC controller [1]
 ┣ 📂tbai_rl_perceptive        # Perceptive RL controller [2]
 ┣ 📂tbai_rl_blind             # Blind RL controller [2]
 ┣ 📂tbai_dtc                  # DTC controller (perceptive) [3]
 ┣ 📂tbai_joe                  # Perceptive NMPC controller with NN-based tracking controller [1],[3]

 [1] Perceptive Locomotion through Nonlinear Model Predictive Control
     https://arxiv.org/abs/2208.08373
 [2] Learning robust perceptive locomotion for quadrupedal robots in the wild
     https://arxiv.org/abs/2201.08117
 [3] DTC: Deep Tracking Control
     https://arxiv.org/abs/2309.15462
```

## Perceptive MPC



https://github.com/lnotspotl/tbai/assets/82883398/f451c12d-7525-4606-b722-726f63d852ca




## Blind MPC



https://github.com/lnotspotl/tbai/assets/82883398/1bf86da1-a3d4-44db-88c4-877ec78b06cc




## Perceptive RL



https://github.com/lnotspotl/tbai/assets/82883398/7f6bdefa-4299-454b-a0ef-55e463e0c88d




## Blind RL


https://github.com/lnotspotl/tbai/assets/82883398/ebc2d90d-5c03-4207-a868-2e9436c140d4



## Dtc


https://github.com/lnotspotl/tbai/assets/82883398/6cf672db-b737-4724-a6da-afa0c8dd19d5


## Joe


https://github.com/lnotspotl/tbai/assets/82883398/e3455dd3-10e8-41da-bb02-87fbdf3de041


## System architecture

![overview_01](https://github.com/lnotspotl/tbai/assets/82883398/2c17f08d-6994-4982-8739-2b8246dfcb32)

## Controller architectures

## Mpc 
![mpc_03](https://github.com/lnotspotl/tbai/assets/82883398/daabb2c2-8ced-4ffd-956e-35279b78563b)


## Rl (Bob)

![bob_03](https://github.com/lnotspotl/tbai/assets/82883398/3ea71f1c-b58c-4028-93d3-971592aa364d)


## Dtc

![dtc_03](https://github.com/lnotspotl/tbai/assets/82883398/10b3481d-7782-4a0e-ac31-24e2786c3402)

## Joe

![joe_03](https://github.com/lnotspotl/tbai/assets/82883398/0139df20-d2ce-4de1-884f-ce37e770ee08)

If any of the steps throws an error for you, please let use know and we will try to extend this guideline with a fix as soon as possible. Thanks 🤗

## Credits
This project stands on the shoulders of giants.
None of this would have been possible were it not for many amazing open-source projects.
Here are a couple that most inspiration was drawn from and that were instrumental during the development:

- https://github.com/leggedrobotics/ocs2
- https://github.com/qiayuanl/legged_control
- https://github.com/leggedrobotics/legged_gym
- https://github.com/leggedrobotics/rsl_rl
- https://github.com/ANYbotics/elevation_mapping
- https://github.com/leggedrobotics/elevation_mapping_cupy
- https://github.com/bernhardpg/quadruped_locomotion
- https://github.com/stack-of-tasks/pinocchio
- https://x-io.co.uk/open-source-imu-and-ahrs-algorithms/
- https://github.com/mayataka/robotoc
- https://github.com/mayataka/legged_state_estimator
- hundreds of others ...

Thank you all 🤗

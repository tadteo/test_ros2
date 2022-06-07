# turtlebot exploration
We are using:
 - turtlebot: 3
 - ros: noetic
 - ubuntu: 20.04
 - slam: gmapping

## setup 
Install the following:
```bash
sudo apt install ros-noetic-turtlebot3 ros-noetic-slam-gmapping ros-noetic-dwa-local-planner
```

create workspace and download repo in it:
```bash
mkdir -p ~/turtle_catkin_ws/src
cd ~/turtle_catkin_ws/src
git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
git clone git@github.com:tadteo/turtlebot_exploration.git
cd ~/turtle_catkin_ws
catkin_make
```

remember to export the turtlebot env variable:
```bash
export TURTLEBOT3_MODEL=burger
```

## run

```bash
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```

## other
Moreo info [here](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#gazebo-simulation) 

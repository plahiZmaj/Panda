# Zapiski

## Priprava okolja 

libfranka mora biti verzija 0.5.0 (je že inštalirana)
rosfranka verzija 0.6.0 (jo je potrebno inštalirati za vsak workspace posebej)

### Catkin workspace
Ustvari nav catkin workspace.

```console
mkdir -p ./catkin_ws_feros/src
cd catkin_ws_feros/
catkin_make
source devel/setup.bash
```

### franka_ros

https://frankaemika.github.io/docs/installation_linux.html

```console
git clone --recursive https://github.com/frankaemika/franka_ros src/franka_ros
cd src/franka_ros/
git checkout 0.6.0
cd ../..
rosdep install --from-paths src --ignore-src --rosdistro melodic -y --skip-keys libfranka
catkin_make -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=/home/bambus/libfranka/build
source devel/setup.sh
```

### moveit config

Kopiraj pravi moveit config (`config` in `launch` mape).

V svojem catkin workspace izbriši mapo `panda_moveit_config` na poti `src/franka_ros` in jo nadomesti z mapo `panda_moveit_config` iz `/home/bambus/franka_ros/catkin_ws_ros_test01/src/franka_ros/`.


## Zagon Franka robota

### Osnovni Panda ROS kontroler

```console
cd /home/bambus/franka_ros/catkin_ws_sola
source ./devel/setup.sh
source devel/setup.bash

```

#### Primer 1

Franka ROS kontroler
Se ne zgodi nic posebnega, vazno je, da ni nobene napaka v zagonu.

Terminal 1
```console
cd /home/bambus/franka_ros/catkin_ws_sola
source devel/setup.bash
roslaunch franka_control franka_control.launch robot_ip:=192.168.4.20 load_gripper:=true
```

#### Primer 2

Franka ROS kontroler z vizualizacijo.
Odpre se rviz vizualizacija z robotom. Ce pritisnete na crn gumb in robota pomikate z roko, se bo to videlo v vizualizaciji.

Terminal 1
```console
cd /home/bambus/franka_ros/catkin_ws_sola
source devel/setup.bash
roslaunch franka_visualization franka_visualization.launch robot_ip:=192.168.4.20   load_gripper:=true
```

#### Primer 3

Impedanćno vodenje.
POZOR, robot se bo zacel pomikati po kroznici.

Terminal 2
```console
cd /home/bambus/franka_ros/catkin_ws_sola
source devel/setup.bash
roslaunch franka_example_controllers joint_impedance_example_controller.launch robot_ip:=192.168.4.20 load_gripper:=true
```

#### Primer 4

Pomik v zacetno tocko.
POZOR! Robot se bo zelo hitro premaknil v zacetno tocko.

```console
roslaunch franka_example_controllers move_to_start.launch robot_ip:=192.168.4.20 load_gripper:=true
```

#### Primer 5

Pomik v zacetno tocko.
POZOR! Robot se bo zelo hitro premaknil v zacetno tocko.
Nato se odpre uporabniski vmesnik za premikanje robota. 
POZOR! Premiki so hitri.

```console
roslaunch franka_example_controllers move_to_start.launch robot_ip:=192.168.4.20 load_gripper:=true

roslaunch franka_example_controllers cartesian_impedance_example_controller.launch robot_ip:=192.168.4.20 load_gripper:=true
```

#### Primer 6

Odpiranje in zapiranje prijemala

Terminal 1: zaženi franka kontroler.
```console
roslaunch franka_control franka_control.launch robot_ip:=192.168.4.20 load_gripper:=true
```
Terminal 2: ukazi za prijemalo
```console
rostopic pub /franka_gripper/move/goal franka_gripper/MoveActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: ''
goal:
  width: 0.00
  speed: 0.05" 
```

```console
rostopic pub /franka_gripper/move/goal franka_gripper/MoveActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: ''
goal:
  width: 0.02
  speed: 0.05" 
```


```console
rostopic pub -1 /franka_gripper/move/goal franka_gripper/MoveActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: ''
goal:
  width: 0.08
  speed: 0.05" 
```


```console
rostopic pub /franka_gripper/homing/goal franka_gripper/HomingActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: ''
goal: {}" 
```

```console
rostopic pub -1 /franka_gripper/grasp/goal franka_gripper/GraspActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: ''
goal:     
  width: 0.05
  epsilon:
    inner: 0.02
    outer: 0.02
  speed: 0.02
  force: 5.0" 
```

roslaunch franka_gripper franka_gripper.launch robot_ip:=192.168.4.20


### Moveit in pilz kontroler
Terminal 1: zaženi moveit konfiguracijo.

```console
cd /home/bambus/franka_ros/catkin_ws_sola
source devel/setup.bash
roslaunch panda_moveit_config panda_control_moveit_rviz.launch robot_ip:=192.168.4.20 launch_rviz:=true
```

Kopiraj iz mape `/home/bambus/franka_ros/catkin_ws_ros_test01/src/franka_ros/franka_example_controllers/scripts` skripto `panda_example_pilz.py`. Skopiraj jo v svojo mapo `src/franka_ros/franka_example_controllers/`.

Terminal 2:

```console
cd /home/bambus/franka_ros/catkin_ws_sola
source devel/setup.bash
```

Terminal 2: kalibracija prijemala

```console
rostopic pub /franka_gripper/homing/goal franka_gripper/HomingActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: ''
goal: {}" 
```

Terminal 2: zaženi python skripto

```console
cd src/franka_ros/franka_example_controllers/
python panda_example_pilz.py
```

### Vision

Terminal 1: zaženi kamero

```console
roslaunch realsense2_camera rs_camera.launch align_depth:=true
```

Terminal 2: zaženi zaznavo objektov

```console
roslaunch find_object_2d find_object_3d.launch rgb_topic:=/camera/color/image_raw    depth_topic:=/camera/aligned_depth_to_color/image_raw    camera_info_topic:=/camera/color/camera_info
```

Terminal 3: pretvorbe v transformacije

```console
rosrun find_object_2d tf_example
```

Terminal 4: zaženi rviz

```console
rviz
```


## Uporabne povezav

Uradna dokumentacija za Franka Emika Panda robota
https://frankaemika.github.io/docs/index.html


### Alternativa za franka_ros

Nepreizkušena alternativa, ima pa nekaj lepih primerov za kodo, naprimer za prijemalo.

https://projects.saifsidhik.page/franka_ros_interface/index.html
https://projects.saifsidhik.page/franka_ros_interface/_modules/index.html

### Prijemalo

https://projects.saifsidhik.page/franka_ros_interface/_modules/franka_interface/gripper.html


### Realsense

https://github.com/IntelRealSense/realsense-ros


```console
roslaunch realsense2_camera rs_d435_camera_with_model.launch
```

```console
realsense-viewer
```

### Movet calibration

git clone https://github.com/PickNikRobotics/rviz_visual_tools.git
git clone https://github.com/ros-planning/geometric_shapes.git
git clone https://github.com/ros-planning/moveit_visual_tools.git -b melodic-devel
git clone https://github.com/ros-planning/moveit_calibration.git

rosdep install --from-paths src --ignore-src --rosdistro melodic

catkin_make

sudo apt-get install ros-melodic-rviz-visual-tools
sudo apt-get install ros-melodic-moveit-visual-tools

python2.7 -m pip install opencv-python==3.4.0.12

https://linuxize.com/post/how-to-install-opencv-on-ubuntu-18-04/






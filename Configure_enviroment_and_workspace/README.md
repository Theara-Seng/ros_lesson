# Create a ROS Workspace

## create a folder name -> it can be anyname but using catkin_ws is recommended for beginner

* create a folder name catkin_ws with src directory element

```sh
mkdir -p ~/catkin_ws/src
```

* Go to catkin_ws directory

```sh
cd ~/catkin_ws/
```

* Run the catkin_make

```sh
catkin_make
```

* source your new setup file 

```sh
source devel/setup.bash
```


# Configure your ROS Environment

* To make it easy by not running the source file all the time, open a new terminal and run

```sh
gedit ~/.bashrc
```

* Then past these two lines into the last of the page

```sh
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
```




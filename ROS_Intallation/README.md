# Install ROS Noetic in Ubuntu 20.04

## Open New Terminal 

* **ctrl+alt+t** to open a new terminal 

## Setup sources list

* Setup your computer to accept software from packages.ros.org. 
```sh
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```

## Setup your Keys

* Install curl 

```sh
sudo apt install curl
```

* Curl the key

```sh
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
```

* Update the Debian package index

```sh
sudo apt update
```

## Installation

* Install desktop-full ROS Package

```sh
sudo apt install ros-noetic-desktop-full
```

## Environment Setup

* You must source this script in every bash
  
```sh
source /opt/ros/noetic/setup.bash
```

## Dependencies for bilding the packages

* To install this tool and other dependencies for bilding ROS package, run:

```sh
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
```

* Initialize the rosdep

```sh
sudo apt install python3-rosdep
```

* With the following, you can initialize rosdep:

```sh
sudo rosdep init
```

* and 

```sh
rosdep update
```

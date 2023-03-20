# Create Custom Message in ROS

### Custom Message

In this section, we are goin to learn how to create msg and srv files for usin in our node. They are files where we put a specification about the type of data to be 
transmitted and the values of this data. ROS will use these files to create the necessary code for us to implement the msg and srv files to be used in our node.

Now, we are going to learn how to create a custom messages with the tools that ROS has

First, create a new msg folder in our package.
```sh
mkdir msg
```
Then go into this folder and create a new message file called
```sh
rc.msg
```

Then gedit this file and put
```sh
int32 R
int32 C
```
Now edit package.xml and put
```sh
  <build_depend>message_generation</build_depend>
  <exec_depend>message_runtime</exec_depend>
```

Then edit CMakeList.txt and add the message_generation line as follow:
```sh
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)
```

Find the next lines, uncomment, and the name of the new message as follow
```sh
 add_message_files(
   FILES
   rc.msg
#   Message2.msg
 )
 ```
 and also uncomment 
 ```sh
 generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
 )
```

Then save all of this and go into catkin_ws and run
```sh
catkin_make
```

To check whether all is OK, you can use the rosmsg command:
```sh
rosmsg show ros_package/rc
```

If you see the same content as that of the rc.msg file, all is ok

Then we can create a topic publisher using our custom message

Go into script and create new file
```sh
touch rc.py
```
Then 
```sh
chmod +x rc.py
```

Then put write a code to publish r and c value
```sh
#!/usr/bin/env python3
import rospy
from ros_test.msg import rc 

rospy.init_node("rc_circuit")
rc_pub = rospy.Publisher("rc",rc, queue_size=10)

rate = rospy.Rate(1)
r1 = 10
r2 = 20

c1 = 20
c2 = 20
while not rospy.is_shutdown():
    value = rc()
    value.R = r1+r2
    value.C = (c1*c2)/(c1+c2)
    rc_pub.publish(value)
    rate.sleep()
    
```

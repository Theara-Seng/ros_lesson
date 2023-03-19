# ROS Architecture

## ROS Computation Graph Level
  
  ROS create a network where all the processes are connected. Any node in the system can access this network, interact with other nodes, see the information that they are sending, and transmit data to the network.
  
  
  ![com](https://github.com/Theara-Seng/ros_lesson/edit/main/image/computation_graph.png "com")

The basic concepts in this level are noeds, the master, Parameter Server, messages, services, topic, and bags, all of which provide data to the graph in different ways

* **Nodes**: Nodes are processes where computation is done. If you want to have a process that can interact with other nodes, you neeed to creat a node with this process to connect it to the ROS network. Usually a system will have many nodes to control different functions.

* **The master**: The master provides the registration of names and the lookup service to the rest of the nodes. It is also sets up connections between the nodes. If you don't have it in your system, you can't communicate with nodes, services, messages, and other. 

* **Topics**: Each message must have a name to be routed by the ROS network. When a node is sending data, we say that the node is publishing a topic. Nodes can receive topics from other nodes simply by subscribing to the topic. A node can subscribe to a topic.

* **Messages** Nodes communicate with each other through messages. A message contains data that send information to other nodes. ROS has many types of messages, and you can also develop your own type of message using standard messages.

* **Services**: When you publish topics, you are sending data in a many-to-many fashion, but when you need a request or an answer from a nod, you can't do it with topics. Services give us the possibility of interacting with nodes. Also services must have a unique name. When a node has a service, all the nodes can communicate with it, thanks to ROS client Library.

* **Parameter Server** gives us the possibility of using keys to store data in a central location. With this parameter, it is possible to configure nodes while it's running or to change the working of the nodes.

* **Bags** are a format to save and play back the ROS message data. Bags are an important mechanism to store data, such as sensor data, that can be difficult to collect but is necessary to develop and test algorithm. You will use bags a log while working with complex robot.

## NODES

* Nodes are executables that can commincate with other porcesses using topics, sevives, or the Parameter Server. Using nodes in ROS provides us with fault tolerance and separates the code and functionalities, making the system simpler.

* A node must have a unique name in the system. THis name is used to permit the node to communicate with another node using its name.  A node can be written using different libraries, such as **roscpp** and **rospy**. **roscpp** is for C++ and **rospy** is for Python.

* ROS has tools to handle nodes and give us information about it, such as **rosnode**, which is a command-line tool used to display information about nodes, such as listing the currently running nodes. 

* The supported commands are as follows:
  * **rosnode info** Node: This prints information about a node.
  * **rosnode kill** Node: This kills a runing node or sends a given signal 
  * **rosnode list** Node: This lists the active nodes
  
 ### Example
 
 * Open a new terminal and run the master 
 
 ```sh
 roscore
 ```
 
 * Then run the turtlesim node
 
 ```sh
 rosrun turtlesim turtlesim_node
 ```
 
 * Then the turtlesim GUI should show up with a turble bot in it. and we can display the node by calling 
 
 ```sh
 rosnode list
 ```
 This will display the node list of the turtlesim
 
 * Then running the info
 
 ```sh
 rosnode info
 ```
 This will list all the publisher and subscriber that the turtle sim has
 
 * If we want to kill the node of the turtlesim we can do
 
 ```sh
 rosnode kill [node_name]
 ```
 This will kill the node name.

### Create your own rospy ndoe

* In here we will create a rospy node to interact with another node. But first we need to create a package

-Go to ~/catkin_ws/src by:
```sh
cd catkin_ws/src
```

* The we create a new package call ros_test by:

```sh
catkin_create_pkg ros_test rospy
```
* Then 
```sh
cd ..
```
* and run:
```sh
catkin_make
```

* Afterthat we can go to the ros_test package
```sh
roscd ros_test
```
*  and create new folder name script:

```sh
mkdir script
```

* Go to the folder script and create a python file call node_test.py by:
```sh
touch node_test.py
```

* Then we need to make it executable by chmod 

```sh
chmod +x node_test.py
```

* Then edit the file noed_test.py by running:

```sh
gedit node_test.py
```

* and pass this code 

```sh
#!/usr/bin/env python3
import rospy

rospy.init_node('node1')

rate = rospy.Rate(2)

while not rospy.is_shutdown():

	rate.sleep()
```

## Topics 

Topics are buses used by nodes to transmit data. Topics can be transmitted without a direct connection between nodes, which mean that the production and consumption of data are decoupled. A topic can have various subscribers and can also have various publishers, but you can take care about publisheing the same topic with differrent nodes because it can create conflicts.

ROS has a tool to work with topics called **rostopic**. It is a command-line tool that gives us information about the topic or publises data directly on the network.

This tool has the following parameter:

* **rostopic bw /topic**: This display the bandwidth used by the topic.
* **rostopic echo /topic**: This print messages to the screen
* **rostopic find message_type**: This find topics by their type
* **rostopic info /topic**: This print information about the active topic, topics published, and services
* **rostopic list**: This prints information baout active topics
* **rostopic pub** /topic type args**: This publises data to the topic. It allows us to create and publish data in whatever topic we want, directly from the command-line

### Example 

* Run the turtlesim node

```sh
rosrun turtlesim turtlesim_node
```

* Then open a new terminal and run:

```sh
rostopic list
```
This will list all topic about the turtlesim node

* Then run:

```sh
rostopic echo /turtle1/pose
```
This will display the turtlesim position as well as the velocity

### Message 

A node sends information to another node using messages that are published by topics. The message has a simple structure that uess standard tupes or types developed by the user.

ROS has the **rosmsg** command-line tool to get information about messages. THe accepted parameters are follow

* **rosmsg show**: This diplays the fiels of a message

* **rosmsg list**: This list all messages

* **rosmsg package**: This lists all the messages in a package

* **rosmsg packages**: This lists all of the packages that have the message



### Create your own topic 

#### Topic Publisher
First go the folder script of your package through terminal and create a python file by using:

```sh
touch topic_publisher.py
```
Then runnng 
```sh 
chmod +x topic_publisher
```
Since we are going to be running the noeds we write as a programs, we also have to set execute permissions on them using the linux chmod command:


Below shows the basic code for advertising a topic and publishing messages on it. This node publishes consecutive integers on the topic counter at a rate of 2 Hz

```sh
#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int32

rospy.init_node('node1')
pub = rospy.Publisher("counter", Int32, queue_size=10)
rate = rospy.Rate(2)

count = 0
while not rospy.is_shutdown():
	pub.publish(count)
	count += 1
	rate.sleep()
```

The first line 
```sh
#!/usr/bin/env python3
```
This line lets the operating system know that this is a pyton file, and that it shuld be passed to the python interpreter. 

The second line

```sh
import rospy
```
appears in every ROS Python node and imports all of the basic functionlity that we'll need. 

The next line imports the definition of the message that we are going to send over the topic
```sh
from std_msgs.msg import Int32
```
In this case, we are going to use a 32-bit interger, defined in ROS standard message package, std_msgs.

Then we initialize the node by
```sh
rospy.init_node("topic_publisher")
```

After initialize the node, we advertise it with a publisher
```sh
pub = rospy.Publisher("counter", Int32, queue_size=10)
```

This give a topic name (counter) and specifies the type of message that will be sent (Int32). Behind the scences, the publisher also sets up a connection to roscore and sends some information to it. When another node tries to subscribe to the counter topic, roscore will share its list of publishers and subscribers, which the nodes will then use to create direct connections between all publishers and of all subscribers to each topic.

At this point, the topic is advertised and is available for other nodes to subscribe to. Now we can go about actually publishing messages over the topic
```sh
rate = rospy.Rate(2)

count = 0
while not rospy.is_shutdown():
	pub.publish(count)
	count += 1
	rate.sleep()
```

First, we set the rate, in hertz, at which we want to publish. For this example, we are going to publish twice a second.

The is_shutdown() function will return True if the node is ready to be shut down and False otherwise, so we can use this to determine if it is time to exit the while loop.

Inside the while loop, we publish the current value of the counter, increment its value by 1, and then sleep for a while. The call to rate.sleep() will sleep for long enough to make sure that we run the body of the while loop at approximately 2Hz.

#### Topic Subscriber

Now we create a new file call topic_subscriber 

```sh
touch topic_subscriber.py
```

Then make it executable

```sh
chmod +x topic_subscriber.py
```

Then the code below show a node that subscribes to the counter topic and prints out the value in the messages as they arrive.

```sh
#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int32

def callback(msg):
    print(msg.data)

rospy.init_node('topic_subscriber')
sub = rospy.Subscriber('counter', Int32, callback)
rospy.spin()

```

The first interesting part of this code is the callback that handles the messages as they come in:

```sh
def callback(msg)
    print(msg.data)
```

ROS is an event-driven system, and it uses callback function heavily. Once a node has subscribed to a topic, every time a message arrives on it the associated callback function is called, with the message as its parameter. In this case, the function simply prints out the data contained in the message.

After initilizing the node, as before, we subscribe to the counter topic

```sh
sub = rospy.Subscriber("counter", Int32, callback)
```

We give the nage of the topic, the message type of the topic, and the name of the callback function. Behind the scenes, the subscriber passes this information on the roscore and tries to make a direct connection with the publishers of this topic. If the topic does not exist, or if the type is wrong, there are no error messages: the node will simply wait until messages start being published on the topic.


Once the subscription is made, we give control over to ROS by calling rospy.spin(). This function will only return when the node is ready to shut down. This is just a useful shortcut to avoid having to define a top-level while loop.

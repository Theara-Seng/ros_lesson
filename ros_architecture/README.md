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

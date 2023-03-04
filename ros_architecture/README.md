# ROS Architecture

## ROS Computation Graph Level
  
  ROS create a network where all the processes are connected. Any node in the system can access this network, interact with other nodes, see the information that they are sending, and transmit data to the network.
  
  
  ![com](https://github.com/Theara-Seng/ros_lesson/edit/main/image/computation_graph.png "com")

The basic concepts in this level are noeds, the master, Parameter Server, messages, services, topic, and bags, all of which provide data to the graph in different ways

* **Nodes**: Nodes are processes where computation is done. If you want to have a process that can interact with other nodes, you neeed to creat a node with this process to connect it to the ROS network. Usually a system will have many nodes to control different functions.

* **The master**: The master provides the registration of names and the lookup service to the rest of the nodes. It is also sets up connections between the nodes. If you don't have it in your system, you can't communicate with nodes, services, messages, and other. 

* **Topics**: Each message must have a name to be routed by the ROS network. When a node is sending data, we say that the node is publishing a topic. Nodes can receive topics from other nodes simply by subscribing to the topic. A node can subscribe to a topic.
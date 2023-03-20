# Create your own topic 

## Topic Publisher
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

## Topic Subscriber

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

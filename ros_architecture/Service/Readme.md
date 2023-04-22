# Service 

## Learning how to use service

Service are another way through which nodes can communicate with each other.  Services allow nodes to send a request and recive a response.

The tool that we are going to use to interact with service is called rosservice. The accepted parameters for this command are as follows:


* **rosservice list**: This lists the active services
* **rosservice args /service**: This prints the service arguments
* **rosservice call /service**: This calls services with the arguments provided
* **rosservice info /service**: This print information about the service

For example, run the turtlesim node using the following command

```sh
rosrun turtlesim turtlesim_node
```

Then list all the service by 

```sh
rosservice list
```
If you want to kill the turtlesim from the window, first check the args with 

```sh
rosservice args /kill
```
Then if you want to kill the turtlesim then run:

```sh
rosservice call /kill "name: 'turtle1'"
```

If you want to see the type of any service, for the /reset service, use the following command

```sh
rosservice type /reset
```

You will see something similar to the following output:

* **std_srvs/Empty**

To invoke a service, you will use 
```sh
rosservice call /reset
```

## Create your own service

Create a new folder in your package:
```sh
mkdir srv
```
Then in create a new service in the folder srv

```
touch sum2int.srv
```

Then chmod in the terminal:

```
chmod +x sum2int.srv
```
and create a service message

```
int64 first
int64 second
int64 sum
```

And we also need to configue the CMakeLists.txt in the line Generate services with:

```sh
 add_service_files(
   FILES
   sum2int.srv
 )
 ```
 
 ### Create a service node
 
 we create a node name service_node.py in the folder script and add the following code:
 
 ```sh
#!/usr/bin/env python3

import rospy
from lab1.srv import *

def service_callback(req):
    ans = req.first + req.second

    rospy.loginfo("Received [%s, %s], returning %s"%
                                (req.first, req.second, ans))

    resp = sum2intResponse()
    resp.sum = ans
    return resp

if __name__ == "__main__":

    rospy.init_node('sum2int')
    rospy.Service('sum2int', sum2int, service_callback)

    rospy.loginfo("Ready to add two ints.")
    rospy.spin()
 ```
 After this we need to do the catkin_make in the catkin_ws and also source file

Then we can run the rosmaster:

```sh
roscore
```
and run the service node:

```sh
rosrun lab1 service_node.py
```

Then run :

```sh
 rosservice call /sum2int "first: 2
second: 2"
```

Then you can see the response from the service is 4.

### Create a client node

we create a client node with the following:


```sh
#!/usr/bin/env python3

import rospy, random
from lab1.srv import *

if __name__ == "__main__":
    rospy.init_node('client_node')

    calc_client = rospy.ServiceProxy('sum2int', sum2int)

    r = rospy.Rate(1)

    while not rospy.is_shutdown():
        a = random.randint(1, 10)
        b = random.randint(1, 10)

        rospy.loginfo("Generated [%d, %d], sending addition request..." % (a, b))

        req = sum2intRequest()
        req.first = a
        req.second = b

        resp = calc_client(req)
        rospy.loginfo("Received response: %d" % resp.sum)

        r.sleep()
```

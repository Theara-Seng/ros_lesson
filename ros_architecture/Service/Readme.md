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


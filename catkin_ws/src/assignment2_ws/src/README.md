Code 1: Circle.py
This code moves the turtle in a circular trajectory.
In this code we simply give the linear and angular velocities to the turtle.
This velocity is published at a rate of 10 per second.
This means that both the velocities are being published at the same timw due to which we get a perfect circle.
The following is the trajectory achieved.
![](https://github.com/uchaudh/AuE893Spring20_ChaudhariUtkarsha/blob/master/catkin_ws/src/assignment2_ws/src/images/circle.png)

Code 2: square_openloop.py
This code moves the turtle in a square trajectory with a given linear and angular velocity.
In this code we provide the linear velocity first at the rate of 5 per second.
In this way, we can achieve how far the turtle goes by controlling the time that the linear velocity is being publised for.
After the publishing of the linear velocity is done, the angular velocity is published for the given time to achieve a 90 degree turn.
![](https://github.com/uchaudh/AuE893Spring20_ChaudhariUtkarsha/blob/master/catkin_ws/src/assignment2_ws/src/images/Sqaure_openloop.png)

Code 3: square_closedloop.py
This code moves the turtle in a square trajectory with given goal points.
In this code we provide the turtle with the x and y coordinates of the four corners of a square.
The euclidian distance and the angular difference are used as the controlling parameters to control the rotation of the turtle.
We get the following trajectory, because we start with (0,0)
![](https://github.com/uchaudh/AuE893Spring20_ChaudhariUtkarsha/blob/master/catkin_ws/src/assignment2_ws/src/images/square_closedloop.png)

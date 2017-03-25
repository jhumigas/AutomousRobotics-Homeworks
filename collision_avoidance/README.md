# Collision Avoidance with ROS

In this assignment, our goal is to control the velocity of a small wheeled robots in a simulated environment.

Basically, there are three tasks to consider : 

* Consider points in the direction of motion, less than a maximum distance away and not too far on the side.
* Set a minimum distance below which maximum velocity is set to 0.0
* Between that and the maximum distance implement a smooth linear scaling.

## Topics, subscribers and publishers

Here are the topics we subscribe to : 

* `/vrep/hokuyoSensor` : To collect points in the surroundings and detect potential source of collisions
* `cmd_vel`: To get the current velocity of the robot
* `/joy`: To listen to the joystic 


Here are the topics we publish to : 
* `output_vel`: To broadcast the actual velocity command
* `/vrep/rightWheelCommand` and `/vrep/leftWheelCommand`: To send the velocity command to the robot

## Processing messages

We have the following pipeline from the JoyStick to the wheels of the robots: 

joy -> twist -> Float64

It might be better to just convert joy to Float64.

## Controlling velocity

The minimum security distance is set thanks to the radius parameter.
Basically we chose the closest point to the robot in the acquired PointCloud. This point is in the direction of motion, 
and not too far aside. This will prevent the robot to get stuck if it is nearby an object but not moving towards it.

We add another layer of security: before freezing the robot in case it is running toward an object, we decrease the velocity. 

> One might decrease the velocity in a linear manner.

Here, the velocity will decrease with a factor of K*1/(robot.distance - minimal_distance).
We chose K s.t the product is greater than one. It would be a shame is the robot accelerated instead of stopping.

We found R = 0.3 is the optimal distance value.

# Rover driver 

This is part of my work to homework 4. The main goal is to control a 6 wheel rover using a joystick, therefore applying basics of mobile robot kinematics.

## Speed and Steering distribution 

We compute each speed and steering wheel using a standard velocity tensor i.e `Vw = Vb + Ωb ∧ bw`. 
Since each wheel sensor is accessible thanks to `driver_cfg`, we can deduce the steering and speed by applying classic formulas.

Our first approach was reducing the number of wheels, by grouping them by pair, resulting in an artificial rover of three wheels. However, this only allows the robots to move forward. 
Here is a quick representation of the reduction, with `x` the origin of the robot frame and `o` a wheel.
```
   -----      -
   o   o      o
   | x |      x
   o   o  =>  o
   |   |      |
   o---o      o
```
The rover cannot turn 	effectively on itself with that configuration. As matter of fact, applying a full right command (or left) would result for the wheel to have 90 degrees position, which is valid for a three  wheeled vehicles whose wheels are aligned with the frame origin.

Illustrations

Wheels grouped by pair
![Grouped wheels](https://gitlab.centralesupelec.fr/mugisha_dav/rover_driver_base/raw/bc6974fcfe8def8c173de7cc2841bb1a5523276a/snapshots/rover_turn_conf_3_wheels.png)
Wheels not grouped
![Decoupled wheels](https://gitlab.centralesupelec.fr/mugisha_dav/rover_driver_base/raw/bc6974fcfe8def8c173de7cc2841bb1a5523276a/snapshots/rover_driver_6_wheel_config.png)



The second approach is more thorough and consider each wheel without any reduction step.

## 2D Odometry

We used a least square method to compute the position and steering of the robot in the global frame.
We had 12 equations each one resulting from relations between wheels data and position in the global frame. 

For each wheel we have the following : 
`Vw = V - Ω*W` 
`Sw = V*dt - Ω∧W*dt`
resulting in 
`Sw*cos(beta_w) = delta_x - delta_theta*Wy` 
and 
`Sw*sin(beta_w) = delta_y + delta_theta*Wx`
after projection
where 
* Sw, delta_x stands for the position of each wheel respectively in the robot  and global frame, 
* Ω rotation speed in the global frame and position of the wheel in the global frame
* `beta_w` bearing of the wheel
* `Wy` and `Wx` are actually the radius of each wheel


For the odometry, we adopted a Least Mean Square method to estimate x, y and theta.
We visualized odometry using _rviz_ to see how the vehicle is tracked based on motion sensors. 
There is some glitch in the overall behavior, might be linked to the periodicity of the estimated angles.

## Task Manager

TODO
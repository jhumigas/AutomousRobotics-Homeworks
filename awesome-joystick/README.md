# Joy to Turtle 

Here are a few notes on how to control the little turtle bot with a joystick.

## Driver 

First thing is to make sure you have the write driver to communicate with the joystick. 

Check the version of ros you have by typing 

```
rosversion -d
``` 

Then install the appropriate driver, for ROS Kinetic, I used

```
sudo apt-get install ros-kinetic-joy
```

To check if it is working correctly, we can read the output on the joy topic, with the following : 
```
# Print bunch of stuff related to the inputs
# Look for 'jsX' with X a number
ls /dev/input/
# Test if it is working correctly with the following 
# and click, move joysticks to see value change
sudo jstest /dev/input/jsX
```

You should also check if you have the correct right, so that ROS can compile related packages. 
For that matter, and to see how to start a joy node, refer to [Configuring and Using a Linux-Supported Joystick with ROS](http://wiki.ros.org/joy/Tutorials/ConfiguringALinuxJoystick).

## Creating a package

In the following are a few steps to get a ROS package to command the little turtle bot with a linux supported joystick.
A good tutorial to write a small node in C++ is also available [here](http://wiki.ros.org/joy/Tutorials/WritingTeleopNode)

First things first create a package with : 
```
catkin_create_pkg awesome-joystick rospy joy turtlesim
``` 

Head to src and create the node, called here turtle_teleop_joy.py. Courtesy to [Andrew's tutorial](http://andrewdai.co/xbox-controller-ros.html#rosjoy).

The code is : 
```
#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

def callback(data):
    """
    Broadcast joystick data to the turtle
    Since the turtle is expecting twist, we convert those data to twists
    Once the twist is ready, it publish to the turtle's topic

    Args: 
        data(dict): Values sent by the joy stick
    """
    twist = Twist()
    twist.linear.x = 5*data.axes[1]
    twist.angular.z = 4*data.axes[0]
    pub.publish(twist)

def start():
    """
    Initialize publisher and subscriber
    """
    global pub
    pub = rospy.Publisher('turtle1/cmd_vel', Twist)
    rospy.Subscriber('joy', Joy, callback)
    rospy.init_node('Joy2Turtle')
    rospy.spin()

if __name__ == '__main__':
    start()

```

Make sure ROS can compile this file by doing this : 
```
chmod +x turtel_teleop_joy.py
```

Almost ready. 
We actually need to run three nodes to make everything work:
* `turtlesim turtlesim_node` : Turtle bot
* `joy joy_node` : Joystick node, make sure you have the correct value for jsX
* `awesome_joy turtle_teleop_joy.py`: Our node

Let's create a .launch file handling that teadious task : 

At the root of our package folder, create launch folder, and inside the file turtle_joy.launch with the following: 

```
<launch>
 <!-- Turtlesim Node-->
  <node pkg="turtlesim" type="turtlesim_node" name="sim"/>

 <!-- Joy Node -->
  <node respawn="true" pkg="joy"
        type="joy_node" name="turtle_joy" >
    <param name="dev" type="string" value="/dev/input/jsX" />
    <param name="deadzone" value="0.12" />
  </node>

 <!-- Axes -->
  <param name="axis_linear" value="1" type="int"/>
  <param name="axis_angular" value="0" type="int"/>
  <param name="scale_linear" value="2" type="double"/>
  <param name="scale_angular" value="2" type="double"/>
  <node pkg="awesome_joy" type="turtle_teleop_joy.py" name="teleop"/>
</launch>
```

All set : time to compile and test : 
```
catkin build awesome-joystick
roscore
roslaunch awesome_joy turtle_joy.launch
```

# Controlling a simulated robot 

There is a small implementation to control a robot with a joystick. The scene can be found [here](http://sirien.metz.supelec.fr/depot/SIR/RobotiqueAutonome/scene.ttt).

The main code is in src/driver.py

Just run : `roslaunch awesome_joy toto.launch`

The overall implementation is the same as controlling the little turtle. The only conversion required is from the joy command to the wheel velocities.

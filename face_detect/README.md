# Face Detect

Demonstration for face and object detection using haar-like features. 
FYI the base code is from the assignment, the task is to broadcast the detected faces in terms of region of interests. 
We use [ROS RegionOfInterest](http://docs.ros.org/jade/api/sensor_msgs/html/msg/RegionOfInterest.html) to send the detected face to a topic.
In the case of multiple detection, we built a custom message : an array of Region Of Interests.

## Creating custom message

Useful task to create a custom message are : 
* Edit `package.xml` :
```
<build_depend>message_generation</build_depend>
<run_depend>message_runtime</run_depend>
```
* Add those packages to the `CMakeLists.txt`
* Add message files, previously created in msg folder
* Uncomment `generate_messages` and edit according to your message dependencies

Once the package is build, you can check if the message are indeed accessible with `rosmsg show CustomMsg`
You can import the custom message in python like this: from package_name.msg import CustomMsg

To get an example on how to use custom message in ROS, refer to the [beginner tutorials](http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv).

## Listening to the topic 

After running `roscore`, vrep and the package. Type in a new terminal:

```
rostopic echo myROI #myROI2 for arrays
```
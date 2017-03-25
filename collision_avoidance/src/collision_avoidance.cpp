
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float64.h>

class CollisionAvoidance {
    protected:
        // Sensor subscriber to read the PointClouds
        ros::Subscriber scanSub;
        // Velocity subscriber to control to process the velocity during the
        // the collision avoidance phase
        ros::Subscriber velSub;
        // Joy subscriber to retrieve Joy commands and convert them to Twist
        ros::Subscriber joySub;
        // Velocity publisher
        ros::Publisher velPub;
        // Left and right wheels publishers
        ros::Publisher wheelPubL;
        ros::Publisher wheelPubR;


        ros::NodeHandle nh;

        // Minimum avoidance collision distance
        double radius;

        // Right and left wheel velocities
        std_msgs::Float64 wl;
        std_msgs::Float64 wr;

        pcl::PointCloud<pcl::PointXYZ> lastpc;
        
        /*
         * Processing initial velocity command to avoid collision
         * then publishing the processed command to left and right wheels
         *
         * @param msg Initial velocity command
         */
        void velocity_filter(const geometry_msgs::Twist& msg) {
            // Processing Velocity w.r.t the point cloud received
            geometry_msgs::Twist filtered = findClosestAcceptableVelocity(msg);
            velPub.publish(filtered);
            wl.data = (filtered.linear.y - filtered.angular.z*0.33/2)/0.1;
            wr.data = (filtered.linear.y + filtered.angular.z*0.33/2)/0.1;
            wheelPubL.publish(wl);
            wheelPubR.publish(wr);

        }
        
        /*
         * Converting data from sensors to messages
         *
         * @param msg PointCloud to convert
         */
        void pc_callback(const sensor_msgs::PointCloud2ConstPtr msg) {
            pcl::fromROSMsg(*msg, lastpc);
             // unsigned int n = lastpc.size();
             // ROS_INFO("New point cloud: %d points",n);
             // for (unsigned int i=0;i<n;i++) {
             //     float x = lastpc[i].x;
             //     float y = lastpc[i].y;
             //     float z = lastpc[i].z;
             //     ROS_INFO("%d %.3f %.3f %.3f",i,x,y,z);
             // }
             // printf("\n\n\n");
        }
        
        /*
         * Converting joystick commands to Twist
         *
         * @param Joy command broadcast by the Joystick
         */
        void joy_callback(const sensor_msgs::Joy::ConstPtr& joy) {
		  geometry_msgs::Twist twist;
		  twist.angular.z = joy->axes[0];
		  twist.linear.y = 0.5*joy->axes[1];
		  velocity_filter(twist);
		}
        
        /*
         * Process velocity  to avoid collision
         *
         * @param desired Initial velocity to filter
         */
        geometry_msgs::Twist findClosestAcceptableVelocity(const geometry_msgs::Twist& desired) {
            geometry_msgs::Twist res = desired;
             unsigned int n = lastpc.size();
             // ROS_INFO("New point cloud: %d points",n);
             float closest = 10000;
             // Selecting the closest point to the robot
             for (unsigned int i=0;i<n;i++) {
                 // Selecting points in the direction of motion and not too far aside
                 if(lastpc[i].x * res.linear.y > 0 && lastpc[i].y > -0.2 && lastpc[i].y < 0.2){
                     // Pythagora theorem to compute the distance from the robot to a 
                     // to a given point
                     float current = lastpc[i].x*lastpc[i].x + lastpc[i].y*lastpc[i].y;
                     if(current < closest && current > 0){
                         closest = current;
                         // ROS_INFO("%.3f %.3f",lastpc[i].x,lastpc[i].y);
                     }
                 }

                 // ROS_INFO("%.12f",closest);
             }
             if(closest < radius*radius){
                 res.linear.y = 0;
                 // ROS_INFO("too close",n);
             }else if(closest < 3*radius*radius){
                 res.linear.y /= 1000*(closest - radius);
             }

            return res;
        }

    public:
        CollisionAvoidance() : nh("~"), radius(1.0) {
            scanSub = nh.subscribe("/vrep/hokuyoSensor",1,&CollisionAvoidance::pc_callback,this);
            velSub = nh.subscribe("cmd_vel",1,&CollisionAvoidance::velocity_filter,this);
            joySub = nh.subscribe("/joy",1, &CollisionAvoidance::joy_callback, this);
            velPub = nh.advertise<geometry_msgs::Twist>("output_vel",1);
            wheelPubL = nh.advertise<std_msgs::Float64>("/vrep/leftWheelCommand", 1);
            wheelPubR = nh.advertise<std_msgs::Float64>("/vrep/rightWheelCommand", 1);
            nh.param("radius",radius,1.0);
        }

};

int main(int argc, char * argv[]) 
{
    ros::init(argc,argv,"collision_avoidance");

    CollisionAvoidance ca;

    ros::spin();
}



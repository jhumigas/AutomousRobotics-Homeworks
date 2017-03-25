#include <math.h>
#include "TaskGoToPose.h"
#include "floor_nav/TaskGoToPoseConfig.h"
using namespace task_manager_msgs;
using namespace task_manager_lib;
using namespace floor_nav;

// #define DEBUG_GOTOPOSE
#ifdef DEBUG_GOTOPOSE
#warning Debugging task GOTOPOSE
#endif


TaskIndicator TaskGoToPose::initialise() 
{
    ROS_INFO("Going to %.2f %.2f",cfg.goal_x,cfg.goal_y, cfg.target*180./M_PI);
    return TaskStatus::TASK_INITIALISED;
}


TaskIndicator TaskGoToPose::iterate()
{
    const geometry_msgs::Pose2D & tpose = env->getPose2D();
    double r = hypot(cfg.goal_y-tpose.y,cfg.goal_x-tpose.x);
    if (r < cfg.dist_threshold) {
        // Turn 
        double alpha = remainder(cfg.target-tpose.theta,2*M_PI);
        if (fabs(alpha) < cfg.angle_threshold) {
            return TaskStatus::TASK_COMPLETED;
        }
        double rot = cfg.k_theta*alpha;
        if (rot > cfg.max_angular_velocity) rot = cfg.max_angular_velocity;
        if (rot <-cfg.max_angular_velocity) rot =-cfg.max_angular_velocity;
        env->publishVelocity(0.0, rot);
        // Stop turn
		return TaskStatus::TASK_RUNNING;
    }else{
        double alpha = remainder(atan2((cfg.goal_y-tpose.y),cfg.goal_x-tpose.x)-tpose.theta,2*M_PI);
#ifdef DEBUG_GOTOPOSE
    printf("c %.1f %.1f %.1f g %.1f %.1f r %.3f alpha %.1f\n",
            tpose.x, tpose.y, tpose.theta*180./M_PI,
            cfg.goal_x,cfg.goal_y,r,alpha*180./M_PI);
#endif
    if (fabs(alpha) > M_PI/9) {
        double rot = ((alpha>0)?+1:-1)*cfg.max_angular_velocity;
#ifdef DEBUG_GOTOPOSE
        printf("Cmd v %.2f r %.2f\n",0.,rot);
#endif
        env->publishVelocity(0,rot);
    } else {
        double vel = cfg.k_v * r;
        double rot = std::max(std::min(cfg.k_alpha*alpha,cfg.max_angular_velocity),-cfg.max_angular_velocity);
        if (vel > cfg.max_velocity) vel = cfg.max_velocity;
#ifdef DEBUG_GOTOPOSE
        printf("Cmd v %.2f r %.2f\n",vel,rot);
#endif
        env->publishVelocity(vel, rot);
    }
    return TaskStatus::TASK_RUNNING;

    }
    
}

TaskIndicator TaskGoToPose::terminate()
{
    env->publishVelocity(0,0);
	return TaskStatus::TASK_TERMINATED;
}

DYNAMIC_TASK(TaskFactoryGoToPose);

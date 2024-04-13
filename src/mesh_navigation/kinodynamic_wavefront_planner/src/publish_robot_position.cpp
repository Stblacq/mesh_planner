#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include "kinodynamic_wavefront_planner/kinodynamic_wavefront_planner.h"


namespace kinodynamic_wavefront_planner
{

KinodynamicWavefrontPlanner::State KinodynamicWavefrontPlanner::getStateFromMsg(const nav_msgs::Odometry::ConstPtr& msg){
    const auto& pose = msg->pose.pose;
    const auto& lin_vel = msg->twist.twist.linear;
    const auto& ang_vel = msg->twist.twist.angular;
    const auto& face_normals = mesh_map->faceNormals();

    mesh_map::Vector position_vector = mesh_map::Vector(pose.position.x,pose.position.y,pose.position.z);
    const lvr2::FaceHandle& face = mesh_map->getContainingFace(position_vector, 0.4).unwrap();


    State state;
    state.x = pose.position.x;
    state.y = pose.position.y;
    state.z = pose.position.z;

    state.qw = pose.orientation.w;
    state.qx = pose.orientation.x;
    state.qy = pose.orientation.y;
    state.qz = pose.orientation.z;

    
    state.vx = lin_vel.x;
    state.vy = lin_vel.y;
    state.vz = lin_vel.z;


    state.wx = ang_vel.x;
    state.wy = ang_vel.y;
    state.wz = ang_vel.z;

    state.faceNormal = face_normals[face];

    return state;
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {

    KinodynamicWavefrontPlanner planner;
    KinodynamicWavefrontPlanner::State state = planner.getStateFromMsg(msg);

    const auto& pos = msg->pose.pose.position;
    const auto& ori = msg->pose.pose.orientation;
    ROS_INFO("Robot Position: [x: %f, y: %f, z: %f]", pos.x, pos.y, pos.z);
    ROS_INFO("Robot Orientation: [x: %f, y: %f, z: %f, w: %f]", ori.x, ori.y, ori.z, ori.w);

    const auto& lin_vel = msg->twist.twist.linear;
    const auto& ang_vel = msg->twist.twist.angular;
    ROS_INFO("Linear Velocity: [x: %f, y: %f, z: %f]", lin_vel.x, lin_vel.y, lin_vel.z);
    ROS_INFO("Angular Velocity: [x: %f, y: %f, z: %f]", ang_vel.x, ang_vel.y, ang_vel.z);


}

} /* namespace kinodynamic_wavefront_planner */

int main(int argc, char **argv) {
    ros::init(argc, argv, "pose_and_velocity_listener");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/gazebo_odom", 1000, kinodynamic_wavefront_planner::odomCallback);

    ros::spin();

    return 0;
}
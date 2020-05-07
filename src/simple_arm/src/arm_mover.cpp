#include "ros/ros.h"
#include "simple_arm/GoToPosition.h"
#include <std_msgs/Float64.h>

ros::Publisher joint1_pub, joint2_pub;

std::vector<float> clamp_at_boundaries(float requested_j1, float requested_j2)
{
    float clamped_j1 = requested_j1;
    float clamped_j2 = requested_j2;
    float min_j1, max_j1, min_j2, max_j2;

    ros::NodeHandle n2;

    std::string node_name = ros::this_node::geetName();

    n2.getParam(node_name + "/min_joint_1_angle", min_j1);
    n2.getParam(node_name + "/max_joint_1_angle", max_j1);
    n2.getParam(node_name + "/min_joint_2_angle", min_j2);
    n2.getParam(node_name + "/max_joint_2_angle", max_j2);

    if(requested_j1 < min_j1 || requested_j1 > max_j1)
    {
        clamped_j1 = std::min(std::max(requested_j1, min_j1), max_j1);
        ROS_WARN("j1 is out of bounds");
    }

    if(requested_j2 < min_j2 || requested_j2 > max_j2)
    {
        clamped_j2 = std::min(std::max(requested_j2, min_j2), max_j2);
        ROS_WARN("j2 is out of bounds");
    }

    std::vector<float> clamped_data = {clamped_j1, clamped_j2};
    
    return clamped_data;
}

bool handle_safe_move_request(simple_arm::GoToPosition::Requests& req,
                                simple_arm::GoToPosition::Response& res)
{
    ROS_INFO("GoToPositionRequest received - j1: %1.2f, j2: %1.2f", (float)req.joint_1, (float)req.joint_2);
    std::vector<float> joint_angles = clamp_at_boundaries(req.joint_1, req.joint2);
    std_msgs::Float64 joint1_angle, joint2_angle;

    joint1_pub.publish(joint1_angle);
    joint2_pub.publish(joint2_angle);

    ros::Duration(3).sleep();

    res.msg_feedback = "Joint angles set - j1: " + std::to_string(joints_angle[0]) + ", j2: " + std::to_string(joints_angles[1]);
    ROS_INFO_STREAM(res.msg_feedback);

    return true;
}

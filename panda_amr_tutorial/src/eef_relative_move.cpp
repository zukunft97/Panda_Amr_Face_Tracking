#include <ros/ros.h>
#include <std_msgs/String.h>
#include <array>

// MoveIt
#include <moveit/move_group_interface/move_group_interface.h>
#include "panda_amr_tutorial/panda_arm_controller.h"

// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Transform.h>

class EEFRelativeMoveNode
{
public:
    EEFRelativeMoveNode(ros::NodeHandle& nh)
        : move_group_("panda_arm"),
          grid_x_(0),
          grid_y_(0)
    {
        /* ================================
         * 시작 위치로 이동
         * ================================ */
        PandaArmController arm("panda_arm");

        start_joints_ = {
            7.322118496522308e-05,
            -0.7853116683289186,
            9.577233898453416e-05,
            -2.356122405050535,
            2.636757367290556e-05,
            3.1454202351145444,
            0.7853191739090496
        };

        arm.moveToJointPositions(start_joints_);
        ros::Duration(0.5).sleep();

        /* ================================
         * MoveIt 설정
         * ================================ */
        move_group_.setEndEffectorLink("panda_link8");
        move_group_.setPlanningTime(5.0);
        move_group_.setMaxVelocityScalingFactor(0.2);
        move_group_.setMaxAccelerationScalingFactor(0.2);

        ROS_INFO("Planning frame: %s",
                 move_group_.getPlanningFrame().c_str());
        ROS_INFO("End effector link: %s",
                 move_group_.getEndEffectorLink().c_str());

        dir_sub_ = nh.subscribe(
            "/face_direction",
            1,
            &EEFRelativeMoveNode::directionCallback,
            this);

        last_msg_time_ = ros::Time::now();

        watchdog_timer_ =
            nh.createTimer(
                ros::Duration(0.5),
                &EEFRelativeMoveNode::watchdogCallback,
                this);
    }

private:
    ros::Subscriber dir_sub_;

    ros::Timer watchdog_timer_;

    moveit::planning_interface::MoveGroupInterface move_group_;

    bool is_moving_ = false;

    bool returned_to_home_ = false;

    ros::Time last_msg_time_;

    PandaArmController::JointArray start_joints_;

    int grid_x_;
    int grid_y_;
    const int GRID_LIMIT = 10;

    /* ================================
     * 방향 콜백
     * ================================ */
    void directionCallback(const std_msgs::String::ConstPtr& msg)
    {
        /******** [ADD] 메시지 수신 시간 갱신 ********/
        last_msg_time_ = ros::Time::now();
        returned_to_home_ = false;

        const std::string& dir = msg->data;
        ROS_INFO("Received direction: %s", dir.c_str());

        const double step = 0.03;

        if (is_moving_)
            return;

        if (dir == "up" && grid_y_ < GRID_LIMIT)
        {
            is_moving_ = true;
            grid_y_++;
            moveRelative(step, -step, 0.0);
        }
        else if (dir == "down" && grid_y_ > -GRID_LIMIT)
        {
            is_moving_ = true;
            grid_y_--;
            moveRelative(-step, step, 0.0);
        }
        else if (dir == "left" && grid_x_ < GRID_LIMIT)
        {
            is_moving_ = true;
            grid_x_++;
            moveRelative(-step, -step, 0.0);
        }
        else if (dir == "right" && grid_x_ > -GRID_LIMIT)
        {
            is_moving_ = true;
            grid_x_--;
            moveRelative(step, step, 0.0);
        }
    }

    void watchdogCallback(const ros::TimerEvent&)
    {
        double timeout = 5.0;

        if ((ros::Time::now() - last_msg_time_).toSec() > timeout)
        {
            if (!returned_to_home_ && !is_moving_)
            {
                ROS_WARN("No topic for 5 seconds → Return Home");

                PandaArmController arm("panda_arm");
                is_moving_ = true;
                arm.moveToJointPositions(start_joints_);

                grid_x_ = 0;
                grid_y_ = 0;

                returned_to_home_ = true;
                is_moving_ = false;
            }
        }
    }

    /* ================================
     * EE 상대 이동
     * ================================ */
    void moveRelative(double dx, double dy, double dz)
    {
        geometry_msgs::Pose target_pose = getTargetPose(dx, dy, dz);

        move_group_.setStartStateToCurrentState();
        move_group_.setPoseTarget(target_pose);

        moveit::planning_interface::MoveGroupInterface::Plan plan;

        bool success =
            (move_group_.plan(plan)
             == moveit::core::MoveItErrorCode::SUCCESS);

        if (success)
        {
            ROS_INFO("Plan successful. Executing...");
            move_group_.execute(plan);

            move_group_.stop();
        }
        else
        {
            ROS_ERROR("Planning failed!");
        }

        is_moving_ = false;
    }

    /* ================================
     * 현재 Pose + 상대 변환
     * ================================ */
    geometry_msgs::Pose getTargetPose(double dx, double dy, double dz)
    {
        geometry_msgs::PoseStamped current_pose_stamped =
            move_group_.getCurrentPose();

        tf2::Transform tf_current;
        tf2::fromMsg(current_pose_stamped.pose, tf_current);

        tf2::Vector3 delta(dx, dy, dz);
        tf2::Vector3 new_origin = tf_current * delta;

        tf_current.setOrigin(new_origin);

        geometry_msgs::Pose target_pose;
        tf2::toMsg(tf_current, target_pose);

        return target_pose;
    }
};

/* ================================
 * main
 * ================================ */
int main(int argc, char** argv)
{
    ros::init(argc, argv, "eef_relative_move_node");
    ros::NodeHandle nh;

    ros::AsyncSpinner spinner(2);
    spinner.start();

    EEFRelativeMoveNode node(nh);

    ros::waitForShutdown();
    return 0;
}
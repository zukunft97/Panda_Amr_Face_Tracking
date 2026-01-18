#include "panda_amr_tutorial/grid_motion_node.h"

GridMotionNode::GridMotionNode(ros::NodeHandle& nh)
    : arm_("panda_arm"), x_(0), y_(0)
{
    // JointArray start_pose = {
    //     7.322118496522308e-05,
    //     -0.7853116683289186,
    //     9.577233898453416e-05,
    //     -2.356122405050535,
    //     2.6367573672905566e-05,
    //     3.1454202351145444,
    //     0.7853191739090496
    // };

    // arm_.moveToJointPositions(start_pose);
    // ros::Duration(1.0).sleep();

    JointArray left_up = {
        0.6804024603071476, -0.2715405562790646, -0.9710998956552547, -1.0048412368150725, 0.37128307071281863, 2.3262433749512397, 0.2936813735595401
    };

    JointArray right_up = {
        -0.645689793810727, -0.26208567942714245, 0.9565993658932572, -1.0079469203488876, -0.40012269976480014, 2.323986197945473, 1.2882801840621678
    };

    JointArray left_down = {
        -2.293707706052068, 1.6514404426384974, 1.9271856306679431, -2.467208708105068, -1.9507644133087476, 2.8032119189144433, 1.3701801987408666
    };

    JointArray right_down = {
        -0.6061795945554684, -1.0511081557775055, 1.1137748776585934, -2.6863247761535574, 0.3627392039472189, 3.4786940779860838, 1.3421199417175207
    };

    JointGridGenerator generator(
    left_up, right_up, left_down, right_down);

    grid_ = generator.generate(10);

    arm_.moveToJointPositions(grid_[x_][y_]);

    sub_ = nh.subscribe("/grid_cmd", 1, &GridMotionNode::commandCallback, this);
}

void GridMotionNode::commandCallback(
    const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("1");
    const std::string& cmd = msg->data;

    if (cmd == "right" || cmd == "rightup" || cmd == "rightdown")
    {
        if (x_ < 9) x_++;
    }

    if (cmd == "left" || cmd == "leftup" || cmd == "leftdown")
    {
        if (x_ > 0) x_--;
    }

    if (cmd == "up" || cmd == "rightup" || cmd == "leftup")
    {
        if (y_ < 9) y_++;
    }

    if (cmd == "down" || cmd == "rightdown" || cmd == "leftdown")
    {
        if (y_ > 0) y_--;
    }

    ROS_INFO("2");
    ROS_INFO("x=%zu, y=%zu", x_, y_);
    move();
}

void GridMotionNode::move()
{
    arm_.moveToJointPositions(grid_[x_][y_]);
}

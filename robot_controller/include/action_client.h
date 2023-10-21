#include "ros/ros.h"
#include <geometry_msgs/Pose2D.h>
#include <robot_msgs/MoveToPoseAction.h>
#include <actionlib/client/simple_action_client.h>

#define PI 3.14159

class ActionClient
{
  private:
    ros::NodeHandle nh_;
    actionlib::SimpleActionClient<robot_msgs::MoveToPoseAction> ac_;
    std::vector<geometry_msgs::Pose2D> targets_;
  
  public:
    ActionClient() : ac_("pose_control", true) {
      geometry_msgs::Pose2D p1;
      p1.x = 2.0;
      p1.y = 2.0;
      p1.theta = PI;
      targets_.push_back(p1);

      geometry_msgs::Pose2D p2;
      p2.x = -2.0;
      p2.y = 2.0;
      p2.theta = -PI / 2.0;
      targets_.push_back(p2);

      geometry_msgs::Pose2D p3;
      p3.x = -2.0;
      p3.y = -2.0;
      p3.theta = 0;
      targets_.push_back(p3);

      geometry_msgs::Pose2D p4;
      p4.x = 2.0;
      p4.y = -2.0;
      p4.theta = PI / 2.0;
      targets_.push_back(p4);
    }

    ~ActionClient() {}

    void doneCB(const actionlib::SimpleClientGoalState& state, const robot_msgs::MoveToPoseResultConstPtr& result) {
      ROS_INFO("Finished in state [%s]", state.toString().c_str());
      ROS_INFO_STREAM("Error: d - " << result -> distance_error << ", angle - " << result -> angle_error);
    }
    
    void activeCB() {}

    void feedbackCB(const robot_msgs::MoveToPoseFeedbackConstPtr& feedback) {
      ROS_INFO_STREAM("Process: " << feedback -> percentage << "%");
    }

    void execute()
    {
      ac_.waitForServer(); // Đợi server start
      int index = 0;
      while (ros::ok)
      {
        robot_msgs::MoveToPoseGoal goal;
        goal.target_pose = targets_[index % 4];
        index++;

        ac_.sendGoal( goal, 
                      boost::bind(&ActionClient::doneCB, this, _1, _2),
                      boost::bind(&ActionClient::activeCB, this),
                      boost::bind(&ActionClient::feedbackCB, this, _1)
                    );
                    
        ac_.waitForResult();
      }
    }
};
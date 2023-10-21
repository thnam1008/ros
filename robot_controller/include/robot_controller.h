#include "ros/ros.h"
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Transform.h>
//#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_srvs/SetBool.h>
#include <robot_msgs/MoveToPoseAction.h>
#include <actionlib/server/simple_action_server.h>

class RobotController
{
    private:
        ros::NodeHandle nh_;
        ros::Subscriber path_sub_;
        ros::Publisher cmd_vel_pub_;
        ros::ServiceServer enable_srv;

        nav_msgs::Path path_;
        geometry_msgs::Pose2D robot_pose_;

        bool enable_moving = false;

        tf2_ros::Buffer robot_tfbuffer_;
        tf2_ros::TransformListener robot_listener_;

        actionlib::SimpleActionServer<robot_msgs::MoveToPoseAction> as_;

    public:
        RobotController() : as_(nh_, "pose_control", boost::bind(&RobotController::executeCB, this, _1), false), robot_listener_(robot_tfbuffer_)
        {
            path_sub_= nh_.subscribe("path", 1, &RobotController::pathCB, this);

            cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
            enable_srv = nh_.advertiseService("enable_moving", &RobotController::enableCB, this);

            as_.start();
        }

        ~RobotController()
        {
        }

        void pathCB(const nav_msgs::Path& path)
        {
            path_=path;
        }

        void executeCB(const robot_msgs::MoveToPoseGoalConstPtr& goal)
        {
            enable_moving = true;

            ros::Rate loop_rate(20);
            double dx = goal -> target_pose.x - robot_pose_.x;
            double dy = goal -> target_pose.y - robot_pose_.y;
            double init_distance = sqrt(dx * dx + dy * dy);

            while (ros::ok()) {
                ros::spinOnce();
                double current_distance;
                geometry_msgs::Twist cmd_vel = this -> control(goal -> target_pose, current_distance);

                cmd_vel_pub_.publish(cmd_vel);
                robot_msgs::MoveToPoseFeedback f;
                f.percentage = 100 * (1 - current_distance / init_distance);
                as_.publishFeedback(f);

                if (as_.isPreemptRequested()) {
                    cmd_vel.linear.x = 0;
                    cmd_vel.angular.z = 0;
                    cmd_vel_pub_.publish(cmd_vel);
                    as_.setPreempted();
                    break;
                }

                double distance_tol = 0.02;
                double angle_tol = 0.02;
                double angle_diff = goal -> target_pose.theta - robot_pose_.theta;
                angle_diff = norm_angle(angle_diff);

                if (current_distance < distance_tol && angle_diff < angle_tol) {
                    robot_msgs::MoveToPoseResult res;
                    res.success = true;
                    res.distance_error = current_distance;
                    res.angle_error = angle_diff;

                    as_.setSucceeded();
                    break;
                }

                loop_rate.sleep();
            }
            enable_moving = false;
            return;
        }

        bool enableCB(std_srvs::SetBool::Request& req,
                      std_srvs::SetBool::Response& res)
        {
            enable_moving = req.data;
            res.success = true;
            res.message = "OK";
            return true;
        }

        geometry_msgs::Twist control(const geometry_msgs::Pose2D& target, double& current_distance) {
            double dx = target.x - robot_pose_.x;
            double dy = target.y - robot_pose_.y;
            double rho = sqrt(dx * dx + dy * dy);
            double beta = atan2(dy, dx);
            double alpha = beta - robot_pose_.theta;
            const double gamma = 1.0;
            const double k = 1.0, h = 0.05;
            double u = gamma * tanh(rho) * cos(alpha);
            double omega = k * alpha + gamma * sin(alpha) / alpha * tanh(rho) / rho * cos(alpha) * (alpha - h * beta);
            if (rho < 0.02) {
                u = 0.0;
                omega = -k * (robot_pose_.theta - target.theta);
            }
            geometry_msgs::Twist cmd_vel;
            cmd_vel.linear.x = u;
            cmd_vel.angular.z = omega;
            current_distance = rho;
            return cmd_vel;
        }

        double norm_angle(double in)
        {
            double out;
            const double pi = 3.14159;

            while (in > pi) {
                in -= pi;
            }

            while (in < pi) {
                in += pi;
            }

            return in;
        }

        void execute()
        {
            //code cua bo dieu khien
            geometry_msgs::Twist vel;
            if (enable_moving) {
                return;
            }
            // else {
            //     //van toc tinh tien
            //     vel.linear.x = 0.0;
            //     //toc do goc quay quanh truc z
            //     vel.angular.z = 0.0;
            // }
            //van toc tinh tien
            vel.linear.x = 0.0;
            //toc do goc quay quanh truc z
            vel.angular.z = 0.0;
            // std::cout << "Publish vel" << std::endl;
            cmd_vel_pub_.publish(vel);
        }

        void getRobotPose()
        {
            geometry_msgs::TransformStamped robot_pose_stamped;
            try {
                // Code in try
                // Param parent, child, 
                robot_pose_stamped = robot_tfbuffer_.lookupTransform("map", "base_footprint", ros::Time(0), ros::Duration(1.0));
            }
            catch (tf2::TransformException &ex) {
                ROS_WARN("%s",ex.what());
                return;
            }

            // Biến đổi transform -> pose2D
            // x = x, y = y, theta <- quaternion
            tf2::Transform robot_pose_tf2;
            tf2::fromMsg(robot_pose_stamped.transform, robot_pose_tf2);
            double roll, pitch, yaw;
            robot_pose_tf2.getBasis().getRPY(roll, pitch, yaw);
            robot_pose_.x = robot_pose_stamped.transform.translation.x;
            robot_pose_.y = robot_pose_stamped.transform.translation.y;
        }
};
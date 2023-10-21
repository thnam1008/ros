#include "ros/ros.h"
#include <std_srvs/SetBool.h>

class Client
{
    private:
        ros::NodeHandle nh_;
        ros::ServiceClient client_;
        double stop_time = 1.0, move_time = 3.0;
    public:
        Client()
        {
            client_ = nh_.serviceClient<std_srvs::SetBool>("enable_moving");
            nh_.getParam("a", stop_time);
            nh_.getParam("b", move_time);
        }
        ~Client()
        {
        }
        void execute()
        {
            while(ros::ok())
            {
                std_srvs::SetBool srv_call;

                srv_call.request.data = true;
                client_.call(srv_call);
                ros::Duration(move_time).sleep();

                srv_call.request.data = false;
                client_.call(srv_call);
                ros::Duration(stop_time).sleep();
            }
        }
};
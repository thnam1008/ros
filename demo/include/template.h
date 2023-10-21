#include "ros/ros.h"

class Template {
    private:
        ros::NodeHandle nh;
        int a;
    public:
        Template()
        {
            this->a = nh.getParam("a", this->a);
        }
        ~Template()
        {
        }
        void execute()
        {
            std::cout << this->a;
        }
};
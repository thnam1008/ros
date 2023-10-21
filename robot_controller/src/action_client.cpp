#include <action_client.h>

int main(int argc ,char** argv)
{
    ros::init(argc, argv, "action_client");
    ActionClient actionclient;
    actionclient.execute();
    return 1;
}
#include <template.h>

int main(int argc ,char** argv)
{
    ros::init(argc, argv, "Template");
    Template template1;
    template1.execute();

    return 1;
}


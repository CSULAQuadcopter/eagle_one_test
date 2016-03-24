#include <ros/ros.h>
#include <eagle_one_test/Pid2.h>

bool pid(eagle_one_test::Pid2::Request &req,
         eagle_one_test::Pid2::Response &res)
{
    if((req.error > 10) && (req.time > 10))
    {
            res.update = 3.14159;
    }
    else if((req.error < 10) && (req.time < 10))
    {
        res.update = 2.71828;
    }
    else
    {
        res.update = 0;
    }

    return true;
}

int main (int argc, char **argv)
{
    ros::init(argc, argv, "pid_server");
    ros::NodeHandle n;

    ros::ServiceServer service = n.advertiseService("pid", pid);
    ROS_INFO("Ready to control some stuff.");
    ros::spin();

    return 0;
}

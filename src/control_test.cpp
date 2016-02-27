/*
Name: sub_nav

Testing how to subscribe to different topics that the QuadCopter publishes

Written by: Josh Saunders (jay3ss)
Date created: 1/26/2016
Modified by: Josh Saunders (jay3ss)
Date modified: 2/8/2016
*/

#include <ros/ros.h>
#include <ardrone_autonomy/Navdata.h>
#include <nav_msgs/Odometry.h>
#include <eagle_one_test/ARNavdata.h>
#include <eagle_one_test/Drone.h>
#include <eagle_one_test/Odometry.h>
#include <string>
#include <iostream>

void Odometry::getOdomData(const nav_msgs::Odometry::ConstPtr& msg)
{

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sub_nav");
    ros::NodeHandle n;
    ros::Rate rate(10);

    // previous velocity
    double x_pos = 0;
    double y_pos = 0;

    geometry_msgs::Twist twist_msg;

    Drone qc;
    Odometry od;

    double start_time = (double) ros::Time::now().toSec();
    double run_time;
    double theta = 10;

    ros::Subscriber vel_info = n.subscribe("/ardrone/navdata", 1000, &Drone::set_navdata, &qc);
    ros::Subscriber odom = n.subscribe("/ardrone/odometry", 1000, &Odometry::getOdomData, &od);
    ros::Publisher follow = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);


    twist_msg.linear.x = 1;
    while(ros::ok())
    {
        run_time = (double) ros::Time::now().toSec() - start_time;
        //twist_msg.linear.x = 0.01;
        if (qc.getTagCount() >= 1)
        {
            x_pos = qc.calcTagDistanceX(qc.getTagX());

            // CHECKING THE BOUNDING BOX
            // is the QC too far ahead?
            if(x_pos > (5.5 * qc.getAltd() * 0.001))
            {
                // slow down
                twist_msg.linear.x /= 0.5;
            }
            // is the QC too far behind?
            else if (x_pos < -(5.5 * qc.getAltd() * 0.001))
            {
                // speed up
                twist_msg.linear.x *= 0.5;
            }

            // limit QC velocity to 1
            if (twist_msg.linear.x > 1)
            {
                twist_msg.linear.x = 1;
            }
            else if (twist_msg.linear.x < 0.000001)
            {
                twist_msg.linear.x = 0.01;
            }
            //twist_msg.linear.x = qc.calcTagDistanceX(qc.getTagX()) / 6400; //+ 0.25 * (qc.getVx() - qc.getPrevVx());
            /*
            if (twist_msg.linear.x > 1)
            {
                twist_msg.linear.x = 1;
            }
            else if(twist_msg.linear.x < -1)
            {
                twist_msg.linear.x = -1;
            }
            */
            //twist_msg.linear.y = qc.calcTagDistanceY(qc.getTagY()) / 600 + 0.25 * (qc.getVx() - qc.getPrevVx());
/*
            if ((qc.getYaw() > theta ) && (qc.getYaw() <= 180))
            {
                // angular.z < 0 => turn right
                twist_msg.angular.z = qc.degreesToRads(qc.getYaw()) / -4;
            }
            else if ((qc.getYaw() > 180 ) && (qc.getYaw() <= (360 - theta)))
            {
                // angular.z > 0 => turn left
                twist_msg.angular.z = qc.degreesToRads(qc.getYaw()) / -4;
            }
            else
            {
                twist_msg.angular.z = 0;
            }
*/
        }
        else
        {
            /*
            twist_msg.linear.x = 0.0;
            twist_msg.linear.y = 0.0;
            twist_msg.angular.z = 0.0;
            */
        }
        /*
        std::cout << "Tag: (" << qc.getTagX() << ", " << qc.getTagX() << ")\n";
        std::cout << "Vel: (" << twist_msg.linear.x << ", " << twist_msg.linear.y << ")\n";
        std::cout << "Yaw: (" << qc.getYaw() << ")\n";
        std::cout << "Yaw Vel: (" << twist_msg.angular.z << ")\n";
        qc.print_tag_distance();
        //twist_msg.linear.x = qc.getTagX();
        //twist_msg.linear.y = qc.getTagY();
        qc.setPrevVels();
        */
        std::cout << "Time " << run_time << "\tVel " << twist_msg.linear.x << "\tBB " << (5.5 * qc.getAltd() * 0.001) << "\tTagDist " << x_pos << "\n";
        follow.publish(twist_msg);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

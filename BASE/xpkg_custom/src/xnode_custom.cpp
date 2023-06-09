//////////////////////////////////////////////////
//xnode for vehicle control V2.0
//xpkg_vehicle
//////////////////////////////////////////////////
#include <ros_interface_custom.h>

using namespace XROS_VEHICLE;

void TimeCallback() {
    ROSInterface& ros_interface = ROSInterface::GetInterface();

    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 1;
    cmd_vel.linear.y = 0.0;
    cmd_vel.linear.z = 0.0;
    cmd_vel.angular.x = 0.0;
    cmd_vel.angular.y = 0.0;
    cmd_vel.angular.z = 1.0;
    ros_interface.PubCustomXstd(cmd_vel);

}
/*------------------------------------------------------------------------------------------------------------------
 * name: main
 -----------------------------------------------------------------------------------------------------------------*/
int main(int argc, char **argv)
{
    //system("gnome-terminal -x bash -c 'source /opt/ros/melodic/setup.bash;roscore'&");

    ROSInterface& ros_interface = ROSInterface::GetInterface();   
    ros_interface.BaseInit(argc, argv, "xnode_custom", 20.0, TimeCallback);
    ros_interface.ROSLog(LogLevel::kInfo," Running custom node");

    ros_interface.Work(); // ros.spin()
    return 0;


}

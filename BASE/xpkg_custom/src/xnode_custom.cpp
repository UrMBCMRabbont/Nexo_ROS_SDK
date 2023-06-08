//////////////////////////////////////////////////
//xnode for vehicle control V2.0
//xpkg_vehicle
//////////////////////////////////////////////////
#include "../xpkg_vehicle/include/FUNC/vehicle_func.h"
#include <BASE/xpkg_vehicle/include/FUNC/vehicle_func_odom.h>
#include <ros_interface_custom.h>

using namespace XROS_VEHICLE;

void TimeCallback() {
    VehicleFunc& vehicle_func = VehicleFunc::GetVehicleFunc();
    ROSInterface& ros_interface = ROSInterface::GetInterface();
    VehicleFuncOdom& func_odom = VehicleFuncOdom::GetVehicleFuncOdom();
    VehicleInfo info;
    static char sys_state = SYS_BASE_INIT;

    vehicle_func.ComDataIn();
    vehicle_func.OnlineCheck();
    vehicle_func.EnableCheck();
    vehicle_func.ModeCheck();

    vehicle_func.DevMove(0.5,0.5,0.0);
}
/*------------------------------------------------------------------------------------------------------------------
 * name: main
 -----------------------------------------------------------------------------------------------------------------*/
int main(int argc, char **argv)
{
    //system("gnome-terminal -x bash -c 'source /opt/ros/melodic/setup.bash;roscore'&");

    ROSInterface& ros_interface = ROSInterface::GetInterface();   
    ros_interface.BaseInit(argc, argv, "xnode_custom", 20.0);
    ros_interface.ROSLog(LogLevel::kInfo," Running custom node");

    ros_interface.Work(); // ros.spin()
    return 0;


}

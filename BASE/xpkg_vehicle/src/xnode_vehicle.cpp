//////////////////////////////////////////////////
//xnode for vehicle control V2.0
//xpkg_vehicle
//////////////////////////////////////////////////
#include <vehicle_func.h>
#include <vehicle_func_odom.h>
#include <ros_interface.h>
#include <lib_file_ini.h>

#define SYS_BASE_INIT 0
#define SYS_FUNC_INIT 1
#define SYS_READY 2
#define BEGINNER_STRAIGHT 3

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

    switch(sys_state)
    {
        case SYS_BASE_INIT:
            vehicle_func.BaseInit();
            if(vehicle_func.IsReady())sys_state = SYS_FUNC_INIT;
            break;
        case SYS_FUNC_INIT:
            func_odom.BasiInit(vehicle_func.GetVehicleInfo().s_type);
            ros_interface.ROSLog(LogLevel::kInfo,"\033[1;32m xnode_vehicle: =========== VEHICLE READY TO GO =========== \033[0m");
            sys_state = SYS_READY;
            break;
        case SYS_READY:            
            vehicle_func.VelDataIn();
            info = vehicle_func.GetVehicleInfo();
            vehicle_func.ComDataOut();
            vehicle_func.JsonDataOut();
            if(ros_interface.m_calc_speed)func_odom.CalcWithSpeed(info.s_speed_x,info.s_speed_y,info.s_speed_r,info.s_time,info.s_rotate_mode);
            else func_odom.CalcWithOdom(info.s_odo_left,info.s_odo_right,info.s_odo_left_s,info.s_odo_right_s,info.s_time,info.s_rotate_mode);
            func_odom.ShowLoc();
            func_odom.PubDevOdom();
            if(!vehicle_func.IsOnline())sys_state = SYS_BASE_INIT;

            if(sys_state == SYS_READY){
                sys_state = BEGINNER_STRAIGHT;
            }
            break;
        case BEGINNER_STRAIGHT:
            vehicle_func.DevMove(0.5,0.5,0.0);
            break;
    }
}
/*------------------------------------------------------------------------------------------------------------------
 * name: main
 -----------------------------------------------------------------------------------------------------------------*/
int main(int argc, char **argv)
{
    //system("gnome-terminal -x bash -c 'source /opt/ros/melodic/setup.bash;roscore'&");

    ROSInterface& ros_interface = ROSInterface::GetInterface();   
    ros_interface.BaseInit(argc, argv, "xnode_vehicle", 20.0, TimeCallback);

    LibFileIni& lib_file_ini = LibFileIni::GetLibFileIni();
    lib_file_ini.OpenFile(ros_interface.m_ini_path.c_str());

    ros_interface.Work(); // ros.spin()
    return 0;


}

#include <vehicle_func.h>
using namespace XROS_VEHICLE;
#define DEV_CLASS 1

VehicleFunc::VehicleFunc()
{
    f_online = false;
    f_enable = false;
    f_mode = false;
    f_new_vel = false;
    f_ready = false;
    m_online_count = 200;
}
/*------------------------------------------------------------------------------------------------------------------
 * name: GetInterface
 -----------------------------------------------------------------------------------------------------------------*/
VehicleFunc& VehicleFunc::GetVehicleFunc()
{
    static VehicleFunc vehicle_func;
    return vehicle_func;
}
/*------------------------------------------------------------------------------------------------------------------
 * name: BaseInit
 -----------------------------------------------------------------------------------------------------------------*/
void VehicleFunc::BaseInit(void)
{    
    ROSInterface& ros_interface = ROSInterface::GetInterface();
    LibFileIni& lib_file_ini = LibFileIni::GetLibFileIni();
    f_ready = false;
    if(f_online && f_enable && f_mode)
    {
        if(lib_file_ini.GetKeyValue(lib_file_ini.ToHexStr(m_dev_info.s_type).c_str(),"type").size() == 0)
        {
            ros_interface.ROSLog(LogLevel::kError," xnode_vehicle: Unknown vehicle!! \033[0m");
            ros_interface.Shutdown();
        }
        ros_interface.ROSLog(LogLevel::kInfo,"\033[1;34m xnode_vehicle: Device type = %s \033[0m",
                             lib_file_ini.GetKeyValue(lib_file_ini.ToHexStr(m_dev_info.s_type).c_str(),"type").c_str());
        ros_interface.ROSLog(LogLevel::kInfo,"\033[1;32m xnode_vehicle: ### Vehicle init finish ### \033[0m");
        DevZero();
        usleep(500000);
        f_ready = true;
    }
}
/*------------------------------------------------------------------------------------------------------------------
 * name: OnlineCheck
 -----------------------------------------------------------------------------------------------------------------*/
#define ONLINE_INIT 0
#define ONLINE 1
#define OFFINE 2
void VehicleFunc::OnlineCheck()
{
    ROSInterface& ros_interface = ROSInterface::GetInterface();
    static char online_state = ONLINE_INIT;
    if(m_online_count == 0)f_online = false;    
    else m_online_count--;
    switch(online_state)
    {
    case ONLINE_INIT:
        if(m_online_count == 1)
        {
            online_state = OFFINE;
            ros_interface.ROSLog(LogLevel::kError," xnode_vehicle: Vehicle offline,please check cable connection");
        }
        if(f_online)
        {
            online_state = ONLINE;
            ros_interface.ROSLog(LogLevel::kInfo,"\033[1;32m xnode_vehicle: Vehicle online \033[0m");
        }
        break;
    case OFFINE:
        if(f_online)
        {
            ros_interface.ROSLog(LogLevel::kInfo,"\033[1;32m xnode_vehicle: Vehicle online \033[0m");
            online_state = ONLINE;
        }
        break;
    case ONLINE:
        if(!f_online)
        {
            ros_interface.ROSLog(LogLevel::kError," xnode_vehicle: Vehicle offline,please check cable connection");
            online_state = OFFINE;
        }
        break;
    }
}
/*------------------------------------------------------------------------------------------------------------------
 * name: EnableCheck
 -----------------------------------------------------------------------------------------------------------------*/
#define ENABLE_INIT 0
#define ENABLE_INIT_FINISH 1
#define ENABLE 2
#define DISABLE 3
void VehicleFunc::EnableCheck()
{
    if(!f_online)return;
    ROSInterface& ros_interface = ROSInterface::GetInterface();
    static char enable_state = ENABLE_INIT;
    switch(enable_state)
    {
    case ENABLE_INIT:
        DevSetEnable(1);
        usleep(100000);
        DevSetEnable(1);
        usleep(500000);
        enable_state = ENABLE_INIT_FINISH;
        break;
    case ENABLE_INIT_FINISH:
        if(m_dev_info.s_en_state)
        {
            ros_interface.ROSLog(LogLevel::kInfo,"\033[1;32m xnode_vehicle: Vehicle enable \033[0m");
            enable_state = ENABLE;
            f_enable = true;
        }
        else
        {
            ros_interface.ROSLog(LogLevel::kError," xnode_vehicle: Vehicle disable");
            enable_state = DISABLE;
            f_enable = false;
        }
        break;
    case ENABLE:
        if(!m_dev_info.s_en_state)
        {
            ros_interface.ROSLog(LogLevel::kError," xnode_vehicle: Vehicle disable");
            enable_state = DISABLE;
            f_enable = false;
        }
        break;
    case DISABLE:
        if(m_dev_info.s_en_state)
        {
            ros_interface.ROSLog(LogLevel::kInfo,"\033[1;32m xnode_vehicle: Vehicle enable \033[0m");
            enable_state = ENABLE;
            f_enable = true;
            break;
        }
        DevSetEnable(1);
        usleep(500000);
        break;
    }
}
/*------------------------------------------------------------------------------------------------------------------
 * name: ModeCheck
 -----------------------------------------------------------------------------------------------------------------*/
#define MODE_INIT 0
#define MODE_INIT_FINISH 1
#define MODE_OK 2
#define MODE_WRONG 3
void VehicleFunc::ModeCheck()
{
    if(!f_online)return;
    ROSInterface& ros_interface = ROSInterface::GetInterface();
    static unsigned char mode_state = MODE_INIT,mode_old = 99;

    switch(mode_state)
    {
    case MODE_INIT:
        if(ros_interface.m_mode_lock == false)
        {
            f_mode = true;
            ros_interface.ROSLog(LogLevel::kInfo,"\033[1;32m xnode_vehicle: mode unlock \033[0m");
            mode_state = MODE_OK;
            break;
        }
        mode_state = MODE_INIT_FINISH;
        DevMode(MODE_CAN,BEEP_OFF,0,0);
        DevCtrlInfo(0);
        DevDrvState(0);
        DevSensorInfo(0);
        DevDrvMotion(0);
        usleep(200000);
        break;
    case MODE_INIT_FINISH:
        if(ros_interface.m_mode_lock == false)break;
        if(m_dev_info.s_work_mode == MODE_CAN)
        {
            mode_state = MODE_OK;
            ros_interface.ROSLog(LogLevel::kInfo,"\033[1;32m xnode_vehicle: mode OK(CAN mode) \033[0m");
            f_mode = true;
        }
        else mode_state = MODE_WRONG;
        break;
    case MODE_OK:
        if(ros_interface.m_mode_lock == false)break;
        if(m_dev_info.s_work_mode != MODE_CAN)
        {
            mode_state = MODE_WRONG;
            f_mode = false;
        }
        break;
    case MODE_WRONG:
        if(ros_interface.m_mode_lock == false)break;
        if(m_dev_info.s_work_mode == MODE_CAN)
        {
             ros_interface.ROSLog(LogLevel::kInfo,"\033[1;32m xnode_vehicle: mode OK(CAN mode) \033[0m");
             mode_state = MODE_OK;
             f_mode = true;
             mode_old = m_dev_info.s_work_mode;
             break;
        }
        if(mode_old != m_dev_info.s_work_mode)ros_interface.ROSLog(LogLevel::kWarn," xnode_vehicle: Wrong mode--%X (need be 2),"
                                                                                   "please close remote control or switch SWA to bottom ",m_dev_info.s_work_mode);
        mode_old = m_dev_info.s_work_mode;
        DevMode(MODE_CAN,BEEP_OFF,0,0);
        DevCtrlInfo(0);
        DevDrvState(0);
        DevSensorInfo(0);
        DevDrvMotion(0);
        usleep(200000);
        break;
    }
}
/*------------------------------------------------------------------------------------------------------------------
 * name: ComDataOut
 -----------------------------------------------------------------------------------------------------------------*/
void VehicleFunc::ComDataOut()
{
    if(f_new_vel == false)return;
    ROSInterface& ros_interface = ROSInterface::GetInterface();
    double rate_x = ros_interface.m_rate_x;
    double rate_y = ros_interface.m_rate_y;
    double rate_az = ros_interface.m_rate_az;
    DevMove(m_data_vel.line_x*rate_x,m_data_vel.line_y*rate_y,m_data_vel.ang_z*rate_az);
    f_new_vel = false;
}
/*------------------------------------------------------------------------------------------------------------------
 * name: JsonDataOut
 * detail: Pub vehicle info
 -----------------------------------------------------------------------------------------------------------------*/
void VehicleFunc::JsonDataOut()
{
    if(f_online == false)return;
    ROSInterface& ros_interface = ROSInterface::GetInterface();
    std::string send_data;
    DynamicJsonDocument doc(1024);
    doc["work_state"] = m_dev_info.s_work_state;
    doc["beep_state"] = m_dev_info.s_beep_state;
    doc["brake_state"] = m_dev_info.s_brake_state;
    doc["spec_func_state"] = m_dev_info.s_func_state;
    doc["work_mode"] = m_dev_info.s_work_mode;
    doc["battery_vol"] = m_dev_info.s_bat_vol;
    doc["err_motor"] = m_dev_info.s_err_motor;
    doc["err_driver"] = m_dev_info.s_err_driver;
    doc["err_driver_offline"] = m_dev_info.s_err_driver_offline;
    doc["err_bat_down"] = m_dev_info.s_err_bat_down;
    doc["err_bat_low"] = m_dev_info.s_err_bat_low;
    doc["err_ctrl_lost"] = m_dev_info.s_err_ctrl_lost;
    doc["err_bump"] = m_dev_info.s_err_bump;
    doc["speed_x"] = m_dev_info.s_speed_x;
    doc["speed_y"] = m_dev_info.s_speed_y;
    doc["speed_r"] = m_dev_info.s_speed_r;
    doc["odom_left_main"] = m_dev_info.s_odo_left;
    doc["odom_right_main"] = m_dev_info.s_odo_right;
    doc["odom_left_second"] = m_dev_info.s_odo_left_s;
    doc["odmo_right_second"] = m_dev_info.s_odo_right_s;
    //for(unsigned long i=0;i<8;i++){doc["bump_state"][i] = m_dev_info.s_bump[i];}
    serializeJson(doc, send_data);
    ros_interface.PubDevState(send_data);
}
/*------------------------------------------------------------------------------------------------------------------
 * name: VelDataIn
 -----------------------------------------------------------------------------------------------------------------*/
void VehicleFunc::VelDataIn()
{
    ROSInterface& ros_interface = ROSInterface::GetInterface();
    if(ros_interface.GetVelFlag() == 0)return;
    vector<VelData> data_vel_list = ros_interface.GetVelMsg();
    m_data_vel = *(data_vel_list.end()-1);
    f_new_vel = true;
    ros_interface.ResetVelFlag();
    ros_interface.ClearVelMsg();
}
/*------------------------------------------------------------------------------------------------------------------
 * name: ComDataIn
 * detail: Convert data from COM to std dev status
 -----------------------------------------------------------------------------------------------------------------*/
void VehicleFunc::ComDataIn()
{
    ROSInterface& ros_interface = ROSInterface::GetInterface();
    if(ros_interface.GetComXstdFlag() == 0)return;
    vector<XstdData> data_com_list = ros_interface.GetComXstdMsg();
    short temp;
    int temp_odom;
    XstdData data_recv;
    for(unsigned long i=0;i<data_com_list.size();i++)
    {
        data_recv = data_com_list.at(i);
        if(data_recv.id_c != 0x01) continue;
        m_dev_info.s_num = data_recv.id_n;
        m_dev_info.s_type = data_recv.id_t;
        switch(data_recv.id_f)
        {
            /************************firmware version***************************************/
            case 0xA2:
                m_dev_info.s_ver_hard = std::to_string(data_recv.data[3]) + std::to_string(data_recv.data[2]) + "." + std::to_string(data_recv.data[1]) + std::to_string(data_recv.data[0]);
                m_dev_info.s_ver_soft = std::to_string(data_recv.data[7]) + std::to_string(data_recv.data[6]) + "." + std::to_string(data_recv.data[5]) + std::to_string(data_recv.data[4]);
            break;
            /************************heart beat***************************************/
            case 0xB0:
                m_dev_info.s_en_state = data_recv.data[0];
                m_online_count = 200;
                f_online = true;
            break;
            /************************sys status***************************************/
            case 0xB1:
                m_dev_info.s_work_state = data_recv.data[0];
                m_dev_info.s_work_mode = data_recv.data[1];
                memcpy(&temp,&data_recv.data[2],2);
                m_dev_info.s_bat_vol = static_cast<float>(temp)/10;
                m_dev_info.s_beep_state = data_recv.data[4];
                m_dev_info.s_err_bump = data_recv.data[5] & 0x01;
                m_dev_info.s_err_motor = (data_recv.data[5]>>1) & 0x01;
                m_dev_info.s_err_driver = (data_recv.data[5]>>2) & 0x01;
                m_dev_info.s_err_driver_offline = (data_recv.data[5]>>3) & 0x01;
                m_dev_info.s_err_bat_down = (data_recv.data[5]>>4) & 0x01;
                m_dev_info.s_err_bat_low = (data_recv.data[5]>>5) & 0x01;
                m_dev_info.s_err_ctrl_lost = (data_recv.data[5]>>6) & 0x01;
                m_dev_info.s_brake_state = data_recv.data[6];
                m_dev_info.s_func_state = data_recv.data[7];
            break;
            /************************motion status***************************************/
            case 0xB2:
                memcpy(&temp,&data_recv.data[0],2);
                m_dev_info.s_speed_x = static_cast<double>(temp)/1000;
                memcpy(&temp,&data_recv.data[2],2);
                m_dev_info.s_speed_y = static_cast<double>(temp)/1000;
                memcpy(&temp,&data_recv.data[4],2);
                m_dev_info.s_speed_r = static_cast<double>(temp)/1000;
                m_dev_info.s_time = data_recv.time;
            break;
            /************************main odom***************************************/
            case 0xB3:
                memcpy(&temp_odom,&data_recv.data[0],4);
                m_dev_info.s_odo_left = static_cast<double>(temp_odom)/1000;
                memcpy(&temp_odom,&data_recv.data[4],4);
                m_dev_info.s_odo_right = static_cast<double>(temp_odom)/1000;
            break;
            /************************secondary odom***************************************/
            case 0xB4:
                memcpy(&temp_odom,&data_recv.data[0],4);
                m_dev_info.s_odo_left_s = static_cast<double>(temp_odom)/1000;
                memcpy(&temp_odom,&data_recv.data[4],4);
                m_dev_info.s_odo_right_s = static_cast<double>(temp_odom)/1000;
            break;
            /************************ctrl status***************************************/
            case 0xB5:
                m_dev_info.s_ctrl_swa =data_recv.data[0] & 0x03;
                m_dev_info.s_ctrl_swb =(data_recv.data[0]>>2) & 0x03;
                m_dev_info.s_ctrl_swc =(data_recv.data[0]>>4) & 0x03;
                m_dev_info.s_ctrl_swd =(data_recv.data[0]>>6) & 0x03;
                m_dev_info.s_ctrl_left_h = data_recv.data[1];
                m_dev_info.s_ctrl_left_v = data_recv.data[2];
                m_dev_info.s_ctrl_right_h = data_recv.data[3];
                m_dev_info.s_ctrl_right_v = data_recv.data[4];
                m_dev_info.s_ctrl_left_vra = data_recv.data[5];
                m_dev_info.s_ctrl_right_vra = data_recv.data[6];
            break;
            /************************safety sensor***************************************/
            case 0xB6:
                for(int i=0;i<8;i++)m_dev_info.s_bump[i] = (data_recv.data[0]>>i) & 0x01;
                for(unsigned long j=1;j<7;j++)m_dev_info.s_sonic[j-1] = data_recv.data[j];
            break;
            /************************driver motion info***************************************/
            case 0xB7:
                memcpy(&m_dev_info.s_driver_speed[data_recv.data[0]],&data_recv.data[1],2);
                memcpy(&m_dev_info.s_driver_pulse[data_recv.data[0]],&data_recv.data[3],4);
            break;
            /************************driver status info***************************************/
            case 0xB8:
                memcpy(&temp,&data_recv.data[1],2);
                m_dev_info.s_driver_volt[data_recv.data[0]] = static_cast<float>(temp)/10;
                memcpy(&temp,&data_recv.data[3],2);
                m_dev_info.s_driver_current[data_recv.data[0]] = static_cast<float>(temp)/10;
                m_dev_info.s_driver_heat[data_recv.data[0]] = static_cast<char>(data_recv.data[5]);
                m_dev_info.s_motor_heat[data_recv.data[0]] = static_cast<char>(data_recv.data[6]);
                m_dev_info.s_driver_err_volt_low[data_recv.data[0]] = data_recv.data[7] & 0x01;
                m_dev_info.s_driver_err_heat_over[data_recv.data[0]] = (data_recv.data[7]>>1) & 0x01;
                m_dev_info.s_motor_err_current_over[data_recv.data[0]] = (data_recv.data[7]>>2) & 0x01;
                m_dev_info.s_motor_err_heat_over[data_recv.data[0]] = (data_recv.data[7]>>3) & 0x01;
            break;
            default:
            break;
        }

    }
    ros_interface.ResetComXstdFlag();
    ros_interface.ClearComXstdMsg();
}
/*------------------------------------------------------------------------------------------------------------------
 * name: DevReset
 * detail: Vehicle reset CMD
 -----------------------------------------------------------------------------------------------------------------*/
void VehicleFunc::DevReset()
{
    XstdData order;
    ROSInterface& ros_interface = ROSInterface::GetInterface();
    order.id_c = DEV_CLASS;
    order.id_t = m_dev_info.s_type;
    order.id_n = m_dev_info.s_num;
    order.id_f = 0x01;
    order.len = 3;
    order.data[0] = DEV_CLASS;
    order.data[1] = m_dev_info.s_type;
    order.data[2] = m_dev_info.s_num;
    ros_interface.PubComXstd(order);
}
/*------------------------------------------------------------------------------------------------------------------
 * name: DevClear
 * detail: vehicle clear all error status
 -----------------------------------------------------------------------------------------------------------------*/
void VehicleFunc::DevClear()
{
    XstdData order;
    ROSInterface& ros_interface = ROSInterface::GetInterface();
    order.id_c = DEV_CLASS;
    order.id_t = m_dev_info.s_type;
    order.id_n = m_dev_info.s_num;
    order.id_f = 0x04;
    order.len = 1;
    order.data[0] = 0xCC;
    ros_interface.PubComXstd(order);
}
/*------------------------------------------------------------------------------------------------------------------
 * name: DevVersion
 -----------------------------------------------------------------------------------------------------------------*/
void VehicleFunc::DevVersion()
{
    XstdData order;
    ROSInterface& ros_interface = ROSInterface::GetInterface();
    order.id_c = DEV_CLASS;
    order.id_t = m_dev_info.s_type;
    order.id_n = m_dev_info.s_num;
    order.id_f = 0x02;
    order.len = 0;
    ros_interface.PubComXstd(order);
}
/*------------------------------------------------------------------------------------------------------------------
 * name: DevSetState
 -----------------------------------------------------------------------------------------------------------------*/
void VehicleFunc::DevSetEnable(bool en)
{
    XstdData order;
    ROSInterface& ros_interface = ROSInterface::GetInterface();
    order.id_c = DEV_CLASS;
    order.id_t = m_dev_info.s_type;
    order.id_n = m_dev_info.s_num;
    order.id_f = 0x03;
    order.len = 4;
    order.data[0] = DEV_CLASS;
    order.data[1] = m_dev_info.s_type;
    order.data[2] = m_dev_info.s_num;
    order.data[3] = en;
    ros_interface.PubComXstd(order);
}
/*------------------------------------------------------------------------------------------------------------------
 * name: DevZero
 * detail: Set motion data to zero
 -----------------------------------------------------------------------------------------------------------------*/
void VehicleFunc::DevZero()
{
    XstdData order;
    ROSInterface& ros_interface = ROSInterface::GetInterface();
    order.id_c = DEV_CLASS;
    order.id_t = m_dev_info.s_type;
    order.id_n = m_dev_info.s_num;
    order.id_f = 0x05;
    order.len = 1;
    order.data[0] = 0xCC;
    ros_interface.PubComXstd(order);
}
/*------------------------------------------------------------------------------------------------------------------
 * name: DevMode
 * detail: Set vehicle mode
 -----------------------------------------------------------------------------------------------------------------*/
void VehicleFunc::DevMode(unsigned char mode,bool beep,bool brake,bool func)
{
    if(mode > 2)mode = 0;
    XstdData order;
    ROSInterface& ros_interface = ROSInterface::GetInterface();
    order.id_c = DEV_CLASS;
    order.id_t = m_dev_info.s_type;
    order.id_n = m_dev_info.s_num;
    order.id_f = 0x11;
    order.len = 4;
    order.data[0] = mode;
    order.data[1] = beep;
    order.data[2] = brake;
    order.data[3] = func;
    ros_interface.PubComXstd(order);
}
/*------------------------------------------------------------------------------------------------------------------
 * name: DevMove
 -----------------------------------------------------------------------------------------------------------------*/
void VehicleFunc::DevMove(double speed_x, double speed_y, double rotate)
{
    // std::cout << "DevMove Called: " << speed_x << speed_y << rotate << std::endl;

    XstdData order;
    ROSInterface& ros_interface = ROSInterface::GetInterface();
    short data_speed_x = static_cast<short>(speed_x*1000);
    short data_speed_y = static_cast<short>(speed_y*1000);
    short data_rotate = static_cast<short>(rotate*1000);
    order.id_c = DEV_CLASS;
    order.id_t = m_dev_info.s_type;
    order.id_n = m_dev_info.s_num;
    order.id_f = 0x12;
    order.len = 6;
    memcpy(&order.data[0],&data_speed_x,2);
    memcpy(&order.data[2],&data_speed_y,2);
    memcpy(&order.data[4],&data_rotate,2);
    ros_interface.PubComXstd(order);
}
/*------------------------------------------------------------------------------------------------------------------
 * name: DevCtrlInfo
 -----------------------------------------------------------------------------------------------------------------*/
void VehicleFunc::DevCtrlInfo(unsigned char state)
{
    XstdData order;
    ROSInterface& ros_interface = ROSInterface::GetInterface();
    order.id_c = DEV_CLASS;
    order.id_t = m_dev_info.s_type;
    order.id_n = m_dev_info.s_num;
    order.id_f = 0x15;
    order.len = 1;
    order.data[0] = state;
    ros_interface.PubComXstd(order);
}
/*------------------------------------------------------------------------------------------------------------------
 * name: DevSensorInfo
 -----------------------------------------------------------------------------------------------------------------*/
void VehicleFunc::DevSensorInfo(unsigned char state)
{
    XstdData order;
    ROSInterface& ros_interface = ROSInterface::GetInterface();
    order.id_c = DEV_CLASS;
    order.id_t = m_dev_info.s_type;
    order.id_n = m_dev_info.s_num;
    order.id_f = 0x16;
    order.len = 1;
    order.data[0] = state;
    ros_interface.PubComXstd(order);
}
/*------------------------------------------------------------------------------------------------------------------
 * name: DevDrvMotion
 -----------------------------------------------------------------------------------------------------------------*/
void VehicleFunc::DevDrvMotion(unsigned char state)
{
    XstdData order;
    ROSInterface& ros_interface = ROSInterface::GetInterface();
    order.id_c = DEV_CLASS;
    order.id_t = m_dev_info.s_type;
    order.id_n = m_dev_info.s_num;
    order.id_f = 0x17;
    order.len = 1;
    order.data[0] = state;
    ros_interface.PubComXstd(order);
}
/*------------------------------------------------------------------------------------------------------------------
 * name: DevDrvState
 -----------------------------------------------------------------------------------------------------------------*/
void VehicleFunc::DevDrvState(unsigned char state)
{
    XstdData order;
    ROSInterface& ros_interface = ROSInterface::GetInterface();
    order.id_c = DEV_CLASS;
    order.id_t = m_dev_info.s_type;
    order.id_n = m_dev_info.s_num;
    order.id_f = 0x18;
    order.len = 1;
    order.data[0] = state;
    ros_interface.PubComXstd(order);
}

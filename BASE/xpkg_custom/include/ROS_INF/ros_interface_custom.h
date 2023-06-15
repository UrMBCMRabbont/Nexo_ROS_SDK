
#ifndef ROS_INTERFACE_H
#define ROS_INTERFACE_H

#define _Kp 35.0
#define _Ki 1.0
#define _Kd 20.0

#include <string>
#include <vector>
#include <memory>
#include <ros/ros.h>
#include <stdarg.h>
#include <string>

//add message include below////////////////////////////////
#include <xpkg_msgs/XmsgCommData.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/String.h>


using namespace std;

struct VehicleInfo
{
    //vehicle info
    unsigned char s_type = 0;
    unsigned char s_num = 0;
    //hardware and software version
    std::string s_ver_hard = "--";
    std::string s_ver_soft = "--";
    //system status
    bool s_en_state = false;   //enable state
    bool s_work_state = true;    //0x00 fine,0x01 error
    bool s_beep_state = false;
    bool s_brake_state = false;
    bool s_func_state = false;
    unsigned char s_work_mode = 0xFF;     //wokring mode:0x00 standby,0x01 remote,0x02 CAN,0x03 free
    unsigned char s_rotate_mode = 0x00;   //angle_vel:0x00 angel:0x01
    float s_bat_vol = 0.0;     // *0.1V
    bool s_err_motor = true;
    bool s_err_driver_offline = true;
    bool s_err_driver = true;
    bool s_err_bat_down = true; //<22V
    bool s_err_bat_low = true;  //<23V
    bool s_err_ctrl_lost = true;
    bool s_err_bump = true;
    //motion status
    double s_speed_x = 0.0;   //m/s
    double s_speed_y = 0.0;   //m/s
    double s_speed_r = 0.0;   //rad/s
    //contraller status
    unsigned char s_ctrl_swa = 0x00;     //0x01 up,0x02 mid,0x03 down
    unsigned char s_ctrl_swb = 0x00;     //0x01 up,0x02 mid,0x03 down
    unsigned char s_ctrl_swc = 0x00;     //0x01 up,0x02 mid,0x03 down
    unsigned char s_ctrl_swd = 0x00;     //0x01 up,0x02 mid,0x03 down
    unsigned char s_ctrl_left_h = 0x00;  //-100~100
    unsigned char s_ctrl_left_v = 0x00;  //-100~100
    unsigned char s_ctrl_right_h = 0x00; //-100~100
    unsigned char s_ctrl_right_v = 0x00; //-100~100
    unsigned char s_ctrl_left_vra = 0x00; //-100~100
    unsigned char s_ctrl_right_vra = 0x00; //-100~100
    //odometer
    double s_odo_left = 0.0;   //m
    double s_odo_right = 0.0;  //m
    double s_odo_left_s = 0.0;  //m
    double s_odo_right_s = 0.0;  //m
    //bumper
    bool s_bump[8] = {0};
    //sonic
    unsigned char s_sonic[6] = {0};
    //driver status
    short s_driver_speed[4] = {0};    //PRM
    float s_driver_current[4] = {0};  //*0.1A
    int s_driver_pulse[4] = {0};
    float s_driver_volt[4] = {0};     //*0.1V
    char s_driver_heat[4] = {0};     //1C
    char s_motor_heat[4] = {0};     //1C
    bool s_driver_err_volt_low[4] = {1,1,1,1};
    bool s_driver_err_heat_over[4] = {1,1,1,1};
    bool s_motor_err_current_over[4] = {1,1,1,1};
    bool s_motor_err_heat_over[4] = {1,1,1,1};
    double s_time = 0.0;
};

namespace XROS_VEHICLE
{
////////////////////////////////////////////////////////////
enum class LogLevel
{
  kDebug = 0,
  kInfo,
  kWarn,
  kError,
  kFatal
};
struct XstdData
{
  unsigned char id_c;
  unsigned char id_t;
  unsigned char id_n;
  unsigned char id_f;
  unsigned char len;
  unsigned char data[8];
  double time;
};
struct VelData
{
  double line_x;
  double line_y;
  double line_z;
  double ang_x;
  double ang_y;
  double ang_z;
};

struct SPEED
{
  double x;
  double y;
  double r;
  double s_time;
};

struct XYW
{
  double x;
  double y;
  double w;
};
struct PID
{
  XYW P;
  XYW I;
  XYW D;
  XYW value;
};
////////////////////////////////////////////////////////////
class ROSInterface
{
  /*********************************************************
                    Base function zone
  *********************************************************/
  public:
    static ROSInterface& GetInterface();
    void BaseInit(int argc, char* argv[], std::string node_name,double period, void (*handle)());
    inline void Work() {ros::spin();}
    inline void WorkOnce() {ros::spinOnce();}
    inline void Shutdown() {ros::shutdown();}
    inline bool Ok() {return ros::ok();}
    inline ros::Time GetTime(){return ros::Time::now();};
    void ROSLog(LogLevel, const char*, ...);
    //inline void TimerStart(){m_timer.start();}
    //inline void TimerStop(){m_timer.stop();}
  
  private:
    ROSInterface() = default;
    virtual ~ROSInterface() = default;
    void VariableInit();
    void ParameterInit();
    void PublisherInit();
    void SubscriptionInit();
    void TimerInit(double period, void (*handle)());

  protected:
    inline void TimerCallback(const ros::TimerEvent&) { m_timer_handle(); }

  private:
    ros::NodeHandle* m_node_ptr;
    ros::NodeHandle* m_node_local_ptr;
    ros::Timer m_timer;
    void (*m_timer_handle)();

  /*********************************************************
                        Custom zone
  *********************************************************/
  public:
    //add pub function below////////////////////////////////
    // void PubCustomXstd(const XstdData& data);
    void PubCustomXstd(const geometry_msgs::Twist& cmd_vel);

    //add sub function below////////////////////////////////
    void PIDCustomCallback(const nav_msgs::Odometry& odom);
    void CustomXstdCallback(const xpkg_msgs::XmsgCommData& data);

    PID pid_control(nav_msgs::Odometry odom, nav_msgs::Odometry target, SPEED s_speed);
    inline bool GetComXstdFlag() { return m_f_com_xstd; }
    inline void ResetComXstdFlag() { m_f_com_xstd = false; }
    inline bool GetVelFlag() { return m_f_vel; }
    inline void ResetVelFlag() { m_f_vel = false; }
    inline vector<XstdData> GetComXstdMsg() { return m_list_com_xstd; }
    inline vector<VelData> GetVelMsg() { return m_list_vel; }
    inline void ClearComXstdMsg() { m_list_com_xstd.clear(); }
    inline void ClearVelMsg() { m_list_vel.clear(); }
    inline PID ReturnPID() { return GAIN; }

    //add sub callback below////////////////////////////////////

  public:
    //add param Variable below///////////////////////////////
    std::string m_ini_path;
    bool m_show_loc;
    bool m_calc_speed;
    bool m_mode_lock;
    bool m_show_path;
    double m_rate_x;
    double m_rate_y;
    double m_rate_z;
    double m_rate_az; 

  private:
    //add sub flag below/////////////////////////////////////
    bool m_f_com_xstd;
    bool m_f_vel;
    PID GAIN;

    //add pub Variable below/////////////////////////////////
    ros::Publisher pub_custom_xstd;

    //add sub Variable below/////////////////////////////////
    ros::Subscriber sub_custom_odom;
    ros::Subscriber sub_custom_xstd;

    //add nomal Variable below///////////////////////////////
    vector<XstdData> m_list_com_xstd;
    vector<VelData> m_list_vel;
    vector<SPEED> m_list_speed;
  
};

}//namespace HEXROS
#endif // ROS_INTERFACE_H

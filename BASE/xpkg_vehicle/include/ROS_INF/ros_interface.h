
#ifndef ROS_INTERFACE_H
#define ROS_INTERFACE_H

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
    void PubComXstd(const XstdData& data);
    void PubOdom(double lx,double ly,double th,double vx,double vy,double vth);
    void PubDevState(const std::string& data);

    //add sub function below////////////////////////////////
    inline bool GetComXstdFlag() { return m_f_com_xstd; }
    inline void ResetComXstdFlag() { m_f_com_xstd = false; }
    inline bool GetVelFlag() { return m_f_vel; }
    inline void ResetVelFlag() { m_f_vel = false; }
    inline vector<XstdData> GetComXstdMsg() { return m_list_com_xstd; }
    inline vector<VelData> GetVelMsg() { return m_list_vel; }
    inline void ClearComXstdMsg() { m_list_com_xstd.clear(); }
    inline void ClearVelMsg() { m_list_vel.clear(); }

    //add sub callback below////////////////////////////////////
    void ComXstdCallback(const xpkg_msgs::XmsgCommData& data);
    void VelCallback(const geometry_msgs::Twist& cmd_vel);
    void CustomXstdCallback(const geometry_msgs::Twist& cmd_vel);
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

    //add pub Variable below/////////////////////////////////
    ros::Publisher pub_odom;
    ros::Publisher pub_com_xstd;
    ros::Publisher pub_device_state;
    ros::Publisher pub_path;

    //add sub Variable below/////////////////////////////////
    ros::Subscriber sub_vel;
    ros::Subscriber sub_com_xstd;
    ros::Subscriber sub_custom_xstd;

    //add nomal Variable below///////////////////////////////
    vector<XstdData> m_list_com_xstd;
    vector<VelData> m_list_vel;    
  
};

}//namespace HEXROS
#endif // ROS_INTERFACE_H

#include <ros_interface_custom.h>

namespace XROS_VEHICLE
{
void ROSInterface::BaseInit(int argc, char* argv[], std::string node_name,double period, void (*handle)())
{
  ros::init(argc, argv, node_name);
  static ros::NodeHandle nh;
  static ros::NodeHandle nh_local("~");
  m_node_ptr = &nh;
  m_node_local_ptr = &nh_local;
  VariableInit();
  ParameterInit();
  PublisherInit();
  SubscriptionInit();
  TimerInit(period, handle);
  ROSLog(LogLevel::kInfo,"\033[1;32m %s: ### ROS interface init finish ### \033[0m",node_name.data());
}
/*------------------------------------------------------------------------------------------------------------------
 * name: GetInterface
 -----------------------------------------------------------------------------------------------------------------*/
ROSInterface& ROSInterface::GetInterface()
{
  static ROSInterface ros_interface;
  return ros_interface;
}
/*------------------------------------------------------------------------------------------------------------------
 * name: ROSLog
 -----------------------------------------------------------------------------------------------------------------*/
void ROSInterface::ROSLog(LogLevel level, const char* format, ...)
{
  char* buffer;

  va_list args;
  va_start(args, format);
  int32_t len = vasprintf(&buffer, format, args);
  va_end(args);

  if (len < 0)
  {
    ROS_FATAL("### Wrong Log Message ###");
    return;
  }

  switch (level)
  {
    case LogLevel::kDebug:
      ROS_DEBUG("%s", buffer);
      break;
    case LogLevel::kInfo:
      ROS_INFO("%s", buffer);
      break;
    case LogLevel::kWarn:
      ROS_WARN("%s", buffer);
      break;
    case LogLevel::kError:
      ROS_ERROR("%s", buffer);
      break;
    case LogLevel::kFatal:
      ROS_FATAL("%s", buffer);
      break;
    default:
      ROS_FATAL("### Wrong Log Level ###");
      ROS_FATAL("%s", buffer);
      break;
  }
  free(buffer);
}
/*------------------------------------------------------------------------------------------------------------------
 * name: TimerInit
 -----------------------------------------------------------------------------------------------------------------*/
void ROSInterface::TimerInit(double period, void (*handle)())
{
  m_timer_handle = handle;
  m_timer = m_node_ptr->createTimer(ros::Duration(period * 0.001),&ROSInterface::TimerCallback, this);
}
/*------------------------------------------------------------------------------------------------------------------
 * name: ParameterInit
 -----------------------------------------------------------------------------------------------------------------*/
void ROSInterface::ParameterInit()
{
  /* add more later */   
}
/*------------------------------------------------------------------------------------------------------------------
 * name: VariableInit
 -----------------------------------------------------------------------------------------------------------------*/
void ROSInterface::VariableInit()
{
  /* add more later */
}
/*------------------------------------------------------------------------------------------------------------------
 * name: PublisherInit
 -----------------------------------------------------------------------------------------------------------------*/
void ROSInterface::PublisherInit()
{
  pub_custom_xstd = m_node_local_ptr->advertise<geometry_msgs::Twist>("/xtopic_comm/custom_cmd_send", 50);
}
/*------------------------------------------------------------------------------------------------------------------
 * name: SubscriptionInit
 -----------------------------------------------------------------------------------------------------------------*/
void ROSInterface::SubscriptionInit()
{ 
  sub_custom_xstd = m_node_local_ptr->subscribe("/xtopic_comm/com_recv_xstd_vehicle", 1000, &ROSInterface::CustomXstdCallback, this);
  sub_custom_odom = m_node_local_ptr->subscribe("/odom", 1000, &ROSInterface::PIDCustomCallback, this);
}
/*------------------------------------------------------------------------------------------------------------------
 * name: PubCustomXstd
 -----------------------------------------------------------------------------------------------------------------*/
void ROSInterface::PubCustomXstd(const geometry_msgs::Twist& cmd_vel)
{
    pub_custom_xstd.publish(cmd_vel);
}
/*------------------------------------------------------------------------------------------------------------------
 * name: sub callback
 -----------------------------------------------------------------------------------------------------------------*/
void ROSInterface::CustomXstdCallback(const xpkg_msgs::XmsgCommData& data){
  m_f_com_xstd = true;
  XstdData data_com;
  data_com.len = data.len;
  memcpy(&data_com.data[0],&data.data[0],data.len);
  data_com.time = data.time.toSec();
  m_list_com_xstd.push_back(data_com);
  if(m_list_com_xstd.size()>500)m_list_com_xstd.clear();
} 

PID ROSInterface::pid_control(nav_msgs::Odometry odom, nav_msgs::Odometry target, SPEED s_speed){
  ROSInterface& ros_interface = ROSInterface::GetInterface();
  double _dt = ros_interface.GetTime().toSec() - s_speed.s_time;
  std::cout << "speed_time: " <<  _dt << std::endl;
  PID gain;
  gain.P.y = _Kp*(0.0 - odom.pose.pose.position.y);
  gain.I.y = _Ki*(0.0 - odom.pose.pose.position.y)*_dt;
  gain.D.y = _Kd*s_speed.x;
  gain.value.y = gain.P.y + gain.I.y + gain.D.y;

  std::cout << "\nP: "<< gain.P.y << "\nI: "<< gain.I.y << "\nD: "<< gain.D.y << std::endl;
  return gain;
}

void ROSInterface::PIDCustomCallback(const nav_msgs::Odometry& odom)
{
  std::cout << "Received Odom info" << std::endl;

  ROSInterface& ros_interface = ROSInterface::GetInterface();
  vector<XstdData> data_com_list = ros_interface.GetComXstdMsg();
  XstdData data_recv;
  short temp;
  SPEED s_speed;
  for(unsigned long i = 0; i < data_com_list.size(); i++)
  {
      data_recv = data_com_list.at(i);
      if(data_recv.id_c != 0x01) continue;
      switch(data_recv.id_f)
      {
          /************************motion status***************************************/
          case 0xB2:
              memcpy(&temp,&data_recv.data[0],2);
              s_speed.x = static_cast<double>(temp)/1000;
              memcpy(&temp,&data_recv.data[2],2);
              s_speed.y = static_cast<double>(temp)/1000;
              memcpy(&temp,&data_recv.data[4],2);
              s_speed.r = static_cast<double>(temp)/1000;
              s_speed.s_time = data_recv.time;
      }
  }
  GAIN = pid_control(odom, odom, s_speed);

  // memcpy(&data_com.data[0],&data.data[0],data.len);
  // m_data_com_xstd.push_back(data_com);
  // if(m_data_com_xstd.size()>500)m_data_com_xstd.clear();
}

}//namespace HEXROS

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
PID ROSInterface::pid_gain(nav_msgs::Odometry odom, nav_msgs::Odometry target){
  PID gain;
  gain.P.x = 0;
  gain.P.y = 10*(odom.pose.pose.position.y - 0.0);
  gain.P.w = 10*(odom.pose.pose.orientation.w - 1.0);

  std::cout << "gain.P.y: " << gain.P.y << std::endl;
  std::cout << "gain.P.w: " << gain.P.w << std::endl;
  return gain;
}
void ROSInterface::PIDCustomCallback(const nav_msgs::Odometry& odom)
{
  std::cout << "Received Odom info" << std::endl;
  // odom.header.stamp = current_time;
  // odom.header.frame_id = "odom";
  // odom.child_frame_id = "base_link";
  GAIN = pid_gain(odom, odom);
  // std::cout << odom.pose.pose.position.x << std::endl;
  // std::cout << odom.pose.pose.position.y << std::endl;
  // std::cout << odom.pose.pose.position.z << std::endl;
  // std::cout << odom.pose.pose.orientation << std::endl;
  // std::cout << odom.twist.twist.angular.z << std::endl;

  // memcpy(&data_com.data[0],&data.data[0],data.len);
  // m_data_com_xstd.push_back(data_com);
  // if(m_data_com_xstd.size()>500)m_data_com_xstd.clear();
}

}//namespace HEXROS

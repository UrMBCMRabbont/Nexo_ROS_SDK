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
  /* add more later */
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


}//namespace HEXROS

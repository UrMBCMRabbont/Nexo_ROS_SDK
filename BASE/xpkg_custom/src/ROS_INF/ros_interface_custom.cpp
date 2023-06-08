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
  pub_custom_xstd; = m_node_local_ptr->advertise<geometry_msgs::Twist>("/xtopic_comm/custom_cmd_send", 50);
}
/*------------------------------------------------------------------------------------------------------------------
 * name: SubscriptionInit
 -----------------------------------------------------------------------------------------------------------------*/
void ROSInterface::SubscriptionInit()
{ 
  /* add more later */
}
/*------------------------------------------------------------------------------------------------------------------
 * name: PubComXstd
 -----------------------------------------------------------------------------------------------------------------*/
void ROSInterface::PubComXstd(const XstdData& data)
{
    xpkg_msgs::XmsgCommData data_com;
    data_com.len = data.len;
    data_com.id_c = data.id_c;
    data_com.id_t = data.id_t;
    data_com.id_n = data.id_n;
    data_com.id_f = data.id_f;
    memcpy(&data_com.data[0],&data.data[0],data.len);
    data_com.time = ros::Time::now();
    pub_com_xstd.publish(data_com);
}
/*------------------------------------------------------------------------------------------------------------------
 * name: PubOdom
 -----------------------------------------------------------------------------------------------------------------*/
void ROSInterface::PubOdom(double lx,double ly,double th,double vx,double vy,double vth)
{
    ros::Time current_time = ros::Time::now();
    static tf::TransformBroadcaster odom_broadcaster;
    //publish tf////////////////////////////
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);
    geometry_msgs::TransformStamped odom_tf;
    odom_tf.header.stamp = current_time;
    odom_tf.header.frame_id = "odom";
    odom_tf.child_frame_id = "base_link";
    odom_tf.transform.translation.x = lx;
    odom_tf.transform.translation.y = ly;
    odom_tf.transform.translation.z = 0.0;
    odom_tf.transform.rotation = odom_quat;
    odom_broadcaster.sendTransform(odom_tf);

    //publish odom//////////////////////////
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";
    odom.pose.pose.position.x = lx;
    odom.pose.pose.position.y = ly;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;
    pub_odom.publish(odom);

    //publish path//////////////////////////
    if(m_show_path)
    {
        static nav_msgs::Path path;
        geometry_msgs::PoseStamped this_pose_stamped;
        this_pose_stamped.pose.position.x = odom.pose.pose.position.x;
        this_pose_stamped.pose.position.y = odom.pose.pose.position.y;
        this_pose_stamped.pose.orientation = odom.pose.pose.orientation;
        this_pose_stamped.header.stamp = current_time;
        this_pose_stamped.header.frame_id = "odom";
        path.poses.push_back(this_pose_stamped);
        if(path.poses.size() > 1000) path.poses.erase(path.poses.begin());

        path.header.stamp = ros::Time::now();
        path.header.frame_id="odom";
        pub_path.publish(path);
    }
} 
/*------------------------------------------------------------------------------------------------------------------
 * name: PubDevList
 -----------------------------------------------------------------------------------------------------------------*/
void ROSInterface::PubDevState(const std::string& data)
{
    std_msgs::String buff;
    buff.data.append(data);
    pub_device_state.publish(buff);
}
/*------------------------------------------------------------------------------------------------------------------
 * name: sub callback
 -----------------------------------------------------------------------------------------------------------------*/
void ROSInterface::ComXstdCallback(const xpkg_msgs::XmsgCommData& data)
{
  m_f_com_xstd = true;
  XstdData data_com;
  data_com.len = data.len;
  data_com.id_c = data.id_c;
  data_com.id_t = data.id_t;
  data_com.id_n = data.id_n;
  data_com.id_f = data.id_f;
  memcpy(&data_com.data[0],&data.data[0],data.len);
  data_com.time = data.time.toSec();
  m_list_com_xstd.push_back(data_com);
  if(m_list_com_xstd.size()>500)m_list_com_xstd.clear();
}
///////////////////////////////////////////////////////////////////////////////////////////////
void ROSInterface::CustomVelCallback(const geometry_msgs::Twist& cmd_vel)
{
  m_f_vel = true;
  VelData data_vel;
  data_vel.line_x = cmd_vel.linear.x;
  data_vel.line_y = cmd_vel.linear.y;
  data_vel.line_z = cmd_vel.linear.z;
  data_vel.ang_x = cmd_vel.angular.x;
  data_vel.ang_y = cmd_vel.angular.y;
  data_vel.ang_z = cmd_vel.angular.z;
  m_list_vel.push_back(data_vel);
  if(m_list_vel.size()>500)m_list_vel.clear();
}

}//namespace HEXROS

//////////////////////////////////////////////////
//xnode for communication V2.0
//xpkg_comm
//////////////////////////////////////////////////
#include <comm_func.h>
#include <ros_interface.h>

using namespace XROS_COMM;

void TimeCallback() {
  CommFUNC& comm_func = CommFUNC::GetCommFUNC();
  ROSInterface& ros_interface = ROSInterface::GetInterface();
  if(ros_interface.m_tcp_en)
  {
      comm_func.TcpInit();     //reboot
      comm_func.TcpSendData();
      comm_func.TcpRecvData();
  }
  if(ros_interface.m_com_en)
  {
      comm_func.ComInit();      //reboot
      comm_func.ComSendData();
      comm_func.ComRecvData();
  }
}
/*------------------------------------------------------------------------------------------------------------------
 * name: main
 -----------------------------------------------------------------------------------------------------------------*/
int main(int argc, char **argv)
{
    //system("gnome-terminal -x bash -c 'source /opt/ros/melodic/setup.bash;roscore'&");
    ROSInterface& ros_interface = ROSInterface::GetInterface();
    ros_interface.BaseInit(argc, argv, "xnode_comm", 10.0, TimeCallback);

    CommFUNC& comm_func = CommFUNC::GetCommFUNC();
    if(comm_func.BaseInit() == false)return 0;

    ros_interface.Work();
    return 0;
}

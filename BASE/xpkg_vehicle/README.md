##ROS driver for vehicle

How to use vehicle ros driver
=====================================================================
        1) Clone this ros package folder(xpkg_vehicle) to your catkin's workspace src folder
        2) Run catkin_make to build
        3) Add path of setup.bash to ~/.bashrc like: echo "source ~/workspace/devel/setup.bash">> ~/.bashrc	source ~/.bashrc
        4) Run script_vehicle.sh(xpkg_vehicle/scripts/script_vehicle.sh) to install 
        5) Change parameters in xnode_vehicle.launch(xpkg_vehicle/launch/xnode_vehicle.launch)
        6) prepare xpkg_comm and Roslaunch xnode_comm.launch(xpkg_comm/launch/xnode_comm.launch)
        7) Roslaunch xnode_vehicle.launch(xpkg_vehicle/launch/xnode_vehicle.launch)

Note
=====================================================================
        1) If open mode_lock and mode is right,ros will print out:"xnode_vehicle: mode OK"
        2) When vehicle enable successfully,ros will print out:"xnode_vehicle: Vehicle enable"
        3) Don't Change parameter of "ini_path"
        4) This node will publish message "/odom","/xtopic_comm/com_send_xstd" and "/xtopic_vehicle/device_state_json"
        5) Must run with xpkg_comm
        6) Odom calculate mode has better precision than speed mode

Parameter
=====================================================================
        1) rate_x/y/z/az :      (0.0 to 1.0) rate of speed x,y,z and angular z
        2) show_loc :           (false or true) show location calculated from odom or speed
        3) show_path :          (false or true) show path in rviz
        4) calc_speed :         (false or true) use speed or odom calculate mode
        5) mode_lock :          (false or true)if must work on CAN mode
        6) ini_path :           DON'T CHANGE

json out list(/xtopic_vehicle/device_state_json)
=====================================================================
        work_state : 		0=fine 1=error
        beep_state : 		0=off 1=on
        brake_state :		0=off 1=on 
        work_mode : 		0=standby 1=remote 2=CAN 3=free
        spec_func_state :       0=off 1=on, cheak datasheet for special function details
        battery_vol : 		(float) voltage of battery
        err_motor :             (0=fine 1=error) motor error(over current or over heat)
        err_driver :            (0=fine 1=error) driver error
        err_driver_offline :	(0=fine 1=error) driver offline
        err_bat_down :		(0=fine 1=error) battery shutdown (<5%)
        err_bat_low :		(0=fine 1=error) battery voltage low(<15%)
        err_ctrl_lost :		(0=fine 1=error) lost controller
        err_bump :		(0=fine 1=error) bumper touch
        speed_x :		(double m/s) speed of x axis
        speed_y :		(double m/s) speed of y axis
        speed_r :		(double rad/s) speed of rotate axis
        odom_left_main :	(double m) odom of left front wheel
        odom_right_main :	(double m) speed of right front wheel
        odom_left_second :	(double m) speed of left rear wheel
        odmo_right_second :	(double m) speed of right rear wheel

note: please use include/LIB_JSON/ArduinoJson.h,view https://arduinojson.org for details

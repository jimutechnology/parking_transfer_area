Date: 03/18/2019

++++++++++++++ launch自启动配置说明 ++++++++++++++++++++++++++++++++++++++++++++
1. area_launch
    1) 根据机器人情况，修改配置ROS_IP/ROS_MASTER_URI/AREA_ID
    2) 修改要启动的launch文件名，如 roslaunch mecanum_robot start_robot_B3.launch
2. 执行配置脚本 sudo ./config_selfstart
3. reboot TX1


自启动launch操作相关命令
停止launch       sudo systemctl stop area_launch
重启launch       sudo systemctl restart area_launch
关闭自启动功能     sudo systemctl disable area_launch
打开自启动功能     sudo systemctl enable area_launch

保证bashrc文件中的配置与 area_launch 文件相同。启动后可以使用ros相关命令查看机器人运行状况
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++



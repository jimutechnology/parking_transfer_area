Date: 08/20/2020

++++++++++++++ launch自启动配置说明 ++++++++++++++++++++++++++++++++++++++++++++
1. 修改transfer_sys_init文件
2. 执行配置脚本 sudo ./config_transfer_sys_init
3. reboot TX2


自启动transfer_sys_init操作相关命令
停止transfer_sys_init          sudo systemctl stop transfer_sys_init
重启transfer_sys_init          sudo systemctl restart transfer_sys_init
关闭自启动功能                   sudo systemctl disable transfer_sys_init
打开自启动功能                   sudo systemctl enable transfer_sys_init

+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++



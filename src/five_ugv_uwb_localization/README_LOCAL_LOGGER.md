# UGV offline logger

定位与记录一起启动：

```bash
source /home/wheeltec/formation_ws/scripts/env.sh
roslaunch five_ugv_uwb_localization ugv.launch
```

定位已经运行时，只启动记录器：

```bash
roslaunch five_ugv_uwb_localization offline_logger_only.launch
```

数据写入：

```text
/home/wheeltec/formation_ws/logs/uwb/ugv1_test_YYYYMMDD_HHMMSS/
```

目录中的车辆编号由车端 `UGV_ID` 决定。按 Ctrl+C 后生成 CSV、汇总文件和 PNG。
三台 Melodic 小车的绘图依赖已安装；重新配置时使用：

```bash
sudo apt install python-matplotlib
```

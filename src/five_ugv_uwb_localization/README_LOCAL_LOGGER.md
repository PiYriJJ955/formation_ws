# UGV1 offline logger

定位与记录一起启动：

```bash
roslaunch five_ugv_uwb_localization ugv1.launch
```

定位已经运行时，只启动记录器：

```bash
roslaunch five_ugv_uwb_localization offline_logger_only.launch
```

数据写入：

```text
/home/wheeltec/wheeltec_robot/uwb_logs/ugv1_test_YYYYMMDD_HHMMSS/
```

按 Ctrl+C 后生成 CSV、汇总文件和 PNG。绘图依赖：

```bash
sudo apt install python3-matplotlib
```

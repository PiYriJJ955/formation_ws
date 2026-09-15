# 独立 IOT 角度拟合测试

`scripts/iot_calibration.py` 是与编队控制台分开的标定工具。它读取主软件的
`settings.json`，因此车辆 IP、`ugvN` 编号和 `iot_ports` 不需要再复制一份；
ugv3 的左右板按实时 UID 分成两个模块。UID 仍以 `nlink_parser/IotFrame0.uid`
为准，启动时会由车端协议检查，不接受 launch 参数猜测 UID。

启动：

```bash
python3 scripts/iot_calibration.py
# 指定主软件配置或只检查桌面：
python3 scripts/iot_calibration.py --config /path/to/settings.json --check-gui
```

在窗口中选择来源模块和目标模块。独立标定不限制在实时编队图的 12 条关系内，
任意两个不同 IoT 模块都可以选择（例如 ugv1→ugv4、ugv1→ugv5、
ugv1→ugv3_left、ugv4→ugv2、ugv4→ugv1、ugv4→ugv3_right），
点击“连接 IoT / 新建记录”。来源、目标车辆的 IOT 会通过已有的
`fleet_iot.IotSession` 启动 `nlink_parser`，因此仍会保存车端 `iot.bag`、日志以及
主机 JSONL。连接成功后，对车辆按 `0, 5, …, 50°` 的实际夹角逐组摆放，点击该组
按钮静止采集 60 秒。实际夹角组为 `-60, -55, …, 0, …, 55, 60°`（每 5° 一组）。
每个来源帧都写入 `observations.csv`，目标缺测也保留为空行；
组均值、标准差、来源帧数、目标出现数和缺测率写入 `session.json`。
每组完成后还会立即更新便于表格查看的 `points.csv`；达到三组后生成完整指标
`fit.json`。

拟合结果会显示有向编号：`f23` 表示 ugv2 测得 ugv3 的水平角，`f13` 表示 ugv1
测得 ugv3；来源和目标为同一车辆的双板时会显示带板名的编号，避免歧义。

同一角度可以重测，界面会替换该角度的均值而不删除旧逐帧记录。历史下拉框可再次
打开记录和拟合图；“查看原始逐帧记录”查看 CSV，“导出汇总 CSV”导出均值和拟合值，
“导出完整 ZIP”打包整个记录目录，“复制拟合结果”复制可直接粘贴到 MATLAB 的
`x/y/a/b/r` 文本。记录默认保存到 `logs/iot_calibration/cal_*/`。

拟合采用当前临时公式 `actual_angle_deg = a + b * measured_horizontal_deg`，
`a`、`b`、`r`、`Sy`、`Sq`、`Syhat`、`Sa`、`Sb` 和误差范围与用户给出的 MATLAB
代码一致（包括 `Syhat = sqrt(U/(n-2))`）。至少三组有效角度才显示拟合；建议完成
全部 25 组后再将参数用于独立标定或在线偏置估计。

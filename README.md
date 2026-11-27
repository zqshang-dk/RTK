# RTK 解算项目

本项目包含 RTK 解算源代码、LSQ 与 EKF 结果、绘图脚本及项目报告。

## 目录说明

- [src/](src/)：RTK 源代码。
- [src--linux/](src--linux/)：Linux 相关源代码。
- [ctrl/](ctrl/)：RTK 配置文件所在目录；[RTKConfiglinuxnew.ini](ctrl/RTKConfiglinuxnew.ini)。
- [output/](output/)：LSQ 与 EKF 解算结果。
- [fig/EKF/](fig/EKF/)：EKF 结果绘图脚本。
- [fig/LSQ/](fig/LSQ/)：LSQ 结果绘图脚本。
- [fig/](fig/)：项目流程与程序结构图（PNG）。
- [report/](report/)：[项目报告 PDF](report/报告.pdf)。

## 绘图脚本

[fig/EKF/](fig/EKF/) 和 [fig/LSQ/](fig/LSQ/) 中包含以下类型的绘图脚本：

- ENU 误差时间序列
- 水平与垂直误差
- ENU 误差直方图
- 水平误差散点图
- 精度对比
- 卫星数量
- 固定解比例
- Ratio 时间序列

各目录中的 `main.py` 和 `rtk_common.py` 分别用于组织绘图流程及提供公共绘图功能。
```**
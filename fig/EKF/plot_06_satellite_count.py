
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.ticker import MaxNLocator

from rtk_common import (
    read_rtk_data,
    add_grid,
    save_figure
)


def plot(config):

    # ============================================================
    # 1. 读取数据
    # ============================================================

    df = read_rtk_data(
        config,
        ["Week", "SOW", "GPSNum", "BDSNum", "Status"]
    )

    # ============================================================
    # 2. 相对时间计算：Nov-23 18:00 对应 0 h
    # ============================================================

    start_time = pd.Timestamp(
        config["enu_timeseries_start"]
    )

    end_time = pd.Timestamp(
        config["enu_timeseries_end"]
    )

    # 只保留指定时间段内的数据
    df = df[
        (df["Time"] >= start_time) &
        (df["Time"] <= end_time)
    ].copy()

    # 计算相对于起始时刻的累计小时数
    df["Time_h"] = (
        df["Time"] - start_time
    ).dt.total_seconds() / 3600.0

    if df.empty:
        raise ValueError(
            "指定时间范围内没有有效数据！"
        )

    # ============================================================
    # 3. 创建卫星数量时序图
    # ============================================================

    fig, ax = plt.subplots(
        figsize=config["figsize"]["satellite_count"]
    )

    # ============================================================
    # 4. 绘制 GPS 卫星数量
    # ============================================================

    ax.plot(
        df["Time_h"],
        df["GPSNum"],
        color=config["colors"]["GPS"],
        linewidth=config["line_width"],
        marker=".",
        markersize=config["marker_size"],
        label="GPS",
        alpha=0.85
    )

    # ============================================================
    # 5. 绘制 BDS 卫星数量
    # ============================================================

    ax.plot(
        df["Time_h"],
        df["BDSNum"],
        color=config["colors"]["BDS"],
        linewidth=config["line_width"],
        marker=".",
        markersize=config["marker_size"],
        label="BDS",
        alpha=0.85
    )

    # ============================================================
    # 6. 设置纵坐标
    # ============================================================

    ax.set_ylabel(
        "Number of Satellites",
        fontsize=config["label_size"]
    )

    # 纵坐标只显示整数
    ax.yaxis.set_major_locator(
        MaxNLocator(integer=True)
    )

    # ============================================================
    # 7. 设置横坐标：Time (h)
    # ============================================================

    # 横坐标范围
    ax.set_xlim(0, 24)

    # 设置刻度位置
    xticks = config["enu_timeseries_xticks_h"]

    ax.set_xticks(xticks)

    # 0 h 不显示标签，其余刻度显示数字
    xtick_labels = [
        "" if x == 0 else str(x)
        for x in xticks
    ]

    ax.set_xticklabels(
        xtick_labels,
        fontsize=config["tick_size"]
    )

    # 横坐标名称
    ax.set_xlabel(
        "Time (h)",
        fontsize=config["label_size"]
    )

    # ============================================================
    # 8. 统一图例：参考 Plot 03
    # ============================================================

    handles, labels = ax.get_legend_handles_labels()

    fig.legend(
        handles,
        labels,
        loc="upper center",
        bbox_to_anchor=(0.5, 0.995),
        ncol=2,
        fontsize=config["legend_size"],
        frameon=False
    )

    # ============================================================
    # 9. 网格设置
    # ============================================================

    add_grid(ax, config)

    # ============================================================
    # 10. 调整整体布局
    # ============================================================

    fig.tight_layout(
        rect=[0, 0, 1, 0.92]
    )

    # ============================================================
    # 11. 保存图片
    # ============================================================

    path = save_figure(
        fig,
        config,
        "06_GPS_BDS_Satellite_Count.png"
    )

    print(f"保存：{path}")


if __name__ == "__main__":

    from main import CONFIG

    plot(CONFIG)

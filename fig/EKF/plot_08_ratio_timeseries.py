
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

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
        ["Week", "SOW", "Ratio", "Status"]
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

    # 只保留指定时间范围内的数据
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

    # 提取有效 Ratio 数据
    valid = df.dropna(
        subset=["Ratio"]
    ).copy()

    # ============================================================
    # 3. Ratio 数据统计
    # ============================================================

    print("\n========== Ratio 数据统计 ==========")

    print(f"指定时间范围内总历元数：{len(df)}")
    print(f"有效 Ratio 历元数：{len(valid)}")

    for status in ["FIXED", "FLOAT", "OTHER"]:

        sub = valid[
            valid["StatusGroup"] == status
        ]

        if sub.empty:
            continue

        print(
            f"{status}: "
            f"N={len(sub)}, "
            f"Mean={sub['Ratio'].mean():.4f}, "
            f"Median={sub['Ratio'].median():.4f}, "
            f"Min={sub['Ratio'].min():.4f}, "
            f"Max={sub['Ratio'].max():.4f}"
        )

    # ============================================================
    # 4. 创建 Ratio 时序散点图
    # ============================================================

    fig, ax = plt.subplots(
        figsize=config["figsize"]["ratio_timeseries"]
    )

    # ============================================================
    # 5. 绘制 FIXED、FLOAT、OTHER 散点
    # ============================================================

    for status in ["FIXED", "FLOAT", "OTHER"]:

        sub = valid[
            valid["StatusGroup"] == status
        ].dropna(
            subset=["Time_h", "Ratio"]
        )

        if sub.empty:
            continue

        ax.scatter(
            sub["Time_h"],
            sub["Ratio"],
            s=config["marker_size"] ,
            color=config["colors"][status],
            label=status,
            alpha=config["scatter_alpha"],
            linewidths=0,
            rasterized=True
        )

    # ============================================================
    # 6. 绘制 Ratio 阈值参考线
    # ============================================================

    threshold = config.get("ratio_threshold")

    if threshold is not None:

        ax.axhline(
            float(threshold),
            color=config["colors"]["ZERO"],
            linestyle="--",
            linewidth=config["reference_line_width"],
            label=f"Ratio Threshold ({threshold})",
            zorder=1
        )

    # ============================================================
    # 7. 设置纵坐标
    # ============================================================

    ax.set_ylabel(
        "Ambiguity Ratio",
        fontsize=config["label_size"]
    )

    # ============================================================
    # 8. 设置横坐标：Time (h)
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
    # 9. 统一图例：参考 Plot 02
    # ============================================================

    handles, labels = ax.get_legend_handles_labels()

    if handles:

        fig.legend(
            handles,
            labels,
            loc="upper center",
            bbox_to_anchor=(0.5, 0.995),
            ncol=len(labels),
            fontsize=config["legend_size"],
            frameon=False,
            markerscale=1.5
        )

    # ============================================================
    # 10. 网格设置
    # ============================================================

    add_grid(ax, config)

    # ============================================================
    # 11. 调整整体布局
    # ============================================================

    fig.tight_layout(
        rect=[0, 0, 1, 0.92]
    )

    # ============================================================
    # 12. 保存图片
    # ============================================================

    path = save_figure(
        fig,
        config,
        "08_Ambiguity_Ratio_TimeSeries.png"
    )

    print(f"保存：{path}")


if __name__ == "__main__":

    from main import CONFIG

    plot(CONFIG)

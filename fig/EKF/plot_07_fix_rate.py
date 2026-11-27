
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.patches import Patch

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
        ["Week", "SOW", "Status"]
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

    if end_time <= start_time:
        raise ValueError("结束时间必须晚于起始时间！")

    df = df.dropna(subset=["Time"]).copy()

    # 只保留指定时间范围内的数据
    df = df[
        (df["Time"] >= start_time) &
        (df["Time"] <= end_time)
    ].copy()

    if df.empty:
        raise ValueError("指定时间范围内没有有效数据！")

    # 计算相对时间（小时）
    df["Time_h"] = (
        df["Time"] - start_time
    ).dt.total_seconds() / 3600.0

    duration_hours = (
        end_time - start_time
    ).total_seconds() / 3600.0

    # ============================================================
    # 3. 解状态分类
    # ============================================================

    df["StatusGroup"] = df["StatusGroup"].where(
        df["StatusGroup"].isin(["FIXED", "FLOAT"]),
        "OTHER"
    )

    n_total = len(df)

    n_fixed = int(
        (df["StatusGroup"] == "FIXED").sum()
    )

    n_float = int(
        (df["StatusGroup"] == "FLOAT").sum()
    )

    n_other = int(
        (df["StatusGroup"] == "OTHER").sum()
    )

    # RTK 有效解历元数
    n_valid = n_fixed + n_float

    # 总体固定解比例
    overall_rate = (
        n_fixed / n_valid * 100
        if n_valid > 0 else np.nan
    )

    # 固定解总体可用率
    availability = (
        n_fixed / n_total * 100
        if n_total > 0 else np.nan
    )

    print("\n========== 解状态统计 ==========")

    print(f"总历元数：{n_total}")
    print(f"FIXED：{n_fixed}")
    print(f"FLOAT：{n_float}")
    print(f"OTHER：{n_other}")
    print(f"RTK有效解历元数：{n_valid}")

    print(
        f"总体固定解比例：{overall_rate:.2f}%"
        if np.isfinite(overall_rate)
        else "总体固定解比例：N/A"
    )

    # print(
    #     f"固定解总体可用率：{availability:.2f}%"
    #     if np.isfinite(availability)
    #     else "固定解总体可用率：N/A"
    # )

    # ============================================================
    # 4. 创建上下两个子图
    # ============================================================

    fig, (ax1, ax2) = plt.subplots(
        2, 1,
        figsize=config["figsize"]["fix_rate"],
        sharex=False,
        gridspec_kw={
            "height_ratios": [1.15, 1.0],
            "hspace": 0.35
        }
    )

    # ============================================================
    # 5. 上面子图：逐历元解状态时序散点图
    # ============================================================

    # 只保留 FIXED 和 FLOAT 两行
    state_y = {
        "FIXED": 0,
        "FLOAT": 1
    }

    # 从底层到顶层绘制
    for status in ["FLOAT", "FIXED"]:

        sub = df[
            df["StatusGroup"] == status
        ]

        if sub.empty:
            continue

        ax1.scatter(
            sub["Time_h"],
            np.full(len(sub), state_y[status]),
            s=12,
            marker=".",
            color=config["colors"][status],
            label=status,
            alpha=config["scatter_alpha"],
            linewidths=0,
            rasterized=True,
            zorder={
                "FLOAT": 2,
                "FIXED": 3
            }[status]
        )

    # 纵坐标设置：只保留 FIXED 和 FLOAT
    ax1.set_yticks([0, 1])

    ax1.set_yticklabels(
        ["FIXED", "FLOAT"],
        fontsize=config["tick_size"]
    )

    ax1.set_ylim(-0.5, 1.5)

    ax1.set_ylabel(
        "Solution Status",
        fontsize=config["label_size"]
    )

    # ============================================================
    # 6. 上面子图：横坐标设置，参考 Plot 02
    # ============================================================

    ax1.set_xlim(0, duration_hours)

    # 使用 main.py 中统一的时间刻度配置
    xticks = config["enu_timeseries_xticks_h"]

    ax1.set_xticks(xticks)

    # 0 h 不显示标签，其余显示数字
    xtick_labels = [
        "" if x == 0 else str(x)
        for x in xticks
    ]

    ax1.set_xticklabels(
        xtick_labels,
        fontsize=config["tick_size"]
    )

    # 上面子图也显示横坐标名称
    ax1.set_xlabel(
        "Time (h)",
        fontsize=config["label_size"]
    )

    # ============================================================
    # 7. 上面子图：右上角固定解统计标注
    # ============================================================
    # ============================================================
    # 上面子图：左上角分别标注 FIXED / FLOAT 数量
    # ============================================================

    # FIXED 数量：蓝色
    ax1.text(
        0.015, 0.96,
        f"FIXED: {n_fixed:,}",
        transform=ax1.transAxes,
        ha="left",
        va="top",
        fontsize=config["annotation_size"],
        color=config["colors"]["FLOAT"]
    )

    # FLOAT 数量：红色
    ax1.text(
        0.22, 0.96,
        f"FLOAT: {n_float:,}",
        transform=ax1.transAxes,
        ha="left",
        va="top",
        fontsize=config["annotation_size"],
        color=config["colors"]["FIXED"]
    )

    # ============================================================
    # 右上角：总体固定解比例
    # ============================================================

    overall_text = (
        f"Overall Fix Rate = {overall_rate:.2f}%"
        if np.isfinite(overall_rate)
        else "Overall Fix Rate = N/A"
    )

    ax1.text(
        0.985, 0.96,
        overall_text,
        transform=ax1.transAxes,
        ha="right",
        va="top",
        fontsize=config["annotation_size"],
        color="black"
    )

    # ax1.text(
    #     0.985, 0.96,
    #     statistics_text,
    #     transform=ax1.transAxes,
    #     ha="right",
    #     va="top",
    #     fontsize=config["annotation_size"],
    #     color="black",
    #     bbox={
    #         "facecolor": "white",
    #         "edgecolor": "none",
    #         "alpha": 0.88,
    #         "pad": 5
    #     },
    #     zorder=10
    # )
    add_grid(ax1, config, axis="x")

    # ============================================================
    # 8. 下面子图：每4小时统计一次
    # ============================================================

    interval_hours = 4.0

    # 生成时间分段边界
    bin_edges = np.arange(
        0,
        duration_hours,
        interval_hours
    )

    if len(bin_edges) == 0 or not np.isclose(bin_edges[0], 0):
        bin_edges = np.insert(bin_edges, 0, 0)

    if not np.isclose(bin_edges[-1], duration_hours):
        bin_edges = np.append(
            bin_edges,
            duration_hours
        )

    n_bins = len(bin_edges) - 1

    # 时间段中心位置
    bin_centers = (
        bin_edges[:-1] + bin_edges[1:]
    ) / 2

    # 时间段标签
    bin_labels = [
        f"{left:g}–{right:g}"
        for left, right in zip(
            bin_edges[:-1],
            bin_edges[1:]
        )
    ]

    # 每根柱子的宽度
    bar_widths = (
        bin_edges[1:] - bin_edges[:-1]
    ) * 0.78

    # 将历元分配到对应时间段
    df["BinIndex"] = np.floor(
        df["Time_h"] / interval_hours
    ).astype(int)

    df["BinIndex"] = df["BinIndex"].clip(
        lower=0,
        upper=n_bins - 1
    )

    # ============================================================
    # 9. 统计每个时间段的解状态比例
    # ============================================================

    percentages = {
        "FIXED": np.full(n_bins, np.nan),
        "FLOAT": np.full(n_bins, np.nan),
        "OTHER": np.full(n_bins, np.nan)
    }

    bin_counts = np.zeros(n_bins, dtype=int)

    bin_fix_rates = np.full(n_bins, np.nan)

    print("\n========== 分时段解状态统计 ==========")

    for i in range(n_bins):

        sub = df[
            df["BinIndex"] == i
        ]

        total = len(sub)

        bin_counts[i] = total

        if total == 0:
            print(f"{bin_labels[i]} h：无有效数据")
            continue

        fixed_count = int(
            (sub["StatusGroup"] == "FIXED").sum()
        )

        float_count = int(
            (sub["StatusGroup"] == "FLOAT").sum()
        )

        other_count = int(
            (sub["StatusGroup"] == "OTHER").sum()
        )

        # 各状态占该时间段全部历元的比例
        percentages["FIXED"][i] = (
            fixed_count / total * 100
        )

        percentages["FLOAT"][i] = (
            float_count / total * 100
        )

        percentages["OTHER"][i] = (
            other_count / total * 100
        )

        # 各时间段固定解比例
        valid_count = fixed_count + float_count

        if valid_count > 0:

            bin_fix_rates[i] = (
                fixed_count / valid_count * 100
            )

        rate_text = (
            f"{bin_fix_rates[i]:.2f}%"
            if np.isfinite(bin_fix_rates[i])
            else "N/A"
        )

        print(
            f"{bin_labels[i]} h: "
            f"N={total}, "
            f"FIXED={fixed_count}, "
            f"FLOAT={float_count}, "
            f"OTHER={other_count}, "
            f"Fix Rate={rate_text}"
        )

    # ============================================================
    # 10. 绘制下面子图：100%堆叠柱状图
    # ============================================================

    status_counts = {
        "FIXED": n_fixed,
        "FLOAT": n_float,
        "OTHER": n_other
    }

    # 只绘制实际存在的状态
    bar_statuses = [
        status
        for status in ["FIXED", "FLOAT", "OTHER"]
        if status_counts[status] > 0
    ]

    bottom = np.zeros(n_bins)

    for status in bar_statuses:

        values = np.nan_to_num(
            percentages[status],
            nan=0.0
        )

        ax2.bar(
            bin_centers,
            values,
            width=bar_widths,
            bottom=bottom,
            color=config["colors"][status],
            alpha=0.85,
            edgecolor="white",
            linewidth=0.6,
            label=status,
            zorder=3
        )

        bottom += values

    # ============================================================
    # 11. 每根柱子上方标注固定解比例
    # ============================================================

    for i, center in enumerate(bin_centers):

        if bin_counts[i] == 0:

            label = "No data"

        elif np.isfinite(bin_fix_rates[i]):

            label = (
                f"Fix Rate: "
                f"{bin_fix_rates[i]:.1f}%"
            )

        else:

            label = "Fix Rate: N/A"

        ax2.text(
            center,
            102,
            label,
            ha="center",
            va="bottom",
            fontsize=config["annotation_size"] - 1,
            color="black"
        )

    # ============================================================
    # 12. 下面子图：坐标轴设置
    # ============================================================

    ax2.set_ylim(0, 115)

    ax2.set_yticks(
        np.arange(0, 101, 20)
    )

    ax2.set_ylabel(
        "Epoch Distribution (%)",
        fontsize=config["label_size"]
    )

    ax2.set_xlabel(
        "Time (h)",
        fontsize=config["label_size"]
    )

    # 横坐标显示4小时区间
    ax2.set_xticks(bin_centers)

    ax2.set_xticklabels(
        bin_labels,
        fontsize=config["tick_size"]
    )

    ax2.set_xlim(0, duration_hours)

    add_grid(ax2, config, axis="y")

    # ============================================================
    # 13. 统一图例：顶部居中，无边框
    # ============================================================

    # 图例只显示 FIXED 和 FLOAT，不显示 OTHER
    legend_statuses = [
        status for status in ["FIXED", "FLOAT"]
        if status in bar_statuses
    ]

    legend_handles = [
        Patch(
            facecolor=config["colors"][status],
            edgecolor=config["colors"][status],
            label=status
        )
        for status in legend_statuses
    ]
    if legend_handles:

        fig.legend(
            handles=legend_handles,
            loc="upper center",
            bbox_to_anchor=(0.5, 0.995),
            ncol=len(legend_handles),
            fontsize=config["legend_size"],
            frameon=False
        )

    # ============================================================
    # 14. 整体布局
    # ============================================================

    fig.tight_layout(
        rect=[0, 0, 1, 0.92],
        h_pad=2.0
    )

    # ============================================================
    # 15. 保存图片
    # ============================================================

    path = save_figure(
        fig,
        config,
        "07_Solution_Status_Fix_Rate.png"
    )

    print(f"\n保存：{path}")


if __name__ == "__main__":

    from main import CONFIG

    plot(CONFIG)

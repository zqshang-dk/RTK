import matplotlib.pyplot as plt
from rtk_common import read_rtk_data, add_enu_errors, status_scatter, add_grid, set_time_axis, save_figure
import pandas as pd

def plot(config):
    df = read_rtk_data(config, ["Week", "SOW", "dX", "dY", "dZ", "Status"])
    df, _, _, _ = add_enu_errors(df, config)
    
    # ============================================================
    # 相对时间计算：Nov-23 18:00 对应 0 h
    # ============================================================

    start_time = pd.Timestamp(config["enu_timeseries_start"])
    end_time = pd.Timestamp(config["enu_timeseries_end"])

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
        raise ValueError("指定时间范围内没有有效数据！")

    fig, axes = plt.subplots(
        3, 1,
        figsize=config["figsize"]["enu_timeseries"],
        sharex=True
    )

    components = [
        ("eE", "East Error (m)"),
        ("eN", "North Error (m)"),
        ("eU", "Up Error (m)"),
    ]

    for ax, (col, ylabel) in zip(axes, components):
        # 按照 FIXED、FLOAT、OTHER 分别绘制散点
        for status in ["FIXED", "FLOAT", "OTHER"]:

            sub = df[
                df["StatusGroup"] == status
            ].dropna(subset=["Time_h", col])

            if sub.empty:
                continue

            ax.scatter(
                sub["Time_h"],
                sub[col],
                s=config["marker_size"]*2,
                marker=".",
                color=config["colors"][status],
                label=status,
                alpha=config["scatter_alpha"],
                linewidths=0,
                rasterized=True
            )
        ax.axhline(0, color=config["colors"]["ZERO"],
                   linestyle="--", linewidth=config["reference_line_width"])
        ax.set_ylabel(ylabel, fontsize=config["label_size"])
        ax.set_ylim(-5, 5)
        add_grid(ax, config, axis="x")
    
    # 获取第一个子图中的图例句柄
    handles, labels = axes[0].get_legend_handles_labels()

    # 创建整个 Figure 共用的图例
    fig.legend(
        handles,
        labels,
        loc="upper center",
        bbox_to_anchor=(0.5, 0.995),
        ncol=3,
        fontsize=config["legend_size"],
        frameon=False,
        markerscale=5
    )
    # ============================================================
    # 横坐标设置：Time (h)
    # ============================================================

    # 横坐标范围
    axes[-1].set_xlim(0, 24)

    # 设置刻度位置
    xticks = config["enu_timeseries_xticks_h"]

    axes[-1].set_xticks(xticks)

    # 0 h 不显示标签，其余刻度显示数字
    xtick_labels = [
        "" if x == 0 else str(x)
        for x in xticks
    ]

    axes[-1].set_xticklabels(
        xtick_labels,
        fontsize=config["tick_size"]
    )

    # 只有最下面的子图显示横坐标标题
    axes[-1].set_xlabel(
        "Time (h)",
        fontsize=config["label_size"]
    )

    # 上面两个子图隐藏横坐标刻度标签
    for ax in axes[:-1]:
        ax.tick_params(
            axis="x",
            labelbottom=False
        )

    fig.tight_layout(rect=[0, 0, 1, 0.96])

    path = save_figure(fig, config, "01_ENU_Error_TimeSeries.png")
    print(f"保存：{path}")


if __name__ == "__main__":
    from main import CONFIG
    plot(CONFIG)

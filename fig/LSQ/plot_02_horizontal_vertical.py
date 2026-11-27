import matplotlib.pyplot as plt
import pandas as pd
from rtk_common import read_rtk_data, add_enu_errors, add_grid, save_figure


def plot(config):
    df = read_rtk_data(config, ["Week", "SOW", "dX", "dY", "dZ", "Status"])
    df, _, _, _ = add_enu_errors(df, config)

    # ============================================================
    # 相对时间计算：Nov-23 18:00 对应 0 h
    # ============================================================

    start_time = pd.Timestamp(
        config.get("enu_timeseries_start", "2026-11-23 18:00:00")
    )
    end_time = pd.Timestamp(
        config.get("enu_timeseries_end", "2026-11-24 18:00:00")
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
        raise ValueError("指定时间范围内没有有效数据！")

    fig, axes = plt.subplots(
        2, 1,
        figsize=config["figsize"]["horizontal_vertical"],
        sharex=True
    )

    components = [
        ("EH", "Horizontal Error (m)"),
        ("EV", "Vertical Error (m)"),
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
                s=config["marker_size"],
                marker=".",
                color=config["colors"][status],
                label=status,
                alpha=config["scatter_alpha"],
                linewidths=0,
                rasterized=True
            )

        ax.set_ylabel(ylabel, fontsize=config["label_size"])
        ax.set_ylim(-0.5, 5)
        add_grid(ax, config)

    # ============================================================
    # 统一图例：只保留一个，放在图外最上方中间
    # ============================================================

    handles, labels = axes[0].get_legend_handles_labels()

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
    xticks = config.get("enu_timeseries_xticks_h", [0, 4, 8, 12, 16, 24])
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

    # 上面那张图隐藏横坐标刻度标签
    for ax in axes[:-1]:
        ax.tick_params(
            axis="x",
            labelbottom=False
        )

    fig.tight_layout(rect=[0, 0, 1, 0.94])

    path = save_figure(fig, config, "02_Horizontal_Vertical_Error.png")
    print(f"保存：{path}")


if __name__ == "__main__":
    from main import CONFIG
    plot(CONFIG)
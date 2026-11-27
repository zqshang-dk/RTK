
import numpy as np
import matplotlib.pyplot as plt

from rtk_common import (
    read_rtk_data,
    add_enu_errors,
    add_grid,
    save_figure
)


def plot(config):

    # ============================================================
    # 1. 读取数据并计算 ENU 误差
    # ============================================================

    df = read_rtk_data(
        config,
        ["Week", "SOW", "dX", "dY", "dZ", "Status"]
    )

    df, _, _, _ = add_enu_errors(df, config)

    # 仅保留有效的 East 和 North 误差
    df = df.dropna(subset=["eE", "eN"])

    # ============================================================
    # 2. 创建水平误差空间散点图
    # ============================================================

    fig, ax = plt.subplots(
        figsize=config["figsize"]["horizontal_scatter"]
    )

    # ============================================================
    # 3. 按照绘图层级绘制不同解状态
    # ============================================================

    # 从底层到顶层：OTHER -> FLOAT -> FIXED
    # 保证红色 FIXED 散点始终显示在蓝色 FLOAT 散点上方

    zorder_map = {
        "OTHER": 1,
        "FLOAT": 2,
        "FIXED": 3
    }

    for status in ["OTHER", "FLOAT", "FIXED"]:

        sub = df[
            df["StatusGroup"] == status
        ]

        if sub.empty:
            continue

        ax.scatter(
            sub["eE"],
            sub["eN"],
            s=config["marker_size"] * 1.5,
            color=config["colors"][status],
            label=status,
            alpha=config["scatter_alpha"],
            linewidths=0,
            rasterized=True,
            zorder=zorder_map[status]
        )

    # ============================================================
    # 4. 绘制零误差参考线
    # ============================================================

    ax.axhline(
        0,
        color=config["colors"]["ZERO"],
        linestyle="--",
        linewidth=config["reference_line_width"],
        zorder=0
    )

    ax.axvline(
        0,
        color=config["colors"]["ZERO"],
        linestyle="--",
        linewidth=config["reference_line_width"],
        zorder=0
    )

    # ============================================================
    # 5. 设置坐标轴范围
    # ============================================================

    # East：-2 ~ +2 m
    ax.set_xlim(-0.2, 0.2)

    # North：-2 ~ +2 m
    ax.set_ylim(-0.2, 0.2)

    # 横纵坐标等比例
    ax.set_aspect("equal", adjustable="box")

    # 坐标轴名称
    ax.set_xlabel(
        "East Error (m)",
        fontsize=config["label_size"]
    )

    ax.set_ylabel(
        "North Error (m)",
        fontsize=config["label_size"]
    )

    # ============================================================
    # 6. 统一图例：参考 Plot 03 的写法
    # ============================================================

    handles, labels = ax.get_legend_handles_labels()

    # 按 FIXED、FLOAT、OTHER 的顺序排列图例
    handle_map = dict(zip(labels, handles))

    legend_order = [
        status for status in ["FIXED", "FLOAT", "OTHER"]
        if status in handle_map
    ]

    legend_handles = [
        handle_map[status] for status in legend_order
    ]

    if legend_handles:

        fig.legend(
            legend_handles,
            legend_order,
            loc="upper center",
            bbox_to_anchor=(0.5, 0.995),
            ncol=3,
            fontsize=config["legend_size"],
            frameon=False,
            markerscale=2
        )

    # ============================================================
    # 7. 网格设置
    # ============================================================

    add_grid(ax, config)

    # ============================================================
    # 8. 调整整体布局
    # ============================================================

    fig.tight_layout(
        rect=[0, 0, 1, 0.95]
    )

    # ============================================================
    # 9. 保存图片
    # ============================================================

    path = save_figure(
        fig,
        config,
        "04_Horizontal_Error_Scatter.png"
    )

    print(f"保存：{path}")


if __name__ == "__main__":

    from main import CONFIG

    plot(CONFIG)

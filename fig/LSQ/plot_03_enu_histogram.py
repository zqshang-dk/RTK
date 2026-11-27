
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

    # ============================================================
    # 2. 创建三个横向排列的子图
    # ============================================================

    fig, axes = plt.subplots(
        1, 3,
        figsize=config["figsize"]["histogram"]
    )

    components = [
        ("eE", "East Error (m)"),
        ("eN", "North Error (m)"),
        ("eU", "Up Error (m)"),
    ]

    # ============================================================
    # 3. 分别绘制 East、North、Up 误差直方图
    # ============================================================

    for ax, (col, xlabel) in zip(axes, components):

        # --------------------------------------------------------
        # 3.1 提取 FIXED 和 FLOAT 数据
        # --------------------------------------------------------

        fixed = df.loc[
            df["StatusGroup"] == "FIXED", col
        ].dropna().to_numpy()

        float_data = df.loc[
            df["StatusGroup"] == "FLOAT", col
        ].dropna().to_numpy()

        arrays = [
            arr for arr in [fixed, float_data]
            if len(arr) > 0
        ]

        # --------------------------------------------------------
        # 3.2 判断是否存在有效数据
        # --------------------------------------------------------

        if not arrays:

            ax.text(
                0.5, 0.5,
                "No valid data",
                ha="center",
                va="center",
                transform=ax.transAxes,
                fontsize=config["label_size"]
            )

            ax.set_xlabel(
                xlabel,
                fontsize=config["label_size"]
            )

            add_grid(ax, config)

            continue

        # --------------------------------------------------------
        # 3.3 两种状态共用相同的分箱边界
        # --------------------------------------------------------

        all_data = np.concatenate(arrays)

        bins = np.histogram_bin_edges(
            all_data,
            bins=config["histogram_bins"]
        )

        # --------------------------------------------------------
        # 3.4 绘制 FIXED 直方图及均值线
        # --------------------------------------------------------

        if len(fixed) > 0:

            fixed_mean = np.mean(fixed)

            ax.hist(
                fixed,
                bins=bins,
                density=True,
                color=config["colors"]["FIXED"],
                alpha=0.45,
                label="FIXED"
            )

            # FIXED 均值：红色虚线
            ax.axvline(
                fixed_mean,
                color=config["colors"]["FIXED"],
                linestyle="--",
                linewidth=config["line_width"]
            )

        # --------------------------------------------------------
        # 3.5 绘制 FLOAT 直方图及均值线
        # --------------------------------------------------------

        if len(float_data) > 0:

            float_mean = np.mean(float_data)

            ax.hist(
                float_data,
                bins=bins,
                density=True,
                color=config["colors"]["FLOAT"],
                alpha=0.45,
                label="FLOAT"
            )

            # FLOAT 均值：蓝色虚线
            ax.axvline(
                float_mean,
                color=config["colors"]["FLOAT"],
                linestyle="--",
                linewidth=config["line_width"]
            )

        # --------------------------------------------------------
        # 3.6 右上角标注 FIXED 和 FLOAT 的 Mean
        # --------------------------------------------------------

        # FIXED Mean：红色字体
        if len(fixed) > 0:

            ax.text(
                0.97, 0.96,
                f"Mean = {fixed_mean:.3f} m",
                transform=ax.transAxes,
                ha="right",
                va="top",
                color=config["colors"]["FIXED"],
                fontsize=config["legend_size"]
            )

        # FLOAT Mean：蓝色字体
        if len(float_data) > 0:

            ax.text(
                0.97, 0.86,
                f"Mean = {float_mean:.3f} m",
                transform=ax.transAxes,
                ha="right",
                va="top",
                color=config["colors"]["FLOAT"],
                fontsize=config["legend_size"]
            )

        # --------------------------------------------------------
        # 3.7 设置横坐标名称
        # --------------------------------------------------------

        ax.set_xlabel(
            xlabel,
            fontsize=config["label_size"]
        )

        # 网格
        add_grid(ax, config)

    # ============================================================
    # 4. 统一设置三个子图的坐标轴范围
    # ============================================================

    for i, ax in enumerate(axes):

        # 横坐标范围：-5 ~ +5 m
        ax.set_xlim(-2, 2)

        # 纵坐标范围：0 ~ 20
        ax.set_ylim(0, 20)

        # --------------------------------------------------------
        # 只有最左边的子图显示纵坐标名称和刻度数字
        # --------------------------------------------------------

        if i == 0:

            ax.set_ylabel(
                "Probability Density",
                fontsize=config["label_size"]
            )

        else:

            ax.set_ylabel("")

            # 隐藏纵坐标刻度数字
            ax.tick_params(
                axis="y",
                labelleft=False
            )

    # ============================================================
    # 5. 统一图例：只保留一个，放在最上方中间
    # ============================================================

    handles = []
    labels = []
    seen_labels = set()

    # 从三个子图中收集图例，避免某个方向缺少某种解状态时
    # 总图例遗漏对应的状态
    for ax in axes:

        ax_handles, ax_labels = ax.get_legend_handles_labels()

        for handle, label in zip(ax_handles, ax_labels):

            if label not in seen_labels:

                handles.append(handle)
                labels.append(label)
                seen_labels.add(label)

    if handles:

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
    # 6. 调整整体布局
    # ============================================================

    fig.tight_layout(
        rect=[0, 0, 1, 0.92]
    )

    # ============================================================
    # 7. 保存图片
    # ============================================================

    path = save_figure(
        fig,
        config,
        "03_ENU_Error_Histogram.png"
    )

    print(f"保存：{path}")


if __name__ == "__main__":

    from main import CONFIG

    plot(CONFIG)


import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.patches import Patch

from rtk_common import (
    read_rtk_data,
    add_enu_errors,
    calculate_statistics,
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

    # 只分析 East、North、Up 三个方向
    metrics = {
        "East": "eE",
        "North": "eN",
        "Up": "eU",
    }

    statuses = ["FIXED", "FLOAT"]

    # 保存误差数据
    error_data = {}

    # 保存 RMS 数值
    rms_values = {}

    # 保存统计结果
    stats_rows = []

    # ============================================================
    # 2. 计算 FIXED 和 FLOAT 的统计指标
    # ============================================================

    print("\n========== FIXED / FLOAT 精度统计 ==========")

    for status in statuses:

        sub = df[df["StatusGroup"] == status]

        print(f"\n--- {status}，有效历元数：{len(sub)} ---")

        for metric_name, column in metrics.items():

            values = pd.to_numeric(
                sub[column],
                errors="coerce"
            ).dropna().to_numpy()

            error_data[(status, metric_name)] = values

            stats = calculate_statistics(
                pd.Series(values)
            )

            if stats is None:

                rms = np.nan

                print(f"{metric_name}: 无有效数据")

            else:

                rms = stats["RMS_m"]

                print(
                    f"{metric_name}: "
                    f"N={stats['N']}, "
                    f"Bias={stats['Bias_m']:.6f} m, "
                    f"STD={stats['STD_m']:.6f} m, "
                    f"RMS={stats['RMS_m']:.6f} m, "
                    f"MaxAbs={stats['MaxAbs_m']:.6f} m, "
                    f"P95={stats['P95Abs_m']:.6f} m"
                )

                stats_rows.append({
                    "Status": status,
                    "Metric": metric_name,
                    **stats
                })

            rms_values[(status, metric_name)] = rms

    # ============================================================
    # 3. 保存精度统计 CSV
    # ============================================================

    config["output_dir"].mkdir(
        parents=True,
        exist_ok=True
    )

    stats_path = (
        config["output_dir"] /
        "05_Accuracy_Statistics.csv"
    )

    pd.DataFrame(stats_rows).to_csv(
        stats_path,
        index=False,
        encoding="utf-8-sig"
    )

    print(f"\n统计表已保存：{stats_path}")

    # ============================================================
    # 4. 创建上下两个子图
    # ============================================================

    fig, axes = plt.subplots(
        2, 1,
        figsize=config["figsize"]["accuracy_comparison"],
        sharex=True
    )

    # 子图配置：上面 FIXED，下面 FLOAT
    panel_settings = [
        ("FIXED", axes[0], (-0.05, 0.05)),
        ("FLOAT", axes[1], (-5, 5)),
    ]

    direction_positions = {
        "East": 1,
        "North": 2,
        "Up": 3,
    }

    # 小提琴图宽度
    violin_width = 0.48

    # ============================================================
    # 5. 分别绘制 FIXED 和 FLOAT 小提琴图
    # ============================================================

    for status, ax, ylim in panel_settings:

        color = config["colors"][status]

        for metric_name, column in metrics.items():

            position = direction_positions[metric_name]

            values = error_data[(status, metric_name)]

            if len(values) == 0:
                continue

            # ----------------------------------------------------
            # 5.1 绘制小提琴图
            # ----------------------------------------------------

            if len(values) >= 2 and np.ptp(values) > 0:

                violin = ax.violinplot(
                    [values],
                    positions=[position],
                    widths=violin_width,
                    showmeans=False,
                    showmedians=True,
                    showextrema=False,
                    quantiles=[[0.25, 0.75]]
                )

                # 小提琴填充
                for body in violin["bodies"]:

                    body.set_facecolor(color)
                    body.set_edgecolor(color)
                    body.set_alpha(0.42)
                    body.set_linewidth(1.0)

                # 中位数线
                if "cmedians" in violin:

                    violin["cmedians"].set_color("black")
                    violin["cmedians"].set_linewidth(1.3)

                # 四分位数线
                if "cquantiles" in violin:

                    violin["cquantiles"].set_color("black")
                    violin["cquantiles"].set_linewidth(1.0)

            else:

                # 数据量过少或数据完全相同时，使用散点表示
                ax.scatter(
                    np.full(len(values), position),
                    values,
                    color=color,
                    s=config["marker_size"],
                    alpha=0.8,
                    zorder=3
                )

            # ----------------------------------------------------
            # 5.2 绘制 Mean 黑色菱形
            # ----------------------------------------------------

            mean_value = np.mean(values)

            ax.scatter(
                position,
                mean_value,
                marker="D",
                s=22,
                color="black",
                zorder=5
            )

            # ----------------------------------------------------
            # 5.3 在每个方向上方横向标注 RMS
            # ----------------------------------------------------

            rms = rms_values[(status, metric_name)]

            if np.isfinite(rms):

                ax.text(
                    position,
                    0.97,
                    f"RMS={rms:.3f} m",
                    transform=ax.get_xaxis_transform(),
                    ha="center",
                    va="top",
                    color=color,
                    fontsize=config["annotation_size"],
                    rotation=0,
                    clip_on=False
                )

        # ========================================================
        # 5.4 设置当前子图的纵坐标范围
        # ========================================================

        ax.set_ylim(ylim[0], ylim[1])

        # 零误差参考线
        ax.axhline(
            0,
            color=config["colors"]["ZERO"],
            linestyle="--",
            linewidth=config["reference_line_width"],
            zorder=0
        )

        # 纵坐标名称
        ax.set_ylabel(
            "Positioning Error (m)",
            fontsize=config["label_size"]
        )

        # 子图左上角标记解状态
        ax.text(
            0.015,
            0.82,
            status,
            transform=ax.transAxes,
            color=color,
            fontsize=config["title_size"],
            fontweight="bold",
            ha="left",
            va="top"
        )

        # 网格
        add_grid(ax, config, axis="y")

    # ============================================================
    # 6. 横坐标设置
    # ============================================================

    axes[-1].set_xticks([1, 2, 3])

    axes[-1].set_xticklabels(
        ["East", "North", "Up"],
        fontsize=config["tick_size"]
    )

    axes[-1].set_xlim(0.5, 3.5)

    # 上面的 FIXED 子图不显示横坐标标签
    axes[0].tick_params(
        axis="x",
        labelbottom=False
    )

    # ============================================================
    # 7. 统一图例：参考 Plot 03
    # ============================================================

    legend_handles = [
        Patch(
            facecolor=config["colors"]["FIXED"],
            edgecolor=config["colors"]["FIXED"],
            alpha=0.42,
            label="FIXED"
        ),
        Patch(
            facecolor=config["colors"]["FLOAT"],
            edgecolor=config["colors"]["FLOAT"],
            alpha=0.42,
            label="FLOAT"
        ),
    ]

    fig.legend(
        handles=legend_handles,
        loc="upper center",
        bbox_to_anchor=(0.5, 0.995),
        ncol=2,
        fontsize=config["legend_size"],
        frameon=False
    )

    # ============================================================
    # 8. 整体布局
    # ============================================================

    fig.tight_layout(
        rect=[0, 0, 1, 0.93]
    )

    # ============================================================
    # 9. 保存图片
    # ============================================================

    path = save_figure(
        fig,
        config,
        "05_FIXED_FLOAT_Accuracy_Comparison.png"
    )

    print(f"\n保存：{path}")


if __name__ == "__main__":

    from main import CONFIG

    plot(CONFIG)

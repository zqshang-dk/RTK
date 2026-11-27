from pathlib import Path
import importlib
import traceback
import sys
import matplotlib
matplotlib.use("Agg")  # 批量绘图，不弹出8个交互窗口

# ============================================================
# 0. 添加当前脚本所在目录到 Python 模块搜索路径
# ============================================================

BASE_DIR = Path(__file__).resolve().parent

if str(BASE_DIR) not in sys.path:
    sys.path.insert(0, str(BASE_DIR))

# ============================================================
# 1. 全局路径与绘图配置：所有字体、字号、颜色、线宽集中在这里
# ============================================================

DATA_PATH = Path(r"D:\Course\RTK\output\EKF.txt")
OUTPUT_DIR = Path(r"D:\Course\RTK\output\EKF")

CONFIG = {
    # 数据与输出
    "data_path": DATA_PATH,
    "output_dir": OUTPUT_DIR,

    # 参考基线向量（ECEF），单位 m
    "reference_baseline_ecef": (-469.568, 702.675, -1405.092),

    # 基准站 ECEF 坐标，单位 m
    "base_station_ecef": (-2267335.1970, 5008648.3409, 3222378.2018),

    # 字体配置
    "font_family": "Arial",
    "font_size": 16,
    "title_size": 17,
    "label_size": 16,
    "tick_size": 16,
    "legend_size": 16,
    "annotation_size": 16,

    # 图片配置
    "dpi": 600,
    "figsize": {
        "enu_timeseries": (14, 9),
        "horizontal_vertical": (14, 7),
        "histogram": (15, 4.8),
        "horizontal_scatter": (8, 8),
        "accuracy_comparison": (10, 6),
        "satellite_count": (13, 5),
        "fix_rate": (14, 7),
        "ratio_timeseries": (14, 5),
    },

    # 统一颜色
    "colors": {
        "FIXED": "red",
        "FLOAT": "blue",
        "OTHER": "gray",
        "GPS": "blue",
        "BDS": "red",
        "OVERALL": "red",
        "ZERO": "black",
    },

    # 线条与散点
    "marker_size": 8,
    "scatter_alpha": 0.75,
    "line_width": 1.0,
    "reference_line_width": 1.0,
    "grid_alpha": 0.6,
    "grid_linewidth": 1.0,
    "grid_linestyle": "--",

    # 分析参数
    "rolling_window_epochs": 300,
    "histogram_bins": 60,

    # 如果 C++ 程序确实使用了明确的 Ratio 阈值，在这里填写数值。
    # 未确认程序阈值时保持 None，不绘制阈值线。
    "ratio_threshold": 3.0,
    
    # ENU时序图横坐标配置
    "enu_timeseries_start": "2026-11-23 18:00:00",
    "enu_timeseries_end": "2026-11-24 18:00:00",

    # 横坐标刻度位置（小时）
    "enu_timeseries_xticks_h": [0, 4, 8, 12, 16,20, 24],
}

# Matplotlib 全局样式
matplotlib.rcParams.update({
    "font.family": CONFIG["font_family"],
    "font.size": CONFIG["font_size"],
    "axes.titlesize": CONFIG["title_size"],
    "axes.labelsize": CONFIG["label_size"],
    "xtick.labelsize": CONFIG["tick_size"],
    "ytick.labelsize": CONFIG["tick_size"],
    "legend.fontsize": CONFIG["legend_size"],
    "axes.unicode_minus": False,
    "mathtext.fontset": "stix",
    "figure.dpi": 100,
    "savefig.dpi": CONFIG["dpi"],
    "axes.linewidth": 1.5,
})

PLOT_MODULES = [
    ("plot_01_enu_timeseries", "ENU三方向定位误差时序图"),
    ("plot_02_horizontal_vertical", "水平及垂向定位误差时序图"),
    ("plot_03_enu_histogram", "ENU三方向误差分布直方图"),
    ("plot_04_horizontal_scatter", "水平误差空间散点图"),
    ("plot_05_accuracy_comparison", "FIXED与FLOAT定位精度对比图"),
    ("plot_06_satellite_count", "GPS/BDS卫星数量时序图"),
    ("plot_07_fix_rate", "解状态及固定解比例图"),
    ("plot_08_ratio_timeseries", "Ratio时序图"),
]


def main():
    if not CONFIG["data_path"].is_file():
        raise FileNotFoundError(
            f"找不到原始数据文件：{CONFIG['data_path']}\n"
            "请检查 main.py 中的 DATA_PATH。"
        )

    CONFIG["output_dir"].mkdir(parents=True, exist_ok=True)

    print("=" * 62)
    print("RTK 定位结果绘图程序")
    print(f"数据文件：{CONFIG['data_path']}")
    print(f"图片目录：{CONFIG['output_dir']}")
    print("=" * 62)

    succeeded = []
    failed = []

    for index, (module_name, description) in enumerate(PLOT_MODULES, start=1):
        print(f"\n[{index}/8] 正在生成：{description}")

        try:
            module = importlib.import_module(module_name)
            module.plot(CONFIG)
            succeeded.append(description)
            print(f"[{index}/8] 完成：{description}")

        except Exception as exc:
            failed.append((description, str(exc)))
            print(f"[{index}/8] 失败：{description}")
            print(f"错误信息：{exc}")
            traceback.print_exc()

    print("\n" + "=" * 62)
    print(f"绘图完成：成功 {len(succeeded)} 张，失败 {len(failed)} 张")

    if succeeded:
        print("\n成功生成：")
        for item in succeeded:
            print(f"  ✓ {item}")

    if failed:
        print("\n生成失败：")
        for item, error in failed:
            print(f"  ✗ {item}: {error}")

    print(f"\n图片保存目录：{CONFIG['output_dir']}")
    print("=" * 62)


if __name__ == "__main__":
    main()

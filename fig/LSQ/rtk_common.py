from pathlib import Path
import re
import numpy as np
import pandas as pd
import matplotlib.dates as mdates


GPS_EPOCH = pd.Timestamp("1980-01-06 00:00:00")


def read_rtk_data(config, required_columns=None):
    """读取原始 LSQ.txt，统一列名、数值类型、状态和 GPS 时间。"""
    path = Path(config["data_path"])
    if not path.is_file():
        raise FileNotFoundError(f"找不到数据文件：{path}")

    df = pd.read_csv(path, sep=r"\s+", engine="python")
    df.columns = [re.sub(r"\([^)]*\)$", "", str(col).lstrip("#")) for col in df.columns]

    if required_columns is None:
        required_columns = ["Week", "SOW", "Status"]

    missing = [col for col in required_columns if col not in df.columns]
    if missing:
        raise ValueError(f"LSQ.txt 缺少必要字段：{missing}")

    for col in ["Week", "SOW"]:
        if col in df.columns:
            df[col] = pd.to_numeric(df[col], errors="coerce")

    numeric_candidates = [
        "X", "Y", "Z", "dX", "dY", "dZ", "dE", "dN", "dU",
        "Ratio", "GPSNum", "BDSNum"
    ]
    for col in numeric_candidates:
        if col in df.columns:
            df[col] = pd.to_numeric(df[col], errors="coerce")

    df["Status"] = (
        df["Status"].fillna("UNKNOWN").astype(str).str.strip().str.upper()
    )

    df = df.dropna(subset=["Week", "SOW"]).copy()

    df["Time"] = (
        GPS_EPOCH
        + pd.to_timedelta(df["Week"] * 7, unit="D")
        + pd.to_timedelta(df["SOW"], unit="s")
    )

    df = df.sort_values("Time").reset_index(drop=True)

    df["StatusGroup"] = df["Status"].where(
        df["Status"].isin(["FIXED", "FLOAT"]), "OTHER"
    )

    return df


def ecef_to_geodetic_lat_lon(x, y, z):
    """WGS84 ECEF 转大地纬度、经度，返回弧度。"""
    a = 6378137.0
    f = 1.0 / 298.257223563
    e2 = f * (2.0 - f)

    p = np.hypot(x, y)
    lon = np.arctan2(y, x)

    lat = np.arctan2(z, p * (1.0 - e2))

    for _ in range(30):
        sin_lat = np.sin(lat)
        N = a / np.sqrt(1.0 - e2 * sin_lat**2)
        h = p / np.cos(lat) - N
        denominator = p * (1.0 - e2 * N / (N + h))
        lat_new = np.arctan2(z, denominator)

        if abs(lat_new - lat) < 1e-14:
            lat = lat_new
            break
        lat = lat_new

    return lat, lon


def get_ecef_to_enu_rotation(base_ecef):
    """构造 ECEF -> ENU 旋转矩阵。"""
    B, L = ecef_to_geodetic_lat_lon(*base_ecef)

    R = np.array([
        [-np.sin(L), np.cos(L), 0.0],
        [-np.sin(B) * np.cos(L), -np.sin(B) * np.sin(L), np.cos(B)],
        [np.cos(B) * np.cos(L), np.cos(B) * np.sin(L), np.sin(B)],
    ])

    return R, B, L


def add_enu_errors(df, config):
    """从每历元 ECEF 基线向量计算 ECEF 残差，再旋转为 ENU 残差。"""
    required = ["dX", "dY", "dZ"]
    missing = [col for col in required if col not in df.columns]
    if missing:
        raise ValueError(f"无法计算定位误差，缺少字段：{missing}")

    ref = np.asarray(config["reference_baseline_ecef"], dtype=float)
    base = np.asarray(config["base_station_ecef"], dtype=float)

    xyz = df[["dX", "dY", "dZ"]].to_numpy(dtype=float)
    error_xyz = xyz - ref

    R, B, L = get_ecef_to_enu_rotation(base)
    error_enu = error_xyz @ R.T

    result = df.copy()
    result["eX"] = error_xyz[:, 0]
    result["eY"] = error_xyz[:, 1]
    result["eZ"] = error_xyz[:, 2]
    result["eE"] = error_enu[:, 0]
    result["eN"] = error_enu[:, 1]
    result["eU"] = error_enu[:, 2]
    result["EH"] = np.sqrt(result["eE"]**2 + result["eN"]**2)
    result["EV"] = np.abs(result["eU"])
    result["E3D"] = np.sqrt(
        result["eE"]**2 + result["eN"]**2 + result["eU"]**2
    )

    return result, R, B, L


def save_figure(fig, config, filename):
    """统一保存图片并关闭 figure。"""
    output_dir = Path(config["output_dir"])
    output_dir.mkdir(parents=True, exist_ok=True)

    path = output_dir / filename
    fig.savefig(
        path,
        dpi=config["dpi"],
        bbox_inches="tight",
        facecolor="white"
    )

    import matplotlib.pyplot as plt
    plt.close(fig)
    return path


def set_time_axis(ax, config):
    locator = mdates.AutoDateLocator(minticks=5, maxticks=10)
    ax.xaxis.set_major_locator(locator)
    ax.xaxis.set_major_formatter(mdates.ConciseDateFormatter(locator))
    ax.tick_params(axis="both", labelsize=config["tick_size"])


def add_grid(ax, config, axis="both"):
    ax.grid(
        True,
        axis=axis,
        linestyle=config["grid_linestyle"],
        linewidth=config["grid_linewidth"],
        alpha=config["grid_alpha"]
    )
    ax.set_axisbelow(True)


def status_scatter(ax, df, column, config, marker=None):
    """按 FIXED/FLOAT/OTHER 状态统一绘制时序散点。"""
    colors = config["colors"]
    marker = marker or "."

    for status in ["FIXED", "FLOAT", "OTHER"]:
        sub = df[df["StatusGroup"] == status].dropna(
            subset=["Time", column]
        )
        if sub.empty:
            continue

        ax.scatter(
            sub["Time"],
            sub[column],
            s=config["marker_size"],
            marker=marker,
            color=colors[status],
            label=status,
            alpha=config["scatter_alpha"],
            linewidths=0,
            rasterized=True
        )


def calculate_statistics(values):
    values = np.asarray(values, dtype=float)
    values = values[np.isfinite(values)]

    if len(values) == 0:
        return None

    return {
        "N": int(len(values)),
        "Bias_m": float(np.mean(values)),
        "STD_m": float(np.std(values, ddof=1)) if len(values) > 1 else np.nan,
        "RMS_m": float(np.sqrt(np.mean(values**2))),
        "MaxAbs_m": float(np.max(np.abs(values))),
        "P95Abs_m": float(np.percentile(np.abs(values), 95)),
    }

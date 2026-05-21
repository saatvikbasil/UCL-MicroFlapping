from __future__ import annotations
from pathlib import Path
import itertools
import re
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.signal import find_peaks, butter, sosfiltfilt
from scipy.interpolate import make_interp_spline
from scipy.optimize import curve_fit


DATASETS = {
    "No Wing (Control)": Path("data/rod without wing 1.csv"),
    "No Root Chord": Path("data/5.708 Nmm-rad/A1 Mylar/A1 Mylar 1.csv"),
    "Rigid Root Chord": Path("data/5.708 Nmm-rad/A1 Mylar Rigid Root Chord/A1 Mylar Rigid 1.csv"),
    "Elastic Root Chord": Path("data/5.708 Nmm-rad/A1 Mylar Elastic Root Chord/A1 Mylar Elastic 2.csv"),
    "Flexible Leading Edge": Path("data/5.708 Nmm-rad/A1 Mylar Flexible LE/A1 Mylar Flexible LE.csv"),
}

SIM_DATASETS = None

DIAGNOSE_PATH = next(iter(DATASETS.values()))

FORCE_TO_MN         = 1.0
COMMANDED_FREQS_HZ  = list(range(1, 26))

ENV_WINDOW_S        = 0.03
ACTIVE_THRESHOLD_MN = 12.0
QUIET_GAP_S         = 0.7
MIN_BURST_DUR_S     = 0.2

PEAK_MIN_MN         = -40.0
PEAK_MAX_MN         =  40.0

LPF_CUTOFF_HZ       = 75.0
LPF_ORDER           = 4

SHOW_SCATTER        = False   # set True to re-enable individual data points
PLOT_FREQ_MIN_HZ    = 5       # ignore bursts below this frequency in the comparison plot


def load_force(csv_path):
    df = pd.read_csv(csv_path, header=None, usecols=[0, 1], names=["t", "F"])
    t  = df["t"].to_numpy(dtype=float)
    F  = df["F"].to_numpy(dtype=float) * FORCE_TO_MN
    fs = 1.0 / np.median(np.diff(t))
    return t, F, fs
 
 
def lpf(seg, fs, cutoff=LPF_CUTOFF_HZ, order=LPF_ORDER):
    nyq = 0.5 * fs
    if cutoff >= nyq:
        return seg
    sos = butter(order, cutoff / nyq, btype="low", output="sos")
    return sosfiltfilt(sos, seg)
 
 
def dominant_freq(seg, fs, f_cmd=None):
    if len(seg) < 8:
        return None
    if f_cmd is not None:
        f_lo = max(0.5, f_cmd * 0.5)
        f_hi = min(35.0, f_cmd * 2.5)
    else:
        f_lo, f_hi = 0.5, 35.0
    spectrum = np.abs(np.fft.rfft(seg - seg.mean()))
    freqs    = np.fft.rfftfreq(len(seg), d=1.0 / fs)
    mask     = (freqs >= f_lo) & (freqs <= f_hi)
    if not mask.any():
        return None
    return float(freqs[mask][np.argmax(spectrum[mask])])
 
 
def peak_params(f_cmd, fs):
    if f_cmd is None:
        return 0.30, 1
    prom = 0.40 if f_cmd < 5.0 else (0.30 if f_cmd < 13.0 else 0.25)
    dist = max(1, int(round(0.80 * fs / f_cmd)))
    return prom, dist
 
 
def detect_bursts(t, F):
    fs  = 1.0 / np.median(np.diff(t))
    win = max(int(round(ENV_WINDOW_S * fs)), 5)
    env = (pd.Series(np.abs(F))
             .rolling(win, center=True, min_periods=1).mean()
             .to_numpy())
    active = env > ACTIVE_THRESHOLD_MN
 
    edges  = np.diff(active.astype(int), prepend=0, append=0)
    starts = np.where(edges ==  1)[0]
    ends   = np.where(edges == -1)[0] - 1
 
    if starts.size > 1:
        ms, me = [starts[0]], [ends[0]]
        for s, e in zip(starts[1:], ends[1:]):
            if (s - me[-1]) / fs < QUIET_GAP_S:
                me[-1] = e
            else:
                ms.append(s)
                me.append(e)
        starts, ends = np.array(ms), np.array(me)
 
    keep = (ends - starts) / fs >= MIN_BURST_DUR_S
    return starts[keep], ends[keep], env
 
 
def analyse_burst(seg, fs, f_cmd=None):
    seg_f      = lpf(seg, fs)
    prom, dist = peak_params(f_cmd, fs)
    prom_abs   = prom * np.abs(seg_f).max()
 
    raw_crests,  _ = find_peaks( seg_f, prominence=prom_abs, distance=dist)
    raw_troughs, _ = find_peaks(-seg_f, prominence=prom_abs, distance=dist)
 
    crests  = raw_crests [seg_f[raw_crests]  <= PEAK_MAX_MN]
    troughs = raw_troughs[seg_f[raw_troughs] >= PEAK_MIN_MN]
 
    if crests.size and troughs.size:
        crest  = seg_f[crests].mean()
        trough = seg_f[troughs].mean()
    else:
        crest  = min(seg_f.max(), PEAK_MAX_MN)
        trough = max(seg_f.min(), PEAK_MIN_MN)
 
    clipped = seg[(seg >= PEAK_MIN_MN) & (seg <= PEAK_MAX_MN)]
    mean_mN = float(clipped.mean()) if clipped.size else float(seg.mean())
 
    return {
        "f_est_hz":    f_cmd if f_cmd is not None else float("nan"),
        "crests":      crests,
        "troughs":     troughs,
        "raw_crests":  raw_crests,
        "raw_troughs": raw_troughs,
        "crest_mN":    crest,
        "trough_mN":   trough,
        "mean_mN":     mean_mN,
        "midline_mN":  (crest + trough) / 2,
        "amp_mN":      (crest - trough) / 2,
    }
 
 
def analyse_file(csv_path, commanded_freqs=COMMANDED_FREQS_HZ):
    t, F, fs = load_force(csv_path)
    starts, ends, env = detect_bursts(t, F)
 
    if commanded_freqs and len(commanded_freqs) == len(starts):
        freqs  = list(commanded_freqs)
        xlabel = "Commanded frequency (Hz)"
    else:
        if commanded_freqs and len(starts) > 1:
            gaps     = np.diff(t[starts])
            gaps_str = ", ".join(f"{g:.1f}" for g in gaps)
            print(f"  ! {csv_path.name}: {len(starts)} bursts vs "
                  f"{len(commanded_freqs)} commanded — using burst index")
            print(f"    burst-to-burst gaps (s): {gaps_str}")
        freqs  = list(range(1, len(starts) + 1))
        xlabel = "Burst #"
 
    bursts = [
        analyse_burst(F[s:e + 1], fs, f_cmd=f)
        for s, e, f in zip(starts, ends, freqs)
    ]
 
    return {
        "t": t, "F": F, "fs": fs, "env": env,
        "starts": starts, "ends": ends,
        "bursts": bursts, "freqs": freqs, "xlabel": xlabel,
    }
 
 
def per_burst_table(csv_path, commanded_freqs=COMMANDED_FREQS_HZ):
    r = analyse_file(csv_path, commanded_freqs)
    t = r["t"]
    rows = []
    for k, (s, e, b, f) in enumerate(zip(r["starts"], r["ends"],
                                         r["bursts"], r["freqs"])):
        rows.append({
            "burst":      k + 1,
            "freq_hz":    f,
            "f_est_hz":   b["f_est_hz"],
            "t_start_s":  t[s],
            "t_end_s":    t[e],
            "dur_s":      t[e] - t[s],
            "n_crests":   int(b["crests"].size),
            "n_troughs":  int(b["troughs"].size),
            "crest_mN":   b["crest_mN"],
            "trough_mN":  b["trough_mN"],
            "mean_mN":    b["mean_mN"],
            "midline_mN": b["midline_mN"],
            "amp_mN":     b["amp_mN"],
        })
    return pd.DataFrame(rows)
 
 
def diagnose(csv_path):
    r = analyse_file(csv_path)
    t, F, env = r["t"], r["F"], r["env"]
 
    fig, (ax1, ax2) = plt.subplots(
        2, 1, figsize=(13, 8), gridspec_kw={"height_ratios": [2, 1]}
    )
 
    ax1.plot(t, F, color="0.4", lw=0.4, label="F(t)")
    ax1.plot(t, env, color="tab:red", lw=1.0, label="|F| envelope")
    ax1.axhline(ACTIVE_THRESHOLD_MN, color="tab:red", ls="--", lw=0.8,
                label=f"threshold = {ACTIVE_THRESHOLD_MN:.0f} mN")
    ax1.axhspan(PEAK_MIN_MN, PEAK_MAX_MN, color="yellow", alpha=0.06,
                label=f"valid peak window [{PEAK_MIN_MN:.0f}, {PEAK_MAX_MN:.0f}] mN")
    ax1.axhline(PEAK_MAX_MN, color="goldenrod", ls=":", lw=0.8)
    ax1.axhline(PEAK_MIN_MN, color="goldenrod", ls=":", lw=0.8)
 
    for s, e, b in zip(r["starts"], r["ends"], r["bursts"]):
        seg_f = lpf(F[s:e + 1], r["fs"])
        ax1.axvspan(t[s], t[e], color="green", alpha=0.08)
        if b["crests"].size:
            ax1.scatter(t[s + b["crests"]], seg_f[b["crests"]],
                        color="tab:red", s=6, zorder=3)
        if b["troughs"].size:
            ax1.scatter(t[s + b["troughs"]], seg_f[b["troughs"]],
                        color="tab:blue", s=6, zorder=3)
        ax1.plot([t[s], t[e]], [b["mean_mN"]]    * 2, color="tab:green", lw=1.6)
        ax1.plot([t[s], t[e]], [b["midline_mN"]] * 2, "k--", lw=1.0)
 
    ax1.set_xlabel("Time (s)", fontsize=14)
    ax1.set_ylabel("Force (mN)", fontsize=14)
    ax1.tick_params(labelsize=14)
    ax1.set_title(f"{csv_path.name} — {len(r['bursts'])} bursts  "
                  f"[peak gate {PEAK_MIN_MN:.0f}–{PEAK_MAX_MN:.0f} mN]")
    ax1.grid(alpha=0.3)
    ax1.legend(loc="upper left", fontsize=12)
 
    means    = [b["mean_mN"]    for b in r["bursts"]]
    midlines = [b["midline_mN"] for b in r["bursts"]]
    ax2.plot(r["freqs"], means,    "o-", color="tab:green",
             label="mean F(t) — time-average lift")
    ax2.plot(r["freqs"], midlines, "^:", color="tab:purple",
             label="midline (crest+trough)/2")
    ax2.axhline(0, color="0.6", lw=0.8)
    ax2.set_xlabel(r["xlabel"], fontsize=14)
    ax2.set_ylabel("Force (mN)", fontsize=14)
    ax2.tick_params(labelsize=14)
    ax2.set_title("Mean lift vs midline — divergence = waveform asymmetry")
    ax2.legend(fontsize=12)
    ax2.grid(alpha=0.3)
 
    plt.tight_layout()
    return fig
 
 
# ---------------------------------------------------------------------------
# Curve fitting helpers
# ---------------------------------------------------------------------------
 
def _gaussian(x, A, mu, sigma):
    return A * np.exp(-0.5 * ((x - mu) / sigma) ** 2)
 
 
def gaussian_fit_curve(x, y, n_pts=300):
    x = np.asarray(x, dtype=float)
    y = np.asarray(y, dtype=float)
 
    i0    = int(np.argmax(y))
    A0    = float(y[i0])
    mu0   = float(x[i0])
    sig0  = 3.0
    x_fine = np.linspace(x.min(), x.max(), n_pts)
 
    try:
        popt, _ = curve_fit(
            _gaussian, x, y,
            p0=[A0, mu0, sig0],
            bounds=([0, x.min(), 0.5], [np.inf, x.max(), 20.0]),
            maxfev=4000,
        )
        y_hat  = _gaussian(x, *popt)
        ss_res = np.sum((y - y_hat) ** 2)
        ss_tot = np.sum((y - y.mean()) ** 2)
        r2     = 1.0 - ss_res / ss_tot if ss_tot > 0 else float("nan")
        y_fine = np.clip(_gaussian(x_fine, *popt), 0.0, None)
        return x_fine, y_fine, r2, tuple(popt)
    except RuntimeError:
        print(f"  ! Gaussian fit failed — plotting zeros")
        return x_fine, np.zeros_like(x_fine), float("nan"), (0, 0, 0)
 
 
def smooth_curve(x, y, n_pts=300):
    """Cubic spline smooth — used for simulated data."""
    x = np.asarray(x, dtype=float)
    y = np.asarray(y, dtype=float)
    if len(x) < 4:
        return x, y
    x_fine = np.linspace(x.min(), x.max(), n_pts)
    return x_fine, make_interp_spline(x, y, k=3)(x_fine)
 
 
def _extract_stiffness_key(label):
    """Pull the first float from a label string for colour matching."""
    m = re.search(r"[\d]+\.[\d]+", label)
    return m.group(0) if m else None
 
 
def compare_lift(datasets, sim_datasets=None, save_path=None):
    fig, ax = plt.subplots(figsize=(10, 6))
    xlabels = set()
 
    default_colors = plt.rcParams["axes.prop_cycle"].by_key()["color"]
    color_cycle    = itertools.cycle(default_colors)
 
    stiffness_color: dict[str, str] = {}
 
    # --- experimental datasets ---
    for label, path in datasets.items():
        r = analyse_file(path)
        if not r["bursts"]:
            print(f"  ! {label}: no bursts detected")
            continue
        freqs = np.asarray(r["freqs"], dtype=float)
        means = np.clip([b["mean_mN"] for b in r["bursts"]], 0.0, None)
 
        mask  = freqs >= PLOT_FREQ_MIN_HZ
        freqs, means = freqs[mask], means[mask]
        color = next(color_cycle)
 
        key = _extract_stiffness_key(label)
        if key:
            stiffness_color[key] = color
 
        x_fit, y_fit, r2, popt = gaussian_fit_curve(freqs, means)
        fit_label = f"{label}  (R²={r2:.3f}, peak {popt[1]:.1f} Hz)"
        ax.plot(x_fit, y_fit, lw=2.0, color=color, label=fit_label)
 
        if SHOW_SCATTER:
            ax.scatter(freqs, means, s=8, color=color, zorder=4)
 
        xlabels.add(r["xlabel"])
 
    # --- simulated datasets: dashed, matched colour ---
    if sim_datasets:
        for label, path in sim_datasets.items():
            if not path.exists():
                print(f"  ! Sim CSV not found: {path}")
                continue
            df    = pd.read_csv(path)
            freqs = df["Frequency_Hz"].to_numpy(dtype=float)
            means = np.clip(df["AvgLift_mN"].to_numpy(dtype=float), 0.0, None)
 
            mask  = freqs >= PLOT_FREQ_MIN_HZ
            freqs, means = freqs[mask], means[mask]
 
            key   = _extract_stiffness_key(label)
            color = stiffness_color.get(key, next(color_cycle))
 
            x_s, y_s = smooth_curve(freqs, means)
            ax.plot(x_s, y_s, lw=1.2, ls="--", color=color,
                    alpha=0.6, label=f"{label} (sim)")
 
    ax.axhline(0, color="0.6", lw=0.8)
    y_max = ax.get_ylim()[1]
    ax.set_ylim(bottom=0, top=y_max + 2)
    ax.set_xlabel(xlabels.pop() if len(xlabels) == 1 else "Frequency (Hz)", fontsize=14)
    ax.set_ylabel("Mean lift (mN)", fontsize=14)
    ax.tick_params(labelsize=14)
    ax.set_title("Mean lift vs frequency — spring comparison")
    ax.legend(fontsize=12)
    ax.grid(alpha=0.3)
    plt.tight_layout()
    if save_path:
        fig.savefig(save_path, dpi=300, bbox_inches="tight")
    return fig
 
 
def main():
    results = per_burst_table(DIAGNOSE_PATH)
    print(results.to_string(index=False))
    results.to_csv(DIAGNOSE_PATH.with_name(DIAGNOSE_PATH.stem + "_midline.csv"),
                   index=False)
 
    diagnose(DIAGNOSE_PATH)
    plt.savefig(DIAGNOSE_PATH.with_name(DIAGNOSE_PATH.stem + "_diagnostic.png"),
                dpi=300, bbox_inches="tight")
 
    compare_lift(DATASETS, sim_datasets=SIM_DATASETS,
                 save_path=Path("lift_comparison.png"))
 
    plt.show()
 
 
if __name__ == "__main__":
    main()
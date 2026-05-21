"""Batch-run track_wing on every dataset folder under DATA_ROOT.

Expects flat folder layout where each subfolder is named like:
    {freq}hz_{wing|nospring|nowing}
e.g. 7.5hz_wing, 10hz_nospring, 25hz_nowing

Usage:
    python batch_all.py [DATA_ROOT] [--out OUT_DIR]
"""
import argparse
import csv
import re
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt

from track_wing import (process, detect_cycles, stack_cycles,
                        stroke_stats, plot, diagnostic_overlay)

THRESHOLD = 80
N_CYCLES = 3
SAVE_MASKS = False
CENTER_MODE = 'median'    # 'median' (default, robust to amplitude asymmetry),
                          # 'mean' (biased), 'geometric' (trivially zero),
                          # 'none' = 0°, or 'custom' with CUSTOM_CENTER set.
CUSTOM_CENTER = None      # set to a number (deg) to use a fixed reference

FOLDER_RE = re.compile(r'^(\d+(?:\.\d+)?)hz_(wing|nospring|nowing)$',
                       re.IGNORECASE)
COND_LABEL = {'wing': 'Wing and spring',
              'nospring': 'No spring',
              'nowing': 'No wing'}
COND_ORDER = ['wing', 'nospring', 'nowing']

# Stats columns written to every per-dataset and summary CSV, in this order.
STAT_KEYS = ['center', 'amp_pp_mean', 'amp_pp_std',
             'peak_mean', 'trough_mean',
             'amp_right', 'amp_left', 'asymmetry']


def discover(root):
    out = []
    for p in sorted(root.iterdir()):
        if not p.is_dir():
            continue
        m = FOLDER_RE.match(p.name)
        if not m:
            print(f'  skip (does not match pattern): {p.name}')
            continue
        out.append((p, float(m.group(1)), m.group(2).lower()))
    return out


def save_timeseries_csv(path, t, angles):
    """Write a two-column CSV: time_s, stroke_angle_deg."""
    with open(path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['time_s', 'stroke_angle_deg'])
        for ti, ai in zip(t, angles):
            writer.writerow([f'{ti:.6f}',
                             '' if np.isnan(ai) else f'{ai:.4f}'])


def save_cycle_csv(path, phase, stack, center):
    """Write per-phase mean ± std for every stacked cycle, plus raw cycles.

    Columns: phase, cycle_0, cycle_1, …, mean, std
    Values are angle minus center so they are comparable across datasets.
    """
    if stack is None or not len(stack):
        return
    centered = stack - center
    with open(path, 'w', newline='') as f:
        writer = csv.writer(f)
        cycle_headers = [f'cycle_{k}' for k in range(len(centered))]
        writer.writerow(['phase'] + cycle_headers + ['mean', 'std'])
        mean = centered.mean(0)
        std  = centered.std(0)
        for i, ph in enumerate(phase):
            row = [f'{ph:.6f}']
            row += [f'{centered[k, i]:.4f}' for k in range(len(centered))]
            row += [f'{mean[i]:.4f}', f'{std[i]:.4f}']
            writer.writerow(row)


def save_stats_csv(path, stats):
    """Write a two-column key/value CSV for the scalar summary stats."""
    with open(path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['metric', 'value'])
        for k in STAT_KEYS:
            v = stats.get(k, float('nan'))
            writer.writerow([k, '' if np.isnan(v) else f'{v:.4f}'])


def save_summary_csv(path, summary, freqs):
    """Write one row per (condition, frequency) with all scalar stats."""
    with open(path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['condition', 'condition_label', 'freq_hz'] + STAT_KEYS)
        for cond in COND_ORDER:
            for freq in freqs:
                stats = summary.get((cond, freq), {})
                row = [cond, COND_LABEL[cond], freq]
                for k in STAT_KEYS:
                    v = stats.get(k, float('nan'))
                    row.append('' if np.isnan(v) else f'{v:.4f}')
                writer.writerow(row)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('data_root', nargs='?', default='.',
                    help='parent folder containing the {freq}hz_{cond} subfolders')
    ap.add_argument('--out', default='results', help='output folder')
    args = ap.parse_args()

    data_root = Path(args.data_root)
    out_root = Path(args.out)
    out_root.mkdir(exist_ok=True)

    folders = discover(data_root)
    print(f'found {len(folders)} datasets in {data_root.resolve()}')
    if not folders:
        return
    for p, f, c in folders:
        print(f'  {p.name:25s}  f={f:>4} Hz  {COND_LABEL[c]}')

    summary = {}
    cycle_data = {}  # (cond, freq) -> (phase, stack, center)
    for folder, freq, cond in folders:
        tag = folder.name
        print(f'\n=== {tag} ===')
        masks_dir = out_root / f'{tag}_masks' if SAVE_MASKS else None
        try:
            t, ang, clamp_box = process(folder, threshold=THRESHOLD,
                                         save_masks_dir=masks_dir)
        except Exception as e:
            print(f'  failed: {e}')
            continue
        nans = int(np.isnan(ang).sum())
        starts = detect_cycles(ang, freq)
        phase, stack = stack_cycles(t, ang, starts, n=N_CYCLES)
        stats = stroke_stats(stack, center_mode=CENTER_MODE,
                              custom_center=CUSTOM_CENTER)
        summary[(cond, freq)] = stats
        cycle_data[(cond, freq)] = (phase, stack, stats.get('center', 0.0))
        n_used = 0 if stack is None else len(stack)
        note = f' (asked {N_CYCLES}, used {n_used})' if n_used != N_CYCLES else ''
        print(f'  {nans} NaN frames, {len(starts)} cycle starts, '
              f'{n_used} cycles averaged{note}, '
              f'amp_pp={stats.get("amp_pp_mean", float("nan")):.2f}°, '
              f'center={stats.get("center", 0.0):.2f}°, '
              f'asym={stats.get("asymmetry", float("nan")):.2f}°')

        # --- save outputs ---
        np.savez(out_root / f'{tag}.npz', t=t, angles=ang,
                 clamp_box=clamp_box, freq=freq,
                 cycle_starts=starts, **stats)

        save_timeseries_csv(out_root / f'{tag}_timeseries.csv', t, ang)
        save_cycle_csv(out_root / f'{tag}_cycles.csv',
                       phase, stack, stats.get('center', 0.0))
        save_stats_csv(out_root / f'{tag}_stats.csv', stats)
        print(f'  wrote {tag}_timeseries.csv, {tag}_cycles.csv, {tag}_stats.csv')

        plot(t, ang, starts, phase, stack,
             f'{COND_LABEL[cond]} — {freq} Hz', out_root / f'{tag}.png',
             center=stats.get('center', 0.0))
        diagnostic_overlay(folder, clamp_box, THRESHOLD,
                           out_root / f'{tag}_diag.png')

    if not summary:
        print('\nno successful datasets to summarise')
        return

    freqs = sorted({f for (_, f) in summary})

    # --- summary CSV ---
    save_summary_csv(out_root / 'summary.csv', summary, freqs)
    print(f'\nwrote results/summary.csv')

    plt.rcParams.update({
    "axes.titlesize": 16,
    "axes.labelsize": 14,
    "xtick.labelsize": 12,
    "ytick.labelsize": 12,
    "legend.fontsize": 12
    })
    fig, axes = plt.subplots(1, 3, figsize=(16, 4.8), sharex=True)
    for cond in COND_ORDER:
        amp = [summary.get((cond, f), {}).get('amp_pp_mean', np.nan) for f in freqs]
        center = [summary.get((cond, f), {}).get('center', np.nan) for f in freqs]
        asym = [summary.get((cond, f), {}).get('asymmetry', np.nan) for f in freqs]
        label = COND_LABEL[cond]
        axes[0].plot(freqs, amp, 'o-', label=label)
        axes[1].plot(freqs, center, 'o-', label=label)
        axes[2].plot(freqs, asym, 'o-', label=label)
    axes[0].set(xlabel='f (Hz)', ylabel='peak-to-peak amplitude (°)',
                title='Stroke amplitude')
    axes[1].set(xlabel='f (Hz)', ylabel='center / median angle (°)',
                title='Stroke offset')
    axes[2].set(xlabel='f (Hz)', ylabel='right − left amplitude (°)',
                title='Asymmetry')
    axes[2].axhline(0, color='k', lw=0.5)
    for ax in axes:
        ax.grid(alpha=0.3); ax.legend(fontsize=14)
    plt.tight_layout()
    plt.savefig(out_root / 'summary.png', dpi=300)
    print(f'wrote {out_root}/summary.png')

    # 3-panel overlay: one panel per condition, all frequencies overlaid.
    # Centered version (stroke shape comparison)
    cmap = plt.get_cmap('viridis')
    if freqs:
        fmin, fmax = min(freqs), max(freqs)
        norm = lambda f: (f - fmin) / (fmax - fmin) if fmax > fmin else 0.5

        fig, axes = plt.subplots(1, 3, figsize=(16, 5))
        for ax, cond in zip(axes, COND_ORDER):
            for f in freqs:
                if (cond, f) not in cycle_data:
                    continue
                phase, stack, c = cycle_data[(cond, f)]
                if stack is None:
                    continue
                ax.plot(phase, stack.mean(0) - c,
                        color=cmap(norm(f)), lw=1.6, label=f'{f} Hz')
            ax.set(xlabel='phase', ylabel='stroke angle − center (°)',
                   title=COND_LABEL[cond])
            ax.axhline(0, color='k', lw=0.5)
            ax.grid(alpha=0.3)
            if ax.has_data():
                ax.legend(fontsize=14, loc='best')
            else:
                ax.text(0.5, 0.5, 'no data', ha='center', va='center',
                        transform=ax.transAxes)
        plt.tight_layout()
        plt.savefig(out_root / 'cycle_overlays_centered.png', dpi=300)
        print(f'wrote {out_root}/cycle_overlays_centered.png')

        # Absolute version (rest position comparison)
        fig, axes = plt.subplots(1, 3, figsize=(16, 5))
        for ax, cond in zip(axes, COND_ORDER):
            for f in freqs:
                if (cond, f) not in cycle_data:
                    continue
                phase, stack, _ = cycle_data[(cond, f)]
                if stack is None:
                    continue
                ax.plot(phase, stack.mean(0),
                        color=cmap(norm(f)), lw=1.6, label=f'{f} Hz')
            ax.set(xlabel='phase', ylabel='stroke angle (°)',
                   title=COND_LABEL[cond])
            ax.grid(alpha=0.3)
            if ax.has_data():
                ax.legend(fontsize=14, loc='best')
            else:
                ax.text(0.5, 0.5, 'no data', ha='center', va='center',
                        transform=ax.transAxes)
        plt.tight_layout()
        plt.savefig(out_root / 'cycle_overlays_absolute.png', dpi=300)
        print(f'wrote {out_root}/cycle_overlays_absolute.png')


if __name__ == '__main__':
    main()
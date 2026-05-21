"""Track flapping-wing leading-edge angle from a folder of high-speed frames.

Usage:
    python track_wing.py FRAMES_DIR FREQ_HZ [--threshold 80] [--cycles 3]

Convention: stroke angle is signed, measured from image-vertical (rod pointing
straight down from the clamp = 0). Positive = rod tilts toward image +x (right).
Works at any rod orientation including near-horizontal and through-vertical.
"""
from __future__ import annotations
import argparse
from pathlib import Path

import cv2
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import savgol_filter

FPS = 800
EXTS = {'.png', '.jpg', '.jpeg', '.tif', '.tiff', '.bmp'}


def find_clamp_box(mask, erode_size=9, pad=15):
    """Locate the clamp by erosion: the rod is thin and erodes away, the clamp
    survives. Returns (x0, y0, x1, y1) inflated by `pad` to absorb the eroded
    margin and any wire stubs."""
    H, W = mask.shape
    cm = cv2.erode(mask, np.ones((erode_size, erode_size), np.uint8))
    upper = cm.copy(); upper[H * 2 // 3:] = 0
    n, _, stats, _ = cv2.connectedComponentsWithStats(upper, connectivity=8)
    if n <= 1:
        return None
    i = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    x0 = int(stats[i, cv2.CC_STAT_LEFT])
    y0 = int(stats[i, cv2.CC_STAT_TOP])
    x1 = x0 + int(stats[i, cv2.CC_STAT_WIDTH])
    y1 = y0 + int(stats[i, cv2.CC_STAT_HEIGHT])
    return (max(0, x0 - pad), max(0, y0 - pad),
            min(W, x1 + pad), min(H, y1 + pad))


def angle_from_frame(im, clamp_box, threshold=80, max_clamp_gap=30,
                     min_area=100, min_ar=5.0, return_fit=False):
    """Find the rod via connected components and fit a line.

    Strategy: threshold; mask out the clamp's bounding box; pick the connected
    component that is (a) close to the clamp (touches or within `max_clamp_gap`
    px), (b) has high aspect ratio (thin and long), (c) has area > min_area;
    fit a line through its pixels with HUBER loss.
    """
    mask = (im < threshold).astype(np.uint8) * 255
    bx0, by0, bx1, by1 = clamp_box
    mask[by0:by1, bx0:bx1] = 0
    n, lab, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
    if n <= 1:
        return (np.nan, None) if return_fit else np.nan

    cx, cy = (bx0 + bx1) / 2, (by0 + by1) / 2
    best_i, best_score = -1, -1.0
    for j in range(1, n):
        area = int(stats[j, cv2.CC_STAT_AREA])
        if area < min_area:
            continue
        cl = stats[j, cv2.CC_STAT_LEFT]; ct = stats[j, cv2.CC_STAT_TOP]
        cw = stats[j, cv2.CC_STAT_WIDTH]; ch = stats[j, cv2.CC_STAT_HEIGHT]
        dx = max(bx0 - (cl + cw), cl - bx1, 0)
        dy = max(by0 - (ct + ch), ct - by1, 0)
        if np.hypot(dx, dy) > max_clamp_gap:
            continue
        ys, xs = np.where(lab == j)
        cov = np.cov(xs, ys)
        evals = np.linalg.eigvalsh(cov)
        ar = float(np.sqrt(evals[1] / max(evals[0], 0.5)))
        if ar < min_ar:
            continue
        score = ar * np.log(area)
        if score > best_score:
            best_score, best_i = score, j

    if best_i == -1:
        return (np.nan, None) if return_fit else np.nan

    ys, xs = np.where(lab == best_i)
    pts = np.column_stack([xs, ys]).astype(np.float32).reshape(-1, 1, 2)
    vx, vy, x_l, y_l = cv2.fitLine(pts, cv2.DIST_HUBER, 0, 0.01, 0.01).flatten()
    if (xs.mean() - cx) * vx + (ys.mean() - cy) * vy < 0:
        vx, vy = -vx, -vy
    a = float(np.degrees(np.arctan2(vx, vy)))
    if return_fit:
        return a, (float(vx), float(vy), float(x_l), float(y_l), mask,
                   (lab == best_i))
    return a


def list_frames(folder):
    return sorted(p for p in Path(folder).iterdir() if p.suffix.lower() in EXTS)


def calibrate_clamp(paths, threshold, k=10):
    boxes = []
    for p in paths[:k]:
        im = cv2.imread(str(p), cv2.IMREAD_GRAYSCALE)
        m = (im < threshold).astype(np.uint8) * 255
        b = find_clamp_box(m)
        if b is not None:
            boxes.append(b)
    if not boxes:
        raise RuntimeError('clamp not found in any calibration frame')
    boxes = np.array(boxes)
    return tuple(int(v) for v in np.median(boxes, axis=0))


def process(folder, threshold=80, save_masks_dir=None):
    paths = list_frames(folder)
    if not paths:
        raise FileNotFoundError(f'no frames in {folder}')
    clamp_box = calibrate_clamp(paths, threshold)
    angles = np.full(len(paths), np.nan)
    if save_masks_dir is not None:
        Path(save_masks_dir).mkdir(parents=True, exist_ok=True)
    for i, p in enumerate(paths):
        im = cv2.imread(str(p), cv2.IMREAD_GRAYSCALE)
        angles[i] = angle_from_frame(im, clamp_box, threshold=threshold)
        if save_masks_dir is not None:
            mask = (im < threshold).astype(np.uint8) * 255
            cv2.imwrite(str(Path(save_masks_dir) / f'mask_{i:04d}.png'), mask)
    t = np.arange(len(paths)) / FPS
    return t, angles, clamp_box


def diagnostic_overlay(folder, clamp_box, threshold, out_path, n_samples=9):
    paths = list_frames(folder)
    idx = np.linspace(0, len(paths) - 1, n_samples).astype(int)
    cols = 3
    rows = int(np.ceil(n_samples / cols))
    fig, axes = plt.subplots(rows, cols, figsize=(cols * 5, rows * 4))
    axes = np.atleast_2d(axes).flatten()
    bx0, by0, bx1, by1 = clamp_box
    cx, cy = (bx0 + bx1) / 2, (by0 + by1) / 2
    for k, i in enumerate(idx):
        im = cv2.imread(str(paths[i]), cv2.IMREAD_GRAYSCALE)
        a, fit = angle_from_frame(im, clamp_box, threshold=threshold,
                                   return_fit=True)
        axes[k].imshow(im, cmap='gray')
        axes[k].add_patch(plt.Rectangle((bx0, by0), bx1 - bx0, by1 - by0,
                                         fill=False, color='c', lw=1.5))
        axes[k].plot(cx, cy, 'bo', ms=6)
        if fit is not None:
            vx, vy, x_l, y_l, _, rod = fit
            axes[k].imshow(rod.astype(np.uint8) * 255, cmap='Reds', alpha=0.4)
            L = max(im.shape) * 0.7
            axes[k].plot([x_l - L * vx, x_l + L * vx],
                         [y_l - L * vy, y_l + L * vy], 'g-', lw=1.2)
        axes[k].set_xlim(0, im.shape[1]); axes[k].set_ylim(im.shape[0], 0)
        axes[k].set_title(f'frame {i}, t={i / FPS * 1000:.1f} ms, a={a:.1f}°', fontsize=14)
        axes[k].axis('off')
    for k in range(len(idx), len(axes)):
        axes[k].axis('off')
    plt.tight_layout()
    fig.savefig(out_path, dpi=110)
    plt.close(fig)


def detect_cycles(angles, freq):
    a = angles - np.nanmean(angles)
    a = np.where(np.isnan(a), 0.0, a)
    win = max(5, int(0.05 * FPS / freq)) | 1
    a = savgol_filter(a, win, 3)
    zc = np.where((a[:-1] < 0) & (a[1:] >= 0))[0]
    if not len(zc):
        return np.array([], dtype=int)
    spacing = int(0.7 * FPS / freq)
    keep = [int(zc[0])]
    for z in zc[1:]:
        if z - keep[-1] >= spacing:
            keep.append(int(z))
    return np.array(keep)


def stack_cycles(t, angles, starts, n=3):
    """Stack n consecutive cycles onto a normalised phase grid. Falls back to
    however many cycles are actually available if fewer than n have been
    detected (so a 7.5 Hz run that fits 2.x cycles still produces a plot)."""
    n_avail = len(starts) - 1
    if n_avail < 1:
        return None, None
    n_use = min(n, n_avail)
    phase = np.linspace(0, 1, 200)
    out = []
    for k in range(n_use):
        i0, i1 = starts[k], starts[k + 1]
        seg_t = (t[i0:i1] - t[i0]) / (t[i1 - 1] - t[i0])
        seg_a = angles[i0:i1]
        good = ~np.isnan(seg_a)
        if good.sum() < 5:
            continue
        out.append(np.interp(phase, seg_t[good], seg_a[good]))
    return phase, (np.array(out) if out else None)


def stroke_stats(stack, center_mode='median', custom_center=None):
    """Per-cycle peak/trough/amplitude/asymmetry, all computed relative to a
    stroke center.

    The choice of `center_mode` matters for the asymmetry metric:

      'median' (default)  np.median over all stacked samples. For a periodic
                          signal sampled at a fixed frame rate, this is the
                          angle at which the wing spends equal time above and
                          below — i.e. the symmetric-time rest position. It is
                          insensitive to amplitude asymmetry between the two
                          half-strokes, so the asymmetry metric reports the
                          true (peak − rest) − (rest − trough) without
                          self-cancellation. For purely sinusoidal/symmetric
                          strokes it coincides with the mean.

      'mean'              np.mean over all stacked samples. The time-average
                          is biased toward the larger-amplitude side: for an
                          amplitude-asymmetric stroke with peaks +A_r and −A_l
                          (equal half-periods), mean ≈ (A_r − A_l)/π, which
                          drags both (peak − mean) and (mean − trough) toward
                          equality and shrinks the reported asymmetry by a
                          factor of (1 − 2/π) ≈ 0.36. Do not use this mode if
                          you care about asymmetry.

      'geometric'         (peak + trough)/2. This forces (peak − c) ≡ (c −
                          trough) by construction, so asymmetry is trivially
                          zero for every dataset. Useful only as a sanity
                          check that the rest of the pipeline is wired up.

      'none'              0° (image-vertical). Only correct if the camera
                          0° has been physically aligned with the spring's
                          neutral angle, which is generally not the case here.

      'custom' + number   user-supplied centre angle in degrees.
    """
    if stack is None or not len(stack):
        return {}
    pk = stack.max(axis=1)
    tr = stack.min(axis=1)
    if center_mode == 'none':
        c = 0.0
    elif center_mode == 'geometric':
        c = float((pk.mean() + tr.mean()) / 2)
    elif center_mode == 'custom' and custom_center is not None:
        c = float(custom_center)
    elif center_mode == 'mean':
        c = float(stack.mean())
    else:  # 'median' is the default
        c = float(np.median(stack))
    return dict(
        center=c,
        amp_pp_mean=float((pk - tr).mean()),
        amp_pp_std=float((pk - tr).std()),
        peak_mean=float(pk.mean()),
        trough_mean=float(tr.mean()),
        amp_right=float(pk.mean() - c),
        amp_left=float(c - tr.mean()),
        asymmetry=float((pk.mean() - c) - (c - tr.mean())),
    )


def plot(t, angles, starts, phase, stack, title, out_path, center=0.0):
    plt.rcParams.update({
    "axes.titlesize": 16,
    "axes.labelsize": 14,
    "xtick.labelsize": 12,
    "ytick.labelsize": 12,
    "legend.fontsize": 12
    })
    fig, axes = plt.subplots(2, 1, figsize=(10, 8))
    axes[0].plot(t, angles, '.-', ms=3, lw=0.8)
    for c in starts:
        axes[0].axvline(t[c], color='r', alpha=0.25)
    if center != 0.0:
        axes[0].axhline(center, color='g', lw=1, ls='--', alpha=0.6,
                        label=f'center = {center:.1f}°')
        axes[0].legend(loc='upper right', fontsize=12)
    axes[0].set(xlabel='t (s)', ylabel='stroke angle (°)', title=title)
    axes[0].grid(alpha=0.3)

    if stack is not None and len(stack):
        centered = stack - center
        for s in centered:
            axes[1].plot(phase, s, alpha=0.4)
        m, sd = centered.mean(0), centered.std(0)
        axes[1].plot(phase, m, 'k-', lw=2, label='mean')
        axes[1].fill_between(phase, m - sd, m + sd, alpha=0.2)
        axes[1].axhline(0, color='k', lw=0.5)
        ttl = f'{len(stack)} cycle{"s" if len(stack) != 1 else ""}, mean ± 1σ'
        if center != 0.0:
            ttl += f' (centered on {center:.1f}°)'
        axes[1].set(xlabel='phase',
                    ylabel='stroke angle − center (°)' if center else 'stroke angle (°)',
                    title=ttl)
        axes[1].legend(); axes[1].grid(alpha=0.3)
    else:
        axes[1].text(0.5, 0.5, 'not enough cycles detected',
                     ha='center', va='center', transform=axes[1].transAxes)
    plt.tight_layout()
    fig.savefig(out_path, dpi=120)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('folder')
    ap.add_argument('freq', type=float, help='commanded flap frequency (Hz)')
    ap.add_argument('--threshold', type=int, default=80)
    ap.add_argument('--cycles', type=int, default=3)
    ap.add_argument('--out', default=None)
    ap.add_argument('--save-masks', action='store_true',
                    help='save per-frame binary masks to <out>_masks/')
    ap.add_argument('--diag-samples', type=int, default=9)
    ap.add_argument('--center', default='median',
                    help="stroke center for cycle plot/stats: 'median' "
                         "(default, robust to amplitude asymmetry), 'mean' "
                         "(biased toward larger-amplitude side), 'geometric' "
                         "(trivially zeroes asymmetry), 'none' = 0°, or a "
                         "number in degrees (e.g. --center 66.0)")
    args = ap.parse_args()

    # Parse center mode
    try:
        custom_c = float(args.center)
        c_mode = 'custom'
    except ValueError:
        custom_c = None
        c_mode = args.center

    out = args.out or Path(args.folder).name
    masks_dir = f'{out}_masks' if args.save_masks else None
    t, angles, clamp_box = process(args.folder, args.threshold,
                                    save_masks_dir=masks_dir)
    nans = int(np.isnan(angles).sum())
    print(f'clamp_box={clamp_box}, {nans}/{len(angles)} NaN frames')

    starts = detect_cycles(angles, args.freq)
    print(f'{len(starts)} cycle starts detected')

    phase, stack = stack_cycles(t, angles, starts, n=args.cycles)
    stats = stroke_stats(stack, center_mode=c_mode, custom_center=custom_c)
    for k, v in stats.items():
        print(f'  {k}: {v:.3f}')

    center = stats.get('center', 0.0)
    np.savez(f'{out}.npz', t=t, angles=angles, clamp_box=clamp_box,
             freq=args.freq, cycle_starts=starts, **stats)
    plot(t, angles, starts, phase, stack,
         f'{Path(args.folder).name} — f = {args.freq} Hz', f'{out}.png',
         center=center)
    diagnostic_overlay(args.folder, clamp_box, args.threshold,
                       f'{out}_diag.png', n_samples=args.diag_samples)
    extras = f', {masks_dir}/' if masks_dir else ''
    print(f'wrote {out}.npz, {out}.png, {out}_diag.png{extras}')


if __name__ == '__main__':
    main()
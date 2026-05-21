# Python

Two independent analysis pipelines: high-speed camera stroke-angle tracking for kinematic validation, and force-sensor lift-frequency analysis for aerodynamic characterisation.

---

## Files

| File | Description |
|---|---|
| `track_wing.py` | Frame-by-frame stroke-angle extraction from high-speed video |
| `batch_all.py` | Batch runner: processes every `{freq}hz_{condition}` folder and writes a summary CSV and overlay plots |
| `Frequency-Lift.py` | Burst detection and Gaussian curve fitting on PCB 208C force-sensor CSV data |

---

## Installation

```bash
pip install numpy pandas scipy matplotlib opencv-python
```

---

## Stroke Tracking (`track_wing.py` / `batch_all.py`)

Processes folders of high-speed frames (800 fps, Optronis Sprinter-HD-M) to extract the leading-edge stroke angle each frame. The clamp is located by morphological erosion; the rod is found as the connected component with the highest aspect ratio adjacent to the clamp; a HUBER-loss line fit recovers the angle.

**Single dataset**
```bash
python track_wing.py path/to/frames 10.0 --cycles 3
```

**Batch (all conditions and frequencies)**

Folder names must follow the pattern `{freq}hz_{wing|nospring|nowing}`, e.g. `10hz_wing`, `20hz_nospring`.

```bash
python batch_all.py path/to/data --out results
```

Outputs per dataset: `.npz` array file, `_timeseries.csv`, `_cycles.csv`, `_stats.csv`, cycle-overlay plot, and a diagnostic overlay image. A combined `summary.csv` and `summary.png` are written to the output directory.

**Stroke center modes**

| Mode | Description |
|---|---|
| `median` (default) | Robust to amplitude asymmetry; recommended |
| `mean` | Biased toward the larger-amplitude half-stroke |
| `geometric` | Forces asymmetry to zero; sanity check only |
| `none` | Uses 0° (requires physical alignment) |
| number | Fixed reference angle in degrees |

---

## Lift-Force Analysis (`Frequency-Lift.py`)

Processes CSV files recorded by LabVIEW from the PCB 208C ICP force transducer. Detects activity bursts (one per commanded frequency step), computes mean lift per burst using clipped peak detection, and fits a Gaussian curve to the lift-frequency response. Simulated data can be overlaid as dashed curves.

**Configuration** (edit at the top of the file)

```python
DATASETS = {
    "No Root Chord":      Path("data/5.708 Nmm-rad/A1 Mylar/A1 Mylar 1.csv"),
    "Rigid Root Chord":   Path("data/.../A1 Mylar Rigid 1.csv"),
    "Elastic Root Chord": Path("data/.../A1 Mylar Elastic 2.csv"),
    "Flexible Leading Edge": Path("data/.../A1 Mylar Flexible LE.csv"),
}
COMMANDED_FREQS_HZ = list(range(1, 26))   # 1–25 Hz sweep
```

**Run**
```bash
python Frequency-Lift.py
```

Outputs: `lift_comparison.png`, per-dataset `_midline.csv`, and a `_diagnostic.png` showing burst detection and peak annotation.

---

## Output Summary (`batch_all.py`)

| File | Contents |
|---|---|
| `summary.csv` | Amplitude, offset, asymmetry for every (condition, frequency) pair |
| `summary.png` | Three-panel plot: amplitude, stroke offset, asymmetry vs frequency |
| `cycle_overlays_centered.png` | Centred stroke shapes across all frequencies, one panel per condition |
| `cycle_overlays_absolute.png` | Absolute stroke angles, showing rest-position drift |

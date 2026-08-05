# ZeroLab/PICO Offline Evaluation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Implement a deterministic two-stage CLI that converts the paired PICO/ZeroLab capture into aligned 50 Hz canonical data and produces human-side agreement, timing, smoothness, and stability reports.

**Architecture:** Add focused I/O, alignment, and metric modules inside the existing `zerolab` Mod package. `canonical_cli` reads one paired capture, writes two equally sampled canonical NPZ files plus `alignment.json`; `metrics_cli` consumes those artifacts and writes JSON/CSV/Markdown reports. Existing online protocol, recorder, converter, and state-machine behavior remain unchanged.

**Tech Stack:** Python 3.10, NumPy, SciPy (`Rotation`/`Slerp`), standard-library `argparse`, `json`, `csv`, and pytest.

## Global Constraints

- Preserve the existing 992-byte ZeroLab packet protocol and `<QQ992s>` raw-record format.
- Use Ubuntu `time.monotonic` timestamps; never use terminal Enter times for alignment.
- Keep PICO and ZeroLab canonical arrays on the same 50 Hz time grid after residual-delay correction.
- Treat results as cross-device agreement/consistency; do not claim absolute accuracy without optical ground truth.
- Fail clearly on malformed inputs, failed calibration, insufficient overlap, or non-empty output directories.
- Do not modify online Sonic/PICO/ZeroLab control behavior.
- Do not stage or commit unrelated existing worktree changes.

---

### Task 1: Canonical sequence I/O and source decoders

**Files:**
- Create: `src/bxi_example_py_elf3/mods/com.bxi.sonic/zerolab/evaluation_io.py`
- Create: `src/bxi_example_py_elf3/test/test_zerolab_evaluation_io.py`
- Modify: none of `protocol.py`, `recording.py`, or `converter.py`

**Interfaces:**
- Produces `CanonicalSequence` with fields `time_s`, `frame_index`, `smpl_pose_axis_angle`, `smpl_joints_local`, `body_quat_w`, `joint_pos`, and `source`.
- Produces `load_pico_sequence(directory, rate_hz=50.0) -> CanonicalSequence`.
- Produces `load_zerolab_sequence(directory) -> CanonicalSequence` using the existing converter's 100-frame default T-pose window.
- Produces `save_canonical(path, sequence, metadata)` and `load_canonical(path)`.

- [ ] **Step 1: Write failing tests for canonical validation and PICO overlap recovery.**

```python
def test_load_pico_deduplicates_overlapping_batches(tmp_path):
    write_pico_batch(tmp_path, 0, 0.18, np.arange(10))
    write_pico_batch(tmp_path, 1, 0.20, np.arange(1, 11))
    sequence = load_pico_sequence(tmp_path)
    assert sequence.frame_index.tolist() == list(range(11))
    assert sequence.time_s.shape == (11,)
    np.testing.assert_allclose(sequence.time_s[-1] - sequence.time_s[0], 0.20)

def test_load_pico_rejects_missing_required_field(tmp_path):
    np.savez(tmp_path / "pose_000000.npz", frame_index=np.arange(10))
    with pytest.raises(ValueError, match="smpl_pose"):
        load_pico_sequence(tmp_path)
```

- [ ] **Step 2: Run the focused tests and verify they fail because the module does not exist.**

Run: `cd src/bxi_example_py_elf3 && pytest -q test/test_zerolab_evaluation_io.py`

Expected: FAIL with an import error for `zerolab.evaluation_io`.

- [ ] **Step 3: Implement strict canonical validation and PICO decoding.**

Read sorted `pose_*.npz` files with `allow_pickle=False`; require the six pose
fields plus `timestamp_monotonic`. Associate each batch timestamp with its
maximum `frame_index`, backfill earlier batch frames at `1 / rate_hz`, then
deduplicate by frame index while preserving the first occurrence. Validate
finite values, exact shapes, strictly increasing output times, and non-empty
input. Store the timestamp-reconstruction assumption in metadata returned by
the loader.

- [ ] **Step 4: Add ZeroLab decoding through existing protocol/converter APIs.**

Iterate `iter_raw_records`, call `parse_zerolab_packet`, feed packets to a new
`ZeroLabMotionConverter`, skip only the converter's expected pre-calibration
frames, and raise `ValueError` if no converted frames exist. Convert nanosecond
receive times to seconds and preserve each converted frame's source index.

- [ ] **Step 5: Implement NPZ save/load and run the focused tests.**

Run: `cd src/bxi_example_py_elf3 && pytest -q test/test_zerolab_evaluation_io.py`

Expected: PASS, including tests for ZeroLab calibration failure, truncated raw
records, finite-value validation, and canonical round-trip equality.

---

### Task 2: 50 Hz resampling and residual-delay alignment

**Files:**
- Create: `src/bxi_example_py_elf3/mods/com.bxi.sonic/zerolab/alignment.py`
- Create: `src/bxi_example_py_elf3/test/test_zerolab_alignment.py`
- Modify: `zerolab/evaluation_io.py` only if a small shared sequence helper is needed

**Interfaces:**
- Produces `AlignmentResult` with `grid_time_s`, aligned PICO/ZeroLab sequences, `estimated_delay_s`, `correlation`, and `quality_flags`.
- Produces `resample_sequence(sequence, grid_time_s) -> CanonicalSequence`.
- Produces `estimate_residual_delay(pico, zerolab, rate_hz=50.0, search_s=1.5) -> tuple[float, float]`.
- Produces `align_sequences(pico, zerolab, rate_hz=50.0, search_s=1.5, min_overlap_s=1.0) -> AlignmentResult`.

- [ ] **Step 1: Write a failing test using a known synthetic 120 ms delay.**

```python
def test_estimate_residual_delay_recovers_known_offset():
    pico = make_motion_sequence(start=0.0, delay=0.0)
    zero = make_motion_sequence(start=0.0, delay=0.12)
    delay_s, corr = estimate_residual_delay(pico, zero, rate_hz=50.0)
    assert abs(delay_s - 0.12) <= 0.02
    assert corr > 0.8
```

- [ ] **Step 2: Run the focused test and verify it fails.**

Run: `cd src/bxi_example_py_elf3 && pytest -q test/test_zerolab_alignment.py`

Expected: FAIL because the alignment module and functions are not implemented.

- [ ] **Step 3: Implement interpolation on a common 50 Hz grid.**

Use the closed overlap interval of the two input time arrays. Interpolate
axis-angle and position arrays linearly. Interpolate `body_quat_w` using
shortest-path normalized quaternion interpolation (`scipy Rotation`/`Slerp`).
Reject grids with fewer than 50 samples or non-finite input.

- [ ] **Step 4: Implement motion-energy delay estimation with the documented sign.**

Compute angular-speed magnitudes for root/shoulder/elbow/knee signals, normalize
each source, evaluate normalized correlation for integer shifts from
`-round(search_s * rate_hz)` through `+round(search_s * rate_hz)`, and return
the shift whose ZeroLab signal is later than PICO as a positive delay. Return a
low-confidence flag when the best correlation is below `0.35`.

- [ ] **Step 5: Implement final alignment and write tests for edge cases.**

Apply the positive delay by shifting ZeroLab earlier, recompute the final
intersection grid, and return equally sized canonical arrays. Add tests for
quaternion sign continuity, no overlap, overlap shorter than one second,
non-finite input, and low-correlation quality flags.

- [ ] **Step 6: Run the alignment tests.**

Run: `cd src/bxi_example_py_elf3 && pytest -q test/test_zerolab_alignment.py`

Expected: PASS, including known-delay recovery within 20 ms.

---

### Task 3: Human-side metrics and report serialization

**Files:**
- Create: `src/bxi_example_py_elf3/mods/com.bxi.sonic/zerolab/metrics.py`
- Create: `src/bxi_example_py_elf3/test/test_zerolab_metrics.py`

**Interfaces:**
- Produces `compute_timing_metrics(sequence, expected_rate_hz=50.0)`.
- Produces `compute_pose_agreement(pico, zerolab)`.
- Produces `compute_position_and_dof_agreement(pico, zerolab)`.
- Produces `compute_dynamic_agreement(pico, zerolab, rate_hz=50.0)`.
- Produces `compute_smoothness(sequence, rate_hz=50.0)` and `compute_stability(sequence, rate_hz=50.0)`.
- Produces `build_report(pico, zerolab, alignment_metadata, expected_rate_hz=50.0)` as JSON-serializable data.
- Produces `write_metrics_csv(path, report)` and `write_summary_markdown(path, report)`.

- [ ] **Step 1: Write failing tests for quaternion, position, timing, and report shapes.**

```python
def test_quaternion_error_treats_q_and_negative_q_as_equal():
    pico = make_identity_sequence()
    zero = make_identity_sequence()
    zero.body_quat_w[:] *= -1.0
    result = compute_pose_agreement(pico, zero)
    assert result["overall"]["rmse_deg"] == pytest.approx(0.0)

def test_report_marks_missing_low_motion_window():
    sequence = make_high_motion_sequence()
    result = compute_stability(sequence, rate_hz=50.0)
    assert result["status"] == "insufficient_low_motion_window"
```

- [ ] **Step 2: Run the focused tests and verify they fail.**

Run: `cd src/bxi_example_py_elf3 && pytest -q test/test_zerolab_metrics.py`

Expected: FAIL because the metrics module is absent.

- [ ] **Step 3: Implement timing and gap metrics.**

Compute interval mean/std/P95/max, rate, duration, and intervals over 30 ms.
For PICO include frame-index gaps; for ZeroLab include the raw-record counts
provided by the loader metadata.

- [ ] **Step 4: Implement pose, position, DOF, and velocity agreement.**

Use sign-invariant quaternion geodesic angles for 21 SMPL pose joints; compute
Euclidean errors for 24 local SMPL joints and scalar RMSE/P95 for 29 DOF values.
Use 50 Hz finite differences for angular and position velocity RMSE.

- [ ] **Step 5: Implement source smoothness and low-motion stability.**

Compute RMS first, second, and third finite differences for pose and second
differences for local joint positions. Detect contiguous low-motion windows of
at least 50 samples at aggregate angular speed below 5 degrees/second; return
explicit `insufficient_low_motion_window` when none qualify.

- [ ] **Step 6: Implement JSON-safe report, CSV, and Markdown serialization and run tests.**

Run: `cd src/bxi_example_py_elf3 && pytest -q test/test_zerolab_metrics.py`

Expected: PASS, with finite values, per-joint/per-DOF rows, explicit units, and
no fabricated zeros for insufficient data.

---

### Task 4: Canonical generation CLI

**Files:**
- Create: `src/bxi_example_py_elf3/mods/com.bxi.sonic/zerolab/canonical_cli.py`
- Create: `src/bxi_example_py_elf3/test/test_zerolab_canonical_cli.py`

**Interfaces:**
- CLI module `python3 -m zerolab.canonical_cli`.
- Required options: `--capture`, `--output`; optional `--rate-hz` (default `50`) and `--delay-search-s` (default `1.5`).
- Outputs exactly `pico_canonical.npz`, `zerolab_canonical.npz`, and `alignment.json` in a new output directory.

- [ ] **Step 1: Write failing CLI tests for output creation and non-empty output rejection.**

```python
def test_canonical_cli_writes_two_sequences_and_alignment(tmp_path):
    capture = make_pair_capture(tmp_path / "capture")
    result = run_cli(canonical_cli.main, ["--capture", str(capture), "--output", str(tmp_path / "out")])
    assert result == 0
    assert (tmp_path / "out" / "pico_canonical.npz").is_file()
    assert (tmp_path / "out" / "zerolab_canonical.npz").is_file()
    assert (tmp_path / "out" / "alignment.json").is_file()
```

- [ ] **Step 2: Run the CLI tests and verify they fail.**

Run: `cd src/bxi_example_py_elf3 && pytest -q test/test_zerolab_canonical_cli.py`

Expected: FAIL because `canonical_cli` is absent.

- [ ] **Step 3: Implement argument parsing and strict directory checks.**

Require `<capture>/pico` and `<capture>/zerolab`; reject an existing output
directory containing any entry; catch expected `ValueError`/`OSError` and print
one-line diagnostics to stderr with exit status 2.

- [ ] **Step 4: Orchestrate both loaders, alignment, and artifact writing.**

Load source sequences, call `align_sequences`, save equally sized aligned
canonical arrays, and write JSON metadata including timestamp reconstruction,
overlap, delay sign, correlation, calibration frame count, and quality flags.

- [ ] **Step 5: Run CLI tests and a source-tree help smoke test.**

Run:

```bash
cd src/bxi_example_py_elf3
pytest -q test/test_zerolab_canonical_cli.py
python3 -m zerolab.canonical_cli --help
```

Expected: PASS and a help message listing all options.

---

### Task 5: Metrics report CLI

**Files:**
- Create: `src/bxi_example_py_elf3/mods/com.bxi.sonic/zerolab/metrics_cli.py`
- Create: `src/bxi_example_py_elf3/test/test_zerolab_metrics_cli.py`

**Interfaces:**
- CLI module `python3 -m zerolab.metrics_cli`.
- Required options: `--pico`, `--zerolab`, `--alignment`, `--output`.
- Outputs `report.json`, `metrics.csv`, and `summary.md` in a new report directory.

- [ ] **Step 1: Write failing tests for all three report files and CSV columns.**

```python
def test_metrics_cli_writes_json_csv_and_summary(tmp_path):
    pico, zero, alignment = make_aligned_artifacts(tmp_path)
    assert metrics_cli.main(["--pico", str(pico), "--zerolab", str(zero), "--alignment", str(alignment), "--output", str(tmp_path / "report")]) == 0
    assert (tmp_path / "report" / "report.json").is_file()
    assert (tmp_path / "report" / "metrics.csv").is_file()
    assert (tmp_path / "report" / "summary.md").is_file()
```

- [ ] **Step 2: Run the focused test and verify it fails.**

Run: `cd src/bxi_example_py_elf3 && pytest -q test/test_zerolab_metrics_cli.py`

Expected: FAIL because `metrics_cli` is absent.

- [ ] **Step 3: Implement strict input/output validation and report orchestration.**

Load both canonical files, require matching 50 Hz lengths and time grids,
load `alignment.json`, call `build_report`, and write JSON/CSV/Markdown through
the serializers from `metrics.py`. Return status 2 for invalid inputs.

- [ ] **Step 4: Include quality warnings in summary output.**

Render PICO rate warnings, ZeroLab receive-rate warnings, low-correlation flags,
insufficient stability windows, and the no-ground-truth caveat. Keep values and
units in machine-readable JSON and long-form CSV.

- [ ] **Step 5: Run the CLI tests.**

Run: `cd src/bxi_example_py_elf3 && pytest -q test/test_zerolab_metrics_cli.py`

Expected: PASS.

---

### Task 6: Documentation and real-capture smoke test

**Files:**
- Modify: `src/bxi_example_py_elf3/mods/com.bxi.sonic/ZEROLAB_F2_PRO.md`
- Create: `src/bxi_example_py_elf3/test/test_zerolab_evaluation_smoke.py` (opt-in only)

**Interfaces:**
- Document the two exact commands and explain that `pair-static-013` is a
  preliminary capture with quality warnings.
- Smoke test is skipped unless `ZERO_LAB_EVAL_CAPTURE` points to a real pair
  directory, so normal CI never depends on `/tmp` hardware data.

- [ ] **Step 1: Add the documentation section and opt-in smoke test.**

The documentation must show canonical generation first, then metrics report
generation, expected output files, and the interpretation of positive delay.
The test must invoke both `main()` functions against the environment-provided
capture and assert the six expected artifacts.

- [ ] **Step 2: Run the complete offline test suite.**

Run: `cd src/bxi_example_py_elf3 && pytest -q test/test_zerolab_*`

Expected: PASS with the opt-in real-data smoke test skipped when its environment
variable is absent.

- [ ] **Step 3: Run the real smoke test on the current capture.**

Run:

```bash
cd src/bxi_example_py_elf3/mods/com.bxi.sonic
rm -rf /tmp/pair-static-013-eval
python3 -m zerolab.canonical_cli --capture /tmp/pair-static-013 --output /tmp/pair-static-013-eval
python3 -m zerolab.metrics_cli \
  --pico /tmp/pair-static-013-eval/pico_canonical.npz \
  --zerolab /tmp/pair-static-013-eval/zerolab_canonical.npz \
  --alignment /tmp/pair-static-013-eval/alignment.json \
  --output /tmp/pair-static-013-eval/report
```

Expected: all six artifacts exist, the report contains the known rate-quality
flags, and no online process or UDP port is started.

- [ ] **Step 4: Inspect the generated summary and run diff checks.**

Run:

```bash
sed -n '1,220p' /tmp/pair-static-013-eval/report/summary.md
git diff --check
```

Expected: summary clearly labels cross-device agreement rather than accuracy,
and `git diff --check` reports no whitespace errors.

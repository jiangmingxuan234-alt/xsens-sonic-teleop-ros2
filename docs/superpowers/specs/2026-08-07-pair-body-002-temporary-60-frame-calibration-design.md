# Pair Body 002 Temporary 60-Frame Calibration Design

## Goal

Generate PICO and ZeroLab canonical data for `/tmp/pair-body-002` so both
recordings can be inspected through the existing Sonic-to-MuJoCo replay path.
For this recording only, ZeroLab T-pose calibration requires 60 consecutive
stable frames instead of the production default of 100.

## Scope

- Keep the production 100-frame default unchanged.
- Do not edit the ZeroLab converter, CLI, tests, or configuration.
- Do not overwrite `/tmp/pair-body-002` or any existing replay output.
- Put all derived canonical and replay artifacts in newly created `/tmp`
  directories.

## Considered Approaches

1. **Temporary converter override (selected).** Reuse the existing loader and
   alignment implementation while substituting a converter whose calibrator
   requires 60 frames. This is isolated to one Python process and leaves the
   repository unchanged.
2. Add a permanent `--calibration-frames` CLI option. This is useful for
   repeated experiments but is unnecessary for this one recording.
3. Change the global default to 60. This would silently change online and
   offline behavior for future captures and is outside the requested scope.

## Data Flow

```text
/tmp/pair-body-002/{pico,zerolab}
  -> load PICO with the existing loader
  -> load ZeroLab with a temporary 60-frame calibrator
  -> estimate residual delay
  -> resample both streams onto the same 50 Hz overlap
  -> save pico_canonical.npz and zerolab_canonical.npz
  -> run the existing PICO and ZeroLab Sonic-to-MuJoCo replay entry points
```

The temporary canonical metadata records `calibration_frames: 60` so the
artifact cannot be mistaken for a normal 100-frame evaluation.

## Failure Handling

The conversion stops without producing a misleading replay if:

- ZeroLab still cannot collect 60 stable frames;
- the converted PICO and ZeroLab sequences have no usable overlap;
- either canonical file fails its existing loader validation; or
- either replay fails to consume a live Sonic reference.

## Verification

Before handing off viewer commands:

1. Confirm both canonical files exist and contain matching 50 Hz frame counts.
2. Confirm metadata says `calibration_frames: 60`.
3. Run 20-frame headless smoke replays for PICO and ZeroLab.
4. Require both replays to report a live reference and produce replay output.

This is a visual diagnostic only. Results created with 60-frame calibration
must not be mixed with formal 100-frame evaluation reports without being
explicitly labelled.

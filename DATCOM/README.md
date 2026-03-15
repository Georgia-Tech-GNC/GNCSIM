# Syndrome DATCOM Workflow

## What DATCOM is used for

Syndrome uses DATCOM only to generate aerodynamic tables for the body and static fins.

Those tables are saved as `.mat` files and then reused by MATLAB. `TunePID` should normally use precomputed aero, not rerun DATCOM every rollout.

Relevant directories:

- `src/DATCOM/`: DATCOM source and platform executables
- `DATCOM/cache/`: runtime DATCOM cache keyed by deck contents
- `DATCOM/precomputed/`: reusable precomputed aero files for Syndrome

## MATLAB modes

`DatcomManager` supports three aero modes:

- `runtime_cache`: run DATCOM if needed, then cache the result
- `prefer_precomputed`: load precomputed aero first, otherwise run DATCOM and save it
- `precomputed_only`: never run DATCOM; require precomputed aero files

For `TunePID`, use `precomputed_only`.

## Recommended workflow

1. Build or use a DATCOM executable for your platform.
2. Run `BuildSyndromePrecomputedAero` once after geometry or DATCOM grid changes.
3. Run `TunePID` with precomputed aero.

Regenerate precomputed aero if you change:

- body geometry
- fin geometry
- `mach_numbers`
- `altitudes_m`
- `alphas_deg`

You do not need to regenerate DATCOM files for normal Monte Carlo rollouts or for different PID gains.

## macOS

Build the native DATCOM binary:

```bash
./scripts/build_datcom_macos.sh
```

Then generate precomputed aero in MATLAB:

```matlab
BuildSyndromePrecomputedAero
```

Then run tuning:

```matlab
TunePID
```

## Linux

Use the bundled `src/DATCOM/datcom_unix` if it is compatible with your machine.

Then run:

```matlab
BuildSyndromePrecomputedAero
TunePID
```

## Windows

Use the bundled `src/DATCOM/datcom_win.exe`.

Then run:

```matlab
BuildSyndromePrecomputedAero
TunePID
```

## Notes

- `BuildSyndromePrecomputedAero` generates the exact Syndrome body/fin decks needed by the current codepath and promotes them into `DATCOM/precomputed/`.
- Parallel Monte Carlo is safe because Syndrome now uses worker-local DATCOM deck files instead of shared `BODY.dat` / `STATIC_FIN.dat` writes.
- If DATCOM is unavailable on your machine, generate `DATCOM/precomputed/` on another supported machine and copy that directory into this repo.

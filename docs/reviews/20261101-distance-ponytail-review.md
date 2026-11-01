What this change does: It keeps official Hesai FAST-LIO and LiDAR-only navigation.
It adds time-matched MPPI obstacle scoring, short motion memory, and strict benchmark
exit checks. The latest repair reduces repeated distance calculations without changing
the score and rejects incomplete mapping commands.

Assumed load: one fixed simulation at a time, 15 Hz control, 2,000 trajectories
with 56 poses each, and the frozen five-actor Arena scenarios. This is not a
hardware, full-suite, or dense-crowd acceptance review.

## Must fix

1. **Final evidence is not from the final source identity** (`integration/results/v0/timed-mppi-distance-repeat/run-provenance.json`, per-case files)

   - **What this is:** Each benchmark stores the exact source and runtime hashes used during collection.
   - **Problem:** The interim candidate passes Arena3/3 and freshly mapped BARN22, but mapping exit/resume fixes were made afterward. BARN55/77 are missing. This root cannot prove the final source passes the required six cases.
   - **Fix:** Freeze current source, collect Arena Quick3 and freshly mapped BARN Quick3 in a fresh root, stop on strict failure, and replay scores and hashes before independent final review. Do not rewrite old results or loosen NAV150.
   - **If we skip it:** A release claim would mix code versions and omit two required navigation cases.

## Checks and resolved findings

Reviewed connected native plugin exports/config/tests, LiDAR projection, motion
tracking and publishing, FAST-LIO bridge, simulated sensor generation, scoring,
provenance and benchmark callers. The independent delivery reviewer additionally
checked all owned untracked source as new code.

The distance-loop rewrite matches the original per-surface score in a multi-point,
rotated-trajectory native test. Empty clouds still score exactly zero. Frame,
layout, finite-value, speed, and timestamp checks remain enabled. No new dependency
or navigation GT input was introduced.

The independent review found mapping TIMEOUT returning zero and existing incomplete
mapping results being silently skipped. Both are fixed. Missing/empty saved PCD
now also returns failure. Focused tests execute the exit expression and real
no-ROS shell branches, including preservation of incomplete prior results.

Verification: 54 Python tests PASS, 5 native tests PASS, build PASS, installed ROS
interface smoke PASS, shell syntax and Lizard thresholds PASS. Jscpd was not installed.
Interim Arena NPZ score replay, source/runtime/archive/input hashes and TF ownership
pass; those are historical after the mapping-source changes, not current acceptance.

The interim warehouse succeeds twice (102.516s and105.272s); Arena is3/3.
BARN22 navigation succeeds in148.46s against150s, so it has only1.54s margin and
does not establish robust timing improvement. No further algorithm finding is
claimed without a failing concrete case.

Verdict: fix 1 first.

Not checked: final current-source dual-Quick3, full suites, hardware calibration,
real sensor noise and wheel dynamics. Constant-velocity motion memory lasts only
0.5s; recovery and rotation-shim actions do not use the temporal MPPI score.

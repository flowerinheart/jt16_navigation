# V1 interface diagnosis — 2026-11-07

Status: 54 diagnostic trials completed, all input/identity checks passed. V1 qualification remains BLOCK. Diagnostic reference inputs receive no acceptance credit. Frozen profile: `integration/results/v1/p143-native-retry-barn111/orchestration/nav2-params.yaml`; BARN maps from candidate O. These are not the current generator defaults.

## Results

A=baseline; B=reference localization chain; C=sensor-visible known obstacle geometry/current commanded velocity; D=B+C; E=D+scene-derived static map. Each cell is strict successes / three predeclared runs. Arena E equals D and is not repeated.

| Case | A | B | C | D | E |
|---|---|---|---|---|---|
| warehouse-seed3 | 2/3 | 0/3 | 2/3 | 2/3 | same as D; skipped |
| warehouse-seed2 | 1/3 | 1/3 | 1/3 | 2/3 | same as D; skipped |
| barn111 | 1/3 | 3/3 | 2/3 | 3/3 | 3/3 |
| barn244 | 3/3 | 3/3 | 3/3 | 3/3 | 3/3 |

- 38/54 strict passes; all 54 runs usable and identity verified before subsequent source edits. All failures, collisions and timeouts retained. G and matrix-01 unchanged.
- BARN111: reference localization B/D/E 3/3 versus baseline 1/3; C 2/3. This supports a state/coordinate-quality contribution, not an exclusive estimator cause. E adds no observed success gain over D.
- Warehouse: combined D 4/6 versus A/C 3/6 and B 1/6. Small samples and initialization timing confound interpretation; no evidence of a universal perception-only or localization-only repair. B/D start around8.3s, A/C around10.2s, shifting encounter phase by about1.9s.
- G earliest-contact replay: 14 of20 collision cases had fused translation error above0.5m before earliest static/dynamic contact. Warmup is included; this is association, not causality.
- Current-velocity reference replay: across six D runs, forecast center error P95 at2s is0.150–0.269m, at4s0.822–1.002m. Actor turns are unknown to the reference. Forecast errors alone do not establish collision causality.

## What was fixed

- Shared arena motion extraction preserves original native pose requests:35,420 comparisons equal. Diagnostic truth carries the last commanded segment velocity without reading future commands.
- Navigation startup reuses the same input checks as runtime within the existing5s budget, preventing reference localization from starting before all required actor callbacks.
- Rolling cloud message epoch now matches the first retained point after filtering. HESAI preprocessing subtracts that point timestamp; the previous earlier header shifted reconstructed acquisition times by up to20ms. Copies the header, preserves point timestamps and empty frames. One fix serves mapping and both navigation datasets, with no world/seed branches.
- Constructive epoch regression failed before and passes after;83 integration and15 diagnostic tests pass. BARN111 ROS smoke:423 frames, maximum header/first-point error0s; navigation still FAILED30.818s, zero collision/input failure. Warehouse ROS smoke:889 frames, header/first-point error0s, FAILED73.466s with zero collision/input failure. Fresh BARN111 mapping completed20/20 waypoints, valid map, input failure absent and provenance verified. These fix a measurement contract, not V1 success.

## Hardware and sensor limits

- Old ComfyUI PID2924368 was stopped by the user. SIGSTOP alone previously failed to release GPU activity. Before matrix-02 no ROCm compute PID was listed, but driver GPU busy remained100%.
- Service has Restart=always and restarted with a new PID at15:18:44 during final trials; final snapshot queue running/pending both0. These receipts cannot certify continuously idle GPU or isolate GPU-load causality. Controller deadline warnings exist despite real-time factor near1. A final service-level stop confirmed MainPID0/inactive and no ROCm compute PID, while GPU busy stayed100% in three samples. ComfyUI restored through its original unit; HTTP healthy, running/pending queues0. This rules out the live ComfyUI service as a sufficient explanation of that final busy reading, not GPU health or other graphics activity.
- No physical LiDAR/IMU/chassis experiment. Simulator transport/measurement-contract errors are distinct from real sensor defects.

## Remaining evidence needed

- For hardware attribution, obtain verified idle GPU/healthy driver, record device/process utilization continuously, then compare control deadlines under controlled load.
- For warehouse attribution, align mission start phase across diagnostic conditions and replay identical input recordings. Preserve this matrix as whole-chain closed-loop observations.
- Make observed surfaces, unknown/unmapped surfaces and forecasts with uncertainty distinct at their existing interfaces; evaluate forecast uncertainty against recorded turns before adopting a new model or risk policy.
- A supported navigation improvement still needs a fresh Dev16 gate, prescribed repeats and full evaluation. Current epoch fix smoke did not unlock BARN111; do not freeze or release a qualifying candidate.

## Evidence

`integration/results/v1/input-diagnosis-20261107/`: matrix-02/analysis.json, matrix-02/completion-check.json, matrix-02/reference-forecast-replay.json, g-first-contact-analysis.json, rolling-epoch-checks/, GPU snapshots and archived replay scripts.

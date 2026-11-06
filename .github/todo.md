# TODO

## Resolve EDIC peak detection and pressure filtering requirements

- The firmware currently applies a rolling median over three altitude samples before using altitude for the 90 m motor cutoff.
- EDIC 2026 Section 7, paragraph 2.1.1(c), calls for filtering that can recognize and reject abrupt pressure transients caused by flight dynamics, motor activation, or wind gusts.
- EDIC Section 7, paragraphs 2.3.4 and 2.4.6, says processing must preserve the maximum height represented by a single peak sample and must not add filtering or smoothing that downgrades peak detection.
- These requirements appear to pull in different directions. Treat the current median filter as provisional; do not claim compliance until the intended interpretation is clarified and the cutoff behavior is tested.
- Revisit whether transient rejection should be limited to known sensor artifacts or whether any filtering is permissible in the cutoff path. Preserve the required sample rate and single-sample peak capability.
- Source: [EDIC 2026, Section 7](https://www.fai.org/sites/default/files/2026-03/sc4_vol_edic_26.pdf).

## Evaluate refreshing the ground reference while idle

- Current behavior: capture the ground reference during startup, within the two-second initialization window, and keep that reference for the flight. There is no periodic reference update.
- Consider the proposed behavior: after initial reference capture, refresh the ground-pressure estimate in small batches (for example, five readings) while the receiver throttle remains below the motor-start threshold. The initial reference would still be captured during startup; later completed batches could update the reference while the model remains at the launch location.
- Do not delay motor start for a refresh. On the first receiver pulse above 1.2 ms, use the most recently completed valid reference, start the motor timer, and begin altitude monitoring immediately. Never commit a partial or stale refresh after motor start.
- A refresh may compensate for sensor warm-up or pressure drift before launch, but averaging alone does not remove a persistent sensor offset. It can also hide a real height change if the model is moved or its location changes; only refresh while the aircraft is known to remain at the launch point.
- Determine whether refresh cadence and sample count are worthwhile, then test reference stability, idle-to-launch response, and cutoff accuracy before implementing. The EDIC's two-second limit applies to determining the zero-height reference; confirm how any post-initialization refresh should be treated before relying on it for compliance.
- Source: [EDIC 2026, Section 7](https://www.fai.org/sites/default/files/2026-03/sc4_vol_edic_26.pdf), especially paragraphs 2.1.1, 2.4.4, and 2.5.

## Verify altitude and motor-stop accuracy

- Plan tests for the specified measurement accuracy, firmware ISA calculation accuracy, motor-stop height accuracy, and motor timing accuracy.
- Include sensor drift and temperature variation in the accuracy evaluation, including the EDIC's stated two-hour thermal-drift condition at 15 °C.
- Record the method, equipment, operating conditions, results, and any calibration evidence.

## Verify physical receiver and ESC behavior

- Test the assembled device in series between receiver and ESC.
- Confirm receiver pulse capture and pass-through over the currently accepted 800-2200 us range.
- Confirm the repeating 900 us safe output reliably keeps the specific ESC off during startup, after manual stop, and in the locked state.
- Verify motor stop and restart prevention on the actual hardware before relying on field operation.

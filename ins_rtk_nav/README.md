# INS/RTK Navigation MVP

The framework supports three selectable integration modes: LC (RTK position/velocity), TC (satellite-level TD-DDCP), and LTC (RTK position plus L1 TDCP-derived interval velocity).

- CMake/C++17 project using the existing `../eigen` headers.
- Multi-file JSON loading with deep object merge and paths resolved relative to the JSON file that declares them.
- Module registry/lifecycle and typed topic message bus.
- Whitespace-delimited IMU and GNSS text decoders.
- Rover/base GNSS epoch and SVID pairing with the configured 1 microsecond tolerance.
- Field-wise zero-as-missing handling; P2 is enabled for dual-frequency RTK through JSON; D1 retains its signed Hz value; satellite state is taken from the base record and missing base satellites/positions are skipped.
- Replay publishes `ImuSample` on `sensor.imu`, paired `PreparedGnssEpoch` on `sensor.gnss.prepared`, and data diagnostics on `dataset.summary`.
- The configured base coordinate file is parsed as the known base antenna phase-center ECEF coordinate and carried separately on each `PreparedGnssEpoch`; this is distinct from each satellite's ECEF position in the base observation row.
- RTKLIB 2.4.2 p13 core is built into the application through `RtklibAdapter`; it consumes the known base ECEF coordinate and rover/base C1, L1, L2, and D1 observations.
- `gnss_cycle_slip_preprocessor` is a separate module between rover/base pairing and navigation consumers. It estimates L1 cycle slips from Doppler/phase consistency, rejects non-integer candidates, and confirms candidates against the L1/L2 geometry-free phase. It detects single-band slips from integer-wavelength geometry-free changes and marks signal reacquisition after a phase-data gap. Confirmed per-receiver L1/L2 flags are carried on `PreparedSatellite`; RTKLIB receives LLI flags and excludes that frequency's carrier phase for the flagged epoch while retaining available code measurements. TD-DDCP drops rows that touch flagged satellite/pivot epochs and Doppler-screened outliers.
- Cycle-slip thresholds and wavelengths are configured independently in `config/preprocessing/cycle_slip.json`. Dual-frequency ambiguity mode, ratio threshold, lock count, and fix count remain in `config/navigation/rtk_solution.json`.
- The adapter translates base-record per-epoch satellite ECEF positions and clocks into an RTKLIB precise-ephemeris cache. RTKLIB's interpolator needs 11 epochs per satellite, so short missing satellite-state runs are linearly interpolated for the ephemeris cache; observations for missing satellites remain excluded.
- Dual-frequency RTK maps the dataset's P2 pseudorange through `observation_mapping.use_p2_for_dual_frequency_rtk` (currently enabled). RTKLIB needs per-frequency code observations to initialize its carrier-phase ambiguity states; with P2 omitted, the L2 ambiguity states remain uninitialized and WL/NL cannot run.
- The experimental `wide_lane_narrow_lane` ambiguity mode is implemented in `src/adapters/rtklib/wide_lane_narrow_lane.cpp`: it searches double-differenced wide-lane integers, conditions the float state/covariance on accepted WL integers, then searches narrow-lane integers with the required WL/NL parity consistency. The experiment config enables P2 and writes into a dedicated build output directory; `rtk_continuous_preview.json` is the P2-enabled continuous-LAMBDA comparison.
- `rtk_solution` publishes accepted `FLOAT` and `FIX` solutions with ECEF position/velocity, status, satellite count, ambiguity ratio, covariance, and component-validity mask. `rtk_csv_sink` writes `rtk_solution.csv`.
- During the full dataset run, position is valid for all 1,564 epochs; velocity covariance is above the configured limit for the first two epochs, so those rows carry only the position-valid bits. The mask is `0x07` for XYZ position and `0x38` for XYZ velocity.
- The RTK module caches the offline dataset and solves it after `DatasetReplayComplete` so it can build the RTKLIB 11-epoch satellite-state interpolation context. It publishes `RtkBatchComplete` after timestamped P/V observations. The selected fusion mode then replays cached IMU chronologically: LC uses RTK P/V, TC uses satellite-level TD-DDCP, and LTC uses RTK position plus an L1 TDCP WLS interval-velocity observation. Each mode writes to its own configured output folder.
- Dataset replay now merges IMU and paired GNSS epochs into event-time order before publishing.
- `td_ddcp_builder` constructs adjacent-epoch time differences of rover/base and satellite/pivot double-differenced L1/L2 carrier phase. It retains per-row satellite states, pivot identity, timestamps, shared-pivot covariance, and Doppler phase-consistency cycle-slip flags.
- The TD-DDCP model configuration is in `config/navigation/fusion_model.json`: combination mode, selected state/process/measurement model IDs, wavelengths, epoch-gap limit, phase noise, Doppler sign convention, and cycle-slip threshold.
- `td_ddcp_csv_sink` writes the cleaned measurements to `output/td_ddcp_measurements.csv` when the app is run. The remaining rows are marked as screened and have no detected cycle slip; rejected row counts are included in the builder summary.
- A review run using `config/td_ddcp_preview.json` produced 1,563 adjacent-epoch measurement sets and 27,521 satellite/band rows. With the configured signed-Doppler relation (`phase_rate = -D1`), the absolute Doppler/phase residual median was 0.09 cycles for L1 and 0.07 cycles for L2; 1,030 rows exceeded the configured 0.5-cycle screening threshold. This is a consistency screen, not a truth-based cycle-slip validation.
- INS Navigator converts the rover's initial ECEF coordinate to LLA and mechanizes the body-frame IMU stream in NED, publishing `NavigationSolution` on `navigation.ins`.
- The raw INS CSV sink writes one solution per IMU sample to the configured output directory (`ins_solution.csv` by default). The fused navigation solution sink can decimate its CSV output with `output_rate_hz`; the LC/TC/LTC `ltc_solution.csv` is configured to 1 Hz while propagation and filter updates continue at their original rates. Its attitude covariance output includes the 3D trace RMS and Euler roll, pitch, and heading 1σ values mapped from the NED left-error covariance.

The existing dataset decode summary reports 158,328 IMU samples, 15,411 rover records, 16,490 base records, 1,564 paired epochs, and 15,411 paired satellite records. No rover/base epoch or satellite-position mismatches were reported under the configured pairing rules. The first Doppler-only pass marked 475 L1 candidates, but most were not corroborated by integer-cycle changes in the independent geometry-free phase. The preprocessor therefore confirms Doppler candidates against dual-frequency phase where available, detects single-band integer jumps from geometry-free changes, and marks signal reacquisition after a phase-data gap. On the current data it reports 128 L1 and 132 L2 flags, including reacquisitions; this is a quality-screening result, not an independently validated ground-truth slip count. The clean TD-DDCP output contains 26,399 rows; the builder reports 1,141 rejected or affected phase observations, including preprocessor flags and Doppler outliers. C1-only continuous RTK remains FLOAT for all 1,564 epochs, with a maximum ratio of 1.76 versus the configured threshold 3.0. Enabling P2 for dual-frequency RTK produces 1,162 FIX and 402 FLOAT epochs in the continuous-LAMBDA run (ratio median 7.91); this establishes that L2 code availability was a major ambiguity-initialization bottleneck. WL/NL comparison is still being validated after this implementation. An instantaneous-AR run with the same clean data and ratio threshold returned FIX at only GPS seconds 63865 and 64523 in the earlier C1-only setup. The current SVID-to-GPS-PRN mapping remains unconfirmed, and centimeter-level accuracy/fix reliability must be checked against truth or an independent reference trajectory.

## Build and run (Visual Studio 2022)

```powershell
cmake -S . -B build -G "Visual Studio 17 2022" -A x64
cmake --build build --config Release
.\build\Release\ins_rtk_app.exe .\config\system.json
```

The JSON reader handles standard JSON values and Unicode escapes, recursive `include` files, and deep object merging. It does not yet implement JSON Schema validation or full source-location reporting for merged configuration errors.

## Fusion architecture and modes

The fusion pipeline is separated into modules:

- `FilterManagerModule` buffers replay input, matches observations to IMU epochs, and constructs time-aligned measurement bundles.
- `InertialNavigationModule` owns the nominal INS state, strapdown mechanization, error feedback, and final navigation output.
- `EstimationModule` owns navigation-specific process and LC/TC/LTC measurement models, prepares `Phi/Q/innovation/H/R`, and manages a filter session.
- Generic `StateEstimator` owns only the numeric state/covariance and performs prediction, Kalman update, NIS gating, and reset-Jacobian operations. It does not contain INS/GNSS or measurement-model logic.

The 18-state error vector is NED position, velocity and attitude errors, gyro and accelerometer bias errors, and body-frame IMU-to-antenna lever-arm error. To preserve the previous LC/TC/LTC models, `estimate_lever_arm` defaults to `false`; online lever-arm estimation is opt-in and requires the configured attitude-excitation threshold. Lever-arm attitude and gyro-bias Jacobian signs have been corrected. NIS gate values of `0` disable rejection, preserving the previous all-valid-epoch update policy by default.

| Config | Mode | Measurements |
|---|---|---|
| `config/mode_lc.json` | LC | RTK antenna position and valid velocity |
| `config/mode_tc.json` | TC | Satellite-level L1/L2 time-differenced double-difference carrier phase geometry; no RTK P/V |
| `config/mode_ltc.json` | LTC | RTK position plus L1 TDCP WLS interval-average velocity; RTK velocity defaults off |

Build and run all three modes from `ins_rtk_nav`:

```powershell
cmake --build build --config Release --target ins_rtk_app
.\build\Release\ins_rtk_app.exe .\config\mode_lc.json
.\build\Release\ins_rtk_app.exe .\config\mode_tc.json
.\build\Release\ins_rtk_app.exe .\config\mode_ltc.json
python .\tools\analyze_ltc_performance.py
```

Each replay writes its CSV files in `build/mode_lc`, `build/mode_tc`, or `build/mode_ltc`. The analysis creates `output/LC_TC_LTC_Performance_Report.md` and `output/lc_tc_ltc_performance.png` at the workspace root. The current full-data run produced 158,328 navigation epochs per mode, with LC accepting 1,511 RTK update epochs, TC 1,563 TD-DDCP epochs, and LTC 1,564 RTK-position plus 1,557 TDCP-velocity update epochs. The constraint-enabled configs add ZUPT/NHC while keeping the original configs available as no-constraint baselines.

The first 18-state refactor changed filter behavior by enabling lever-arm updates and NIS rejection, and by adding a variance term instead of applying the original minimum-variance floor. These were corrected: lever-arm estimation and NIS gating are now opt-in, the Jacobian signs were fixed, and the original noise-floor semantics were restored. The final position 3D 1σ is now 0.03176 m / 0.05735 m / 0.03075 m for LC/TC/LTC, close to the pre-refactor recorded 0.03174 m / 0.05717 m / 0.03061 m. LC/LTC again use all 1,564 RTK epochs. These are covariance estimates, not truth errors.

## GMIS-LC model-validation slice

An isolated `nav_gmis` C++ library contains the GMIS full-state model under `nav::gmis`; it links shared domain-agnostic `nav_core` and has no dependency on the TMIS mechanization target. `nav::core::StateEstimator` provides a nonlinear prediction overload accepting `f(x)`, its Jacobian, and process noise while preserving the existing TMIS linear API. `GmisLcEstimatorSession` combines the 21-state kinematic model with IMU and RTK P/V observation updates.

The GMIS LC convention is NED navigation, FRD body, ZYX rotation, and attitude state order `[roll, pitch, heading]`. Its reduced state is 21-dimensional: position, velocity, acceleration, attitude, angular rate, gyro bias, and accelerometer bias; sensor scale factors are not estimated. `config/gmis/gmis_lc.json` selects the Earth-aware IMU model and provides the fixed body-frame antenna lever arm. The raw gyro is modeled as body-relative-inertial rate; NED Earth rate, transport rate, normal gravity, and Coriolis acceleration are included. Local NED position uses the configured geodetic origin. The current fixed lever value `(0,0,0) m` is inherited from the earlier replay output, where lever estimation was disabled; it is not an independently calibrated value.

Centered finite-difference checks report maximum absolute Jacobian errors of `4.85e-9` (transition), `7.92e-11` (rotation partials), `1.35e-9` (simplified IMU observation), `7.48e-10` (Earth-aware IMU observation), `7.67e-10` (dynamic Earth-aware IMU observation), and `6.04e-9` (RTK P/V observation with a nonzero fixed lever arm). The static Earth-aware model returns the expected Earth-rate gyro and gravity specific force. The checks were run with:

```powershell
cmake --build build --config Release --target gmis_lc_math_validation
ctest --test-dir build -C Release --output-on-failure
```

`gmis_lc_app` is a separate runtime executable from `ins_rtk_app`. It loads the GMIS JSON profile, reads the IMU text stream and an existing RTK solution CSV, schedules RTK P/V and IMU updates through `GmisLcEstimatorSession`, and writes a 1 Hz solution/covariance/innovation CSV. Data paths, initialization seeds, fixed lever arm, and initial uncertainty are set in `config/gmis/gmis_lc.json`. RTK velocity is down-weighted when the source CSV reports weak variance. The replay remains a prototype without independent truth. GMIS_LTC, raw-GNSS replay, and module-bus integration remain future work. Existing TMIS runner/configuration and outputs remain separate.

Run it from the `ins_rtk_nav` directory after building:

```powershell
cmake --build build --config Release --target gmis_lc_app
.\build\Release\gmis_lc_app.exe .\config\gmis\gmis_lc.json
```

The experimental DynamicStatus/ZUPT/NHC pipeline can be selected with `config/mode_lc_constraints.json`, `mode_tc_constraints.json`, or `mode_ltc_constraints.json`. Add static-anchor position/heading constraints with `config/mode_lc_anchor_constraints.json`, `mode_tc_anchor_constraints.json`, or `mode_ltc_anchor_constraints.json`. ZPC and ZHC each require an accepted same-epoch ZUPT and are gated independently; rejecting ZPC does not suppress ZHC. ZPC uses the 4-state static-anchor clone. ZHC uses a separate `HeadingLockState`: after its first accepted stationary update, the INS output heading is projected to the captured value and the reported lock 1σ remains at the captured value until release. The free ESKF heading 1σ is emitted separately as `free_heading_rms_1sigma_deg`; this remains the filter's unconstrained covariance diagnostic. The three anchor-constraint modes have been replayed and summarized in `../output/ZPC_ZHC_Performance_Report.md`, including lock stability and covariance statistics. The lock holds an existing estimate and is not an independent heading observation or a truth RMSE. Noise values and dynamic thresholds are experimental and not independently validated.

TC remains a separate model issue: it has 21.89 m 3D RMS difference from RTK in the unconstrained baseline and mean NIS/dof 0.050 (the pre-refactor report recorded 0.048). TD-DDCP supplies relative motion but no absolute position measurement; the current initial position/absolute datum observability and temporal innovation correlation need review. TC-relative differences are diagnostics because RTK shares raw carrier observations, not independent truth. LTC does not retain full cross-epoch state covariance or RTK/phase cross-covariance. No independent rover truth is available, so absolute RMSE and centimeter-level accuracy are unverified. The current satellite mapping assumes GPS PRNs and requires confirmation. Full cross-correlation modeling between static constraints and GNSS/TDCP remains future work.

## INS initialization assumptions

`config/navigation/ins.json` holds INS initialization and bias values. Until measured initial values are supplied, this configuration uses zero NED velocity, zero roll/pitch/heading (level and north-aligned in NED), and zero initial gyro/accelerometer bias. Change these values for a meaningful standalone INS trajectory. The mechanization treats the decoded IMU values as angular rate (rad/s) and specific force (m/s²), uses trapezoidal sample integration with coning/sculling terms, and includes Earth rotation, transport rate, and normal gravity. The initial position is the rover GNSS antenna center; the lever arm remains zero in this step. GNSS observations are fused by RTK and the LTC measurement models; online lever-arm estimation remains future work.

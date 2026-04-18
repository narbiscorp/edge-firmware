/*******************************************************************************
 * Smart Glasses Firmware v4.13.8
 *
 * Current build: ESP32-based Narbis Edge firmware with PPG heartbeat
 * detection, on-device HRV frequency-domain analysis (coherence, band
 * powers, resp peak), BLE streaming, and lens drive control. Full
 * version history in the CHANGELOG blocks below.
 *
 * PPG pipeline: PulseSensor analog on GPIO35 (ADC1_CH7), 50 Hz sampling,
 * on-device Elgendi 2013 peak detection with raw-ADC sensor-presence
 * gate (v4.13.1), periodic detector auto-refresh every 30s (v4.13.2),
 * beat/IBI/HR streaming over BLE characteristic 0xFF04.
 *
 * Coherence pipeline (v4.13.0+): rolling 120-entry IBI ring →
 * 4Hz × 256-pt grid → Hanning-windowed radix-2 FFT → VLF/LF/HF band
 * integration → LF resonance peak (single-bin) → coherence score
 * matching client's Lehrer/Vaschillo formula. Emitted on 0xFF03
 * status char as 0xF2 packet once per second.
 *
 * Dashboard v13.10+ receives raw samples, firmware detection results,
 * and firmware coherence values; displays both client-side and
 * firmware-side HRV for cross-validation.
 * 
 * Full LED control surface: strobe, static, and breathing modes.
 * All pattern timing runs on-device via gptimer ISR (strobe) and led_task
 * 10ms tick loop (breathing). BLE app only sends configuration commands.
 *
 * Three on-device programs cycled by a short magnet tap (0.3-4s):
 *   1. BREATHE        — 6 BPM sine, lens tint follows waveform (default at boot)
 *   2. BREATHE+STROBE — 10Hz strobe whose dark-phase duty is modulated by the
 *                       breathing waveform (strobe "breathes")
 *   3. STROBE         — plain 10Hz strobe, no breathing
 * Long magnet close (>=5s) still enters deep sleep.
 *
 * BLE ADVERTISING AUTO-OFF (v4.11.0, hardened in v4.11.1):
 *   To minimize idle current when the device is used standalone (hall-only,
 *   no app), the entire BLE stack is torn down after BLE_IDLE_TIMEOUT_MS
 *   of no connection. v4.11.0 only called esp_ble_gap_stop_advertising(),
 *   which left the BT controller and Bluedroid stack running and still
 *   scheduling periodic housekeeping RF bursts (visible as sporadic ~1.5A
 *   spikes in current traces). v4.11.1 escalates: full teardown via
 *   esp_bluedroid_disable/deinit + esp_bt_controller_disable/deinit.
 *   Radio is completely off; no RF activity of any kind.
 *
 *   Expected usage: user opens arms (boots/wakes), has a 60-second window
 *   to connect from the app. If no connection occurs in that window, BLE
 *   tears down fully and the device continues running its current
 *   hall-selected program with the radio cold.
 *
 *   Re-arming: any hall event (tap or hold) re-initializes the BLE stack
 *   from scratch and starts advertising. Cost is ~100ms of init time,
 *   which is imperceptible between the user reaching for the magnet and
 *   then reaching for their phone to connect. A long close (>=5s) still
 *   enters deep sleep normally; on wake, BLE starts fresh.
 *
 *   While connected: timeout is disabled. On disconnect, a new 60s window
 *   starts so the user can reconnect if they want.
 *   During OTA: timeout is blocked (same pattern as sleep blocking).
 *
 *   Idle current: ~8mA during active session with BLE down (just ESP32
 *   + lens drive). During the 60s advertising window after boot/wake/
 *   disconnect, idle is ~15mA (v4.9.12 advertising config).
 * 
 * FEATURES:
 * - AC differential drive: GPIO26/GPIO27 via gptimer 100µs ISR (100Hz AC,
 *   phase-synced to strobe edges for beat-free operation)
 * - Three LED modes: STROBE, STATIC, BREATHE
 * - DDS strobe: gptimer hardware ISR, ±100µs precision, sub-Hz to 50Hz
 * - Breathing engine: cosf sine / linear, configurable BPM, ratio, holds
 * - OTA firmware updates via BLE (deferred, with write drain)
 * - Power saving: -6dBm adv / -12dBm connected, 100-200ms adv interval
 * 
 * BLE COMMANDS (multi-byte to 0xFF01):
 * 
 *   COMMON:
 *   - A2 [brightness]     Set max tint 0-100%
 *   - A4 [minutes]        Set session duration 1-60 min
 *   - A7 00               Sleep immediately
 * 
 *   MODE SWITCHING:
 *   - A5 [duty]           Enter STATIC mode at given duty 0-100%
 *   - A6 00               Enter STROBE mode
 *   - B0 00               Enter BREATHE mode
 * 
 *   STROBE PARAMS (take effect immediately):
 *   - AB [freq]           Set strobe frequency 1-50 Hz
 *   - AC [duty_pct]       Set strobe duty cycle 10-90% (% of period dark)
 * 
 *   BREATHE PARAMS (take effect immediately):
 *   - B1 [bpm]            Set breathing rate 1-30 BPM
 *   - B2 [pct]            Set inhale ratio 10-90%
 *   - B3 [val]            Set hold-at-top 0-50 (×100ms, max 5s)
 *   - B4 [val]            Set hold-at-bottom 0-50
 *   - B5 [wave]           Waveform: 0=sine, 1=linear
 * 
 *   OTA:
 *   - A8 00               Start OTA mode
 *   - A9 00               Finish OTA (validate + reboot)
 *   - AA 00               Cancel OTA
 *   - AD 01               Confirm page (write buffer to flash)
 *   - AD 00               Reject page (discard, resend)
 * 
 * LEGACY: Single byte 0x00-0xFF → static mode at byte*100/255
 * 
 * CHANGELOG v4.13.8:
 * - Coherence denominator reverted to VLF+LF+HF. My v4.13.5 change to
 *   "full spectrum total" was based on misreading the client code.
 *   Actually reading dashboard v13.10 reveals:
 *
 *     const psd = lombScargle(times, centered, freqs);  // Lomb-Scargle, not FFT
 *     freqs = 0.003..0.4 Hz, 200 points                  // only this range
 *     for(i=0..199) { tp += psd[i]*df; ... }             // tp over 0.003-0.4 only
 *
 *   Client never includes frequencies above 0.4 Hz in tp. My v4.13.5-v4.13.7
 *   firmware summed bins 1-127 (covering up to 2 Hz Nyquist), making the
 *   denominator 2-3× too large and driving coherence systematically low
 *   (FW=3 vs client=12 in Devon's v4.13.7 test).
 *
 *   Correct denominator is bins 1-25 (0.016-0.391 Hz, essentially the
 *   Task Force VLF+LF+HF total). Same total used for coherence and for
 *   the displayed "Total Power" field.
 *
 *   Note: client uses Lomb-Scargle on irregular IBI series; firmware uses
 *   FFT on uniformly-resampled series. These methods produce similar but
 *   not identical PSDs. Expect residual ±5 coherence agreement even with
 *   all other bugs fixed. Porting Lomb-Scargle to firmware is possible
 *   but high CPU/memory cost for marginal improvement.
 *
 * CHANGELOG v4.13.7:
 * - Coherence peak window narrowed from ±1 bin (3 bins) to single bin.
 *   Client code:
 *     if(Math.abs(freqs[i]-resPeakF)<=0.015) resP+=psd[i]*df;
 *   With df = 4/256 = 0.015625 Hz, adjacent bins are 0.015625 Hz from
 *   the peak — GREATER than 0.015, so excluded. Client's effective
 *   window is just the peak bin itself. My v4.13.4-v4.13.6 firmware
 *   summed ±1 bin (3 bins total), making the numerator 2-3× too large
 *   and inflating coherence — Devon saw FW=100 vs client=44. Fixed.
 *
 *   This is the third coherence-math iteration. For posterity, the
 *   cumulative formula evolution was:
 *     v4.13.0-3:  ratio² × 250 × N (single bin, squared, × 256)
 *     v4.13.4:    ratio × 250       (±1 bin window, Task Force denom)
 *     v4.13.5-6:  ratio × 250       (±1 bin window, full-spectrum denom)
 *     v4.13.7:    ratio × 250       (single bin, full-spectrum denom)
 *
 * CHANGELOG v4.13.6:
 * - Bumped PPG_RAW_AC_MIN from 10 to 200. Field test with sensor
 *   plugged in but no finger confirmed the PulseSensor amplifier
 *   self-oscillates, producing ~150-200 code p-p swing that looked
 *   enough like signal to clear the too-loose gate. Beats were
 *   being emitted with wildly varying IBIs (400-1200ms) which then
 *   inflated coherence via bogus LF content.
 *
 *   Real finger pulses produce 300-1000+ code swings — there's a
 *   clear discrimination line. 200 catches no-finger amplifier noise
 *   while leaving margin for weak-contact cases.
 *
 *   If weak-but-valid pulses get rejected, can lower to 150 or add a
 *   secondary discriminator (e.g. recent IBI coefficient-of-variation
 *   check to reject noise-beat distributions while accepting real
 *   weak-but-consistent pulses).
 *
 * CHANGELOG v4.13.5:
 * - Coherence denominator fix. Client computes tp as sum of psd over
 *   ALL frequency bins (0 to Nyquist), not just VLF+LF+HF. On noisy
 *   sessions, bins above 0.4 Hz carry substantial energy from ectopic
 *   beats, detection jitter, and interpolation artifacts — energy that
 *   dilutes the "peak concentration" ratio and correctly reduces
 *   coherence. Firmware was excluding all that energy, making coherence
 *   look artificially high (Devon saw FW=100 vs client=30 on real data).
 *   Now uses full-spectrum total for coherence while keeping Task Force
 *   VLF+LF+HF for the displayed "Total Power" field.
 *
 * CHANGELOG v4.13.4:
 * - Coherence math now matches client (Lehrer/Vaschillo LF resonance
 *   method, as implemented in dashboard v13.10). Two bugs in one:
 *
 *     1. Was computing ratio² × 250. Client computes ratio × 250.
 *        Squaring made the score far too low for typical non-paced
 *        breathing — a ratio of 0.13 (reasonable for normal HRV)
 *        produced firmware=4 vs client=19 in field testing.
 *
 *     2. Was using single-bin peak power. Client sums power across
 *        ±1 bin window around peak (captures the peak's shoulders).
 *        Real peaks span 2-3 bins at our 0.0156 Hz resolution;
 *        single-bin systematically under-reports.
 *
 *   Both fixed. Firmware coherence should now land within ~1-2
 *   points of client. Coh Agreement metric validates.
 *
 * CHANGELOG v4.13.3:
 * - Surface actual CPU frequency in FW Log heartbeat. Previously the
 *   boot-time "CPU clock: XXMHz" message used ESP_LOGI which only
 *   reaches UART, not the BLE FW Log tab where we actually look.
 *   Also, that message fires at boot before BLE is even up, so it
 *   wouldn't reach a dashboard client anyway.
 *   Now the 5-second heartbeat line includes cpu=NNMHz sourced from
 *   rtc_clk_cpu_freq_get_config(). You'll see it within 5s of
 *   connecting. If it says 80MHz you're getting the power savings;
 *   if it says 160MHz or 240MHz sdkconfig changes didn't take.
 *
 * CHANGELOG v4.13.2:
 * - Coherence formula fix. v4.13.0-v4.13.1 had `coherence = ratio² × 250
 *   × COH_GRID_N` which made every value 256× too large and saturated
 *   at 100. Python simulation (which validated the algorithm) used just
 *   × 250 — firmware now matches. Expect Coh Agreement with client to
 *   land within a few points.
 *
 * - Reset function fix. ppg_reset_detector() did not reset
 *   ppg_sample_count, so after manual/BLE reset the warmup period
 *   didn't re-engage. Under-filled MA_beat vs filled MA_peak produced
 *   a burst of false positives for ~660ms after every reset —
 *   effectively making the "Reset FW Detector" button worse than
 *   useless. Now forces full 1s warmup re-engage after reset.
 *
 * - Periodic auto-reset every 30s. Over long noisy sessions (motion
 *   artifact on connected sensor, not caught by gate since signal
 *   has plausible DC+AC), the adaptive MA_beat baseline drifts up to
 *   accommodate the noise floor. Eventually any tiny burst clears
 *   threshold and produces false beats. Hard-resetting every 30s
 *   gives the baseline a fresh start and prevents this drift.
 *   Auto-reset skipped if in_block to avoid interrupting real beats.
 *
 * CHANGELOG v4.13.1:
 * - Sensor-presence gate. Field data confirmed the failure mode: with
 *   PulseSensor plugged into the board but not on skin, raw ADC sits
 *   in a plausible midrail range with ~200 code p-p swing (electrical
 *   noise from nearby traces). The squared-bandpass signal clears
 *   PPG_MIN_MA_BEAT=4.0 easily, so ghost beats are emitted with
 *   nonsense IBIs (600-1900ms jumping around, fake HR 40bpm etc).
 *
 *   Fix: gate ppg_detect on raw ADC signature via the existing
 *   adc_stats_ring (25-sample 500ms window). A physically-connected
 *   sensor pressed against skin holds near midrail with a bounded
 *   and consistent pulse envelope. Parameters chosen from observed
 *   data (raw=0 rail-low on disconnect, raw ~1800-2100 no-skin,
 *   raw ~1700-2200 with pulse):
 *     PPG_RAW_DC_MIN = 300   (catches rail-low disconnect)
 *     PPG_RAW_DC_MAX = 3900  (catches rail-high disconnect)
 *     PPG_RAW_AC_MIN = 10    (catches flat floats; 10 leaves margin
 *                             for weak contact with loose fit)
 *   Dropped the AC_MAX upper bound from the reviewer's suggestion —
 *   motion rejection is a separate concern and the proposed 3000
 *   threshold would never fire in practice.
 *
 *   On gate fail: ppg_in_block, ppg_above_run, and ppg_last_beat_ms
 *   all cleared so reconnection starts cleanly. Also clears the
 *   coherence IBI ring so pre/post-disconnect beats don't glue together
 *   into a nonsense time series for frequency-domain analysis.
 *
 *   PPG_MIN_MA_BEAT retained as secondary check (catches the "raw
 *   looks plausible but no pulse energy" case).
 *
 *   New ble_log field 'gate' in heartbeat line shows whether gate is
 *   currently admitting samples.
 *
 * CHANGELOG v4.13.0:
 * - MAJOR: Coherence & frequency-domain HRV now computed on-device.
 *   Previously all frequency-domain math (coherence, VLF/LF/HF bands,
 *   resp freq) was dashboard-side only. This created two problems:
 *   (a) values lagged during browser/BLE issues, (b) can't drive lens
 *   modulation from coherence without firmware knowing the score.
 *
 *   New architecture:
 *     - Rolling 120-entry IBI ring buffer (~2min at 60bpm)
 *     - coherence_task @ priority 4, fires once per second
 *     - Resamples IBI series onto 4Hz × 256 sample grid (64s window)
 *     - Detrend + Hanning window + 256-pt radix-2 FFT
 *     - Integrates VLF (0.003-0.04 Hz), LF (0.04-0.15 Hz),
 *       HF (0.15-0.4 Hz) band powers
 *     - Finds LF resonance peak
 *     - Computes coherence = peak² / total² × 250, clamped 0-100
 *     - Emits new 0xF2 status packet with all values
 *
 *   Self-contained FFT (Cooley-Tukey radix-2) — no esp-dsp dependency,
 *   so build system untouched. At 80MHz CPU a 256-pt FFT takes ~3ms.
 *   Coherence task total ~5ms per second = negligible CPU.
 *
 *   Memory: ~5KB static (IBI ring + FFT bufs + Hanning precompute).
 *
 *   Values emitted in 0xF2 packet (18 bytes):
 *     [0xF2][coherence u8][resp_peak u16 mHz][vlf u16][lf u16][hf u16]
 *     [total u16][lf_norm u8][hf_norm u8][lf_hf u16 fp8.8][n_ibis u8]
 *     [reserved u16]
 *
 *   Dashboard v13.10+ will parse and display. Future v4.14.0 will
 *   tie the coherence value into a LED_MODE_COHERENCE_PACER that
 *   modulates lens tint by real-time coherence score.
 *
 * CHANGELOG v4.12.9:
 * - CPU frequency dropped 240MHz → 80MHz for ~20mA power savings.
 *   Workload (50Hz PPG detection) uses <1% CPU at 80MHz so no perf
 *   hit. BLE requires 80MHz minimum APB clock so we can't go lower.
 *
 *   Runtime approach: attempts esp_pm_configure if CONFIG_PM_ENABLE
 *   is set, falls back to rtc_clk_cpu_freq_set_config direct call
 *   otherwise. If neither works (verify via "CPU clock: 80MHz" log
 *   line at boot), set CONFIG_ESP32_DEFAULT_CPU_FREQ_MHZ=80 in
 *   sdkconfig as a guaranteed boot-time setting.
 *
 * CHANGELOG v4.12.8:
 * - Power reduction. v4.12.5 tightened BLE connection interval to
 *   7.5-15ms for throughput, but this drove average current from
 *   ~40mA to ~100mA (radio waking 100×/sec instead of 5×/sec).
 *   With dashboard v13.8+ smoothing buffer absorbing bursty delivery,
 *   we can back off the interval. New 20-30ms with latency=1 should
 *   land around 50-60mA average while still giving 3-6× better
 *   throughput than the v4.12.4 baseline.
 *
 * CHANGELOG v4.12.7:
 * - Build fix. v4.12.6 had ppg_reset_detector() referenced in
 *   process_command (0xD0 handler at line ~1668) before its definition
 *   with the PPG module (line ~2306), causing implicit-declaration
 *   error. Added forward declaration near the GLOBAL STATE block.
 * - Cleanup: swap deprecated ADC_ATTEN_DB_11 → ADC_ATTEN_DB_12
 *   (ESP-IDF notes they're identical; new name is preferred).
 *   Removed unused gpios[] in scan helper and stale
 *   ppg_jitter_window_start declaration.
 *
 * CHANGELOG v4.12.6:
 * - Beefed-up stuck-state recovery. v4.12.5 watchdog only cleared
 *   ppg_in_block after 2.5s of silence — it didn't touch the moving
 *   averages. When a burst of noise elevates ma_beat, the threshold
 *   (ma_beat × 1.05) becomes high enough that real beats can't cross
 *   it until the W2=33-sample (660ms) window fully decays. During
 *   that window the detector appears "stuck" — no beats emitted while
 *   real beats are visible on-screen.
 *
 *   Fix: after 3s of silence with MAb-above-floor (signal is alive),
 *   also zero the MA buffers. Detector starts fresh at the current
 *   signal level. Recovery time goes from ~1.5s to ~250ms.
 *
 * - New BLE command 0xD0 — manual detector reset. Dashboard can send
 *   this whenever it notices firmware detection is falling behind.
 *   Clears all detector state (MA buffers, in_block, run counter,
 *   last beat time). Keeps cumulative counter (beat_count_total).
 *
 * CHANGELOG v4.12.5:
 * - Detector robustness pass for noisy (PC-connected) environments.
 *   Field data showed MAp toggling sample-to-sample even with the
 *   v4.12.4 narrow-bandpass filter, causing the firmware detector to
 *   miss real beats and emit false ones. Three minimal changes:
 *
 *   1. PPG_ALPHA_Q10 raised 21 → 51 (α ≈ 0.02 → 0.05). The 2% threshold
 *      margin was Elgendi-paper-optimal for clean clinical ECG. On
 *      noisy PPG the detector needs wider margin to not cross on
 *      residual HF energy. 5% is the center of the commonly cited
 *      Elgendi range for PPG (0.04-0.06).
 *
 *   2. PPG_BLOCK_ENTRY_MIN — require 2 consecutive above-threshold
 *      samples before entering in-block state. Previously a single
 *      spurious crossing entered a block, then immediately exited on
 *      the next sample, causing "toggling" between in/out. This kills
 *      the toggling directly. Real systolic peaks stay above the
 *      threshold for 5-7 samples; this requirement costs nothing for
 *      true beats.
 *
 *   3. BLE connection interval tightened. Was 40-60ms with latency=4
 *      (2 events/sec ACK'd, 5+ samples piling up per event). Now
 *      7.5-15ms with latency=0. Expected outcome: ble_err growth drops
 *      from ~5/sec to near zero, and dashboard "Gap max" should drop
 *      from ~120ms toward ~30ms. Tradeoff: slightly higher phone power
 *      use during connection, irrelevant for a wall-powered test rig
 *      or a desk session.
 *
 * CHANGELOG v4.12.4:
 * - Mains interference rejection. Field data from PC-connected testing
 *   revealed 60Hz powerline hash riding on the PPG signal, far stronger
 *   than battery-powered tablet testing. At 50Hz sample rate, 60Hz
 *   mains aliases to 10Hz, which WAS in the v4.12.0-3 passband (0.5-8Hz
 *   stopband wasn't tight enough to kill the alias). Two fixes:
 *
 *   1. Bandpass narrowed to 0.5-4Hz. Covers 30-240 BPM (physiological
 *      range) with comfortable margin. Much tighter stopband against
 *      mains alias. Edge detail slightly softer, but for peak DETECTION
 *      (not waveform display) this is a clear win. Recomputed scipy
 *      Butterworth coefficients accordingly.
 *
 *   2. ADC oversampling changed from MEAN to MEDIAN of 8 reads. Mean
 *      averages noise spikes in — a single 60Hz zero-crossing during
 *      the 8-read burst contaminates all 8 reads. Median rejects
 *      outliers: up to 3 of 8 reads can be mains-corrupted and the
 *      median still returns a clean sample. Cost: tiny (sort 8 ints,
 *      ~30 cycles).
 *
 * - Known residual noise path: the bodge wire on pin 11 of the QFN is
 *   inherently a small antenna. For production, route PulseSensor
 *   signal with proper trace + ground pour. For now the filter pair
 *   above makes the firmware tolerant of the noise this bodge picks
 *   up in mains-heavy environments (e.g., PC desks with USB hubs).
 *
 * CHANGELOG v4.12.3:
 * - CRITICAL FIX: PPG ADC pin was wrong since v4.12.0. Defaulted to
 *   GPIO36 (SENSOR_VP) but on this PCB the PulseSensor is bodge-wired
 *   to physical pin 11 = GPIO35 (ADC1_CH7). This was the choice from
 *   the original PCB review because pin 11 has NC neighbors on both
 *   sides (GPIO34 above, 32K_XP below), making it the safest bodge
 *   target on the left edge of the QFN.
 *   Symptom: raw=0 streaming from the firmware while the detector
 *   dutifully ran on zeros. Fixed by switching PPG_ADC_GPIO to
 *   GPIO_NUM_35 and PPG_ADC_CHANNEL to ADC1_CHANNEL_7.
 *
 * CHANGELOG v4.12.2:
 * - Diagnostic telemetry pass. Motivated by field report that sample
 *   rate degrades from 50 Hz to ~30 Hz over a few minutes (oscillating
 *   not steady-declining), suggesting periodic CPU starvation or BLE
 *   backpressure.
 *
 * - Per-sample jitter measurement in ppg_task. Each tick records the
 *   actual delta between wake-ups and tracks max jitter over a 5-second
 *   window. Published via ble_log. If the task is being preempted we'll
 *   see jitter spikes well above 20ms.
 *
 * - ADC scan mode. BLE cmd 0xC0 0x00 enters a diagnostic mode that
 *   scans all 8 ADC1 channels (GPIO 32-39) every 500ms and emits the
 *   reads via ble_log. Lets us find which pin actually has the
 *   PulseSensor signal without reflashing. Revert with 0xC0 0x01 or
 *   reboot.
 *
 * - Task health report. Every 10 seconds, ble_log reports: free heap,
 *   min free heap, number of tasks, CPU idle percentage (from Task Watchdog
 *   idle counter), and current ppg_task high-water mark. Tells us if
 *   heap is leaking, stack is near overflow, or idle-CPU is dropping.
 *
 * - BLE send error counter. ppg_send_sample and ADC stats emit both
 *   track the return value of esp_ble_gatts_send_indicate. Errors
 *   (buffer full) are counted and reported in the health log. If BLE
 *   is backpressuring, this counter will be non-zero.
 *
 * - Raised ppg_task priority from 3 to 10 (same tier as the OTA task)
 *   to make it unambiguously higher-priority than led_task (1) and
 *   hall_task (2, when enabled). At 50Hz with a hard timing requirement
 *   this task should preempt almost everything.
 *
 * CHANGELOG v4.12.1:
 * - New LED_MODE_PULSE_ON_BEAT. Lens briefly flashes dark on every
 *   detected heartbeat. 150ms pulse width, 80% tint peak, cosine decay
 *   envelope. Driven by ppg_task setting a volatile deadline tick that
 *   led_task reads on its 10ms tick. BLE command 0xB6 0x00 enters this
 *   mode, replacing whatever mode was active (same pattern as A5/A6/B0).
 * - Live ADC telemetry over 0xFF03 (status characteristic) — firmware
 *   now streams periodic ADC diagnostics (every 500ms): min/max/mean of
 *   the last 25 raw ADC reads. Lets the dashboard show whether the
 *   sensor is alive at the electrical level, independent of detection.
 *   Status format byte 0 = 0xF0 (ADC_STATS), bytes 1-10 follow.
 * - BLE-echoed ESP_LOGI output. New helper ble_log() sends a formatted
 *   string on 0xFF03 with leading byte 0xF1. Dashboard parses it and
 *   shows it in a new "Firmware Log" debug tab. Lets us debug hardware
 *   issues without needing a USB serial connection.
 * - Detector: added MIN_MA_BEAT floor. When the moving-average envelope
 *   is below 4.0 (noise floor for a disconnected/dead sensor), detection
 *   is suppressed. Prevents false beats from ADC noise.
 *
 * CHANGELOG v4.12.0:
 * - PPG input path added. PulseSensor analog → GPIO36 (ADC1_CH0, input-only
 *   SENSOR_VP pin, so no conflict with existing PWM/hall GPIOs). 50 Hz
 *   sampling in a new ppg_task (priority 3, between led_task and hall_task).
 *   8× oversampling per sample period for noise reduction (each ADC read
 *   is ~30µs so 8× costs ~240µs of the 20ms tick — negligible).
 * - On-device detection pipeline matches the v13.1 dashboard:
 *     * 2nd-order Butterworth bandpass 0.5–8 Hz (biquad, cascaded form)
 *     * Elgendi 2013 two-moving-averages (W1=6 ~111ms, W2=33 ~667ms)
 *     * α=0.02 threshold offset (static for firmware — adaptive α and
 *       template matching are refinements the dashboard owns)
 *     * Refractory max(300ms, 0.6·IBI) as backstop
 *   Detection latency: ~100–150ms (no template lookahead, unlike v13.1).
 * - New BLE characteristic 0xFF04 (PPG stream) with notify. Packet format
 *   is 13 bytes: [type=0x02][raw_u16][idx_u16][ts_u32][flags][ibi_u16][bpm].
 *   flags bit 0 = beat, bit 1 = in_block. Stream rate = 50 Hz when a client
 *   has subscribed (CCCD write 0x0001). Off otherwise — no idle RF cost.
 * - GATTS_NUM_HANDLE 10 → 12 to accommodate the new char + CCCD.
 * - Coexists with all existing modes (strobe/static/breathe/breathe_strobe)
 *   without interaction. Future LED_MODE_PULSE_ON_BEAT (v4.13.x) will read
 *   beat events directly from a volatile set by the detector.
 *
 * CHANGELOG v4.11.1:
 * - BLE auto-off hardened: full BT stack teardown instead of just stopping
 *   advertising. v4.11.0 left the BT controller and Bluedroid running,
 *   which scheduled periodic housekeeping RF bursts (~1.5A spikes visible
 *   in current traces every few seconds). v4.11.1 tears down via
 *   esp_bluedroid_disable + esp_bluedroid_deinit +
 *   esp_bt_controller_disable + esp_bt_controller_deinit. Radio is cold.
 * - Added ble_stack_init() and ble_stack_teardown() helpers. Both are
 *   idempotent; state tracked via ble_stack_up flag.
 * - Re-arm path now re-initializes the full stack. Cost is ~100ms,
 *   acceptable in the user-facing latency budget (tap magnet, wait,
 *   open app, connect).
 * - Handle invalidation: gatts_if_global, service_handle, char_handles
 *   all become invalid across teardown. They get repopulated by the GATT
 *   event handler when the new GATT app registers. Code that uses these
 *   (notifications, indicate) already guards on is_connected, which is
 *   false while BLE is down.
 *
 * CHANGELOG v4.11.0:
 * - BLE advertising auto-off after 60 seconds of no connection. Cuts idle
 *   current from ~15mA (advertising at -6dBm, 100-200ms interval, inherited
 *   from v4.9.12) back down to ~11mA baseline when device is used standalone
 *   via hall-only. Matches the standalone-first UX that the hall program
 *   cycling (v4.10.0) is built around.
 * - Behavior: on boot or wake from deep sleep, advertising starts with a
 *   60s deadline. If no client connects, advertising is stopped via
 *   esp_ble_gap_stop_advertising(). BT controller and stack stay up so
 *   re-arming is instant. Any hall event (tap for program advance, or
 *   start of a long hold) resets the deadline and restarts advertising if
 *   it was off. While connected: deadline is disabled. On disconnect:
 *   new 60s window starts. During OTA: timeout is blocked.
 * - Trade: user must perform a hall gesture to reconnect after 60s of
 *   standalone use. Acceptable given the design intent (standalone OR
 *   connected, not both simultaneously for long periods).
 * - Note: the webapp side must be updated to handle the case where the
 *   device is running but not advertising — connect attempts will fail
 *   with "device not found," same as if it were deep-sleeping. User guidance:
 *   "tap the magnet once to wake BLE, then connect."
 * - Implementation: deadline checked in app_main 1Hz loop (already runs for
 *   session-end sleep). Single volatile tick deadline, zero new tasks.
 *
 * CHANGELOG v4.10.1:
 * - BLE supervision timeout 4s -> 20s. The 4-second .timeout=400 value
 *   was firing during esp_ota_begin(OTA_SIZE_UNKNOWN) partition erase
 *   (6-19s of radio silence), causing the phone to declare the link
 *   dead mid-erase and closing the connection before OTA_STATUS_READY
 *   could be sent. Symptom: "GATT operation failed" ~4s after 0xA8.
 * - 20s is well under the BLE spec limit (32s max) and still recovers
 *   quickly if the device genuinely walks out of range.
 * - No measurable power impact — supervision timeout only governs how
 *   long the central waits during anomalous silence, not normal traffic.
 * - Note: connection params are a *request* from peripheral to central.
 *   Phones may apply immediately, delay, or ignore entirely. Explains
 *   why pre-v4.10.1 OTAs worked intermittently — attempts using phone's
 *   default (often 20+ seconds) succeeded; attempts after phone applied
 *   the 4s request failed.
 *
 * CHANGELOG v4.10.0:
 * - Startup program changed from 10Hz strobe to 6 BPM sine breathing.
 * - Added hall-sensor program cycling. Short close (0.3s-4s) advances
 *   through three on-device programs: BREATHE, BREATHE+STROBE, STROBE.
 *   Long close (>=5s) still enters deep sleep, same as before.
 * - New LED mode LED_MODE_BREATHE_STROBE: 10Hz strobe where dark-phase
 *   duty is scaled by the breathing waveform (makes the strobe "breathe").
 *   ISR reads a volatile breathe_frac_q8 updated by led_task each 10ms tick.
 * - Hall polling moved off the 1Hz main loop into a dedicated 50ms task
 *   so 0.3s gestures are detectable. 50ms debounce on edges. Gesture is
 *   decided on release: short release = advance, 5s of continuous HIGH
 *   = sleep. 4-5s release window is dead-zone (no action).
 * - BLE commands (A5/A6/B0 etc.) still work and override the physical
 *   program. Hall advance always cycles the physical programs regardless
 *   of current BLE-set mode.
 *
 * CHANGELOG v4.9.12:
 * - BLE connection reliability pass, paired with webapp retry loops
 *   (controller v4.9.4, OTA v13). Addresses intermittent GATT errors
 *   that previously required multiple manual reconnect attempts.
 * - Advertising TX power: -12dBm → -6dBm. Roughly doubles link margin
 *   at the phone (+6dB ≈ 4× signal strength) so advertising packets
 *   and the connection handshake arrive well above the phone's noise
 *   floor. Biggest single contributor to unreliable first-try connects
 *   was marginal RSSI during handshake, not advertising rate.
 * - Connection TX power: remains at -12dBm. Once connected the link
 *   is established and weaker signal is sufficient — keeps per-session
 *   power draw close to v4.9.11 baseline.
 * - Advertising interval: 200-320ms → 100-200ms. Halves time between
 *   advertising events so phone scan windows align with ours sooner,
 *   cutting typical connect latency. Power cost is modest (see below).
 * - Est. idle current impact: ~+1-2mA average vs v4.9.11 (roughly
 *   +0.5-1mA from TX power bump during adv-only radio-on windows,
 *   +0.5-1mA from doubled advertising rate). Active session current
 *   essentially unchanged since connected TX stays at -12dBm.
 * - Boot banner updated to reflect new power/interval settings.
 *
 * CHANGELOG v4.9.11:
 * - AC frequency back to 100Hz (best lens charging / strongest tint)
 * - Added AC phase-sync: at each strobe clear→dark transition, AC phase is
 *   reset with alternating polarity. Every dark burst sees identical drive
 *   magnitude (no beat artifacts at any strobe freq), while alternating
 *   start polarity maintains DC balance across burst pairs.
 * - Previous versions traded tinting for beat rejection via high AC freq;
 *   phase-sync separates those concerns entirely.
 ******************************************************************************/

#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_sleep.h"
#include "esp_timer.h"
#include "esp_crc.h"
#include "driver/gptimer.h"
#include "driver/adc.h"     /* v4.12.0: PPG ADC input (legacy API — simple, stable) */
#include "esp_heap_caps.h"  /* v4.12.2: health report — free heap etc. */
#include "esp_pm.h"         /* v4.12.9: CPU frequency / power management */
#include "soc/rtc.h"        /* v4.12.9: rtc_clk_cpu_freq_set_config fallback */
#include "soc/ledc_struct.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/*******************************************************************************
 * VERSION AND IDENTIFICATION
 ******************************************************************************/
#define FIRMWARE_VERSION "4.13.8"
static const char *TAG = "SG_v4.13.8";

/*******************************************************************************
 * BUILD MODE (v4.12.0)
 *
 * PPG_TEST_BUILD gates the hall sensor entirely:
 *   1 = test/bench build. hall_task is not started, hall GPIO is not
 *       configured. The magnet does NOTHING — no program cycling, no
 *       sleep, no side effects. Device stays awake indefinitely on
 *       whatever program it booted into (default: BREATHE). All BLE
 *       commands still work. Session timer still applies (sleep on
 *       session end) — use A4 60 to set a 60-minute session if you
 *       want to avoid even that.
 *   0 = production build. Full hall sensor behavior restored: short
 *       tap advances program, 5s hold enters deep sleep.
 *
 * Flip to 0 for production flashes. Leave at 1 while bringing up PPG.
 ******************************************************************************/
#define PPG_TEST_BUILD          1

/*******************************************************************************
 * BLE CONFIGURATION - ACTIVE MODE (Power Optimized)
 ******************************************************************************/
#define GATTS_SERVICE_UUID      0x00FF
#define GATTS_CHAR_UUID_CTRL    0xFF01  /* Control commands */
#define GATTS_CHAR_UUID_OTA     0xFF02  /* OTA data chunks */
#define GATTS_CHAR_UUID_STATUS  0xFF03  /* Status notifications */
#define GATTS_CHAR_UUID_PPG     0xFF04  /* v4.12.0: PPG stream (raw + detected beats) */
#define GATTS_NUM_HANDLE        12      /* Service + 4 chars + 2 CCCDs (v4.12.0: was 10 for 3 chars + 1 CCCD) */

#define DEVICE_NAME             "Narbis_Edge"
#define GATTS_APP_ID            0

/* v4.9.12: Advertising interval 100-200ms (was 200-320ms in v4.9.11).
 * Halving the interval gets advertising packets in front of the phone's
 * scan window sooner, cutting typical first-try connect latency. Combined
 * with webapp retry loops (controller v4.9.4 / OTA v13) this should
 * deliver reliable first-attempt connects in most conditions.
 * Encoding: value × 0.625ms = interval in ms.
 *   0x0A0 = 160 × 0.625ms = 100ms
 *   0x140 = 320 × 0.625ms = 200ms */
#define ADV_INT_MIN             0x0A0
#define ADV_INT_MAX             0x140

/* BLE idle advertising timeout (v4.11.0).
 * If no connection arrives within this window after boot/wake/disconnect,
 * advertising is stopped to save the ~4mA of idle PA burst current from
 * the v4.9.12 advertising config (-6dBm, 100-200ms interval). Any hall
 * event (tap or hold edge) restarts advertising and resets the window. */
#define BLE_IDLE_TIMEOUT_MS     60000

/*******************************************************************************
 * HARDWARE CONFIGURATION
 ******************************************************************************/
#define HALL_PIN                GPIO_NUM_4

/* v4.12.0: PulseSensor analog input — CORRECTED in v4.12.3.
 *
 * PulseSensor Vout is bodge-wired to physical pin 11 of the WROOM-32
 * module, which is GPIO35 (ADC1_CHANNEL_7). From the earlier PCB review:
 * pin 11 was the left-edge choice because it's NC on both adjacent pins
 * (GPIO34 above, 32K_XP below), input-only (so no risk of accidental
 * drive), and on ADC1 (works with WiFi even though we don't use it).
 *
 * GPIO35, like GPIO34/36/39, is input-only and lacks internal pull-up
 * or pull-down — that's fine because PulseSensor drives a stable
 * ~1.65V midrail Vout.
 *
 * EARLIER v4.12.0–v4.12.2 WRONGLY DEFAULTED TO GPIO36. GPIO36 is
 * NO_CONNECT on this PCB, which is why raw=0 was streaming — the ADC
 * was reading a floating pad with no sensor connection. */
#define PPG_ADC_GPIO            GPIO_NUM_35
#define PPG_ADC_CHANNEL         ADC1_CHANNEL_7
#define PPG_SAMPLE_RATE_HZ      50
#define PPG_TICK_MS             (1000 / PPG_SAMPLE_RATE_HZ)  /* 20ms */
#define PPG_OVERSAMPLE          8     /* Average N ADC reads per sample — cheap AA filter */

/* Hall gesture detection (v4.10.0):
 * Short close = advance program, long close = sleep.
 * Poll at HALL_POLL_MS so short gestures are catchable.
 * On release: if duration in [SHORT_MIN, SHORT_MAX) → advance. Otherwise ignore.
 * While held: if duration reaches HALL_LONG_MS → sleep immediately. */
#define HALL_POLL_MS            50
#define HALL_DEBOUNCE_MS        50      /* Debounce on both edges */
#define HALL_SHORT_MIN_MS       300     /* Ignore < 300ms (noise) */
#define HALL_SHORT_MAX_MS       4000    /* Short gesture upper bound */
#define HALL_LONG_MS            5000    /* Long hold → sleep */

/* PWM Channel 1 - GPIO27 */
#define PWM1_TIMER              LEDC_TIMER_0
#define PWM1_MODE               LEDC_LOW_SPEED_MODE
#define PWM1_OUTPUT_IO          27
#define PWM1_CHANNEL            LEDC_CHANNEL_0
#define PWM1_DUTY_RES           LEDC_TIMER_10_BIT
#define PWM1_FREQUENCY          10000   /* 10kHz carrier */

/* PWM Channel 2 - GPIO26 */
#define PWM2_TIMER              LEDC_TIMER_0
#define PWM2_MODE               LEDC_LOW_SPEED_MODE
#define PWM2_OUTPUT_IO          26
#define PWM2_CHANNEL            LEDC_CHANNEL_1
#define PWM2_DUTY_RES           LEDC_TIMER_10_BIT
#define PWM2_FREQUENCY          10000   /* 10kHz carrier */

/*******************************************************************************
 * TIMING CONFIGURATION
 * 
 * FreeRTOS runs at 100Hz (CONFIG_FREERTOS_HZ=100), so 1 tick = 10ms.
 * AC + strobe: gptimer 100µs hardware ISR (DDS phase accumulator for strobe)
 * Breathing: led_task 10ms tick loop sets effective_duty
 ******************************************************************************/
#define AC_PERIOD_TICKS         1       /* 10ms tick for led_task (breathing mode) */
#define LED_TICK_MS             10      /* LED task tick for breathe mode */
#define AC_HALF_TICKS           50      /* 50 × 100µs = 5ms = 100Hz AC */
#define PHASE_FULL              100000U /* DDS wrap: 10000 ticks/sec × 10 for deci-Hz */
#define DEFAULT_SESSION_MIN     10      /* 10 minute session */
#define DEFAULT_BRIGHTNESS      100     /* 100% brightness */
#define DEFAULT_STROBE_DHZ      100       /* 10Hz default strobe (deci-Hz) */
#define MIN_STROBE_HZ           1
#define MAX_STROBE_HZ           50

/* OTA page buffer size — must match web app PAGE_SIZE */
#define OTA_PAGE_SIZE           4096

/*******************************************************************************
 * LCD DEADZONE COMPENSATION
 * 
 * The LCD lens doesn't respond below ~40% raw PWM duty.
 * Duty 0 = raw 0 (fully clear)
 * Duty 1-100 = raw 400-1023 (visible range)
 ******************************************************************************/
#define LCD_DEADZONE_RAW        400
#define PWM_MAX_RAW             1023

static inline uint32_t duty_to_raw(uint8_t duty_percent) {
    if (duty_percent == 0) return 0;
    if (duty_percent > 100) duty_percent = 100;
    return LCD_DEADZONE_RAW + (duty_percent * (PWM_MAX_RAW - LCD_DEADZONE_RAW) / 100);
}

/*******************************************************************************
 * LED MODE DEFINITIONS
 ******************************************************************************/
typedef enum {
    LED_MODE_STROBE         = 0,
    LED_MODE_STATIC         = 1,
    LED_MODE_BREATHE        = 2,
    LED_MODE_BREATHE_STROBE = 3,  /* v4.10.0: strobe with breathing-modulated duty */
    LED_MODE_PULSE_ON_BEAT  = 4,  /* v4.12.1: lens flash on each detected heartbeat */
} led_mode_t;

/*******************************************************************************
 * PHYSICAL PROGRAMS (v4.10.0)
 * 
 * Hall-sensor-selectable programs cycled by short magnet tap.
 * Each program maps to a specific led_mode + parameter set.
 ******************************************************************************/
typedef enum {
    PROG_BREATHE         = 0,  /* Startup default */
    PROG_BREATHE_STROBE  = 1,
    PROG_STROBE          = 2,
    PROG_COUNT           = 3,
} program_t;

/*******************************************************************************
 * GLOBAL STATE
 ******************************************************************************/
/* Session state */
static volatile bool session_active = false;
static volatile uint8_t brightness = DEFAULT_BRIGHTNESS;
static volatile uint32_t session_duration_ms = DEFAULT_SESSION_MIN * 60 * 1000;
static volatile uint32_t session_start_tick = 0;

/* LED mode — v4.10.0: startup now BREATHE (Program 1), was STROBE.
 * v4.12.1: in PPG_TEST_BUILD mode, start in PULSE_ON_BEAT so the lens
 * actually responds to detected heartbeats immediately on boot. */
#if PPG_TEST_BUILD
static volatile led_mode_t led_mode = LED_MODE_PULSE_ON_BEAT;
#else
static volatile led_mode_t led_mode = LED_MODE_BREATHE;
#endif

/* Current physical program (hall-selectable). Starts at PROG_BREATHE. */
static volatile program_t current_program = PROG_BREATHE;

/* v4.12.2: forward-declare diagnostic state that's referenced by BLE
 * command handlers (which are defined earlier in the file than the PPG
 * module body where these live). Using a tentative definition (no
 * initializer) here; the actual initialized definition lives with the
 * PPG module. Valid C: multiple tentative definitions of the same
 * static are merged into one by the linker. */
static bool adc_scan_enabled;

/* v4.12.7 forward declaration. ppg_reset_detector is defined with the
 * PPG module later in the file but referenced by process_command (BLE
 * cmd 0xD0 handler). Must be declared before first use. */
static void ppg_reset_detector(void);

/* v4.13.1 forward declaration. coh_clear is defined with the coherence
 * module but called from ppg_detect when the sensor-presence gate
 * trips. */
static void coh_clear(void);

/* BLE auto-off state (v4.11.0, expanded v4.11.1).
 * ble_adv_active: tracks whether advertising is currently running.
 * ble_idle_deadline_tick: FreeRTOS tick when BLE should tear down if
 *   still not connected. 0 = deadline disabled (while connected, or
 *   after it has already been consumed and BLE is down).
 * ble_stack_up: tracks whether the BT controller + Bluedroid are
 *   initialized and enabled. False means full radio-off. Guards the
 *   teardown/init helpers to make them idempotent. */
static volatile bool ble_adv_active = false;
static volatile uint32_t ble_idle_deadline_tick = 0;
static volatile bool ble_stack_up = false;

/* Strobe parameters */
static volatile uint16_t strobe_dhz = DEFAULT_STROBE_DHZ;   /* Deci-Hz */
static volatile uint8_t strobe_duty_pct = 50;               /* 10-90% of period dark */
static volatile uint32_t strobe_dark_thresh = PHASE_FULL * 50 / 100;  /* Cached threshold */

/* Breathe parameters */
static volatile uint8_t breathe_bpm        = 6;    /* 1-30 BPM */
static volatile uint8_t breathe_inhale_pct = 40;   /* 10-90% of cycle is inhale */
static volatile uint8_t breathe_hold_top   = 0;    /* 0-50, units of 100ms */
static volatile uint8_t breathe_hold_bot   = 0;    /* 0-50, units of 100ms */
static volatile uint8_t breathe_wave       = 0;    /* 0=sine, 1=linear */

/* AC drive state - shared between tasks */
static volatile uint8_t effective_duty = 0;

/* Breathing fraction for ISR consumption (v4.10.0).
 * 0..255 representing 0.0..1.0 of the breath cycle.
 * Written by led_task @10ms, read by drive_timer_cb @100µs.
 * Used by LED_MODE_BREATHE_STROBE to scale the strobe dark duty. */
static volatile uint8_t breathe_frac_q8 = 0;

/* v4.12.1: beat-pulse state for LED_MODE_PULSE_ON_BEAT.
 * beat_pulse_until_tick — absolute FreeRTOS tick when the current
 *   pulse ends. Set by ppg_task on beat detection; read by led_task.
 * PULSE_DURATION_MS — total pulse duration (cosine decay inside).
 * PULSE_PEAK_DUTY    — maximum tint at t=0 of the pulse. */
#define PULSE_DURATION_MS       150
#define PULSE_PEAK_DUTY         80
static volatile uint32_t beat_pulse_start_tick = 0;

/* Unified drive timer state (gptimer 100µs ISR — AC alternation + strobe) */
static gptimer_handle_t drive_timer = NULL;
static volatile uint32_t ac_tick = 0;
static volatile uint8_t ac_phase = 0;
static volatile uint32_t strobe_acc = 0;  /* DDS phase accumulator */

/* Strobe→AC phase-sync state (v4.9.11):
 * At each clear→dark strobe transition, AC phase is reset with alternating
 * polarity so every dark burst sees identical-magnitude AC drive (no beat),
 * while consecutive bursts start with opposite polarity (DC-balanced). */
static volatile uint8_t strobe_was_dark = 0;
static volatile uint8_t ac_reset_polarity = 0;

/* OTA state */
static volatile bool in_ota_mode = false;
static const esp_partition_t *ota_partition = NULL;
static esp_ota_handle_t ota_handle = 0;
static uint32_t ota_bytes_written = 0;

/* Page-based OTA transfer state */
static uint8_t ota_page_buf[OTA_PAGE_SIZE];
static uint16_t ota_page_offset = 0;
static uint16_t ota_page_num = 0;
static bool ota_page_pending = false;

/* OTA task — deferred execution to avoid blocking BLE callback */
typedef enum {
    OTA_TASK_NONE = 0,
    OTA_TASK_BEGIN,
    OTA_TASK_FINISH,
    OTA_TASK_CANCEL,
} ota_task_cmd_t;

static volatile ota_task_cmd_t ota_pending_cmd = OTA_TASK_NONE;
static TaskHandle_t ota_task_handle = NULL;

/* BLE handles */
static uint16_t gatts_if_global = ESP_GATT_IF_NONE;
static uint16_t conn_id_global = 0;
static uint16_t service_handle = 0;
static uint16_t ctrl_char_handle = 0;
static uint16_t ota_char_handle = 0;
static uint16_t status_char_handle = 0;
static uint16_t ppg_char_handle = 0;               /* v4.12.0 */
static uint16_t cccd_handle = 0;                   /* CCCD for status (0xFF03) */
static uint16_t ppg_cccd_handle = 0;               /* v4.12.0: CCCD for PPG (0xFF04) */
static uint8_t  gatts_init_step = 0;               /* v4.12.0: tracks which CCCD we're adding */
static bool notifications_enabled = false;         /* Status notifications (0xFF03) */
static bool ppg_notifications_enabled = false;     /* v4.12.0: PPG notifications (0xFF04) */
static bool is_connected = false;

/* Task handles */
static TaskHandle_t led_task_handle = NULL;
static TaskHandle_t ppg_task_handle = NULL;   /* v4.12.0 */

/* Forward declarations for BLE auto-off helpers (v4.11.0/v4.11.1).
 * Defined after adv_params so they can reference it. Declared here so
 * hall_task and app_main (which live above adv_params) can call them. */
static void ble_adv_rearm(void);
static void ble_adv_reset_deadline(void);
static esp_err_t ble_stack_init(void);
static esp_err_t ble_stack_teardown(void);

/* Forward declarations for the GAP/GATT event handlers (v4.11.1).
 * ble_stack_init() calls esp_ble_gap_register_callback(gap_event_handler)
 * and esp_ble_gatts_register_callback(gatts_event_handler), both of which
 * are defined further down in the file. Without these forward decls the
 * v4.11.1 build fails with "undeclared identifier" on both names. */
static void gap_event_handler(esp_gap_ble_cb_event_t event,
                              esp_ble_gap_cb_param_t *param);
static void gatts_event_handler(esp_gatts_cb_event_t event,
                                esp_gatt_if_t gatts_if,
                                esp_ble_gatts_cb_param_t *param);

/*******************************************************************************
 * PWM FUNCTIONS
 ******************************************************************************/
static void pwm_init(void) {
    /* Configure timer (shared by both channels) */
    ledc_timer_config_t timer_conf = {
        .speed_mode       = PWM1_MODE,
        .duty_resolution  = PWM1_DUTY_RES,
        .timer_num        = PWM1_TIMER,
        .freq_hz          = PWM1_FREQUENCY,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_conf));

    /* Configure PWM1 channel (GPIO27) */
    ledc_channel_config_t pwm1_conf = {
        .speed_mode     = PWM1_MODE,
        .channel        = PWM1_CHANNEL,
        .timer_sel      = PWM1_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = PWM1_OUTPUT_IO,
        .duty           = 0,
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&pwm1_conf));

    /* Configure PWM2 channel (GPIO26) */
    ledc_channel_config_t pwm2_conf = {
        .speed_mode     = PWM2_MODE,
        .channel        = PWM2_CHANNEL,
        .timer_sel      = PWM2_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = PWM2_OUTPUT_IO,
        .duty           = 0,
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&pwm2_conf));
    
    ESP_LOGI(TAG, "PWM initialized: GPIO26 + GPIO27, 10kHz, 10-bit");
}

static void pwm1_set_raw(uint32_t raw_duty) {
    ledc_set_duty(PWM1_MODE, PWM1_CHANNEL, raw_duty);
    ledc_update_duty(PWM1_MODE, PWM1_CHANNEL);
}

static void pwm2_set_raw(uint32_t raw_duty) {
    ledc_set_duty(PWM2_MODE, PWM2_CHANNEL, raw_duty);
    ledc_update_duty(PWM2_MODE, PWM2_CHANNEL);
}

static void pwm_both_off(void) {
    pwm1_set_raw(0);
    pwm2_set_raw(0);
}

/*******************************************************************************
 * ISR-SAFE PWM WRITES (direct LEDC register access, IRAM-resident)
 * 
 * The LEDC driver functions (ledc_set_duty etc.) live in flash, not IRAM,
 * so they crash if called from hardware ISR context. These write LEDC
 * registers directly.
 * 
 * ESP32: LEDC_LOW_SPEED_MODE = group index 1
 * PWM1 = channel 0 (GPIO27), PWM2 = channel 1 (GPIO26)
 ******************************************************************************/
static void IRAM_ATTR pwm1_set_isr(uint32_t raw_duty) {
    LEDC.channel_group[1].channel[0].duty.duty = raw_duty << 4;
    LEDC.channel_group[1].channel[0].conf1.duty_start = 1;
    LEDC.channel_group[1].channel[0].conf0.low_speed_update = 1;
}

static void IRAM_ATTR pwm2_set_isr(uint32_t raw_duty) {
    LEDC.channel_group[1].channel[1].duty.duty = raw_duty << 4;
    LEDC.channel_group[1].channel[1].conf1.duty_start = 1;
    LEDC.channel_group[1].channel[1].conf0.low_speed_update = 1;
}

static inline uint32_t IRAM_ATTR duty_to_raw_isr(uint8_t duty_pct) {
    if (duty_pct == 0) return 0;
    if (duty_pct > 100) duty_pct = 100;
    return 400 + (duty_pct * 623 / 100);
}

/*******************************************************************************
 * DRIVE TIMER (gptimer hardware timer, 100µs ISR)
 * 
 * True hardware interrupt — zero RTOS involvement, zero scheduling jitter.
 * Handles both AC electrode alternation and strobe phase in one ISR.
 * 
 * AC: toggle every AC_HALF_TICKS (50 × 100µs = 5ms = 100Hz). In strobe mode,
 *   AC phase is reset at each clear→dark transition with alternating polarity
 *   so every dark burst is identical in magnitude with DC balance preserved.
 * Strobe: DDS phase accumulator. Phase wraps at PHASE_FULL (100000).
 *   Increment per tick = strobe_dhz (deci-Hz). Average frequency is exact.
 *   Max per-cycle error = ±100µs.
 * 
 * Breathing/static: ISR reads effective_duty set by led_task (10ms loop).
 ******************************************************************************/
static bool IRAM_ATTR drive_timer_cb(gptimer_handle_t timer,
                                      const gptimer_alarm_event_data_t *edata,
                                      void *user_ctx) {
    /* ── OTA / session override ── */
    if (in_ota_mode || !session_active) {
        effective_duty = 0;
    }

    /* ── Strobe DDS phase accumulator with AC phase-sync ── */
    if ((led_mode == LED_MODE_STROBE || led_mode == LED_MODE_BREATHE_STROBE)
        && session_active && !in_ota_mode) {
        strobe_acc += strobe_dhz;
        if (strobe_acc >= PHASE_FULL) strobe_acc -= PHASE_FULL;

        uint8_t is_dark = (strobe_acc < strobe_dark_thresh) ? 1 : 0;

        /* On clear→dark transition: reset AC phase with alternating polarity.
         * This makes every dark burst identical in magnitude (no beat) while
         * alternating consecutive bursts keeps DC balance across burst pairs. */
        if (is_dark && !strobe_was_dark) {
            ac_tick = 0;
            ac_phase = ac_reset_polarity;
            ac_reset_polarity ^= 1;
        }
        strobe_was_dark = is_dark;

        if (led_mode == LED_MODE_BREATHE_STROBE) {
            /* Scale dark-phase duty by current breathing fraction (0..255).
             * At top of breath: full brightness dark bursts.
             * At bottom: dark bursts nearly invisible. Clear phase unchanged. */
            uint32_t scaled = (uint32_t)brightness * breathe_frac_q8 / 255;
            effective_duty = is_dark ? (uint8_t)scaled : 0;
        } else {
            effective_duty = is_dark ? brightness : 0;
        }
    }

    /* ── AC alternation ── */
    ac_tick++;
    if (ac_tick >= AC_HALF_TICKS) {
        ac_tick = 0;
        ac_phase = !ac_phase;
    }

    /* ── Apply PWM with AC phase ── */
    uint32_t raw = duty_to_raw_isr(effective_duty);
    if (ac_phase == 0) {
        pwm1_set_isr(raw);
        pwm2_set_isr(1);
    } else {
        pwm1_set_isr(1);
        pwm2_set_isr(raw);
    }

    return false;  /* no task wake needed */
}

static void strobe_start(void) {
    strobe_acc = 0;
    strobe_was_dark = 0;
    ac_reset_polarity = 0;
    strobe_dark_thresh = PHASE_FULL * strobe_duty_pct / 100;
    ESP_LOGI(TAG, "Strobe: %d.%dHz %d%% duty (phase-sync AC)",
             strobe_dhz / 10, strobe_dhz % 10, strobe_duty_pct);
}

static void strobe_stop(void) {
    /* Timer keeps running for AC drive.
     * Strobe stops because led_mode != LED_MODE_STROBE/BREATHE_STROBE. */
}

static void strobe_update(void) {
    strobe_dark_thresh = PHASE_FULL * strobe_duty_pct / 100;
    if ((led_mode == LED_MODE_STROBE || led_mode == LED_MODE_BREATHE_STROBE)
        && session_active) {
        strobe_start();
    }
}

/*******************************************************************************
 * PHYSICAL PROGRAM APPLY (v4.10.0)
 *
 * Sets led_mode + strobe state for a given program. Called from hall-gesture
 * handler. Safe to call even before session_active — strobe_start() is gated.
 ******************************************************************************/
static void apply_program(program_t p) {
    current_program = p;
    switch (p) {
        case PROG_BREATHE:
            strobe_stop();
            led_mode = LED_MODE_BREATHE;
            /* Clear any residual strobe duty immediately; led_task will
             * overwrite effective_duty on next 10ms tick with the waveform. */
            effective_duty = 0;
            ESP_LOGI(TAG, "Program 1: BREATHE %d BPM %s",
                     breathe_bpm, breathe_wave == 0 ? "sine" : "linear");
            break;

        case PROG_BREATHE_STROBE:
            led_mode = LED_MODE_BREATHE_STROBE;
            if (session_active) strobe_start();
            ESP_LOGI(TAG, "Program 2: BREATHE+STROBE %d BPM + %d.%dHz %d%% duty",
                     breathe_bpm, strobe_dhz / 10, strobe_dhz % 10, strobe_duty_pct);
            break;

        case PROG_STROBE:
            led_mode = LED_MODE_STROBE;
            if (session_active) strobe_start();
            ESP_LOGI(TAG, "Program 3: STROBE %d.%dHz %d%% duty",
                     strobe_dhz / 10, strobe_dhz % 10, strobe_duty_pct);
            break;

        default:
            break;
    }
}

static void drive_timer_init(void) {
    gptimer_config_t cfg = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000,  /* 1MHz = 1µs resolution */
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&cfg, &drive_timer));

    gptimer_alarm_config_t alarm = {
        .alarm_count = 100,        /* 100µs period */
        .reload_count = 0,
        .flags.auto_reload_on_alarm = true,
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(drive_timer, &alarm));

    gptimer_event_callbacks_t cbs = {
        .on_alarm = drive_timer_cb,
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(drive_timer, &cbs, NULL));
    ESP_ERROR_CHECK(gptimer_enable(drive_timer));
    ESP_ERROR_CHECK(gptimer_start(drive_timer));

    ESP_LOGI(TAG, "Drive timer started (100µs gptimer ISR / 100Hz AC, phase-synced)");
}

/*******************************************************************************
 * OTA STATUS NOTIFICATIONS
 ******************************************************************************/
#define OTA_STATUS_READY        0x01
#define OTA_STATUS_PROGRESS     0x02
#define OTA_STATUS_SUCCESS      0x03
#define OTA_STATUS_ERROR        0x04
#define OTA_STATUS_CANCELLED    0x05

#define OTA_ERR_BEGIN           0x01
#define OTA_ERR_WRITE           0x02
#define OTA_ERR_END             0x03
#define OTA_ERR_NOT_IN_OTA      0x04
#define OTA_ERR_PARTITION       0x05

/* Page-based CRC transfer status codes (sent via 0xFF03 notify) */
#define OTA_STATUS_PAGE_CRC     0x06    /* [0x06, page_hi, page_lo, crc3..crc0] (7 bytes) */
#define OTA_STATUS_PAGE_OK      0x07    /* [0x07, page_hi, page_lo] (3 bytes) */
#define OTA_STATUS_PAGE_RESEND  0x08    /* [0x08, page_hi, page_lo] (3 bytes) */

/* Page confirmation command (received on 0xFF01) */
#define OTA_CMD_PAGE_CONFIRM    0xAD    /* [0xAD, 0x01]=commit, [0xAD, 0x00]=resend */

static void send_ota_status(uint8_t status, uint8_t extra1, uint8_t extra2, uint8_t extra3) {
    if (!notifications_enabled || !is_connected) return;
    
    uint8_t notify_data[4] = {status, extra1, extra2, extra3};
    size_t len = (status == OTA_STATUS_PROGRESS) ? 4 : 
                 (status == OTA_STATUS_ERROR) ? 2 : 1;
    
    esp_ble_gatts_send_indicate(gatts_if_global, conn_id_global, 
                                 status_char_handle, len, notify_data, false);
}

/* Send arbitrary-length OTA status notification (for page CRC etc.) */
static void send_ota_status_raw(uint8_t *data, size_t len) {
    if (!notifications_enabled || !is_connected) return;
    esp_ble_gatts_send_indicate(gatts_if_global, conn_id_global,
                                 status_char_handle, len, data, false);
}

/*******************************************************************************
 * BLE LOG + ADC STATS (v4.12.1)
 *
 * ble_log(fmt, ...) — printf-style, emits on 0xFF03 (status char) with a
 *   leading type byte 0xF1 followed by the formatted string. Dashboard
 *   parses this into a "Firmware Log" debug tab. Max 48 chars of payload
 *   to stay within the BLE MTU and keep notify cost low.
 *
 * ADC stats — periodic summary of the last N raw ADC readings (min, max,
 *   mean). Emitted from ppg_task every 500ms. Type byte 0xF0, 11 bytes.
 *   Purpose: let the dashboard show whether the sensor is alive at the
 *   electrical level, completely independent of the detection pipeline.
 *   This is the key diagnostic for "is my PulseSensor actually plugged in
 *   and producing signal" — you can see raw ADC min/max even when
 *   detection is producing nothing or garbage.
 ******************************************************************************/
#include <stdarg.h>

static void ble_log(const char *fmt, ...) {
    if (!notifications_enabled || !is_connected) return;
    uint8_t pkt[64];
    pkt[0] = 0xF1;   /* Type: firmware log string */
    va_list args;
    va_start(args, fmt);
    int n = vsnprintf((char *)&pkt[1], sizeof(pkt) - 1, fmt, args);
    va_end(args);
    if (n < 0) return;
    if (n > (int)(sizeof(pkt) - 1)) n = sizeof(pkt) - 1;
    esp_ble_gatts_send_indicate(gatts_if_global, conn_id_global,
                                status_char_handle, 1 + n, pkt, false);
}

/* Ring of recent ADC reads for stats — updated by ppg_task inline,
 * summarized by ppg_emit_adc_stats() below. 25 samples = 500ms at 50Hz. */
#define ADC_STATS_N         25
static uint16_t adc_stats_ring[ADC_STATS_N];
static uint8_t  adc_stats_idx = 0;
static uint8_t  adc_stats_count = 0;

static void adc_stats_push(uint16_t v) {
    adc_stats_ring[adc_stats_idx] = v;
    adc_stats_idx = (adc_stats_idx + 1) % ADC_STATS_N;
    if (adc_stats_count < ADC_STATS_N) adc_stats_count++;
}

static void ppg_emit_adc_stats(void) {
    if (!notifications_enabled || !is_connected) return;
    if (adc_stats_count == 0) return;

    uint16_t mn = 4095, mx = 0;
    uint32_t sum = 0;
    for (int i = 0; i < adc_stats_count; i++) {
        uint16_t v = adc_stats_ring[i];
        if (v < mn) mn = v;
        if (v > mx) mx = v;
        sum += v;
    }
    uint16_t mean = (uint16_t)(sum / adc_stats_count);

    /* Packet: [0xF0][min u16][max u16][mean u16][count][reserved u2][reserved u2]
     *  total 11 bytes */
    uint8_t pkt[11];
    pkt[0]  = 0xF0;
    pkt[1]  = (uint8_t)(mn & 0xFF);
    pkt[2]  = (uint8_t)((mn >> 8) & 0xFF);
    pkt[3]  = (uint8_t)(mx & 0xFF);
    pkt[4]  = (uint8_t)((mx >> 8) & 0xFF);
    pkt[5]  = (uint8_t)(mean & 0xFF);
    pkt[6]  = (uint8_t)((mean >> 8) & 0xFF);
    pkt[7]  = adc_stats_count;
    pkt[8]  = 0;
    pkt[9]  = 0;
    pkt[10] = 0;
    esp_ble_gatts_send_indicate(gatts_if_global, conn_id_global,
                                status_char_handle, sizeof(pkt), pkt, false);
}

/*******************************************************************************
 * LED CONTROL TASK
 * 
 * Session management + breathing engine. Unified timer handles AC drive and
 * strobe. led_task handles: session timeout, breathing mode duty computation,
 * OTA pause, status logging.
 ******************************************************************************/
static void led_task(void *param) {
    uint32_t tick_count = 0;
    uint32_t last_log_tick = 0;
    
    ESP_LOGI(TAG, "LED task started - mode %d", led_mode);
    
    /* Start session */
    session_active = true;
    session_start_tick = xTaskGetTickCount();
    if (led_mode == LED_MODE_STROBE || led_mode == LED_MODE_BREATHE_STROBE) {
        strobe_start();
    }
    
    while (1) {
        /* Check session timeout */
        uint32_t elapsed = (xTaskGetTickCount() - session_start_tick) * portTICK_PERIOD_MS;
        if (elapsed >= session_duration_ms) {
            ESP_LOGI(TAG, "Session ended after %lu minutes", elapsed / 60000);
            session_active = false;
            strobe_stop();
            effective_duty = 0;
            break;
        }
        
        /* Handle OTA mode - pause everything */
        if (in_ota_mode) {
            strobe_stop();
            effective_duty = 0;
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        
        /* ── BREATHE / BREATHE_STROBE MODE — compute frac every 10ms tick ──
         * Both modes share the same waveform math. BREATHE drives effective_duty
         * directly (lens tint follows waveform). BREATHE_STROBE publishes
         * breathe_frac_q8 for the ISR to scale strobe dark-phase duty. */
        if (led_mode == LED_MODE_BREATHE || led_mode == LED_MODE_BREATHE_STROBE) {
            uint32_t cycle_ms = 60000 / breathe_bpm;
            uint32_t hold_top_ms = (uint32_t)breathe_hold_top * 100;
            uint32_t hold_bot_ms = (uint32_t)breathe_hold_bot * 100;
            
            if (hold_top_ms + hold_bot_ms >= cycle_ms) {
                hold_top_ms = 0;
                hold_bot_ms = 0;
            }
            
            uint32_t breathing_ms = cycle_ms - hold_top_ms - hold_bot_ms;
            if (breathing_ms < 200) breathing_ms = 200;
            
            uint32_t inhale_ms = breathing_ms * breathe_inhale_pct / 100;
            uint32_t exhale_ms = breathing_ms - inhale_ms;
            if (inhale_ms < 100) inhale_ms = 100;
            if (exhale_ms < 100) exhale_ms = 100;
            
            uint32_t t = (tick_count * LED_TICK_MS) % cycle_ms;
            float frac = 0.0f;
            
            if (t < inhale_ms) {
                float p = (float)t / (float)inhale_ms;
                frac = (breathe_wave == 0) 
                    ? (1.0f - cosf((float)M_PI * p)) / 2.0f
                    : p;
            }
            else if (t < inhale_ms + hold_top_ms) {
                frac = 1.0f;
            }
            else if (t < inhale_ms + hold_top_ms + exhale_ms) {
                float p = (float)(t - inhale_ms - hold_top_ms) / (float)exhale_ms;
                frac = (breathe_wave == 0)
                    ? (1.0f + cosf((float)M_PI * p)) / 2.0f
                    : 1.0f - p;
            }
            /* else: hold at bottom, frac = 0 */

            /* Publish for ISR (BREATHE_STROBE scales dark duty by this). */
            breathe_frac_q8 = (uint8_t)(frac * 255.0f);

            if (led_mode == LED_MODE_BREATHE) {
                effective_duty = (uint8_t)(frac * (float)brightness);
            }
            /* BREATHE_STROBE: ISR sets effective_duty from breathe_frac_q8 */
        }

        /* v4.12.1: PULSE_ON_BEAT mode. ppg_task writes beat_pulse_start_tick
         * each time a beat is detected. We read that tick here on the 10ms
         * led_task tick and compute the current pulse envelope.
         *
         * Envelope: cosine half-cycle from full tint to zero over
         * PULSE_DURATION_MS. Result: a brief visible flash timed to each
         * heartbeat. If no beat has occurred recently (or ever), lens
         * stays clear.
         *
         * Why envelope-style and not square pulse: a square ON/OFF flash
         * looks jarring on the electrochromic lens which has its own
         * response time. A cosine decay matches the lens transfer
         * function roughly and looks like a soft throb rather than a
         * click. Feels biological. */
        else if (led_mode == LED_MODE_PULSE_ON_BEAT) {
            uint32_t now_tick = xTaskGetTickCount();
            uint32_t since_beat_ms = (now_tick - beat_pulse_start_tick) * portTICK_PERIOD_MS;
            if (beat_pulse_start_tick != 0 && since_beat_ms < PULSE_DURATION_MS) {
                /* Cosine decay: 1.0 at t=0, 0.0 at t=PULSE_DURATION_MS */
                float p = (float)since_beat_ms / (float)PULSE_DURATION_MS;
                float env = (1.0f + cosf((float)M_PI * p)) / 2.0f;
                uint8_t tint = (uint8_t)(env * (float)PULSE_PEAK_DUTY * (float)brightness / 100.0f);
                effective_duty = tint;
            } else {
                effective_duty = 0;   /* Between beats: lens fully clear */
            }
        }
        
        /* STROBE mode: gptimer ISR handles effective_duty, nothing to do here */
        /* STATIC mode: effective_duty set by command handler, nothing here */
        
        vTaskDelay(AC_PERIOD_TICKS);  /* 10ms tick */
        tick_count++;
        
        /* Log status every 30 seconds */
        if (xTaskGetTickCount() - last_log_tick >= pdMS_TO_TICKS(30000)) {
            last_log_tick = xTaskGetTickCount();
            uint32_t remaining_sec = (session_duration_ms - elapsed) / 1000;
            const char *mstr = (led_mode == LED_MODE_STROBE) ? "STROBE" :
                               (led_mode == LED_MODE_STATIC) ? "STATIC" :
                               (led_mode == LED_MODE_BREATHE_STROBE) ? "BR+STRB" :
                               "BREATHE";
            ESP_LOGI(TAG, "%s duty=%d%% bright=%d%% %lu sec left", 
                     mstr, effective_duty, brightness, remaining_sec);
        }
    }
    
    /* Session ended */
    led_task_handle = NULL;
    vTaskDelete(NULL);
}

/*******************************************************************************
 * DEEP SLEEP
 ******************************************************************************/
static void enter_deep_sleep(void) {
    ESP_LOGI(TAG, "Entering deep sleep...");
    
    /* Clear lens and stop unified timer */
    effective_duty = 0;
    vTaskDelay(pdMS_TO_TICKS(10));  /* Let timer apply zero duty */
    gptimer_stop(drive_timer);
    gptimer_disable(drive_timer);
    pwm_both_off();
    
    /* Configure wake on Hall sensor LOW (arm opened) */
    esp_sleep_enable_ext0_wakeup(HALL_PIN, 0);
    
    esp_deep_sleep_start();
}

/*******************************************************************************
 * HALL SENSOR — GESTURE STATE MACHINE (v4.10.0)
 *
 * Polled at HALL_POLL_MS (50ms) from a dedicated task. Two gestures:
 *   - Short close (HALL_SHORT_MIN_MS .. HALL_SHORT_MAX_MS, decided on release)
 *       → advance to next physical program
 *   - Long close (>= HALL_LONG_MS continuous HIGH, decided while still held)
 *       → enter deep sleep
 * 4000-5000ms release window is intentional dead zone (no action) so the
 * user has a clear gap between "quick tap" and "hold to sleep".
 * 50ms debounce on both edges.
 *
 * The Hall pin reads HIGH when the magnet is near (arm closed), LOW when far.
 * In deep sleep we wake on LOW (arm opened) via ext0 — unchanged.
 ******************************************************************************/

#if !PPG_TEST_BUILD
/* Raw-edge debouncer. Returns the stable level (0 or 1). */
static uint8_t hall_debounced_level(void) {
    static uint8_t stable = 0;
    static uint8_t candidate = 0;
    static uint32_t candidate_since = 0;

    uint8_t raw = gpio_get_level(HALL_PIN) ? 1 : 0;
    uint32_t now = xTaskGetTickCount();

    if (raw != stable) {
        if (raw != candidate) {
            candidate = raw;
            candidate_since = now;
        } else if ((now - candidate_since) * portTICK_PERIOD_MS >= HALL_DEBOUNCE_MS) {
            stable = raw;
        }
    } else {
        candidate = stable;
    }
    return stable;
}

static void hall_task(void *param) {
    uint8_t prev_level = 0;
    uint32_t high_start_tick = 0;
    bool sleep_fired = false;       /* Prevent double-firing the 5s threshold */

    ESP_LOGI(TAG, "Hall gesture task started (poll=%dms short=%d-%dms long=%dms)",
             HALL_POLL_MS, HALL_SHORT_MIN_MS, HALL_SHORT_MAX_MS, HALL_LONG_MS);

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(HALL_POLL_MS));

        /* Block gesture handling during OTA */
        if (in_ota_mode) {
            prev_level = 0;
            sleep_fired = false;
            continue;
        }

        uint8_t level = hall_debounced_level();
        uint32_t now = xTaskGetTickCount();

        /* Rising edge: magnet just closed */
        if (level == 1 && prev_level == 0) {
            high_start_tick = now;
            sleep_fired = false;
            /* v4.11.0: any hall activity re-arms BLE advertising.
             * Fires on the leading edge so advertising is up by the time
             * the user releases — whether they're tapping to advance
             * programs or starting a hold-to-sleep. No-op if already up. */
            ble_adv_rearm();
        }

        /* Still held HIGH: check for long-hold sleep threshold */
        if (level == 1 && !sleep_fired) {
            uint32_t held_ms = (now - high_start_tick) * portTICK_PERIOD_MS;
            if (held_ms >= HALL_LONG_MS) {
                ESP_LOGI(TAG, "Hall long-hold %lums → sleep", held_ms);
                sleep_fired = true;
                session_active = false;
                enter_deep_sleep();
                /* enter_deep_sleep() doesn't return */
            }
        }

        /* Falling edge: magnet just opened — decide the gesture */
        if (level == 0 && prev_level == 1) {
            uint32_t held_ms = (now - high_start_tick) * portTICK_PERIOD_MS;

            if (sleep_fired) {
                /* Already went to sleep — shouldn't reach here, belt-and-braces */
            } else if (held_ms >= HALL_SHORT_MIN_MS && held_ms < HALL_SHORT_MAX_MS) {
                program_t next = (program_t)((current_program + 1) % PROG_COUNT);
                ESP_LOGI(TAG, "Hall short-tap %lums → advance to program %d",
                         held_ms, next);
                apply_program(next);
            } else if (held_ms < HALL_SHORT_MIN_MS) {
                ESP_LOGI(TAG, "Hall tap %lums < %dms, ignored (noise)",
                         held_ms, HALL_SHORT_MIN_MS);
            } else {
                /* Dead zone: 4000-4999ms release. Do nothing. */
                ESP_LOGI(TAG, "Hall tap %lums in dead-zone, ignored", held_ms);
            }
        }

        prev_level = level;
    }
}
#endif /* !PPG_TEST_BUILD */

/*******************************************************************************
 * OTA DEFERRED OPERATIONS
 * 
 * esp_ota_begin() and esp_ota_end() can block for seconds during flash ops.
 * Running them inside the BLE GATT callback would crash the BLE stack.
 * These functions run from a dedicated OTA task instead.
 ******************************************************************************/
static void ota_do_begin(void) {
    if (in_ota_mode) {
        ESP_LOGW(TAG, "Already in OTA mode");
        return;
    }

    ota_partition = esp_ota_get_next_update_partition(NULL);
    if (!ota_partition) {
        ESP_LOGE(TAG, "No OTA partition found");
        send_ota_status(OTA_STATUS_ERROR, OTA_ERR_PARTITION, 0, 0);
        return;
    }

    esp_err_t err = esp_ota_begin(ota_partition, OTA_SIZE_UNKNOWN, &ota_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "OTA begin failed: %s", esp_err_to_name(err));
        send_ota_status(OTA_STATUS_ERROR, OTA_ERR_BEGIN, 0, 0);
        return;
    }

    in_ota_mode = true;
    ota_bytes_written = 0;
    ota_page_offset = 0;
    ota_page_num = 0;
    ota_page_pending = false;
    memset(ota_page_buf, 0xFF, OTA_PAGE_SIZE);
    effective_duty = 0;
    ESP_LOGI(TAG, "OTA started, partition: %s", ota_partition->label);
    send_ota_status(OTA_STATUS_READY, 0, 0, 0);
}

static void ota_do_finish(void) {
    if (!in_ota_mode) {
        send_ota_status(OTA_STATUS_ERROR, OTA_ERR_NOT_IN_OTA, 0, 0);
        return;
    }

    /* Wait for any pending page confirm (esp_ota_write in BTC task) to complete.
     * The OTA API is not thread-safe — watch ota_bytes_written until stable. */
    {
        uint32_t last_written = 0;
        int stable_count = 0;
        while (stable_count < 2) {
            vTaskDelay(pdMS_TO_TICKS(100));
            if (ota_bytes_written == last_written) {
                stable_count++;
            } else {
                last_written = ota_bytes_written;
                stable_count = 0;
            }
        }
    }

    /* Write any remaining data in the page buffer (last partial page) */
    if (ota_page_offset > 0) {
        ESP_LOGI(TAG, "OTA: Writing final partial page (%d bytes)", ota_page_offset);
        esp_err_t err = esp_ota_write(ota_handle, ota_page_buf, ota_page_offset);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "OTA: Final page write failed: %s", esp_err_to_name(err));
            send_ota_status(OTA_STATUS_ERROR, OTA_ERR_WRITE, 0, 0);
            in_ota_mode = false;
            return;
        }
    }

    ESP_LOGI(TAG, "OTA: Finishing, %lu bytes in %d pages + %d remainder",
             (unsigned long)ota_bytes_written, ota_page_num, ota_page_offset);

    esp_err_t err = esp_ota_end(ota_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "OTA end failed: %s", esp_err_to_name(err));
        send_ota_status(OTA_STATUS_ERROR, OTA_ERR_END, 0, 0);
        in_ota_mode = false;
        return;
    }

    err = esp_ota_set_boot_partition(ota_partition);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Set boot partition failed: %s", esp_err_to_name(err));
        send_ota_status(OTA_STATUS_ERROR, OTA_ERR_END, 0, 0);
        in_ota_mode = false;
        return;
    }

    ESP_LOGI(TAG, "OTA complete! %lu bytes written. Rebooting...", 
             (unsigned long)ota_bytes_written);
    send_ota_status(OTA_STATUS_SUCCESS, 0, 0, 0);
    vTaskDelay(pdMS_TO_TICKS(500));
    esp_restart();
}

static void ota_do_cancel(void) {
    if (in_ota_mode) {
        esp_ota_abort(ota_handle);
        in_ota_mode = false;
        ota_bytes_written = 0;
        ota_page_offset = 0;
        ota_page_num = 0;
        ota_page_pending = false;
        ESP_LOGI(TAG, "OTA cancelled");
        send_ota_status(OTA_STATUS_CANCELLED, 0, 0, 0);
    }
}

static void ota_task(void *param) {
    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        ota_task_cmd_t cmd = ota_pending_cmd;
        ota_pending_cmd = OTA_TASK_NONE;

        switch (cmd) {
            case OTA_TASK_BEGIN:  ota_do_begin();  break;
            case OTA_TASK_FINISH: ota_do_finish(); break;
            case OTA_TASK_CANCEL: ota_do_cancel(); break;
            default: break;
        }
    }
}

/*******************************************************************************
 * COMMAND PROCESSING
 ******************************************************************************/
static void process_command(uint8_t *data, uint16_t len) {
    /* Single-byte: legacy duty control → enters static mode */
    if (len == 1) {
        uint8_t byte = data[0];
        uint8_t duty = (byte * 100) / 255;
        ESP_LOGI(TAG, "Legacy duty: %d%% (byte 0x%02X)", duty, byte);
        brightness = duty;
        strobe_stop();
        led_mode = LED_MODE_STATIC;
        effective_duty = brightness;
        return;
    }
    
    /* Multi-byte commands */
    if (len < 2) return;
    
    uint8_t cmd = data[0];
    uint8_t arg = data[1];
    
    switch (cmd) {
        /* ── COMMON ────────────────────────────────────────── */
        case 0xA2:  /* Set brightness / max tint */
            if (arg > 100) arg = 100;
            brightness = arg;
            ESP_LOGI(TAG, "Brightness: %d%%", brightness);
            break;
            
        case 0xA4:  /* Set session duration */
            if (arg < 1) arg = 1;
            if (arg > 60) arg = 60;
            session_duration_ms = arg * 60 * 1000;
            ESP_LOGI(TAG, "Session: %d minutes", arg);
            break;
            
        /* ── MODE SWITCHING ────────────────────────────────── */
        case 0xA5:  /* Enter STATIC mode */
            if (arg > 100) arg = 100;
            brightness = arg;
            strobe_stop();
            led_mode = LED_MODE_STATIC;
            effective_duty = brightness;
            ESP_LOGI(TAG, "Mode: STATIC @ %d%%", arg);
            break;
            
        case 0xA6:  /* Enter STROBE mode */
            led_mode = LED_MODE_STROBE;
            if (session_active) strobe_start();
            ESP_LOGI(TAG, "Mode: STROBE %d.%dHz %d%% duty", 
                     strobe_dhz / 10, strobe_dhz % 10, strobe_duty_pct);
            break;
            
        case 0xA7:  /* Sleep immediately */
            ESP_LOGI(TAG, "Sleep command received");
            session_active = false;
            enter_deep_sleep();
            break;
            
        /* ── STROBE PARAMS ─────────────────────────────────── */
        case 0xAB:  /* Set strobe frequency */
            if (arg < MIN_STROBE_HZ) arg = MIN_STROBE_HZ;
            if (arg > MAX_STROBE_HZ) arg = MAX_STROBE_HZ;
            strobe_dhz = arg * 10;
            strobe_update();
            ESP_LOGI(TAG, "Strobe freq: %dHz", arg);
            break;
            
        case 0xAC:  /* Set strobe duty cycle */
            if (arg < 10) arg = 10;
            if (arg > 90) arg = 90;
            strobe_duty_pct = arg;
            strobe_update();
            ESP_LOGI(TAG, "Strobe duty: %d%%", strobe_duty_pct);
            break;
            
        /* ── OTA ───────────────────────────────────────────── */
        case 0xA8:  /* Start OTA (deferred to OTA task) */
            ESP_LOGI(TAG, "OTA: Start command (deferred)");
            session_active = false;
            strobe_stop();
            effective_duty = 0;
            ota_pending_cmd = OTA_TASK_BEGIN;
            if (ota_task_handle) xTaskNotifyGive(ota_task_handle);
            break;
            
        case 0xA9:  /* Finish OTA (deferred to OTA task) */
            ESP_LOGI(TAG, "OTA: Finish command (deferred)");
            ota_pending_cmd = OTA_TASK_FINISH;
            if (ota_task_handle) xTaskNotifyGive(ota_task_handle);
            break;
            
        case 0xAA:  /* Cancel OTA (deferred to OTA task) */
            ESP_LOGI(TAG, "OTA: Cancel command (deferred)");
            ota_pending_cmd = OTA_TASK_CANCEL;
            if (ota_task_handle) xTaskNotifyGive(ota_task_handle);
            break;
            
        case OTA_CMD_PAGE_CONFIRM:  /* 0xAD: Page confirm/resend */
            if (len >= 2 && in_ota_mode && ota_page_pending) {
                if (data[1] == 0x01) {
                    /* Page confirmed — write buffer to flash */
                    esp_err_t err = esp_ota_write(ota_handle, ota_page_buf, OTA_PAGE_SIZE);
                    if (err != ESP_OK) {
                        ESP_LOGE(TAG, "OTA: Flash write failed page %d: %s",
                                 ota_page_num, esp_err_to_name(err));
                        send_ota_status(OTA_STATUS_ERROR, OTA_ERR_WRITE, 0, 0);
                        esp_ota_abort(ota_handle);
                        in_ota_mode = false;
                        ota_page_pending = false;
                        return;
                    }
                    ESP_LOGI(TAG, "OTA: Page %d committed to flash", ota_page_num);

                    uint8_t ok_status[] = {
                        OTA_STATUS_PAGE_OK,
                        (uint8_t)(ota_page_num >> 8),
                        (uint8_t)(ota_page_num & 0xFF)
                    };
                    send_ota_status_raw(ok_status, sizeof(ok_status));

                    ota_page_num++;
                    ota_page_offset = 0;
                    ota_page_pending = false;
                } else {
                    /* Page rejected — discard, web app will resend */
                    ESP_LOGW(TAG, "OTA: Page %d rejected, awaiting resend", ota_page_num);
                    ota_page_offset = 0;
                    ota_bytes_written -= OTA_PAGE_SIZE;
                    ota_page_pending = false;

                    uint8_t resend_status[] = {
                        OTA_STATUS_PAGE_RESEND,
                        (uint8_t)(ota_page_num >> 8),
                        (uint8_t)(ota_page_num & 0xFF)
                    };
                    send_ota_status_raw(resend_status, sizeof(resend_status));
                }
            }
            break;
            
        /* ── BREATHE MODE + PARAMS ─────────────────────────── */
        case 0xB0:  /* Enter BREATHE mode */
            strobe_stop();
            led_mode = LED_MODE_BREATHE;
            ESP_LOGI(TAG, "Mode: BREATHE %dBPM %d/%d %s", 
                     breathe_bpm, breathe_inhale_pct, 100 - breathe_inhale_pct,
                     breathe_wave == 0 ? "sine" : "linear");
            break;
            
        case 0xB1:  /* Set breathe BPM */
            if (arg < 1) arg = 1;
            if (arg > 30) arg = 30;
            breathe_bpm = arg;
            ESP_LOGI(TAG, "Breathe BPM: %d", breathe_bpm);
            break;
            
        case 0xB2:  /* Set inhale ratio */
            if (arg < 10) arg = 10;
            if (arg > 90) arg = 90;
            breathe_inhale_pct = arg;
            ESP_LOGI(TAG, "Breathe inhale: %d%% exhale: %d%%", arg, 100 - arg);
            break;
            
        case 0xB3:  /* Set hold-at-top (100ms units) */
            if (arg > 50) arg = 50;
            breathe_hold_top = arg;
            ESP_LOGI(TAG, "Breathe hold top: %dms", arg * 100);
            break;
            
        case 0xB4:  /* Set hold-at-bottom (100ms units) */
            if (arg > 50) arg = 50;
            breathe_hold_bot = arg;
            ESP_LOGI(TAG, "Breathe hold bot: %dms", arg * 100);
            break;
            
        case 0xB5:  /* Set breathe waveform */
            breathe_wave = (arg > 0) ? 1 : 0;
            ESP_LOGI(TAG, "Breathe wave: %s", breathe_wave == 0 ? "sine" : "linear");
            break;

        case 0xB6:  /* v4.12.1: Enter PULSE_ON_BEAT mode */
            strobe_stop();
            led_mode = LED_MODE_PULSE_ON_BEAT;
            effective_duty = 0;
            beat_pulse_start_tick = 0;  /* Clear any stale pulse from previous mode */
            ESP_LOGI(TAG, "Mode: PULSE_ON_BEAT (flash on each detected heartbeat)");
            break;

        case 0xC0:  /* v4.12.2: ADC scan mode — arg=0 enable, arg=1 disable.
                     * Scans ADC1 channels every 500ms and logs values via
                     * ble_log. Lets us find which GPIO the PulseSensor is
                     * actually wired to without reflashing.
                     * NOTE: while enabled, the normal PPG ADC read will be
                     * briefly disturbed (we reconfigure attenuation on each
                     * channel). Expect detection to drop out during scan.
                     * Disable when done. */
            if (arg == 0) {
                adc_scan_enabled = true;
                ESP_LOGI(TAG, "ADC scan mode ENABLED");
                ble_log("ADC scan enabled");
            } else {
                adc_scan_enabled = false;
                /* Restore the normal PPG channel's attenuation */
                adc1_config_channel_atten(PPG_ADC_CHANNEL, ADC_ATTEN_DB_12);
                ESP_LOGI(TAG, "ADC scan mode disabled");
                ble_log("ADC scan disabled");
            }
            break;

        case 0xD0:  /* v4.12.6: Manual detector reset.
                     * Clears all detection state (MA buffers, block state,
                     * run counter, last-beat time). Preserves
                     * ppg_beat_count_total (session cumulative counter).
                     * Useful when dashboard notices firmware detection
                     * falling behind real beats after a noisy patch.
                     * arg is ignored. */
            ppg_reset_detector();
            ESP_LOGI(TAG, "Detector reset via BLE");
            ble_log("Detector reset");
            break;

        default:
            ESP_LOGW(TAG, "Unknown command: 0x%02X", cmd);
            break;
    }
}

/*******************************************************************************
 * OTA DATA PROCESSING
 ******************************************************************************/
/*******************************************************************************
 * OTA DATA PROCESSING — Page-Based with CRC
 *
 * Buffers chunks in ota_page_buf (4KB). When full, computes CRC32 and notifies
 * the web app. Web app compares CRCs: [0xAD, 0x01] to commit, [0xAD, 0x00]
 * to resend. Enables fast WriteWithoutResponse with per-page integrity.
 ******************************************************************************/
static void process_ota_data(uint8_t *data, uint16_t len) {
    if (!in_ota_mode) {
        ESP_LOGW(TAG, "OTA data received but not in OTA mode");
        send_ota_status(OTA_STATUS_ERROR, OTA_ERR_NOT_IN_OTA, 0, 0);
        return;
    }

    if (ota_page_pending) {
        ESP_LOGW(TAG, "OTA: Data received while page pending confirmation");
        return;
    }

    uint16_t data_offset = 0;
    while (data_offset < len) {
        uint16_t space = OTA_PAGE_SIZE - ota_page_offset;
        uint16_t copy_len = (len - data_offset < space) ? (len - data_offset) : space;

        memcpy(ota_page_buf + ota_page_offset, data + data_offset, copy_len);
        ota_page_offset += copy_len;
        data_offset += copy_len;
        ota_bytes_written += copy_len;

        /* Page full — compute CRC and notify web app */
        if (ota_page_offset >= OTA_PAGE_SIZE) {
            uint32_t crc = esp_crc32_le(0, ota_page_buf, OTA_PAGE_SIZE);

            ESP_LOGI(TAG, "OTA: Page %d full (%lu total), CRC=0x%08lX",
                     ota_page_num, (unsigned long)ota_bytes_written, (unsigned long)crc);

            uint8_t status[] = {
                OTA_STATUS_PAGE_CRC,
                (uint8_t)(ota_page_num >> 8),
                (uint8_t)(ota_page_num & 0xFF),
                (uint8_t)((crc >> 24) & 0xFF),
                (uint8_t)((crc >> 16) & 0xFF),
                (uint8_t)((crc >> 8) & 0xFF),
                (uint8_t)(crc & 0xFF)
            };
            send_ota_status_raw(status, sizeof(status));
            ota_page_pending = true;

            if (data_offset < len) {
                ESP_LOGW(TAG, "OTA: %d bytes crossed page boundary (discarded)",
                         len - data_offset);
            }
            break;
        }
    }
}

/*******************************************************************************
 * BLE ADVERTISING PARAMETERS (Power Optimized)
 ******************************************************************************/
static esp_ble_adv_params_t adv_params = {
    .adv_int_min        = ADV_INT_MIN,   /* 100ms (v4.9.12) */
    .adv_int_max        = ADV_INT_MAX,   /* 200ms (v4.9.12) */
    .adv_type           = ADV_TYPE_IND,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy  = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp        = false,
    .include_name        = true,
    .include_txpower     = false,
    .flag                = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

/*******************************************************************************
 * BLE STACK LIFECYCLE HELPERS (v4.11.0 / v4.11.1)
 *
 * Rationale: v4.9.12 changed advertising to -6dBm at 100-200ms for faster
 * first-try connects, which added ~4mA to idle current. That cost is only
 * worth paying in the window when the user actually wants to connect.
 *
 * v4.11.0 stopped advertising after BLE_IDLE_TIMEOUT_MS but left the
 * controller and Bluedroid running — current traces showed residual
 * ~1.5A RF spikes from BT controller housekeeping. v4.11.1 escalates to
 * full BT stack teardown so the radio is completely cold.
 *
 * ble_stack_init():      Bring up BT controller + Bluedroid + GATT app.
 *                        Idempotent. Returns ESP_OK if already up.
 * ble_stack_teardown():  Tear down everything initialized by init().
 *                        Idempotent. Returns ESP_OK if already down.
 * ble_adv_rearm():       Ensure stack is up and advertising; reset deadline.
 *                        Called on boot, hall events, disconnect.
 * ble_adv_reset_deadline(): Push deadline out another window.
 *
 * The actual teardown trigger happens in app_main's 1Hz loop when the
 * deadline elapses, not in these helpers.
 *
 * State invariants:
 *   ble_stack_up == false  ⟹  ble_adv_active == false AND is_connected == false
 *   ble_adv_active == true ⟹  ble_stack_up == true
 *
 * Handle lifetimes: gatts_if_global, service_handle, ctrl_char_handle,
 * ota_char_handle, status_char_handle, cccd_handle all become invalid
 * across teardown and are repopulated by the GATT event handler when
 * the new GATT app registers on the next init.
 ******************************************************************************/
static void ble_adv_reset_deadline(void) {
    ble_idle_deadline_tick = xTaskGetTickCount() + pdMS_TO_TICKS(BLE_IDLE_TIMEOUT_MS);
}

static esp_err_t ble_stack_init(void) {
    if (ble_stack_up) return ESP_OK;

    ESP_LOGI(TAG, "BLE stack init...");

    esp_err_t err;
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    err = esp_bt_controller_init(&bt_cfg);
    if (err != ESP_OK) { ESP_LOGE(TAG, "bt_controller_init: %s", esp_err_to_name(err)); return err; }

    err = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (err != ESP_OK) { ESP_LOGE(TAG, "bt_controller_enable: %s", esp_err_to_name(err)); return err; }

    err = esp_bluedroid_init();
    if (err != ESP_OK) { ESP_LOGE(TAG, "bluedroid_init: %s", esp_err_to_name(err)); return err; }

    err = esp_bluedroid_enable();
    if (err != ESP_OK) { ESP_LOGE(TAG, "bluedroid_enable: %s", esp_err_to_name(err)); return err; }

    /* Re-register callbacks and GATT app. The GATT app registration will
     * trigger ADD_CHAR events etc. in the GATT handler, which repopulates
     * the service/char handles and ultimately starts advertising via the
     * ADV_DATA_SET_COMPLETE_EVT path. */
    err = esp_ble_gap_register_callback(gap_event_handler);
    if (err != ESP_OK) { ESP_LOGE(TAG, "gap_register_callback: %s", esp_err_to_name(err)); return err; }

    err = esp_ble_gatts_register_callback(gatts_event_handler);
    if (err != ESP_OK) { ESP_LOGE(TAG, "gatts_register_callback: %s", esp_err_to_name(err)); return err; }

    err = esp_ble_gatts_app_register(GATTS_APP_ID);
    if (err != ESP_OK) { ESP_LOGE(TAG, "gatts_app_register: %s", esp_err_to_name(err)); return err; }

    err = esp_ble_gatt_set_local_mtu(517);
    if (err != ESP_OK) { ESP_LOGW(TAG, "set_local_mtu: %s", esp_err_to_name(err)); /* non-fatal */ }

    ble_stack_up = true;
    ESP_LOGI(TAG, "BLE stack up");
    return ESP_OK;
}

static esp_err_t ble_stack_teardown(void) {
    if (!ble_stack_up) return ESP_OK;

    ESP_LOGI(TAG, "BLE stack teardown (radio off)");

    /* Order matters: disable/deinit Bluedroid first (it uses the controller),
     * then disable/deinit the controller. Ignore errors — we're going down
     * regardless, and partial teardown is worse than ugly logs. */
    if (ble_adv_active) {
        esp_ble_gap_stop_advertising();
        /* ADV_STOP_COMPLETE_EVT may not fire before we kill the stack —
         * clear the flag manually so state stays consistent. */
        ble_adv_active = false;
    }

    esp_err_t err;
    err = esp_bluedroid_disable();
    if (err != ESP_OK) ESP_LOGW(TAG, "bluedroid_disable: %s", esp_err_to_name(err));
    err = esp_bluedroid_deinit();
    if (err != ESP_OK) ESP_LOGW(TAG, "bluedroid_deinit: %s", esp_err_to_name(err));
    err = esp_bt_controller_disable();
    if (err != ESP_OK) ESP_LOGW(TAG, "bt_controller_disable: %s", esp_err_to_name(err));
    err = esp_bt_controller_deinit();
    if (err != ESP_OK) ESP_LOGW(TAG, "bt_controller_deinit: %s", esp_err_to_name(err));

    /* Invalidate BLE-dependent handles. is_connected must already be false
     * here since timeout gates on !is_connected. */
    ble_stack_up = false;
    gatts_if_global = ESP_GATT_IF_NONE;
    ctrl_char_handle = 0;
    ota_char_handle = 0;
    status_char_handle = 0;
    cccd_handle = 0;
    service_handle = 0;
    notifications_enabled = false;

    ble_idle_deadline_tick = 0;
    return ESP_OK;
}

static void ble_adv_rearm(void) {
    /* If connected, no point advertising; leave deadline disabled. */
    if (is_connected) {
        ble_idle_deadline_tick = 0;
        return;
    }

    /* Bring stack up if it was torn down by the auto-off timeout.
     * Advertising itself starts asynchronously via the GATT service-create
     * flow, so once the stack is up the first time we just wait. */
    if (!ble_stack_up) {
        esp_err_t err = ble_stack_init();
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "BLE re-arm failed to init stack: %s", esp_err_to_name(err));
            return;
        }
        /* Advertising will start asynchronously via ADV_DATA_SET_COMPLETE_EVT.
         * Arm the deadline now; if adv never comes up the flag stays false
         * and the timeout check is gated on ble_adv_active anyway. */
        ble_adv_reset_deadline();
        ESP_LOGI(TAG, "BLE re-armed via stack init (60s window)");
        return;
    }

    /* Stack was already up (e.g. boot path, or rapid re-arm). Just make
     * sure advertising is running and reset the deadline. */
    if (!ble_adv_active) {
        esp_err_t err = esp_ble_gap_start_advertising(&adv_params);
        if (err == ESP_OK) {
            ble_adv_active = true;
            ESP_LOGI(TAG, "BLE advertising re-armed (60s window)");
        } else {
            ESP_LOGW(TAG, "BLE adv start failed: %s", esp_err_to_name(err));
            return;
        }
    }
    ble_adv_reset_deadline();
}

/*******************************************************************************
 * GAP EVENT HANDLER
 ******************************************************************************/
static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    switch (event) {
        case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
            esp_ble_gap_start_advertising(&adv_params);
            break;
            
        case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
            if (param->adv_start_cmpl.status == ESP_BT_STATUS_SUCCESS) {
                ble_adv_active = true;  /* v4.11.0: track state */
                ESP_LOGI(TAG, "Advertising started");
            } else {
                ble_adv_active = false;
                ESP_LOGE(TAG, "Advertising failed");
            }
            break;

        case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:  /* v4.11.0: track state */
            ble_adv_active = false;
            ESP_LOGI(TAG, "Advertising stopped");
            break;

        default:
            break;
    }
}

/*******************************************************************************
 * GATT EVENT HANDLER
 ******************************************************************************/
static void gatts_event_handler(esp_gatts_cb_event_t event, 
                                 esp_gatt_if_t gatts_if,
                                 esp_ble_gatts_cb_param_t *param) {
    
    switch (event) {
        case ESP_GATTS_REG_EVT:
            ESP_LOGI(TAG, "GATT app registered");
            gatts_if_global = gatts_if;
            
            /* Set device name */
            esp_ble_gap_set_device_name(DEVICE_NAME);
            
            /* v4.9.12: Dual TX power scheme for reliable connects + low power.
             *   - ADV / DEFAULT at -6dBm: advertising packets arrive at the
             *     phone with ~4× the signal strength of v4.9.11, giving the
             *     handshake plenty of link margin. Biggest single win for
             *     first-try connect reliability.
             *   - CONN_HDL0 at -12dBm: once connected, the link is already
             *     established and a weaker signal is sufficient. Keeps
             *     per-session power draw close to v4.9.11 baseline.
             * Previously all three were -12dBm (overly conservative for adv). */
            esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT,   ESP_PWR_LVL_N6);
            esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV,       ESP_PWR_LVL_N6);
            esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_CONN_HDL0, ESP_PWR_LVL_N12);
            
            /* Configure advertising */
            esp_ble_gap_config_adv_data(&adv_data);
            
            /* Create service */
            esp_gatt_srvc_id_t service_id = {
                .is_primary = true,
                .id = {
                    .inst_id = 0,
                    .uuid = {
                        .len = ESP_UUID_LEN_16,
                        .uuid = {.uuid16 = GATTS_SERVICE_UUID}
                    }
                }
            };
            esp_ble_gatts_create_service(gatts_if, &service_id, GATTS_NUM_HANDLE);
            break;
            
        case ESP_GATTS_CREATE_EVT:
            service_handle = param->create.service_handle;
            ESP_LOGI(TAG, "Service created, handle %d", service_handle);
            esp_ble_gatts_start_service(service_handle);
            
            /* Add Control characteristic (0xFF01) */
            esp_bt_uuid_t ctrl_uuid = {
                .len = ESP_UUID_LEN_16,
                .uuid = {.uuid16 = GATTS_CHAR_UUID_CTRL}
            };
            esp_ble_gatts_add_char(service_handle, &ctrl_uuid,
                                   ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                                   ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE,
                                   NULL, NULL);
            break;
            
        case ESP_GATTS_ADD_CHAR_EVT: {
            uint16_t char_uuid = param->add_char.char_uuid.uuid.uuid16;
            
            if (char_uuid == GATTS_CHAR_UUID_CTRL) {
                ctrl_char_handle = param->add_char.attr_handle;
                ESP_LOGI(TAG, "Char added step 0, handle %d", ctrl_char_handle);
                
                /* Add OTA Data characteristic (0xFF02) */
                esp_bt_uuid_t ota_uuid = {
                    .len = ESP_UUID_LEN_16,
                    .uuid = {.uuid16 = GATTS_CHAR_UUID_OTA}
                };
                esp_ble_gatts_add_char(service_handle, &ota_uuid,
                                       ESP_GATT_PERM_WRITE,
                                       ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_WRITE_NR,
                                       NULL, NULL);
            }
            else if (char_uuid == GATTS_CHAR_UUID_OTA) {
                ota_char_handle = param->add_char.attr_handle;
                ESP_LOGI(TAG, "Char added step 1, handle %d", ota_char_handle);
                
                /* Add Status characteristic (0xFF03) with notify */
                esp_bt_uuid_t status_uuid = {
                    .len = ESP_UUID_LEN_16,
                    .uuid = {.uuid16 = GATTS_CHAR_UUID_STATUS}
                };
                esp_ble_gatts_add_char(service_handle, &status_uuid,
                                       ESP_GATT_PERM_READ,
                                       ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY,
                                       NULL, NULL);
            }
            else if (char_uuid == GATTS_CHAR_UUID_STATUS) {
                status_char_handle = param->add_char.attr_handle;
                ESP_LOGI(TAG, "Char added step 2, handle %d", status_char_handle);
                
                /* Add CCCD for status notifications (first of two CCCDs).
                 * gatts_init_step lets ADD_CHAR_DESCR_EVT know this one
                 * belongs to the status char. */
                gatts_init_step = 0;
                esp_bt_uuid_t cccd_uuid = {
                    .len = ESP_UUID_LEN_16,
                    .uuid = {.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG}
                };
                esp_ble_gatts_add_char_descr(service_handle, &cccd_uuid,
                                              ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                                              NULL, NULL);
            }
            else if (char_uuid == GATTS_CHAR_UUID_PPG) {
                /* v4.12.0: PPG characteristic added. Now add its CCCD
                 * so clients can subscribe. */
                ppg_char_handle = param->add_char.attr_handle;
                ESP_LOGI(TAG, "Char added step 3 (PPG), handle %d", ppg_char_handle);

                gatts_init_step = 1;
                esp_bt_uuid_t cccd_uuid = {
                    .len = ESP_UUID_LEN_16,
                    .uuid = {.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG}
                };
                esp_ble_gatts_add_char_descr(service_handle, &cccd_uuid,
                                              ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                                              NULL, NULL);
            }
            break;
        }
        
        case ESP_GATTS_ADD_CHAR_DESCR_EVT:
            /* v4.12.0: two CCCDs are added in sequence (one for 0xFF03,
             * one for 0xFF04). gatts_init_step tracks which one we just
             * got back. On the first one, also kick off adding the PPG
             * characteristic — the chain has to go CHAR → CCCD → CHAR
             * → CCCD to keep the ESP BLE stack happy. */
            if (gatts_init_step == 0) {
                cccd_handle = param->add_char_descr.attr_handle;
                ESP_LOGI(TAG, "Status CCCD handle %d", cccd_handle);

                /* Now add the PPG characteristic (0xFF04). */
                esp_bt_uuid_t ppg_uuid = {
                    .len = ESP_UUID_LEN_16,
                    .uuid = {.uuid16 = GATTS_CHAR_UUID_PPG}
                };
                esp_ble_gatts_add_char(service_handle, &ppg_uuid,
                                       ESP_GATT_PERM_READ,
                                       ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY,
                                       NULL, NULL);
            } else {
                ppg_cccd_handle = param->add_char_descr.attr_handle;
                ESP_LOGI(TAG, "PPG CCCD handle %d - BLE setup complete", ppg_cccd_handle);
                ESP_LOGI(TAG, "Handles: ctrl=%d ota=%d status=%d ppg=%d",
                         ctrl_char_handle, ota_char_handle, status_char_handle, ppg_char_handle);
                ESP_LOGI(TAG, "OTA: A8=start A9=finish AA=cancel");
            }
            break;
            
        case ESP_GATTS_CONNECT_EVT:
            is_connected = true;
            conn_id_global = param->connect.conn_id;
            ble_idle_deadline_tick = 0;  /* v4.11.0: disable auto-off while connected */
            ESP_LOGI(TAG, "Client connected, conn_id %d", conn_id_global);
            
            /* Update connection parameters for power saving.
             * v4.10.1: supervision timeout 4s -> 20s. OTA partition erase
             * (esp_ota_begin) holds the radio silent for 6-19s, which was
             * exceeding the 4s supervision timeout and causing the central
             * to drop the link mid-erase before OTA_STATUS_READY could be
             * sent. 20s (0x7D0) is well under the 32s BLE spec max.
             * Note: these params are a *request* to the central — phones
             * may honor, defer, or ignore them.
             *
             * v4.12.5: interval tightened 40-60ms → 7.5-15ms.
             * v4.12.8: loosened to 20-30ms with latency=1 — the 7.5-15ms
             * setting caused average current draw to spike from ~40mA
             * to ~100mA because the radio wakes 5-10× more frequently.
             * With dashboard v13.8+ smoothing buffer now absorbing
             * bursty delivery, we don't need the extreme-low-latency
             * interval. 20-30ms still gives 3-6× better throughput than
             * the v4.12.4 settings, and effective wake period (interval
             * × (latency+1) ≈ 40ms) gives back most of the power.
             *
             * BLE conn-event wake duty cycle scaling (ballpark):
             *   v4.12.4   40-60ms, lat=4  → wake ~every 200ms  → ~40mA
             *   v4.12.5    7.5-15ms, lat=0 → wake ~every 10ms   → ~100mA
             *   v4.12.8   20-30ms, lat=1  → wake ~every 40ms   → ~50-60mA */
            esp_ble_conn_update_params_t conn_params = {
                .latency = 1,           /* Allow skipping 1 event (power)  */
                .max_int = 0x18,        /* 30ms max interval */
                .min_int = 0x10,        /* 20ms min interval */
                .timeout = 2000,        /* 20 second supervision timeout */
            };
            memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
            esp_ble_gap_update_conn_params(&conn_params);
            break;
            
        case ESP_GATTS_DISCONNECT_EVT:
            is_connected = false;
            notifications_enabled = false;
            ppg_notifications_enabled = false;  /* v4.12.0 */
            ESP_LOGI(TAG, "Client disconnected");
            
            /* Cancel OTA if in progress */
            if (in_ota_mode) {
                esp_ota_abort(ota_handle);
                in_ota_mode = false;
                ESP_LOGW(TAG, "OTA cancelled due to disconnect");
            }
            
            /* Restart advertising with a fresh 60s auto-off window (v4.11.0).
             * If no reconnect arrives in that window, advertising stops and
             * device continues standalone until a hall gesture or sleep. */
            esp_ble_gap_start_advertising(&adv_params);
            ble_adv_reset_deadline();
            break;
            
        case ESP_GATTS_WRITE_EVT:
            if (param->write.handle == ctrl_char_handle) {
                process_command(param->write.value, param->write.len);
            }
            else if (param->write.handle == ota_char_handle) {
                process_ota_data(param->write.value, param->write.len);
            }
            else if (param->write.handle == cccd_handle) {
                if (param->write.len == 2) {
                    uint16_t cccd_value = param->write.value[0] | (param->write.value[1] << 8);
                    notifications_enabled = (cccd_value == 0x0001);
                    ESP_LOGI(TAG, "Status notifications %s", notifications_enabled ? "enabled" : "disabled");
                    /* v4.12.1: announce firmware on subscribe so the
                     * dashboard's firmware log immediately shows what
                     * build is running and which mode we're in. */
                    if (notifications_enabled) {
                        ble_log("Narbis fw v%s test=%d mode=%d",
                                FIRMWARE_VERSION, PPG_TEST_BUILD, (int)led_mode);
                    }
                }
            }
            else if (param->write.handle == ppg_cccd_handle) {
                /* v4.12.0: client subscribing to PPG stream on 0xFF04 */
                if (param->write.len == 2) {
                    uint16_t cccd_value = param->write.value[0] | (param->write.value[1] << 8);
                    ppg_notifications_enabled = (cccd_value == 0x0001);
                    ESP_LOGI(TAG, "PPG notifications %s", ppg_notifications_enabled ? "enabled" : "disabled");
                }
            }
            
            /* Send response if needed */
            if (param->write.need_rsp) {
                esp_ble_gatts_send_response(gatts_if, param->write.conn_id,
                                            param->write.trans_id, ESP_GATT_OK, NULL);
            }
            break;
            
        default:
            break;
    }
}

/*******************************************************************************
 * PPG PIPELINE (v4.12.0)
 *
 * 50 Hz PulseSensor ADC sampling → bandpass → Elgendi peak detection →
 * BLE stream on 0xFF04. Mirrors the v13.1 dashboard pipeline so client
 * and firmware produce identical beats on identical signals — useful
 * for cross-validation during integration.
 *
 * What's NOT here (owned by the dashboard instead):
 *   - Template matching (NCC against a learned beat morphology)
 *   - Kalman gating of IBIs
 *   - Self-tuning α
 * Rationale: these are refinements that catch edge cases (motion
 * artifacts, ectopic beats, drift). They cost ~400ms of lookahead
 * latency (v13.1's HALF=20 samples). For the firmware's eventual job
 * of driving the lens beat-synchronously, latency matters more than
 * PPV — a rare false beat is harmless, a 400ms-late pulse is not.
 * Dashboard still gets the raw signal and can re-run the full pipeline
 * on it for HRV analysis.
 ******************************************************************************/

/* Butterworth 2nd-order bandpass, 0.5–8 Hz @ 50 Hz.
 * Same coefficients as v13.1 dashboard — computed via scipy:
 *   b, a = signal.butter(2, [0.5, 8], btype='band', fs=50)
 * Cascaded direct-form II biquad, 5 taps.
 */
/* Butterworth 2nd-order bandpass — v4.12.4: narrowed to 0.5-4 Hz from
 * the v4.12.0 value of 0.5-8 Hz. At 50 Hz sample rate, 60 Hz mains
 * aliases to 10 Hz which was weakly in the old passband. 4 Hz upper
 * corner with 2nd-order rolloff attenuates 10 Hz by ~24 dB (vs ~6 dB
 * at 8 Hz). Still covers 30-240 BPM physiological range.
 * Edge detail of the PPG trace is slightly smoother (dicrotic notch
 * less visible) but peak DETECTION is strictly better since the
 * detector squares the filtered signal — surviving HF becomes energy
 * the detector can mistake for beat content.
 */
static const float PPG_BP_B[5] = { 0.03657484f, 0.0f, -0.07314967f, 0.0f, 0.03657484f };
static const float PPG_BP_A[5] = { 1.0f, -3.33661174f, 4.22598625f, -2.42581874f, 0.53719462f };

static float ppg_bp_x[5] = {0};   /* input history */
static float ppg_bp_y[5] = {0};   /* output history */

static inline float ppg_bandpass(float sample) {
    /* Shift history */
    for (int i = 4; i > 0; i--) {
        ppg_bp_x[i] = ppg_bp_x[i-1];
        ppg_bp_y[i] = ppg_bp_y[i-1];
    }
    ppg_bp_x[0] = sample;
    ppg_bp_y[0] = PPG_BP_B[0]*ppg_bp_x[0] + PPG_BP_B[1]*ppg_bp_x[1]
                + PPG_BP_B[2]*ppg_bp_x[2] + PPG_BP_B[3]*ppg_bp_x[3]
                + PPG_BP_B[4]*ppg_bp_x[4]
                - PPG_BP_A[1]*ppg_bp_y[1] - PPG_BP_A[2]*ppg_bp_y[2]
                - PPG_BP_A[3]*ppg_bp_y[3] - PPG_BP_A[4]*ppg_bp_y[4];
    return ppg_bp_y[0];
}

/* Elgendi 2013 detector state.
 * W1 = 6 samples ≈ 111ms at 50Hz (MA_peak window — emphasizes systolic upstroke)
 * W2 = 33 samples ≈ 667ms at 50Hz (MA_beat window — tracks slow envelope)
 * α  = 0.02 (offset threshold: MA_peak > MA_beat × (1+α) → in a beat block)
 */
#define PPG_W1                  6
#define PPG_W2                  33
#define PPG_ALPHA_Q10           51     /* v4.12.5: α × 1024, 51/1024 ≈ 0.0498
                                        * was 21 (0.02). Raised for noise
                                        * robustness — see changelog. */
#define PPG_BLOCK_ENTRY_MIN     2      /* v4.12.5: require N consecutive above-
                                        * threshold samples to enter block.
                                        * Kills single-sample false entries. */
#define PPG_REFRACTORY_MIN_MS   300
#define PPG_WARMUP_SAMPLES      50     /* 1 second — let filter + MA_beat settle */
#define PPG_MIN_MA_BEAT         4.0f   /* v4.12.1: noise floor. Below this the signal
                                        * is effectively DC/noise — suppress detection
                                        * so a disconnected sensor doesn't emit false beats */

/* v4.13.1: raw-ADC sensor-presence gate. A physically-connected
 * PulseSensor on skin sits near midrail with bounded p-p envelope.
 * A disconnected/floating/no-skin sensor either rails low (raw=0 —
 * observed on this PCB), rails high (near 4095), or floats in a
 * plausible DC range but with very low or very chaotic p-p.
 *
 * DC_MIN/MAX bracket the midrail range where a functioning sensor
 * can sit; outside this range indicates disconnect.
 *
 * AC_MIN (v4.13.6, was 10): caught ghost beats that made it past the
 * initial gate. The PulseSensor amplifier self-oscillates with no
 * finger, producing ~150-200 code p-p swing that looks enough like
 * signal to clear a too-loose threshold. Real finger pulses produce
 * 300-1000+ code swings. 200 discriminates amplifier noise from
 * real pulses while leaving margin for weak-contact cases.
 *
 * Not currently gating upper AC — motion rejection is a separate
 * problem and reasonable motion produces <1500 p-p anyway. */
#define PPG_RAW_DC_MIN          300
#define PPG_RAW_DC_MAX          3900
#define PPG_RAW_AC_MIN          200

/* Running-sum moving averages, O(1) per sample */
static float ppg_peak_buf[PPG_W1] = {0};
static uint8_t ppg_peak_idx = 0, ppg_peak_count = 0;
static float ppg_peak_sum = 0.0f;

static float ppg_beat_buf[PPG_W2] = {0};
static uint8_t ppg_beat_idx = 0, ppg_beat_count = 0;
static float ppg_beat_sum = 0.0f;

/* Block state machine */
static uint32_t ppg_sample_count = 0;
static bool ppg_in_block = false;
/* v4.12.5: tracks consecutive above-threshold samples before entering
 * in-block state. Must reach PPG_BLOCK_ENTRY_MIN to transition. Reset
 * to 0 on any below-threshold sample (consecutive requirement). */
static uint8_t ppg_above_run = 0;
static uint32_t ppg_block_start_ms = 0;
static float ppg_block_peak_val = -1e9f;
static uint32_t ppg_block_peak_ms = 0;

/* Beat bookkeeping */
static uint32_t ppg_last_beat_ms = 0;
static uint32_t ppg_current_ibi_ms = 0;   /* 0 until we have 2+ beats */
static uint8_t  ppg_current_bpm = 0;
static uint32_t ppg_beat_count_total = 0;

/* v4.12.2 telemetry */
static uint32_t ble_send_errors = 0;        /* incr'd when send_indicate returns non-OK */
static uint32_t ppg_jitter_max_us = 0;      /* max tick jitter in 5s window (µs) */
/* adc_scan_enabled is forward-declared earlier in the file near GLOBAL STATE
 * (needed by process_command). Zero-initialized by the tentative definition. */

/* v4.12.6: clear all detector state. Used by the auto-stuck-recovery
 * watchdog AND by the 0xD0 manual reset BLE command.
 * Preserves ppg_beat_count_total (cumulative counter for the session)
 * and ppg_sample_count (keeps warmup logic honest — we don't want a
 * reset to cause a fresh warmup window right after).
 * Clears: both MA ring buffers, block state, run counter, last-beat
 * time, current IBI/BPM readouts. */
static void ppg_reset_detector(void) {
    for (int i = 0; i < PPG_W1; i++) ppg_peak_buf[i] = 0.0f;
    ppg_peak_sum = 0.0f;
    ppg_peak_idx = 0;
    ppg_peak_count = 0;
    for (int i = 0; i < PPG_W2; i++) ppg_beat_buf[i] = 0.0f;
    ppg_beat_sum = 0.0f;
    ppg_beat_idx = 0;
    ppg_beat_count = 0;
    ppg_in_block = false;
    ppg_above_run = 0;
    ppg_block_peak_val = -1e9f;
    ppg_last_beat_ms = 0;
    ppg_current_ibi_ms = 0;
    ppg_current_bpm = 0;
    /* v4.13.2 FIX: also reset sample_count so warmup period re-engages.
     * Without this, the 1 second warmup check (ppg_sample_count < 50)
     * passes immediately after reset, and the under-filled MA_beat buffer
     * vs the filled-faster MA_peak buffer causes a burst of false
     * positives for ~660ms. Re-entering warmup makes reset behave
     * identically to boot — detector resumes only after MAs stabilize. */
    ppg_sample_count = 0;
}

/* v4.13.1: sensor-presence gate.
 * Scans the existing adc_stats_ring to check whether the last ~500ms of
 * raw ADC readings look like a physically-connected sensor. Used by
 * ppg_detect to hard-gate detection when the sensor is unplugged or
 * off-skin.
 *
 * Criteria (all must pass):
 *   1. Mean raw ADC ∈ [PPG_RAW_DC_MIN, PPG_RAW_DC_MAX] — sensor is
 *      biased near midrail, not railed low or high.
 *   2. p-p swing ≥ PPG_RAW_AC_MIN — some envelope present (a truly
 *      flat DC is either floating-with-no-coupling or no-skin).
 *
 * The adc_stats_ring is 25 samples (500ms) and is updated in-place by
 * ppg_task before detection runs, so we always have fresh stats.
 *
 * Returns true if sensor appears present. Updates g_sensor_gate_ok so
 * other code (FW Log, future lens pacer) can query gate state without
 * recomputing. */
static volatile bool g_sensor_gate_ok = true;

static bool ppg_sensor_looks_present(void) {
    if (adc_stats_count < 5) return true;  /* too early — don't gate yet */

    uint16_t mn = 4095, mx = 0;
    uint32_t sum = 0;
    for (int i = 0; i < adc_stats_count; i++) {
        uint16_t v = adc_stats_ring[i];
        if (v < mn) mn = v;
        if (v > mx) mx = v;
        sum += v;
    }
    uint16_t mean = (uint16_t)(sum / adc_stats_count);
    uint16_t pp = mx - mn;

    bool ok = (mean >= PPG_RAW_DC_MIN) && (mean <= PPG_RAW_DC_MAX)
           && (pp >= PPG_RAW_AC_MIN);
    return ok;
}

/* Returns true if this sample resulted in a confirmed beat.
 * Unlike v13.1 (which waits 400ms for template matching), this emits
 * the beat at block-end — latency from true peak to return is ~100–150ms.
 */
static bool ppg_detect(float filtered, uint32_t time_ms) {
    ppg_sample_count++;

    /* Rectify (clip negatives to 0) and square */
    float rect = filtered > 0.0f ? filtered : 0.0f;
    float sq = rect * rect;

    /* MA_peak — short window */
    if (ppg_peak_count == PPG_W1) ppg_peak_sum -= ppg_peak_buf[ppg_peak_idx];
    ppg_peak_buf[ppg_peak_idx] = sq;
    ppg_peak_sum += sq;
    ppg_peak_idx = (ppg_peak_idx + 1) % PPG_W1;
    if (ppg_peak_count < PPG_W1) ppg_peak_count++;
    float ma_peak = ppg_peak_sum / ppg_peak_count;

    /* MA_beat — long window */
    if (ppg_beat_count == PPG_W2) ppg_beat_sum -= ppg_beat_buf[ppg_beat_idx];
    ppg_beat_buf[ppg_beat_idx] = sq;
    ppg_beat_sum += sq;
    ppg_beat_idx = (ppg_beat_idx + 1) % PPG_W2;
    if (ppg_beat_count < PPG_W2) ppg_beat_count++;
    float ma_beat = ppg_beat_sum / ppg_beat_count;

    /* Warmup — suppress detection until MAs have settled */
    if (ppg_sample_count < PPG_WARMUP_SAMPLES) return false;

    /* v4.13.1: raw-ADC sensor-presence gate. Hard-reject detection if
     * the raw ADC doesn't look like a physically-connected sensor.
     * Cheaper and more reliable than the bandpass-based check below
     * because a disconnected pin's bandpass output can still clear
     * the squared-energy floor via 60Hz coupling or drift bursts,
     * producing ghost beats with nonsense IBIs. */
    bool gate_now = ppg_sensor_looks_present();
    if (!gate_now) {
        /* On transition from OK → not OK, clean detector state and the
         * coherence ring so reconnection starts from zero rather than
         * gluing pre/post-disconnect beats together. */
        if (g_sensor_gate_ok) {
            ppg_in_block = false;
            ppg_above_run = 0;
            ppg_last_beat_ms = 0;
            ppg_current_ibi_ms = 0;
            ppg_current_bpm = 0;
            coh_clear();
            ble_log("gate: sensor disconnected (raw out of range)");
        }
        g_sensor_gate_ok = false;
        return false;
    }
    if (!g_sensor_gate_ok) {
        /* Transition OK → not OK now done; mark gate-ok. */
        g_sensor_gate_ok = true;
        ble_log("gate: sensor reconnected");
    }

    /* v4.12.1: noise floor — secondary check. If MA_beat is below
     * PPG_MIN_MA_BEAT, the squared filtered signal has essentially
     * no energy. This catches weak-contact cases where raw looks
     * plausible but there's no pulse content. */
    if (ma_beat < PPG_MIN_MA_BEAT) {
        if (ppg_in_block) ppg_in_block = false;
        return false;
    }

    /* Threshold test — using integer-Q10 alpha to keep the hot path fast */
    float threshold = ma_beat + (ma_beat * PPG_ALPHA_Q10) / 1024.0f;
    bool above = (ma_peak > threshold);

    bool beat = false;

    /* v4.12.5: consecutive-above debounce. PPG_BLOCK_ENTRY_MIN samples
     * must be above threshold before we enter a block. Single spurious
     * crossings no longer open blocks that immediately close on the
     * next sample, which was the v4.12.4 "toggling" failure mode on
     * noisy PC-connected signals. */
    if (above) {
        if (ppg_above_run < 255) ppg_above_run++;
    } else {
        ppg_above_run = 0;
    }

    if (above && !ppg_in_block && ppg_above_run >= PPG_BLOCK_ENTRY_MIN) {
        /* Block start — but backdate by (PPG_BLOCK_ENTRY_MIN - 1) ticks
         * since that's when the first above-threshold sample actually
         * crossed. Keeps block_width_ms honest at the exit-side check. */
        ppg_in_block = true;
        uint32_t backdate_ms = (PPG_BLOCK_ENTRY_MIN - 1) * PPG_TICK_MS;
        ppg_block_start_ms = (time_ms > backdate_ms) ? (time_ms - backdate_ms) : time_ms;
        ppg_block_peak_val = filtered;
        ppg_block_peak_ms = time_ms;
    } else if (above && ppg_in_block) {
        /* In block — track argmax of filtered signal (NOT squared signal
         * — argmax of bandpass preserves peak location; argmax of squared
         * would shift slightly toward max-slope point) */
        if (filtered > ppg_block_peak_val) {
            ppg_block_peak_val = filtered;
            ppg_block_peak_ms = time_ms;
        }
    } else if (!above && ppg_in_block) {
        /* Block end — decide if this was a valid beat */
        uint32_t block_width_ms = time_ms - ppg_block_start_ms;
        uint32_t min_block_ms = PPG_W1 * PPG_TICK_MS;   /* 6×20 = 120ms */
        uint32_t since_last = (ppg_last_beat_ms > 0) ? (time_ms - ppg_last_beat_ms) : UINT32_MAX;

        /* Refractory: max(300ms, 0.6·IBI) — 0.6·IBI auto-adapts to HR;
         * 300ms floor rejects anything above 200 bpm (non-physiological) */
        uint32_t refractory_ms = PPG_REFRACTORY_MIN_MS;
        if (ppg_current_ibi_ms > 0) {
            uint32_t adaptive = (ppg_current_ibi_ms * 6) / 10;
            if (adaptive > refractory_ms) refractory_ms = adaptive;
        }

        if (block_width_ms >= min_block_ms && since_last > refractory_ms) {
            /* Valid beat. Update IBI + BPM. */
            if (ppg_last_beat_ms > 0) {
                uint32_t ibi = ppg_block_peak_ms - ppg_last_beat_ms;
                if (ibi >= 300 && ibi <= 2000) {
                    ppg_current_ibi_ms = ibi;
                    uint32_t bpm32 = 60000 / ibi;
                    ppg_current_bpm = (bpm32 > 220) ? 220 : (uint8_t)bpm32;
                }
            }
            ppg_last_beat_ms = ppg_block_peak_ms;
            ppg_beat_count_total++;
            beat = true;
        }

        ppg_in_block = false;
        ppg_block_peak_val = -1e9f;
    }

    /* v4.12.6: beefed-up stuck-state recovery. If we've had no beats
     * for 3 seconds but the signal is clearly alive (ma_beat is above
     * the noise floor), the detector is stuck — most often because a
     * burst of noise inflated ma_beat so high that the threshold is
     * above the peak of real beats. Full reset clears both MA buffers
     * so the detector can start fresh at the current signal level
     * instead of waiting ~660ms for the W2 window to flush naturally. */
    if (ppg_last_beat_ms > 0
        && (time_ms - ppg_last_beat_ms) > 3000
        && ma_beat > PPG_MIN_MA_BEAT * 2.0f) {
        ppg_reset_detector();
    }

    return beat;
}

/*******************************************************************************
 * COHERENCE / FREQUENCY-DOMAIN HRV (v4.13.0)
 *
 * Computes on-device what the dashboard previously did in JavaScript:
 * VLF/LF/HF band powers, LF resonance peak, coherence score.
 *
 * Pipeline (runs once per second in coherence_task):
 *   1. Take rolling IBI ring (pushed to by ppg_task on each beat)
 *   2. Linear-interpolate onto uniform 4Hz × 256 sample grid (64s window)
 *   3. Detrend (subtract mean)
 *   4. Apply Hanning window
 *   5. 256-point real-input FFT (input imag part = 0)
 *   6. Magnitude² → PSD (one-sided)
 *   7. Integrate power in VLF/LF/HF bands (Task Force 1996 bands)
 *   8. Find peak freq in LF resonance band (0.04-0.15 Hz)
 *   9. Coherence = peak_power² / total_power² × 250, clamped 0-100
 *
 * Publishes results to global `coh_state` which is:
 *   - Read by BLE emitter to build 0xF2 status packets
 *   - Will be read by future LED_MODE_COHERENCE_PACER to drive lens tint
 ******************************************************************************/

#define COH_IBI_RING_SIZE     120   /* ~2 min at 60 bpm */
#define COH_GRID_N            256   /* FFT size, power of 2 */
#define COH_GRID_HZ           4.0f  /* Resample rate (standard for HRV analysis) */
#define COH_WINDOW_S          (COH_GRID_N / COH_GRID_HZ)  /* 64 seconds */
#define COH_UPDATE_MS         1000  /* Recompute every 1 second */
#define COH_MIN_IBIS          20    /* Need at least this many beats to compute */

/* IBI ring — time + value. ppg_task writes on beat detection, coherence_task
 * reads on its 1Hz tick. Accessed from different tasks so writes are
 * protected by disabling interrupts briefly (cheap for such a small struct). */
typedef struct {
    uint32_t beat_ms;   /* absolute ms timestamp of the beat */
    uint16_t ibi_ms;    /* IBI ending at this beat */
} coh_ibi_entry_t;

static coh_ibi_entry_t coh_ibi_ring[COH_IBI_RING_SIZE];
static uint8_t coh_ibi_head = 0;        /* where next push goes */
static uint8_t coh_ibi_count = 0;       /* how many valid entries (caps at ring size) */
static portMUX_TYPE coh_mux = portMUX_INITIALIZER_UNLOCKED;

/* Push a beat into the ring. Called from ppg_task on every detected beat. */
static void coh_push_ibi(uint32_t beat_ms, uint16_t ibi_ms) {
    portENTER_CRITICAL(&coh_mux);
    coh_ibi_ring[coh_ibi_head].beat_ms = beat_ms;
    coh_ibi_ring[coh_ibi_head].ibi_ms = ibi_ms;
    coh_ibi_head = (coh_ibi_head + 1) % COH_IBI_RING_SIZE;
    if (coh_ibi_count < COH_IBI_RING_SIZE) coh_ibi_count++;
    portEXIT_CRITICAL(&coh_mux);
}

/* v4.13.1: clear IBI ring on sensor disconnect. Otherwise pre-disconnect
 * beats remain in the ring and get glued to post-reconnect beats in
 * coherence computation, producing a nonsense time series with a large
 * temporal gap. Called from ppg_detect when the raw-ADC gate trips. */
static void coh_clear(void) {
    portENTER_CRITICAL(&coh_mux);
    coh_ibi_head = 0;
    coh_ibi_count = 0;
    portEXIT_CRITICAL(&coh_mux);
}

/* Working buffers (kept static to avoid stack blowup — coherence_task
 * runs with only a 4KB stack) */
static float coh_fft_re[COH_GRID_N];
static float coh_fft_im[COH_GRID_N];
static float coh_hann[COH_GRID_N];

/* Published coherence state. Updated atomically by coherence_task,
 * read by BLE emitter + (future) LED pacer. The fields are single
 * words so torn reads don't matter for our use case. */
typedef struct {
    uint8_t  coherence;        /* 0-100 score */
    uint16_t resp_peak_mhz;    /* LF resonance peak × 1000 (milli-Hz) */
    uint16_t vlf_power;        /* clamped to u16 (scaled) */
    uint16_t lf_power;
    uint16_t hf_power;
    uint16_t total_power;
    uint8_t  lf_norm;          /* LF / (LF+HF) × 100 */
    uint8_t  hf_norm;          /* HF / (LF+HF) × 100 */
    uint16_t lf_hf_fp88;       /* LF/HF ratio × 256, clamped */
    uint8_t  n_ibis_used;      /* how many IBIs were in the most recent compute */
    uint32_t last_update_ms;
} coh_state_t;
static coh_state_t coh_state = {0};

/* One-time init: precompute Hanning window. Called once from app_main. */
static void coh_init(void) {
    for (int i = 0; i < COH_GRID_N; i++) {
        coh_hann[i] = 0.5f * (1.0f - cosf(2.0f * (float)M_PI * i / (COH_GRID_N - 1)));
    }
}

/* In-place Cooley-Tukey radix-2 FFT. N must be power of 2.
 * Hand-written, no external dependencies. ~30 lines.
 * For N=256 on 80MHz ESP32: roughly 3ms wall time. Once per second
 * that's 0.3% CPU. Fine.
 *
 * Input: re[N], im[N] — time domain (im = 0 for real input)
 * Output: re[N], im[N] — frequency domain (same arrays, in-place)
 * Output format: bin k = freq k × (Fs / N). First N/2 bins are unique
 * (Nyquist), remaining N/2 are conjugate-mirror. */
static void coh_fft(float *re, float *im, int N) {
    /* Bit-reversal permutation */
    for (int i = 1, j = 0; i < N; i++) {
        int bit = N >> 1;
        for (; j & bit; bit >>= 1) j ^= bit;
        j ^= bit;
        if (i < j) {
            float tr = re[i]; re[i] = re[j]; re[j] = tr;
            float ti = im[i]; im[i] = im[j]; im[j] = ti;
        }
    }
    /* Butterfly */
    for (int len = 2; len <= N; len <<= 1) {
        float ang = -2.0f * (float)M_PI / (float)len;
        float wlen_re = cosf(ang);
        float wlen_im = sinf(ang);
        int half = len >> 1;
        for (int i = 0; i < N; i += len) {
            float w_re = 1.0f, w_im = 0.0f;
            for (int j = 0; j < half; j++) {
                float u_re = re[i + j];
                float u_im = im[i + j];
                float v_re = re[i + j + half] * w_re - im[i + j + half] * w_im;
                float v_im = re[i + j + half] * w_im + im[i + j + half] * w_re;
                re[i + j] = u_re + v_re;
                im[i + j] = u_im + v_im;
                re[i + j + half] = u_re - v_re;
                im[i + j + half] = u_im - v_im;
                float new_w_re = w_re * wlen_re - w_im * wlen_im;
                w_im = w_re * wlen_im + w_im * wlen_re;
                w_re = new_w_re;
            }
        }
    }
}

/* Interpolate irregularly-spaced IBI series onto uniform time grid.
 * Linear interpolation (Task Force doesn't specify, linear is the common choice
 * and matches what Kubios uses for short windows).
 *
 * Input: beat_ms[] and ibi_ms[] arrays of length N_beats, sorted by beat_ms.
 * Output: grid[COH_GRID_N] at spacing 1/COH_GRID_HZ starting from beat_ms[0].
 * The grid represents COH_WINDOW_S seconds of IBI time series resampled
 * to COH_GRID_HZ. */
static void coh_resample(const uint32_t *beat_ms, const uint16_t *ibi_ms,
                         int n_beats, float *grid) {
    uint32_t t0 = beat_ms[0];
    for (int i = 0; i < COH_GRID_N; i++) {
        float t = (float)i / COH_GRID_HZ;        /* seconds from t0 */
        uint32_t abs_ms = t0 + (uint32_t)(t * 1000.0f);
        /* Find bracketing beats j, j+1 such that beat_ms[j] ≤ abs_ms ≤ beat_ms[j+1] */
        if (abs_ms <= beat_ms[0]) { grid[i] = (float)ibi_ms[0]; continue; }
        if (abs_ms >= beat_ms[n_beats - 1]) { grid[i] = (float)ibi_ms[n_beats - 1]; continue; }
        /* Binary search for efficiency */
        int lo = 0, hi = n_beats - 1;
        while (hi - lo > 1) {
            int mid = (lo + hi) / 2;
            if (beat_ms[mid] <= abs_ms) lo = mid; else hi = mid;
        }
        /* Linear interp between [lo] and [hi] */
        uint32_t dt = beat_ms[hi] - beat_ms[lo];
        float frac = (dt > 0) ? ((float)(abs_ms - beat_ms[lo]) / (float)dt) : 0.0f;
        grid[i] = (float)ibi_ms[lo] + frac * ((float)ibi_ms[hi] - (float)ibi_ms[lo]);
    }
}

/* Main coherence computation — full pipeline.
 * Reads coh_ibi_ring (atomically snapshots it first), does all math,
 * writes results to coh_state. */
static void coh_compute(void) {
    /* Snapshot ring under critical section */
    static uint32_t snap_beat_ms[COH_IBI_RING_SIZE];
    static uint16_t snap_ibi_ms[COH_IBI_RING_SIZE];
    uint8_t snap_count;

    portENTER_CRITICAL(&coh_mux);
    snap_count = coh_ibi_count;
    if (snap_count > 0) {
        /* Ring is a circular buffer with head pointing at next-insert.
         * Oldest entry is at (head - count + RING_SIZE) % RING_SIZE.
         * Copy in chronological order. */
        int start = (coh_ibi_head + COH_IBI_RING_SIZE - snap_count) % COH_IBI_RING_SIZE;
        for (int i = 0; i < snap_count; i++) {
            int idx = (start + i) % COH_IBI_RING_SIZE;
            snap_beat_ms[i] = coh_ibi_ring[idx].beat_ms;
            snap_ibi_ms[i] = coh_ibi_ring[idx].ibi_ms;
        }
    }
    portEXIT_CRITICAL(&coh_mux);

    if (snap_count < COH_MIN_IBIS) return;

    /* Restrict to last COH_WINDOW_S seconds — the FFT grid covers only
     * this duration. Older beats would wrap the grid or be unused. */
    uint32_t window_start_ms = snap_beat_ms[snap_count - 1] > (uint32_t)(COH_WINDOW_S * 1000.0f)
        ? snap_beat_ms[snap_count - 1] - (uint32_t)(COH_WINDOW_S * 1000.0f)
        : 0;
    int first_in_window = 0;
    while (first_in_window < snap_count
           && snap_beat_ms[first_in_window] < window_start_ms) {
        first_in_window++;
    }
    int n_used = snap_count - first_in_window;
    if (n_used < COH_MIN_IBIS) return;

    /* Interpolate onto uniform grid */
    coh_resample(&snap_beat_ms[first_in_window], &snap_ibi_ms[first_in_window],
                 n_used, coh_fft_re);

    /* Detrend — subtract mean, critical for FFT because DC bin
     * dominates otherwise */
    float sum = 0.0f;
    for (int i = 0; i < COH_GRID_N; i++) sum += coh_fft_re[i];
    float mean = sum / COH_GRID_N;
    for (int i = 0; i < COH_GRID_N; i++) coh_fft_re[i] -= mean;

    /* Apply Hanning window + zero imaginary part */
    for (int i = 0; i < COH_GRID_N; i++) {
        coh_fft_re[i] *= coh_hann[i];
        coh_fft_im[i] = 0.0f;
    }

    /* FFT */
    coh_fft(coh_fft_re, coh_fft_im, COH_GRID_N);

    /* One-sided PSD — store magnitude² back into coh_fft_re.
     * Only need bins 0..N/2-1 since real input produces conjugate-symmetric
     * output. Freq at bin k: f_k = k × Fs / N = k × 4/256 = k × 0.015625 Hz */
    float psd[COH_GRID_N / 2];
    for (int i = 0; i < COH_GRID_N / 2; i++) {
        psd[i] = coh_fft_re[i] * coh_fft_re[i] + coh_fft_im[i] * coh_fft_im[i];
    }

    /* Band integration — Task Force 1996 bands.
     * Bin indices (0.015625 Hz/bin):
     *   VLF: 0.003-0.04 → bins 1-2  (skip DC bin 0)
     *   LF:  0.04-0.15  → bins 3-9
     *   HF:  0.15-0.4   → bins 10-25
     *
     * v4.13.8: this sum (bins 1-25, covering 0.016-0.391 Hz) IS the
     * coherence denominator. Client v13.10 uses Lomb-Scargle with 200
     * points over 0.003-0.4 Hz — same effective range. My v4.13.5-v4.13.7
     * incorrectly used full-spectrum total (bins 1-127, covering up to
     * Nyquist at 2 Hz), which is 2-3× larger and systematically drove
     * coherence down. Now matches client. */
    float vlf = 0.0f, lf = 0.0f, hf = 0.0f;
    for (int i = 1; i <= 2; i++)  vlf += psd[i];  /* skip DC bin 0 */
    for (int i = 3; i <= 9; i++)   lf += psd[i];
    for (int i = 10; i <= 25; i++) hf += psd[i];
    float total = vlf + lf + hf;

    /* LF resonance peak (Lehrer/Vaschillo method, matches client v13.10):
     * 1. Find argmax within LF band (0.04-0.15 Hz = bins 3-9)
     * 2. Use single peak bin as numerator.
     *
     *    v4.13.7 fix: client code has `|freqs[i]-peak| <= 0.015` window.
     *    With df = 4/256 = 0.015625 Hz, adjacent bins are 0.015625 Hz
     *    from the peak — which is GREATER than 0.015, so excluded.
     *    The client's window is effectively just the peak bin itself.
     *    Previous firmware used ±1 bin = 3 bins, making numerator 2-3×
     *    too large and inflating coherence. Now matches client.
     *
     * 3. Coherence = (peak_power / total_power_full_spectrum) × 250,
     *    clamped 0-100. */
    int peak_bin = 3;
    float peak_pow = psd[3];
    for (int i = 4; i <= 9; i++) {
        if (psd[i] > peak_pow) {
            peak_pow = psd[i];
            peak_bin = i;
        }
    }

    float coherence = 0.0f;
    if (total > 1e-6f) {
        float ratio = peak_pow / total;
        coherence = ratio * 250.0f;
        if (coherence > 100.0f) coherence = 100.0f;
        if (coherence < 0.0f) coherence = 0.0f;
    }

    /* Normalize band powers to fit u16 for the display/status packet. */
    float scale = 1.0f;
    float maxband = total;
    if (maxband > 6.5e4f) scale = 6.5e4f / maxband;

    float lfhf = (hf > 1.0f) ? (lf / hf) : 0.0f;
    uint16_t lfhf_fp88 = (uint16_t)(lfhf * 256.0f + 0.5f);
    if (lfhf > 255.0f) lfhf_fp88 = 0xFFFF;

    float lf_plus_hf = lf + hf;
    uint8_t lf_norm = (lf_plus_hf > 0) ? (uint8_t)(lf / lf_plus_hf * 100.0f) : 0;
    uint8_t hf_norm = (lf_plus_hf > 0) ? (uint8_t)(hf / lf_plus_hf * 100.0f) : 0;

    /* Publish. Single-word writes so torn reads on other tasks are
     * benign — they'll see the new value on next access. */
    coh_state.coherence      = (uint8_t)coherence;
    coh_state.resp_peak_mhz  = (uint16_t)((float)peak_bin * COH_GRID_HZ / COH_GRID_N * 1000.0f);
    coh_state.vlf_power      = (uint16_t)(vlf * scale);
    coh_state.lf_power       = (uint16_t)(lf * scale);
    coh_state.hf_power       = (uint16_t)(hf * scale);
    coh_state.total_power    = (uint16_t)(total * scale);
    coh_state.lf_norm        = lf_norm;
    coh_state.hf_norm        = hf_norm;
    coh_state.lf_hf_fp88     = lfhf_fp88;
    coh_state.n_ibis_used    = (n_used > 255) ? 255 : (uint8_t)n_used;
    coh_state.last_update_ms = (uint32_t)(esp_timer_get_time() / 1000);
}

/* Emit coherence packet on 0xFF03 status characteristic.
 * 0xF2 type, 18 bytes total. Dashboard v13.10+ parses.
 *
 * [0]   type = 0xF2
 * [1]   coherence 0-100
 * [2-3] resp peak freq in mHz (LF resonance peak)
 * [4-5] vlf_power (u16, scaled to fit)
 * [6-7] lf_power
 * [8-9] hf_power
 * [10-11] total_power
 * [12]  lf_norm 0-100
 * [13]  hf_norm 0-100
 * [14-15] lf_hf_fp88 (fixed point, divide by 256 for ratio)
 * [16]  n_ibis_used
 * [17]  reserved/flags */
static void coh_emit_packet(void) {
    if (!notifications_enabled || !is_connected) return;
    if (coh_state.last_update_ms == 0) return;  /* never computed yet */

    uint8_t pkt[18];
    pkt[0]  = 0xF2;
    pkt[1]  = coh_state.coherence;
    pkt[2]  = (uint8_t)(coh_state.resp_peak_mhz & 0xFF);
    pkt[3]  = (uint8_t)((coh_state.resp_peak_mhz >> 8) & 0xFF);
    pkt[4]  = (uint8_t)(coh_state.vlf_power & 0xFF);
    pkt[5]  = (uint8_t)((coh_state.vlf_power >> 8) & 0xFF);
    pkt[6]  = (uint8_t)(coh_state.lf_power & 0xFF);
    pkt[7]  = (uint8_t)((coh_state.lf_power >> 8) & 0xFF);
    pkt[8]  = (uint8_t)(coh_state.hf_power & 0xFF);
    pkt[9]  = (uint8_t)((coh_state.hf_power >> 8) & 0xFF);
    pkt[10] = (uint8_t)(coh_state.total_power & 0xFF);
    pkt[11] = (uint8_t)((coh_state.total_power >> 8) & 0xFF);
    pkt[12] = coh_state.lf_norm;
    pkt[13] = coh_state.hf_norm;
    pkt[14] = (uint8_t)(coh_state.lf_hf_fp88 & 0xFF);
    pkt[15] = (uint8_t)((coh_state.lf_hf_fp88 >> 8) & 0xFF);
    pkt[16] = coh_state.n_ibis_used;
    pkt[17] = 0;
    esp_err_t err = esp_ble_gatts_send_indicate(gatts_if_global, conn_id_global,
                                                status_char_handle, sizeof(pkt), pkt, false);
    if (err != ESP_OK) ble_send_errors++;
}

/* Task: once per second, recompute coherence and emit packet.
 * Priority 4 — above led_task (1), hall_task (2), and below ppg_task (10).
 * Makes sure PPG detection never competes with coherence math. */
static void coherence_task(void *arg) {
    ESP_LOGI(TAG, "Coherence task v4.13.8 started (Fs=%.1fHz, N=%d, window=%.0fs)",
             COH_GRID_HZ, COH_GRID_N, COH_WINDOW_S);
    TickType_t last_wake = xTaskGetTickCount();
    while (1) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(COH_UPDATE_MS));
        uint32_t t0 = (uint32_t)(esp_timer_get_time() / 1000);
        coh_compute();
        uint32_t dt = (uint32_t)(esp_timer_get_time() / 1000) - t0;
        coh_emit_packet();
        /* Log occasionally so dashboard FW Log tab shows us life + timing */
        static uint8_t log_counter = 0;
        if (++log_counter >= 10) {  /* every 10 sec */
            log_counter = 0;
            ble_log("coh=%u resp=%umHz n=%u dt=%ums",
                    coh_state.coherence, coh_state.resp_peak_mhz,
                    coh_state.n_ibis_used, dt);
        }
    }
}

/* Emit one PPG sample packet on 0xFF04.
 * Format (13 bytes, little-endian):
 *   [0x02]  type — v13.2+ parses this; legacy 0x01 preserved for older clients
 *   [raw   u16]  12-bit ADC reading, 0–4095 (high bits always 0)
 *   [idx   u16]  sample index (wraps at 65536 = 21.8 min at 50 Hz — fine for session-length sessions)
 *   [ts    u32]  ms timestamp from esp_timer_get_time (wraps every 49 days)
 *   [flags u8]   bit 0 = beat detected on this sample, bit 1 = in_block
 *   [ibi   u16]  current IBI in ms (0 until first valid IBI)
 *   [bpm   u8]   current BPM (0 until first valid IBI)
 *
 * Only emits if a client has subscribed to notifications on the PPG
 * CCCD (ppg_notifications_enabled). No subscription → zero RF cost
 * from PPG streaming.
 */
static void ppg_send_sample(uint16_t raw, uint16_t idx, uint32_t ts,
                            bool beat, bool in_block,
                            uint16_t ibi, uint8_t bpm) {
    if (!ppg_notifications_enabled || !is_connected) return;

    uint8_t pkt[13];
    pkt[0]  = 0x02;
    pkt[1]  = (uint8_t)(raw & 0xFF);
    pkt[2]  = (uint8_t)((raw >> 8) & 0xFF);
    pkt[3]  = (uint8_t)(idx & 0xFF);
    pkt[4]  = (uint8_t)((idx >> 8) & 0xFF);
    pkt[5]  = (uint8_t)(ts & 0xFF);
    pkt[6]  = (uint8_t)((ts >> 8) & 0xFF);
    pkt[7]  = (uint8_t)((ts >> 16) & 0xFF);
    pkt[8]  = (uint8_t)((ts >> 24) & 0xFF);
    pkt[9]  = (beat ? 0x01 : 0x00) | (in_block ? 0x02 : 0x00);
    pkt[10] = (uint8_t)(ibi & 0xFF);
    pkt[11] = (uint8_t)((ibi >> 8) & 0xFF);
    pkt[12] = bpm;

    esp_err_t err = esp_ble_gatts_send_indicate(gatts_if_global, conn_id_global,
                                                ppg_char_handle, 13, pkt, false);
    if (err != ESP_OK) ble_send_errors++;
}

/* Oversampled ADC read — v4.12.4: MEDIAN of PPG_OVERSAMPLE reads.
 *
 * Original v4.12.0 used MEAN averaging for noise reduction. In field
 * testing this failed against powerline hash on the bodge-wired input:
 * mean averages noise INTO the output, so a single 60Hz zero-crossing
 * during the 8-read burst contaminates the result. MEDIAN is far more
 * robust to impulsive noise — up to 3 of 8 reads can be corrupted and
 * the median still returns a clean sample.
 *
 * Cost: trivial. Insertion sort on 8 uint16s is ~30 cycles. The 8 ADC
 * reads themselves dominate at ~240µs total; the sort is <1µs.
 *
 * Alternative considered: trimmed mean (sort, drop top 2 and bottom 2,
 * average the middle 4). Slightly better noise performance than pure
 * median but more code. Revisit if median isn't enough.
 */
static inline uint16_t ppg_read_oversampled(void) {
    uint16_t samples[PPG_OVERSAMPLE];
    for (int i = 0; i < PPG_OVERSAMPLE; i++) {
        int r = adc1_get_raw(PPG_ADC_CHANNEL);
        if (r < 0) r = 0;
        if (r > 4095) r = 4095;
        samples[i] = (uint16_t)r;
    }
    /* Insertion sort (best for small N, stable, branch-friendly) */
    for (int i = 1; i < PPG_OVERSAMPLE; i++) {
        uint16_t key = samples[i];
        int j = i - 1;
        while (j >= 0 && samples[j] > key) {
            samples[j + 1] = samples[j];
            j--;
        }
        samples[j + 1] = key;
    }
    /* Median of 8: average of samples[3] and samples[4] */
    return (uint16_t)((samples[PPG_OVERSAMPLE / 2 - 1] +
                       samples[PPG_OVERSAMPLE / 2]) / 2);
}

/* ppg_task — the whole pipeline runs here at 50 Hz.
 * Priority 3: above led_task (1) and hall_task (2), below ota_task (5).
 * The detection pipeline must never starve for CPU — drifting sample
 * timing wrecks IBI accuracy. At 50 Hz this task uses <0.5% CPU so
 * priority is mostly defensive.
 */
/* v4.12.2: ADC scan mode — cycles through all ADC1 channels and reports
 * their readings to the FW log. Called from ppg_task when
 * adc_scan_enabled is true. Lets us find which pin has the PulseSensor
 * signal without reflashing. Enabled by BLE cmd 0xC0 0x00. */
static void ppg_emit_adc_scan(void) {
    /* ADC1 channels map to these GPIOs on ESP32:
     *   CH0=GPIO36 CH3=GPIO39 CH4=GPIO32 CH5=GPIO33
     *   CH6=GPIO34 CH7=GPIO35
     * (CH1/CH2 exist but GPIO37/38 are not broken out on WROOM-32)
     * We configure attenuation for each channel just-in-time so any
     * pin can be read without pre-setup. */
    const adc1_channel_t chans[] = {
        ADC1_CHANNEL_0, ADC1_CHANNEL_3, ADC1_CHANNEL_4, ADC1_CHANNEL_5,
        ADC1_CHANNEL_6, ADC1_CHANNEL_7
    };
    uint16_t reads[6];
    for (int i = 0; i < 6; i++) {
        adc1_config_channel_atten(chans[i], ADC_ATTEN_DB_12);
        int r = adc1_get_raw(chans[i]);
        reads[i] = (r < 0) ? 0 : (r > 4095 ? 4095 : r);
    }
    ble_log("SCAN 36=%u 39=%u 32=%u 33=%u 34=%u 35=%u",
            reads[0], reads[1], reads[2], reads[3], reads[4], reads[5]);
}

static void ppg_task(void *arg) {
    uint16_t sample_idx = 0;
    TickType_t last_wake = xTaskGetTickCount();
    int64_t last_wake_us = esp_timer_get_time();
    uint32_t last_stats_ms = 0;
    uint32_t last_hb_ms = 0;
    uint32_t last_health_ms = 0;
    uint32_t last_scan_ms = 0;
    uint32_t last_auto_reset_ms = 0;   /* v4.13.2: periodic detector refresh */
    uint32_t tick_count_local = 0;
    uint32_t jitter_ticks_over = 0;   /* ticks where actual delta > 25ms (25% over target) */

    ESP_LOGI(TAG, "PPG task v4.13.8 started @ %d Hz (GPIO%d)",
             PPG_SAMPLE_RATE_HZ, PPG_ADC_GPIO);

    while (1) {
        int64_t now_us = esp_timer_get_time();
        uint32_t now_ms = (uint32_t)(now_us / 1000);

        /* v4.12.2: measure actual wake-to-wake delta in µs. Target is
         * 20000µs at 50Hz. Record max over the 5s window. Also count
         * how many ticks ran >25ms late (≥25% overrun) — that's the
         * best signal of preemption under load. */
        uint32_t delta_us = (uint32_t)(now_us - last_wake_us);
        last_wake_us = now_us;
        tick_count_local++;
        if (tick_count_local > 1) {   /* Skip first tick (undefined delta) */
            if (delta_us > ppg_jitter_max_us) ppg_jitter_max_us = delta_us;
            if (delta_us > 25000) jitter_ticks_over++;
        }

        uint16_t raw = ppg_read_oversampled();
        adc_stats_push(raw);

        float filtered = ppg_bandpass((float)raw);
        bool is_beat = ppg_detect(filtered, now_ms);

        /* v4.12.1: drive LED_MODE_PULSE_ON_BEAT. */
        if (is_beat) {
            beat_pulse_start_tick = xTaskGetTickCount();
            /* v4.13.0: also feed the coherence IBI ring. Only push
             * beats with a valid IBI (not the very first beat of a
             * session, which has no predecessor). */
            if (ppg_current_ibi_ms > 0) {
                coh_push_ibi(now_ms, (uint16_t)ppg_current_ibi_ms);
            }
        }

        ppg_send_sample(raw, sample_idx, now_ms,
                        is_beat, ppg_in_block,
                        (uint16_t)ppg_current_ibi_ms, ppg_current_bpm);

        /* ADC stats every 500ms */
        if (now_ms - last_stats_ms >= 500) {
            last_stats_ms = now_ms;
            ppg_emit_adc_stats();
        }

        /* v4.12.2: ADC scan mode every 500ms when enabled */
        if (adc_scan_enabled && now_ms - last_scan_ms >= 500) {
            last_scan_ms = now_ms;
            ppg_emit_adc_scan();
        }

        /* v4.13.2: periodic detector freshening. Every 30 seconds,
         * reset the adaptive MA buffers so they can't drift into a
         * noise-responsive state. Over long noisy sessions the MA_beat
         * floor can settle at a value that makes every small burst of
         * ambient energy clear the threshold, producing a flood of
         * false positives. Hard-resetting once per 30s gives the
         * adaptive baseline a fresh start.
         *
         * Only refresh if not currently in a block (don't interrupt
         * an in-progress real beat). If the detector is chronically
         * in_block on noise, we'll still catch it on the next 30s
         * tick when it happens to be out-of-block.
         *
         * Cost: ~1 second of blind detection per 30s window after
         * warmup re-engages. Acceptable tradeoff for robustness. */
        if (now_ms - last_auto_reset_ms >= 30000) {
            last_auto_reset_ms = now_ms;
            if (!ppg_in_block) {
                ppg_reset_detector();
                ble_log("auto-reset detector (30s refresh)");
            }
        }

        /* Heartbeat every 5s — basic liveness + PPG state */
        if (now_ms - last_hb_ms >= 5000) {
            last_hb_ms = now_ms;
            /* v4.13.3: query actual current CPU frequency via RTC clock config.
             * Visible here instead of just UART because the boot-time ESP_LOGI
             * message doesn't reach the BLE FW Log tab (BLE not up yet). */
            rtc_cpu_freq_config_t cur_cpu;
            rtc_clk_cpu_freq_get_config(&cur_cpu);
            ble_log("ppg t=%us cpu=%uMHz raw=%u gate=%s MAb=%.1f beats=%u jitmax=%uus over=%u",
                    now_ms / 1000, (unsigned)cur_cpu.freq_mhz,
                    (unsigned)raw,
                    g_sensor_gate_ok ? "ok" : "off",
                    ppg_beat_sum / (ppg_beat_count ? ppg_beat_count : 1),
                    (unsigned)ppg_beat_count_total,
                    (unsigned)ppg_jitter_max_us, (unsigned)jitter_ticks_over);
            /* Reset jitter window */
            ppg_jitter_max_us = 0;
            jitter_ticks_over = 0;
        }

        /* v4.12.2: health report every 10s — heap, stack, BLE errors */
        if (now_ms - last_health_ms >= 10000) {
            last_health_ms = now_ms;
            UBaseType_t hwm = uxTaskGetStackHighWaterMark(NULL);
            uint32_t heap_free = esp_get_free_heap_size();
            uint32_t heap_min  = esp_get_minimum_free_heap_size();
            ble_log("health heap=%u min=%u stack=%u ble_err=%u",
                    (unsigned)heap_free, (unsigned)heap_min,
                    (unsigned)hwm, (unsigned)ble_send_errors);
        }

        sample_idx++;
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(PPG_TICK_MS));
    }
}

/*******************************************************************************
 * MAIN APPLICATION
 ******************************************************************************/
void app_main(void) {
    esp_err_t ret;

    /* v4.12.9: drop CPU to 80MHz for power savings.
     *
     * Two paths, whichever works:
     *   1. If CONFIG_PM_ENABLE=y: use esp_pm_configure to cap the CPU
     *      at 80MHz (also enables DFS so CPU drops further when idle).
     *   2. Otherwise: use rtc_clk_cpu_freq_set_config directly.
     *      Lower-level API, works without PM framework.
     *
     * Note: 80MHz is the MINIMUM CPU frequency compatible with BLE —
     * APB bus clock is tied to CPU clock and BLE requires 80MHz APB.
     * Going below 80MHz will break the BLE stack.
     *
     * Expected savings: 240→80 MHz is ~20mA reduction on ESP32-D0WD.
     * Our PPG workload at 50Hz uses <1% CPU at 80MHz so no performance
     * impact. Lomb-Scargle is client-side not firmware.
     *
     * If this appears to do nothing, set CONFIG_ESP32_DEFAULT_CPU_FREQ_MHZ=80
     * in sdkconfig and rebuild — that forces it at boot regardless. */
#if CONFIG_PM_ENABLE
    {
        esp_pm_config_esp32_t pm_cfg = {
            .max_freq_mhz = 80,
            .min_freq_mhz = 80,
            .light_sleep_enable = false,  /* light sleep breaks BLE streaming */
        };
        esp_err_t pm_err = esp_pm_configure(&pm_cfg);
        if (pm_err == ESP_OK) {
            ESP_LOGI(TAG, "CPU clock: 80MHz (via esp_pm_configure)");
        } else {
            ESP_LOGW(TAG, "esp_pm_configure failed: %s — trying rtc_clk path", esp_err_to_name(pm_err));
        }
    }
#else
    {
        rtc_cpu_freq_config_t freq_cfg;
        if (rtc_clk_cpu_freq_mhz_to_config(80, &freq_cfg)) {
            rtc_clk_cpu_freq_set_config(&freq_cfg);
            ESP_LOGI(TAG, "CPU clock: 80MHz (via rtc_clk direct)");
        } else {
            ESP_LOGW(TAG, "rtc_clk_cpu_freq_mhz_to_config(80) failed");
        }
    }
#endif

    /* Print startup banner */
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "Smart Glasses v%s", FIRMWARE_VERSION);
    ESP_LOGI(TAG, "  Boot: Program 1 = BREATHE %d BPM %s @ %d%% brightness",
             breathe_bpm, breathe_wave == 0 ? "sine" : "linear", DEFAULT_BRIGHTNESS);
#if PPG_TEST_BUILD
    ESP_LOGW(TAG, "  *** PPG TEST BUILD — Hall sensor DISABLED ***");
    ESP_LOGW(TAG, "  Magnet will do nothing. BLE commands still work.");
#else
    ESP_LOGI(TAG, "  Hall: short tap (%d-%dms) = next program, %ds hold = sleep",
             HALL_SHORT_MIN_MS, HALL_SHORT_MAX_MS, HALL_LONG_MS / 1000);
#endif
    ESP_LOGI(TAG, "  Programs: 1=BREATHE 2=BREATHE+STROBE 3=STROBE");
    ESP_LOGI(TAG, "  Session: %d minutes", DEFAULT_SESSION_MIN);
    ESP_LOGI(TAG, "  AC Drive: 100Hz gptimer ISR, strobe phase-synced (GPIO26/27)");
    ESP_LOGI(TAG, "  Power: -6dBm TX (adv) / -12dBm (conn), 100-200ms adv interval");
    ESP_LOGI(TAG, "  BLE auto-off: full stack teardown after %ds no connect (v4.11.1)",
             BLE_IDLE_TIMEOUT_MS / 1000);
    
    /* Get partition info */
    const esp_partition_t *running = esp_ota_get_running_partition();
    ESP_LOGI(TAG, "  Partition: %s @ 0x%lx", running->label, running->address);
    
    /* Log wake reason */
    esp_sleep_wakeup_cause_t wake = esp_sleep_get_wakeup_cause();
    ESP_LOGI(TAG, "  Wake reason: %d", wake);
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "Hall: tap=next program, 5s hold=sleep");
    ESP_LOGI(TAG, "BLE: adv auto-off after 60s — tap hall to re-arm");
    ESP_LOGI(TAG, "BLE modes: A5=static A6=strobe B0=breathe");
    ESP_LOGI(TAG, "Common: A2=brightness A4=session A7=sleep");
    ESP_LOGI(TAG, "Strobe: AB=freq AC=duty | Breathe: B1-B5");

    /* Initialize NVS */
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* Initialize Bluetooth — release Classic BT memory once; stack itself
     * is brought up via the ble_stack_init helper so the same code path
     * runs at boot and on every hall re-arm after an auto-off teardown. */
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
    ESP_ERROR_CHECK(ble_stack_init());

    /* v4.11.0: arm the initial BLE idle-off deadline.
     * Advertising itself starts asynchronously (triggered by the GAP
     * ADV_DATA_SET_COMPLETE_EVT after adv data is configured during GATT
     * service creation). Setting the deadline before adv is live is safe:
     * the main-loop timeout check gates on ble_adv_active first, so it
     * can't fire until advertising has actually started. */
    ble_adv_reset_deadline();

#if PPG_TEST_BUILD
    /* v4.12.0 PPG_TEST_BUILD: hall sensor completely disabled.
     * GPIO not configured, task not started, no polling, no gestures.
     * The pin floats (input with internal pull-up would be normal, but
     * even that's skipped here for total isolation). Magnet has zero
     * effect on the device. */
    ESP_LOGW(TAG, "PPG_TEST_BUILD=1 — skipping hall GPIO init");
#else
    /* Initialize Hall sensor GPIO */
    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << HALL_PIN,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
#endif

    /* Initialize PWM */
    pwm_init();

    /* Start hardware drive timer (100µs gptimer ISR — AC + strobe) */
    drive_timer_init();

    /* v4.12.0: initialize ADC1 for PulseSensor.
     * Width 12-bit = 0–4095 codes. Attenuation 11dB gives ~0–3.1V
     * full-scale, which cleanly maps the PulseSensor's ~1.65V
     * midrail ±0.4V swing into mid-ADC range with headroom. */
    ESP_ERROR_CHECK(adc1_config_width(ADC_WIDTH_BIT_12));
    ESP_ERROR_CHECK(adc1_config_channel_atten(PPG_ADC_CHANNEL, ADC_ATTEN_DB_12));
    ESP_LOGI(TAG, "PPG ADC initialized (GPIO%d, 12-bit, 11dB atten)", PPG_ADC_GPIO);

    /* Create LED control task (session management + breathing) */
    xTaskCreate(led_task, "led_ctrl", 4096, NULL, 1, &led_task_handle);

    /* Create OTA task (deferred OTA ops — esp_ota_begin/end block too long for BLE callback) */
    xTaskCreate(ota_task, "ota_task", 8192, NULL, 5, &ota_task_handle);

#if PPG_TEST_BUILD
    /* v4.12.0 PPG_TEST_BUILD: hall_task NOT started. No gesture polling,
     * no program cycling, no sleep-on-hold. The device stays in its
     * boot program indefinitely until the session timer expires or BLE
     * A7 00 (sleep) is sent. */
#else
    /* Create Hall gesture task (50ms polling, short-tap = next program, 5s hold = sleep) */
    xTaskCreate(hall_task, "hall_task", 2048, NULL, 2, NULL);
#endif

    /* v4.12.0: Create PPG sampling + detection task (50 Hz).
     * v4.12.2: priority raised 3 → 10. At 50 Hz with hard timing
     * requirements this task must not be starved by BLE housekeeping,
     * OTA work, or anything else. Priority 10 puts it above ota_task (5)
     * and well above led_task (1) and hall_task (2, when enabled).
     * Stack 4096 — filter arrays are globals so the stack only holds
     * locals. */
    xTaskCreate(ppg_task, "ppg_task", 4096, NULL, 10, &ppg_task_handle);

    /* v4.13.0: Create coherence computation task (1 Hz).
     * Priority 4 — above led_task (1), hall_task (2), below ota_task (5)
     * and ppg_task (10). The FFT work is bursty (~3-5ms per second) so
     * this priority guarantees it can't starve PPG detection but also
     * won't be starved by low-priority work.
     *
     * Stack 6144 — FFT helpers use small locals, mostly static buffers,
     * but interp snapshots two static arrays of 120 × 2 bytes each on
     * stack-like access. 6KB is generous. */
    coh_init();
    xTaskCreate(coherence_task, "coh_task", 6144, NULL, 4, NULL);

    /* Main loop - monitor session state and BLE idle timeout */
    while (1) {
        /* Check if session ended */
        if (!session_active && led_task_handle == NULL && !in_ota_mode) {
            ESP_LOGI(TAG, "Session complete, entering sleep");
            enter_deep_sleep();
        }

        /* v4.11.0/v4.11.1: BLE idle auto-off.
         * After BLE_IDLE_TIMEOUT_MS of no connection, tear down the entire
         * BT stack (controller + Bluedroid). v4.11.0 only stopped advertising,
         * which left the controller running and emitting periodic RF
         * housekeeping bursts (~1.5A spikes every few seconds in traces).
         * v4.11.1 kills the radio completely.
         * Gated on ble_stack_up so it only fires once per window, and on
         * !is_connected / !in_ota_mode to avoid pulling the rug during use.
         *
         * v4.12.0 PPG_TEST_BUILD: SKIPPED. Without hall gestures there's no
         * way to re-arm BLE once it's off — user would have to power-cycle
         * the device, which is annoying during bench testing. Leave BLE
         * advertising forever in test builds. */
#if !PPG_TEST_BUILD
        if (ble_stack_up && ble_adv_active && !is_connected && !in_ota_mode &&
            ble_idle_deadline_tick != 0 &&
            xTaskGetTickCount() >= ble_idle_deadline_tick) {
            ESP_LOGI(TAG, "BLE idle timeout (%ds no connect) — radio off",
                     BLE_IDLE_TIMEOUT_MS / 1000);
            ble_stack_teardown();
        }
#endif

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

# RF-A250 / Zhengbang HLH oven: serial protocol and replacement controller

I recovered this by static analysis of `ZBHLH_V1.0\Main.exe`, an MFC/C++ program. I disassembled it but never ran it. Addresses in parentheses point to the code in that binary.

## Link settings
* **9600 baud, 8N1, no flow control.** The app first opens the port at 56000 7E1 (0x401df9), then immediately switches to 9600 8N1 (0x405934).
* Before every write the app purges both buffers. Read timeout is about 1 s total, with 20 ms between bytes.
* **CRC:** CRC-16/MODBUS (poly 0xA001 reflected, init 0xFFFF) (0x401c10). It is **sent high byte first**, which is the reverse of real Modbus RTU, so a stock Modbus library won't work.

## Frames

| Purpose | TX | RX | Notes |
|---|---|---|---|
| Link / enter PC mode | `02` | `06` (1 byte) | App waits 500 ms before reading (0x404cf0) |
| Unlink | `03 00 00 C0 81` | 3 bytes, first = `06` | (0x404d90) |
| Read temperature | `A1 38 02 23 B2` | `A1 38 02 TH TL CH CL` (7) | Temperature = `TH<<8 \| TL`, whole °C. CRC is over the first 5 bytes (0x404470). `38` looks like an address and `02` a byte count |
| Heater power | `A0 34 01 PP CH CL` | `A0 34 01 CH CL` (5) | PP = 0–100 % (0x404f50) |
| **Cooling fan speed** (0x35) | `A0 35 01 VV CH CL` | 5 bytes | VV = 0–100 %. App writes 0, 50 or 100 (0x4050d0) |
| **Convection fan enable** (0x36) | `A0 36 01 VV CH CL` | 5 bytes | 0/1. App writes 1 at run start and 0 at the end (0x405250) |

Examples: heater 0 % = `A0 34 01 00 7A 62`, heater 100 % = `A0 34 01 64 91 63`, fan enable on = `A0 36 01 01 7A 02`, fan speed 100 % = `A0 35 01 64 51 32`.

These seven frames are every serial write in the program.

## What the vendor app does (and why it overshoots)

Run loop (0x403c10): every 0.5 s it reads the temperature and runs one PID step.

* **PID** (0x407260): `e = SP − T`, `P = Kp·e`, `I += Ki·e` (I resets to 0 whenever |P| > 100), `D = Kd·Δe`. The output is clamped to 0–100 %.
* **Gains:** Kp ≈ 35, Ki ≈ 0.5, Kd ≈ 0.1. These come from `config.ini`, which is a raw 80-byte memory dump that also holds the 5 stage temperatures and hold times.
* **Setpoints are steps.** There are four stages, each a target temperature plus a hold time. Hold time is counted only once T ≥ target, and then the setpoint jumps to the next stage.

Why this overshoots:

1. With Kp = 35 %/°C, the heater runs at **100 % until the oven is within about 3 °C of the target**. With nothing limiting the ramp rate, the stored heat in the IR rods then carries the temperature past the setpoint.
2. The output can't go negative. The only braking is fan speed 50 % when T > SP + 5 °C.
3. Overshoot during the reflow stage stretches time-above-liquidus, because the hold timer is already running.
4. The temperature arrives as an integer, and the derivative is computed on error, so the D term does almost nothing.
5. Cool-down sets heater = 0 and fan speed 100 % until T ≤ stage-5 temperature, then turns the fan off.

### Fans (confirmed on the RF-A250)
**0x36 is the convection (circulation) fan enable; 0x35 is the cooling fan speed — two separate fans.** The vendor app sets enable = 1 but **speed = 0 for the whole heating phase**. The fan only spins at 50 % when the oven overshoots by more than 5 °C, and at 100 % during cool-down. The convection fan does run during heating; the cooling fan is purely reactive — it only spins up after the oven has already overshot, so it does little to prevent overshoot.

The new tools default the cooling fan to **100 % during the whole profile**; you may want to lower it while heating. Change it with `--fan-speed` on the command line or "Cooling fan % during run" in the GUI.

## Board-temperature calibration
The vendor app has no calibration command; the seven frames above are all it sends. If the oven firmware has a calibration or offset setting, it's on the front panel, not on the serial link. So calibration lives in this software instead. Both the GUI and `reflow.py run --cal` use a table `{"points": [[oven_C, board_C], ...]}` (see `calibration_example.json`).

How the table is applied:
* Between points the correction is linear.
* Above the top point, the last segment is extended.
* Below the lowest point, the correction fades to zero at 25 °C, because a cold oven and a cold board read the same.

With a calibration applied, **the profile is in board temperature**: the controller regulates the estimated board temperature. The over-temperature cut-off uses whichever is hotter, the raw reading or the estimate.

**Collecting points (GUI → Calibrate tab):**
1. Tape a thermocouple to a scrap board where your real boards sit, and read it on a meter.
2. Enter a temperature, then click **Hold**. The oven holds that *raw sensor* temperature, with the fan at the run setting.
3. Wait for **STEADY ✓**: at least 2 minutes in, and within ±0.5 °C for the last 60 s. Give the board another minute or two to soak, then type the meter reading and click **Add point**. The oven value saved is the 30-second average.
4. Repeat across your range, for example 100, 150, 180, 220 and 240 °C. Then **Save…**, and tick **Apply to profile runs**.

This is a *steady-state* correction. During fast ramps the board also lags behind the air, which no static table captures. Recording the board thermocouple live alongside the oven reading would be the next step.

## GUI (`gui.py`)
Double-click **`Start GUI.bat`**, or run `python gui.py`. Close the vendor app first, because only one program can hold the COM port.

* **Connection:** pick the COM port and click **Connect**. You can also tick **Simulator** to try it without the oven; the "×" box sets how fast simulated time runs. Once connected, the oven is in *monitor* mode: heater off, temperature read every 0.5 s.
* **Profile run:**
  * **Load profile…** takes a JSON file of `[seconds, °C]` points, like `sac305.json`.
  * Set the fan outputs and controller settings, then click **Start profile**. When the profile ends, the oven cools down (heater off, fan 100 %) until it reaches *Cool to °C*, then returns to monitor.
* **Calibrate tab:** load, build and save a calibration (see above).
* **Manual control:** sliders for heater % and cooling fan %, plus a convection-fan checkbox. Use it for step tests. The over-temperature cut-off still applies.
* **STOP / HEATER OFF:** stops at any time, sets all outputs to 0, and keeps monitoring.
* **CSV recording:** start or stop at any time, in any mode. There is one row per sample with these columns:
  `timestamp, t_s, mode, phase, setpoint_C, temp_oven_C (raw oven thermocouple), temp_board_est_C (calibrated, blank if none), predicted_C, heater_pct, fan_speed_pct, fan_enable`.

The chart shows oven temperature, setpoint, the profile (grey), heater % and cooling fan % (right-hand axis).
The grey profile overlay appears only while **Preview profile** is ticked *and* the Profile tab is open.
**Clear chart** wipes the chart and restarts the time axis (and the CSV `t_s` column) at 0 s. A running profile or hold carries on unaffected, and the CSV `timestamp` column stays absolute.
Closing the window, disconnecting, or losing the connection always turns the heater off.

## Tools in this folder
* `oven.py`: protocol driver (`link`, `read_temp`, `heater`, `set_output`, `raw`).
* `reflow.py`:
  * `temp`: poll the temperature.
  * `identify`: fan test.
  * `step`: open-loop step test for tuning.
  * `run`: profile runner.
  * `vendor`: replica of the vendor algorithm, for A/B comparison.
* `gui.py` / `Start GUI.bat`: the desktop GUI described above.
* `calibration.py`: oven→board calibration table.
* `sim.py`: a fake oven for testing without hardware. Its thermal constants are invented.

### The new controller (`run`)
* Follows a piecewise-linear profile `[[t_s, °C], ...]` (see `sac305.json`) instead of step setpoints. This means you choose the ramp rates.
* Feed-forward from the setpoint slope.
* PI acting on a *predicted* temperature `T + lead·dT/dt`, so it eases off the heater before the lag catches up.
* Anti-windup, a hard over-temperature cut-off (default 260 °C), heater forced off on any exit or error, and a CSV log.

The gains (`--kp 3 --ki 0.05 --kff 20 --lead 6`) are only starting points. Run `step` once, for example `--power 40 --seconds 240`, and send the CSV over. The heating rate and lag from that run give proper gains.

### Suggested first session (stay at the oven)
1. `python reflow.py temp --port COM3 --verbose` confirms the link and frames.
2. `python reflow.py step --port COM3 --power 40 --seconds 240 --max-temp 200 --csv step40.csv` (fan at 100 % by default, same as a run)
3. `python reflow.py run --port COM3 --csv run1.csv` with an empty drawer first.

## Caveats
* I have not yet tested anything on a real oven; only the simulator.
* I don't know what the oven does if the PC goes silent mid-run (it may or may not have its own watchdog). Don't leave it unattended.
* The sensor reads oven or air temperature, not the board. For true board temperature, tape a thermocouple to a scrap board and log it alongside.
* Other `A1 xx 02` read addresses might expose more data (a second sensor, for example). `oven.raw("A1 39 02", 7)` is the way to explore, but stick to `A1` reads. Unknown `A0` writes could change settings.

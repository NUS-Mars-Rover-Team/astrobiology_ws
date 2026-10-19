# Spectrometer calibration and validation — 2026-10-19

This is a record of the first end-to-end bring-up of the spectroscope (`spectro_cam`) on the Pi:
getting the data to a PC, calibrating the wavelengths, checking them against a dye and a laser,
tuning the camera for the halogen lamp, and the code fixes those tests led to. The code changes
are in commit `b3fcfb1` on `feat/spectro-calibration`.

For how the node works and every parameter, see
[`src/spectro_cam/README.md`](../src/spectro_cam/README.md).

## Status

| Item | Status |
|---|---|
| Pi → PC spectrum stream | Working, ~10 Hz over Wi-Fi |
| Wavelength calibration | Done; ±1.5 nm over 404–611 nm (6 CFL lines) |
| Dye check (Brilliant Blue) | Absorption band in the right place (623 nm with subtraction; see below) |
| 532 nm laser check | **Open:** read 520 nm, cause not yet identified |
| Camera settings for the 5 V halogen lamp | Done: 15 fps, exposure 660 |
| Absorbance mode, zero-reference and gamma fixes | Committed; **not deployed to the Pi yet** |

## 1. Getting the data from the Pi

The node runs on the Pi and publishes `/spectro/spectrum`. A PC on the same network sees it
directly if both machines have:

```bash
export ROS_DOMAIN_ID=67
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# ROS_LOCALHOST_ONLY unset or 0
```

On the PC, build the message package and the plotter, then plot:

```bash
colcon build --symlink-install --packages-select astrobiology_msgs spectro_cam
source install/setup.bash
ros2 run spectro_cam spectrum_plot            # x-axis in nm
ros2 run spectro_cam spectrum_plot --pixels   # x-axis in ROI columns, for calibration
```

Lessons from the first session:

- **Topics showing but no Pi data:** if `ros2 topic list` shows only `/parameter_events` and
  `/rosout`, the node isn't running or the Pi has different `ROS_DOMAIN_ID` / RMW settings.
  First check the Pi responds to `ping`.
- **CycloneDDS config files go stale:** a `CYCLONEDDS_URI` file pins a network interface name and
  peer IPs. If either is wrong (e.g. a USB-Ethernet interface while the PC is on Wi-Fi), discovery
  fails silently. Only use one when multicast is blocked, and keep it up to date.
- **SSH host key warning:** reflashing the Pi, or DHCP giving its address to another device,
  triggers `REMOTE HOST IDENTIFICATION HAS CHANGED`. Make sure you know why before clearing it
  with `ssh-keygen -R <ip>`.
- **Parameters can be changed from the PC.** `ros2 param set /spectro/spectrum_node ...` run on
  the PC changes the node on the Pi through ROS; no SSH needed. These changes live in memory only
  and are lost when the node restarts, unless they're also in the Pi's `spectro.yaml`.

## 2. Wavelength calibration (mercury CFL)

The node maps ROI column → wavelength with a two-point linear fit
(`processing.py`, `Calibration`). Points chosen: the two strongest unambiguous mercury lines of a
CFL.

1. Pointed the spectroscope at a CFL with the default (uncalibrated) parameters, exposure 50 at
   30 fps.
2. On the uncalibrated nm axis, the Hg 435.8 nm line showed up at "531 nm" and Hg 546.1 nm at
   "634 nm" (the taller half of the double peak; the shorter half is the Tb ~543 nm line).
   Converted back to ROI columns, those are **456** and **666**.
3. Applied the values live, then checked every other line the CFL shows:

| Line | True (nm) | Measured (nm) | Error |
|---|---|---|---|
| Hg violet | 404.7 | 404.3 | −0.4 |
| Hg blue (calibration point) | 435.8 | 435.8 | 0 |
| Tb cyan | ~487 | 487.8 | ~+0.8 |
| Hg green (calibration point) | 546.1 | 546.1 | 0 |
| Hg yellow doublet | 577.0 / 579.1 | 576.6 | unresolved pair |
| Eu red | 611.6 | 610.2 | −1.4 |

Result: `low_wavelength 435.8 / low_index 456`, `high_wavelength 546.1 / high_index 666`, which is
0.525 nm per pixel. The full 1920-px ROI spans about 200–1200 nm on paper; the light actually
reaching the camera covers about 400–1000 nm.

A repeat measurement later in the session gave the same values (435.8, 487.3, 546.0, 610.1 nm),
so the calibration doesn't drift between sessions.

Notes:
- **Error grows with distance from 435.8–546.1 nm:** the red end reads 1–2 nm low. A wider
  span (e.g. 435.8 and 611.6) would spread the error more evenly.
- **`roi_x` shifts the calibration:** `low_index`/`high_index` are ROI columns. If you move
  `roi_x`, subtract the change from both.
- **The preview is half size:** `/spectro/preview/compressed` is scaled to 960 px wide, so double
  preview pixel coordinates to get `roi_*` values.

## 3. Laser check (open)

A "532 nm" green laser pointer read **520 nm**. The CFL lines on either side (487.8 and 546.1)
are correct, so the calibration isn't the cause, and the CFL re-check rules out the spectroscope
having moved. Remaining explanations:

1. **The laser is actually a ~520 nm direct-diode green.** These are common and often labelled
   532 nm. A real 532 nm DPSS pointer usually leaks 808 nm or 1064 nm infrared, which this camera
   can see: check a laser-only spectrum for a peak near 808 nm.
2. **The beam entered at an angle:** a beam aimed straight at the slit can land on the sensor at
   a different position than diffuse light does.

**To settle it:** shine the laser onto white paper next to a running CFL so both reach the slit.
The laser's position relative to the 487/546 lines in the same frame decides between the two.
Diffuse light from real samples enters like the CFL's light does, so this question doesn't affect
sample measurements.

## 4. Dye check (Brilliant Blue FCF)

Brilliant Blue absorbs most strongly at about 628–630 nm. Measured with the halogen lamp, a water
blank as the zero reference, and `I − I0` subtraction (the only mode available on the Pi at the
time):

| | Measured | Expected |
|---|---|---|
| Deepest point of the dip (parabola fit) | 623.6 nm | ~628–630 nm |
| Half-depth band | 583–659 nm | broad, ~560–690 nm |

The ~6 nm offset comes from the method, not the calibration. Subtraction gives `I0·(T − 1)`, so
the dip is weighted by the lamp spectrum. The halogen/camera spectrum has a local maximum at
622 nm, which pulls the minimum towards it. Absorbance, `−log10(I/I0)`, removes the lamp
weighting; that's why the new `reference_mode: absorbance` was added. **Re-measure the dye once
the new code is on the Pi;** the peak should move towards 630 nm.

An earlier, undiluted run absorbed everything from ~570 to ~660 nm (a flat-bottomed gap). Dilute
until the dip has a rounded bottom, at about 20–50% of the surrounding light.

## 5. Light source and camera settings

The 5 V halogen lamp gives a smooth curve, but what the camera records has bumps (peaks ~621,
694, 776 nm; dips ~640, 740 nm). These come from the camera's response, not the lamp, and the
zero reference cancels them. There is very little signal below ~470 nm (a low-voltage halogen
gives little blue, and the sensor is weak there too). The useful range for absorption is
**~500–900 nm**. For blue-region work (carotenoids, chlorophyll at 430 nm), add a cool-white LED.

The camera has no infrared-blocking filter, so it records out to ~1000 nm.

Exposure for halogen + water cuvette, aiming for ~0.15–0.25 combined (saturation is 0.333):

| fps | `exposure_time_absolute` | Combined max |
|---|---|---|
| 30 | 50 (5 ms) | 0.030 |
| 30 | 330 (33 ms, longest that fits at 30 fps) | 0.076 |
| **15** | **660 (66 ms)** | **0.204** (brightest channel 0.238) |

The exposure can't be longer than one frame (`exposure_time_absolute ≤ 10000 / fps`), so a
longer exposure needs a lower frame rate. The CFL is much brighter: use exposure ~50 at 30 fps
for line sources, or the lines clip.

**If you see no light at all:** if the raw spectrum sits at ~0.001 everywhere and a 40× longer
exposure only shows a faint, featureless glow, no light is getting through the slit. During this
session the setup had been put together backwards. Turn on the preview
(`preview_rate_hz 1.0` + `rqt_image_view`) and look for the rainbow band before changing any
settings.

Other camera observations:
- **The R/G/B channels don't separate colour:** B reads highest even at the red Eu line. Treat
  them as meaningless; use only `combined`.
- **The camera output is gamma-encoded:** a 6.6× exposure increase gave only ~2.6× signal. With
  `linearize: "off"`, peak positions are right but heights aren't proportional to light. The
  `srgb` mode is now fixed, but it hasn't been checked against this camera. Before turning it on,
  double the exposure and confirm the reading doubles.

## 6. Zero-reference procedure

1. Light on, blank (water cuvette) in place, exposure set so the raw spectrum is ~0.2.
2. `ros2 service call /spectro/spectrum_node/clear_zero_reference std_srvs/srv/Trigger`
3. Wait ~2 s for the averaging buffer to refill with the blank.
4. `ros2 service call /spectro/spectrum_node/set_zero_reference std_srvs/srv/Trigger`.
   The plot should go flat at 0.
5. Swap in the sample without touching the lamp, exposure or cuvette position.

The reference is only valid for the light path it was taken with. A reference taken in the dark
just removes the camera's dark offset, and a sample measured against it shows nothing useful.
Clear the reference before going back to line sources (CFL/laser).

## 7. Code changes (commit `b3fcfb1`)

| Change | Files |
|---|---|
| Calibration and halogen camera settings | `config/spectro.yaml` |
| New `reference_mode` parameter: `subtract` (spectro-cam-rs) or `absorbance` (`−log10(I/I0)`, capped at 3, 0 where the reference is below 0.002) | `processing.py`, `spectrum_node.py`, `Spectrum.msg` (comment) |
| **Fix:** `set_zero_reference` snapshotted the already-referenced output, so a second call undid the first (full spectrum back) and a third zeroed everything | `processing.py` |
| **Fix:** `linearize` applied the gamma curve to channels normalised to 0..1/3 instead of 0..1, so `srgb` turned a saturated pixel into 0.09 | `processing.py` |
| Tests for all of the above (27 pass) | `test/test_processing.py` |

`Spectrum.msg` only changed in a comment, so it stays compatible with machines still on the old
build.

## 8. How accurate is it?

| | Value |
|---|---|
| Wavelength accuracy | ±1.5 nm, 404–611 nm |
| Optical resolution | ~3–5 nm (can't split the 2 nm Hg 577/579 pair) |
| Dispersion | 0.525 nm/px |
| Noise (combined, averaged) | ~0.0003 against a ~0.2 signal |

This is enough for broad absorption bands (30–100 nm wide): dyes, colorimetric assays (Bradford
~595 nm, ninhydrin ~570 nm), chlorophyll's red band (~662 nm). It is not enough for Raman,
separating atomic lines, or detecting few-nm fluorescence shifts.

## 9. Open items

- [ ] Deploy to the Pi: push the branch, then on the Pi `git checkout feat/spectro-calibration`,
      `colcon build --symlink-install --packages-select astrobiology_msgs spectro_cam`, and
      restart the launch. Until then, restarting the Pi node loses the calibration.
- [ ] Re-measure Brilliant Blue with `reference_mode: absorbance`; expect a peak at ~628–630 nm.
- [ ] Settle the laser question (CFL + laser on paper; look for an 808 nm leak).
- [ ] Check whether `linearize: srgb` matches this camera (double the exposure, confirm the
      signal doubles), then enable it for quantitative work.
- [ ] Add a blue light source if 400–480 nm measurements are needed.
- [ ] Optionally crop the ROI to the lit region (~400–1000 nm) to save CPU on the Pi; adjust the
      calibration indices by the `roi_x` change.
- [ ] Fill in the spectroscope's USB vendor/product IDs in the udev rule (still `XXXX` in the
      root README).

# LeArm Drawing Training Checklist

This checklist trains open-loop drawing without camera feedback or model
training. Use the GUI to discover safe poses, write them into a pose YAML file,
then replay staged trajectories with `trajectory_player`.

## Stage 0: Safety And Tool Setup

- Fix the arm base, paper, and pen holder.
- Use a soft tip or elastic pen mount so the arm can tolerate small height errors.
- Calibrate `home_pose`, `hover_center`, and `touch_center`.
- Pass when the arm can return to `home_pose` from every training pose.

## Stage 1: Joint Understanding

- Move only one of `joint_2` to `joint_6` at a time.
- Start with movements within `+/-10 deg`.
- Record which joint changes forward/back, left/right, height, and pen angle.

## Stage 2: Pen Tap

- Run `phase2_pen_tap.yaml` in `dry_run:=true` first.
- After calibration, run it with `dry_run:=false`.
- Pass when 10 taps touch the paper without visibly bending the arm.

## Stage 3: Short Lines

- Calibrate `left/right/front/back` hover and touch poses.
- Run `phase3_short_lines.yaml`.
- Pass when horizontal and vertical lines are continuous and endpoints are close.

## Stage 4: Basic Shapes

- Calibrate the four drawing-area corners.
- Run `phase4_basic_shapes.yaml`.
- Pass when the rectangle closes and repeats with similar shape three times.

## Stage 5: Speed And Smoothness

- Repeat the same line at 2000 ms, 1500 ms, 1000 ms, and 700 ms.
- Keep the slowest value that avoids broken lines, shaking, or heavy dragging.
- Record recommended durations for tap, line, and corner moves.

## Stage 6: Paper Coordinate Training

- Keep the drawing area within about 80 mm x 80 mm.
- Treat the calibrated corner poses as the first paper-coordinate mapping.
- Do not expand the area until rectangle repeatability is stable.

## Stage 7: Complex Paths

- Train wave, spiral, star, and simple letters.
- Start with `phase7_letters.yaml`, then add more YAML trajectories.
- Pass when pen-up and pen-down transitions happen only at stroke boundaries.

## Stage 8: Writing Preparation

- Start with simple single-stroke characters: `1`, `7`, `L`, `T`.
- Then use multi-stroke characters: `A`, `H`, `square`, `cross`.
- Save a separate action group or trajectory YAML after each stable character.

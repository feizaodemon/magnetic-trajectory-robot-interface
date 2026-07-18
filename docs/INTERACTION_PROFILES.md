# Interaction Profiles

Board and Mouse input share recognition, candidate presentation, and confirmation components while preserving different sampling semantics.

## Board profile

The Board profile treats the tracking-field convention `tracking_mz > 0.0` as an application-level interaction clutch:

- disengaged movement updates navigation without recording points or accumulating dwell;
- engaged movement in the drawing zone records trajectory points;
- engaged dwell over an enabled control activates that control;
- `B Recognize` freezes and submits the current sample;
- `C Confirm` confirms the reviewed candidate;
- `A Reject` returns to drawing;
- `X Clear` removes the current sample.

This convention suppresses transfer paths between the drawing area and controls. It is not general magnetic polarity detection, adaptive calibration, or a safety-rated mechanism.

## Mouse profile

- Press and drag in the drawing area to create a stroke.
- Stroke release completes the sample and triggers recognition automatically.
- Review the ranked candidates.
- Use `C Confirm` to publish the selected label when confirmation publication is enabled.
- Use `A Stop` or `X Clear` to reset the current interaction.

The Mouse profile does not use the Board clutch.

## Shared confirmation contract

Candidate display is informational. Confirmation produces `/colmag/confirmed_label`, not `/colmag/task_command`. Only the dispatcher can accept a confirmed label and create a task command. GUI-only profiles can disable confirmed-label publication entirely.

## Stale-data guards

The dashboard and confirmation helpers preserve sample/sequence identity so a candidate from an earlier sample cannot be confirmed as though it belonged to the current trajectory.

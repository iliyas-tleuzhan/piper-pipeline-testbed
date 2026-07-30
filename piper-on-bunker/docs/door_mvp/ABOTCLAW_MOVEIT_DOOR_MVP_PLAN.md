# ABot-Claw MoveIt Door MVP Plan

Status: plan only. No implementation has started in this branch checkpoint.

## Target Architecture

User instruction -> ABot-Claw mission agent -> deterministic door mission state machine -> MoveIt manipulation backend.

The MoveIt backend is temporary. The high-level mission API should remain stable so the manipulation backend can later be replaced by a VLA/OpenPI direct-joint backend after PiPER-X FK, hand-eye calibration and checkpoint validation are complete.

## Semantic Mission Actions

- `navigate_to_door`
- `observe_door`
- `approach_button`
- `press_button`
- `retract`
- `verify_door_open`
- `drive_through`

## Initial Constrained MVP

Start with the narrowest arm-only task:

- fixed Bunker staging position;
- fixed door and button;
- known arm home pose;
- manually measured or taught button target;
- MoveIt pre-contact pose;
- slow short Cartesian press;
- brief hold;
- retract;
- operator or camera confirmation;
- no arbitrary visual localization initially.

The first runnable sequence should be:

home -> pre-button -> press -> hold -> retract -> home

## Recommended Stages

1. Arm-only baseline:

   `home -> pre-button -> press -> hold -> retract -> home`

2. ABot-Claw restricted API:

   - `move_arm_home`
   - `move_to_button_prepose`
   - `press_button`
   - `retract_from_button`
   - `verify_door_open`

3. Add Bunker navigation:

   staging pose -> lock base -> manipulate -> verify -> stow -> drive through

4. Add perception correction:

   Use perception only to make bounded corrections around a known door/button setup.

5. Replace the MoveIt manipulation backend later:

   Preserve the high-level mission API and replace only the backend after the VLA path is trained and validated.

## Caveat: PiPER-X FK Issue

The same PiPER-X kinematic-model issue that invalidated D435i hand-eye validation may also affect MoveIt. Do not pretend the current normal-PiPER URDF is correct for PiPER-X.

Until the correct PiPER-X model is verified, the initial MoveIt MVP may need to use one of these constrained options:

- a fixed taught joint sequence;
- a fixed placement with manually verified joint targets;
- a MoveIt setup that is explicitly marked provisional and validated against real stopped-pose FK diagnostics before use.

Do not use the rejected hand-eye calibration as a basis for visual servoing, button localization, or VLA execution.

## Safety Boundary

Default to no physical execution. The first implementation pass should provide:

- planning-only checks;
- dry-run state-machine transitions;
- explicit command-authority checks;
- one bounded manual physical-test command only after model, frame and command-path assumptions are verified.

Do not start MoveIt execution, robot motion, OpenPI, VLA inference, or trajectory replay as part of this pause checkpoint.

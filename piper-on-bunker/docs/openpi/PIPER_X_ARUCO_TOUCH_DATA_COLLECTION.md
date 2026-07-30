# PiPER-X ArUco Touch Data Collection

This runbook prepares demonstrations for:

`Touch the center of the ArUco marker and retract.`

The first checkpoint uses one fixed-base AgileX PiPER-X arm, one wrist-mounted RGB camera, a vertical ArUco marker target, current 7D state, and 7D absolute joint-action labels. There is no external scene camera for this checkpoint. The OpenPI `observation/exterior_image` channel is a deterministic zero-image placeholder so training and inference use the same contract.

Physical execution remains blocked until a PiPER-X-specific checkpoint exists, PiPER-X limits are measured, gripper fixed-hold behavior is verified, offline validation passes, and hardware verification passes.

## Setup Requirements

- Mount the wrist camera rigidly on the active PiPER-X wrist.
- Place the ArUco marker on a vertical, rigid, flat surface. This matches the future door-button geometry.
- Use a safe pressing tip or compliant contact surface so contact demos do not damage the marker, wall, or arm.
- Keep the gripper in a fixed safe hold. Do not train gripper open/close behavior for this dataset.
- Complete local manifest copies from:
  - `piper-on-bunker/config/manifests/piper_x_hardware.template.yaml`
  - `piper-on-bunker/config/manifests/piper_x_wrist_camera.template.yaml`
  - `piper-on-bunker/config/manifests/piper_x_wrist_mount.template.yaml`
  - `piper-on-bunker/config/manifests/aruco_vertical_target.template.yaml`

Do not edit the template files with site-specific secrets or unverified guesses. Create `.local.yaml` copies.

## Read-Only Preflight

```bash
cd ~/piper-pipeline-testbed

./tools/run_in_noetic_container.sh \
  python3 piper-on-bunker/scripts/inspect_piper_x_aruco_collection_environment.py \
    --profile piper-on-bunker/config/openpi_piper_x_touch_aruco.yaml \
    --can-interface can0 \
    --action-source ros_command_topic \
    --command-topic /piper_x_joint_commands \
    --marker-id 6 \
    --marker-size-m 0.040 \
    --fixed-gripper-target 0.0
```

This does not enable the arm, publish commands, change CAN configuration, calibrate hardware, or move the robot.

The preflight requires a fresh wrist image, fresh state, and a fresh `sensor_msgs/JointState` command-label message. It also requires `can0` to be UP and ERROR-ACTIVE. The fixed gripper value above is only an example; use the measured safe hold value or let the recorder capture it from startup state when the state topic exposes a valid `gripper` field.

## Action-Topic Verification

Before keeping any demonstrations, verify that the action-label topic is the exact converted absolute target stream that the follower/slave receives.

Check the message type:

```bash
cd ~/piper-pipeline-testbed

./tools/run_in_noetic_container.sh \
  bash -lc 'source /opt/ros/noetic/setup.bash && rostopic type /piper_x_joint_commands'
```

It must be:

```text
sensor_msgs/JointState
```

Check the publisher and rate:

```bash
cd ~/piper-pipeline-testbed

./tools/run_in_noetic_container.sh \
  bash -lc 'source /opt/ros/noetic/setup.bash && rostopic info /piper_x_joint_commands && rostopic hz /piper_x_joint_commands'
```

Check the joint names and one sample:

```bash
cd ~/piper-pipeline-testbed

./tools/run_in_noetic_container.sh \
  bash -lc 'source /opt/ros/noetic/setup.bash && rostopic echo -n 1 /piper_x_joint_commands'
```

The message must contain exactly these six arm targets:

```text
joint1 joint2 joint3 joint4 joint5 joint6
```

The recorder appends the fixed gripper target as channel 7.

### One-Joint-At-A-Time Label Check

With the teleoperation/follower system running in its normal demonstration mode, move only one leader joint at a time by a small amount while watching `/piper_x_joint_commands`.

For each joint:

1. Move only `jointN` slowly.
2. Confirm only the matching `jointN` command value changes meaningfully.
3. Confirm the sign is correct.
4. Confirm the value is in radians and absolute target units.
5. Return to the start pose before checking the next joint.

Physical follower motion does not prove the recorded action labels are correct. A follower can move while the recorder is subscribed to the wrong topic, a pre-clamped topic, a feedback topic, stale CAN frames, or a stream with swapped/sign-flipped joints. Do not keep demos until this one-joint-at-a-time check passes.

## Five Throwaway Pilot Recordings

Use these only after the preflight shows fresh wrist image, state, and action-label source. Replace the marker size with the measured outside black-square side length.

```bash
cd ~/piper-pipeline-testbed

./tools/run_in_noetic_container.sh \
  python3 piper-on-bunker/scripts/record_openpi_piper_x_aruco_episode.py \
    --instruction "Touch the center of the ArUco marker and retract." \
    --profile piper-on-bunker/config/openpi_piper_x_touch_aruco.yaml \
    --action-source ros_command_topic \
    --command-topic /piper_x_joint_commands \
    --wrist-image-topic /piper_x/wrist_camera/image_raw \
    --state-topic /joint_states_single \
    --max-image-age-s 0.5 \
    --max-state-age-s 0.5 \
    --max-action-age-s 0.5 \
    --marker-id 6 \
    --marker-size-m 0.040 \
    --operator-notes "throwaway pilot 1"
```

Repeat for pilots 2-5 with different notes. The recorder is passive and never commands motion. It samples synchronized image/state/action rows at 20 Hz by default. It refuses to create an episode until wrist image, state, and action labels are present and fresh.

## Outcome Labeling

```bash
cd ~/piper-pipeline-testbed

./tools/run_in_noetic_container.sh \
  python3 piper-on-bunker/scripts/label_openpi_episode_outcome.py \
    piper-on-bunker/data/local/openpi_piper_x_aruco_episodes/EPISODE_ID \
    --status success \
    --contact-confirmed \
    --marker-touched \
    --retraction-completed \
    --operator-notes "operator confirmed contact and retract"
```

ArUco disappearance does not prove contact. For now, contact is operator-confirmed.

## Inspection And Splits

```bash
cd ~/piper-pipeline-testbed

./tools/run_in_noetic_container.sh \
  python3 piper-on-bunker/scripts/inspect_openpi_piper_x_aruco_episode.py \
    piper-on-bunker/data/local/openpi_piper_x_aruco_episodes \
    --profile piper-on-bunker/config/openpi_piper_x_touch_aruco.yaml \
    --json-report piper-on-bunker/data/local/openpi_piper_x_aruco_inspection.json \
    --write-splits piper-on-bunker/data/local/openpi_piper_x_aruco_splits.json \
    --split-seed 7
```

Splits are by complete episode, never by individual frame.

## Conversion Scaffold

```bash
cd ~/piper-pipeline-testbed

./tools/run_in_noetic_container.sh \
  python3 piper-on-bunker/scripts/convert_openpi_piper_x_aruco_to_lerobot.py \
    --episodes-root piper-on-bunker/data/local/openpi_piper_x_aruco_episodes \
    --output-dir piper-on-bunker/data/local/openpi_piper_x_aruco_lerobot \
    --profile piper-on-bunker/config/openpi_piper_x_touch_aruco.yaml
```

The output preserves wrist image, zero exterior image, 7D state, 7D absolute action, prompt, episode boundaries, timestamps, and profile metadata.

## Future OpenPI Commands

Do not run these until enough real demonstrations exist.

Normalization statistics must be PiPER-X-specific:

```bash
cd /path/to/official/openpi
uv run scripts/compute_norm_stats.py --config-name pi05_piper_x_touch_aruco
```

Future fine-tuning from `pi05_base`:

```bash
cd /path/to/official/openpi
XLA_PYTHON_CLIENT_MEM_FRACTION=0.9 \
uv run scripts/train.py pi05_piper_x_touch_aruco --exp-name=piper_x_aruco_touch_v1 --overwrite
```

Future dataloader smoke test should use the same PiPER-X transform in:

`piper-on-bunker/openpi_extension/piper_x_aruco_policy.py`

## Live Shadow

Shadow captures live wrist image and state, builds the PiPER-X observation, and returns a non-executable smoke response:

```bash
cd ~/piper-pipeline-testbed

./tools/run_in_noetic_container.sh \
  python3 piper-on-bunker/scripts/run_openpi_piper_x_aruco_live_shadow.py \
    --instruction "Touch the center of the ArUco marker and retract." \
    --profile piper-on-bunker/config/openpi_piper_x_touch_aruco.yaml
```

The result must report:

- `execution_allowed: false`
- `physical_motion_performed: false`

## Why Physical Execution Is Blocked

- PiPER-X joint limits are unresolved.
- PiPER-X gripper/tool units and fixed hold value are unresolved.
- No PiPER-X dataset normalization stats exist yet.
- No PiPER-X checkpoint has been trained or validated.
- Public checkpoints and normal PiPER smoke metadata cannot authorize PiPER-X motion.

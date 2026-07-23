# Deployment To PiPER Laptop

Run later on `HKU-CPS`; do not run these hardware steps on this development laptop.

1. Clone:
   `git clone https://github.com/iliyas-tleuzhan/piper-pipeline-testbed.git ~/piper-pipeline-testbed`
2. Start existing ABot-Claw/PiPER stack:
   `cd ~/ABot-Claw/robot_layer/arm_piper/agent_server && ./start_piper_language_stack_tmux.sh`
3. Verify CAN:
   `ip -details link show can0`
4. Verify ROS:
   `rostopic list`
5. Verify PiPER state:
   `rostopic echo -n 1 /joint_states_single` and `rostopic echo -n 1 /end_pose`
6. Verify MoveIt:
   `rosservice list | grep joint_moveit_ctrl`
7. Verify RealSense:
   `./check_realsense_topics.sh`
8. Verify action server:
   `curl --noproxy '*' http://localhost:8891/health`
9. Run read-only:
   `cd ~/piper-pipeline-testbed && make piper-read-only`
10. Record fixture:
   `python3 piper-on-bunker/scripts/record_hardware_fixture.py --output piper-on-bunker/fixtures/hardware/session.json`
11. Run dry-run:
   `make dry-demo`
12. Calibrate safe named poses:
   `make calibrate`
13. Enable physical motion only after review:
   edit `piper-on-bunker/config/piper_laptop_hardware.yaml`
14. Test navigation-view poses:
   use `make dry-demo`, then the calibrated hardware command.
15. Test inspection poses.
16. Test target detection.
17. Test mock-button touch.
18. Test complete tabletop mission:
   `make hardware-demo`
19. Invoke through restricted agent:
   `make agent-api`, then POST `/command` with `run_button_mission`.
20. Stop safely:
   `make stop`; if needed, use existing ABot-Claw/tmux shutdown procedures.

Do not copy files over SSH automatically. Clone this repository on the PiPER laptop.

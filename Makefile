.PHONY: install audit test mock-demo replay-demo agent-api package-deployment piper-read-only camera-check dry-demo calibrate hardware-demo stop

PY ?= python

install:
ifeq ($(OS),Windows_NT)
	$(PY) -X utf8 -m pip install ".[test]"
else
	$(PY) -X utf8 -m pip install ".[test]"
endif

audit:
	$(PY) piper-on-bunker/scripts/audit_existing_repo.py
	$(PY) piper-on-bunker/scripts/inspect_development_machine.py

test:
	$(PY) -m compileall piper-on-bunker/src piper-on-bunker/scripts
	$(PY) -m pytest piper-on-bunker/tests
	$(PY) -m piper_on_bunker.cli --help

mock-demo:
	$(PY) piper-on-bunker/scripts/run_mock_demo.py

replay-demo:
	$(PY) piper-on-bunker/scripts/run_replay_demo.py

agent-api:
	$(PY) piper-on-bunker/scripts/run_agent_api.py

package-deployment:
	$(PY) piper-on-bunker/scripts/package_deployment.py

piper-read-only:
	$(PY) piper-on-bunker/scripts/run_read_only_check.py --config piper-on-bunker/config/piper_laptop_dry_run.yaml

camera-check:
	$(PY) piper-on-bunker/scripts/inspect_piper_machine.py --camera-only

dry-demo:
	$(PY) piper-on-bunker/scripts/run_dry_demo.py

calibrate:
	$(PY) piper-on-bunker/scripts/calibrate_named_pose.py

hardware-demo:
	$(PY) piper-on-bunker/scripts/run_tabletop_demo.py --require-physical-enable

stop:
	$(PY) piper-on-bunker/scripts/emergency_stop.py

#!/usr/bin/env bash
set -euo pipefail

CONTAINER=${CONTAINER:-abot-piper-noetic}
PYAGXARM_REPO=${PYAGXARM_REPO:-https://github.com/agilexrobotics/pyAgxArm.git}
PYAGXARM_COMMIT=${PYAGXARM_COMMIT:-cc498c00af0bcb9e297943e94f4792c0e3ee5b2c}
HOST_CACHE=${HOST_CACHE:-piper-on-bunker/data/local/third_party/pyAgxArm-$PYAGXARM_COMMIT}
CONTAINER_SRC=${CONTAINER_SRC:-/opt/piper_x_deps/pyAgxArm-$PYAGXARM_COMMIT}

cd "$(dirname "$0")/.."

if ! docker ps --format '{{.Names}}' | grep -qx "$CONTAINER"; then
  echo "Container $CONTAINER is not running." >&2
  exit 2
fi

if [[ ! -d "$HOST_CACHE/.git" ]]; then
  mkdir -p "$(dirname "$HOST_CACHE")"
  git clone "$PYAGXARM_REPO" "$HOST_CACHE"
fi

git -C "$HOST_CACHE" fetch --quiet origin "$PYAGXARM_COMMIT"
git -C "$HOST_CACHE" cat-file -e "$PYAGXARM_COMMIT^{commit}"
git -C "$HOST_CACHE" checkout --quiet "$PYAGXARM_COMMIT"
actual_commit=$(git -C "$HOST_CACHE" rev-parse HEAD)
if [[ "$actual_commit" != "$PYAGXARM_COMMIT" ]]; then
  echo "pyAgxArm checkout mismatch: expected $PYAGXARM_COMMIT got $actual_commit" >&2
  exit 2
fi

docker exec -i "$CONTAINER" bash -lc "mkdir -p '$CONTAINER_SRC'"
tar -C "$HOST_CACHE" -cf - . | docker exec -i "$CONTAINER" bash -lc "tar -C '$CONTAINER_SRC' -xf -"

docker exec -i "$CONTAINER" bash -lc "
set -euo pipefail
python3 -m pip install --no-index --no-deps --no-build-isolation --force-reinstall '$CONTAINER_SRC'
python3 - <<'PY'
import inspect
import pyAgxArm
from pyAgxArm import AgxArmFactory, ArmModel, PiperFW, create_agx_arm_config

cfg = create_agx_arm_config(
    robot=ArmModel.PIPER_X,
    comm='can',
    firmeware_version=PiperFW.V189,
    interface='socketcan',
    channel='can0',
    bitrate=1000000,
    auto_connect=False,
)
print('pyagxarm_import: ok')
print('pyagxarm_module:', pyAgxArm.__file__)
print('pyagxarm_version:', getattr(pyAgxArm, '__version__', 'unknown'))
print('pyagxarm_commit:', '$PYAGXARM_COMMIT')
print('arm_model:', ArmModel.PIPER_X)
print('firmware_profile:', PiperFW.V189)
print('create_agx_arm_config_signature:', inspect.signature(create_agx_arm_config))
print('driver_class:', AgxArmFactory.load_class(cfg))
PY
"

echo "installed_pyagxarm_source: $PYAGXARM_REPO"
echo "installed_pyagxarm_commit: $PYAGXARM_COMMIT"
echo "host_cache: $HOST_CACHE"
echo "container_source: $CONTAINER_SRC"

#!/usr/bin/env bash
set -euo pipefail

CONTAINER=${CONTAINER:-abot-piper-noetic}
DECODER_REPO=${DECODER_REPO:-/home/dase-hw101/Iliyas/piper-lora-teleop-bridge}
EXPECTED_COMMIT=${EXPECTED_COMMIT:-521c9c5fdfd9ee63bd96c0f9342fca6b2398092e}

cd "$(dirname "$0")/.."

if [[ ! -d "$DECODER_REPO/.git" ]]; then
  echo "decoder_repo: not_found ($DECODER_REPO)" >&2
  exit 2
fi

actual_commit=$(git -C "$DECODER_REPO" rev-parse HEAD)
if [[ "$actual_commit" != "$EXPECTED_COMMIT" ]]; then
  echo "decoder_repo_commit: mismatch expected=$EXPECTED_COMMIT actual=$actual_commit" >&2
  exit 2
fi

if ! docker ps --format '{{.Names}}' | grep -qx "$CONTAINER"; then
  echo "container: not_running ($CONTAINER)" >&2
  exit 2
fi

docker exec -i "$CONTAINER" bash -lc '
python3 - <<PY
try:
    import can
except Exception as exc:
    raise SystemExit(f"python_can_import: not_ready ({exc!r})")
print("python_can_import: ok")
print("python_can_module:", getattr(can, "__file__", "unknown"))
PY
'

echo "feedback_adapter: passive_socketcan"
echo "decoder_repo: $DECODER_REPO"
echo "decoder_commit: $actual_commit"
echo "network_install_performed: false"
echo "third_party_source_copied: false"

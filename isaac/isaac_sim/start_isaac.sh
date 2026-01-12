#!/usr/bin/env bash
set -euo pipefail

CONTAINER_NAME="${CONTAINER_NAME:-isaac-sim}"
PY_BASE="/isaac-sim/duatic_ws/src/duatic_simulation/isaac"

usage() {
  cat <<USAGE
Usage:
  $0 gui
    Start Isaac Sim GUI (isaac-sim.sh). Ctrl-C stops Kit.

  $0 py <relative_script_path> [-- script args...]
    Run Isaac Sim python.sh with a script located under:
      ${PY_BASE}
    The script path must be RELATIVE to that base folder.

Examples:
  $0 gui

  $0 py scripts/run_scene.py

  $0 py scripts/run_scene.py -- --headless --foo bar
USAGE
}

MODE="${1:-}"
shift || true

if [[ -z "${MODE}" ]]; then
  usage
  exit 2
fi

python3 - "$MODE" "$PY_BASE" "$@" <<'PY'
import sys, time, re
import docker
from pathlib import PurePosixPath

CONTAINER_NAME = "isaac-sim"

mode = sys.argv[1]
py_base = sys.argv[2]
args = sys.argv[3:]

client = docker.DockerClient(base_url="unix://var/run/docker.sock")
c = client.containers.get(CONTAINER_NAME)

def exec_root(argv):
    res = c.exec_run(argv, user="root", stdout=True, stderr=True)
    out = res.output.decode(errors="ignore") if res.output else ""
    return res.exit_code, out

def find_kit_pids():
    _, out = exec_root(["bash", "-lc", "pgrep -af '/isaac-sim/kit/kit' || true"])
    pids = []
    for ln in out.splitlines():
        ln = ln.strip()
        if not ln:
            continue
        m = re.match(r"^(\d+)\s+(.+)$", ln)
        if not m:
            continue
        pid = int(m.group(1))
        cmd = m.group(2)
        if "/isaac-sim/apps/" in cmd and cmd.endswith(".kit"):
            pids.append(pid)
        elif "isaacsim" in cmd and cmd.endswith(".kit"):
            pids.append(pid)
    return sorted(set(pids)), out

def kill_pids(pids, sig):
    for pid in pids:
        exec_root(["bash", "-lc", f"kill -{sig} {pid} || true"])

def stop_kit():
    pids, raw = find_kit_pids()
    if not pids:
        print("No Kit PID found. Current kit matches:\n" + raw)
        return
    print("Found Kit PID(s):", pids)
    for sig, wait in [("INT", 3), ("TERM", 3), ("KILL", 1)]:
        kill_pids(pids, sig)
        time.sleep(wait)
        still, _ = find_kit_pids()
        if not still:
            print("Isaac/Kit stopped.")
            return
        pids = still
        print(f"Still running after SIG{sig}. Remaining PID(s): {pids}")
    still, raw = find_kit_pids()
    print("WARNING: Kit still appears to be running:\n" + raw)

def sh_escape(s: str) -> str:
    return "'" + s.replace("'", "'\"'\"'") + "'"

# Build command
if mode == "gui":
    cmd = ["bash", "-lc", "cd /isaac-sim && ./isaac-sim.sh"]
    print("Mode: GUI (isaac-sim.sh)")
elif mode == "py":
    if not args:
        print("ERROR: py mode requires <relative_script_path> [-- args...]")
        sys.exit(2)

    # Split script args after literal "--"
    if "--" in args:
        idx = args.index("--")
        rel_script = args[0]
        script_args = args[idx+1:]
    else:
        rel_script = args[0]
        script_args = args[1:]

    # Ensure it's a relative posix path (no absolute paths)
    rel_path = PurePosixPath(rel_script)
    if rel_path.is_absolute():
        print("ERROR: Provide a RELATIVE script path (starting from the base folder).")
        print("Base folder:", py_base)
        sys.exit(2)

    full_script = str(PurePosixPath(py_base) / rel_path)

    joined_args = " ".join(sh_escape(a) for a in script_args)
    cmd_str = f"cd /isaac-sim && ./python.sh {sh_escape(full_script)} {joined_args}".strip()
    cmd = ["bash", "-lc", cmd_str]

    print("Mode: Python (python.sh)")
    print("Base:", py_base)
    print("Script (relative):", rel_script)
    print("Script (full):", full_script)
    if script_args:
        print("Args:", script_args)
else:
    print("ERROR: unknown mode:", mode)
    sys.exit(2)

print("Press Ctrl-C to stop Isaac/Kit.")
print("--------------------------------------------------------")

# Run and stream logs
exec_id = client.api.exec_create(c.id, cmd, stdout=True, stderr=True, stdin=False, tty=False)
stream = client.api.exec_start(exec_id, stream=True)

try:
    for chunk in stream:
        sys.stdout.write(chunk.decode(errors="ignore"))
        sys.stdout.flush()
except KeyboardInterrupt:
    print("\nCtrl-C received. Stopping Isaac/Kit...")
    stop_kit()
PY

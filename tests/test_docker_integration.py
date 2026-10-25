"""Docker command contracts without requiring an engine or ROS installation."""

from __future__ import annotations

import json
import os
import shutil
import subprocess
import sys
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture
def command_env(tmp_path: Path) -> dict[str, str]:
    if os.name == "nt" or shutil.which("bash") is None:
        pytest.skip("The container scripts require a POSIX shell")
    binary_dir = tmp_path / "bin"
    binary_dir.mkdir()
    # Record CLI boundaries and configurable exit statuses, not ROS internals.
    stub = f"#!{sys.executable}\n" + '''
import json
import os
import sys
from pathlib import Path

command = Path(sys.argv[0]).name
with open(os.environ["COMMAND_LOG"], "a", encoding="utf-8") as handle:
    handle.write(json.dumps({
        "command": command, "args": sys.argv[1:],
        "image": os.environ.get("ROS_IMAGE_JAZZY"),
        "duration": os.environ.get("EXPERIMENT_DURATION_SECONDS"),
    }) + "\\n")
status = 0
if command == "ros2":
    if sys.argv[1] == "run":
        status = int(os.environ.get("RUN_STATUS", "0"))
    elif sys.argv[-1] == os.environ.get("MISSING_PACKAGE"):
        status = 1
elif command == "python" and sys.argv[1].endswith("generate_ros_validation_report.py"):
    status = int(os.environ.get("REPORT_STATUS", "0"))
elif command == "colcon" and sys.argv[1] == "test-result":
    status = int(os.environ.get("COLCON_STATUS", "0"))
raise SystemExit(status)
'''
    for name in ("ros2", "python", "colcon", "docker"):
        path = binary_dir / name
        path.write_text(stub, encoding="utf-8")
        path.chmod(0o755)
    env = os.environ.copy()
    env.update(PATH=f"{binary_dir}{os.pathsep}{env['PATH']}", COMMAND_LOG=str(tmp_path / "commands.jsonl"))
    for key in (
        "BUILDX_BUILDER", "ROS_IMAGE_JAZZY", "EXPERIMENT_DURATION_SECONDS",
        "RUN_STATUS", "REPORT_STATUS", "COLCON_STATUS", "MISSING_PACKAGE",
    ):
        env.pop(key, None)
    return env


def _commands(env: dict[str, str]) -> list[dict]:
    return [json.loads(line) for line in Path(env["COMMAND_LOG"]).read_text().splitlines()]


@pytest.mark.parametrize("run_status,report_status,expected", [(0, 0, 0), (3, 0, 3), (0, 2, 2), (3, 2, 3)])
def test_experiment_always_reports_and_preserves_failure(
    command_env: dict[str, str], run_status: int, report_status: int, expected: int,
) -> None:
    command_env.update(RUN_STATUS=str(run_status), REPORT_STATUS=str(report_status))
    workspace = Path(command_env["COMMAND_LOG"]).parent
    result = subprocess.run(
        ["bash", str(ROOT / "scripts/docker_headless_experiment.sh")],
        env=command_env, cwd=workspace, check=False,
    )
    assert result.returncode == expected
    runner, report = _commands(command_env)
    run_dir = runner["args"][-1]
    assert run_dir.startswith("Output/Gate1/run_")
    assert (workspace / run_dir).is_dir()
    assert runner["args"] == [
        "run", "hybrid_nav", "gate1_experiment", "--duration", "30",
        "--output-dir", run_dir,
    ]
    assert report["args"] == [
        "/opt/gate1/generate_ros_validation_report.py",
        "--input", f"{run_dir}/summary.json", "--output", f"{run_dir}/plots",
    ]


def test_repeated_experiments_keep_independent_evidence(command_env: dict[str, str]) -> None:
    workspace = Path(command_env["COMMAND_LOG"]).parent
    script = ["bash", str(ROOT / "scripts/docker_headless_experiment.sh")]
    subprocess.run(script, env=command_env, cwd=workspace, check=True)
    first_run = _commands(command_env)[0]["args"][-1]
    evidence = workspace / first_run / "controller_raw.log"
    evidence.write_text("first run evidence", encoding="utf-8")

    subprocess.run(script, env=command_env, cwd=workspace, check=True)
    first_runner, first_report, second_runner, second_report = _commands(command_env)
    second_run = second_runner["args"][-1]
    assert first_runner["args"][-1] != second_run
    assert first_report["args"][-1] == f"{first_run}/plots"
    assert second_report["args"][-1] == f"{second_run}/plots"
    assert evidence.read_text(encoding="utf-8") == "first run evidence"


@pytest.mark.parametrize("missing_package,colcon_status", [("", 0), ("hybrid_nav", 0), ("hybrid_controller", 0), ("", 2)])
def test_workspace_checks_require_both_packages_and_passing_results(
    command_env: dict[str, str], missing_package: str, colcon_status: int,
) -> None:
    command_env.update(MISSING_PACKAGE=missing_package, COLCON_STATUS=str(colcon_status))
    result = subprocess.run(
        ["bash", str(ROOT / "scripts/docker_ros2_checks.sh")],
        env=command_env, cwd=ROOT, check=False,
    )
    assert (result.returncode == 0) == (not missing_package and colcon_status == 0)
    commands = _commands(command_env)
    assert any(item["args"] == ["test", "--event-handlers", "console_direct+"] for item in commands)
    assert any(item["args"] == ["test-result", "--verbose"] for item in commands)
    if result.returncode == 0:
        assert [item["args"] for item in commands if item["command"] == "ros2"] == [
            ["pkg", "prefix", "hybrid_controller"], ["pkg", "prefix", "hybrid_nav"],
        ]


@pytest.mark.parametrize("builder", ["", "cloud-example-default"])
def test_make_build_loads_the_same_image_run_by_compose(
    command_env: dict[str, str], builder: str,
) -> None:
    if shutil.which("make") is None:
        pytest.skip("make is not available")
    command_env.update(BUILDX_BUILDER=builder, ROS_IMAGE_JAZZY="example:gate1", EXPERIMENT_DURATION_SECONDS="12")
    subprocess.run(
        ["make", "--no-print-directory", "docker-test", "docker-experiment"],
        env=command_env, cwd=ROOT, check=True, capture_output=True, text=True,
    )
    build, checks, experiment = _commands(command_env)
    assert build["args"][:2] == ["buildx", "build"]
    assert "--load" in build["args"]
    assert build["args"][build["args"].index("--tag") + 1] == "example:gate1"
    assert "ROS_DISTRO=jazzy" in build["args"]
    if builder:
        assert build["args"][build["args"].index("--builder") + 1] == builder
    else:
        assert "--builder" not in build["args"]
    for command, service in [(checks, "ci"), (experiment, "experiment")]:
        assert command["image"] == "example:gate1"
        assert command["args"] == [
            "compose", "--profile", service, "run", "--rm", "--no-deps",
            "--pull", "never", "-T", service,
        ]
    assert experiment["duration"] == "12"


def test_ci_workflow_uses_run_flags_supported_by_compose(command_env: dict[str, str]) -> None:
    del command_env
    workflow = (ROOT / ".github" / "workflows" / "ci.yml").read_text(encoding="utf-8")
    assert "docker compose --profile ci run --rm --no-deps --pull never -T ci" in workflow
    assert "docker compose --profile experiment run --rm --no-deps --pull never -T experiment" in workflow
    assert "docker compose --profile ci run --rm --no-deps --no-build" not in workflow
    assert "docker compose --profile experiment run --rm --no-deps --no-build" not in workflow
    assert "mkdir -p Output/Gate1" in workflow
    assert "Restore Gate 1 artifact ownership" in workflow


def test_compose_uses_non_deprecated_ros_discovery_environment() -> None:
    compose = (ROOT / "docker-compose.yml").read_text(encoding="utf-8")

    assert "ROS_AUTOMATIC_DISCOVERY_RANGE: LOCALHOST" in compose
    assert 'ROS_STATIC_PEERS: ""' in compose
    assert "ROS_LOCALHOST_ONLY" not in compose


def test_compose_config_when_docker_is_available() -> None:
    docker = shutil.which("docker")
    if docker is None:
        pytest.skip("Docker CLI is not available")
    assert docker is not None
    subprocess.run([docker, "compose", "--profile", "*", "config", "--quiet"], cwd=ROOT, check=True)

# Gate 1 Docker builds and CI

This is provisional integration tooling. Research Gate 0 remains incomplete:
the analytical grids do not replace controlled plant validation and actuator
calibration. No successful Docker/ROS 2 experiment or human sign-off has been
recorded for this implementation as of October 23, 2026.

## Local build and validation

Run from the repository root with Docker Engine (or Docker Desktop), Buildx,
Compose v2 or newer, and GNU Make installed:

```bash
make docker-test
make docker-experiment
```

Both targets build the ROS 2 Jazzy image and load it into the local Docker
Engine before Compose runs it. `docker-test` runs `colcon test`, checks the
test results, and requires both `hybrid_controller` and `hybrid_nav` to be
discoverable. `docker-experiment` runs the installed Gate 1 harness for 30
simulated seconds after its ROS streams are ready, with fixed headless settings
and seeds, then generates the
PNG/PDF plots and Markdown report. A failed runner or failed report makes the
command return nonzero. Independent startup and wall-clock deadlines reject
stalled runs. Fixed inputs do not imply identical samples or solver times
across machines.

```bash
make docker-experiment EXPERIMENT_DURATION_SECONDS=60
```

Each invocation creates a new `Output/Gate1/run_<id>/` directory and prints its
path. Previous runs are retained. Artifacts persist on the host:

- `Output/Gate1/run_<id>/`: launch log, raw controller log, telemetry CSV, summary JSON/CSV.
- `Output/Gate1/run_<id>/plots/`: validation figures and `gate1_validation_report.md`.
- `Output/Gate1/run_<id>/ros_logs/`: ROS launch diagnostics.

CI creates `Output/Gate1` before starting Compose and restores ownership after
the experiment, including failed runs. This keeps the bind-mounted evidence
readable by `actions/upload-artifact` when the container runs as root.

The Compose `experiment` service needs no display or host-network access.
For the existing interactive simulation use `docker compose up jazzy`.
Enable RViz/Gazebo clients explicitly on a display-configured host by passing
`use_rviz:=true use_gz_gui:=true` to a direct launch command.

To reuse an already built image, without rebuilding:

```bash
docker compose --profile ci run --rm --no-deps --pull never -T ci
docker compose --profile experiment run --rm --no-deps --pull never -T experiment
```

## Optional remote builder

Local builds require no Docker Build Cloud account or credentials. A connected
remote builder can be selected with the standard Buildx environment variable:

```bash
BUILDX_BUILDER=cloud-example-default make docker-build
BUILDX_BUILDER=cloud-example-default make docker-test
```

Use the builder name shown by `docker buildx ls`. The Make targets never log
in, create a cloud builder, push an image, or change the selected builder.
Unset `BUILDX_BUILDER` to use the existing default builder, or explicitly set
`BUILDX_BUILDER=default` to choose the local Docker builder.

If a cloud builder has not yet been connected, follow Docker's Build Cloud
setup and sign in on the machine that performs the build. The standard cloud
driver requires Buildx 0.37.0 or newer; check `docker buildx version` separately
from `docker --version`. Compose also accepts `BUILDX_BUILDER` or its
`build --builder` option. Remote building still requires a local Docker Engine
for `--load`, workspace checks, and the experiment.

## GitHub Actions

The ROS 2 job uses Docker's official login, setup-buildx and build-push
actions. The current repository settings are:

| Kind | Name | Meaning |
| --- | --- | --- |
| Secret | `DOCKER_USER` | Docker Hub username and Build Cloud builder owner |
| Secret | `DOCKER_PAT` | Docker PAT authorized for the builder |

When both values are available, CI connects to
`${DOCKER_USER}/ros2-jazzy` with `driver: cloud`. Otherwise, including fork
pull requests where the token is unavailable, CI uses a local
`docker-container` Buildx builder without logging in. A configured cloud
connection that fails is reported as a failure rather than silently retried.

The single-platform `linux/amd64` image is loaded into the runner's Docker
Engine and tested through the same Compose services. It is not published to a
registry. CI retains Gate 1 evidence with `always()` so failed validation runs
remain inspectable in the `gate1-ros2-jazzy` artifact.

Official references:

- [Docker Build Cloud in CI](https://docs.docker.com/build-cloud/ci/)
- [Selecting builders and loading results](https://docs.docker.com/build-cloud/usage/)

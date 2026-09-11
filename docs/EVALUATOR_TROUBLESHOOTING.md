# Evaluator troubleshooting

The submitted Docker image contains the complete Task 3 policy. It stages that
policy into a disposable directory on both deployed ROS computers. It does not
require a pre-existing `/home/aup/tmr-mobile-manipulation` checkout or a private
`/home/aup/tmr_env.sh` file.

## Safe validation order

1. Run `preflight`. This checks only the image payload and Python/shell syntax.
2. Start the standard vendor drivers and camera services.
3. Run `check`. This performs read-only SSH, ROS graph, fresh camera, and fresh
   odometry checks and reports `physical_motion_commanded: false`.
4. Run `execute` only after `check` succeeds and the designated initial state
   has been restored.

## SSH

Do not upload or commit `.ssh`, private keys, passwords, or agent data. Supply
an evaluator-owned key or agent that is authorized for the two deployed hosts.
The defaults are `aup@172.16.0.100` and `tmr-user@172.16.0.50`; override
`EBIM_ARM_HOST` and `EBIM_BASE_HOST` if the venue uses different addresses or
usernames.

An `SSH permission denied` error means the network endpoint was reached but the
evaluator credential is not authorized. A timeout means the configured host is
not reachable from the container. Neither condition should be worked around by
adding credentials to the repository.

## ROS environment

The default staged arm loader reads an allowlist of ROS runtime variables from
an already-running robot process owned by the remote user. This preserves the
deployed ROS distribution, domain, middleware, interface overlay, and DDS
configuration without requiring a private shell script. It never prints or
copies unrelated process-environment entries.

If the robot process environment cannot be read, pass an existing non-secret
setup file with `EBIM_ARM_ENV`, or define `EBIM_ARM_OVERLAYS` in that setup file.
The check fails before motion if `rclpy`, `franka_msgs`, `franka_spine_msgs`, or
`moveit_msgs` cannot be imported.

## Runtime directories and locks

`/run/screen` is a runtime directory created by the host's `screen` package; it
must not be committed. The Task 3 policy uses bounded locks under `/tmp` on the
ROS hosts and a disposable `/root/.tmr_three_object_delivery` tmpfs in the
container. No Task 2 lock file is required.

## Interface mismatch

If `check` reports a missing MoveIt service, camera topic, action, odometry, or
base command topic, do not start a duplicate driver. Report the deployed
interface name and type. The policy should be updated and pinned against that
contract rather than relying on a same-name or cross-distribution DDS guess.

## Venue-dependent settings

The default pickup-table height and the associated arm/spine calibration are
the same as the verified Shanghai setup. Keep those defaults unchanged for the
evaluation table. A height change is not part of this portability patch.

The following deployment values were used in Shanghai but should be confirmed
on the evaluation testbed before `execute`:

| Setting | Current default | Supported adjustment |
| --- | --- | --- |
| Arm SSH endpoint | `aup@172.16.0.100` | Set `EBIM_ARM_HOST` on `docker run`. |
| Base SSH endpoint | `tmr-user@172.16.0.50` | Set `EBIM_BASE_HOST` on `docker run`. |
| Remote staging directory | `/tmp/edl_task3_phase2` | Set `EBIM_REMOTE_ROOT`; it must be writable on both hosts. |
| Arm ROS environment | Auto-detected from a running robot process | Set `EBIM_ARM_ENV` to a non-secret setup file, or use `EBIM_ARM_ROS_DISTRO` and `EBIM_ARM_OVERLAYS`. |
| Base control DDS | Humble, domain 97, CycloneDDS, local host graph | Keep it isolated from the Jazzy arm graph. If the deployed base contract differs, update `docker/load_base_environment.sh` and the `TMR_CYCLE_*` defaults in `base/scripts` consistently. |
| Head ZED | ROS topic and serial configured in the base scripts | Adjust `TMR_CYCLE_VISION_DOMAIN_ID`, `TMR_CYCLE_ZED_FRAME_FILE`, and the serial/topic settings in `base/scripts/14_run_letter_guided_search.sh`, `base/scripts/18_start_zed_stream.sh`, and `mission/config/letter_delivery.json`. |
| Spine REST readiness probe | Shanghai base-controller address | This probe is advisory. If the venue address differs, update `check_tmr_state` in `base/scripts/03_start_navigation.sh`; the authoritative readiness decision remains the controller RPC/FCI handshake. |
| ROS service/action/topic names | Contract listed in `docker/check_remote_runtime.sh` | Update both the checker and the consuming script together after confirming the deployed name and message type. |

Do not compensate for an endpoint or DDS mismatch by changing the calibrated
table height, object descent, navigation distances, or placement offsets. Run
`check` again after any deployment-only adjustment and include its JSON output
when reporting a failure.

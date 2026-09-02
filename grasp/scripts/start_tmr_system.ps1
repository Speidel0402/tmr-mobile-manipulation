[CmdletBinding()]
param(
    [string]$ConfigPath,
    [switch]$EnableTeleop,
    [switch]$SkipArmRestore,
    [switch]$SkipGripperInitialize,
    [switch]$DryRun
)

$ErrorActionPreference = 'Stop'
if ([string]::IsNullOrWhiteSpace($ConfigPath)) {
    $ConfigPath = Join-Path $PSScriptRoot '..\config\system_startup.psd1'
}
$config = Import-PowerShellDataFile -LiteralPath (Resolve-Path $ConfigPath)

function ConvertTo-Base64Utf8([string]$Text) {
    [Convert]::ToBase64String([Text.Encoding]::UTF8.GetBytes($Text))
}

function Get-Target([hashtable]$HostConfig) {
    '{0}@{1}' -f $HostConfig.User, $HostConfig.Address
}

function Get-RemoteWrapper([string]$Command) {
    $payload = ConvertTo-Base64Utf8 $Command
    "printf %s $payload | base64 -d | bash"
}

function Start-RemoteWindow {
    param(
        [string]$Title,
        [hashtable]$HostConfig,
        [string]$Command
    )

    $target = Get-Target $HostConfig
    if ($DryRun) {
        Write-Host "[DRY-RUN][$Title] ssh $target"
        Write-Host $Command
        return
    }

    # Encode the entire child PowerShell command so quoting in ROS launch
    # arguments cannot be reinterpreted by Start-Process.
    $remote64 = ConvertTo-Base64Utf8 (Get-RemoteWrapper $Command)
    $child = @"
`$Host.UI.RawUI.WindowTitle = '$Title'
`$remote = [Text.Encoding]::UTF8.GetString([Convert]::FromBase64String('$remote64'))
& ssh.exe -tt '$target' `$remote
Write-Host 'Remote session ended. Press Enter to close.'
[void](Read-Host)
"@
    $child64 = [Convert]::ToBase64String([Text.Encoding]::Unicode.GetBytes($child))
    Start-Process -FilePath 'powershell.exe' -ArgumentList @(
        '-NoLogo', '-NoProfile', '-EncodedCommand', $child64
    ) | Out-Null
    Write-Host "Started $Title on $target"
}

function Invoke-Remote {
    param(
        [string]$Title,
        [hashtable]$HostConfig,
        [string]$Command
    )

    $target = Get-Target $HostConfig
    if ($DryRun) {
        Write-Host "[DRY-RUN][$Title] ssh $target"
        Write-Host $Command
        return
    }
    Write-Host "Running $Title on $target"
    & ssh.exe -tt $target (Get-RemoteWrapper $Command)
    if ($LASTEXITCODE -ne 0) {
        throw "$Title failed with SSH exit code $LASTEXITCODE"
    }
}

function Wait-Startup([string]$Name) {
    if (-not $DryRun) {
        Start-Sleep -Seconds ([int]$config.StartupDelaySeconds[$Name])
    }
}

function Format-RosArray([object[]]$Values) {
    '[' + (($Values | ForEach-Object { [string]::Format(
        [Globalization.CultureInfo]::InvariantCulture, '{0:R}', [double]$_
    ) }) -join ', ') + ']'
}

$baseHost = $config.Hosts.Base
$robotHost = $config.Hosts.Robot
$teleopHost = $config.Hosts.Teleop

$robotEnvironment = 'source ~/tmr_env.sh'

Start-RemoteWindow -Title 'TMR Base' -HostConfig $baseHost -Command @"
exec bash ~/tmr_cycle/scripts/03_start_navigation.sh
"@
Wait-Startup 'Base'

Invoke-Remote -Title 'TMR base readiness' -HostConfig $baseHost -Command @"
set -e
for attempt in {1..75}; do
  if [[ -s /tmp/tmr_navigation_stack.ready ]] && \
     grep -q '^domain=97$' /tmp/tmr_navigation_stack.ready; then
    echo 'Base controller, odometry, dual LiDAR, SLAM and command adapter are ready.'
    exit 0
  fi
  sleep 1
done
echo 'Timed out waiting for the managed base stack.' >&2
exit 72
"@

Start-RemoteWindow -Title 'ZED Head Camera' -HostConfig $baseHost -Command @"
exec bash ~/tmr_cycle/scripts/18_start_zed_stream.sh
"@
Wait-Startup 'Zed'

Invoke-Remote -Title 'ZED RGB readiness' -HostConfig $baseHost -Command @"
set -e
for attempt in {1..30}; do
  now=`$(date +%s)
  modified=`$(stat -c %Y /tmp/tmr_zed_latest.jpg 2>/dev/null || echo 0)
  if (( now - modified <= 2 )) && curl -fsS --max-time 2 \
      http://127.0.0.1:18082/tmr_zed_latest.jpg >/dev/null; then
    echo 'Serial-bound ZED RGB and HTTP bridge are fresh.'
    exit 0
  fi
  sleep 1
done
echo 'Timed out waiting for the managed ZED stream.' >&2
exit 73
"@

Start-RemoteWindow -Title 'Franka Spine' -HostConfig $robotHost -Command @"
$robotEnvironment
if pgrep -f '[f]ranka_spine_server|[s]pine.launch.py' >/dev/null; then
  echo 'Spine service already appears to be running.'
  exec bash
fi
exec ros2 launch franka_spine_server spine.launch.py spine_ip:=172.16.16.10
"@
Wait-Startup 'Spine'

Start-RemoteWindow -Title 'Robotiq Duo' -HostConfig $robotHost -Command @"
$robotEnvironment
test -e '$($config.Grippers.LeftById)' || { echo 'Missing configured LEFT Robotiq by-id device.'; exit 20; }
test -e '$($config.Grippers.RightById)' || { echo 'Missing configured RIGHT Robotiq by-id device.'; exit 21; }
if pgrep -f '[f]ranka_gripper_manager|[r]obotiq_gripper' >/dev/null; then
  echo 'Robotiq manager already appears to be running.'
  exec bash
fi
exec ros2 launch franka_gripper_manager robotiq_gripper_controller_client.launch.py config_file:=$($config.Grippers.ConfigFile)
"@
Wait-Startup 'Grippers'

Start-RemoteWindow -Title 'FR3 Duo Arms' -HostConfig $robotHost -Command @"
$robotEnvironment
if pgrep -f '[f]ranka_fr3_arm_controllers.launch.py' >/dev/null; then
  echo 'Dual-arm controller already appears to be running.'
  exec bash
fi
exec ros2 launch franka_fr3_arm_controllers franka_fr3_arm_controllers.launch.py robot_config_file:=$($config.Arms.ConfigFile)
"@
Wait-Startup 'Arms'

Start-RemoteWindow -Title 'Left FK IK' -HostConfig $robotHost -Command @"
$robotEnvironment
if pgrep -f '[s]tart_left_ik_service.py' >/dev/null; then
  if ros2 service list 2>/dev/null | grep -qx '/left_ik/compute_fk' && \
     ros2 service list 2>/dev/null | grep -qx '/left_ik/compute_cartesian_path'; then
    echo 'Reusing healthy left-arm FK/IK geometry service.'
    exec bash
  fi
  echo 'Left-arm FK/IK process exists without geometry services; restarting it.'
  pkill -INT -f '[s]tart_left_ik_service.py' 2>/dev/null || true
  for attempt in {1..30}; do
    pgrep -f '[s]tart_left_ik_service.py' >/dev/null || break
    sleep 0.1
  done
  pkill -TERM -f '[s]tart_left_ik_service.py' 2>/dev/null || true
  pkill -TERM -f '[m]ove_group.*-r __ns:=/left_ik' 2>/dev/null || true
fi
cd ~/tmr-mobile-manipulation
exec python3 grasp/scripts/start_left_ik_service.py
"@
Wait-Startup 'LeftIk'

Invoke-Remote -Title 'Left FK IK readiness' -HostConfig $robotHost -Command @"
$robotEnvironment
set -e
for attempt in {1..30}; do
  services=`$(ros2 service list 2>/dev/null || true)
  if grep -qx '/left_ik/compute_fk' <<<"`$services" && \
     grep -qx '/left_ik/compute_cartesian_path' <<<"`$services"; then
    echo 'Left-arm FK and Cartesian-path services are ready.'
    exit 0
  fi
  sleep 1
done
echo 'Timed out waiting for left-arm FK/IK services.' >&2
exit 74
"@

Start-RemoteWindow -Title 'D405 Duo' -HostConfig $robotHost -Command @"
$robotEnvironment
if pgrep -f '[r]s_multi_camera_launch.py|[r]ealsense2_camera_node' >/dev/null; then
  if [[ "`$(ros2 param get /wrist_camera_left serial_no 2>/dev/null)" == 'String value is: _$($config.D405.LeftSerial)' ]] && \
     [[ "`$(ros2 param get /wrist_camera_right serial_no 2>/dev/null)" == 'String value is: _$($config.D405.RightSerial)' ]] && \
     timeout 3 ros2 topic echo --once --qos-reliability best_effort /wrist_camera_left/color/image_raw >/dev/null 2>&1 && \
     timeout 3 ros2 topic echo --once --qos-reliability best_effort /wrist_camera_right/color/image_raw >/dev/null 2>&1; then
    echo 'Reusing healthy serial-bound D405 cameras with fresh RGB frames.'
    exec bash
  fi
  echo 'Existing D405 service is stale or bound incorrectly; restarting only the D405 pair.'
  pkill -INT -f '[r]os2 launch realsense2_camera rs_multi_camera_launch.py' 2>/dev/null || true
  for attempt in {1..30}; do
    pgrep -f '[r]s_multi_camera_launch.py|[r]ealsense2_camera_node.*__node:=wrist_camera_' >/dev/null || break
    sleep 0.1
  done
  pkill -TERM -f '[r]os2 launch realsense2_camera rs_multi_camera_launch.py' 2>/dev/null || true
  pkill -TERM -f '[r]ealsense2_camera_node.*__node:=wrist_camera_' 2>/dev/null || true
fi
exec ros2 launch realsense2_camera rs_multi_camera_launch.py \
  camera_namespace1:='/' camera_name1:=wrist_camera_left serial_no1:=_$($config.D405.LeftSerial) \
  enable_sync1:=false enable_color1:=true enable_depth1:=false \
  depth_module.color_profile1:=640x480x30 depth_module.depth_profile1:=640x480x30 \
  camera_namespace2:='/' camera_name2:=wrist_camera_right serial_no2:=_$($config.D405.RightSerial) \
  enable_sync2:=false enable_color2:=true enable_depth2:=false \
  depth_module.color_profile2:=640x480x30 depth_module.depth_profile2:=640x480x30
"@
Wait-Startup 'D405'

Invoke-Remote -Title 'D405 RGB readiness' -HostConfig $robotHost -Command @"
$robotEnvironment
set -e
timeout 12 ros2 topic echo --once --qos-reliability best_effort /wrist_camera_left/color/image_raw >/dev/null
timeout 12 ros2 topic echo --once --qos-reliability best_effort /wrist_camera_right/color/image_raw >/dev/null
echo 'Both serial-bound D405 RGB streams are fresh.'
"@

Start-RemoteWindow -Title 'Left Wrist Snapshot' -HostConfig $robotHost -Command @"
$robotEnvironment
cd ~/tmr-mobile-manipulation
if curl -fsS --max-time 2 http://127.0.0.1:18080/healthz 2>/dev/null | \
   python3 -c "import json,sys; d=json.load(sys.stdin); assert d['status']=='ok' and d['camera_role']=='left_wrist' and d['rgb_topic']=='/wrist_camera_left/color/image_raw' and d['rgb_sequence']>0"; then
  echo 'Left-wrist snapshot service is already healthy.'
  exec bash
fi
pkill -INT -f '[c]amera_mjpeg_viewer.py.*--port 18080' 2>/dev/null || true
for attempt in {1..20}; do
  pgrep -f '[c]amera_mjpeg_viewer.py.*--port 18080' >/dev/null || break
  sleep 0.1
done
pkill -TERM -f '[c]amera_mjpeg_viewer.py.*--port 18080' 2>/dev/null || true
exec python3 tools/camera_mjpeg_viewer.py \
  --port 18080 --camera-role left_wrist \
  --rgb-topic /wrist_camera_left/color/image_raw \
  --depth-topic /wrist_camera_left/depth/image_rect_raw \
  --camera-info-topic /wrist_camera_left/color/camera_info \
  --title Left_Wrist_D405
"@
Wait-Startup 'WristSnapshot'

Invoke-Remote -Title 'Left wrist snapshot readiness' -HostConfig $robotHost -Command @"
set -e
for attempt in {1..20}; do
  first=`$(curl -fsS --max-time 2 http://127.0.0.1:18080/healthz 2>/dev/null | python3 -c "import json,sys; print(json.load(sys.stdin)['rgb_sequence'])" 2>/dev/null || echo 0)
  sleep 0.15
  second=`$(curl -fsS --max-time 2 http://127.0.0.1:18080/healthz 2>/dev/null | python3 -c "import json,sys; print(json.load(sys.stdin)['rgb_sequence'])" 2>/dev/null || echo 0)
  if (( first > 0 && second > first )); then
    echo "Left-wrist RGB snapshot service is fresh and advancing (`$first -> `$second)."
    exit 0
  fi
  sleep 0.5
done
echo 'Timed out waiting for left-wrist snapshot service.' >&2
exit 75
"@

# Acquire the adapter lease with a zero target; never publish beside the
# adapter on the controller's private input.
Invoke-Remote -Title 'base mission-neutral mode' -HostConfig $baseHost -Command @"
exec bash ~/tmr_cycle/scripts/17_control_mode.sh mission
"@

if (-not $SkipArmRestore) {
    $left = Format-RosArray $config.Arms.Left
    $right = Format-RosArray $config.Arms.Right
    $velocity = [string]::Format(
        [Globalization.CultureInfo]::InvariantCulture,
        '{0:R}', [double]$config.Arms.MaximumJointVelocity
    )
    $tolerance = [string]::Format(
        [Globalization.CultureInfo]::InvariantCulture,
        '{0:R}', [double]$config.Arms.GoalTolerance
    )
    $velocities = '[' + ((1..7 | ForEach-Object { $velocity }) -join ', ') + ']'

    Invoke-Remote -Title 'dual-arm grasp initial state' -HostConfig $robotHost -Command @"
$robotEnvironment
set -e
switch_arm() {
  arm=`$1
  activate=`$2
  deactivate=`$3
  ros2 service call /`$arm/controller_manager/switch_controller \
    controller_manager_msgs/srv/SwitchController \
    "{activate_controllers: [`$activate], deactivate_controllers: [`$deactivate], strictness: 2, activate_asap: false, timeout: {sec: 5, nanosec: 0}}"
}
move_arm() {
  arm=`$1
  goal=`$2
  switch_arm "`$arm" '' joint_impedance_controller
  if ros2 action send_goal /`$arm/action_server/ptp_motion franka_msgs/action/PTPMotion \
      "{goal_joint_configuration: `$goal, maximum_joint_velocities: $velocities, goal_tolerance: $tolerance}"; then
    switch_arm "`$arm" joint_impedance_controller ''
  else
    switch_arm "`$arm" joint_impedance_controller '' || true
    exit 30
  fi
}
move_arm right '$right'
move_arm left '$left'
"@
}

if (-not $SkipGripperInitialize) {
    $opening = [string]::Format(
        [Globalization.CultureInfo]::InvariantCulture,
        '{0:R}', [double]$config.Grippers.InitialWidthPercent
    )
    Invoke-Remote -Title 'open both grippers' -HostConfig $robotHost -Command @"
$robotEnvironment
ros2 topic pub --once /left/gripper/gripper_client/target_gripper_width_percent std_msgs/msg/Float32 "{data: $opening}"
ros2 topic pub --once /right/gripper/gripper_client/target_gripper_width_percent std_msgs/msg/Float32 "{data: $opening}"
"@
}

if ($null -ne $config.SpineHomePositionMeters) {
    $spinePosition = [string]::Format(
        [Globalization.CultureInfo]::InvariantCulture,
        '{0:R}', [double]$config.SpineHomePositionMeters
    )
    $spineVelocity = [string]::Format(
        [Globalization.CultureInfo]::InvariantCulture,
        '{0:R}', [double]$config.SpineHomeVelocity
    )
    $spineAcceleration = [string]::Format(
        [Globalization.CultureInfo]::InvariantCulture,
        '{0:R}', [double]$config.SpineHomeAcceleration
    )
    $spineDeceleration = [string]::Format(
        [Globalization.CultureInfo]::InvariantCulture,
        '{0:R}', [double]$config.SpineHomeDeceleration
    )
    Invoke-Remote -Title 'Spine home' -HostConfig $robotHost -Command @"
$robotEnvironment
set -e
ros2 service call /franka_spine_node/switch_on franka_spine_msgs/srv/SwitchOn '{}'
ros2 action send_goal /franka_spine_node/move_absolute franka_spine_msgs/action/MoveAbsolute \
  "{position: $spinePosition, velocity: $spineVelocity, acceleration: $spineAcceleration, deceleration: $spineDeceleration}" \
  --feedback
"@
} else {
    Write-Warning 'Spine home skipped: SpineHomePositionMeters is not configured.'
}

if ($EnableTeleop) {
    Start-RemoteWindow -Title 'GELLO Duo' -HostConfig $teleopHost -Command @"
$robotEnvironment
exec ros2 launch franka_gello_state_publisher main.launch.py config_file:=franka_gello_duo.yaml
"@
    Start-RemoteWindow -Title 'Pedal Teleop' -HostConfig $teleopHost -Command @"
$robotEnvironment
exec ros2 launch tmr_pedal_teleop mobile_teleop.launch.py
"@
}

Write-Host 'Startup commands dispatched and initialization sequence completed.'
Write-Host 'Passwords are never stored; configure OpenSSH keys/agent for unattended one-click use.'

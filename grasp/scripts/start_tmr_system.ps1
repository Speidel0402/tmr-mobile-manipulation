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

$baseEnvironment = @'
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
'@
$robotEnvironment = 'source ~/tmr_env.sh'

Start-RemoteWindow -Title 'TMR Base' -HostConfig $baseHost -Command @"
$baseEnvironment
if pgrep -f '[t]mrv0_2.launch.py|[s]werve_drive_controller' >/dev/null; then
  echo 'TMR base already appears to be running.'
  exec bash
fi
exec ros2 launch franka_bringup tmrv0_2.launch.py controller_name:=swerve_drive_controller
"@
Wait-Startup 'Base'

Start-RemoteWindow -Title 'ZED Head Camera' -HostConfig $baseHost -Command @"
$baseEnvironment
if pgrep -f '[z]ed_camera.launch.py|[z]ed_container' >/dev/null; then
  echo 'ZED camera already appears to be running.'
  exec bash
fi
cd ~/ros2_ws
exec ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zedm namespace:=head_camera publish_tf:=false serial_number:=17064700
"@
Wait-Startup 'Zed'

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

Start-RemoteWindow -Title 'D405 Duo' -HostConfig $robotHost -Command @"
$robotEnvironment
if pgrep -f '[r]s_multi_camera_launch.py|[r]ealsense2_camera_node' >/dev/null; then
  echo 'D405 cameras already appear to be running.'
  exec bash
fi
exec ros2 launch realsense2_camera rs_multi_camera_launch.py \
  camera_namespace1:='/' camera_name1:=wrist_camera_left serial_no1:=_$($config.D405.LeftSerial) \
  enable_sync1:=false enable_depth1:=true \
  depth_module.color_profile1:=640x480x30 depth_module.depth_profile1:=640x480x30 \
  camera_namespace2:='/' camera_name2:=wrist_camera_right serial_no2:=_$($config.D405.RightSerial) \
  enable_sync2:=false enable_depth2:=true \
  depth_module.color_profile2:=640x480x30 depth_module.depth_profile2:=640x480x30
"@
Wait-Startup 'D405'

# A zero Twist is a safe base neutral command. It intentionally does not try to
# drive the mobile base back to an odometry origin.
Invoke-Remote -Title 'base neutral command' -HostConfig $baseHost -Command @"
$baseEnvironment
ros2 topic pub --once /swerve_drive_controller/cmd_vel geometry_msgs/msg/TwistStamped \
  '{twist: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}'
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

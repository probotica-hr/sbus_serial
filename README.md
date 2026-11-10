# sbus_serial

ROS 2 package for parsing DJI/Futaba SBUS serial data from an RC receiver for remote control (teleoperation) of ROS-based robots. Probotica fork with additional features for robot control.

SBUS is a serial protocol for RC receivers where the values of up to 16 channels are sent over one serial channel.

## Nodes

### sbus_serial_node

Reads SBUS data from a serial port and publishes parsed channel values.

**Publishes:** `/sbus` (`sbus_serial/msg/Sbus`)

**Parameters:**
| Parameter | Default | Description |
|-----------|---------|-------------|
| `port` | `/dev/tty11` | Serial port for SBUS receiver |
| `refresh_rate_hz` | `2` | Polling rate |
| `rxMinValue` | `172` | Raw SBUS minimum value |
| `rxMaxValue` | `1811` | Raw SBUS maximum value |
| `outMinValue` | `0` | Mapped output minimum |
| `outMaxValue` | `255` | Mapped output maximum |
| `silentOnFailsafe` | `false` | Suppress publishing during failsafe |
| `enableChannelNum` | `-1` | Channel for proportional enable (-1 = disabled) |
| `deadband` | `0` | Raw value deadband around midpoint |

### sbus_commands (sbus_cmd_vel_node)

Subscribes to SBUS data and publishes `Twist` or `TwistStamped` velocity commands. Includes turbo mode, deadman switch, connection timeout, and utility command outputs.

**Subscribes:** `/sbus` (`sbus_serial/msg/Sbus`)

**Publishes:**
- `/output/sbus/cmd_vel` (`Twist` or `TwistStamped`) — velocity command, remap as needed
- `/joystick_enable` (`Bool`) — enable state from control channel
- `/joystick_commands` (`sbus_serial/msg/SbusCommands`) — utility commands (lights, horn, battery)

**Parameters:**
| Parameter | Default | Description |
|-----------|---------|-------------|
| `forwardChannelIndx` | `1` | Channel index for forward/reverse |
| `turnChannelIndx` | `0` | Channel index for turning |
| `sbusMinValue` | `0` | Expected SBUS input minimum |
| `sbusMaxValue` | `255` | Expected SBUS input maximum |
| `minSpeed` / `maxSpeed` | `-1.0` / `1.0` | Speed range (m/s) |
| `minTurn` / `maxTurn` | `-1.0` / `1.0` | Turn rate range (rad/s) |
| `minTurboSpeed` / `maxTurboSpeed` | `-1.0` / `1.0` | Turbo speed range (m/s) |
| `minTurboTurn` / `maxTurboTurn` | `-1.0` / `1.0` | Turbo turn range (rad/s) |
| `turboChannelIndx` | `-1` | Channel index for turbo toggle (-1 = disabled) |
| `turboChannelValue` | `100.0` | Threshold to activate turbo |
| `disableCmdVelChannelIndx` | `-1` | Channel index for disabling cmd_vel output |
| `disableCmdVelChannelValue` | `100.0` | Threshold to disable cmd_vel |
| `enableControlChannelIndex` | `-1` | Channel index for joystick enable |
| `enableControlChannelValue` | `100.0` | Threshold for enable |
| `useStamped` | `true` | Publish `TwistStamped` instead of `Twist` |
| `frameId` | `""` | frame_id for `TwistStamped` header |
| `deadband` | `0.0` | Deadband around midpoint |
| `timeoutThreshold` | `0.25` | Seconds without data before sending zero reference |
| `refresh_rate_hz` | `20` | Timer rate for timeout checking |

**Utility commands** (set `enableUtilCommands: true`):
| Parameter | Description |
|-----------|-------------|
| `batteryChannelIndex` / `batteryChannelValue` / `batteryChannelToggle` | Battery charge toggle |
| `lightChannelIndex` / `lightChannelValue` / `lightChannelToggle` | Lights toggle |
| `hornChannelIndex` / `hornChannelValue` / `hornChannelToggle` | Horn toggle |

#### Winch mode (hold-to-run)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `winchModeIndex` | `-1` | Mode value (0/1/2 from the mode switch) that is "winch mode" (-1 = disabled) |
| `winchChannelIndx` | `1` | Channel index read as the winch stick (default: forward stick) |
| `winchChannelThreshold` | `90.0` | `abs(value)` at or above this counts as full deflection |
| `winchInvert` | `false` | `false`: +stick = unwind (pay out), -stick = wind (pull in) |
| `winchWindService` / `winchUnwindService` / `winchStopService` | `/winch/winch/{wind,unwind,stop}` | `std_srvs/Trigger` services of the winch driver |
| `winchStopRetryPeriod` | `0.5` | Retry a failed stop after this many seconds |
| `winchServiceTimeout` | `2.0` | Treat an unanswered call as failed after this long |
| `winchMock` | `false` | Dry run: no service is called; the name of the service that would have been called is published on `winch_mock` (`std_msgs/String`, transient_local), once per change |

In winch mode the published twist is forced to zero (with `joy_enable` still true, so the
joystick slot keeps priority in the reference mux and the robot holds still) and the winch
stick is read as a tri-state: full one way calls the wind service, full the other way calls
the unwind service, anything else calls stop. Each service is called once per change; a
direction change goes through stop first; the stick must be seen centred once after entering
winch mode before a direction is accepted; deadman release, SBUS timeout and leaving winch
mode all send stop. The decision logic lives in `include/winch_hold_to_run.hpp` and is unit
tested in `test/` (`colcon test --packages-select sbus_serial`).

To validate a new channel mapping without touching the winch, set `winchMock: true` and watch
`ros2 topic echo /winch_mock` while flipping the mode switch and moving the stick; the twist
is still zeroed in winch mode, so the robot behaves exactly as it will when armed.

### sbus_calibrate

Standalone executable for calibrating raw SBUS values from the receiver. Installed in the package's `share` directory.

## Custom Messages

- `sbus_serial/msg/Sbus` — Header + 16 raw channels + 16 mapped channels + failsafe/frame_lost flags
- `sbus_serial/msg/SbusCommands` — Utility command states (lights, horn, battery_charge)

## Launch

```bash
ros2 launch sbus_serial sbus_ros2.launch.py
```

**Launch arguments:**
| Argument | Default | Description |
|----------|---------|-------------|
| `port` | `/dev/ttyUSB0` | Serial port |
| `params_file` | `config/sbus_config.yaml` | Parameter file |
| `controller_reference_topic` | `/seeker_control/seeker_steering_controller/reference` | Output topic for cmd_vel (remapped) |

The launch file starts both `sbus_serial_node` and `sbus_commands`, remapping the cmd_vel output to the specified controller reference topic.

## Configuration

See `config/sbus_config.yaml` for a complete example configuration used with Probotica robots.

## Building

```bash
colcon build --packages-select sbus_serial
```

Requires Boost (`sudo apt-get install libboost-dev` if missing).

## Origin

Forked from [jenswilly/sbus_serial](https://github.com/jenswilly/sbus_serial). Extended with turbo mode, deadman switch, connection timeout handling, utility commands, and TwistStamped support.

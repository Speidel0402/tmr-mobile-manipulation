@{
    Hosts = @{
        Base = @{ Address = '172.16.0.50'; User = 'tmr-user' }
        Robot = @{ Address = '172.16.0.100'; User = 'aup' }
        Teleop = @{ Address = '172.16.0.101'; User = 'aup' }
    }

    # Delays allow launch processes to construct their ROS graph before the next
    # dependent component is started. They are intentionally configurable.
    StartupDelaySeconds = @{
        Base = 8
        Zed = 8
        Spine = 5
        Grippers = 7
        Arms = 12
        D405 = 8
    }

    D405 = @{
        LeftSerial = '409122272639'
        RightSerial = '409122274492'
    }

    Grippers = @{
        ConfigFile = 'example_fr3_duo_config_robotiq.yaml'
        LeftById = '/dev/serial/by-id/usb-FTDI_USB_TO_RS-485_DAANVRU5-if00-port0'
        RightById = '/dev/serial/by-id/usb-FTDI_USB_TO_RS-485_DAANTK6Q-if00-port0'
        InitialWidthPercent = 0.8
    }

    Arms = @{
        ConfigFile = 'tmr_duo_config.yaml'
        MaximumJointVelocity = 0.04
        GoalTolerance = 0.006
        Left = @(
            -2.4404187202453613,
            -1.0571141242980957,
             2.269590377807617,
            -1.2946336269378662,
             1.6401503086090088,
             1.4328300952911377,
            -1.7251158952713013
        )
        Right = @(
            -1.6818886995315552,
            -0.7537260055541992,
             0.8576784729957581,
            -2.6971640586853027,
            -0.5807996392250061,
             2.465313673019409,
             1.4973987340927124
        )
    }

    # User-confirmed absolute Spine height for the grasp initial state.
    SpineHomePositionMeters = 0.7
    SpineHomeVelocity = 0.05
    SpineHomeAcceleration = 0.1
    SpineHomeDeceleration = 0.1
}

package org.firstinspires.ftc.teamcode.qualifier;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;

import org.firstinspires.ftc.teamcode.scrimmage.RobotHardware;

public class Navigation
{
    RobotHardware robotHardware;

    protected Navigation(RobotHardware hardware)
    {
        robotHardware = hardware;
    }

    protected BNO055IMU getImu(){return robotHardware.imu;}

}

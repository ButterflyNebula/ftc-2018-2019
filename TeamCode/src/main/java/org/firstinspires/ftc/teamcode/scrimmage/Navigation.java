package org.firstinspires.ftc.teamcode.scrimmage;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;

public class Navigation
{
    RobotHardware robotHardware;

    protected Navigation(RobotHardware hardware)
    {
        robotHardware = hardware;
    }

    protected BNO055IMU getImu(){return robotHardware.imu;}

    protected Rev2mDistanceSensor getDistanceSensor(){return robotHardware.rev2mDistanceSensor;}

}

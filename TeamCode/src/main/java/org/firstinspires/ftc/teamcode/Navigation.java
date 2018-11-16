package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

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

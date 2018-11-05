package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by Athira on 10/14/2018.
 */

public class LiftAssembly{

    RobotHardware robotHardware;


    protected LiftAssembly(RobotHardware hardware)
    {
        robotHardware = hardware;

    }


    protected void liftUpRobot(double speed)
    {
        robotHardware.lift.setPower(speed);
    }

    protected void resetLift()
    {
        robotHardware.lift.setPower(0);
    }

    protected void lowerRobot(double speed)
    {
        robotHardware.lift.setPower(-speed);
    }


    protected void lockRobot(double position)
    {
        robotHardware.liftLock.setPosition(position);
    }

    protected void unlockRobot(double position)
    {
        robotHardware.liftLock.setPosition(position);
    }

}

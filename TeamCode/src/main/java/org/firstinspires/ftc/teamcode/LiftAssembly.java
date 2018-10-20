package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by Athira on 10/14/2018.
 */

public class LiftAssembly extends Assembly {


    protected LiftAssembly(RobotHardware hardware)
    {
        super(hardware);

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


    protected void lockRobot()
    {

    }

    protected void unlockRobot()
    {

    }

}

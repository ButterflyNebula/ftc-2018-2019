package org.firstinspires.ftc.teamcode.qualifier;


/**
 * Created by Athira on 10/14/2018.
 */

public class LiftAssembly {

    RobotHardware robotHardware;


    protected LiftAssembly(RobotHardware hardware)
    {
        robotHardware = hardware;

    }

    protected void liftUpRobot(double speed)
    {
        robotHardware.robotLift.setPower(speed);
    }

    protected void resetLift()
    {
        robotHardware.robotLift.setPower(0);
    }

    protected void lowerRobot(double speed)
    {
        robotHardware.robotLift.setPower(-speed);
    }

}

package org.firstinspires.ftc.teamcode.scrimmage;

/**
 * Created by Athira on 10/14/2018.
 */

public class LiftAssembly{

    RobotHardware robotHardware;

    private static final double     UNLOCK_POSITION   = 0.5;
    private static final double     LOCK_POSITION     = 0.7;


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


    protected void lockRobot()
    {
        robotHardware.liftLock.setPosition(LOCK_POSITION);
    }

    protected void unlockRobot()
    {
        robotHardware.liftLock.setPosition(UNLOCK_POSITION);
    }

}

package org.firstinspires.ftc.teamcode.qualifier;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.qualifier.RobotHardware;

/**
 * Created by Athira on 10/14/2018.
 */

public class ArmAssembly
{

    RobotHardware robotHardware;

    protected ArmAssembly(RobotHardware hardware)
    {

        robotHardware = hardware;
    }


    protected void extendGrabber(double speed)
    {
        robotHardware.intakeSlides.setPower(speed);
    }

    protected void retractGrabber(double speed)
    {
        robotHardware.intakeSlides.setPower(-speed);
    }

    protected void stopGrabberExtension()
    {
        robotHardware.intakeSlides.setPower(0);
    }


    protected void extendDeposit(double speed)
    {
        robotHardware.deliveryLift.setPower(speed);
    }

    protected void retractDeposit(double speed)
    {
        robotHardware.deliveryLift.setPower(-speed);
    }

    protected void stopDepositExtension()
    {
        robotHardware.deliveryLift.setPower(0);
    }

}

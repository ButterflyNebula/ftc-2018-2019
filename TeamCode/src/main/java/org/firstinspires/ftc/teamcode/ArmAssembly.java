package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by Athira on 10/14/2018.
 */

public class ArmAssembly{

    RobotHardware robotHardware;

    protected ArmAssembly(RobotHardware hardware)
    {

        robotHardware = hardware;
    }


    protected void Intake(double power)
    {
        robotHardware.leftIntake.setPower(power);
        robotHardware.rightIntake.setPower(power);
    }

    protected void armReturn (double power)
    {
            robotHardware.mineralArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robotHardware.mineralArm.setPower(power);
    }

    protected void armExtend (double power)
    {
        robotHardware.mineralArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robotHardware.mineralArm.setPower(-power);
    }

    protected void wristExtend (double power)
    {
        robotHardware.mineralArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robotHardware.mineralWrist.setPower(power);
    }

    protected void mineralDelivery (double power)
    {
        robotHardware.mineralLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robotHardware.mineralLift.setPower(-power);
    }

    protected void mineralFlip (double position)
    {
        robotHardware.goldBasket.setPosition(position);
        robotHardware.silverBasket.setPosition(1 - position);
    }
}

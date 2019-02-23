package org.firstinspires.ftc.teamcode.qualifier;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.qualifier.RobotHardware;

/**
 * Created by Athira on 10/14/2018.
 */

public class ArmAssembly
{
    double upPosition = 0.1;
    double downPosition = 1;

    RobotHardware robotHardware;

    protected ArmAssembly(RobotHardware hardware)
    {

        robotHardware = hardware;
    }

    protected void resetGrabber()
    {
        robotHardware.intakeSlides.resetDeviceConfigurationForOpMode();
        robotHardware.intakeSlides.setPower(0.0);
    }

    protected void extendGrabber(double speed)
    {
        robotHardware.intakeSlides.setPower(-speed);
    }

    protected void retractGrabber(double speed)
    {

        robotHardware.intakeSlides.setPower(speed);
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

    protected void intakeMineral(double speed)
    {
        robotHardware.intakeMineral.setPower(speed);
    }

    protected void outTakeMineral(double speed)
    {
        robotHardware.intakeMineral.setPower(-speed);
    }

    protected void stopIntake()
    {
        robotHardware.intakeMineral.setPower(0);
    }

    protected void flipUp () { robotHardware.flipper.setPosition(upPosition); }

    protected void flipDown () { robotHardware.flipper.setPosition(downPosition); }

    protected void flip(double position){robotHardware.flipper.setPosition(position);}

    protected void moveWrist (double power)
    {
        robotHardware.mineralWrist.setPower(-power);
    }

    public int getDeliveryLiftCurrentPosition() {return robotHardware.deliveryLift.getCurrentPosition();}

    public void setDeliveryLiftTargetPosition(int position) {robotHardware.deliveryLift.setTargetPosition(position);}

    public void changeToEncoderMode()
    {
        robotHardware.deliveryLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotHardware.deliveryLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setMode(DcMotor.RunMode mode)
    {
        robotHardware.deliveryLift.setMode(mode);
    }

    public boolean isDeliveryLiftBusy()
    {
        return robotHardware.deliveryLift.isBusy();
    }

}


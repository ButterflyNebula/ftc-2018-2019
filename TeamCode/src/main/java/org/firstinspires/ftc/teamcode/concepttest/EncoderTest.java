package org.firstinspires.ftc.teamcode.concepttest;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.qualifier.RoverRobot;

@TeleOp(name="EncoderTest2", group ="ConceptTests")
@Disabled
public class EncoderTest extends LinearOpMode
{

    final double COUNTS_PER_MOTOR_REV = 1120;
    final double DRIVE_GEAR_REDUCTION = 1.0;
    final double WHEEL_DIAMETER_INCHES = 4.0;
       final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
       (WHEEL_DIAMETER_INCHES * 3.1415);
    final double COUNTS_PER_DEGREE = 14.5;

    private RoverRobot roverRuckusBot = new RoverRobot();
    ElapsedTime runtime = new ElapsedTime();

    @Override public void runOpMode()
    {

        roverRuckusBot.initRobot(hardwareMap);


        telemetry.addData("Press Play to Start", "");
        telemetry.update();
        waitForStart();

        encoderDrive(0.4 , 12 , 12 , 5);
        sleep(5000);
        encoderDrive(0.4, -48 , -48 , 5);
    }

    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS)
    {
        int newBackLeftTarget;
        int newBackRightTarget;
        int newFrontLeftTarget;
        int newFrontRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive())
        {

            // Determine new target position, and pass to motor controller
            newBackLeftTarget = roverRuckusBot.getChassisAssembly().getBackLeftWheelCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newBackRightTarget = roverRuckusBot.getChassisAssembly().getBackLeftWheelCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            newFrontLeftTarget = roverRuckusBot.getChassisAssembly().getFrontLeftWheelCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newFrontRightTarget = roverRuckusBot.getChassisAssembly().getFrontRightWheelCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);

            roverRuckusBot.getChassisAssembly().setBackLeftWheelTargetPosition(newBackLeftTarget);
            roverRuckusBot.getChassisAssembly().setBackRightWheelTargetPosition(newBackRightTarget);
            roverRuckusBot.getChassisAssembly().setFrontLeftWheelTargetPosition(newFrontLeftTarget);
            roverRuckusBot.getChassisAssembly().setFrontRightWeelTargetPosition(newFrontRightTarget);


            // Turn On RUN_TO_POSITION
            roverRuckusBot.getChassisAssembly().setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            roverRuckusBot.getChassisAssembly().setBackRightWheelPower(Math.abs(speed));
            roverRuckusBot.getChassisAssembly().setBackLeftWheelPower(Math.abs(speed));
            roverRuckusBot.getChassisAssembly().setFrontLeftWheelPower(Math.abs(speed));
            roverRuckusBot.getChassisAssembly().setFrontRightWheelPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (roverRuckusBot.getChassisAssembly().isBackLeftWheelBusy() && roverRuckusBot.getChassisAssembly().isBackRightWheelBusy() &&
                    roverRuckusBot.getChassisAssembly().isFrontLeftWheelBusy() && roverRuckusBot.getChassisAssembly().isFrontRightWheelBusy()))
            {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newBackLeftTarget, newBackRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        roverRuckusBot.getChassisAssembly().getBackLeftWheelCurrentPosition(),
                        roverRuckusBot.getChassisAssembly().getBackRightWheelCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            roverRuckusBot.getChassisAssembly().setBackLeftWheelPower(0);
            roverRuckusBot.getChassisAssembly().setBackRightWheelPower(0);
            roverRuckusBot.getChassisAssembly().setFrontLeftWheelPower(0);
            roverRuckusBot.getChassisAssembly().setFrontRightWheelPower(0);

            // Turn off RUN_TO_POSITION
            roverRuckusBot.getChassisAssembly().changeToEncoderMode();

            //  sleep(250);   // optional pause after each move
        }
    }
}


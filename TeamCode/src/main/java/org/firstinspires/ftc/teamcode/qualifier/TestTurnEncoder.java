package org.firstinspires.ftc.teamcode.qualifier;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.qualifier.RoverRobot;


@TeleOp(name="EncoderTest", group ="Tests")
public class TestTurnEncoder extends LinearOpMode
{

    final double COUNTS_PER_MOTOR_REV = 1120;
    final double DRIVE_GEAR_REDUCTION = 1.0;
    final double WHEEL_DIAMETER_INCHES = 4.0;
 //   final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
         //   (WHEEL_DIAMETER_INCHES * 3.1415);
    final double COUNTS_PER_DEGREE = 14.5;

    private RoverRobot roverRuckusBot = new RoverRobot();
    ElapsedTime runtime = new ElapsedTime();

    @Override public void runOpMode()
    {

        roverRuckusBot.initRobot(hardwareMap);


        telemetry.addData("Press Play to Start", "");
        telemetry.update();
        waitForStart();

        double blPos = roverRuckusBot.getChassisAssembly().getBackLeftWheelCurrentPosition();
        double brPos = roverRuckusBot.getChassisAssembly().getBackRightWheelCurrentPosition();
        double frPos = roverRuckusBot.getChassisAssembly().getFrontRightWheelCurrentPosition();
        double flPos = roverRuckusBot.getChassisAssembly().getFrontLeftWheelCurrentPosition();

        telemetry.addData("Starting Positions:" , "");
        telemetry.addData("Back Left: " , blPos);
        telemetry.addData("Back Right: " , brPos);
        telemetry.addData("Front Left" , flPos);
        telemetry.addData("Front Right" , frPos);
        telemetry.update();
        sleep(1000);

        //Moving the Robot
        encoderDrive(0.4 , 180 , 2);

        sleep(5000);
        telemetry.addData("Current Positions:" , "");
        telemetry.addData("Back Left: " , blPos);
        telemetry.addData("Back Right: " , brPos);
        telemetry.addData("Front Left" , flPos);
        telemetry.addData("Front Right" , frPos);
        telemetry.update();
        sleep(8000);


        /*while (opModeIsActive())
        {
            if(gamepad1.a)
            {
                roverRuckusBot.getChassisAssembly().turnLeft(0.4);
                blPos = roverRuckusBot.getChassisAssembly().getBackLeftWheelCurrentPosition();
                brPos = roverRuckusBot.getChassisAssembly().getBackRightWheelCurrentPosition();
                frPos = roverRuckusBot.getChassisAssembly().getFrontRightWheelCurrentPosition();
                flPos = roverRuckusBot.getChassisAssembly().getFrontLeftWheelCurrentPosition();

                telemetry.addData("Current Positions:" , "");
                telemetry.addData("Back Left: " , blPos);
                telemetry.addData("Back Right: " , brPos);
                telemetry.addData("Front Left" , flPos);
                telemetry.addData("Front Right" , frPos);
                telemetry.update();
            }
            else
            {
                roverRuckusBot.getChassisAssembly().stopMoving();
            }
        }
        */
    }


    public void encoderDrive(double speed, double degrees, double timeoutS)
    {
        int newBackLeftTarget;
        int newBackRightTarget;
        int newFrontLeftTarget;
        int newFrontRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newBackLeftTarget = roverRuckusBot.getChassisAssembly().getBackLeftWheelCurrentPosition() + (int)(-degrees * COUNTS_PER_DEGREE);
            newBackRightTarget = roverRuckusBot.getChassisAssembly().getBackRightWheelCurrentPosition() + (int)(degrees * COUNTS_PER_DEGREE);
            newFrontLeftTarget = roverRuckusBot.getChassisAssembly().getFrontLeftWheelCurrentPosition() + (int)(-degrees * COUNTS_PER_DEGREE);
            newFrontRightTarget = roverRuckusBot.getChassisAssembly().getFrontRightWheelCurrentPosition() + (int)(degrees * COUNTS_PER_DEGREE);



            roverRuckusBot.getChassisAssembly().setBackLeftWheelTargetPosition(newBackLeftTarget);
            roverRuckusBot.getChassisAssembly().setBackRightWheelTargetPosition(newBackRightTarget);
            roverRuckusBot.getChassisAssembly().setFrontLeftWheelTargetPosition(newFrontLeftTarget);
            roverRuckusBot.getChassisAssembly().setFrontRightWeelTargetPosition(newFrontRightTarget);

            // Turn On RUN_TO_POSITION
            roverRuckusBot.getChassisAssembly().setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();
            roverRuckusBot.getChassisAssembly().setBackLeftWheelPower(Math.abs(speed));
            roverRuckusBot.getChassisAssembly().setBackRightWheelPower(Math.abs(speed));
            roverRuckusBot.getChassisAssembly().setFrontLeftWheelPower(Math.abs(speed));
            roverRuckusBot.getChassisAssembly().setFrontRightWheelPower(Math.abs(speed));


            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (roverRuckusBot.getChassisAssembly().isBackLeftWheelBusy() && roverRuckusBot.getChassisAssembly().isBackRightWheelBusy() &&
                            roverRuckusBot.getChassisAssembly().isFrontLeftWheelBusy() && roverRuckusBot.getChassisAssembly().isFrontRightWheelBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d : %7d :%7d",
                        newBackLeftTarget,  newBackRightTarget, newFrontLeftTarget, newFrontRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d : %7d : %7d",
                        roverRuckusBot.getChassisAssembly().getBackLeftWheelCurrentPosition(),
                        roverRuckusBot.getChassisAssembly().getBackRightWheelCurrentPosition(),
                        roverRuckusBot.getChassisAssembly().getFrontLeftWheelCurrentPosition(),
                        roverRuckusBot.getChassisAssembly().getFrontRightWheelCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            roverRuckusBot.getChassisAssembly().stopMoving();

            // Turn off RUN_TO_POSITION
            roverRuckusBot.getChassisAssembly().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }//end of encoderDrive
}
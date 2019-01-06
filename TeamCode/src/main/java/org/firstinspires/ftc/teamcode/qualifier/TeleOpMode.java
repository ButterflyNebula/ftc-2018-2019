package org.firstinspires.ftc.teamcode.qualifier;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "TeleOpMode" , group = "Qualifier")
public class TeleOpMode extends LinearOpMode
{
    private static       double     WHEEL_SPEED       = 1.0;
    private static final double     LIFT_UP_SPEED     = 0.6;
    private static final double     LIFT_DOWN_SPEED   = 1.0;

    //Creating a Rover robot object
    RoverRobot roverRuckusBot = new RoverRobot();


    @Override public void runOpMode()
    {
        roverRuckusBot.initRobot(hardwareMap);
        waitForStart();

        while (opModeIsActive())
        {

            /**
             * GAME PAD 1
             */
            //Controls
            double drive = gamepad1.left_stick_y;
            double turn = gamepad1.left_stick_x;
            double sideRight = gamepad1.right_trigger;
            double sideLeft = gamepad1.left_trigger;
            boolean liftRobot = gamepad1.dpad_up;
            boolean lowerRobot = gamepad1.dpad_down;


            //Movement
            //forwards
            if (drive > 0) {
                roverRuckusBot.getChassisAssembly().moveForward(-WHEEL_SPEED * drive);
            }
            //backwards
            else if (drive < 0) {
                roverRuckusBot.getChassisAssembly().moveBackwards(WHEEL_SPEED * drive);
            }
            //turn right
            else if (turn > 0)
            {
                roverRuckusBot.getChassisAssembly().turnRight(WHEEL_SPEED * turn);
            }
            //turn left
            else if (turn < 0)
            {
                roverRuckusBot.getChassisAssembly().turnLeft(WHEEL_SPEED * -turn);
            }
            //side right
            else if (sideRight > 0)
            {
                roverRuckusBot.getChassisAssembly().moveRight(-WHEEL_SPEED * sideRight);
            }
            //side left
            else if (sideLeft > 0)
            {
                roverRuckusBot.getChassisAssembly().moveLeft(-WHEEL_SPEED * sideLeft);
            }
            //stop moving
            else {
                roverRuckusBot.getChassisAssembly().stopMoving();
            }


            //Robot Lift
            if (liftRobot == true
                    && roverRuckusBot.getLiftAssembly().robotHardware.topTouch.getState() == true
                    )
            {
                telemetry.addData("gamepad1.left_bumper is pressed" , "");
                telemetry.update();
                roverRuckusBot.getLiftAssembly().liftUpRobot(LIFT_UP_SPEED);
            }
            //move lift down for retracting and lifting robot
            else if (lowerRobot == true)
            {
                roverRuckusBot.getLiftAssembly().lowerRobot(LIFT_DOWN_SPEED);
            }
            //stop moving the lift
            else
            {
                roverRuckusBot.getLiftAssembly().resetLift();
            }

            /**
             * GAME PAD 2
             */
            //Controls
            boolean intake = gamepad2.x;
            boolean outtake = gamepad2.b;
            boolean extendGrabber = gamepad2.dpad_right;
            boolean retractGrabber = gamepad2.dpad_left;
            boolean extendDeposit = gamepad2.dpad_up;
            boolean retractDeposit = gamepad2.dpad_down;
            boolean flip = gamepad2.y;


            //intake Mineral
            if(intake)
            {
                roverRuckusBot.getArmAssembly().intakeMineral(1.0);
            }
            else if(outtake)
            {
                roverRuckusBot.getArmAssembly().outTakeMineral(1.0);
            }
            else
            {
                roverRuckusBot.getArmAssembly().stopIntake();
            }

            //Sliding Out the Grabber
            if(extendGrabber)
            {
                roverRuckusBot.getArmAssembly().extendGrabber(0.8);
            }
            else if(retractGrabber)
            {
                roverRuckusBot.getArmAssembly().retractGrabber(0.8);
            }
            else
            {
                roverRuckusBot.getArmAssembly().stopGrabberExtension();
            }

            //Extending the Deposit
            if(extendDeposit)
            {
                roverRuckusBot.getArmAssembly().extendDeposit(1);
            }
            else if(retractDeposit)
            {
                roverRuckusBot.getArmAssembly().retractDeposit(1);
            }
            else
            {
                roverRuckusBot.getArmAssembly().stopDepositExtension();
            }

            //Flip
            if(flip)
            {
                roverRuckusBot.getArmAssembly().flip(1);
            }
            else
            {
                roverRuckusBot.getArmAssembly().flip(0);
            }



        }
    }




}
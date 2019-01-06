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
            //forwards
            if (gamepad1.left_stick_y > 0) {
                roverRuckusBot.getChassisAssembly().moveForward(-WHEEL_SPEED * gamepad1.left_stick_y);
            }
            //backwards
            else if (gamepad1.left_stick_y < 0) {
                roverRuckusBot.getChassisAssembly().moveBackwards(WHEEL_SPEED * gamepad1.left_stick_y);
            }
            //turn right
            else if (gamepad1.left_stick_x > 0)
            {
                roverRuckusBot.getChassisAssembly().turnRight(WHEEL_SPEED * gamepad1.left_stick_x);
            }
            //turn left
            else if (gamepad1.left_stick_x < 0)
            {
                roverRuckusBot.getChassisAssembly().turnLeft(WHEEL_SPEED * -gamepad1.left_stick_x);
            }
            //side right
            else if (gamepad1.right_trigger > 0)
            {
                roverRuckusBot.getChassisAssembly().moveRight(-WHEEL_SPEED * gamepad1.right_trigger);
            }
            //side left
            else if (gamepad1.left_trigger > 0)
            {
                roverRuckusBot.getChassisAssembly().moveLeft(-WHEEL_SPEED * gamepad1.left_trigger);
            }
            //stop moving
            else {
                roverRuckusBot.getChassisAssembly().stopMoving();
            }

            //intake Mineral
            if(gamepad1.b)
            {
                roverRuckusBot.getArmAssembly().intakeMineral(1.0);
            }
            else if(gamepad1.x)
            {
                roverRuckusBot.getArmAssembly().outTakeMineral(1.0);
            }
            else
            {
                roverRuckusBot.getArmAssembly().stopIntake();
            }

            //Sliding Out the Grabber
            if(gamepad1.dpad_up)
            {
                roverRuckusBot.getArmAssembly().extendGrabber(0.8);
            }
            else if(gamepad1.dpad_down)
            {
                roverRuckusBot.getArmAssembly().retractGrabber(0.8);
            }
            else
            {
                roverRuckusBot.getArmAssembly().stopGrabberExtension();
            }

            //Extending the Deposit
            if(gamepad1.dpad_right)
            {
                roverRuckusBot.getArmAssembly().extendDeposit(1);
            }
            else if(gamepad1.dpad_left)
            {
                roverRuckusBot.getArmAssembly().retractDeposit(1);
            }
            else
            {
                roverRuckusBot.getArmAssembly().stopDepositExtension();
            }

            //move lift up for hooking onto lander
            if (gamepad1.left_bumper == true
                    && roverRuckusBot.getLiftAssembly().robotHardware.topTouch.getState() == true
                    )
            {
                telemetry.addData("gamepad1.left_bumper is pressed" , "");
                telemetry.update();
                roverRuckusBot.getLiftAssembly().liftUpRobot(LIFT_UP_SPEED);
            }
            //move lift down for retracting and lifting robot
            else if (gamepad1.right_bumper == true)
            {
                roverRuckusBot.getLiftAssembly().lowerRobot(LIFT_DOWN_SPEED);
            }
            //stop moving the lift
            else
            {
                roverRuckusBot.getLiftAssembly().resetLift();
            }

            if( gamepad1.y == true)
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
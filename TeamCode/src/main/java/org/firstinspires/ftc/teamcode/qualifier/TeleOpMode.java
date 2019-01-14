package org.firstinspires.ftc.teamcode.qualifier;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "TeleOpMode", group = "Qualifier")
public class TeleOpMode extends LinearOpMode {
    private static double WHEEL_SPEED = 1.0;
    private static final double LIFT_UP_SPEED = 0.6;
    private static final double LIFT_DOWN_SPEED = 1.0;

    double power = 0.2;

    //Creating a Rover robot object
    RoverRobot roverRuckusBot = new RoverRobot();

    private ElapsedTime runtime = new ElapsedTime();

    boolean isDelivServoUpPressed  = false;
    boolean flipState              = false;

    @Override
    public void runOpMode() {
        roverRuckusBot.initRobot(hardwareMap);
        waitForStart();

        while (opModeIsActive()) {

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
            boolean intake = gamepad1.x;
            boolean outtake = gamepad1.b;
            boolean wristUp        = gamepad1.y;
            boolean wristDown      = gamepad1.a;

            /**
             * GAME PAD 2
             */
            //Controls
            boolean extendGrabber  = gamepad2.dpad_right;
            boolean retractGrabber = gamepad2.dpad_left;
            boolean extendDeposit  = gamepad2.dpad_up;
            boolean retractDeposit = gamepad2.dpad_down;
            boolean delivServoUp   = gamepad2.x;
            boolean delivServoDown = gamepad2.b;
            boolean flip           = gamepad2.a ;
            boolean wristAutomate  = gamepad2.y;




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
            else if (turn > 0) {
                roverRuckusBot.getChassisAssembly().turnRight(WHEEL_SPEED * turn);
            }
            //turn left
            else if (turn < 0) {
                roverRuckusBot.getChassisAssembly().turnLeft(WHEEL_SPEED * -turn);
            }
            //side right
            else if (sideRight > 0) {
                roverRuckusBot.getChassisAssembly().moveRight(-WHEEL_SPEED * sideRight);
            }
            //side left
            else if (sideLeft > 0) {
                roverRuckusBot.getChassisAssembly().moveLeft(-WHEEL_SPEED * sideLeft);
            }
            //stop moving
            else {
                roverRuckusBot.getChassisAssembly().stopMoving();
            }


            //Robot Lift
            if (liftRobot == true
                    && roverRuckusBot.getLiftAssembly().robotHardware.topTouch.getState() == true
                    ) {
                telemetry.addData("gamepad1.left_bumper is pressed", "");
                telemetry.update();
                roverRuckusBot.getLiftAssembly().liftUpRobot(LIFT_UP_SPEED);
            }
            //move lift down for retracting and lifting robot
            else if (lowerRobot == true) {
                roverRuckusBot.getLiftAssembly().lowerRobot(LIFT_DOWN_SPEED);
            }
            //stop moving the lift
            else {
                roverRuckusBot.getLiftAssembly().resetLift();
            }

            if (intake) {
                roverRuckusBot.getArmAssembly().intakeMineral(1.0);
            } else if (outtake) {
                roverRuckusBot.getArmAssembly().outTakeMineral(1.0);
            } else {
                roverRuckusBot.getArmAssembly().stopIntake();
            }


            //Sliding Out the Grabber
            if (extendGrabber) {
                roverRuckusBot.getArmAssembly().extendGrabber(0.8);
            } else if (retractGrabber) {
                roverRuckusBot.getArmAssembly().retractGrabber(0.8);
            } else {
                roverRuckusBot.getArmAssembly().stopGrabberExtension();
            }

            //Extending the Deposit
            if (extendDeposit) {
                roverRuckusBot.getArmAssembly().extendDeposit(1);
            } else if (retractDeposit) {
                roverRuckusBot.getArmAssembly().retractDeposit(1);
            } else {
                roverRuckusBot.getArmAssembly().stopDepositExtension();
            }
/*
            //Flip
            if (flip && roverRuckusBot.getArmAssembly().robotHardware.deliveryTouch.getState() == true) {
                roverRuckusBot.getArmAssembly().flip(0.7);
                flipState = true;
            } else if (flip && flipState == true) {
                while (roverRuckusBot.getArmAssembly().robotHardware.deliveryTouch.getState() == false) {
                    roverRuckusBot.getArmAssembly().retractDeposit(1.0);
                }
                roverRuckusBot.getArmAssembly().stopGrabberExtension();
                roverRuckusBot.getArmAssembly().flip(0.1);
                flipState = false;
            }

*/

            if (flip) {
                roverRuckusBot.getArmAssembly().flip(0.7);
                flipState = true;
            } else if (flip && flipState == true) {
                while (roverRuckusBot.getArmAssembly().robotHardware.deliveryTouch.getState() == true) {
                    roverRuckusBot.getArmAssembly().retractDeposit(0.2);
                }
                roverRuckusBot.getArmAssembly().flip(0.09);
                flipState = false;
            }
            else
            {
                roverRuckusBot.getArmAssembly().flip(0.09);
            }
            //Deliv Servo
            if (delivServoUp)
            {
                roverRuckusBot.getArmAssembly().deliveryUp();
                isDelivServoUpPressed = true;
            }
            else if (delivServoDown) {
                roverRuckusBot.getArmAssembly().deliverDown();
                isDelivServoUpPressed = false;
            }
            else
            {
                roverRuckusBot.getArmAssembly().deliveryStop();
            }

        /*
            //Intake wrist automation
            if (wristAutomate)
            {
                runtime.reset();
                while (runtime.seconds() < 0.4) {
                    roverRuckusBot.getArmAssembly().moveWrist(-0.8);
                }
                roverRuckusBot.getArmAssembly().moveWrist(0);
                //while (roverRuckusBot.getArmAssembly().robotHardware.backTouch.getState() == false) {
                //    roverRuckusBot.getArmAssembly().retractGrabber(0.8);
                //}
                //roverRuckusBot.getArmAssembly().stopGrabberExtension();
                while (roverRuckusBot.getArmAssembly().robotHardware.wristTouch.getState() == false) {
                    roverRuckusBot.getArmAssembly().moveWrist(-0.8);
                }
                roverRuckusBot.getArmAssembly().moveWrist(0);
                runtime.reset();
                while (runtime.seconds() < 1.5) {
                    roverRuckusBot.getArmAssembly().intakeMineral(1.0);
                }
            }
*/

            if(wristUp)
            {
                roverRuckusBot.getArmAssembly().moveWrist(0.8);
            }
            else if (wristDown)
            {
                roverRuckusBot.getArmAssembly().moveWrist(-0.8);
            }
            else
            {
                roverRuckusBot.getArmAssembly().moveWrist(0);
            }


        }
    }
}
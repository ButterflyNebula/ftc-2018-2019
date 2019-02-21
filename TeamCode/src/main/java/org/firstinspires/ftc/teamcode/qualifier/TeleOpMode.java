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

    private static final double COUNTS_PER_DELIV_INCH = 193.33;


    boolean intakeOn = false;
    //Creating a Rover robot object
    RoverRobot roverRuckusBot = new RoverRobot();

    private ElapsedTime runtime = new ElapsedTime();

    boolean isDelivServoUpPressed  = false;
    double flipPos = 0.09;

    @Override
    public void runOpMode() {
        roverRuckusBot.initRobot(hardwareMap);

        while (!opModeIsActive() && !isStopRequested())
        {
            telemetry.addData("status" , "waiting for start command...");
            telemetry.update();
        }

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
            double manualFlip = gamepad1.right_stick_y;
            boolean terminate = gamepad1.y;


            /**
             * GAME PAD 2
             */
            //Controls
            boolean extendGrabber  = gamepad2.dpad_right;
            boolean retractGrabber = gamepad2.dpad_left;
            boolean extendDeposit  = gamepad2.dpad_up;
            boolean retractDeposit = gamepad2.dpad_down;
            boolean flip           = gamepad2.x;
            boolean wristDown = gamepad2.a;
            boolean wristReturn = gamepad2.y;
            float manualWristDown = gamepad2.right_trigger;
            float manualWristUp = gamepad2.left_trigger;
            boolean depositReturn = gamepad2.b;




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


            if (outtake) {
                roverRuckusBot.getArmAssembly().outTakeMineral(1.0);
            }
            else if (intake || intakeOn) {
                roverRuckusBot.getArmAssembly().intakeMineral(1.0);
            }
             else {
                roverRuckusBot.getArmAssembly().stopIntake();
            }


            //Sliding Out the Grabber
            if (extendGrabber) {
                roverRuckusBot.getArmAssembly().extendGrabber(0.8);
            }
            //Retracting the Grabber (Manually)
            else if (retractGrabber && roverRuckusBot.getArmAssembly().robotHardware.backTouch.getState()) {
                roverRuckusBot.getArmAssembly().retractGrabber(0.8);
            } else {
                roverRuckusBot.getArmAssembly().stopGrabberExtension();
            }

            //Extending the Deposit
            if (extendDeposit)
            {
                intakeOn = false;
                roverRuckusBot.getArmAssembly().stopIntake();

                encoderDelivery(1, 16, "UP", 8);
            }
            //Retracting the Deposit (Manually)
            else if (retractDeposit && roverRuckusBot.getArmAssembly().robotHardware.deliveryTouch.getState()==true)
            //not touching the sensor at the bottom of the lift
            {
                roverRuckusBot.getArmAssembly().flip(0.09);
                roverRuckusBot.getArmAssembly().retractDeposit(1);
            } else {
                roverRuckusBot.getArmAssembly().stopDepositExtension();
            }


            //Retracting the Deposit (Automatically)
            if(depositReturn)
            {
                roverRuckusBot.getArmAssembly().flip(0.09);
                flipPos = 0.09;

                runtime.reset();
                while(opModeIsActive() && runtime.seconds() < 0.5)
                {
                    roverRuckusBot.getArmAssembly().extendGrabber(0.8);
                }
                roverRuckusBot.getArmAssembly().stopGrabberExtension();

                while(opModeIsActive() && roverRuckusBot.getArmAssembly().robotHardware.deliveryTouch.getState() && !terminate)
                {
                    terminate = gamepad1.y;

                    roverRuckusBot.getArmAssembly().retractDeposit(1);
                }
                roverRuckusBot.getArmAssembly().stopDepositExtension();
            }

            //Flipping to Deposit the Minerals
            if (flip) {
                roverRuckusBot.getArmAssembly().flip(0.7);
                flipPos = 0.7;
            }

            if(manualFlip > 0)
            {
                flipPos = flipPos + 0.01;
                roverRuckusBot.getArmAssembly().flip(flipPos);
            }


            //Wrist Controls
            if(wristDown)
            {
                while(opModeIsActive() && roverRuckusBot.getArmAssembly().robotHardware.wristTouchDown.getState() && !terminate)
                {
                    terminate = gamepad1.y;

                    roverRuckusBot.getArmAssembly().moveWrist(-0.8);

                    //Movement
                    drive = gamepad1.left_stick_y;
                    turn = gamepad1.left_stick_x;
                    sideRight = gamepad1.right_trigger;
                    sideLeft = gamepad1.left_trigger;
                    extendGrabber  = gamepad2.dpad_up;
                    retractGrabber = gamepad2.dpad_down;


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

                    //Sliding Out the Grabber
                    if (extendGrabber) {
                        roverRuckusBot.getArmAssembly().extendGrabber(0.8);
                    } else if (retractGrabber) {
                        roverRuckusBot.getArmAssembly().retractGrabber(0.8);
                    } else {
                        roverRuckusBot.getArmAssembly().stopGrabberExtension();
                    }
                }
                roverRuckusBot.getArmAssembly().moveWrist(0);

                intakeOn = true;
            }




            if(wristReturn)
            {

                roverRuckusBot.getChassisAssembly().stopMoving();

                runtime.reset();

                //Move the Wrist Up
                while(opModeIsActive() && runtime.seconds() < 1.2 && !terminate)
                {
                    terminate = gamepad1.y;

                    roverRuckusBot.getArmAssembly().moveWrist(0.8);


                    //MOVEMENT
                     drive = gamepad1.left_stick_y;
                     turn = gamepad1.left_stick_x;
                     sideRight = gamepad1.right_trigger;
                     sideLeft = gamepad1.left_trigger;
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
                }
                intakeOn = false;
                roverRuckusBot.getArmAssembly().stopIntake();
                roverRuckusBot.getArmAssembly().moveWrist(0);

                //Retract the Extension
                while(opModeIsActive() && roverRuckusBot.getArmAssembly().robotHardware.backTouch.getState() && !terminate)
                {
                    terminate = gamepad1.y;

                    roverRuckusBot.getArmAssembly().retractGrabber(0.8);


                    //Movement
                    drive = gamepad1.left_stick_y;
                    turn = gamepad1.left_stick_x;
                    sideRight = gamepad1.right_trigger;
                    sideLeft = gamepad1.left_trigger;
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
                }
                roverRuckusBot.getArmAssembly().stopGrabberExtension();

                //Move the Wrist All the Way UP
                while (opModeIsActive() && roverRuckusBot.getArmAssembly().robotHardware.wristTouch.getState() && !terminate)
                {
                    terminate = gamepad1.y;

                    roverRuckusBot.getArmAssembly().moveWrist(0.8);


                    //Movement
                    drive = gamepad1.left_stick_y;
                    turn = gamepad1.left_stick_x;
                    sideRight = gamepad1.right_trigger;
                    sideLeft = gamepad1.left_trigger;
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
                }
                roverRuckusBot.getArmAssembly().moveWrist(0);


                runtime.reset();
                while(opModeIsActive() && runtime.seconds() < 1.5 && !terminate)
                {
                    terminate = gamepad1.y;

                    roverRuckusBot.getArmAssembly().intakeMineral(1);

                    //Movement
                    drive = gamepad1.left_stick_y;
                    turn = gamepad1.left_stick_x;
                    sideRight = gamepad1.right_trigger;
                    sideLeft = gamepad1.left_trigger;
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
                }
                roverRuckusBot.getArmAssembly().stopIntake();
            }


            //Controlling the Wrist Manually
            if(manualWristDown > 0 && roverRuckusBot.getArmAssembly().robotHardware.wristTouchDown.getState())
            {
                roverRuckusBot.getArmAssembly().moveWrist(-0.8);
            }
            else if(manualWristUp > 0 && roverRuckusBot.getArmAssembly().robotHardware.wristTouch.getState())
            {
                roverRuckusBot.getArmAssembly().moveWrist(0.8);
            }
            else
            {
                roverRuckusBot.getArmAssembly().moveWrist(0);
            }
        }
    }


    /**
     * ENCODER Delivery
     * @param speed (at which to lift)
     * @param inches (to turn a negative number will reverse direction)
     * @param direction (UP or DOWN)
     * @param timeoutS (robot will stop after this much time has passed)
     */
    public void encoderDelivery(double speed, double inches, String direction , double timeoutS)
    {
        int newTarget;


        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            roverRuckusBot.getArmAssembly().changeToEncoderMode();


            // Determine new target position, and pass to motor controller
            if(direction == "UP")
            {
                newTarget = roverRuckusBot.getArmAssembly().getDeliveryLiftCurrentPosition() + (int) (inches * COUNTS_PER_DELIV_INCH);
            }
            else
            {
                newTarget = roverRuckusBot.getArmAssembly().getDeliveryLiftCurrentPosition() + (int) (-inches * COUNTS_PER_DELIV_INCH);

            }

            roverRuckusBot.getArmAssembly().setDeliveryLiftTargetPosition(newTarget);


            // Turn On RUN_TO_POSITION
            roverRuckusBot.getArmAssembly().setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();
            roverRuckusBot.getArmAssembly().extendDeposit(Math.abs(speed));



            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (roverRuckusBot.getArmAssembly().isDeliveryLiftBusy() )) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d", newTarget);
                telemetry.addData("Path2",  "Running at %7d", roverRuckusBot.getArmAssembly().getDeliveryLiftCurrentPosition());

                telemetry.update();
            }

            // Stop all motion;
            roverRuckusBot.getArmAssembly().stopDepositExtension();

            // Turn off RUN_TO_POSITION
            roverRuckusBot.getArmAssembly().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

    }//end of encoderDelivery
}
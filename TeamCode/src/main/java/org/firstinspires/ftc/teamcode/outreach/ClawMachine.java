package org.firstinspires.ftc.teamcode.outreach;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


import static android.R.attr.right;

/**
 * This program is the teleOp Mode
 * This uses the controls in the class called RobotControl
 * Created by Nikhil
 */

@TeleOp (name="ClawMachine", group="PassionProject")
public class ClawMachine extends LinearOpMode {

    static final double GRIPPER_CLOSE = 0.2;
    static final double GRIPPER_OPEN = 0;
    static final double BASE_MOTOR_SPEED = 0.25;
    static final double SECONDARY_MOTOR_SPEED = 0.3;
    static final double DROP_SPEED = 0.12;
    static final double LIFT_SPEED = .2;
    private ElapsedTime runtime = new ElapsedTime();


    /* Define the robot hardware */
    RobotHardware robot = new RobotHardware();   // Use my RobotHardware

    public void runOpMode() {
        robot.init(hardwareMap);

        //Adding the robotControls
        RobotControl robotControl = new RobotControl(hardwareMap, robot);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();

        Log.d("DriverControlledMode", "Hello Driver");

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        while (opModeIsActive()) {

            /**
             * Variables
             */
            //Movement variables
            double base;
            double side;
            boolean down;
            boolean up;

            //gripper Variables
            boolean close;
            boolean open;


            /**
             * Setting the gamepad buttons
             */
            //movement
            base = gamepad1.right_stick_x;
            side = gamepad1.left_stick_y;
            down = gamepad1.dpad_down;
            close = gamepad1.x;


            telemetry.addData("Base value", base);
            telemetry.update();

            if(robot.sensorBack.getState()== false)
            {
                telemetry.addData("status", "sensorBack is pressed");
                telemetry.update();
            }
            if(robot.sensorFront.getState()== false)
            {
                telemetry.addData("status", "sensorFront is pressed");
                telemetry.update();
            }

            /**
             * Base Motor Controls
             */

            //No movement
            if (base == 0 )
            {
                robotControl.baseStop();
            }
            // Forward
            else if (base > 0 && robot.sensorBack.getState() == true )
            {
                robotControl.moveForward(BASE_MOTOR_SPEED);
            }
            //Backwards
            else if (base < 0 && robot.sensorFront.getState() == true)
            {
                robotControl.moveBackwards(BASE_MOTOR_SPEED);
            }
            else
            {
                robotControl.baseStop();
                telemetry.addData("Status", "In Base Else");
                telemetry.update();
            }
            if (side == 0)
            {
                robotControl.sideStop();
            }
            else if (side < 0 && robot.sensorLeft.getState() == true)
            {
                robotControl.moveLeft(SECONDARY_MOTOR_SPEED);
            }
            else if (side > 0 && robot.sensorRight.getState() == true)
            {
                robotControl.moveRight(SECONDARY_MOTOR_SPEED);
            }
            else
            {
                 robotControl.sideStop();
            }

            /**
             * Dropping Control
             */
            if (down == true)
            {
                robotControl.openHand();
                robotControl.drop(DROP_SPEED);
            }
            else
            {
                robotControl.dropStop();
            }

            // grabbing controls
            if (close == true)
            {
                robotControl.closeHand();
                sleep(2000);
                runtime.reset();

                while (runtime.seconds() < 2)
                {
                    robotControl.lift(LIFT_SPEED);
                }


                robotControl.dropStop();

                runtime.reset();
                 while (robot.sensorFront.getState() == true)
                {
                    robotControl.moveBackwards(BASE_MOTOR_SPEED);
                }

                    robotControl.baseStop();

                while (robot.sensorRight.getState() == true)
                {
                    robotControl.moveRight(SECONDARY_MOTOR_SPEED);
                }

                    robotControl.sideStop();


                    robotControl.openHand();
            }
        }
    }
}
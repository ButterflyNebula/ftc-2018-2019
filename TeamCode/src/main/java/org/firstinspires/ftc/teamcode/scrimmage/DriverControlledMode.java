package org.firstinspires.ftc.teamcode.scrimmage;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="DriverControlledMode", group ="Scrimmage")
@Disabled
public class DriverControlledMode extends LinearOpMode {

    private RoverRobot roverRuckusBot = new RoverRobot();
    private ElapsedTime runtime = new ElapsedTime();

    private static       double     WHEEL_SPEED       = 1.0;
    private static final double     LIFT_UP_SPEED     = 0.6;
    private static final double     LIFT_DOWN_SPEED   = 1.0;
    private static       double     INTAKE_POWER      = 0.6;


    @Override public void runOpMode() {

        roverRuckusBot.initRobot(hardwareMap);


        telemetry.addData("Press Play to Start", "");
        telemetry.update();
        waitForStart();

        while (opModeIsActive())
        {
            //GAMEPAD 1 CONTROLS

            //forwards
            if (gamepad1.left_stick_y > 0) {
                roverRuckusBot.getChassisAssembly().moveForward(WHEEL_SPEED * gamepad1.left_stick_y);
            }
            //backwards
            else if (gamepad1.left_stick_y < 0) {
                roverRuckusBot.getChassisAssembly().moveBackwards(WHEEL_SPEED * -gamepad1.left_stick_y);
            }
            //turn right
            else if (gamepad1.left_stick_x > 0) {
                roverRuckusBot.getChassisAssembly().turnRight(WHEEL_SPEED * gamepad1.left_stick_x);
            }
            //turn left
            else if (gamepad1.left_stick_x < 0) {
                roverRuckusBot.getChassisAssembly().turnLeft(WHEEL_SPEED * -gamepad1.left_stick_x);
            }
            //side right
            else if (gamepad1.right_trigger > 0) {
                roverRuckusBot.getChassisAssembly().moveRight(WHEEL_SPEED * gamepad1.right_trigger);
            }
            //side left
            else if (gamepad1.left_trigger > 0) {
                roverRuckusBot.getChassisAssembly().moveLeft(WHEEL_SPEED * gamepad1.left_trigger);
            }
            //stop moving
            else
            {
                roverRuckusBot.getChassisAssembly().stopMoving();
            }


            //move lift up for hooking onto lander
            if (gamepad1.dpad_up == true
                    && roverRuckusBot.getLiftAssembly().robotHardware.topTouch.getState() == true)
            {
                    roverRuckusBot.getLiftAssembly().liftUpRobot(LIFT_UP_SPEED);
            }
            //move lift down for retracting and lifting robot
            else if (gamepad1.dpad_down == true)
            {
                while(roverRuckusBot.getLiftAssembly().robotHardware.bottomTouch.getState() == true)
                {
                    roverRuckusBot.getLiftAssembly().lowerRobot(LIFT_DOWN_SPEED);
                }
                roverRuckusBot.getLiftAssembly().lockRobot();
                roverRuckusBot.getLiftAssembly().resetLift();
            }
            //stop moving the lift
            else
            {
                roverRuckusBot.getLiftAssembly().resetLift();
            }


            //unlocks the robot lift
            if (gamepad1.b == true)
            {
                roverRuckusBot.getLiftAssembly().unlockRobot();

            }
            //locks the robot lift
            if (gamepad1.x == true)
            {
                roverRuckusBot.getLiftAssembly().lockRobot();
            }


            //GAMEPAD 2 CONTROLS

            //moves mineral lift up
            if (gamepad2.dpad_up == true)
            {
                roverRuckusBot.getArmAssembly().mineralDelivery(0.5);
            }
            //moves mineral lift down
            else if (gamepad2.dpad_down == true)
            {
                roverRuckusBot.getArmAssembly().mineralDelivery(-0.3);
            }
            //stops the mineral lift
            else
            {
                roverRuckusBot.getArmAssembly().mineralDelivery(0);
            }

            //puts minerals into lander
            if (gamepad2.x == true)
            {
                roverRuckusBot.getArmAssembly().mineralFlip(1);
            }
            //returns to the initial position
            else if (gamepad2.b == true)
            {
                roverRuckusBot.getArmAssembly().mineralFlip(0);
            }

            // runs intake wheels inward
            if (gamepad2.a == true)
            {
                while (INTAKE_POWER < 1)
                {
                    roverRuckusBot.getArmAssembly().Intake(-INTAKE_POWER);
                    INTAKE_POWER = INTAKE_POWER + 0.1;
                }
                while (INTAKE_POWER > 0.7) {
                    roverRuckusBot.getArmAssembly().Intake(-INTAKE_POWER);
                    INTAKE_POWER = INTAKE_POWER - 0.1;
                }
                INTAKE_POWER = 0.7;
            }
            //stops the intake wheels
            else {
                roverRuckusBot.getArmAssembly().Intake(0);
            }

            if (gamepad2.y == true)
            {
                while (INTAKE_POWER < 1)
                {
                    roverRuckusBot.getArmAssembly().Outake(-INTAKE_POWER);
                    INTAKE_POWER = INTAKE_POWER + 0.1;
                }
                while (INTAKE_POWER > 0.7) {
                    roverRuckusBot.getArmAssembly().Outake(-INTAKE_POWER);
                    INTAKE_POWER = INTAKE_POWER - 0.1;
                }
                INTAKE_POWER = 0.7;
            }
            //stops the intake wheels
            else
                {
                roverRuckusBot.getArmAssembly().Outake(0);
            }

            //moves the DC motor of the arm downwards
            if (gamepad2.left_stick_y < 0)
            {
                roverRuckusBot.getArmAssembly().armReturn(0.5 * -gamepad2.left_stick_y);
            }
            //moves the DC motor of the arm upwards
            else if (gamepad2.left_stick_y > 0)
            {
                roverRuckusBot.getArmAssembly().armExtend(0.5 * gamepad2.left_stick_y);
            }
            //stops the DC motor of the arm
            else
            {
                roverRuckusBot.getArmAssembly().armExtend(0);
            }
            //moves the DC motor of the wrist
            if (gamepad2.right_stick_y != 0){
                roverRuckusBot.getArmAssembly().wristExtend(-gamepad2.right_stick_y*0.3);
            }
            //stops the DC motor of the wrist
            else {
                roverRuckusBot.getArmAssembly().wristExtend(0);
            }
        }
    }
}
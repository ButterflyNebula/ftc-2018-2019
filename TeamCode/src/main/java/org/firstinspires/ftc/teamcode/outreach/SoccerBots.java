package org.firstinspires.ftc.teamcode.outreach;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="SoccerBots", group="Outreach")
public class SoccerBots extends OpMode
{
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private Servo kickServo = null;

    double speed = 0.5;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        leftDrive = hardwareMap.get(DcMotor.class, "leftDrive");
        rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");
        kickServo = hardwareMap.get(Servo.class, "kickServo");

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {

        telemetry.addData("x", gamepad1.left_stick_x);
        telemetry.addData("y", gamepad1.left_stick_y);

        double position = kickServo.getPosition();

        if(gamepad1.left_stick_y > 0)
        {
            rightDrive.setPower(gamepad1.left_stick_y * speed);
            leftDrive.setPower(gamepad1.left_stick_y * -speed);
        }

        else if (gamepad1.left_stick_y < 0)
        {
            rightDrive.setPower(-gamepad1.left_stick_y * -speed);
            leftDrive.setPower(-gamepad1.left_stick_y * speed);
        }

        else if(gamepad1.right_stick_x > 0) {
            rightDrive.setPower(gamepad1.right_stick_x * speed);
            leftDrive.setPower(gamepad1.right_stick_x * speed);
        }

        else if(gamepad1.right_stick_x<0){
                rightDrive.setPower(-gamepad1.right_stick_x * -speed);
                leftDrive.setPower(-gamepad1.right_stick_x * -speed);
            }

            else{
                rightDrive.setPower(0);
                leftDrive.setPower(0);
            }

        if (gamepad1.a == true)
        {
            kickServo.setPosition(0.5);
            position=kickServo.getPosition();
        }

        else if (gamepad1.b == true)
        {
            kickServo.setPosition(0);
            position=kickServo.getPosition();
        }

        if (gamepad1.a !=true && gamepad1.b!=true) {
            double currentPosition = kickServo.getPosition();
            kickServo.setPosition(currentPosition);
        }
    }

    @Override
    public void stop() {
    }
}

package org.firstinspires.ftc.teamcode.scrimmage;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp (name = "Mineral Lift Test")
@Disabled
public class MineralLiftTest extends LinearOpMode {
    private RoverRobot roverRuckusBot = new RoverRobot();

    @Override
    public void runOpMode() {
        roverRuckusBot.initRobot(hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.dpad_right == true)
            {
                roverRuckusBot.getArmAssembly().mineralDelivery(0.3);
            }
            else if (gamepad1.dpad_left == true)
            {
                roverRuckusBot.getArmAssembly().mineralDelivery(-0.3);
            }
            else
            {
                roverRuckusBot.getArmAssembly().mineralDelivery(0);
            }

            if (gamepad1.x == true)
            {
                roverRuckusBot.getArmAssembly().mineralFlip(1);
            }
            else if (gamepad1.b == true)
            {
                roverRuckusBot.getArmAssembly().mineralFlip(0);
            }
        }
    }
}
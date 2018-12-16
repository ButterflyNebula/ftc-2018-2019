package org.firstinspires.ftc.teamcode.qualifier;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name = "TestArm" , group = "Qualifier")
public class TestArm extends LinearOpMode
{
    //Creating a Rover robot object
    RoverRobot robot = new RoverRobot();


    @Override public void runOpMode()
    {
        robot.initRobot(hardwareMap);

        waitForStart();

        while (opModeIsActive())
        {
            double grabberExtendSpeed = gamepad1.left_stick_y;
            double depositExtendSpeed = gamepad1.right_stick_y;

            if(grabberExtendSpeed > 0)
            {
                robot.getArmAssembly().extendGrabber(grabberExtendSpeed);
            }
            else if(grabberExtendSpeed < 0)
            {
                robot.getArmAssembly().retractGrabber(-grabberExtendSpeed);
            }
            else
            {
                robot.getArmAssembly().stopGrabberExtension();
            }

            if(depositExtendSpeed > 0)
            {
                robot.getArmAssembly().extendDeposit(depositExtendSpeed);
            }
            else if(depositExtendSpeed < 0)
            {
                robot.getArmAssembly().retractDeposit(-depositExtendSpeed);
            }
            else
            {
                robot.getArmAssembly().stopDepositExtension();
            }
        }
    }
}


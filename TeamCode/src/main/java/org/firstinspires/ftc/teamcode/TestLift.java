/* Test Lift Program: Involves simply turning one DC Motot
 */
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp(name="TestNavigation", group ="Tests")
public class TestLift extends LinearOpMode {

    //Declare Hardware
    private DcMotor motor = null;



    @Override public void runOpMode() {

        //Add harware to hardwareMap and set motor direction
        motor  = hardwareMap.get(DcMotor.class, "motor");

        motor.setDirection(DcMotor.Direction.FORWARD);


        waitForStart();

        telemetry.addData("Press Play to Start" , "");

        telemetry.update();

        while(opModeIsActive())
        {

            if(gamepad1.dpad_up == true)
            {
                motor.setPower(0.3);
            }
            else if(gamepad1.dpad_down == true)
            {
                motor.setPower(-0.3);
            }
            else
            {
                motor.setPower(0.0);
            }

            if(gamepad1.a == true)
            {
                motor.setPower(0.0);
            }
        }
    }
}

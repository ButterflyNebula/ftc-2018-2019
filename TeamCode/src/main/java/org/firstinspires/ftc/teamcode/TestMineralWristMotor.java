/* Test Lift Program: Involves simply turning one DC Motot
 */
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="TestMineralWristMotor", group ="Tests")
@Disabled
public class TestMineralWristMotor extends LinearOpMode {

    //Declare Hardware
    private DcMotor motor = null;
    private ElapsedTime runtime = new ElapsedTime();


    @Override public void runOpMode() {

        //Add harware to hardwareMap and set motor direction
        motor  = hardwareMap.get(DcMotor.class, "mineralWrist");

        motor.setDirection(DcMotor.Direction.FORWARD);


        waitForStart();

        telemetry.addData("Press Play to Start" , "");

        telemetry.update();

        while(opModeIsActive())
        {
          while(runtime.seconds() <3) {
              motor.setPower(0.3);
          }
          break;

        }
    }
}

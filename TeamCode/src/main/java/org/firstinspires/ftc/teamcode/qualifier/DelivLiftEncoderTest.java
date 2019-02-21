package org.firstinspires.ftc.teamcode.qualifier;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.qualifier.RoverRobot;

@TeleOp(name="DelivEncoder", group ="Tests")
public class DelivLiftEncoderTest extends LinearOpMode
{
    final double COUNTS_PER_DELIV_INCH = 193.33;

    private RoverRobot robot = new RoverRobot();
    ElapsedTime runtime = new ElapsedTime();

    @Override public void runOpMode()
    {

        robot.initRobot(hardwareMap);


        telemetry.addData("Press Play to Start", "");
        telemetry.update();

        while (!opModeIsActive() && !isStopRequested())
        {
            telemetry.addData("status" , "waiting for start command...");
            telemetry.update();
        }

        double initPos = robot.getArmAssembly().getDeliveryLiftCurrentPosition();
        telemetry.addData("Init ", initPos);
        telemetry.update();

        while (opModeIsActive())
        {
            boolean drive = gamepad1.x;
            boolean showTelemetry = gamepad1.b;
            boolean automate = gamepad1.y;

            if(drive)
            {
                robot.getArmAssembly().extendDeposit(0.3);
                double currentPos = robot.getArmAssembly().getDeliveryLiftCurrentPosition();

                telemetry.addData("Change" ,currentPos-initPos);

                telemetry.update();
            }
            else
            {
                robot.getArmAssembly().stopDepositExtension();
            }

            if(showTelemetry)
            {
                double currentPos = robot.getArmAssembly().getDeliveryLiftCurrentPosition();

                telemetry.addData("Change" ,currentPos-initPos);

                telemetry.update();
            }

            if(automate)
            {
                encoderDelivery(0.3, 5, "UP", 5);

                sleep(5000);

                encoderDelivery(0.3, 3, "DOWN", 5);
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

            robot.getArmAssembly().changeToEncoderMode();


            // Determine new target position, and pass to motor controller
            if(direction == "UP")
            {
                newTarget = robot.getArmAssembly().getDeliveryLiftCurrentPosition() + (int) (inches * COUNTS_PER_DELIV_INCH);
            }
            else
            {
                newTarget = robot.getArmAssembly().getDeliveryLiftCurrentPosition() + (int) (-inches * COUNTS_PER_DELIV_INCH);

            }

            robot.getArmAssembly().setDeliveryLiftTargetPosition(newTarget);


            // Turn On RUN_TO_POSITION
            robot.getArmAssembly().setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();
            robot.getArmAssembly().extendDeposit(Math.abs(speed));



            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.getArmAssembly().isDeliveryLiftBusy() )) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d", newTarget);
                telemetry.addData("Path2",  "Running at %7d", robot.getArmAssembly().getDeliveryLiftCurrentPosition());

                telemetry.update();
            }

            // Stop all motion;
            robot.getArmAssembly().stopDepositExtension();

            // Turn off RUN_TO_POSITION
            robot.getArmAssembly().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

    }//end of encoderDelivery
}



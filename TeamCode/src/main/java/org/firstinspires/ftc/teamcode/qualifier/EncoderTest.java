package org.firstinspires.ftc.teamcode.qualifier;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.qualifier.RoverRobot;

@TeleOp(name="EncoderTest", group ="Tests")
public class EncoderTest extends LinearOpMode
{

    final double COUNTS_PER_MOTOR_REV = 1120;
    final double DRIVE_GEAR_REDUCTION = 1.0;
    final double WHEEL_DIAMETER_INCHES = 4.0;
       final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
       (WHEEL_DIAMETER_INCHES * 3.1415);
    final double TURN_DIAMETER_INCHES = 22.3;

    final double DEGREES_PER_MOTOR_REV = (360 * (WHEEL_DIAMETER_INCHES)) / TURN_DIAMETER_INCHES;
    final double COUNTS_PER_DEGREE = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (DEGREES_PER_MOTOR_REV);

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

        double fLInit = robot.getChassisAssembly().getFrontLeftWheelCurrentPosition();
        double fRInit = robot.getChassisAssembly().getFrontRightWheelCurrentPosition();
        double bLInit = robot.getChassisAssembly().getBackLeftWheelCurrentPosition();
        double bRInit = robot.getChassisAssembly().getBackRightWheelCurrentPosition();


        while (opModeIsActive())
        {
            boolean drive = gamepad1.x;
            boolean showTelemetry = gamepad1.b;
            boolean automate = gamepad1.y;

            if(drive)
            {
                robot.getChassisAssembly().turnRight(0.5);
                double fL = robot.getChassisAssembly().getFrontLeftWheelCurrentPosition();
                double fR = robot.getChassisAssembly().getFrontRightWheelCurrentPosition();
                double bL = robot.getChassisAssembly().getBackLeftWheelCurrentPosition();
                double bR = robot.getChassisAssembly().getBackRightWheelCurrentPosition();

                telemetry.addData("FL" ,fL-fLInit);
                telemetry.addData("FR" , fR-fRInit);
                telemetry.addData("BL" , bL-bLInit);
                telemetry.addData("BR" , bR - bRInit);
                double avg = (Math.abs(fL - fLInit)+Math.abs(fR - fRInit) + Math.abs(bL - bLInit) + Math.abs(bR - bRInit))/4;
                telemetry.addData("Avg", avg);
                telemetry.addData("Counts per degree" , avg/90);
                telemetry.update();
            }
            else
            {
                robot.getChassisAssembly().stopMoving();
            }

            if(showTelemetry)
            {
                double fL = robot.getChassisAssembly().getFrontLeftWheelCurrentPosition();
                double fR = robot.getChassisAssembly().getFrontRightWheelCurrentPosition();
                double bL = robot.getChassisAssembly().getBackLeftWheelCurrentPosition();
                double bR = robot.getChassisAssembly().getBackRightWheelCurrentPosition();

                telemetry.addData("FL" ,fL-fLInit);
                telemetry.addData("FR" , fR-fRInit);
                telemetry.addData("BL" , bL-bLInit);
                telemetry.addData("BR" , bR - bRInit);
                double avg = (Math.abs(fL - fLInit)+Math.abs(fR - fRInit) + Math.abs(bL - bLInit) + Math.abs(bR - bRInit))/4;
                telemetry.addData("Avg", avg);
                telemetry.addData("Counts per degree" , avg/90);
                telemetry.update();
            }

            if(automate)
            {
                encoderTurn(0.5 , 90, "RIGHT" , 5);

                sleep(5000);

                encoderTurn(0.5 , 90 , "LEFT" , 7);
            }
        }

    }

    /**
     * ENCODER TURN
     * @param speed (at which to turn)
     * @param degrees (to turn a negative number will reverse direction)
     * @param direction (LEFT or RIGHT)
     * @param timeoutS (robot will stop after this much time has passed)
     */
    public void encoderTurn(double speed, double degrees, String direction , double timeoutS)
    {
        int newBackLeftTarget;
        int newBackRightTarget;
        int newFrontLeftTarget;
        int newFrontRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            robot.getChassisAssembly().changeToEncoderMode();


            // Determine new target position, and pass to motor controller
            if(direction == "LEFT")
            {
                newBackLeftTarget = robot.getChassisAssembly().getBackLeftWheelCurrentPosition() + (int) (-degrees * COUNTS_PER_DEGREE);
                newBackRightTarget = robot.getChassisAssembly().getBackRightWheelCurrentPosition() + (int) (degrees * COUNTS_PER_DEGREE);
                newFrontLeftTarget = robot.getChassisAssembly().getFrontLeftWheelCurrentPosition() + (int) (-degrees * COUNTS_PER_DEGREE);
                newFrontRightTarget = robot.getChassisAssembly().getFrontRightWheelCurrentPosition() + (int) (degrees * COUNTS_PER_DEGREE);
            }
            else
            {
                newBackLeftTarget = robot.getChassisAssembly().getBackLeftWheelCurrentPosition() + (int) (degrees * COUNTS_PER_DEGREE);
                newBackRightTarget = robot.getChassisAssembly().getBackRightWheelCurrentPosition() + (int) (-degrees * COUNTS_PER_DEGREE);
                newFrontLeftTarget = robot.getChassisAssembly().getFrontLeftWheelCurrentPosition() + (int) (degrees * COUNTS_PER_DEGREE);
                newFrontRightTarget = robot.getChassisAssembly().getFrontRightWheelCurrentPosition() + (int) (-degrees * COUNTS_PER_DEGREE);
            }

            robot.getChassisAssembly().setBackLeftWheelTargetPosition(newBackLeftTarget);
            robot.getChassisAssembly().setBackRightWheelTargetPosition(newBackRightTarget);
            robot.getChassisAssembly().setFrontLeftWheelTargetPosition(newFrontLeftTarget);
            robot.getChassisAssembly().setFrontRightWeelTargetPosition(newFrontRightTarget);

            // Turn On RUN_TO_POSITION
            robot.getChassisAssembly().setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();
            robot.getChassisAssembly().setBackLeftWheelPower(Math.abs(speed));
            robot.getChassisAssembly().setBackRightWheelPower(Math.abs(speed));
            robot.getChassisAssembly().setFrontLeftWheelPower(Math.abs(speed));
            robot.getChassisAssembly().setFrontRightWheelPower(Math.abs(speed));


            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.getChassisAssembly().isBackLeftWheelBusy() && robot.getChassisAssembly().isBackRightWheelBusy() &&
                            robot.getChassisAssembly().isFrontLeftWheelBusy() && robot.getChassisAssembly().isFrontRightWheelBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d : %7d :%7d",
                        newBackLeftTarget,  newBackRightTarget, newFrontLeftTarget, newFrontRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d : %7d : %7d",
                        robot.getChassisAssembly().getBackLeftWheelCurrentPosition(),
                        robot.getChassisAssembly().getBackRightWheelCurrentPosition(),
                        robot.getChassisAssembly().getFrontLeftWheelCurrentPosition(),
                        robot.getChassisAssembly().getFrontRightWheelCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.getChassisAssembly().stopMoving();

            // Turn off RUN_TO_POSITION
            robot.getChassisAssembly().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

    }//end of encoderTurn
}



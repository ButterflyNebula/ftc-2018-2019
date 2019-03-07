package org.firstinspires.ftc.teamcode.qualifier;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;

@TeleOp(name="EncoderGyroTest", group ="Tests")
public class EncoderGyroTest extends LinearOpMode
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

    //----------------------------------------------------------------------------------------------
    // State
    //----------------------------------------------------------------------------------------------

    // The IMU sensor object
    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;

    @Override public void runOpMode()
    {

        robot.initRobot(hardwareMap);

        imu = robot.getChassisAssembly().robotHardware.imu;
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

        // Set up our telemetry dashboard
        composeTelemetry();
        telemetry.update();
        // Start the logging of measured acceleration
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);


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

                telemetry.addLine()
                        .addData("heading", new Func<String>() {
                            @Override public String value() {
                                return formatAngle(angles.angleUnit, angles.firstAngle);
                            }
                        })
                        .addData("roll", new Func<String>() {
                            @Override public String value() {
                                return formatAngle(angles.angleUnit, angles.secondAngle);
                            }
                        })
                        .addData("pitch", new Func<String>() {
                            @Override public String value() {
                                return formatAngle(angles.angleUnit, angles.thirdAngle);
                            }
                        });
                telemetry.update();
            }

            if(automate)
            {
                encoderTurn(0.5 , 90, "RIGHT" , 5);

                sleep(5000);

                //encoderTurn(0.5 , 90 , "LEFT" , 7);
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


    //----------------------------------------------------------------------------------------------
    // Telemetry Configuration
    //----------------------------------------------------------------------------------------------

    void composeTelemetry() {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() { @Override public void run()
        {
            // Acquiring the angles is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the necessary expense.
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            gravity  = imu.getGravity();
        }
        });

        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override public String value() {
                        return imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override public String value() {
                        return imu.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });

        telemetry.addLine()
                .addData("grvty", new Func<String>() {
                    @Override public String value() {
                        return gravity.toString();
                    }
                })
                .addData("mag", new Func<String>() {
                    @Override public String value() {
                        return String.format(Locale.getDefault(), "%.3f",
                                Math.sqrt(gravity.xAccel*gravity.xAccel
                                        + gravity.yAccel*gravity.yAccel
                                        + gravity.zAccel*gravity.zAccel));
                    }
                });
    }

    //----------------------------------------------------------------------------------------------
    // Formatting
    //----------------------------------------------------------------------------------------------

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

}



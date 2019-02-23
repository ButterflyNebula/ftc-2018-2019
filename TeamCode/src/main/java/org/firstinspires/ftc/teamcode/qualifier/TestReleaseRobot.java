package org.firstinspires.ftc.teamcode.qualifier;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="ReleaseRobot", group ="Test")
public class TestReleaseRobot extends LinearOpMode
{

    //Creating a Rover robot object
    RoverRobot robot = new RoverRobot();

    private ElapsedTime runtime = new ElapsedTime();


    //Encoder Constants
    private static final double COUNTS_PER_MOTOR_REV = 1120;
    private static final double DRIVE_GEAR_REDUCTION = 1.0;
    private static final double WHEEL_DIAMETER_INCHES = 4.0;
    private static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);
    private static final double TURN_DIAMETER_INCHES = 22.3;

    private static final double DEGREES_PER_MOTOR_REV = (360 * (WHEEL_DIAMETER_INCHES)) / TURN_DIAMETER_INCHES;
    private static final double COUNTS_PER_DEGREE = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (DEGREES_PER_MOTOR_REV);
    private static final double COUNTS_PER_SIDE_INCH = 100;

    private static final double WHEEL_SPEED = 1;


    @Override
    public void runOpMode()
    {
        robot.initRobot(hardwareMap);

        //Wait for Start
        telemetry.addData(">", "Press Play to begin");
        telemetry.update();
        while (!opModeIsActive() && !isStopRequested())
        {
            telemetry.addData("status", "waiting for start command...");
            telemetry.update();
        }

        releaseRobot();


    }


    /**
     * RELEASE ROBOT METHOD
     */
    public void releaseRobot()
    {


        while (robot.getLiftAssembly().robotHardware.topTouch.getState() == true)
        {
            robot.getLiftAssembly().liftUpRobot(0.5);
            telemetry.addData("in  while loop", "still going on");
            telemetry.update();
        }
        robot.getLiftAssembly().resetLift();
        telemetry.addData("outside while loop: ", "reached the top");
        telemetry.update();

        runtime.reset();
        encoderDrive(1, -5, 4);
        telemetry.addData("in stop moving", "stopped");
        telemetry.update();

    }

    /**
     * ENCODER DRIVE METHOD
     *
     * @param speed    (at which the robot should move)
     * @param inches   (positive is forward, negative is backwards)
     * @param timeoutS (the robot will stop moving if it after this many seconds)
     */
    public void encoderDrive(double speed, double inches, double timeoutS)
    {
        int newBackLeftTarget;
        int newBackRightTarget;
        int newFrontLeftTarget;
        int newFrontRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive())
        {


            // Determine new target position, and pass to motor controller
            newBackLeftTarget = robot.getChassisAssembly().getBackLeftWheelCurrentPosition() - (int) (inches * COUNTS_PER_INCH);
            newBackRightTarget = robot.getChassisAssembly().getBackRightWheelCurrentPosition() - (int) (inches * COUNTS_PER_INCH);
            newFrontLeftTarget = robot.getChassisAssembly().getFrontLeftWheelCurrentPosition() - (int) (inches * COUNTS_PER_INCH);
            newFrontRightTarget = robot.getChassisAssembly().getFrontRightWheelCurrentPosition() - (int) (inches * COUNTS_PER_INCH);


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
                            robot.getChassisAssembly().isFrontLeftWheelBusy() && robot.getChassisAssembly().isFrontRightWheelBusy()))
            {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d : %7d :%7d",
                        newBackLeftTarget, newBackRightTarget, newFrontLeftTarget, newFrontRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d : %7d : %7d",
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

    }//end of encoderDrive

}


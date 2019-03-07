package org.firstinspires.ftc.teamcode.qualifier;


import android.util.Log;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldDetector;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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

import static java.lang.Math.abs;

@Autonomous(name="Depot2", group ="Qualifier")
@Disabled
public class DepotStateChamp extends LinearOpMode
{
    //Creating a Rover robot object
    RoverRobot robot = new RoverRobot();

    // The IMU sensor object
    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;

    private GoldDetector detector;

    private ElapsedTime runtime = new ElapsedTime();

    //Motion Variables
    int goldLoc = 0;
    double forwardDistance = 28;
    double hitGoldDistance = 11;
    double distanceToCrater = -72;
    double robotAngle = 0;

    //heading angle tracking values
    double initialPos = 0;
    double newStartPos = 0;

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
    private static final double SIDE_WHEEL_SPEED = 0.8;


    @Override
    public void runOpMode()
    {
        robot.initRobot(hardwareMap);
        // Set up detector
        detector = new GoldDetector(); // Create detector
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance(), 1, false); // Initialize it with the app context and camera
        detector.useDefaults(); // Set detector to use default settings

        // Optional tuning
        detector.downscale = 0.4; // How much to downscale the input frames

        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.maxAreaScorer.weight = 0.005; //

        detector.ratioScorer.weight = 5; //
        detector.ratioScorer.perfectRatio = 1.0; // Ratio adjustment

        detector.enable(); // Start the detector!

        //Wait for Start
        telemetry.addData(">", "Press Play to begin");
        telemetry.update();
        while (!opModeIsActive() && !isStopRequested())
        {
            telemetry.addData("status" , "waiting for start command...");
            telemetry.update();
        }
        // Set up our telemetry dashboard
        composeTelemetry();
        telemetry.update();
        // Start the logging of measured acceleration
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        robot.getChassisAssembly().changeToEncoderMode();

        //   releaseRobot();
        do
        {
            if(angles !=null) //completed integration - angles updated
            {
                initialPos = angles.firstAngle;
                telemetry.addData("initialPos = " ,  initialPos);
                telemetry.update();
                break;
            }
            else  //initialPos remains as 0
            {
                sleep(50);
            }
        }while (runtime.seconds() < 2);  // wait until imu completes AccelerationIntegration
        Log.d("CraterStateChamp", "initialPos" + initialPos);

        moveFromLander();
        sidewaysAlign();
        hitGold();
        sidewaysAlign();


        sleep (10000);
        //placeMarker();


        detector.disable();

    }


    /**
     * RELEASE ROBOT METHOD
     */
    public void releaseRobot() {


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
        encoderDrive(1 , -5, 4);
        telemetry.addData("in stop moving","stopped");
        telemetry.update();

    }

    /**
     * MOVE FROM LANDER METHOD
     */
    private void moveFromLander()
    {
        encoderSide(WHEEL_SPEED , 15 , "LEFT" , 6);
        encoderDrive(WHEEL_SPEED , 6 , 5);

    }//end of moveFromLander


    /**
     * Sideways align
     */
    private void sidewaysAlign()
    {
        try
        {
            double newPos =  angles.firstAngle;
            telemetry.addData("newPos = " ,  newPos);
            telemetry.update();
            Log.d("CraterStateChamp", "newPos = " +  newPos);

            double angleDiff =  newStartPos- newPos;
            telemetry.addData("angleDiff = " ,  angleDiff);
            telemetry.update();
            Log.d("CraterStateChamp", "angleDiff = " +  angleDiff);
            if(angleDiff > 0){
                telemetry.addData("turning right = " ,  "");
                telemetry.update();
                encoderTurn(SIDE_WHEEL_SPEED,abs(angleDiff), "LEFT",5);
            }
            else if( angleDiff < 0)
            {
                telemetry.addData("turning left = " ,  "");
                telemetry.update();
                encoderTurn(SIDE_WHEEL_SPEED,abs(angleDiff), "RIGHT",5);
            }
            newStartPos =  angles.firstAngle;
            telemetry.addData("newStartPos = " ,  newStartPos);
            telemetry.update();
            Log.d("CraterStateChamp", "newStartPos = " +  newStartPos);
        }
        catch (Exception ex)
        {
            telemetry.addData("imu not available for sideways correction " ,  "");
            telemetry.update();
            Log.e("CraterStateChamp", "imu not available for sideways correction ");
        }


    }//end of sidewaysAlign


    /**
     * HIT GOLD METHOD
     */
    private void hitGold()
    {
        double leftAngle = 55;
        double rightAngle = 110;

        boolean goldFound = false;

        while(goldFound == false)
        {
            runtime.reset();
            while(runtime.seconds() < 0.5 && goldFound == false)
            {
                goldFound = detector.isFound();
            }

            if(goldLoc == -1)
            {
                goldFound = true;
            }
            if(goldFound == true)
            {
                telemetry.addData("Gold Found, Position is" , goldLoc);
                telemetry.addData("Gold Loc: " , goldLoc);
                telemetry.addData("Moving Forward" , "");
                telemetry.update();
                Log.d("CraterStateChamp", "goldFound = true");
                encoderSide(SIDE_WHEEL_SPEED , hitGoldDistance , "LEFT" , 6);
            }
            else
            {
                Log.d("CraterStateChamp", "Gold Loc entering else:" + goldLoc);
                if(goldLoc == 0)
                {
                    telemetry.addData("Gold Loc: " , goldLoc);
                    telemetry.addData("Moving Right" , "");
                    telemetry.update();
                    //Move to Position 1
                    encoderTurn(0.5, leftAngle, "LEFT", 5);
                    goldLoc = 1;
                    forwardDistance = 30;
                    robotAngle = leftAngle;
                }
                else if(goldLoc == 1)
                {
                    //Move to Position 1
                    telemetry.addData("Gold Loc: " , goldLoc);
                    telemetry.addData("Moving Left" , "");
                    telemetry.update();
                    encoderTurn(0.5, rightAngle, "RIGHT", 8);
                    goldLoc = -1;
                    robotAngle = -leftAngle;
                }
                else
                {
                    goldLoc = -1;
                    telemetry.addData("Gold Loc: " , goldLoc);
                    telemetry.addData("Couldn't find Gold - Defaulting Value to True " , goldLoc);
                    telemetry.update();
                }
            }

        }


    }//end of hitGold


    /**
     * PLACE MARKER METHOD METHOD
     */
    private void placeMarker()
    {
        double angleToTurn = 45 + robotAngle;

        encoderTurn(WHEEL_SPEED, angleToTurn, "RIGHT", 5);

        wallAlign();

        laserDistance();

      //  releaseMarker();

        encoderDrive(WHEEL_SPEED, distanceToCrater, 8);

    }//end of place marker

    /**
     * RELEASE MARKER METHOD
     */
    private void releaseMarker()
    {
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 0.75)
        {
            robot.getArmAssembly().extendGrabber(0.8);
        }
        robot.getArmAssembly().stopGrabberExtension();

        robot.getArmAssembly().flipUp();
        sleep(1500);
    }


    /**
     * WALL ALIGN METHOD
     */
    private void wallAlign()
    {
        double angle = 5;

        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 0.5)
        {
            angle = robot.getNavigation().getDepotAngle();
        }

        if(Math.abs(angle) > 5)
        {
            encoderTurn(WHEEL_SPEED, angle, "LEFT", 5);
        }

        double distance = 10;
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 0.5)
        {
            distance = robot.getNavigation().getDepotDistance();

        }

        telemetry.addData("Distance", distance);
        telemetry.update();

        if(distance > 5)
        {
            double distanceToDrive = distance - 2;

            if(distanceToDrive > 2)
            {
                encoderSide(WHEEL_SPEED , distanceToDrive , "LEFT" , 5);
            }
        }

        //Double Check the angle
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 0.5)
        {
            angle = robot.getNavigation().getDepotAngle();
        }


        if(Math.abs(angle) > 3)
        {
            encoderTurn(WHEEL_SPEED, angle, "LEFT", 5);
        }

        if(goldLoc == 1)
        {
            encoderSide(WHEEL_SPEED, 4, "RIGHT", 5);
        }
    }

    /**
     * LASER DISTANCE METHOD
     */
    private void laserDistance()
    {
        double distance = 0;

        runtime.reset();
        while(opModeIsActive() && runtime.seconds() < 0.5)
        {
            distance = robot.getNavigation().getLaserDistance();
        }

        double distanceToDrive = distance - 20;

        if(Math.abs(distanceToDrive) > 5)
        {
            encoderDrive(WHEEL_SPEED, distanceToDrive, 8);
        }
    }



    /**
     *ENCODER DRIVE METHOD
     * @param speed (at which the robot should move)
     * @param inches (positive is forward, negative is backwards)
     * @param timeoutS (the robot will stop moving if it after this many seconds)
     */
    public void encoderDrive(double speed, double inches, double timeoutS)
    {
        int newBackLeftTarget;
        int newBackRightTarget;
        int newFrontLeftTarget;
        int newFrontRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {


            // Determine new target position, and pass to motor controller
            newBackLeftTarget = robot.getChassisAssembly().getBackLeftWheelCurrentPosition() - (int)(inches * COUNTS_PER_INCH);
            newBackRightTarget = robot.getChassisAssembly().getBackRightWheelCurrentPosition() - (int)(inches * COUNTS_PER_INCH);
            newFrontLeftTarget = robot.getChassisAssembly().getFrontLeftWheelCurrentPosition() - (int)(inches * COUNTS_PER_INCH);
            newFrontRightTarget = robot.getChassisAssembly().getFrontRightWheelCurrentPosition() - (int)(inches * COUNTS_PER_INCH);



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

    }//end of encoderDrive


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

    /**
     * ENCODER SIDE METHOD
     * @param speed (at which the robot moves side to side)
     * @param inches (to move side to side negative value will give opposite direction)
     * @param direction ("LEFT" or "RIGHT")
     * @param timeoutS (if time runs out, robot wil stop)
     */
    private void encoderSide(double speed, double inches, String direction , double timeoutS)
    {
        int newBackLeftTarget;
        int newBackRightTarget;
        int newFrontLeftTarget;
        int newFrontRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            if(direction == "RIGHT")
            {
                newBackLeftTarget = robot.getChassisAssembly().getBackLeftWheelCurrentPosition() + (int) (-inches * COUNTS_PER_SIDE_INCH);
                newBackRightTarget = robot.getChassisAssembly().getBackRightWheelCurrentPosition() + (int) (inches * COUNTS_PER_SIDE_INCH);
                newFrontLeftTarget = robot.getChassisAssembly().getFrontLeftWheelCurrentPosition() + (int) (inches * COUNTS_PER_SIDE_INCH);
                newFrontRightTarget = robot.getChassisAssembly().getFrontRightWheelCurrentPosition() + (int) (-inches * COUNTS_PER_SIDE_INCH);
            }
            else
            {
                newBackLeftTarget = robot.getChassisAssembly().getBackLeftWheelCurrentPosition() + (int) (inches * COUNTS_PER_SIDE_INCH);
                newBackRightTarget = robot.getChassisAssembly().getBackRightWheelCurrentPosition() + (int) (-inches * COUNTS_PER_SIDE_INCH);
                newFrontLeftTarget = robot.getChassisAssembly().getFrontLeftWheelCurrentPosition() + (int) (-inches * COUNTS_PER_SIDE_INCH);
                newFrontRightTarget = robot.getChassisAssembly().getFrontRightWheelCurrentPosition() + (int) (inches * COUNTS_PER_SIDE_INCH);
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

    }//end of encoderSide


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


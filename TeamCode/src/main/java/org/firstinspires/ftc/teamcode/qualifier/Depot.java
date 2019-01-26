package org.firstinspires.ftc.teamcode.qualifier;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;

@Autonomous(name="Depot", group ="Qualifier")
public class Depot extends LinearOpMode
{
    //TFod
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    private TFObjectDetector tfod;

    //Vuforia
    private static final String VUFORIA_KEY = "AVsSQB3/////AAABGehU0OjxREoznvNKEBqbvmskci8syRYfMKE0XlaGnZpw68DAZV19s7dfqc0vWrY78bAO2Ym2n1T2rDvNBOVVbMWxXIRo2c18JH6/c2fcKT1bRKxsG7bYq69+n9IHmKedY6rmTU1VOZZdtSTXh7exMsl67IAcnCZ0/ec+P+ZMpkK5v4X8d27rbEigGqqHayGe1/lG2afzgcHY7QxjJ/x5O4yGmVVs8wdzdupke19U+M8Z/x0FcYIfTAHuXcaydEL+h/w/ppcuNarD2ggo2BxdWeOGLx5GOin1yruVfvDAazPEuI0m3yEwXQNZ4e0ar2G0jDCZpAJPJcJRRVttBMwPoAvzTwySUx3qI1eNSJRAH+bk";
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = FRONT;
    VuforiaLocalizer vuforia;

    //Motion Variables
    double distanceToDepot;
    double distanceToWall;
    double forwardDistance = 25;
    double distanceToCrater = -72;

    double robotAngle = 0;

    int goldLoc = 0;


    //Encoder Constants
    final double COUNTS_PER_MOTOR_REV = 1120;
    final double DRIVE_GEAR_REDUCTION = 1.0;
    final double WHEEL_DIAMETER_INCHES = 4.0;
    final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);
    final double TURN_DIAMETER_INCHES = 18.6;

    final double DEGREES_PER_MOTOR_REV = (360 * (WHEEL_DIAMETER_INCHES)) / TURN_DIAMETER_INCHES;
    final double COUNTS_PER_DEGREE = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (DEGREES_PER_MOTOR_REV);
    final double COUNTS_PER_SIDE_INCH = 100;

    //Creating a Rover robot object
    RoverRobot robot = new RoverRobot();

    private static final double WHEEL_SPEED = 1;
    private ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode()
    {
        robot.initRobot(hardwareMap);

        initVuforia();

        //Setup TFod
        if (ClassFactory.getInstance().canCreateTFObjectDetector())
        {
            initTfod();
        }
        else
        {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        //Wait for Start
        telemetry.addData(">", "Press Play to begin");
        telemetry.update();
        waitForStart();

        moveFromLander();

        hitGold();

        placeMarker();

        parkInCrater();

    }



    public void initVuforia()
    {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = VUFORIA_KEY ;
        parameters.cameraDirection   = CAMERA_CHOICE;

        vuforia = ClassFactory.getInstance().createVuforia(parameters);

    }//End of initVuforia


    /**
     * INIT TFOD METHOD
     */
    private void initTfod()
    {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }//End of initTFod


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

            robot.getChassisAssembly().changeToEncoderMode();

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

        sleep(250);
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

        sleep(250);
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

        sleep(250);
    }//end of encoderSide


    /**
     * MOVE FROM LANDER METHOD
     */
    private void moveFromLander()
    {
        double angle = 100;

        encoderSide(WHEEL_SPEED , 12 , "LEFT" , 5);

        encoderTurn(WHEEL_SPEED , angle , "LEFT" , 5);

        robotAngle = angle - 45;

    }//end of moveFromLander


    /**
     * HIT GOLD METHOD
     */
    private void hitGold()
    {
        boolean goldFound = false;

        while(goldFound == false)
        {
            if (tfod != null)
            {
                tfod.activate();
                sleep(2000);
            }
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            goldFound = isGold(updatedRecognitions);
            telemetry.addData("Is Gold Found: ", goldFound);

            if(goldLoc == 1)
            {
                goldFound = true;
            }
            if(goldFound == true)
            {
                telemetry.addData("Gold Found, Position is" , goldLoc);
                telemetry.addData("Gold Loc: " , goldLoc);
                telemetry.addData("Moving Forward" , "");
                telemetry.update();
                encoderDrive(WHEEL_SPEED , forwardDistance , 5);
            }
            else
            {
                if(goldLoc == 0)
                {
                    telemetry.addData("Gold Loc: " , goldLoc);
                    telemetry.addData("Moving Right" , "");
                    telemetry.update();
                    //Move to Position -1
                    encoderSide(WHEEL_SPEED , 12 , "RIGHT" , 5);
                    goldLoc = -1;
                }
                else if(goldLoc == -1)
                {
                    //Move to Position 1
                    telemetry.addData("Gold Loc: " , goldLoc);
                    telemetry.addData("Moving Left" , "");
                    telemetry.update();
                    encoderSide(WHEEL_SPEED , 24 , "LEFT" , 8);
                    goldLoc = 1;
                }
                else
                {
                    goldLoc = 1;
                    telemetry.addData("Gold Loc: " , goldLoc);
                    telemetry.addData("Couldn't find Gold - Defaulting Value to True " , goldLoc);
                    telemetry.update();
                }
            }

        }

    }//end of hitGold


    /**
     * IS GOLD METHOD
     * @param updatedRecognitions must be found prior to calling this method
     * @return true if gold is detected false if gold is not detected
     */
    private boolean isGold(List<Recognition> updatedRecognitions)
    {
        for (Recognition recognition : updatedRecognitions)
        {
            if (recognition.getLabel().equals(LABEL_GOLD_MINERAL))
            {
                telemetry.addData("Mineral found is gold","");
                telemetry.update();
                return true;
            }
        }
        return  false;
    }//end of isGold

    /**
     * PLACE MARKER METHOD METHOD
     */
    private void placeMarker()
    {
        //Turn back to original angle
        encoderTurn(WHEEL_SPEED , robotAngle , "RIGHT" , 5);


        //Move Sideways
        if(goldLoc == -1)
        {
            distanceToWall = 36;
            distanceToDepot = 0;
        }
        else if(goldLoc == 0)
        {
            distanceToWall = 24;
            distanceToDepot = 12;
        }
        else
        {
            distanceToWall = 12;
            distanceToDepot = 36;

        }

        encoderSide(WHEEL_SPEED , distanceToWall , "LEFT" , 5);

        wallAlign();

        encoderDrive(WHEEL_SPEED , distanceToDepot , 5);

        releaseMarker();
    }//end of Place Marker

    /**
     * WALL ALIGN
     */
    private void wallAlign()
    {
        encoderSide(0.1 , 7, "LEFT" , 5);

        encoderSide(WHEEL_SPEED , 5 , "RIGHT" , 5);
    }//end of wall Align


    private void parkInCrater()
    {
        encoderDrive(WHEEL_SPEED , distanceToCrater , 5);
    }

    private void releaseMarker()
    {
        runtime.reset();

        while(opModeIsActive() && runtime.seconds() < 1)
        {
            robot.getArmAssembly().extendDeposit(0.4);
        }
        robot.getArmAssembly().stopDepositExtension();
        sleep(250);

        robot.getArmAssembly().flip(0.7);
        sleep(2000);

        robot.getArmAssembly().flip(0.009);
    }//end of release marker


}

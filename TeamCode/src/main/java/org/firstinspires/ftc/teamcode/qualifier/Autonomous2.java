package org.firstinspires.ftc.teamcode.qualifier;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.CameraDevice;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

@Autonomous(name="Auto2", group ="Qualifier")
@Disabled
public class Autonomous2 extends LinearOpMode
{
    //TFod
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    private TFObjectDetector tfod;


    //Minerals
    OpenGLMatrix LeftMineralLoc;
    OpenGLMatrix RightMineralLoc;
    OpenGLMatrix CenterMineralLoc;
    int goldLoc = -1;

    //Robot Position
    double robotAngle = 0;
    VectorF translation;
    Orientation rotation;

    //Motion Variables
    double forwardDistance;
    double distanceToDepot;
    double distanceToCrater = 72;

    //Encoder Constants
    final double COUNTS_PER_MOTOR_REV = 1120;
    final double DRIVE_GEAR_REDUCTION = 1.0;
    final double WHEEL_DIAMETER_INCHES = 4.0;
    final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);
    final double TURN_DIAMETER_INCHES = 18.6;

    final double DEGREES_PER_MOTOR_REV = (360 * (WHEEL_DIAMETER_INCHES)) / TURN_DIAMETER_INCHES;
    final double COUNTS_PER_DEGREE = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (DEGREES_PER_MOTOR_REV);

    //Conversion Factors
    private static final float mmPerInch = 25.4f;
    private static final float mmFTCFieldWidth = (12 * 6) * mmPerInch;
    private static final float mmTargetHeight = (6) * mmPerInch;

    //Creating a Rover robot object
    RoverRobot robot = new RoverRobot();

    //Vuforia
    private static final String VUFORIA_KEY = "AVsSQB3/////AAABGehU0OjxREoznvNKEBqbvmskci8syRYfMKE0XlaGnZpw68DAZV19s7dfqc0vWrY78bAO2Ym2n1T2rDvNBOVVbMWxXIRo2c18JH6/c2fcKT1bRKxsG7bYq69+n9IHmKedY6rmTU1VOZZdtSTXh7exMsl67IAcnCZ0/ec+P+ZMpkK5v4X8d27rbEigGqqHayGe1/lG2afzgcHY7QxjJ/x5O4yGmVVs8wdzdupke19U+M8Z/x0FcYIfTAHuXcaydEL+h/w/ppcuNarD2ggo2BxdWeOGLx5GOin1yruVfvDAazPEuI0m3yEwXQNZ4e0ar2G0jDCZpAJPJcJRRVttBMwPoAvzTwySUx3qI1eNSJRAH+bk";
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    VuforiaLocalizer vuforia;
    String startingLocation = "UNKNOWN";
    private OpenGLMatrix lastLocation = null;
    private boolean targetVisible = false;
    VuforiaTrackable foundTarget;
    List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();


    private ElapsedTime runtime = new ElapsedTime();

    private static final double WHEEL_SPEED = 0.7;

    @Override
    public void runOpMode()
    {
        robot.initRobot(hardwareMap);

        //Setup Vuforia
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

        moveFromLander(WHEEL_SPEED, 1.2);

        findTrackable(WHEEL_SPEED, "LEFT", 7);

        identifyStartLoc();

        hitGold();

        if(goldLoc == 2)
        {
            encoderTurn(WHEEL_SPEED , robotAngle , "RIGHT" , 5);
            wallAlign(2);
            distanceToDepot = Math.abs(Math.abs(RightMineralLoc.getTranslation().get(1)) - 610);
            distanceToDepot = distanceToDepot / 25.4;

            encoderDrive(WHEEL_SPEED , distanceToDepot , 10);

            encoderDrive(WHEEL_SPEED , distanceToCrater , 10);
        }
        else if(goldLoc == 1)
        {
            encoderTurn(WHEEL_SPEED , robotAngle , "RIGHT" , 5);
            wallAlign(2.5);
        }


    }


    /**
     * INITIALIZE VUFORIA METHOD
     */
    public void initVuforia()
    {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = VUFORIA_KEY ;
        parameters.cameraDirection   = CAMERA_CHOICE;

        vuforia = ClassFactory.getInstance().createVuforia(parameters);


        //Trackables
        VuforiaTrackables targetsRoverRuckus = this.vuforia.loadTrackablesFromAsset("RoverRuckus");
        VuforiaTrackable blueRover = targetsRoverRuckus.get(0);
        blueRover.setName("Blue-Rover");
        VuforiaTrackable redFootprint = targetsRoverRuckus.get(1);
        redFootprint.setName("Red-Footprint");
        VuforiaTrackable frontCraters = targetsRoverRuckus.get(2);
        frontCraters.setName("Front-Craters");
        VuforiaTrackable backSpace = targetsRoverRuckus.get(3);
        backSpace.setName("Back-Space");

        allTrackables.addAll(targetsRoverRuckus);


        //Setting the Location of the Trackables
        OpenGLMatrix blueRoverLocation = OpenGLMatrix
                .translation(0, mmFTCFieldWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0));
        blueRover.setLocation(blueRoverLocation);


        OpenGLMatrix redFootprintLocation = OpenGLMatrix
                .translation(0, -mmFTCFieldWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180));
        redFootprint.setLocation(redFootprintLocation);


        OpenGLMatrix frontCratersLocation = OpenGLMatrix
                .translation(-mmFTCFieldWidth, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90));
        frontCraters.setLocation(frontCratersLocation);


        OpenGLMatrix backSpaceLocation = OpenGLMatrix
                .translation(mmFTCFieldWidth, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 270));
        backSpace.setLocation(backSpaceLocation);

        final int CAMERA_FORWARD_DISPLACEMENT  = 216 ;  // eg: Camera is 216 mm in front of robot center
        final int CAMERA_VERTICAL_DISPLACEMENT = 121;   // eg: Camera is 121 mm above ground
        final int CAMERA_LEFT_DISPLACEMENT     = 19;     // eg: Camera is 19 mm to the left from the robot's center line

        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES,
                        -90, 0, -90));


        //Giving the phone location to all the trackables
        for (VuforiaTrackable trackable : allTrackables)
        {
            ((VuforiaTrackableDefaultListener)trackable.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        }


        //Activate
        targetsRoverRuckus.activate();

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
     * MOVE FROM LANDER METHOD
     */
    private void moveFromLander(double speed , double maxSeconds)
    {
        runtime.reset();

        encoderDrive(WHEEL_SPEED , 5 , 4);

        while(opModeIsActive() && runtime.seconds() < maxSeconds)
        {
            robot.getChassisAssembly().moveLeft(speed);
        }

        robot.getChassisAssembly().stopMoving();
        sleep(250);
    }


    /**
     *
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
     * FIND TRACKABLE METHOD
     * @param speed (at which to turn)
     * @param direction (LEFT or RIGHT)
     * @param turnSeconds Before switching direction
     */
    private void findTrackable(double speed, String direction, double turnSeconds)
    {
        runtime.reset();
        CameraDevice.getInstance().setFlashTorchMode(true) ;
        while(opModeIsActive() && targetVisible==false)
        {
            for (VuforiaTrackable trackable : allTrackables)
            {
                telemetry.addData("Searching for Targets: " , targetVisible);

                if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible())
                {
                    telemetry.addData("Visible Target", trackable.getName());
                    foundTarget = trackable;
                    targetVisible = true;

                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                    if (robotLocationTransform != null)
                    {
                        lastLocation = robotLocationTransform;
                    }
                    break;
                }
            }

            if (targetVisible)
            {

                telemetry.addData("Target Found: " , targetVisible);
                telemetry.update();

                robot.getChassisAssembly().stopMoving();

                //Get the Translational Information in Inches
                translation = lastLocation.getTranslation();
                telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                        translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

                //Get the Rotational Information in Degrees
                rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
                CameraDevice.getInstance().setFlashTorchMode(false) ;
                break;
            }
            else
            {
                telemetry.addData("Visible Target", "none");
                telemetry.update();

                if(direction == "LEFT")
                {
                    robot.getChassisAssembly().turnLeft(speed);
                }
                else if(direction == "RIGHT")
                {
                    robot.getChassisAssembly().turnRight(speed);
                }
                else
                {
                    robot.getChassisAssembly().stopMoving();
                    telemetry.addData("INVALID DIRECTION" , direction);
                    telemetry.update();
                }
                sleep(150);
            }

            robot.getChassisAssembly().stopMoving();
            sleep(300);

            if(runtime.seconds() > turnSeconds)
            {
                runtime.reset();

                if(direction == "LEFT")
                {
                    direction = "RIGHT";
                }
                else if(direction == "RIGHT")
                {
                    direction = "LEFT";
                }
            }

        }

        //Now that the target is visible, stop moving
        robot.getChassisAssembly().stopMoving();
        sleep(250);

    }//end of findTrackable


    /**
     * IDENTIFY STARTING LOCATION METHOD
     */
    private void identifyStartLoc()
    {
        double angle = rotation.thirdAngle;
        double targetAngle = 0;

        //To Prevent Negative Angles
        if(angle < 0)
        {
            angle = angle + 360;
        }

        //Identifying the Position
        if (translation.get(0) > 0 && translation.get(1) > 0)
        {
            startingLocation = "BLUE_CRATER";

            targetAngle = 90;
        }
        else if (translation.get(0) < 0 && translation.get(1) > 0)
        {
            startingLocation = "BLUE_DEPOT";

            targetAngle = 0;
        }
        else if (translation.get(0) > 0 && translation.get(1) < 0)
            {
                startingLocation = "RED_DEPOT";

                //Setting the Positions of the Minerals
                LeftMineralLoc = OpenGLMatrix
                        .translation(24 * mmPerInch,-48 * mmPerInch ,0 );
                CenterMineralLoc = OpenGLMatrix
                        .translation(36 * mmPerInch,-36 * mmPerInch ,0 );
                RightMineralLoc = OpenGLMatrix
                        .translation(48 * mmPerInch,-24 * mmPerInch ,0 );

                targetAngle = 270;


            }
            else if (translation.get(0) < 0 && translation.get(1) < 0)
            {
                startingLocation = "RED_CRATER";

                targetAngle = 180;
            }

        robotAngle = angle - targetAngle;
    }//end of identifyStartLoc


    /**
     * TURN TO MINERAL:
     */
    private void turnToMineral()
    {
        double mineralX;
        double mineralY;

        double robotX = translation.get(0);
        double robotY = translation.get(1);

        double angleToTurn = 0;

        if(goldLoc == -1)
        {
            mineralX = LeftMineralLoc.getTranslation().get(0);
            mineralY = LeftMineralLoc.getTranslation().get(1);

        }
        else if (goldLoc == 0)
        {
            mineralX = CenterMineralLoc.getTranslation().get(0);
            mineralY = CenterMineralLoc.getTranslation().get(1);
        }
        else
        {
            mineralX = RightMineralLoc.getTranslation().get(0);
            mineralY = RightMineralLoc.getTranslation().get(1);
        }


        //Finding the Distance the Robot Will Move Forward
        forwardDistance = Math.sqrt(Math.pow((mineralX - robotX),2) + Math.pow((mineralY-robotY) , 2));
        forwardDistance = forwardDistance / 25.4;


        //Now to find the Angle to Turn
        double angle;
        double division = Math.abs(mineralX-robotX) / Math.abs(mineralY - robotY);

        angle = Math.atan(division);
        angle = Math.toDegrees(angle);

        angleToTurn = angle - robotAngle;

        double offset = 0;
        double forwardAdd = 0;


        if(goldLoc == 0)
        {
            offset = 10;
            forwardAdd = 12;
        }

        if(goldLoc == 1)
        {
            offset = 15;
            forwardAdd = 8;
        }

        angleToTurn = angleToTurn + offset;
        forwardDistance = forwardDistance + forwardAdd;

        telemetry.addData("Robot X" , robotX);
        telemetry.addData("Robot Y" , robotY);
        telemetry.addData("Mineral X" , mineralX);
        telemetry.addData("Mineral Y" , mineralY);
        telemetry.addData("Angle: " , angle);
        telemetry.addData("Angle to Turn" , angleToTurn);
        telemetry.update();
        sleep(250);

        encoderTurn(WHEEL_SPEED , angleToTurn , "LEFT" , 2);

        robotAngle = angle;
    }

    private void hitGold()
    {
        boolean goldFound = false;

        while(goldLoc < 3)
        {
            if(goldLoc > -1)
            {
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                sleep(2000);

                goldFound = isGold(updatedRecognitions);
                telemetry.addData("Gold Found", goldFound);
            }

            if (goldFound == false)
            {
                turnToMineral();
                goldLoc++;

                if (tfod != null)
                {
                    tfod.activate();
                }

            }
            else
            {
                break;
            }

            sleep(1000);
        }

        encoderDrive(WHEEL_SPEED, forwardDistance, 6);
        telemetry.addData("Gold Loc", goldLoc);
        telemetry.update();

    }//end of hitGold


    private boolean isGold(List<Recognition> updatedRecognitions)
    {
        for (Recognition recognition : updatedRecognitions)
        {
            if (recognition.getLabel().equals(LABEL_GOLD_MINERAL))
            {
                telemetry.addData("gold Mineral, Position is: ", goldLoc);
                telemetry.update();
                int leftValue = (int) recognition.getLeft();
                int rightvalue= (int) recognition.getRight();
                int topvalue = (int) recognition.getTop();
                int bottonvalue = (int) recognition.getBottom();
                int imgWidth = (int) recognition.getConfidence();
                int imgconfidence = (int) recognition.getImageWidth();
                int angletoobject = (int) recognition.estimateAngleToObject(AngleUnit.DEGREES);
                telemetry.addData("leftValue: " + leftValue
                        + " rightvalue: " + rightvalue
                        + " topvalue: " + topvalue
                        + " bottonvalue: " + bottonvalue
                        + " imgWidth: " + imgWidth
                        + " imgconfidence" + imgconfidence
                        + " angletoobject" + angletoobject , " gold mineral position data");
                telemetry.update();

                sleep(250);
                return true;
            }
            else if(recognition.getLabel().equals(LABEL_SILVER_MINERAL));
            {
                if(goldLoc < 2)
                {
                    goldLoc++;
                    telemetry.addData("Silver Mineral, Position is: ", goldLoc);
                    telemetry.update();
                }
                else
                {
                    telemetry.addData("No Mineral, Position is: ", goldLoc);
                    telemetry.update();
                }
                return false;
            }
        }

        return  false;
    }//end of isGold


    private void wallAlign(double sec)
    {
        runtime.reset();

        while(opModeIsActive() && runtime.seconds() < sec)
        {
            robot.getChassisAssembly().moveLeft(0.1);
        }

        robot.getChassisAssembly().stopMoving();
        sleep(250);

        runtime.reset();

        while (opModeIsActive() && runtime.seconds() < 0.2)
        {
            robot.getChassisAssembly().moveRight(WHEEL_SPEED);
        }
    }



}

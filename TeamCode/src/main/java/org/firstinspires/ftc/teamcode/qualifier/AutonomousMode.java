/* Copyright (c) 2018 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.qualifier;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;


@Autonomous(name="QualifAutonomous", group ="Qualifier")
public class AutonomousMode extends LinearOpMode
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

    double robotAngle = 0;

    //Encoder Constants
    final double COUNTS_PER_MOTOR_REV = 1120;
    final double DRIVE_GEAR_REDUCTION = 1.0;
    final double WHEEL_DIAMETER_INCHES = 4.0;
    final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);
    final double TURN_DIAMETER_INCHES = 18.6;

    final double DEGREES_PER_MOTOR_REV = (360 * (WHEEL_DIAMETER_INCHES))/TURN_DIAMETER_INCHES;
    final double COUNTS_PER_DEGREE = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (DEGREES_PER_MOTOR_REV);

    //Conversion Factors
    private static final float mmPerInch        = 25.4f;
    private static final float mmFTCFieldWidth  = (12*6) * mmPerInch;
    private static final float mmTargetHeight   = (6) * mmPerInch;

    //Creating a Rover robot object
    RoverRobot robot = new RoverRobot();

    //Vuforia
    private static final String VUFORIA_KEY = "AVsSQB3/////AAABGehU0OjxREoznvNKEBqbvmskci8syRYfMKE0XlaGnZpw68DAZV19s7dfqc0vWrY78bAO2Ym2n1T2rDvNBOVVbMWxXIRo2c18JH6/c2fcKT1bRKxsG7bYq69+n9IHmKedY6rmTU1VOZZdtSTXh7exMsl67IAcnCZ0/ec+P+ZMpkK5v4X8d27rbEigGqqHayGe1/lG2afzgcHY7QxjJ/x5O4yGmVVs8wdzdupke19U+M8Z/x0FcYIfTAHuXcaydEL+h/w/ppcuNarD2ggo2BxdWeOGLx5GOin1yruVfvDAazPEuI0m3yEwXQNZ4e0ar2G0jDCZpAJPJcJRRVttBMwPoAvzTwySUx3qI1eNSJRAH+bk";
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = FRONT;
    VuforiaLocalizer vuforia;
    String startingLocation = "UNKNOWN";
    private OpenGLMatrix lastLocation = null;
    private boolean targetVisible = false;
    VuforiaTrackable foundTarget;
    VectorF translation;
    Orientation rotation;
    List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();


    // Gyro states used for updating telemetry
    Orientation angles;
    Acceleration gravity;


    private ElapsedTime runtime = new ElapsedTime();

    private static final double WHEEL_SPEED = 0.4;

    @Override
    public void runOpMode()
    {
        robot.initRobot(hardwareMap);

        //Setup Vuforia
        initVuforia();


        //Setup TFod
        if (ClassFactory.getInstance().canCreateTFObjectDetector()) { initTfod(); }
        else { telemetry.addData("Sorry!", "This device is not compatible with TFOD"); }

        //Wait for Start
        telemetry.addData(">", "Press Play to begin");
        telemetry.update();
        while (!opModeIsActive() && !isStopRequested())
        {
            telemetry.addData("status" , "waiting for start command...");
            telemetry.update();
        }
        //drop();

        sleep(500);

        moveFromLander(WHEEL_SPEED * 2 , 0.5);

        findTrackable(WHEEL_SPEED , "LEFT" , 5);

        identifyStartLoc();


        if(startingLocation == "RED_CRATER")
        {
            //aligning to the trackable
            alignToTrackable(WHEEL_SPEED , 90, 5);



        }
        else if(startingLocation == "RED_DEPOT")
        {

            alignToTrackable(WHEEL_SPEED , 180, 5);

            //Setting the Positions of the Minerals
            LeftMineralLoc = OpenGLMatrix
                    .translation(24 * mmPerInch,-48 * mmPerInch ,0 );
            CenterMineralLoc = OpenGLMatrix
                    .translation(36 * mmPerInch,-36 * mmPerInch ,0 );
            RightMineralLoc = OpenGLMatrix
                    .translation(48 * mmPerInch,-24 * mmPerInch ,0 );//Setting the Positions of the Minerals
            LeftMineralLoc = OpenGLMatrix
                    .translation(24 * mmPerInch,-48 * mmPerInch ,0 );
            CenterMineralLoc = OpenGLMatrix
                    .translation(36 * mmPerInch,-36 * mmPerInch ,0 );
            RightMineralLoc = OpenGLMatrix
                    .translation(48 * mmPerInch,-24 * mmPerInch ,0 );

            findGold(100);


        }
        else if(startingLocation == "BLUE_CRATER")
        {
            alignToTrackable(WHEEL_SPEED , -90, 5);



        }
        else if(startingLocation == "BLUE_DEPOT")
        {




        }
        else
            {
                telemetry.addData("Could Not Find Location" , startingLocation);
                telemetry.update();
            }


        if (tfod != null)
        {
            tfod.shutdown();
        }



    }//end of runOpMode


    /**
     * INIT VUFORIA METHOD
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


        //Phone Location
        final int CAMERA_FORWARD_DISPLACEMENT  = 254; //camera is 254 mm in front of the robot center
        final int CAMERA_VERTICAL_DISPLACEMENT = 245;
        final int CAMERA_HORIZONTAL_DISPLACEMENT = -51; //camera is 51 mm to the left of the robot center

        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_HORIZONTAL_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES,
                        90 , 90, 180));

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

        while(opModeIsActive() && runtime.seconds() < maxSeconds)
        {
            robot.getChassisAssembly().moveLeft(speed);
        }

        robot.getChassisAssembly().stopMoving();
        sleep(250);
    }


    /**
     * FIND TRACKABLE METHOD
     */
    private void findTrackable(double speed, String direction, double turnSeconds)
    {
        runtime.reset();

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
                sleep(200);
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
        if (translation.get(0) > 0 && translation.get(1) > 0)
        {
            startingLocation = "BLUE_CRATER";
        }
        else if (translation.get(0) < 0 && translation.get(1) > 0)
        {
            startingLocation = "BLUE_DEPOT";
        }
        else if (translation.get(0) > 0 && translation.get(1) < 0)
        {
            startingLocation = "RED_DEPOT";
        }
        else if (translation.get(0) < 0 && translation.get(1) < 0)
        {
            startingLocation = "RED_CRATER";
        }

        telemetry.addData("StartingLocation: ", startingLocation);
        telemetry.addData("Angle: " , rotation.thirdAngle);

        telemetry.update();
    }


    /**
     * ALIGN TO TRACKABLE
     */
    private void alignToTrackable(double speed , double targetAngle , double marginOfError)
    {
        double angle = rotation.thirdAngle;

        while((angle < (targetAngle - marginOfError) || angle > (targetAngle + marginOfError)) && opModeIsActive())
        {
            OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) foundTarget.getListener()).getUpdatedRobotLocation();

            if(robotLocationTransform != null)
            {
                lastLocation = robotLocationTransform;
            }

            rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);

            angle = rotation.thirdAngle;

            if(angle > targetAngle + marginOfError)
            {
                robot.getChassisAssembly().turnRight(speed);
            }
            else if(angle < targetAngle - marginOfError)
            {
                robot.getChassisAssembly().turnLeft(speed);
            }
            else
            {
                robot.getChassisAssembly().stopMoving();
            }
        }

        robot.getChassisAssembly().stopMoving();

        //Update the positions of the robot
        translation = lastLocation.getTranslation();
        rotation = Orientation.getOrientation(lastLocation , EXTRINSIC , XYZ , DEGREES);

        sleep(250);
    }//end of alignToTrackable


    /**
     * TURN TO MINERAL METHOD
     */
    private void turnToMineral()
    {
        double mineralX;
        double mineralY;

        double robotX = translation.get(0);
        double robotY = translation.get(1);


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

        //Now to find the Angle to Turn
        double angle;
        double division = Math.abs(mineralX-robotX) / Math.abs(mineralY - robotY);

        angle = Math.atan(division);
        angle = Math.toDegrees(angle);

        double angleToTurn = angle - robotAngle;

        telemetry.addData("Robot X" , robotX);
        telemetry.addData("Robot Y" , robotY);
        telemetry.addData("Mineral X" , mineralX);
        telemetry.addData("Mineral Y" , mineralY);
        telemetry.addData("Angle: " , angle);
        telemetry.addData("Angle to Turn" , angleToTurn);
        telemetry.update();
        sleep(30000);

        encoderTurn(WHEEL_SPEED , angleToTurn , "LEFT" , 2);

        robotAngle = angle;

    }

    /**
     * FIND GOLD METHOD
     * @param maxSeconds
     */
    private void findGold(double maxSeconds)
    {

        if(tfod != null)
        {
            tfod.activate();
        }

        for(goldLoc = -1 ; goldLoc < 2 ; goldLoc++)
        {
            //First turn toward the Mineral in Question
            if(opModeIsActive())
            {
                turnToMineral();

                if (tfod != null)
                {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    sleep(10000);
                    if (updatedRecognitions != null)
                    {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        boolean isGold = isGold(updatedRecognitions);
                        if (isGold)
                        {
                            telemetry.addData("Gold Found ", goldLoc);
                            telemetry.update();
                            sleep(3000);

                            break;
                        }
                    }
                }
                else
                {
                    telemetry.addData("TFod is Null" , "");
                    telemetry.update();
                    sleep(7000);
                }
            }
            else
            {
                break;
            }
        }

        if(opModeIsActive())
        {
            //Move Forward to displace the mineral
            encoderDrive(WHEEL_SPEED , -28 , 5);

            //Move Backwards
            encoderDrive(WHEEL_SPEED, 10 , 2);
        }


    }//end of findGold Method

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
                        + " angle to object" + angletoobject , " gold mineral position data");
                telemetry.update();

                sleep(250);
                return true;
            }

        }

        return false;
    }


    /**
     * ENCODERDRIVE METHOD - used for moving the robot forward or backwards
     * @param speed at which the robot will move
     * @param inches - the inches to move forward or backwards.  Negative if backwards.
     * @param timeoutS
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
            newBackLeftTarget = robot.getChassisAssembly().getBackLeftWheelCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            newBackRightTarget = robot.getChassisAssembly().getBackRightWheelCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            newFrontLeftTarget = robot.getChassisAssembly().getFrontLeftWheelCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            newFrontRightTarget = robot.getChassisAssembly().getFrontRightWheelCurrentPosition() + (int)(inches * COUNTS_PER_INCH);



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
     * DROP METHOD
     * UNCOMMENT THIS AFTER ADDING THE CODE FOR LIFT ASSEMBLY IN HARDWARE CLASS
     */
  /*  private void drop ()
    {
        while (opModeIsActive())
        {
            //Closing lift to max shrink
            releaseRobot();
            break;
        }
    }
    */

    /**
     * RELEASE ROBOT METHOD
     */
    /*
    private void drop() {

        while (robot.getLiftAssembly().robotHardware.bottomTouch.getState() == true)
        {
            robot.getLiftAssembly().lowerRobot(1);
            telemetry.addData("in while loop: ", "still going on");
            telemetry.update();
        }

        robot.getLiftAssembly().unlockRobot();
        robot.getLiftAssembly().resetLift();
        telemetry.addData("outside while loop: ", "reached the bottom");
        telemetry.update();

        while (robot.getLiftAssembly().robotHardware.topTouch.getState() == true)
        {
            robot.getLiftAssembly().liftUpRobot(0.5);
            telemetry.addData("in second while loop", "still going on");
            telemetry.update();
        }
        robot.getLiftAssembly().resetLift();
        telemetry.addData("outside 2 while loop: ", "reached the top");
        telemetry.update();

        runtime.reset();
        while (runtime.seconds() < 0.3)
        {
            robot.getChassisAssembly().moveForward(0.6);
        }
        robot.getChassisAssembly().stopMoving();
        telemetry.addData("in stop moving","stopped");
        telemetry.update();

        while (robot.getLiftAssembly().robotHardware.bottomTouch.getState() == true)
        {
            robot.getLiftAssembly().lowerRobot(0.5);
        }
        robot.getLiftAssembly().resetLift();
        }//end of drop


    private void uTurn(double maxSeconds)
    {
        runtime.reset();
        while(opModeIsActive() && runtime.seconds() < maxSeconds)
        {
            robot.getChassisAssembly().turnRight(WHEEL_SPEED);
        }

        robot.getChassisAssembly().stopMoving();

        sleep(250);
    }

*/

}//end of class


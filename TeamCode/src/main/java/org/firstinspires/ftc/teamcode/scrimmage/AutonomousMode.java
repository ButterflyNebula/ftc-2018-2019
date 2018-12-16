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

package org.firstinspires.ftc.teamcode.scrimmage;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;


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

import com.qualcomm.robotcore.util.ElapsedTime;


import java.util.Locale;
import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;


@Autonomous(name="Autonomous", group ="Scrimmage")
public class AutonomousMode extends LinearOpMode
{
    //TFod
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    private TFObjectDetector tfod;
    int goldLoc = 0;



    //Encoder Constants
    final double COUNTS_PER_MOTOR_REV = 1120;
    final double DRIVE_GEAR_REDUCTION = 1.0;
    final double WHEEL_DIAMETER_INCHES = 4.0;
    final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

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
        waitForStart();

        drop();

        sleep(500);

        moveFromLander(WHEEL_SPEED * 2 , 0.75);

        findTrackable(WHEEL_SPEED , "LEFT" , 5);

        identifyStartLoc();


        if(startingLocation == "RED_CRATER")
        {
            alignToTrackable(WHEEL_SPEED , 90, 5);

            turnToGold(WHEEL_SPEED, "LEFT", 10);

            //Now to displace the gold
            encoderDrive(WHEEL_SPEED, -28, -28, 5.0);

            //Back away a little
            //encoderDrive(WHEEL_SPEED , 6 , 6 , 2.0);

            //stop moving
            robot.getChassisAssembly().stopMoving();
        }
        else if(startingLocation == "RED_DEPOT")
        {

            alignToTrackable(WHEEL_SPEED , 180, 5);

            turnToGold(WHEEL_SPEED, "LEFT", 10);

            //Now to displace the gold
            encoderDrive(WHEEL_SPEED, -28, -28, 5.0);

            //Back away a little
            encoderDrive(WHEEL_SPEED , 10 , 10 , 2.0);


            //uTurn(2);

            robot.getChassisAssembly().stopMoving();

            sleep(500);

            /*
            // Set up our telemetry dashboard
            composeTelemetry();
            telemetry.update();
            // Start the logging of measured acceleration
            robot.getNavigation().getImu().startAccelerationIntegration(new Position(), new Velocity(), 1000);

            turn180(10);

            sleep(5000);

            //Release Marker
            */
        }
        else if(startingLocation == "BLUE_CRATER")
        {
            alignToTrackable(WHEEL_SPEED , -90, 5);

            turnToGold(WHEEL_SPEED, "LEFT", 10);

            //Now to displace the gold
            encoderDrive(WHEEL_SPEED, -28, -28, 5.0);

            //Back away a little
            encoderDrive(WHEEL_SPEED , 10 , 10 , 2.0);

            //stop moving
            robot.getChassisAssembly().stopMoving();

        }
        else if(startingLocation == "BLUE_DEPOT")
        {

            alignToTrackable(WHEEL_SPEED , 0, 5);

            turnToGold(WHEEL_SPEED, "LEFT", 10);

            //Now to displace the gold
            encoderDrive(WHEEL_SPEED, -28, -28, 5.0);

            //Back away a little
            encoderDrive(WHEEL_SPEED , 10 , 10 , 2.0);

            //stop moving
            robot.getChassisAssembly().stopMoving();



        }
        else
            {
                telemetry.addData("Could Not Find Location" , startingLocation);
                telemetry.update();
            }


        if (tfod != null) {
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
        final int CAMERA_FORWARD_DISPLACEMENT  = 0;
        final int CAMERA_VERTICAL_DISPLACEMENT = 280;
        final int CAMERA_HORIZONTAL_DISPLACEMENT = 0;

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

        sleep(250);
    }//end of alignToTrackable


    /**
     * TURN TO GOLD METHOD
     */
    public void turnToGold(double speed, String direction, double maxSeconds)
    {
        if (tfod != null) {
            tfod.activate();
        }

        int count = 0;

        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < maxSeconds)
        {
            if (tfod != null)
            {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                sleep(1000);
                if (updatedRecognitions != null)
                {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    boolean isGold = findGold(updatedRecognitions);
                    if (isGold)
                    {
                        telemetry.addData("Gold Found ", true);
                        telemetry.update();
                        break;
                    }
                    else
                    {
                        telemetry.addData("did not get gold position ", "");
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
                            telemetry.addData("INVALID DIRECTION" , direction);
                            telemetry.update();
                            robot.getChassisAssembly().stopMoving();
                        }
                        sleep(250);
                    }
                }
            }
            robot.getChassisAssembly().stopMoving();
            count++;
        }

        robot.getChassisAssembly().stopMoving();
        if(direction == "LEFT")
        {
            if(count < 3)
            {
                robot.getChassisAssembly().turnRight(speed);
            }
            else
            {
                robot.getChassisAssembly().turnLeft(speed);
            }
        }
        else
        {
            if(count < 2)
            {
                robot.getChassisAssembly().turnLeft(speed);
            }
            else
            {
                robot.getChassisAssembly().turnRight(speed);
            }
        }
        sleep(250);

        robot.getChassisAssembly().stopMoving();
        sleep(250);
    }//end of turnToGold


    /**
     * FINDING GOLD METHOD
     */
    private boolean findGold(List<Recognition> updatedRecognitions)
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
    }//end of findGold



    /**
     * ENCODER METHOD!
     * @param speed at which the robot will move
     * @param leftInches - the inches to move forward or backwards.  Negative if backwards. Should be the same as rightInches
     * @param rightInches- the inches to move forward or backwards. Negative if backwards.  Should be the same as leftInches
     * @param timeoutS
     */
    public void encoderDrive(double speed, double leftInches, double rightInches, double timeoutS)
    {
        int newBackLeftTarget;
        int newBackRightTarget;
        int newFrontLeftTarget;
        int newFrontRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newBackLeftTarget = robot.getChassisAssembly().getBackLeftWheelCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newBackRightTarget = robot.getChassisAssembly().getBackRightWheelCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            newFrontLeftTarget = robot.getChassisAssembly().getFrontLeftWheelCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newFrontRightTarget = robot.getChassisAssembly().getFrontRightWheelCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);


            robot.getChassisAssembly().setBackLeftWheelTargetPosition(newBackLeftTarget);
            robot.getChassisAssembly().setBackRightWheelTargetPosition(newBackRightTarget);
            robot.getChassisAssembly().setFrontLeftWheelPosition(newFrontLeftTarget);
            robot.getChassisAssembly().setFrontRightWeelPosition(newFrontRightTarget);

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


    public void composeTelemetry() {

        telemetry.addData("Inside compose telementry", "");
        telemetry.update();
        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() { @Override public void run()
        {
            // Acquiring the angles is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the necessary expense.
            angles   = robot.getNavigation().getImu().getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            gravity  = robot.getNavigation().getImu().getGravity();
        }
        });

        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override public String value() {
                        return robot.getNavigation().getImu().getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override public String value() {
                        return robot.getNavigation().getImu().getCalibrationStatus().toString();
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
        if(angles  !=null)
            telemetry.addData("compose telemetry angles.firstAngle is :", angles.firstAngle);
        else
            telemetry.addData("compose telemetry angles.firstAngle is :", "null");

        telemetry.update();
        telemetry.addData("Leaving compose telemetry", "");
        telemetry.update();
        sleep(3000);


    }// end of composeTelemetry

    /**
     * The following two methods are used in the method composeTelemetry
     * They are need to format the angle in degrees so that it is comprehensive
     */
    String formatAngle(AngleUnit angleUnit, double angle) { return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle)); }
    String formatDegrees(double degrees){ return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees)); }


    /**
     * TURN 180 METHOD
     * @param maxSeconds
     */
    public void turn180(double maxSeconds)
    {
        double angle = 180;
        double marginOfError = 2;
        boolean arrivedAtAngle = false;
        double speedReduction = 15;
        boolean speedReduced = false;
        double speed = WHEEL_SPEED;

        telemetry.addData("First Angle is: " , angles.firstAngle);
        telemetry.addData("Second Angle is: " , angles.secondAngle);
        telemetry.addData("Third Angle is: " , angles.thirdAngle);
        telemetry.update();
        sleep(5000);


        while(opModeIsActive() && arrivedAtAngle == false && runtime.seconds() < maxSeconds)
        {
            if(angles.secondAngle < (angle - marginOfError))
            {
                robot.getChassisAssembly().turnRight(speed);
            }
            else if(angles.secondAngle > (angle + marginOfError))
            {
                robot.getChassisAssembly().turnLeft(speed);
            }
            else
            {
                robot.getChassisAssembly().stopMoving();
                arrivedAtAngle = true;
            }

            sleep(250);

            robot.getChassisAssembly().stopMoving();
            sleep(500);
        }

        // Loop and update the dashboard
     /*   while (arrivedAtAngle == false && runtime.seconds() < maxSeconds)
        {
            if (angles.firstAngle < (angle + marginOfError) && angles.firstAngle > (angle - marginOfError))
            {
                arrivedAtAngle = true;
            }

            if(angles.firstAngle < (angle + speedReduction) && angles.firstAngle > (angle - speedReduction) && speedReduced == false)
            {

                //reduce the speed as it gets close to the angle
                speed = speed - 0.1;

                //to ensure that it does not continue to decrement the speed
                speedReduced = true;
            }

            if(arrivedAtAngle == false)
            {

                if(angles.firstAngle >=0)
                {
                    if (angles.firstAngle < angle) {
                        robot.getChassisAssembly().turnLeft(speed);
                    } else if (angles.firstAngle > angle) {
                        robot.getChassisAssembly().turnRight(speed);
                    } else {
                        robot.getChassisAssembly().stopMoving();
                    }
                }
                else if(angles.firstAngle < 0)
                {
                    if (angles.firstAngle > angle) {
                        robot.getChassisAssembly().turnLeft(speed);
                    } else if (angles.firstAngle < angle) {
                        robot.getChassisAssembly().turnRight(speed);
                    } else {
                        robot.getChassisAssembly().stopMoving();
                    }
                }

                sleep(1000);
                robot.getChassisAssembly().stopMoving();
            }

            telemetry.update();
        }
        */

        runtime.reset();

        //Display that the turn is completed
        telemetry.addData("Turned the desired angle", angles.firstAngle);
        telemetry.update();
        sleep(5000);
    }//end of turn180


    /**
     * DROP METHOD
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

}//end of class


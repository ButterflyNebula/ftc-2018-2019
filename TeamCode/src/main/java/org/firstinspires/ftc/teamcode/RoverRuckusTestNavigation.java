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

package org.firstinspires.ftc.teamcode;

import android.view.OrientationEventListener;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import android.util.Log;
import com.qualcomm.robotcore.hardware.DcMotor;


import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import com.qualcomm.robotcore.util.ElapsedTime;


import java.util.Locale;
import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;


@TeleOp(name="Test: Rover Navigation", group ="Test")
public class RoverRuckusTestNavigation extends LinearOpMode {

    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    //Getting a rover robot
    RoverRobot robot = new RoverRobot();

    //Vuforia Setup
    private static final String VUFORIA_KEY = "AVsSQB3/////AAABGehU0OjxREoznvNKEBqbvmskci8syRYfMKE0XlaGnZpw68DAZV19s7dfqc0vWrY78bAO2Ym2n1T2rDvNBOVVbMWxXIRo2c18JH6/c2fcKT1bRKxsG7bYq69+n9IHmKedY6rmTU1VOZZdtSTXh7exMsl67IAcnCZ0/ec+P+ZMpkK5v4X8d27rbEigGqqHayGe1/lG2afzgcHY7QxjJ/x5O4yGmVVs8wdzdupke19U+M8Z/x0FcYIfTAHuXcaydEL+h/w/ppcuNarD2ggo2BxdWeOGLx5GOin1yruVfvDAazPEuI0m3yEwXQNZ4e0ar2G0jDCZpAJPJcJRRVttBMwPoAvzTwySUx3qI1eNSJRAH+bk";
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = FRONT;
    VuforiaLocalizer vuforia;

    //Conversion Factors
    private static final float mmPerInch        = 25.4f;
    private static final float mmFTCFieldWidth  = (12*6) * mmPerInch;       // Field Width (from center to wall) (1828 mm)
    private static final float mmTargetHeight   = (6) * mmPerInch;          // Height to the Center of the Target (146.5 mm)


    private OpenGLMatrix lastLocation = null;
    private boolean targetVisible = false;

    VectorF translation;
    Orientation rotation;

    // Gyro states used for updating telemetry
    Orientation angles;
    Acceleration gravity;

    private ElapsedTime runtime = new ElapsedTime();

    private TFObjectDetector tfod;


    VuforiaTrackable foundTarget;

    int goldLoc = 0;

    //Encoder Constants
    final double COUNTS_PER_MOTOR_REV = 1120;
    final double DRIVE_GEAR_REDUCTION = 1.0;
    final double WHEEL_DIAMETER_INCHES = 4.0;
    final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);


    @Override
    public void runOpMode()
    {

        robot.initRobot(hardwareMap);

        //Vuforia Setup
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

        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
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
        final int CAMERA_FORWARD_DISPLACEMENT  = 0;   //Camera is 0 mm in front of robot center
        final int CAMERA_VERTICAL_DISPLACEMENT = 280;   //Camera is 150 mm above ground
        final int CAMERA_HORIZONTAL_DISPLACEMENT = 0;

        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_HORIZONTAL_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES,
                       90 , 90, -90)); //since the phone is on the side of the robot

        //Giving the phone location to all the trackables
        for (VuforiaTrackable trackable : allTrackables)
        {
            ((VuforiaTrackableDefaultListener)trackable.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        }


        //Location of Field Objects
        OpenGLMatrix blueDepotLocation = OpenGLMatrix
                .translation(-mmFTCFieldWidth,mmFTCFieldWidth , 0)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES,
                        0 , 0,  0));
        double depotSize = 24 * mmPerInch;







        //TFod
        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }


        //Wait for Start
        telemetry.addData(">", "Press Play to begin");
        telemetry.update();
        waitForStart();

        //Bring the robot back by moving sideways
        runtime.reset();

        while(opModeIsActive() && runtime.seconds() < 1)
        {
            robot.getChassisAssembly().moveLeft(0.3);
        }

        robot.getChassisAssembly().stopMoving();
        sleep(1000);

        targetsRoverRuckus.activate();
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

                robot.getChassisAssembly().turnLeft(0.3);
                sleep(200);
            }

            robot.getChassisAssembly().stopMoving();
            telemetry.addData("Restarting Loop", targetVisible);
            telemetry.update();
            sleep(200);

        }

        //Now that the target is visible, stop moving
        robot.getChassisAssembly().stopMoving();
        String startingLocation = "UNKNOWN";

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
        sleep(1000);

        if(startingLocation == "RED_CRATER")
        {
            //Step 1: Turn until back of the robot faces the sampling minerals
            //We need to turn until two minerals are visible (for now skipping this)
            double angle = rotation.thirdAngle;

            while(angle < -5 || angle > 5)
            {
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) foundTarget.getListener()).getUpdatedRobotLocation();

                if(robotLocationTransform != null)
                {
                    lastLocation = robotLocationTransform;
                }

                rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);

                angle = rotation.thirdAngle;

                if(angle > 5)
                {
                    robot.getChassisAssembly().turnRight(0.3);
                }
                else if(angle < -5)
                {
                    robot.getChassisAssembly().turnLeft(0.3);
                }
                else
                {
                    robot.getChassisAssembly().stopMoving();
                }
            }

            robot.getChassisAssembly().stopMoving();

            sleep(5000);



            runtime.reset();

            /** Activate Tensor Flow Object Detection. */
            if (tfod != null) {
                tfod.activate();
            }
            boolean mineralsFound = false;

            while (opModeIsActive() && runtime.seconds() < 30 && mineralsFound==false)
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
                            mineralsFound=true;
                            break;
                        }
                        else
                        {
                            telemetry.addData("did not get gold position ", "");
                            telemetry.update();
                            robot.getChassisAssembly().turnLeft(0.4);
                            sleep(250);
                        }
                    }
                }

                robot.getChassisAssembly().stopMoving();
            }

            robot.getChassisAssembly().stopMoving();
            sleep(5000);

            encoderDrive(0.3, 24, 24, 5.0);  // S1: Forward 6 Inches with 5 Sec timeout

            sleep(1000);

            encoderDrive(0.3, -24 , -24, 5.0);//S2: Backwards 30 inches with 5 sec timeout

            sleep(1000);


            /*
            targetVisible = false;
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

                    robot.getChassisAssembly().turnRight(0.3);
                    sleep(200);
                }

                robot.getChassisAssembly().stopMoving();
                telemetry.addData("Restarting Loop", targetVisible);
                telemetry.update();
                sleep(200);

            }

            robot.getChassisAssembly().stopMoving();

            sleep(1000);

            angle = rotation.thirdAngle;

            while(angle < -5 || angle > 5)
            {
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) foundTarget.getListener()).getUpdatedRobotLocation();

                if(robotLocationTransform != null)
                {
                    lastLocation = robotLocationTransform;
                }

                rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);

                angle = rotation.thirdAngle;

                if(angle > 5)
                {
                    robot.getChassisAssembly().turnRight(0.3);
                }
                else if(angle < -5)
                {
                    robot.getChassisAssembly().turnLeft(0.3);
                }
                else
                {
                    robot.getChassisAssembly().stopMoving();
                }
            }

            robot.getChassisAssembly().stopMoving();

            sleep(1000);

            encoderDrive(0.3, -6, -6, 5.0);  // S1: Forward 6 Inches with 5 Sec timeout

            robot.getChassisAssembly().stopMoving();
            sleep(1000);



            double distance = robot.getNavigation().getDistanceSensor().getDistance(DistanceUnit.INCH);

            telemetry.addData("Distance Found: ", distance);
            telemetry.update();

            sleep(1000);

            while(distance > 2)
            {
                robot.getChassisAssembly().moveLeft(0.3);
                sleep(200);

                distance = robot.getNavigation().getDistanceSensor().getDistance(DistanceUnit.INCH);
                telemetry.addData("Distance Found: ", distance);
                telemetry.update();

                robot.getChassisAssembly().stopMoving();

                if(distance < 2)
                {
                    robot.getChassisAssembly().stopMoving();
                }

            }

            robot.getChassisAssembly().stopMoving();
            */



        }

        if (tfod != null) {
            tfod.shutdown();
        }
    }


    /**
     * GYRO METHOD
     */
    public void composeTelemetry() {

        telemetry.addAction(new Runnable()
        { @Override public void run()
        {
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
    }






    /**
     * The following two methods are used in the method composeTelemetry
     * They are need to format the angle in degrees so that it is comprehensive
     */
    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }



    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }

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

                sleep(5000);
                return true;
            }
            else if(recognition.getLabel().equals(LABEL_SILVER_MINERAL));
            {
                if(goldLoc < 2)
                {
                    goldLoc++;
                    telemetry.addData("Silver Mineral, Position is: ", goldLoc);
                    telemetry.update();
                    sleep(3000);
                }
                else
                {
                    telemetry.addData("No Mineral, Position is: ", goldLoc);
                    telemetry.update();
                    sleep(3000);
                }
                return false;
            }
        }

        /*
        String goldPos = "";
        if (updatedRecognitions.size() >= 2) {
            int goldMineralX = -1;
            int silverMineral1X = -1;
            int silverMineral2X = -1;
            for (Recognition recognition : updatedRecognitions) {
                if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                    goldMineralX = (int) recognition.getLeft();
                } else if (silverMineral1X == -1 && recognition.getLabel().equals(LABEL_SILVER_MINERAL)) {
                    silverMineral1X = (int) recognition.getLeft();
                } else {
                    if(recognition.getLabel().equals(LABEL_SILVER_MINERAL)) {
                        silverMineral2X = (int) recognition.getLeft();
                    }
                }
            }
            /*scanning from left for first 2 minerals to check to identify the gold position
             if both the minerals are silver, then gold should be on right side;
             else if gold object location is less than silver object location then gold is on left
             else gold is on center.
             */
        /*
            if (goldMineralX == -1) {
                goldPos = "RIGHT";
                //  telemetry.addData("Did not see Gold - Gold Mineral Position should be ", "Right");
                //telemetry.update();
            } else if (goldMineralX != -1 && (silverMineral1X != -1 || silverMineral2X != -1)) {
                if (goldMineralX > silverMineral1X || goldMineralX > silverMineral2X) {
                    goldPos = "LEFT";
                    //  telemetry.addData("Gold Mineral Position", "Left");
                    //  telemetry.update();
                } else {
                    goldPos = "CENTER";
                    // telemetry.addData("Gold Mineral Position", "Center");
                    // telemetry.update();
                }
            }
            // telemetry.update();
        }

        */
        return  false;
    }





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
    }
}


/* Copyright (c) 2017 FIRST. All rights reserved.
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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.RobotLog;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

import static com.sun.tools.doclint.Entity.chi;
import static com.sun.tools.javac.main.Option.S;

/**
 * This OpMode is used for testing how to enter the positions of the phone and an image target on the field
 * For this, the image "stones" is used.
 * The stones location parameters are adjusted for various tests.
 * It is also a teleOp mode so the robot can be driven around and the position changes be observed
 */

@TeleOp(name="TestNavigation", group ="Tests")
public class TestNavigation extends LinearOpMode {


    OpenGLMatrix lastLocation = null;

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    VuforiaLocalizer vuforia;

    //Declare Hardware
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;



    @Override public void runOpMode() {

        //Add harware to hardwareMap and set motor direction
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");

        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        /*
         * Vuforia Set Up
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = "AVsSQB3/////AAABGehU0OjxREoznvNKEBqbvmskci8syRYfMKE0XlaGnZpw68DAZV19s7dfqc0vWrY78bAO2Ym2n1T2rDvNBOVVbMWxXIRo2c18JH6/c2fcKT1bRKxsG7bYq69+n9IHmKedY6rmTU1VOZZdtSTXh7exMsl67IAcnCZ0/ec+P+ZMpkK5v4X8d27rbEigGqqHayGe1/lG2afzgcHY7QxjJ/x5O4yGmVVs8wdzdupke19U+M8Z/x0FcYIfTAHuXcaydEL+h/w/ppcuNarD2ggo2BxdWeOGLx5GOin1yruVfvDAazPEuI0m3yEwXQNZ4e0ar2G0jDCZpAJPJcJRRVttBMwPoAvzTwySUx3qI1eNSJRAH+bk";

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        /**
         * Load the data set containing stones
         * @see VuMarkInstanceId
         */
        VuforiaTrackables stonesAndChips = this.vuforia.loadTrackablesFromAsset("StonesAndChips");
        VuforiaTrackable stones = stonesAndChips.get(0);
        stones.setName("Stones");  // Stones


        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.add(stones);

        float mmPerInch        = 25.4f;
        float mmBotWidth       = 14 * mmPerInch; //currently not in use but could be useful in the future

        //Position Variables
        //stones
        float stonesTransX = 0;
        float stonesTransY = 1828;
        float stonesTransZ = 0;

        float stonesRotX = 90;
        float stonesRotY = 0;
        float stonesRotZ = 0;

        //phones
        float phoneTransX = 0;
        float phoneTransY = 0;
        float phoneTransZ = 0;

        float phoneRotX = 90;
        float phoneRotY = 0;
        float phoneRotZ = 0;


        //Stones Location
        OpenGLMatrix stonesLoc = OpenGLMatrix
                .translation(stonesTransX , stonesTransY, stonesTransZ)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XYZ,
                        AngleUnit.DEGREES, stonesRotX, stonesRotY, stonesRotZ));

        stones.setLocation(stonesLoc);

        //Phone Location
        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
                .translation(phoneTransX ,phoneTransY, phoneTransZ)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XYZ,
                        AngleUnit.DEGREES, phoneRotX, phoneRotY, phoneRotZ));


        //Giving the phone location to all the trackables
        ((VuforiaTrackableDefaultListener)stones.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);


        telemetry.addData(">", "Press Play to start");
        telemetry.update();
        waitForStart();

        stonesAndChips.activate();

        while (opModeIsActive()) {

            for (VuforiaTrackable trackable : allTrackables)
            {
                //Printing the robot location
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null)
                {
                    telemetry.addData("Current Location Found: " ,true );
                    VectorF translation= robotLocationTransform.getTranslation();
                    Orientation rotation = Orientation.getOrientation(robotLocationTransform, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
                    double tX = translation.get(0);
                    double tY = translation.get(1);
                    double tZ= translation.get(2);
                    telemetry.addData("Translation: " , "(" + tX + ", " + tY + ", " + tZ + ")");

                    double rX = rotation.firstAngle;
                    double rY = rotation.secondAngle;
                    double rZ = rotation.thirdAngle;
                    telemetry.addData("Rotation: " , "(" + rX + ", " + rY + ", " + rZ + ")");

                    lastLocation = robotLocationTransform;
                }

            }

            telemetry.update();


            //Driving
            /*
            The controls are as follows:
            Left joystick for forwards and backwards
            Right joystick for turning right or left
            a button for emergency stop
             */
            if(gamepad1.left_stick_y != 0)
            {
                leftDrive.setPower(gamepad1.left_stick_y/2);
                rightDrive.setPower(gamepad1.left_stick_y/2);
            }
            else if(gamepad1.right_stick_x != 0)
            {
                leftDrive.setPower(gamepad1.right_stick_x/2);
                rightDrive.setPower(-gamepad1.right_stick_x/2);
            }
            else
            {
                leftDrive.setPower(0.0);
                rightDrive.setPower(0.0);
            }

            if(gamepad1.a == true)
            {
                leftDrive.setPower(0.0);
                rightDrive.setPower(0.0);
            }
        }
    }

    //To avoid confusion, I am not using this method as I do not understand fully the order in which the translational and rotational
    //information is outputed. Therefore while testing, I am manually outputing the position instead of using this method
    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }

    //Currently not in use but may be useful for certain kinds of targets
    public boolean isVisible(VuforiaTrackable target)
    {
        VuforiaTrackableDefaultListener listener = (VuforiaTrackableDefaultListener)target.getListener();


        if(target != null && listener != null && listener.isVisible())
        {
            return true;
        }
        else
        {
            return false;
        }
    }

}

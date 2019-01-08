/*
Copyright (c) 2018 FIRST

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of FIRST nor the names of its contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode.concepttest;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * {@link SensorDistance} illustrates how to use the REV Robotics
 * Time-of-Flight Range Sensor.
 *
 * The op mode assumes that the range sensor is configured with a name of "sensor_range".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 *
 * @see <a href="http://revrobotics.com">REV Robotics Web Page</a>
 */
@TeleOp(name = "Sensor: DistanceSensors", group = "ConceptTests")
@Disabled
public class SensorDistance extends LinearOpMode {

    private DistanceSensor sensorRangeFrontLeft;
    private DistanceSensor sensorRangeFrontRight;
    //Range Sensors
    private ModernRoboticsI2cRangeSensor leftRangeSensor;
    private ModernRoboticsI2cRangeSensor rightRangeSensor;


    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    double speed = 0.3;

    @Override
    public void runOpMode() {
        // you can use this as a regular DistanceSensor.
        sensorRangeFrontLeft = hardwareMap.get(DistanceSensor.class, "sensorRangeFrontLeft");
       // sensorRangeFrontRight = hardwareMap.get(DistanceSensor.class, "sensorRangeFrontRight");

        //range sensor
        leftRangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "left_mr_range_sensor");
        rightRangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "right_mr_range_sensor");


        leftDrive = hardwareMap.get(DcMotor.class, "leftDrive");
        rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");

        // you can also cast this to a Rev2mDistanceSensor if you want to use added
        // methods associated with the Rev2mDistanceSensor class.
        Rev2mDistanceSensor sensorRangeFrontLeftTimeOfFlight = (Rev2mDistanceSensor)sensorRangeFrontLeft;
     //   Rev2mDistanceSensor sensorRangeFrontRightTimeOfFlight = (Rev2mDistanceSensor)sensorRangeFrontRight;



        telemetry.addData(">>", "Press start to continue");
        telemetry.update();

        waitForStart();
        while(opModeIsActive()) {

            // generic DistanceSensor methods.
            /*telemetry.addData("deviceName",sensorRangeFrontLeft.getDeviceName() );
            telemetry.addData("range", String.format("%.01f mm", sensorRangeFrontLeft.getDistance(DistanceUnit.MM)));
            telemetry.addData("range", String.format("%.01f cm", sensorRangeFrontLeft.getDistance(DistanceUnit.CM)));
            telemetry.addData("range", String.format("%.01f m", sensorRangeFrontLeft.getDistance(DistanceUnit.METER)));
            telemetry.addData("range", String.format("%.01f in", sensorRangeFrontLeft.getDistance(DistanceUnit.INCH)));

            // Rev2mDistanceSensor specific methods.
            telemetry.addData("ID", String.format("%x", sensorRangeFrontLeftTimeOfFlight.getModelID()));
            telemetry.addData("did time out", Boolean.toString(sensorRangeFrontLeftTimeOfFlight.didTimeoutOccur()));

            telemetry.addData("deviceName",sensorRangeFrontRight.getDeviceName() );
            telemetry.addData("range", String.format("%.01f mm", sensorRangeFrontRight.getDistance(DistanceUnit.MM)));
            telemetry.addData("range", String.format("%.01f cm", sensorRangeFrontRight.getDistance(DistanceUnit.CM)));
            telemetry.addData("range", String.format("%.01f m", sensorRangeFrontRight.getDistance(DistanceUnit.METER)));
            telemetry.addData("range", String.format("%.01f in", sensorRangeFrontRight.getDistance(DistanceUnit.INCH)));

            // Rev2mDistanceSensor specific methods.
            telemetry.addData("ID", String.format("%x", sensorRangeFrontRightTimeOfFlight.getModelID()));
            telemetry.addData("did time out", Boolean.toString(sensorRangeFrontRightTimeOfFlight.didTimeoutOccur()));

            telemetry.update();
         */
            telemetry.addData("leftRangeSensor Distance", String.format("%.01f cm", leftRangeSensor.getDistance(DistanceUnit.CM)));
            //telemetry.addData("cm optical", "%.2f cm", leftRangeSensor.cmOptical());
            telemetry.addData("rightRangeSensor Distance", String.format("%.01f cm", rightRangeSensor.getDistance(DistanceUnit.CM)));
            //telemetry.addData("cm optical", "%.2f cm", rightRangeSensor.cmOptical());

            telemetry.update();


            if(gamepad1.left_stick_y > 0)
            {
                rightDrive.setPower(gamepad1.left_stick_y * speed);
                leftDrive.setPower(gamepad1.left_stick_y * -speed);
            }

            else if (gamepad1.left_stick_y < 0)
            {
                rightDrive.setPower(-gamepad1.left_stick_y * -speed);
                leftDrive.setPower(-gamepad1.left_stick_y * speed);
            }

            else if(gamepad1.right_stick_x > 0) {
                rightDrive.setPower(gamepad1.right_stick_x * speed);
                leftDrive.setPower(gamepad1.right_stick_x * speed);
            }

            else if(gamepad1.right_stick_x<0){
                rightDrive.setPower(-gamepad1.right_stick_x * -speed);
                leftDrive.setPower(-gamepad1.right_stick_x * -speed);
            }
            else{
                rightDrive.setPower(0);
                leftDrive.setPower(0);
            }

        }
    }

}
package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;

import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;


@TeleOp(name="DriverControlledMode", group ="Tests")

public class DriverControlledMode extends LinearOpMode {

    private RoverRobot roverRuckusBot = new RoverRobot();
    private ElapsedTime runtime = new ElapsedTime();

    private static final double     WHEEL_SPEED       = 1.0;
    private static final double     LIFT_UP_SPEED     = 0.6;
    private static final double     LIFT_DOWN_SPEED   = 0.6;
    private static final float      mmPerInch         = 25.4f;
    private static final double     SHOULDER_SPEED    = 1.0;
    boolean  targetVisible = false;
    OpenGLMatrix  lastLocation = null;
    List<VuforiaTrackable> allTrackables = null;

    @Override public void runOpMode() {

        roverRuckusBot.initRobot(hardwareMap);
        allTrackables =   roverRuckusBot.getRobotLocation().getAllTrackables();

        telemetry.addData("Press Play to Start", "");
        telemetry.update();
        waitForStart();

        while (opModeIsActive())
        {
                getCurrentPosition();

                if (gamepad1.dpad_up == true) {
                    roverRuckusBot.getLiftAssembly().liftUpRobot(LIFT_UP_SPEED);
                } else if (gamepad1.dpad_down == true) {
                    roverRuckusBot.getLiftAssembly().lowerRobot(LIFT_DOWN_SPEED);
                } else {
                    roverRuckusBot.getLiftAssembly().resetLift();
                }

                //forwards
                if (gamepad1.left_stick_y > 0) {
                    roverRuckusBot.getChassisAssembly().moveForward(WHEEL_SPEED * gamepad1.left_stick_y);
                }
                //backwards
                else if (gamepad1.left_stick_y < 0) {
                    roverRuckusBot.getChassisAssembly().moveBackwards(WHEEL_SPEED * -gamepad1.left_stick_y);
                }
                //turn right
                else if (gamepad1.left_stick_x > 0) {
                    roverRuckusBot.getChassisAssembly().turnRight(WHEEL_SPEED * gamepad1.left_stick_x);
                }
                //turn left
                else if (gamepad1.left_stick_x < 0) {
                    roverRuckusBot.getChassisAssembly().turnLeft(WHEEL_SPEED * -gamepad1.left_stick_x);
                }
                //side right
                else if (gamepad1.right_trigger > 0) {
                    roverRuckusBot.getChassisAssembly().sideWaysRight(WHEEL_SPEED * gamepad1.right_trigger);
                }
                //side left
                else if (gamepad1.left_trigger > 0) {
                    roverRuckusBot.getChassisAssembly().sideWaysLeft(WHEEL_SPEED * gamepad1.left_trigger);
                } else {
                    roverRuckusBot.getChassisAssembly().stopMoving();
                }

                double power = 0.7;
                //intake wheels
                if (gamepad1.x == true)
                {
                    while (power < 1)
                    {
                        roverRuckusBot.getArmAssembly().Intake(-power);
                        power = power + 0.1;
                    }
                    while (power > 0.7) {
                        roverRuckusBot.getArmAssembly().Intake(-power);
                        power = power - 0.1;
                    }
                }
                else
                {
                    roverRuckusBot.getArmAssembly().Intake(0);
                }

                //   if (gamepad1.y == true)
                // {
                //   roverRuckusBot.getArmAssembly().OpenArm(0.7,0.5 );
                //}

                if (gamepad1.y == true) {
                    roverRuckusBot.getLiftAssembly().unlockRobot(0);

                }
                if (gamepad1.a== true) {
                    roverRuckusBot.getLiftAssembly().lockRobot(0.5);

                }
        }
    }

    public void getCurrentPosition()
    {
        targetVisible = false;
        for (VuforiaTrackable trackable : allTrackables)
        {
            if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible())
            {
                telemetry.addData("Visible Target", trackable.getName());
                targetVisible = true;

                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null)
                {
                    lastLocation = robotLocationTransform;
                }
                break;
            }
        }

        if (targetVisible)
        {
            //Get the Translational Infomration in Inches
            VectorF translation = lastLocation.getTranslation();
            telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                    translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

            //Getthe Rotational Information in Degrees
            Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
            telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
        }
        else
        {
            telemetry.addData("Visible Target", "none");
        }
        telemetry.update();
    }
}

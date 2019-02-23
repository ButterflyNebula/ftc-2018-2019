package org.firstinspires.ftc.teamcode.qualifier;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="DistanceSensor", group ="Tests")
public class DistanceSensorTest extends LinearOpMode
{

    private RoverRobot robot = new RoverRobot();

    final double distanceBetweenSensors = 9;

    @Override public void runOpMode()
    {
        robot.initRobot(hardwareMap);


        telemetry.addData("Press Play to Start", "");
        telemetry.update();

        while (!opModeIsActive() && !isStopRequested())
        {
            telemetry.addData("status" , "waiting for start command...");
            telemetry.update();
        }

        while(opModeIsActive())
        {
            double frontDistance = robot.getChassisAssembly().robotHardware.frontDepotDistanceSensor.getDistance(DistanceUnit.INCH);
            double backDistance = robot.getChassisAssembly().robotHardware.backDepotDistanceSensor.getDistance(DistanceUnit.INCH);

            telemetry.addData("Front Distance:", frontDistance);
            telemetry.addData("Back Distance" , backDistance);


            double differenceInDistance = frontDistance - backDistance;

            double angle = Math.asin(differenceInDistance/distanceBetweenSensors);
            angle = Math.toDegrees(angle);

            double distance = robot.getChassisAssembly().robotHardware.laserDistanceSensor.getDistance(DistanceUnit.INCH);

            telemetry.addData("Angle " , angle);
            telemetry.addData("Distance", distance);

            telemetry.update();
        }
    }
}

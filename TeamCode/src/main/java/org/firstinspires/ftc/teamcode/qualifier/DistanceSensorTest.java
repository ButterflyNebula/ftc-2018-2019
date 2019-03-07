package org.firstinspires.ftc.teamcode.qualifier;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="DistanceSensor", group ="Tests")
@Disabled
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
            double frontDepotDistance = robot.getChassisAssembly().robotHardware.frontDepotDistanceSensor.getDistance(DistanceUnit.INCH);
            double backDepotDistance = robot.getChassisAssembly().robotHardware.backDepotDistanceSensor.getDistance(DistanceUnit.INCH);

            telemetry.addData("Front Depot Distance:", frontDepotDistance);
            telemetry.addData("Back Depot Distance" , frontDepotDistance);


            double differenceInDepotDistance = frontDepotDistance - backDepotDistance;

            double angleDepot = Math.atan(differenceInDepotDistance/distanceBetweenSensors);
            angleDepot = Math.toDegrees(angleDepot);

            double distance = robot.getChassisAssembly().robotHardware.laserDistanceSensor.getDistance(DistanceUnit.INCH);


            telemetry.addData("Angle (depot) " , angleDepot);

            telemetry.addData("Distance", distance);


            //Crater
            double frontCraterDistance = robot.getChassisAssembly().robotHardware.frontCraterDistanceSensor.getDistance(DistanceUnit.INCH);
            double backCraterDistance = robot.getChassisAssembly().robotHardware.backCraterDistanceSensor.getDistance(DistanceUnit.INCH);

            telemetry.addData("Front Crater Distance:", frontCraterDistance);
            telemetry.addData("Back Crater Distance" , backCraterDistance);


            double differenceInCraterDistance = frontCraterDistance - backCraterDistance;

            double angleCrater = Math.atan(differenceInCraterDistance/distanceBetweenSensors);
            angleCrater = Math.toDegrees(angleCrater);

            telemetry.addData("Angle (crater) " , angleCrater);


            telemetry.update();
        }
    }
}

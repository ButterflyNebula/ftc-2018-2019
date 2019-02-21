package org.firstinspires.ftc.teamcode.qualifier;



import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


public class Navigation
{
    RobotHardware robotHardware;

    protected Navigation(RobotHardware hardware)
    {
        robotHardware = hardware;
    }


    protected double getCraterFrontDistance()
    {
        return robotHardware.frontDistanceSensor.getDistance(DistanceUnit.INCH);
    }

    protected double getCraterBackDistance()
    {
        return robotHardware.backDistanceSensor.getDistance(DistanceUnit.INCH);
    }

    protected double getCraterAngle()
    {
        final double distanceBetweenSensors = 9;

        double frontDistance = getCraterFrontDistance();
        double backDistance = getCraterBackDistance();


        double differenceInDistance = frontDistance - backDistance;

        double angle = Math.asin(differenceInDistance/distanceBetweenSensors);
        angle = Math.toDegrees(angle);

        return angle;
    }

    protected double getCraterDistance()
    {
        double frontDistance = getCraterFrontDistance();
        double backDistance = getCraterBackDistance();

        return 0.5 * (frontDistance + backDistance);
    }

}

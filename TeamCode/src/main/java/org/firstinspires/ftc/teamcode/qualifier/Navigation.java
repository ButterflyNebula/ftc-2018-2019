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
        return robotHardware.frontCraterDistanceSensor.getDistance(DistanceUnit.INCH);
    }

    protected double getCraterBackDistance()
    {
        return robotHardware.backCraterDistanceSensor.getDistance(DistanceUnit.INCH);
    }

    protected double getCraterAngle()
    {
        final double distanceBetweenSensors = 9;

        double frontDistance = getCraterFrontDistance();
        double backDistance = getCraterBackDistance();


        double differenceInDistance = frontDistance - backDistance;

        double angle = Math.atan(differenceInDistance/distanceBetweenSensors);
        angle = Math.toDegrees(angle);

        return angle;
    }

    protected double getCraterDistance()
    {
        double frontDistance = getCraterFrontDistance();
        double backDistance = getCraterBackDistance();

        return 0.5 * (frontDistance + backDistance);
    }

    protected double getDepotFrontDistance()
    {
        return robotHardware.frontDepotDistanceSensor.getDistance(DistanceUnit.INCH);
    }

    protected double getDepotBackDistance()
    {
        return robotHardware.backDepotDistanceSensor.getDistance(DistanceUnit.INCH);
    }

    protected double getDepotAngle()
    {
        final double distanceBetweenSensors = 9;

        double frontDistance = getDepotFrontDistance();
        double backDistance = getDepotBackDistance();


        double differenceInDistance = frontDistance - backDistance;

        double angle = Math.atan(differenceInDistance/distanceBetweenSensors);
        angle = Math.toDegrees(angle);

        return angle;
    }

    protected double getDepotDistance()
    {
        double frontDistance = getDepotFrontDistance();
        double backDistance = getDepotBackDistance();

        return 0.5 * (frontDistance + backDistance);
    }

    protected double getFrontLaserDistance()
    {
        return robotHardware.frontLaserDistanceSensor.getDistance(DistanceUnit.INCH);
    }

    protected double getBackLaserDistance()
    {
        return robotHardware.backLaserDistanceSensor.getDistance(DistanceUnit.INCH);
    }

}

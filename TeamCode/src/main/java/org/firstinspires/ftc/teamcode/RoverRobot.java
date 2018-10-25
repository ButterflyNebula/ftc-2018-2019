package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

/**
 * Created by Athira on 10/14/2018.
 */

public class RoverRobot {

    private HardwareMap hardwareMap = null;
    private static RobotHardware robotHardware = null;
    private LiftAssembly liftAssembly = null;
    private ChassisAssembly chassisAssembly = null;
    private ArmAssembly armAssembly = null;
    private Location robotLocation = null;


    public void initRobot (HardwareMap hwMap)
    {
        this.hardwareMap = hwMap;
        robotHardware = new RobotHardware(hwMap);
        buildChassis();
        buildLiftAssembly();
        buildArmAssembly();
        setTargets();
    }

    public void buildChassis () {
        this.chassisAssembly = new ChassisAssembly(robotHardware);

    }
    public void buildLiftAssembly () {
        this.liftAssembly = new LiftAssembly(robotHardware);

    }
    public void buildArmAssembly () {
        this.armAssembly = new ArmAssembly(robotHardware);

    }


    public void setTargets () {
        this.robotLocation = new Location(robotHardware) ;
        robotLocation.initializeNavigation();

    }

    public LiftAssembly getLiftAssembly() {
        return liftAssembly;
    }

    public ChassisAssembly getChassisAssembly() {
        return chassisAssembly;
    }

    public ArmAssembly  getArmAssembly()  {return armAssembly;}

    public Location getRobotLocation() {
        return robotLocation;
    }
}

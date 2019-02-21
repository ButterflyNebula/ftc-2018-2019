package org.firstinspires.ftc.teamcode.qualifier;

import com.qualcomm.robotcore.hardware.HardwareMap;



/**
 * Created by Athira on 10/14/2018.
 */

public class RoverRobot
{

    private static RobotHardware robotHardware = null;
    private ChassisAssembly chassisAssembly= null;
    private ArmAssembly armAssembly= null;
    private LiftAssembly liftAssembly = null;
    private Navigation navigation = null;

    public void initRobot (HardwareMap hwMap)
    {
        robotHardware = new RobotHardware(hwMap);
        buildChassisAssembly();
        buildLiftAssembly();
        buildArmAssembly();
        buildNavigation();

    }

    public void buildChassisAssembly () {
        this.chassisAssembly = new ChassisAssembly(robotHardware);

    }
    public void buildLiftAssembly () {
        this.liftAssembly = new LiftAssembly(robotHardware);

    }

    public void buildArmAssembly () {
        this.armAssembly = new ArmAssembly(robotHardware);
    }

    public void buildNavigation() {this.navigation = new Navigation(robotHardware);}

    public LiftAssembly getLiftAssembly() { return liftAssembly;  }
    public ChassisAssembly getChassisAssembly()
    {
        return chassisAssembly;
    }
    public ArmAssembly  getArmAssembly()  {return armAssembly;}
    public Navigation getNavigation() {return navigation;}


}

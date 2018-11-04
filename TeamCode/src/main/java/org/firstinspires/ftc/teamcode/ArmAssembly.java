package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by Athira on 10/14/2018.
 */

public class ArmAssembly{

    RobotHardware robotHardware;

    protected ArmAssembly(RobotHardware hardware)
    {
        robotHardware = hardware;
    }


    protected void Intake(double power)
    {
        robotHardware.leftIntake.setPower(power);
        robotHardware.rightIntake.setPower(power);
    }

   /* protected void OpenArm (double wristPos, double elbowPos)
    {
        try {
          //  robotHardware.wrist.setPosition(wristPos);
            //Thread.sleep(1000);
            robotHardware.elbow.setPosition(elbowPos);
            //wristPos= wristPos - 0.4;
            elbowPos = elbowPos - 0.3;
            //Thread.sleep(1000);
            //robotHardware.wrist.setPosition(wristPos);
            Thread.sleep(1000);
            robotHardware.elbow.setPosition(elbowPos);
            }
            catch(Exception e)
            {

            }

    }
*/

}

package org.firstinspires.ftc.teamcode.outreach;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;



/**
 * This class contains various methods which are used to control the robot
 */

public class RobotControl
{

    HardwareMap hardwareMap = null;
    public static RobotHardware robot = null;

    //Constructor
    public RobotControl(HardwareMap map, RobotHardware input_robot)
    {
        hardwareMap = map;
        robot = input_robot;
    }


    /**
     *-----------------------------------------------------------------------------------
     * Gripper Functions
     * Used for grabbing objects insides of claw machine
     *  * ----------------------------------------------------------------------------------
     */

    public void closeHand(double servoPos)
    {
        robot.gripper.setPosition(servoPos);
    }

    public void openHand(double servoPos)
    {
        robot.gripper.setPosition(servoPos);
    }
    /**
     *-----------------------------------------------------------------------------------
     * Positioning CONTROLS
     * This is used to move the gripper around.  The two directions of movement allow the gripper
     * to cover all areas
     * ----------------------------------------------------------------------------------
     */

    /**
     * Moves the gripper  forwards by powering the vertical motor
     */
    public void moveBackwards(double  speed)
    {
        robot.baseMotor.setPower(speed);
    }

    /**
     * Moves the gripper backwards by giving negative power the same motor
     */
    public void moveForward(double speed)
    {
        robot.baseMotor.setPower(-speed);
    }

    /**
     * Rotates the gripper to the right by giving the secondary motor power
     * @param speed at which the wheel motors will turn
     */
    public void moveLeft (double speed)
    {
        robot.secondaryMotor.setPower(speed);
    }

    /**
     * Rotates the robot to the left by giving the secondary motor in the opposite direction
     * @param speed at which the wheel motors will turn
     */
    public void moveRight (double speed)
    {
        robot.secondaryMotor.setPower(-speed);
    }

    /**
     * Lowers the gripper to grab items by rotating the top motor
     *  * @param speed at which the wheel motors will turn
     */
    public void drop (double speed)
    {
        robot.dropper.setPower(-speed);
    }
    /**
     * Raises the gripper after item is grabbed
     *  * @param speed at which the wheel motors will turn
     */
    public void lift (double speed)
    {
        robot.dropper.setPower(speed);
    }
    /**
     * Stops the robot by setting the power of all wheels to 0
     */
    public void baseStop()
    {
        robot.baseMotor.setPower(0);
    }
    public void sideStop()
    {
        robot.secondaryMotor.setPower(0);
    }
    public void dropStop()
    {
        robot.dropper.setPower(0);
    }

    public void closeHand()
    {
        robot.gripper.setPosition(0.4);
    }

    public void openHand()
    {
        robot.gripper.setPosition(1);
    }


}


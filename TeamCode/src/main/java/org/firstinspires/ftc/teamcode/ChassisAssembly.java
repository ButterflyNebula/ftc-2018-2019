package org.firstinspires.ftc.teamcode;

/**
 * Created by Athira on 10/14/2018.
 */

public class ChassisAssembly{

    double driveSpeed = 0.3;
    RobotHardware robotHardware;

    protected ChassisAssembly(RobotHardware hardware)
    {
        robotHardware = hardware;
    }

  
    /**
     *-----------------------------------------------------------------------------------
     * WHEEL CONTROLS
     * This is used to move the robot around.  The robot can move in all directions with
     * the use of Mecanum wheels
     * ----------------------------------------------------------------------------------
     */

    /**
     * Moves the robot forward by giving power to all four wheels
     * @param speed at which the wheel motors will turn
     */
    protected void moveForward(double  speed)
    {
        robotHardware.frontLeftWheel.setPower(speed);
        robotHardware.frontRightWheel.setPower(speed);
        robotHardware.backLeftWheel.setPower(speed);
        robotHardware.backRightWheel.setPower(speed);
    }

    /**
     * Moves the robot backwards by giving negative power to all four wheels
     * @param speed at which the wheel motors will turn
     */
    protected void moveBackwards(double speed)
    {
        robotHardware.frontLeftWheel.setPower(-speed);
        robotHardware.frontRightWheel.setPower(-speed);
        robotHardware.backLeftWheel.setPower(-speed);
        robotHardware.backRightWheel.setPower(-speed);
    }

    /**
     * Rotates the robot to the right by giving the left wheels power and the right
     * wheels negative power
     * @param speed at which the wheel motors will turn
     */
    protected void turnRight (double speed)
    {
        robotHardware.frontLeftWheel.setPower(-speed);
        robotHardware.backLeftWheel.setPower(-speed);
        robotHardware.frontRightWheel.setPower(speed);
        robotHardware.backRightWheel.setPower(speed);
    }

    /**
     * Rotates the robot to the left by giving the right wheels power and the left
     * wheels negative power
     * @param speed at which the wheel motors will turn
     */
    protected void turnLeft (double speed)
    {
        robotHardware.frontLeftWheel.setPower(speed);
        robotHardware.backLeftWheel.setPower(speed);
        robotHardware.frontRightWheel.setPower(-speed);
        robotHardware.backRightWheel.setPower(-speed);
    }

    /**
     * Moves the robot to the right laterally.  The front left wheel and back right wheel
     * are given negative power and the front right wheel and back left wheel are given power.
     * @param speed at which the wheel motors will turn
     */
    protected void moveRight (double speed)
    {
        robotHardware.frontLeftWheel.setPower(speed);
        robotHardware.backLeftWheel.setPower(-speed);
        robotHardware.frontRightWheel.setPower(-speed);
        robotHardware.backRightWheel.setPower(speed);
    }

    /**
     * Moves the robot to the left laterally.  The front right wheel and back left wheel
     * are given negative power and the front left wheel and back right wheel are given power.
     * @param speed at which the wheel motors will turn
     */
    protected void moveLeft (double speed)
    {
        robotHardware.frontLeftWheel.setPower(-speed);
        robotHardware.backLeftWheel.setPower(speed);
        robotHardware.frontRightWheel.setPower(speed);
        robotHardware.backRightWheel.setPower(-speed);
    }

    /**
     * Stops the robot by setting the power of all wheels to 0
     */
    protected void stopMoving()
    {
        robotHardware.frontLeftWheel.setPower(0);
        robotHardware.frontRightWheel.setPower(0);
        robotHardware.backLeftWheel.setPower(0);
        robotHardware.backRightWheel.setPower(0);
    }
}

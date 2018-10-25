package org.firstinspires.ftc.teamcode;

/**
 * Created by Athira on 10/14/2018.
 */


import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public class RobotHardware
{
    //Defining the motor

    public DcMotor frontLeftWheel = null;
    public DcMotor frontRightWheel = null;
    public DcMotor backLeftWheel = null;
    public DcMotor backRightWheel = null;
    public DcMotor lift = null;
    public Servo   phoneSwivel = null;
    public CRServo leftIntake = null;
    public CRServo rightIntake = null;
   // public DcMotor shoulder = null;
   // public Servo   elbow = null;
   // public Servo   wrist = null;
    public Servo   liftLock = null;

    //Adding the Hardware Map
    private HardwareMap hwMap  = null;

    public  RobotHardware(HardwareMap ahwMap)
    {
        hwMap = ahwMap;
        //Initialize the wheel motors
        frontLeftWheel = hwMap.get(DcMotor.class, "frontLeft");
        frontRightWheel = hwMap.get(DcMotor.class, "frontRight");
        backLeftWheel = hwMap.get(DcMotor.class, "backLeft");
        backRightWheel = hwMap.get(DcMotor.class, "backRight");

        frontRightWheel.setDirection(DcMotor.Direction.REVERSE);
        backRightWheel.setDirection(DcMotor.Direction.REVERSE);
        frontLeftWheel.setDirection(DcMotor.Direction.FORWARD);
        backLeftWheel.setDirection(DcMotor.Direction.FORWARD);

        //Initialize the lift motor and lock
        lift = hwMap.get(DcMotor.class, "lift");
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setDirection(DcMotor.Direction.FORWARD);
        liftLock = hwMap.get(Servo.class, "liftLock");

        //Initialize the intake motor
        leftIntake = hwMap.get(CRServo.class, "leftIntake");
        rightIntake = hwMap.get(CRServo.class, "rightIntake");
        leftIntake.setDirection(CRServo.Direction.FORWARD);
        rightIntake.setDirection(CRServo.Direction.REVERSE);

        //Initializing the Arm motors
       // shoulder = hwMap.get(DcMotor.class, "shoulder");
       // elbow  = hwMap.get(Servo.class, "elbow");
       // wrist = hwMap.get(Servo.class, "wrist");

    }

    public HardwareMap getHwMap() {
        return hwMap;
    }
}

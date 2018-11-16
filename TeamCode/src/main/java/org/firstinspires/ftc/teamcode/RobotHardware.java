package org.firstinspires.ftc.teamcode;

/**
 * Created by Athira on 10/14/2018.
 */


import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DigitalChannel;


public class RobotHardware
{
    //Defining the motor

    public DcMotor frontLeftWheel = null;
    public DcMotor frontRightWheel = null;
    public DcMotor backLeftWheel = null;
    public DcMotor backRightWheel = null;
    public DcMotor lift = null;
    public DcMotor mineralLift = null;
    public Servo   phoneSwivel = null;
    public CRServo leftIntake = null;
    public CRServo rightIntake = null;
    public DcMotor mineralArm = null;
    public Servo   leftSortServo = null;
    public Servo   rightSortServo = null;
    public Servo   liftLock = null;
    public Servo   goldBasket = null;
    public Servo silverBasket = null;
    public DigitalChannel bottomTouch = null;
    public DigitalChannel topTouch = null;
    public DigitalChannel backTouch = null;
    public DigitalChannel frontTouch = null;

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

        //Initialize the mineral lift motor and basket servos
        mineralLift = hwMap.get(DcMotor.class, "mineralLift");
        goldBasket = hwMap.get(Servo.class, "goldBasket");
        silverBasket = hwMap.get(Servo.class, "silverBasket");

        //Initializing Touch Sensors
        bottomTouch = hwMap.get(DigitalChannel.class, "bottomTouch");
        topTouch = hwMap.get(DigitalChannel.class, "topTouch");
        backTouch = hwMap.get(DigitalChannel.class, "backTouch");
        frontTouch = hwMap.get(DigitalChannel.class, "frontTouch");

        //Initialize the intake motor
        leftIntake = hwMap.get(CRServo.class, "leftIntake");
        rightIntake = hwMap.get(CRServo.class, "rightIntake");
        leftIntake.setDirection(CRServo.Direction.FORWARD);
        rightIntake.setDirection(CRServo.Direction.REVERSE);

        //Initializing the Arm motors
        mineralArm = hwMap.get(DcMotor.class, "mineralArm");
        leftSortServo = hwMap.get(Servo.class, "leftSortServo");
        rightSortServo = hwMap.get(Servo.class, "rightSortServo");
    }

    public HardwareMap getHwMap() {
        return hwMap;
    }
}

package org.firstinspires.ftc.teamcode.qualifier;

/**
 * Created by Athira on 10/14/2018.
 */


import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public class RobotHardware
{
    //Chassis
    public DcMotor frontLeftWheel = null;
    public DcMotor frontRightWheel = null;
    public DcMotor backLeftWheel = null;
    public DcMotor backRightWheel = null;

    //Lift
    public DcMotor robotLift = null;
    public DigitalChannel topTouch = null;

    //Arm
    public CRServo intakeSlides = null;
    public DcMotor deliveryLift = null;
    public DcMotor mineralWrist = null;
    public DcMotor intakeMineral = null;
    public Servo flipper = null;

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

        frontRightWheel.setDirection(DcMotor.Direction.FORWARD);
        backRightWheel.setDirection(DcMotor.Direction.FORWARD);
        frontLeftWheel.setDirection(DcMotor.Direction.REVERSE);
        backLeftWheel.setDirection(DcMotor.Direction.REVERSE);

        frontLeftWheel.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));
        backLeftWheel.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));
        frontRightWheel.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));
        backRightWheel.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));


        //Initialize the lift motor and sensors
        robotLift = hwMap.get(DcMotor.class, "robotLift");
        robotLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robotLift.setDirection(DcMotor.Direction.FORWARD);
        topTouch = hwMap.get(DigitalChannel.class, "topTouch");


        //Arm
        intakeSlides = hwMap.get(CRServo.class , "intakeSlides");
        deliveryLift = hwMap.get(DcMotor.class , "deliveryLift");
        deliveryLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mineralWrist = hwMap.get(DcMotor.class , "mineralWrist");
        intakeMineral = hwMap.get(DcMotor.class , "intakeMotor");
        intakeMineral.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMineral.setDirection(DcMotor.Direction.FORWARD);
        flipper = hwMap.get(Servo.class, "flipper");

    }

    public HardwareMap getHwMap() {
        return hwMap;
    }
}

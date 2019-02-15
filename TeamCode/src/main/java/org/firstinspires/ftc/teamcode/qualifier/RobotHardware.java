package org.firstinspires.ftc.teamcode.qualifier;

/**
 * Created by Athira on 10/14/2018.
 */


import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorMRRangeSensor;


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
    public DcMotor intakeSlides = null;
    public DcMotor deliveryLift = null;
    public CRServo mineralWrist = null;
    public DcMotor intakeMineral = null;
    public Servo flipper = null;

    //Touch Sensors
    public DigitalChannel wristTouch = null;
    public DigitalChannel backTouch = null;
    public DigitalChannel deliveryTouch = null;
    public DigitalChannel wristTouchDown = null;

    //Distance Sensors
    public ModernRoboticsI2cRangeSensor frontDistanceSensor = null;
    public ModernRoboticsI2cRangeSensor backDistanceSensor = null;



    //Adding the Hardware Map
    private HardwareMap hwMap  = null;

    public  RobotHardware(HardwareMap ahwMap)
    {
        hwMap = ahwMap;


        //Wheel motors
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

        //Lift
        robotLift = hwMap.get(DcMotor.class, "robotLift");
        robotLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robotLift.setDirection(DcMotor.Direction.FORWARD);
        topTouch = hwMap.get(DigitalChannel.class, "topTouch");

        //Arm
        intakeSlides = hwMap.get(DcMotor.class , "intakeSlides");
        deliveryLift = hwMap.get(DcMotor.class , "deliveryLift");
        deliveryLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mineralWrist = hwMap.get(CRServo.class , "mineralWrist");
        intakeMineral = hwMap.get(DcMotor.class , "intakeMotor");
        intakeMineral.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMineral.setDirection(DcMotor.Direction.FORWARD);
        flipper = hwMap.get(Servo.class, "flipper");

        //Touch Sensors
        wristTouch = hwMap.get(DigitalChannel.class, "wristTouch");
        backTouch = hwMap.get(DigitalChannel.class, "backTouch");
        deliveryTouch = hwMap.get(DigitalChannel.class, "deliveryTouch");
        wristTouchDown = hwMap.get(DigitalChannel.class , "touchDown");


        //Distance Sensors
        frontDistanceSensor = hwMap.get(ModernRoboticsI2cRangeSensor.class , "frontDistanceSensor");
        backDistanceSensor = hwMap.get(ModernRoboticsI2cRangeSensor.class, "backDistanceSensor");



    }

    public HardwareMap getHwMap() {
        return hwMap;
    }
}

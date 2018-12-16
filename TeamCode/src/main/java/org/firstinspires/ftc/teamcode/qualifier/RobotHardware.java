package org.firstinspires.ftc.teamcode.qualifier;

/**
 * Created by Athira on 10/14/2018.
 */


import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;



public class RobotHardware
{
    //Chassis
    public DcMotor frontLeftWheel = null;
    public DcMotor frontRightWheel = null;
    public DcMotor backLeftWheel = null;
    public DcMotor backRightWheel = null;

    //Lift
    public DcMotor robotLift = null;

    //Arm
    public CRServo intakeSlides = null;
    public DcMotor deliveryLift = null;
    public DcMotor mineralWrist = null;

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

        //Initialize the lift motor and lock
        robotLift = hwMap.get(DcMotor.class, "robotLift");
        robotLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robotLift.setDirection(DcMotor.Direction.FORWARD);


        //Arm
        intakeSlides = hwMap.get(CRServo.class , "intakeSlides");
        deliveryLift = hwMap.get(DcMotor.class , "deliveryLift");
        mineralWrist = hwMap.get(DcMotor.class , "mineralWrist");

    }

    public HardwareMap getHwMap() {
        return hwMap;
    }
}

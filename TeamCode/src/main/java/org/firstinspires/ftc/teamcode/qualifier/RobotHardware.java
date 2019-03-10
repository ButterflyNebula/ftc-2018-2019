package org.firstinspires.ftc.teamcode.qualifier;

/**
 * Created by Athira on 10/14/2018.
 */


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
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

    //Distance Sensors
    public ModernRoboticsI2cRangeSensor frontCraterDistanceSensor = null;
    public ModernRoboticsI2cRangeSensor backCraterDistanceSensor = null;
    public ModernRoboticsI2cRangeSensor frontDepotDistanceSensor = null;
    public ModernRoboticsI2cRangeSensor backDepotDistanceSensor = null;

    public Rev2mDistanceSensor frontLaserDistanceSensor = null;
    public Rev2mDistanceSensor backLaserDistanceSensor = null;
    // The IMU sensor object
    public BNO055IMU imu = null;


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


        //Distance Sensors
        frontCraterDistanceSensor = hwMap.get(ModernRoboticsI2cRangeSensor.class , "frontCraterDistanceSensor");
        backCraterDistanceSensor = hwMap.get(ModernRoboticsI2cRangeSensor.class, "backCraterDistanceSensor");

        frontDepotDistanceSensor = hwMap.get(ModernRoboticsI2cRangeSensor.class, "frontDepotDistanceSensor");
        backDepotDistanceSensor = hwMap.get(ModernRoboticsI2cRangeSensor.class, "backDepotDistanceSensor");

        frontLaserDistanceSensor = hwMap.get(Rev2mDistanceSensor.class, "frontLaserDistanceSensor");
        backLaserDistanceSensor = hwMap.get(Rev2mDistanceSensor.class, "backLaserDistanceSensor");


        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "RoverRuckus13747.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

    }

    public HardwareMap getHwMap() {
        return hwMap;
    }
}

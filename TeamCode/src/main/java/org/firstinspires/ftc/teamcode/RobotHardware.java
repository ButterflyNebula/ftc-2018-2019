package org.firstinspires.ftc.teamcode;

/**
 * Created by Athira on 10/14/2018.
 */


import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;





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
    Rev2mDistanceSensor rev2mDistanceSensor;  // Hardware Device Object


    public BNO055IMU imu = null;

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

        frontRightWheel.setDirection(DcMotor.Direction.FORWARD);
        backRightWheel.setDirection(DcMotor.Direction.FORWARD);
        frontLeftWheel.setDirection(DcMotor.Direction.REVERSE);
        backLeftWheel.setDirection(DcMotor.Direction.REVERSE);

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


        rev2mDistanceSensor = hwMap.get(Rev2mDistanceSensor.class, "distance_sensor");


        //Initializing the Arm motors
       // shoulder = hwMap.get(DcMotor.class, "shoulder");
       // elbow  = hwMap.get(Servo.class, "elbow");
        //wrist = hwMap.get(Servo.class, "wrist");


        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "FTC13747.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        mineralArm = hwMap.get(DcMotor.class, "mineralArm");
        leftSortServo = hwMap.get(Servo.class, "leftSortServo");
        rightSortServo = hwMap.get(Servo.class, "rightSortServo");
    }

    public HardwareMap getHwMap() {
        return hwMap;
    }
}

package org.firstinspires.ftc.teamcode.outreach;



import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DigitalChannel;



import static android.os.SystemClock.sleep;


public class RobotHardware
{
    //Defining the motor

    public DcMotor baseMotor = null;
    public DcMotor secondaryMotor = null;
    public DcMotor dropper = null;
    public Servo gripper = null;
    public DigitalChannel sensorRight = null;
    public DigitalChannel sensorLeft = null;
    public DigitalChannel sensorBack = null;
    public DigitalChannel sensorFront = null;



    //Adding the Hardware Map
    public HardwareMap hwMap  = null;

    public void init(HardwareMap map)
    {
        hwMap = map;
        //Initialize the movement motors
        baseMotor = hwMap.get(DcMotor.class, "baseMotor");
        secondaryMotor = hwMap.get(DcMotor.class, "secondaryMotor");
        dropper = hwMap.get(DcMotor.class, "dropper");
        gripper= hwMap.get(Servo.class, "gripper");
        sensorRight= hwMap.get(DigitalChannel.class, "sensorRight");
        sensorRight.setMode(DigitalChannel.Mode.INPUT);
        sensorLeft= hwMap.get(DigitalChannel.class, "sensorLeft");
        sensorLeft.setMode(DigitalChannel.Mode.INPUT);
        sensorBack= hwMap.get(DigitalChannel.class, "sensorBack");
        sensorBack.setMode(DigitalChannel.Mode.INPUT);
        sensorFront= hwMap.get(DigitalChannel.class, "sensorFront");
        sensorFront.setMode(DigitalChannel.Mode.INPUT);

    }
}

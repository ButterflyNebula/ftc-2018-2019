package org.firstinspires.ftc.teamcode.concepttest;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.qualifier.RoverRobot;

@Autonomous(name="CVAuto", group ="Tests")
public class DogeCVAuto extends LinearOpMode{
    //Creating a Rover robot object
    RoverRobot robot = new RoverRobot();

    private ElapsedTime runtime = new ElapsedTime();

    private static final double WHEEL_SPEED = 0.4;

    @Override
    public void runOpMode()
    {
        robot.initRobot(hardwareMap);


    }
}

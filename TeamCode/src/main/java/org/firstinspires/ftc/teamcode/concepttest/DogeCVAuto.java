package org.firstinspires.ftc.teamcode.concepttest;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.qualifier.RoverRobot;

@Autonomous(name="CVAuto", group ="Tests")
public class DogeCVAuto extends LinearOpMode{
    //Creating a Rover robot object
    RoverRobot robot = new RoverRobot();

    private GoldDetector detector;

    private ElapsedTime runtime = new ElapsedTime();

    private static final double WHEEL_SPEED = 0.4;

    @Override
    public void runOpMode()
    {
        robot.initRobot(hardwareMap);
        // Set up detector
        detector = new GoldDetector(); // Create detector
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance()); // Initialize it with the app context and camera
        detector.useDefaults(); // Set detector to use default settings

        // Optional tuning
        detector.downscale = 0.4; // How much to downscale the input frames

        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.maxAreaScorer.weight = 0.005; //

        detector.ratioScorer.weight = 5; //
        detector.ratioScorer.perfectRatio = 1.0; // Ratio adjustment

        detector.enable(); // Start the detector!

        //Wait for Start
        telemetry.addData(">", "Press Play to begin");
        telemetry.update();
        while (!opModeIsActive() && !isStopRequested())
        {
            telemetry.addData("status" , "waiting for start command...");
            telemetry.update();
        }

        //releaseRobot();






        detector.disable();

    }

}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Michael Vierra, FTC 8461 on 9/13/2017.
 */

/*
Bytes    16-bit word    Description
        ----------------------------------------------------------------
        0, 1     y              sync: 0xaa55=normal object, 0xaa56=color code object
        2, 3     y              checksum (sum of all 16-bit words 2-6, that is, bytes 4-13)
        4, 5     y              signature number
        6, 7     y              x center of object
        8, 9     y              y center of object
        10, 11   y              width of object
        12, 13   y              height of object
        */

@Autonomous(name="pixy test", group ="tests")
@Disabled
public class TestPixy extends LinearOpMode {
   // I2cDeviceSynch pixy;

    I2cDevice pixy;
    I2cDeviceSynch pixyReader;
    I2cAddr pixyAddress = I2cAddr.create7bit(0x54);
    byte[] readCache;

    @Override
  /*public void runOpMode() throws InterruptedException {
        //setting up Pixy to the hardware map
        pixy = hardwareMap.i2cDeviceSynch.get("pixy");

        //setting Pixy's I2C Address
        pixy.setI2cAddress(I2cAddr.create7bit(0x54));

        //setting Pixy's read window. You'll want these exact parameters, and you can reference the
        //SDK Documentation to learn more
        I2cDeviceSynch.ReadWindow readWindow = new I2cDeviceSynch.ReadWindow (1, 26,
                I2cDeviceSynch.ReadMode.REPEAT);
        pixy.setReadWindow(readWindow);

        //required to "turn on" the device
        pixy.engage();


        waitForStart();


        while(opModeIsActive()) {
            //send every byte of data that we can to the phone screen
            telemetry.addData("Byte 0", pixy.read8(0));
            telemetry.addData("Byte 1", pixy.read8(1));
            telemetry.addData("Byte 2", pixy.read8(2));
            telemetry.addData("Byte 3", pixy.read8(3));
            telemetry.addData("Byte 4", pixy.read8(4));
            telemetry.addData("Byte 5", pixy.read8(5));
            telemetry.addData("Byte 6", pixy.read8(6));
            telemetry.addData("Byte 7", pixy.read8(7));
            telemetry.addData("Byte 8", pixy.read8(8));
            telemetry.addData("Byte 9", pixy.read8(9));
            telemetry.addData("Byte 10", pixy.read8(10));
            telemetry.addData("Byte 11", pixy.read8(11));
            telemetry.addData("Byte 12", pixy.read8(12));
            telemetry.addData("Byte 13", pixy.read8(13));
            telemetry.update();
        }


    }*/

    public void runOpMode() throws InterruptedException {
        pixy = hardwareMap.i2cDevice.get("pixy");
        pixyReader = new I2cDeviceSynchImpl(pixy, pixyAddress, false);
        pixyReader.engage();

        waitForStart();

        while(opModeIsActive()) {
            readCache = pixyReader.read(0x54, 100);

            telemetry.addData("1", readCache[1]);
            telemetry.addData("2", readCache[2]);
            telemetry.addData("6", readCache[6]);
            telemetry.addData("7", readCache[7]);
            telemetry.addData("8", readCache[8]);
            telemetry.addData("9", readCache[9]);
            telemetry.addData("isInReadMode", pixy.isI2cPortInReadMode());
            telemetry.update();
        }
    }
}
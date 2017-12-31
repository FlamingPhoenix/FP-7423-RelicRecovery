package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;

import org.firstinspires.ftc.teamcode.Library.PixyCam;

/**
 * Created by HwaA1 on 10/22/2017.
 */

public class PixyTest extends LinearOpMode {

    I2cDeviceSynch pix;

    PixyCam pixy;
    @Override
    public void runOpMode() throws InterruptedException {
        //pix = hardwareMap.get(I2cDeviceSynch.class, "pixy");

        pixy = hardwareMap.get(PixyCam.class, "pixy");

        waitForStart();

        while(this.opModeIsActive()) {
            //  pixy.GetBiggestBlock();

            telemetry.addData("pixy blue", pixy.GetBiggestBlock(2));
            telemetry.addData("pixy red", pixy.GetBiggestBlock(1));
            telemetry.addData("null", pixy.GetBiggestBlock(3));
            telemetry.update();
        }
    }
}

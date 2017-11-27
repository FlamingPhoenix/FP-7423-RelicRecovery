package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoControllerEx;

import org.firstinspires.ftc.teamcode.FlamingPhoenix.Direction;
import org.firstinspires.ftc.teamcode.FlamingPhoenix.Drive;
import org.firstinspires.ftc.teamcode.FlamingPhoenix.OpModeInitializer;
import org.firstinspires.ftc.teamcode.FlamingPhoenix.Vuforia;

/**
 * Created by HwaA1 on 11/4/2017.
 */

@Autonomous(name = "RED_LEFT", group = "none")
public class CraptonRedLeft extends LinearOpMode {
    DcMotor br;
    DcMotor bl;
    DcMotor fr;
    DcMotor fl;

    Servo grabber;
    Servo upperGrabber;

    Drive wheels;

    BNO055IMU imu;

    @Override
    public void runOpMode() throws InterruptedException {

        Vuforia vu = new Vuforia(this);

        OpModeInitializer opModeInitializer = new OpModeInitializer();

        bl = hardwareMap.dcMotor.get("backleft");
        br = hardwareMap.dcMotor.get("backright");
        fl = hardwareMap.dcMotor.get("frontleft");
        fr = hardwareMap.dcMotor.get("frontright");

        bl.setDirection(DcMotor.Direction.REVERSE);
        fl.setDirection(DcMotor.Direction.REVERSE);

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        grabber = hardwareMap.servo.get("grabber");
        upperGrabber = hardwareMap.servo.get("grabber2");
        opModeInitializer.initializeAutoGrabbers(grabber, upperGrabber);

        wheels = new Drive(fr, br, fl, bl, imu, this);

        telemetry.addData("isOpModeActive", this.isStarted());
        telemetry.update();

        waitForStart();

        telemetry.addData("isOpModeActive", this.isStarted());
        telemetry.update();


        grabber.setPosition(0);

        int cryptodistance = 11;

        wheels.drive(7, Direction.FORWARD, .3, this);
        vu.activate();

        //Allow time for Vuforia to see the image
        Thread.sleep(1000);

        if(vu.scanVuforia() == -1) {
            cryptodistance = 17;
        } else if (vu.scanVuforia() == 0) {
            cryptodistance = 9;
        } else if (vu.scanVuforia() == 1) {
            cryptodistance = 2;
        }

        telemetry.addData("column position", vu.scanVuforia());
        telemetry.update();

        wheels.drive(12, Direction.FORWARD, .5, this);
        wheels.drive(cryptodistance, Direction.FORWARD, 0.5, this);
        wheels.turnByIMU(75, .5, Direction.RIGHT);
        wheels.drive(11, Direction.FORWARD, .6, this);

        grabber.setPosition(1);
        Thread.sleep(1000);

        wheels.drive(3, Direction.BACKWARD, .5, this);

        //Use the upperGrabber to push the glyph in, just in case it is not in.
        upperGrabber.setPosition(0);
        Thread.sleep(1000);
        wheels.drive(3, Direction.FORWARD, .5, this);
        upperGrabber.setPosition(1);
        Thread.sleep(1000);
        wheels.drive(1, Direction.BACKWARD, .5, this);
        Thread.sleep(1000);
    }
}


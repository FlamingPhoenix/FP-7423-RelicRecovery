package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.FlamingPhoenix.Direction;
import org.firstinspires.ftc.teamcode.FlamingPhoenix.Drive;
import org.firstinspires.ftc.teamcode.FlamingPhoenix.OpModeInitializer;
import org.firstinspires.ftc.teamcode.FlamingPhoenix.Vuforia;

/**
 * Created by HwaA1 on 11/4/2017.
 */

@Autonomous(name = "Blue_Left", group = "none")
public class CraptonBlueLeft extends LinearOpMode {
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
        vu.activate();

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

        grabber.setPosition(0);

        wheels.drive(20, Direction.FORWARD, .5, this);

        int strafingDistance = 8;

        if(vu.scanVuforia() == -1) {
            strafingDistance = 2;
        } else if(vu.scanVuforia() == 0) {
            strafingDistance = 5;
        } else if(vu.scanVuforia() == 1) {
            strafingDistance = 10;
        }

        Log.d("[Phoenix-view]", "vu: " + vu.scanVuforia());

        wheels.strafe(strafingDistance, .5, Direction.RIGHT, this);
        wheels.drive(7, Direction.FORWARD, .5, this);

        grabber.setPosition(1);
        Thread.sleep(1000);

        wheels.drive(7, Direction.BACKWARD, .5, this);

        wheels.strafe(3, .5, Direction.RIGHT, this);
        grabber.setPosition(0);
        Thread.sleep(1000);

        wheels.drive(8, Direction.FORWARD, .5, this);
        wheels.drive(5, Direction.BACKWARD, .5, this);

        grabber.setPosition(1);
        Thread.sleep(1000);
    }
}

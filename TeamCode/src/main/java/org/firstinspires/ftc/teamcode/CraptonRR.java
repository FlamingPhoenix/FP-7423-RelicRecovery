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

@Autonomous(name = "Red_RIGHT", group = "none")
public class CraptonRR extends LinearOpMode {

    DcMotor br;
    DcMotor bl;
    DcMotor fr;
    DcMotor fl;

    Servo upperGrabber;
    Servo grabber;

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

        grabber.setPosition(0);

        wheels.drive(5, Direction.FORWARD, .3, this);
        vu.activate();

        //Allow time for Vuforia to see the image
        Thread.sleep(2000);

        wheels.drive(12, Direction.FORWARD, .5, this);
        //wheels.drive(4, Direction.FORWARD, .6, this);

        int strafingDistance = 8;

        if(vu.scanVuforia() == 1) {
            strafingDistance = 3;
        } else if(vu.scanVuforia() == 0) {
            strafingDistance = 8;
        } else if(vu.scanVuforia() == -1) {
            strafingDistance = 13;
        }

        Log.d("[Phoenix-view]", "vu: " + vu.scanVuforia());

        wheels.strafe(strafingDistance, .5, Direction.LEFT, this);
        wheels.drive(5, Direction.FORWARD, .5, this);

        grabber.setPosition(1);
        Thread.sleep(1000);

        wheels.drive(5, Direction.BACKWARD, .5, this);

        wheels.strafe(3, .5, Direction.RIGHT, this);
        grabber.setPosition(0);
        Thread.sleep(1000);

        wheels.drive(5, Direction.FORWARD, .5, this);
        wheels.drive(3, Direction.BACKWARD, .5, this);

        grabber.setPosition(1);
        Thread.sleep(1000);
    }
}
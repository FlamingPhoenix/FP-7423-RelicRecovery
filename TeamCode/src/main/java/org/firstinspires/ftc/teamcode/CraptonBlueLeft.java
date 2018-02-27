package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
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

@Autonomous(name = "Blue_LEFT", group = "none")
public class CraptonBlueLeft extends LinearOpMode {

    DcMotor br;
    DcMotor bl;
    DcMotor fr;
    DcMotor fl;

    Servo upperGrabber;
    Servo grabber;

    Servo jewel;
    Servo jewelbase;

    Servo shoulder;
    Servo wrist;
    Servo elbow;

    Drive wheels;

    BNO055IMU imu;

    ColorSensor color;

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

        color = hardwareMap.get(ColorSensor.class, "color");
        color.enableLed(true);

        grabber = hardwareMap.servo.get("grabber");
        upperGrabber = hardwareMap.servo.get("grabber2");
        jewel = hardwareMap.servo.get("jewel");
        jewelbase = hardwareMap.servo.get("jewelbase");

        jewel.setPosition(.1);
        jewelbase.setPosition(.2);

        opModeInitializer.initializeAutoGrabbers(grabber, upperGrabber, jewel);

        shoulder = hardwareMap.servo.get("shoulder");
        wrist = hardwareMap.servo.get("wrist");
        elbow = hardwareMap.servo.get("elbow");

        wheels = new Drive(fr, br, fl, bl, imu, this);

        telemetry.addData("isOpModeActive", this.isStarted());
        telemetry.update();

        vu.activate();

        waitForStart();

        double strafingDistance = 6.0;

        //scan for image first
        if(vu.scanVuforia() == 1) {
            strafingDistance = 13;
        } else if(vu.scanVuforia() == 0) {
            strafingDistance = 7.5;
        } else if(vu.scanVuforia() == -1) {
            strafingDistance = 3;
        }

        float imageAngle = vu.getXAngle();
        strafingDistance -= vu.getAddDistance(imageAngle, 22);

        Log.d("[Phoenix-Adjustment]", "adjustment distance: " + vu.getAddDistance(imageAngle, 22) + ". image angle: " + imageAngle + ". newstrafingDistance: " + strafingDistance);

        Thread.sleep(1000);

        grabber.setPosition(0);
        upperGrabber.setPosition(0);

        jewelbase.setPosition(.3);
        Thread.sleep(1000);
        jewelbase.setPosition(.2);

        jewel.setPosition(.7);

        Thread.sleep(1000);

        int redValue = color.red();
        int blueValue = color.blue();

        if((blueValue - redValue) >= 10) {
            jewelbase.setPosition(.0);
        } else if ((redValue - blueValue) >= 10) {
            jewelbase.setPosition(.4);
        }
        telemetry.addData("blue", color.blue());
        telemetry.addData("red", color.red());
        telemetry.update();

        Thread.sleep(1000);

        jewel.setPosition(.1);
        Thread.sleep(200);
        jewelbase.setPosition(.2);

        Thread.sleep(500);

        wheels.drive(22, Direction.FORWARD, .4, this);

        Log.d("[Phoenix-auto]", "blue: " + blueValue+ ". red: " + redValue);
        Log.d("[Phoenix-auto]", "vumark: " + vu.scanVuforia());

        Log.d("[Phoenix-view]", "vu: " + vu.scanVuforia());

        Thread.sleep(1000);

        wheels.strafe(strafingDistance, .5, Direction.RIGHT, this);
        wheels.drive(10, Direction.FORWARD, .5, this);

        grabber.setPosition(1);
        upperGrabber.setPosition(1);
        Thread.sleep(1000);

        wheels.drive(5, Direction.BACKWARD, .5, this);

    }
}

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

    Servo shoulder;
    Servo wrist;

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

        opModeInitializer.initializeAutoGrabbers(grabber, upperGrabber, jewel);

        shoulder = hardwareMap.servo.get("shoulder");
        wrist = hardwareMap.servo.get("wrist");

        ServoControllerEx servoController = (ServoControllerEx) shoulder.getController();
        int shoulderServoPort = shoulder.getPortNumber();
        PwmControl.PwmRange shoulderPwmRange = new PwmControl.PwmRange(1015, 1776);
        servoController.setServoPwmRange(shoulderServoPort, shoulderPwmRange);

        ServoControllerEx wristController = (ServoControllerEx) wrist.getController();
        int wristServoPort = wrist.getPortNumber();
        PwmControl.PwmRange wristPwmRange = new PwmControl.PwmRange(750, 2250);
        wristController.setServoPwmRange(wristServoPort, wristPwmRange);

        wheels = new Drive(fr, br, fl, bl, imu, this);

        telemetry.addData("isOpModeActive", this.isStarted());
        telemetry.update();

        vu.activate();

        waitForStart();

        int strafingDistance = 6;

        //scan for image first
        if(vu.scanVuforia() == 1) {
            strafingDistance = 11;
        } else if(vu.scanVuforia() == 0) {
            strafingDistance = 6;
        } else if(vu.scanVuforia() == -1) {
            strafingDistance = 1;
        }

        double zdistance = vu.getZ();
        Log.d("[Phoenix-auto]", "z: " + zdistance);

        jewel.setPosition(0);
        shoulder.setPosition(.5);
        wrist.setPosition(0);

        Thread.sleep(1000);

        grabber.setPosition(0);
        upperGrabber.setPosition(0);

        Thread.sleep(1000);
        wheels.strafe(1, .3, Direction.LEFT, this);

        Thread.sleep(1000);

        zdistance = vu.getZ();
        Log.d("[Phoenix-auto]", "z: " + zdistance);

        //look for image again
        if(vu.scanVuforia() == 1) {
            strafingDistance = 11;
        } else if(vu.scanVuforia() == 0) {
            strafingDistance = 6;
        } else if(vu.scanVuforia() == -1) {
            strafingDistance = 1;
        }

        double jeweldistance = 1.5;
        Direction jeweldirection;
        int redValue = color.red();
        int blueValue = color.blue();

        if((blueValue - redValue) >= 10) {
            jeweldirection = Direction.FORWARD;
        } else if ((redValue - blueValue) >= 10) {
            jeweldirection = Direction.BACKWARD;
        } else {
            jeweldirection = null;
            jeweldistance = 0;
        }

        wheels.drive(jeweldistance, jeweldirection, .15, this);

        Thread.sleep(500);

        wheels.strafe(1, .3, Direction.RIGHT, this);

        Thread.sleep(200);

        zdistance = vu.getZ();
        telemetry.addData("z", zdistance);
        telemetry.update();
        Log.d("[Phoenix-auto]", "z: " + zdistance);

        jewel.setPosition(1);

        wheels.drive((jeweldirection == Direction.BACKWARD ? 25 : jeweldirection == Direction.FORWARD ? 15 : 22), Direction.FORWARD, .4, this);

        Log.d("[Phoenix-auto]", "jeweldirection: " + jeweldirection + ". jeweldistance: " + jeweldistance);
        Log.d("[Phoenix-auto]", "blue: " + blueValue+ ". red: " + redValue);
        Log.d("[Phoenix-auto]", "vumark: " + vu.scanVuforia());

        Log.d("[Phoenix-view]", "vu: " + vu.scanVuforia());

        wheels.strafe(strafingDistance, .5, Direction.RIGHT, this);
        wheels.drive(10, Direction.FORWARD, .5, this);

        grabber.setPosition(1);
        upperGrabber.setPosition(1);
        Thread.sleep(1000);

        wheels.drive(5, Direction.BACKWARD, .5, this);

    }
}

package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoControllerEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.FlamingPhoenix.Direction;
import org.firstinspires.ftc.teamcode.FlamingPhoenix.Drive;
import org.firstinspires.ftc.teamcode.FlamingPhoenix.OpModeInitializer;
import org.firstinspires.ftc.teamcode.FlamingPhoenix.Vuforia;

/**
 * Created by HwaA1 on 11/4/2017.
 */

@Autonomous(name = "Red_LEFT", group = "none")
public class CraptonRedLeft extends LinearOpMode {

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

        jewelbase.setPosition(.2);
        jewel.setPosition(.1);

        opModeInitializer.initializeAutoGrabbers(grabber, upperGrabber, jewel);

        shoulder = hardwareMap.servo.get("shoulder");
        wrist = hardwareMap.servo.get("wrist");
        elbow = hardwareMap.servo.get("elbow");

        wheels = new Drive(fr, br, fl, bl, imu, this);

        telemetry.addData("isOpModeActive", this.isStarted());
        telemetry.update();

        vu.activate();

        telemetry.addData("isOpModeActive", this.isStarted());
        telemetry.update();

        waitForStart();

        grabber.setPosition(0);
        upperGrabber.setPosition(0);

        wheels.drive(1, Direction.FORWARD, .2, this);

        Thread.sleep(1000);

        double distanceFromKey = vu.getZ();
        double difference =(-distanceFromKey) - 370;
        difference *= 0.0394;

        Direction strafeD = Direction.LEFT;
        if(difference > 0) {
            strafeD = Direction.RIGHT;
        }

        Log.d("[Phoenix-vu-strafe]", "distance to strafe: " + difference);

        if (strafeD == Direction.LEFT){
            wheels.strafe(Math.abs(difference), .25, strafeD, this);
        }

        jewelbase.setPosition(.3);
        Thread.sleep(1000);
        jewelbase.setPosition(.2);

        jewel.setPosition(.65);

        Thread.sleep(500);

        distanceFromKey = vu.getZ();
        difference = (-distanceFromKey) - 370;
        difference *= 0.0394;

        if(difference > 0) {
            strafeD = Direction.RIGHT;
        }

        wheels.strafe(Math.abs(difference), .25, strafeD, this);


        Log.d("[Phoenix-vu-strafe]", "distance to strafe: " + difference);

        Thread.sleep(1000);

        int redValue = color.red();
        int blueValue = color.blue();

        if((redValue - blueValue) >= 10) {
            jewelbase.setPosition(0);
        } else if (blueValue - redValue >= 10) {
            jewelbase.setPosition(.4);
        }

        Thread.sleep(1000);

        jewel.setPosition(0);
        Thread.sleep(1000);
        jewelbase.setPosition(.18);

        int cryptodistance = 14;

        wheels.drive(2, Direction.FORWARD, .15, this);

        Thread.sleep(2000);

        if(vu.scanVuforia() == -1) {
            cryptodistance = 17;
        } else if (vu.scanVuforia() == 0) {
            cryptodistance = 10;
        } else if (vu.scanVuforia() == 1) {
            cryptodistance = 2;
        }

        wheels.drive(20, Direction.FORWARD, .4, this);

        wheels.drive(cryptodistance, Direction.FORWARD, 0.5, this);
        wheels.turnByIMU(75, .4, Direction.RIGHT);
        wheels.drive(8, Direction.FORWARD, .6, this);

        grabber.setPosition(1);
        upperGrabber.setPosition(1);

        Thread.sleep(1000);

        wheels.drive(3, Direction.BACKWARD, .5, this);
    }
}


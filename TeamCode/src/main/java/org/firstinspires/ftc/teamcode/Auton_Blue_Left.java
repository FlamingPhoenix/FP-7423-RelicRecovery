package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoControllerEx;

import org.firstinspires.ftc.teamcode.FlamingPhoenix.Direction;
import org.firstinspires.ftc.teamcode.FlamingPhoenix.Drive;
import org.firstinspires.ftc.teamcode.FlamingPhoenix.Vuforia;

/**
 * Created by HwaA1 on 11/25/2017.
 */

@Autonomous(name = "Blue_Left", group = "none")
public class Auton_Blue_Left extends LinearOpMode {
    DcMotor br;
    DcMotor bl;
    DcMotor fr;
    DcMotor fl;

    Servo grabber;

    Drive wheels;

    BNO055IMU imu;

    @Override
    public void runOpMode() throws InterruptedException {
        Vuforia vu = new Vuforia(this);
        vu.activate();

        bl = hardwareMap.dcMotor.get("backleft");
        br = hardwareMap.dcMotor.get("backright");
        fl = hardwareMap.dcMotor.get("frontleft");
        fr = hardwareMap.dcMotor.get("frontright");

        bl.setDirection(DcMotor.Direction.REVERSE);
        fl.setDirection(DcMotor.Direction.REVERSE);

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        grabber = hardwareMap.servo.get("grabber");

        ServoControllerEx grabberController = (ServoControllerEx) grabber.getController();
        int grabberServoPort = grabber.getPortNumber();
        PwmControl.PwmRange grabberPwmRange = new PwmControl.PwmRange(1418, 2200);
        grabberController.setServoPwmRange(grabberServoPort, grabberPwmRange);

        grabber.setPosition(1);

        wheels = new Drive(fr, br, fl, bl, imu, this);

        waitForStart();

        Log.d("[Phoenix-vu]", "vumark: " + vu.scanVuforia());

        telemetry.addData("vumark", vu.scanVuforia());
        telemetry.update();

        grabber.setPosition(0);

        int cryptodistance = 11;

        if(vu.scanVuforia() == -1) {
            cryptodistance = 2;
        } else if (vu.scanVuforia() == 0) {
            cryptodistance = 9;
        } else if (vu.scanVuforia() == 1) {
            cryptodistance = 17;
        }

        grabber.setPosition(0);

        wheels.drive(22, Direction.FORWARD, .5, this);
        wheels.drive(cryptodistance, Direction.FORWARD, .5, this);
        wheels.turnByIMU(82, .5, Direction.LEFT);
        wheels.drive(11, Direction.FORWARD, .6, this);

        grabber.setPosition(1);
        Thread.sleep(1000);

        wheels.drive(2, Direction.BACKWARD, .5, this);

        grabber.setPosition(0);
        Thread.sleep(750);
        wheels.drive(3, Direction.FORWARD, .5, this);
        wheels.drive(3, Direction.BACKWARD, .5, this);
    }
}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.FlamingPhoenix.Arm;
import org.firstinspires.ftc.teamcode.FlamingPhoenix.Drive;

/**
 * Created by HwaA1 on 10/31/2017.
 */

public class Teleop extends OpMode {

    DcMotor br;
    DcMotor bl;
    DcMotor fr;
    DcMotor fl;

    Servo shoulder;
    Servo elbow;
    Servo wrist;

    double elbowPosition;
    double shoulderPosition = .5;

    Drive wheels;

    BNO055IMU imu;

    Arm farrm;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        bl = hardwareMap.dcMotor.get("backleft");
        br = hardwareMap.dcMotor.get("backright");
        fl = hardwareMap.dcMotor.get("frontleft");
        fr = hardwareMap.dcMotor.get("frontright");

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        wheels = new Drive(fr, br, fl, bl, imu, this);

        shoulder = hardwareMap.servo.get("shoulder");
        elbow = hardwareMap.servo.get("elbow");
        wrist = hardwareMap.servo.get("wrist");


        shoulder.setPosition(1);
        elbow.setPosition(0);
        wrist.setPosition(0);

        farrm = new Arm(shoulder, elbow, wrist);

        bl.setDirection(DcMotor.Direction.REVERSE);
        fl.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void loop() {
        wheels.drive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);

        farrm.moveArm(gamepad2);

    }
}
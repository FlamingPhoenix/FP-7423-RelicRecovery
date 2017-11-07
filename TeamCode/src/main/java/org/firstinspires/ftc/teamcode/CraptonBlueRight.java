package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.FlamingPhoenix.Direction;
import org.firstinspires.ftc.teamcode.FlamingPhoenix.Drive;

/**
 * Created by HwaA1 on 11/4/2017.
 */

@Autonomous(name = "CraptonBlueRight", group = "none")
public class CraptonBlueRight extends LinearOpMode {
    DcMotor br;
    DcMotor bl;
    DcMotor fr;
    DcMotor fl;

    Servo grabber;

    Drive wheels;

    BNO055IMU imu;

    @Override
    public void runOpMode() throws InterruptedException {
        bl = hardwareMap.dcMotor.get("backleft");
        br = hardwareMap.dcMotor.get("backright");

        fl = hardwareMap.dcMotor.get("frontleft");
        fr = hardwareMap.dcMotor.get("frontright");

        bl.setDirection(DcMotor.Direction.REVERSE);
        fl.setDirection(DcMotor.Direction.REVERSE);

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        grabber = hardwareMap.servo.get("finger");

        grabber.setPosition(1);

        wheels = new Drive(fr, br, fl, bl, imu, this);

        waitForStart();

        grabber.setPosition(0);

        wheels.drive(22, Direction.FORWARD, .7, this);
        wheels.drive(11, Direction.FORWARD, 1, this);
        wheels.turnByIMU(80, .5, Direction.LEFT);
        wheels.drive(11, Direction.FORWARD, .6, this);

        grabber.setPosition(1);
        Thread.sleep(1000);

        wheels.drive(2, Direction.BACKWARD, .5, this);
    }
}

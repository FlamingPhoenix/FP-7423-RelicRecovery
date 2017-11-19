package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoControllerEx;

import org.firstinspires.ftc.teamcode.FlamingPhoenix.Arm;
import org.firstinspires.ftc.teamcode.FlamingPhoenix.Drive;

/**
 * Created by HwaA1 on 10/31/2017.
 */

@TeleOp(name = "teleop", group = "none")
public class Teleop extends OpMode {

    DcMotor br;
    DcMotor bl;
    DcMotor fr;
    DcMotor fl;

    Servo shoulder;
    Servo elbow;
    Servo wrist;
    Servo grabber;
    Servo grabber2;

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
        grabber = hardwareMap.servo.get("finger");
        grabber2 = hardwareMap.servo.get("finger2");

        ServoControllerEx grabberController = (ServoControllerEx) grabber.getController();
        int grabberServoPort = grabber.getPortNumber();
        PwmControl.PwmRange grabberPwmRange = new PwmControl.PwmRange(899, 2100);
        grabberController.setServoPwmRange(grabberServoPort, grabberPwmRange);

        ServoControllerEx grabber2Controller = (ServoControllerEx) grabber2.getController();
        int grabber2ServoPort = grabber2.getPortNumber();
        PwmControl.PwmRange grabber2PwmRange = new PwmControl.PwmRange(899, 2100);
        grabberController.setServoPwmRange(grabber2ServoPort, grabber2PwmRange);

        shoulder.setPosition(1);

        elbow.setPosition(0);
        wrist.setPosition(0);
        grabber.setPosition(1);
        grabber2.setPosition(1);

        //farrm = new Arm(shoulder, elbow, wrist, ,this);

        bl.setDirection(DcMotor.Direction.REVERSE);
        fl.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void loop() {
        wheels.drive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1);

        if(gamepad1.right_trigger > .5) {
            grabber.setPosition(0);
        } else if (gamepad1.right_bumper)
            grabber.setPosition(1);

        if(gamepad1.left_trigger > .5) {
            grabber2.setPosition(0);
        } else if (gamepad1.left_bumper)
            grabber2.setPosition(1);


    }
}

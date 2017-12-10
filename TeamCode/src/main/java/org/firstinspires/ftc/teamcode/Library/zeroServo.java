package org.firstinspires.ftc.teamcode.Library;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoControllerEx;

/**
 * Created by HwaA1 on 12/7/2017.
 */


@TeleOp(name = "servozero", group = "none")
public class zeroServo extends OpMode {

    Servo elbow;

    @Override
    public void init() {
        elbow = hardwareMap.servo.get("servo1");

        elbow.setPosition(1);

        ServoControllerEx elbowController = (ServoControllerEx) elbow.getController();
        int elbowServoPort = elbow.getPortNumber();
        PwmControl.PwmRange elbowPwmRange = new PwmControl.PwmRange(700, 2300);
        elbowController.setServoPwmRange(elbowServoPort, elbowPwmRange);
    }

    @Override
    public void loop() {
        double elbowPosition = elbow.getPosition();

        if(gamepad1.left_stick_y > .5) {
            elbowPosition += .003;
        } else if(gamepad1.left_stick_y < -.5){
            elbowPosition -= .003;
        } else {}

        if(elbowPosition > 1) {
            elbowPosition = 1;
        } else if(elbowPosition < 0) {
            elbowPosition = 0;
        }

        elbow.setPosition(elbowPosition);

    }
}

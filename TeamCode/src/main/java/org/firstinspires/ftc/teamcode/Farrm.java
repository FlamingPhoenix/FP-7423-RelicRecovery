package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoControllerEx;

import org.firstinspires.ftc.teamcode.FlamingPhoenix.Arm;

/**
 * Created by HwaA1 on 11/2/2017.
 */

public class Farrm extends OpMode {

    Servo shoulder;
    Servo elbow;
    Servo wrist;
    Servo wristation;
    Servo finger;


    Arm arm;

    @Override
    public void init() {
        shoulder = hardwareMap.servo.get("shoulder");
        elbow = hardwareMap.servo.get("elbow");
        wrist = hardwareMap.servo.get("wrist");
        finger  = hardwareMap.servo.get("finger");
        wristation = hardwareMap.servo.get("wristation");

        ServoControllerEx servoController = (ServoControllerEx) shoulder.getController();
        int shoulderServoPort = shoulder.getPortNumber();
        PwmControl.PwmRange shoulderPwmRange = new PwmControl.PwmRange(968, 1748);
        servoController.setServoPwmRange(shoulderServoPort, shoulderPwmRange);

        ServoControllerEx elbowController = (ServoControllerEx) elbow.getController();
        int elbowServoPort = elbow.getPortNumber();
        PwmControl.PwmRange elbowPwmRange = new PwmControl.PwmRange(700, 2300);
        elbowController.setServoPwmRange(elbowServoPort, elbowPwmRange);

        ServoControllerEx wristController = (ServoControllerEx) wrist.getController();
        int wristServoPort = wrist.getPortNumber();
        PwmControl.PwmRange wristPwmRange = new PwmControl.PwmRange(750, 2250);
        wristController.setServoPwmRange(wristServoPort, wristPwmRange);


        double shoulderInitialize = 1;

        shoulder.setPosition(shoulderInitialize);
        elbow.setPosition(1);
        wrist.setPosition(1);
        wristation.setPosition(1);
        finger.setPosition(1);

        arm = new Arm(shoulder, elbow, wrist, wristation, finger, shoulderInitialize, this);

        telemetry.addData("shoulder", shoulder.getPosition());
        telemetry.addData("elbow", elbow.getPosition());
        telemetry.update();
    }


    @Override
    public void loop() {
        /*double shoulderPos = shoulder.getPosition();

        if(gamepad1.dpad_down) {
            shoulderPos -= .001;
        } else if(gamepad1.dpad_up) {
            shoulderPos += .001;
        }

        shoulder.setPosition(shoulderPos);

        //shoulder.setPosition(1);

        double elbowPos = elbow.getPosition();

        if(gamepad1.a) {
            elbowPos -= .005;
        } else if(gamepad1.y) {
            elbowPos += .005;
        } else {}

        elbow.setPosition(elbowPos);*/

        arm.moveArm(gamepad1);

        if (gamepad1.y) {
            arm.moveOutOfWay();
        } else if(gamepad1.a) {
            arm.pullArmBack();
        }

        telemetry.addData("shoulder", shoulder.getPosition());
        telemetry.addData("elbow", elbow.getPosition());
        telemetry.update();
    }
}

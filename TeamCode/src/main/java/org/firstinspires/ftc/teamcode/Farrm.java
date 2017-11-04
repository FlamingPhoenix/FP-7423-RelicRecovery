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

@TeleOp(name = "Farrm", group = "none")

public class Farrm extends OpMode {

    Servo shoulder;
    Servo elbow;
    Servo wrist;
    Servo thumb;
    Servo finger;
    //Servo lift1;
    //Servo grabber;

    Arm arm;

    @Override
    public void init() {
        shoulder = hardwareMap.servo.get("shoulder");
        elbow = hardwareMap.servo.get("elbow");
        wrist = hardwareMap.servo.get("wrist");
        thumb = hardwareMap.servo.get("thumb");
        finger = hardwareMap.servo.get("finger");
        //lift1 = hardwareMap.servo.get("lift1");
        //grabber = hardwareMap.servo.get("grabber");

        ServoControllerEx servoController = (ServoControllerEx) shoulder.getController();
        int shoulderServoPort = shoulder.getPortNumber();
        PwmControl.PwmRange shoulderPwmRange = new PwmControl.PwmRange(899, 2105);
        servoController.setServoPwmRange(shoulderServoPort, shoulderPwmRange);

        ServoControllerEx elbowController = (ServoControllerEx) elbow.getController();
        int elbowServoPort = elbow.getPortNumber();
        PwmControl.PwmRange elbowPwmRange = new PwmControl.PwmRange(899, 2105);
        elbowController.setServoPwmRange(elbowServoPort, elbowPwmRange);

        ServoControllerEx wristController = (ServoControllerEx) wrist.getController();
        int wristServoPort = wrist.getPortNumber();
        PwmControl.PwmRange wristPwmRange = new PwmControl.PwmRange(899, 2105);
        wristController.setServoPwmRange(wristServoPort, wristPwmRange);


        shoulder.setPosition(1);
        elbow.setPosition(1);
        wrist.setPosition(.5);
        thumb.setPosition(0);
        finger.setPosition(0);
        //lift1.setPosition(0);
        //grabber.setPosition(0);

        arm = new Arm(shoulder, elbow, wrist);

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

        telemetry.addData("shoulder", shoulder.getPosition());
        telemetry.addData("elbow", elbow.getPosition());
        telemetry.update();
    }
}

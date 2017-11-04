package org.firstinspires.ftc.teamcode.FlamingPhoenix;

import android.graphics.Path;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by HwaA1 on 10/31/2017.
 */

public class Arm {


    Servo shoulder;
    Servo elbow;
    Servo wrist;

    OpMode op;

    public Arm(Servo s, Servo e, Servo w, OpMode o) {
        shoulder = s;
        elbow = e;
        wrist = w;
        op = o;
    }

    public double shoulderToDegree() {
        double pos = shoulder.getPosition();

        double newpos = pos * 180;

        return newpos;
    }

    public double degreeToShoulder(double degree) {
        return degree / 180;
    }

    public void moveArm(Gamepad gamepad2) {

        double elbowPosition = elbow.getPosition();
        if(gamepad2.dpad_down) {
            elbowPosition += .001;
        } else if(gamepad2.dpad_up){
            elbowPosition -= .001;
        } else {}
        elbow.setPosition(elbowPosition);

        double shoulderPosition = shoulder.getPosition();
        if(gamepad2.a) {
            shoulderPosition += .001;
        } else if(gamepad2.y){
            shoulderPosition -= .001;
        } else {}
        //base.setPosition(basePosition);

        double x = elbow.getPosition();

        double shoulderPos = (.45 - (x / 2)) + shoulderPosition;
        if(shoulderPos > 1) {
            shoulderPos = 1;
        } else if(shoulderPos < 0) {
            shoulderPos = 0;
        } else {}

        op.telemetry.addData("shoulderPos", shoulderPos);
        op.telemetry.addData("x", x);
        op.telemetry.update();

        shoulder.setPosition(shoulderPos);
    }
}

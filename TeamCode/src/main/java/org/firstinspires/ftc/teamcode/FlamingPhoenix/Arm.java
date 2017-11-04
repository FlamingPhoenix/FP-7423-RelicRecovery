package org.firstinspires.ftc.teamcode.FlamingPhoenix;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by HwaA1 on 10/31/2017.
 */

public class Arm {


    Servo shoulder;
    Servo elbow;
    Servo wrist;



    public Arm(Servo s, Servo e, Servo w) {
        shoulder = s;
        elbow = e;
        wrist = w;
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
        if(gamepad2.dpad_up) {
            elbowPosition += .001;
        } else if(gamepad2.dpad_down){
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

        double basePos = (.5 - (x / 2)) + shoulderPosition;
        if(basePos > 1) {
            basePos = 1;
        } else if(basePos < 0) {
            basePos = 0;
        } else {}

        shoulder.setPosition(basePos);
    }
}

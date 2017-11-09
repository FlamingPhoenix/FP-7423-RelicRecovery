package org.firstinspires.ftc.teamcode.FlamingPhoenix;

import android.graphics.Path;
import android.util.Log;

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

    double imaginaryShoulderPosition = 1;

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
        //Log.d("[Phoenix]", "read elbow position:" + elbowPosition);

        if(gamepad2.left_stick_y < -.5) {
            elbowPosition += .002;

            //Log.d("[Phoenix]", "move down elbow position:" + elbowPosition);
        } else if(gamepad2.left_stick_y > .5){
            elbowPosition -= .002;

            //Log.d("[Phoenix]", "move up elbow position:" + elbowPosition);
        } else {}

        //Log.d("[Phoenix]", "set elbow position:" + elbowPosition);
        elbow.setPosition(elbowPosition);

        //double shoulderPosition = shoulder.getPosition();
        //Log.d("[Phoenix]", "read shoulder position:" + shoulderPosition);

        if(gamepad2.y) {
            imaginaryShoulderPosition += .002;

            //Log.d("[Phoenix]", "Shoulder position: " + shoulderPosition);
        } else if(gamepad2.a){
            imaginaryShoulderPosition -= .002;

            //Log.d("[Phoenix]", "Shoulder position: " + shoulderPosition);
        } else {}
        //base.setPosition(basePosition);

        double x = elbow.getPosition();

        double shoulderPos = (x / 2) + imaginaryShoulderPosition - .5;
        Log.d("[Phoenix]", "shoulderPos:" + shoulderPos);

        if(shoulderPos > 1) {
            shoulderPos = 1;
        } else if(shoulderPos < 0) {
            shoulderPos = 0;
        } else {}

        shoulder.setPosition(shoulderPos);


        double wristPosition = wrist.getPosition();

        if(gamepad2.dpad_up)
            wristPosition += .01;
        else if(gamepad2.dpad_down)
            wristPosition -= .01;

        wrist.setPosition(wristPosition);


        op.telemetry.addData("shoulderPos", shoulderPos);
        op.telemetry.addData("x", x);
        //op.telemetry.addData("shoulderPosition ", shoulderPosition);
        op.telemetry.update();
        //Log.d("[Phoenix]", "set shoulder position " + shoulderPos);
    }
}

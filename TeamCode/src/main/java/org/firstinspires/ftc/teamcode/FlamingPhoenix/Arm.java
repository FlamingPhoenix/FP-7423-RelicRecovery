package org.firstinspires.ftc.teamcode.FlamingPhoenix;

import android.graphics.Paint;
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
    Servo wristation;
    Servo finger;

    OpMode op;

    double imaginaryShoulderPosition;

    public Arm(Servo s, Servo e, Servo w, Servo wr, Servo f, double should, OpMode o) {
        shoulder = s;
        elbow = e;
        wrist = w;
        wristation = wr;
        finger = f;

        imaginaryShoulderPosition = should;

        op = o;
    }

    /*public Arm(Servo s, Servo e, Servo w, Servo wr, OpMode o) {
        shoulder = s;
        elbow = e;
        wrist = w;
        wristation = wr;

        op = o;
    }*/

    public double shoulderToDegree() {
        double pos = shoulder.getPosition();

        double newpos = pos * 180;

        return newpos;
    }

    public double degreeToShoulder(double degree) {
        return degree / 180;
    }

    public void moveArm(Gamepad gamepad) {

        double elbowPosition = elbow.getPosition();
        double originalElbowPosition = elbow.getPosition();

        if(gamepad.right_stick_y > .5) {
            elbowPosition += .001;
        } else if(gamepad.right_stick_y < -.5){
            elbowPosition -= .001;
        } else {}

        double diffEl = elbowPosition - originalElbowPosition;
        elbow.setPosition(elbowPosition);

        double rotationPos = wristation.getPosition();
        if(gamepad.right_stick_x > .5) {
            rotationPos += .0025;
        } else if(gamepad.right_stick_x < -.5) {
            rotationPos -= .0025;
        }
        wristation.setPosition(rotationPos);


        double originalShould = imaginaryShoulderPosition;

        if(gamepad.left_stick_y > .5) {
            imaginaryShoulderPosition += .001;

            //Log.d("[Phoenix]", "Shoulder position: " + shoulderPosition);
        } else if(gamepad.left_stick_y < -.5){
            imaginaryShoulderPosition -= .001;

            //Log.d("[Phoenix]", "Shoulder position: " + shoulderPosition);
        } else {}
        //base.setPosition(basePosition);

        double newShould = imaginaryShoulderPosition;

        double x = elbow.getPosition();

        double shoulderPos = imaginaryShoulderPosition + .5 - (elbowPosition / 2);
        Log.d("[Phoenix]", "shoulderPos:" + shoulderPos);

        if(shoulderPos > 1) {
            shoulderPos = 1;
        } else if(shoulderPos < 0) {
            shoulderPos = 0;
        } else {}

        double shoulderDifference = newShould - originalShould;
        shoulder.setPosition(shoulderPos);

        double wristPosition = wrist.getPosition();
        double origWristPosition = wristPosition;
        if(gamepad.left_bumper) {
            wristPosition = wristPosition - (diffEl/2) - shoulderDifference + .005;
        }
        else if(gamepad.left_trigger > .5 ) {
            wristPosition = wristPosition - (diffEl/2) - shoulderDifference - .005;
        }
        else if ((diffEl != 0) || (shoulderDifference != 0))
            wristPosition = wristPosition - (diffEl/2) - shoulderDifference;

        Log.d("[Phoenix-wrist]", "diffEl: " + Double.toString(diffEl));
        Log.d("[Phoenix-wrist]", "shoulderDifference: " + Double.toString(shoulderDifference));
        Log.d("[Phoenix-wrist]", "Original wrist: " + Double.toString(origWristPosition));
        Log.d("[Phoenix-wrist]", "wristPosition: " + Double.toString(wristPosition));
        wrist.setPosition(wristPosition);

        double fingerPos = finger.getPosition();
        if(gamepad.right_bumper) {
            fingerPos = 1;
        } else if(gamepad.right_trigger > .5) {
            fingerPos = 0;
        }
        finger.setPosition(fingerPos);



        op.telemetry.addData("shoulderPos", shoulderPos);
        op.telemetry.addData("x", x);
        //op.telemetry.addData("shoulderPosition ", shoulderPosition);
        op.telemetry.update();
        //Log.d("[Phoenix]", "set shoulder position " + shoulderPos);
    }

    public void moveArm(JointMovement shoulderMove, JointMovement elbowMove, JointMovement wristMove, WristDirection wristationDirection, GraberMovement grabber, MoveSpeed speed) {
        double elbowPosition = elbow.getPosition();
        double originalElbowPosition = elbow.getPosition();

        if(elbowMove == JointMovement.BACKWORD) {
            elbowPosition += speed == MoveSpeed.SLOW ? 0.01 : speed == MoveSpeed.MEDIUM ? 0.02 : 0.03;
        } else if(elbowMove == JointMovement.FORWARD){
            elbowPosition -= speed == MoveSpeed.SLOW ? 0.01 : speed == MoveSpeed.MEDIUM ? 0.02 : 0.03;
        }

        double diffEl = elbowPosition - originalElbowPosition;
        elbow.setPosition(elbowPosition);

        double rotationPos = wristation.getPosition();
        if(wristationDirection == WristDirection.CLOCKWISE) {
            rotationPos += .0025;
        } else if(wristationDirection == WristDirection.COUNTERCLOKWISE) {
            rotationPos -= .0025;
        }
        wristation.setPosition(rotationPos);


        double originalShould = imaginaryShoulderPosition;

        if(shoulderMove == JointMovement.FORWARD) {
            imaginaryShoulderPosition += .001;

            //Log.d("[Phoenix]", "Shoulder position: " + shoulderPosition);
        } else if(shoulderMove == JointMovement.BACKWORD){
            imaginaryShoulderPosition -= .001;

            //Log.d("[Phoenix]", "Shoulder position: " + shoulderPosition);
        } else {}
        //base.setPosition(basePosition);

        double newShould = imaginaryShoulderPosition;

        double x = elbow.getPosition();

        double shoulderPos = imaginaryShoulderPosition + .5 - (elbowPosition / 2);
        Log.d("[Phoenix]", "shoulderPos:" + shoulderPos);

        if(shoulderPos > 1) {
            shoulderPos = 1;
        } else if(shoulderPos < 0) {
            shoulderPos = 0;
        } else {}

        double shoulderDifference = newShould - originalShould;
        shoulder.setPosition(shoulderPos);

        double wristPosition = wrist.getPosition();
        double origWristPosition = wristPosition;
        if(wristMove == JointMovement.BACKWORD) {
            wristPosition = wristPosition - (diffEl/2) - shoulderDifference + .005;
        }
        else if(wristMove == JointMovement.FORWARD) {
            wristPosition = wristPosition - (diffEl/2) - shoulderDifference - .005;
        }
        else if ((diffEl != 0) || (shoulderDifference != 0))
            wristPosition = wristPosition - (diffEl/2) - shoulderDifference;

        Log.d("[Phoenix-wrist]", "diffEl: " + Double.toString(diffEl));
        Log.d("[Phoenix-wrist]", "shoulderDifference: " + Double.toString(shoulderDifference));
        Log.d("[Phoenix-wrist]", "Original wrist: " + Double.toString(origWristPosition));
        Log.d("[Phoenix-wrist]", "wristPosition: " + Double.toString(wristPosition));
        wrist.setPosition(wristPosition);

        double fingerPos = finger.getPosition();
        if(grabber == GraberMovement.OPEN) {
            fingerPos = 1;
        } else if(grabber == GraberMovement.CLOSE) {
            fingerPos = 0;
        }
        finger.setPosition(fingerPos);



        op.telemetry.addData("shoulderPos", shoulderPos);
        op.telemetry.addData("x", x);
        //op.telemetry.addData("shoulderPosition ", shoulderPosition);
        op.telemetry.update();
        //Log.d("[Phoenix]", "set shoulder position " + shoulderPos);
    }

    public void moveArm(double shoulderPost, double elbowPost, double wristPost, double wristationPost) {

        double elbowPosition = elbow.getPosition();
        double originalElbowPosition = elbow.getPosition();

        if(originalElbowPosition < elbowPost) {
            elbowPosition += .01;
        }
        else if (originalElbowPosition > elbowPost) {
            elbowPosition -= .01;
        }
        elbow.setPosition(elbowPosition);

        double diffEl = elbowPosition - originalElbowPosition;
        elbow.setPosition(elbowPosition);

        double rotationPos = wristation.getPosition();
        if(rotationPos < wristationPost) {
            rotationPos += .0025;
        } else if(rotationPos > wristationPost) {
            rotationPos -= .0025;
        }
        wristation.setPosition(rotationPos);


        double originalShould = imaginaryShoulderPosition;

        double newShould = imaginaryShoulderPosition;

        double x = elbow.getPosition();

        double shoulderPos = imaginaryShoulderPosition + .5 - (elbowPosition / 2);
        Log.d("[Phoenix]", "shoulderPos:" + shoulderPos);

        if(shoulderPos < shoulderPost) {
            shoulderPos += .01;
        } else if(shoulderPos > shoulderPost) {
            shoulderPos -= .01;
        } else {}

        double shoulderDifference = newShould - originalShould;
        shoulder.setPosition(shoulderPos);

        double wristPosition = wrist.getPosition();
        double origWristPosition = wristPosition;
        if(wristPosition < wristPost) {
            wristPosition += .01;
        }
        else if(wristPosition > wristPost ) {
            wristPosition -= .01;
        }

        Log.d("[Phoenix-wrist]", "diffEl: " + Double.toString(diffEl));
        Log.d("[Phoenix-wrist]", "shoulderDifference: " + Double.toString(shoulderDifference));
        Log.d("[Phoenix-wrist]", "Original wrist: " + Double.toString(origWristPosition));
        Log.d("[Phoenix-wrist]", "wristPosition: " + Double.toString(wristPosition));
        wrist.setPosition(wristPosition);



        op.telemetry.addData("shoulderPos", shoulderPos);
        op.telemetry.addData("x", x);
        //op.telemetry.addData("shoulderPosition ", shoulderPosition);
        op.telemetry.update();
        //Log.d("[Phoenix]", "set shoulder position " + shoulderPos);
    }

    //This is meant to be called from Teleop loop.  It will raise the shoulder to a certain height, then extend elbow
    public void placeRelic() {
        if (shoulder.getPosition() < 0.8) {
            moveArm(JointMovement.FORWARD, JointMovement.STILL, JointMovement.STILL, WristDirection.STILL, GraberMovement.STILL, MoveSpeed.SLOW);
            return;
        }
        else if (shoulder.getPosition() > 0.8){
            moveArm(JointMovement.BACKWORD, JointMovement.STILL, JointMovement.STILL, WristDirection.STILL, GraberMovement.STILL, MoveSpeed.SLOW);
            return;
        }
        else if (elbow.getPosition() < 0.7 ) {
            moveArm(JointMovement.STILL, JointMovement.FORWARD, JointMovement.STILL, WristDirection.STILL, GraberMovement.STILL, MoveSpeed.FAST);
            return;
        }
        else if (elbow.getPosition() < 0.9 ) {
            moveArm(JointMovement.STILL, JointMovement.FORWARD, JointMovement.STILL, WristDirection.STILL, GraberMovement.STILL, MoveSpeed.MEDIUM);

            return;
        }
        else if (elbow.getPosition() < 1 ) {
            moveArm(JointMovement.STILL, JointMovement.FORWARD, JointMovement.STILL, WristDirection.STILL, GraberMovement.STILL, MoveSpeed.SLOW);
            return;
        }
    }

}

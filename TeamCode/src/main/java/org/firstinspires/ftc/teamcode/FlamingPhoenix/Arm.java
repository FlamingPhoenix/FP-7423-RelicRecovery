package org.firstinspires.ftc.teamcode.FlamingPhoenix;

import android.graphics.Paint;
import android.provider.Settings;
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

    int placeRelicStep = 0;
    long lastStepTime;

    int grabRelicStep = 0;
    long lastGrabTime;

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

    public double degreeTo0houlder(double degree) {
        return degree / 180;
    }

    public void moveArm(Gamepad gamepad) {

        double elbowPosition = elbow.getPosition();
        double originalElbowPosition = elbow.getPosition();

        if(gamepad.left_stick_y > .5) {
            elbowPosition += .003;
        } else if(gamepad.left_stick_y < -.5){
            elbowPosition -= .003;
        } else {}

        if(elbowPosition > 1) {
            elbowPosition = 1;
        } else if(elbowPosition < 0) {
            elbowPosition = 0;
        }

        double diffEl = elbowPosition - originalElbowPosition;
        elbow.setPosition(elbowPosition);

        double rotationPos = wristation.getPosition();
        if(gamepad.right_stick_x > .5) {
            rotationPos += .0025;
        } else if(gamepad.right_stick_x < -.5) {
            rotationPos -= .0025;
        }
        if(rotationPos > 1) {
            rotationPos = 1;
        } else if(elbowPosition < 0) {
            rotationPos = 0;
        }
        wristation.setPosition(rotationPos);

        double originalShould = imaginaryShoulderPosition;

        if(gamepad.right_stick_y > .5) {
            imaginaryShoulderPosition += .003;
        }
        if(imaginaryShoulderPosition > 1) {
            imaginaryShoulderPosition = 1;
        } else if(imaginaryShoulderPosition < 0) {
            imaginaryShoulderPosition = 0;
        }

        double newShould = imaginaryShoulderPosition;

        double shoulderPos = imaginaryShoulderPosition - .5 + (elbowPosition / 2);
        Log.d("[Phoenix]", "shoulderPos:" + shoulderPos);

        if(shoulderPos > 1) {
            shoulderPos = 1;
        } else if(shoulderPos < 0) {
            shoulderPos = 0;
        }

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

        if(wristPosition > 1) {
            wristPosition = 1;
        } else if(wristPosition < 0) {
            wristPosition = 0;
        }
        wrist.setPosition(wristPosition);

        double fingerPos = finger.getPosition();
        if(gamepad.right_bumper) {
            fingerPos = 1;
        } else if(gamepad.right_trigger > .5) {
            fingerPos = 0;
        }

        if(fingerPos > 1) {
            fingerPos = 1;
        } else if(shoulderPos < 0) {
            fingerPos = 0;
        }
        finger.setPosition(fingerPos);
    }

    public void moveArm(JointMovement shoulderMove, JointMovement elbowMove, JointMovement wristMove, WristDirection wristationDirection, GrabberMovement grabber, MoveSpeed speed) {
        double elbowPosition = elbow.getPosition();
        double originalElbowPosition = elbow.getPosition();

        if(elbowMove == JointMovement.BACKWARD) {
            elbowPosition += speed == MoveSpeed.SLOW ? 0.01 : speed == MoveSpeed.MEDIUM ? 0.02 : 0.03;
        } else if(elbowMove == JointMovement.FORWARD){
            elbowPosition -= speed == MoveSpeed.SLOW ? 0.01 : speed == MoveSpeed.MEDIUM ? 0.02 : 0.03;
        }

        if(elbowPosition > 1) {
            elbowPosition = 1;
        } else if(elbowPosition < 0) {
            elbowPosition = 0;
        }

        double diffEl = elbowPosition - originalElbowPosition;
        elbow.setPosition(elbowPosition);

        double rotationPos = wristation.getPosition();
        if(wristationDirection == WristDirection.CLOCKWISE) {
            rotationPos += .0025;
        } else if(wristationDirection == WristDirection.COUNTERCLOCKWISE) {
            rotationPos -= .0025;
        }
        if(rotationPos > 1) {
            rotationPos = 1;
        } else if(elbowPosition < 0) {
            rotationPos = 0;
        }
        wristation.setPosition(rotationPos);


        double originalShould = imaginaryShoulderPosition;

        if(shoulderMove == JointMovement.FORWARD) {
            imaginaryShoulderPosition -= .0015;
        } else if(shoulderMove == JointMovement.BACKWARD){
            imaginaryShoulderPosition += .0015;
        }
        if(imaginaryShoulderPosition > 1) {
            imaginaryShoulderPosition = 1;
        } else if(imaginaryShoulderPosition < 0) {
            imaginaryShoulderPosition = 0;
        }

        double newShould = imaginaryShoulderPosition;

        double shoulderPos = imaginaryShoulderPosition - .5 + (elbowPosition / 2);

        if(shoulderPos > 1) {
            shoulderPos = 1;
        } else if(shoulderPos < 0) {
            shoulderPos = 0;
        }

        double shoulderDifference = newShould - originalShould;
        shoulder.setPosition(shoulderPos);

        double wristPosition = wrist.getPosition();
        double origWristPosition = wristPosition;
        if(wristMove == JointMovement.BACKWARD) {
            wristPosition = wristPosition - (diffEl/2) - shoulderDifference + .005;
        }
        else if(wristMove == JointMovement.FORWARD) {
            wristPosition = wristPosition - (diffEl/2) - shoulderDifference - .005;
        }
        else if ((diffEl != 0) || (shoulderDifference != 0))
            wristPosition = wristPosition - (diffEl/2) - shoulderDifference;
        if(wristPosition > 1) {
            wristPosition = 1;
        } else if(wristPosition < 0) {
            wristPosition = 0;
        }

        wrist.setPosition(wristPosition);

        double fingerPos = finger.getPosition();
        if(grabber == GrabberMovement.OPEN) {
            fingerPos = 1;
        } else if(grabber == GrabberMovement.CLOSE) {
            fingerPos = 0;
        }
        if(fingerPos > 1) {
            fingerPos = 1;
        } else if(shoulderPos < 0) {
            fingerPos = 0;
        }
        finger.setPosition(fingerPos);
    }

    public void moveArm(double imgShoulder, double elbowPost, double wristPost, double wristationPost) {
        imaginaryShoulderPosition = imgShoulder;
        double shoulderPos = imaginaryShoulderPosition - .5 + (elbowPost / 2);

        shoulder.setPosition(shoulderPos);
        elbow.setPosition(elbowPost);
        wrist.setPosition(wristPost);
        wristation.setPosition(wristationPost);
    }

    public void grabRelic() {
        if(grabRelicStep == 0) {
            moveArm(.6, elbow.getPosition(), wrist.getPosition(), wristation.getPosition());
            grabRelicStep++;
        } else if(grabRelicStep == 1) {
            if(elbow.getPosition() > .5);
                //moveArm(JointMovement.STILL, JointMovement.BACKWARD, JointMovement.STILL, GrabberMovement.STILL);
        }
    }

    public void moveOutOfWay() {
        moveArm(.5, .95, 0, 1);
    }

    public void pullArmBack(){
        placeRelicStep = 0;
    }

    /*public void placeRelic() {
        if (imaginaryShoulderPosition != .7 && placeRelicStep == 0) {
            moveArm(.85, .9, .85, 1);
            placeRelicStep = 1;
            lastStepTime = System.currentTimeMillis();
        } else if (((placeRelicStep == 1) && ((System.currentTimeMillis() - lastStepTime) > (long) 750))) {
            moveArm(.85, .8, .85, 1);
            placeRelicStep = 2;
            lastStepTime = System.currentTimeMillis();
        } else if (((placeRelicStep == 2) && ((System.currentTimeMillis() - lastStepTime) > (long) 200))) {
            moveArm(.85, .75, .85, 1);
            placeRelicStep = 3;
            lastStepTime = System.currentTimeMillis();
        } else if (((placeRelicStep == 3) && ((System.currentTimeMillis() - lastStepTime) > (long) 200))) {
            moveArm(.85, .7, .85, 1);
            placeRelicStep = 4;
            lastStepTime = System.currentTimeMillis();
        } else if (((placeRelicStep == 4) && ((System.currentTimeMillis() - lastStepTime) > (long) 100))) {
            moveArm(.5, .7, .85, 1);
            placeRelicStep = 5;
            lastStepTime = System.currentTimeMillis();
        } else if (((placeRelicStep == 5) && ((System.currentTimeMillis() - lastStepTime) > (long) 500))) {
            moveArm(.5, .5, .85, 1);
            placeRelicStep = 6;
            lastStepTime = System.currentTimeMillis();
        } else if (((placeRelicStep == 6) && ((System.currentTimeMillis() - lastStepTime) > (long) 500))) {
            moveArm(.4, .5, .85, 1);
            placeRelicStep = 7;
            lastStepTime = System.currentTimeMillis();
        } else if (((placeRelicStep == 7) && ((System.currentTimeMillis() - lastStepTime) > (long) 200))) {
            moveArm(.3, .4, .85, 1);
            placeRelicStep = 8;
            lastStepTime = System.currentTimeMillis();
        } else if (((placeRelicStep == 8) && ((System.currentTimeMillis() - lastStepTime) > (long) 200))) {
            moveArm(.2, .3, .85, 1);
            placeRelicStep = 9;
            lastStepTime = System.currentTimeMillis();
        }
    }*/

}

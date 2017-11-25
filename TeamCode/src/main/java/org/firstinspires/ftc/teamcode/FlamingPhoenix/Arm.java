package org.firstinspires.ftc.teamcode.FlamingPhoenix;

import android.graphics.Paint;
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
            elbowPosition += .003;
        } else if(gamepad.right_stick_y < -.5){
            elbowPosition -= .003;
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
            imaginaryShoulderPosition += .003;

            //Log.d("[Phoenix]", "Shoulder position: " + shoulderPosition);
        } else if(gamepad.left_stick_y < -.5){
            imaginaryShoulderPosition -= .003;

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



        //op.telemetry.addData("shoulderPos", shoulderPos);
        //op.telemetry.addData("x", x);
        //op.telemetry.addData("shoulderPosition ", shoulderPosition);
        //op.telemetry.update();
        //Log.d("[Phoenix]", "set shoulder position " + shoulderPos);
    }

    public void moveArm(JointMovement shoulderMove, JointMovement elbowMove, JointMovement wristMove, WristDirection wristationDirection, GrabberMovement grabber, MoveSpeed speed) {
        double elbowPosition = elbow.getPosition();
        double originalElbowPosition = elbow.getPosition();

        if(elbowMove == JointMovement.BACKWARD) {
            elbowPosition += speed == MoveSpeed.SLOW ? 0.01 : speed == MoveSpeed.MEDIUM ? 0.02 : 0.03;
        } else if(elbowMove == JointMovement.FORWARD){
            elbowPosition -= speed == MoveSpeed.SLOW ? 0.01 : speed == MoveSpeed.MEDIUM ? 0.02 : 0.03;
        }

        double diffEl = elbowPosition - originalElbowPosition;
        elbow.setPosition(elbowPosition);

        double rotationPos = wristation.getPosition();
        if(wristationDirection == WristDirection.CLOCKWISE) {
            rotationPos += .0025;
        } else if(wristationDirection == WristDirection.COUNTERCLOCKWISE) {
            rotationPos -= .0025;
        }
        wristation.setPosition(rotationPos);


        double originalShould = imaginaryShoulderPosition;

        if(shoulderMove == JointMovement.FORWARD) {
            imaginaryShoulderPosition += .0015;

            //Log.d("[Phoenix]", "Shoulder position: " + shoulderPosition);
        } else if(shoulderMove == JointMovement.BACKWARD){
            imaginaryShoulderPosition -= .0015;

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
        if(wristMove == JointMovement.BACKWARD) {
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
        if(grabber == GrabberMovement.OPEN) {
            fingerPos = 1;
        } else if(grabber == GrabberMovement.CLOSE) {
            fingerPos = 0;
        }
        finger.setPosition(fingerPos);
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

        JointMovement shoulderMovement = JointMovement.STILL;
        JointMovement elbowMovement = JointMovement.STILL;
        JointMovement wristMovement = JointMovement.STILL;
        WristDirection wristDirection =WristDirection.STILL;
        MoveSpeed elbowSpeed = MoveSpeed.SLOW;

        if(imaginaryShoulderPosition > .3 && elbow.getPosition() > .5) {
            moveArm(JointMovement.BACKWARD, JointMovement.STILL, JointMovement.STILL, WristDirection.STILL, GrabberMovement.STILL, MoveSpeed.SLOW);
            return;
        }

        if ((imaginaryShoulderPosition < 0.5) && (elbow.getPosition() < 0.1))
            shoulderMovement = JointMovement.FORWARD;
        else if ((imaginaryShoulderPosition < 0.4) && (elbow.getPosition() < 0.5))
            shoulderMovement = JointMovement.FORWARD;
        else if(imaginaryShoulderPosition < .25) {
            moveArm(JointMovement.FORWARD, JointMovement.STILL, JointMovement.STILL, WristDirection.STILL, GrabberMovement.STILL, MoveSpeed.SLOW);
            return;
        }

        Log.d("[Phoenix-Shoulder]", "imaginary shoulder pos: " + imaginaryShoulderPosition + ". elbow pos: " + elbow.getPosition());

        if (elbow.getPosition() > 0.5) {
            elbowMovement = JointMovement.FORWARD;
            elbowSpeed = MoveSpeed.FAST;
        } else if ((elbow.getPosition() > 0.3) && (imaginaryShoulderPosition > 0.45)) {
            elbowMovement = JointMovement.FORWARD;
            elbowSpeed = MoveSpeed.MEDIUM;
        } else if(elbow.getPosition() > 0) {
            elbowMovement = JointMovement.FORWARD;
            elbowSpeed = MoveSpeed.SLOW;
        }

        moveArm(shoulderMovement, elbowMovement, wristMovement, wristDirection, GrabberMovement.STILL, elbowSpeed);
    }
}

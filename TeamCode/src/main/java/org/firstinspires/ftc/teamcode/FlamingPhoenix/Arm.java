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

    int pullArmStep = 0;
    long lastPullTime;

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
            rotationPos += .004;
        } else if(gamepad.right_stick_x < -.5) {
            rotationPos -= .004;
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
        } else if (gamepad.right_stick_y < -.5) {
            imaginaryShoulderPosition -= .003;
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
        if(gamepad.left_bumper) {
            wristPosition = wristPosition + (diffEl/2) - shoulderDifference + .005;
        }
        else if(gamepad.left_trigger > .5 ) {
            wristPosition = wristPosition + (diffEl/2) - shoulderDifference - .005;
        }
        else if ((diffEl != 0) || (shoulderDifference != 0))
            wristPosition = wristPosition + (diffEl/2) - shoulderDifference;

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
        float elbowPosition = (float) elbow.getPosition();
        float originalElbowPosition = (float) elbow.getPosition();

        if(elbowMove == JointMovement.BACKWARD) {
            elbowPosition += speed == MoveSpeed.SLOW ? 0.01f : speed == MoveSpeed.MEDIUM ? 0.02f : 0.03f;
        } else if(elbowMove == JointMovement.FORWARD){
            elbowPosition -= speed == MoveSpeed.SLOW ? 0.01f : speed == MoveSpeed.MEDIUM ? 0.02f : 0.03f;
        }

        if(elbowPosition > 1) {
            elbowPosition = 1;
        } else if(elbowPosition < 0) {
            elbowPosition = 0;
        }

        float diffEl = (float) (elbowPosition - originalElbowPosition);
        elbow.setPosition(elbowPosition);

        double rotationPos = wristation.getPosition();
        if(wristationDirection == WristDirection.CLOCKWISE) {
            rotationPos += .004;
        } else if(wristationDirection == WristDirection.COUNTERCLOCKWISE) {
            rotationPos -= .004;
        }
        if(rotationPos > 1) {
            rotationPos = 1;
        } else if(elbowPosition < 0) {
            rotationPos = 0;
        }
        wristation.setPosition(rotationPos);

        float originalShould = (float) imaginaryShoulderPosition;

        if(shoulderMove == JointMovement.FORWARD) {
            imaginaryShoulderPosition -= .003;
        } else if(shoulderMove == JointMovement.BACKWARD){
            imaginaryShoulderPosition += .003;
        }
        if(imaginaryShoulderPosition > 1) {
            imaginaryShoulderPosition = 1;
        } else if(imaginaryShoulderPosition < 0) {
            imaginaryShoulderPosition = 0;
        }

        float newShould = (float) imaginaryShoulderPosition;

        double shoulderPos = imaginaryShoulderPosition - .5 + (elbowPosition / 2);

        if(shoulderPos > 1) {
            shoulderPos = 1;
        } else if(shoulderPos < 0) {
            shoulderPos = 0;
        }

        float shoulderDifference = newShould - originalShould;
        shoulder.setPosition(shoulderPos);

        float wristPosition = (float) wrist.getPosition();
        float origWristPosition = (float) wristPosition;
        if(wristMove == JointMovement.BACKWARD) {
            wristPosition = wristPosition + (diffEl/2) - shoulderDifference + .005f;
        }
        else if(wristMove == JointMovement.FORWARD) {
            wristPosition = wristPosition + (diffEl/2) - shoulderDifference - .005f;
        }
        else if ((diffEl != 0) || (shoulderDifference != 0))
            wristPosition = wristPosition + (diffEl/2) - shoulderDifference;

        if(wristPosition > 1) {
            wristPosition = 1;
        } else if(wristPosition < 0) {
            wristPosition = 0;
        }

        wrist.setPosition(wristPosition);
        //Log.d("[Phoenix:AutoArm", "WP:" + wristPosition + ";del:" + diffEl + ";sd:" + shoulderDifference + ";ep:" + elbowPosition + ";oep:" + originalElbowPosition + ";np:" + newShould + ";osp:" + originalShould);

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
            moveArm(.5, elbow.getPosition(), 0.5, wristation.getPosition());
            grabRelicStep = 1;
            lastGrabTime = System.currentTimeMillis();
        } else if ((grabRelicStep == 1) && ((System.currentTimeMillis() - lastGrabTime) > 900)) {
            if (elbow.getPosition() > 0.5) {
                moveArm(JointMovement.STILL, JointMovement.FORWARD, JointMovement.STILL, WristDirection.STILL, GrabberMovement.STILL, MoveSpeed.SLOW);
            }
            else {
                grabRelicStep = 2;
                lastGrabTime = System.currentTimeMillis();
            }
        } else if(grabRelicStep == 2) {
            resetArmAutoVariables();
        }
        wristation.setPosition(1);
    }

    public double getImaginaryShoulderPosition() {
        return imaginaryShoulderPosition;
    }

    public void moveOutOfWay() {
        moveArm(.5, .95, 0, 1);
    }

    public void pullArmBack(){
        if(pullArmStep == 0) {
            moveArm(.9, elbow.getPosition(), wrist.getPosition(), wristation.getPosition());
            pullArmStep = 1;
            lastPullTime = System.currentTimeMillis();
        } else if ((pullArmStep == 1) && ((System.currentTimeMillis() - lastPullTime) > 2000)) {
            if (elbow.getPosition() < 0.8) {
                moveArm(JointMovement.STILL, JointMovement.BACKWARD, JointMovement.STILL, WristDirection.STILL, GrabberMovement.STILL, MoveSpeed.SLOW);
            }
            else {
                pullArmStep = 2;
                lastPullTime = System.currentTimeMillis();
            }
        } else if(pullArmStep == 2) {
            resetArmAutoVariables();
        }
    }

    public void placeRelic() {
        float elbowPosition = (float) elbow.getPosition();

        if (placeRelicStep == 0) {
            if (elbowPosition > 0.5) {
                moveArm(JointMovement.STILL, JointMovement.FORWARD, JointMovement.STILL, WristDirection.STILL, GrabberMovement.STILL, MoveSpeed.SLOW);
            }
            else {
                placeRelicStep = 1;
                lastStepTime = System.currentTimeMillis();
            }
        }
        else if(placeRelicStep == 1) {
            if (shoulder.getPosition() > 0.65) {
                moveArm(.65, elbow.getPosition(), wrist.getPosition(), wristation.getPosition());
            } else {
                placeRelicStep = 2;
                lastStepTime = System.currentTimeMillis();
            }
        } else if ((placeRelicStep == 2) && ((System.currentTimeMillis() - lastStepTime) > 2000)) {
            if (elbowPosition > 0) {
                moveArm(JointMovement.STILL, JointMovement.FORWARD, JointMovement.STILL, WristDirection.STILL, GrabberMovement.STILL, MoveSpeed.SLOW);
            }
            else {
                placeRelicStep = 3;
                lastStepTime = System.currentTimeMillis();
            }
        } else if(placeRelicStep == 3) {
            float shouldPos = (float) getImaginaryShoulderPosition();

            if (shouldPos > 0.5) {
                moveArm(JointMovement.FORWARD, JointMovement.STILL, JointMovement.STILL, WristDirection.STILL, GrabberMovement.STILL, MoveSpeed.SLOW);
            }
            else {
                placeRelicStep = 4;
            }
        } else if (placeRelicStep == 4) {
            resetArmAutoVariables();
        }

        wristation.setPosition(1);

        Log.d("[Phoenix:placeRelic]", "ep:" + elbowPosition);
    }

    public void resetArmAutoVariables() {
        lastPullTime = 0;
        pullArmStep = 0;

        lastGrabTime = 0;
        grabRelicStep = 0;

        lastStepTime = 0;
        placeRelicStep = 0;
    }
}

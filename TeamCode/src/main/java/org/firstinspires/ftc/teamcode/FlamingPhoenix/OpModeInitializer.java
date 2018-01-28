package org.firstinspires.ftc.teamcode.FlamingPhoenix;

import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoControllerEx;

/**
 * Created by Steve on 11/26/2017.
 */

public class OpModeInitializer {

    //Initialize 2 Grabber servos for automouse routine
    public void initializeAutoGrabbers(Servo grabber, Servo grabber2, Servo jewel) {

        ServoControllerEx grabberController = (ServoControllerEx) grabber.getController();
        int grabberServoPort = grabber.getPortNumber();
        PwmControl.PwmRange grabberPwmRange = new PwmControl.PwmRange(899, 2200);
        grabberController.setServoPwmRange(grabberServoPort, grabberPwmRange);

        grabber.setPosition(1);

        ServoControllerEx grabber2Controller = (ServoControllerEx) grabber2.getController();
        int grabber2ServoPort = grabber2.getPortNumber();
        PwmControl.PwmRange grabber2PwmRange = new PwmControl.PwmRange(899, 2200);
        grabber2Controller.setServoPwmRange(grabber2ServoPort, grabber2PwmRange);

        grabber2.setPosition(1);

        /*ServoControllerEx jewelController = (ServoControllerEx) jewel.getController();
        int jewelServoPort = jewel.getPortNumber();
        PwmControl.PwmRange jewelPwmRange = new PwmControl.PwmRange(899, 2105);
        jewelController.setServoPwmRange(jewelServoPort, jewelPwmRange);

        jewel.setPosition(1);*/

    }

    public void initializeAutoGrabbers(Servo lowerGrabber, Servo upperGrabber) {

        ServoControllerEx grabberController = (ServoControllerEx) lowerGrabber.getController();
        int grabberServoPort = lowerGrabber.getPortNumber();
        PwmControl.PwmRange grabberPwmRange = new PwmControl.PwmRange(1418, 2200);
        grabberController.setServoPwmRange(grabberServoPort, grabberPwmRange);

        lowerGrabber.setPosition(1);


        ServoControllerEx grabber2Controller = (ServoControllerEx) upperGrabber.getController();
        int grabber2ServoPort = upperGrabber.getPortNumber();
        PwmControl.PwmRange grabber2PwmRange = new PwmControl.PwmRange(899, 2200);
        grabber2Controller.setServoPwmRange(grabber2ServoPort, grabber2PwmRange);

        upperGrabber.setPosition(1);
    }

    public void initalizeArm(Servo shoulder, Servo elbow, Servo wrist, Servo wristation) {
        ServoControllerEx servoController = (ServoControllerEx) shoulder.getController();
        int shoulderServoPort = shoulder.getPortNumber();
        PwmControl.PwmRange shoulderPwmRange = new PwmControl.PwmRange(1015, 1776);
        servoController.setServoPwmRange(shoulderServoPort, shoulderPwmRange);

        ServoControllerEx elbowController = (ServoControllerEx) elbow.getController();
        int elbowServoPort = elbow.getPortNumber();
        PwmControl.PwmRange elbowPwmRange = new PwmControl.PwmRange(700, 2300);
        elbowController.setServoPwmRange(elbowServoPort, elbowPwmRange);

        ServoControllerEx wristController = (ServoControllerEx) wrist.getController();
        int wristServoPort = wrist.getPortNumber();
        PwmControl.PwmRange wristPwmRange = new PwmControl.PwmRange(750, 2250);
        wristController.setServoPwmRange(wristServoPort, wristPwmRange);

        ServoControllerEx wristationController = (ServoControllerEx) wristation.getController();
        int wristationServoPort = wristation.getPortNumber();
        PwmControl.PwmRange wristationPwmRange = new PwmControl.PwmRange(750, 2250);
        wristationController.setServoPwmRange(wristationServoPort, wristationPwmRange);
    }

    public void initalizeAutoArm(Servo shoulder, Servo elbow, Servo wrist) {
        ServoControllerEx servoController = (ServoControllerEx) shoulder.getController();
        int shoulderServoPort = shoulder.getPortNumber();
        PwmControl.PwmRange shoulderPwmRange = new PwmControl.PwmRange(1015, 1776);
        servoController.setServoPwmRange(shoulderServoPort, shoulderPwmRange);

        ServoControllerEx elbowController = (ServoControllerEx) elbow.getController();
        int elbowServoPort = elbow.getPortNumber();
        PwmControl.PwmRange elbowPwmRange = new PwmControl.PwmRange(700, 2300);
        elbowController.setServoPwmRange(elbowServoPort, elbowPwmRange);

        ServoControllerEx wristController = (ServoControllerEx) wrist.getController();
        int wristServoPort = wrist.getPortNumber();
        PwmControl.PwmRange wristPwmRange = new PwmControl.PwmRange(750, 2250);
        wristController.setServoPwmRange(wristServoPort, wristPwmRange);
    }
}

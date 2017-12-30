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

        ServoControllerEx jewelController = (ServoControllerEx) jewel.getController();
        int jewelServoPort = jewel.getPortNumber();
        PwmControl.PwmRange jewelPwmRange = new PwmControl.PwmRange(899, 2105);
        jewelController.setServoPwmRange(jewelServoPort, jewelPwmRange);

        jewel.setPosition(1);

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
}

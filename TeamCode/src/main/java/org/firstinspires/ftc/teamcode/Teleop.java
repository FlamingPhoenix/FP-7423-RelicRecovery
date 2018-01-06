package org.firstinspires.ftc.teamcode;

import android.graphics.Paint;
import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoControllerEx;

import org.firstinspires.ftc.teamcode.FlamingPhoenix.Arm;
import org.firstinspires.ftc.teamcode.FlamingPhoenix.Drive;
import org.firstinspires.ftc.teamcode.FlamingPhoenix.GrabberMovement;
import org.firstinspires.ftc.teamcode.FlamingPhoenix.JointMovement;
import org.firstinspires.ftc.teamcode.FlamingPhoenix.MoveSpeed;
import org.firstinspires.ftc.teamcode.FlamingPhoenix.WristDirection;

/**
 * Created by HwaA1 on 10/31/2017.
 */

@TeleOp(name = "teleop", group = "none")
public class Teleop extends OpMode {

    DcMotor br;
    DcMotor bl;
    DcMotor fr;
    DcMotor fl;

    DcMotor lift;

    Servo shoulder;
    Servo elbow;
    Servo wrist;
    Servo wristation;
    Servo finger;

    Servo grabber;
    Servo grabber2;

    Servo elevator;
    Servo jewel;

    double elbowPosition;
    double shoulderPosition = .5;

    Drive wheels;

    BNO055IMU imu;

    Arm arm;

    boolean isBumperBeingPressed;
    boolean isGrabber2Closed;

    boolean isTriggerBeingPressed;
    boolean isGrabberClosed;

    boolean isArmInitialized;
    boolean isArmPositionSet;

    boolean isJewelInitialized;
    boolean isJewelPullBack;
    long jewelInitializeTime;
    ColorSensor color;


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        bl = hardwareMap.dcMotor.get("backleft");
        br = hardwareMap.dcMotor.get("backright");
        fl = hardwareMap.dcMotor.get("frontleft");
        fr = hardwareMap.dcMotor.get("frontright");

        lift = hardwareMap.dcMotor.get("lift");

        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        wheels = new Drive(fr, br, fl, bl, imu, this);

        shoulder = hardwareMap.servo.get("shoulder");
        elbow = hardwareMap.servo.get("elbow");
        wrist = hardwareMap.servo.get("wrist");
        wristation = hardwareMap.servo.get("wristation");
        finger = hardwareMap.servo.get("finger");

        grabber = hardwareMap.servo.get("grabber");
        grabber2 = hardwareMap.servo.get("grabber2");

        elevator = hardwareMap.servo.get("elevator");

        ServoControllerEx grabberController = (ServoControllerEx) grabber.getController();
        int grabberServoPort = grabber.getPortNumber();
        PwmControl.PwmRange grabberPwmRange = new PwmControl.PwmRange(899, 2200);
        grabberController.setServoPwmRange(grabberServoPort, grabberPwmRange);

        ServoControllerEx grabber2Controller = (ServoControllerEx) grabber2.getController();
        int grabber2ServoPort = grabber2.getPortNumber();
        PwmControl.PwmRange grabber2PwmRange = new PwmControl.PwmRange(899, 2200);
        grabber2Controller.setServoPwmRange(grabber2ServoPort, grabber2PwmRange);

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

        jewel = hardwareMap.servo.get("jewel");
        ServoControllerEx jewelController = (ServoControllerEx) jewel.getController();
        int jewelServoPort = jewel.getPortNumber();
        PwmControl.PwmRange jewelPwmRange = new PwmControl.PwmRange(899, 2105);
        jewelController.setServoPwmRange(jewelServoPort, jewelPwmRange);

        wrist.setPosition(0);

        arm = new Arm(shoulder, elbow, wrist, wristation, finger, 1, this);

        bl.setDirection(DcMotor.Direction.REVERSE);
        fl.setDirection(DcMotor.Direction.REVERSE);

        isArmPositionSet = false;
        isArmInitialized = false;

        color = hardwareMap.get(ColorSensor.class, "color");
        color.enableLed(false);

    }
    //Initialize arm servo values
    public void initializeArm() {
        if (!isArmInitialized) {
            shoulder.setPosition(0.5);
            elbow.setPosition(1);
            wrist.setPosition(0);
            wristation.setPosition(.5);
            finger.setPosition(1);

            isArmInitialized = true;
        }
    }

    //Move arm to preset position to so it would not interfere grabber during teleop
    public void presetArmPosition() {

        if (!isArmPositionSet) {
            wristation.setPosition(0.5);

            JointMovement elbowMovement = JointMovement.STILL;
            JointMovement shoulderMovement = JointMovement.STILL;

            if (elbow.getPosition() > 0.95)
                elbowMovement = JointMovement.FORWARD;

            if (arm.getImaginaryShoulderPosition() > 0.55)
                shoulderMovement = JointMovement.FORWARD;

            if ((elbowMovement == JointMovement.STILL) && (shoulderMovement == JointMovement.STILL)) {
                isArmPositionSet = true;
                return;
            }

            arm.moveArm(shoulderMovement, elbowMovement, JointMovement.STILL, WristDirection.STILL, GrabberMovement.OPEN, MoveSpeed.SLOW);
        }

    }

    @Override
    public void loop() {
        if (!isJewelInitialized) {
            jewel.setPosition(0.4);
            isJewelInitialized = true;
            isJewelPullBack = false;
            jewelInitializeTime = System.currentTimeMillis();
        } else if (!isArmInitialized) {//initialize arm servo values here, not in init() so that the arm will not move during init and violate the game rule
            initializeArm();
        } else if (!isArmPositionSet) { //Move the arm out of way from the grabber first, only if it has not yet been moved away
            presetArmPosition();
        } else if ((!isJewelPullBack) && ((System.currentTimeMillis() - jewelInitializeTime) > 4000)) {
            jewel.setPosition(.9);
            isJewelPullBack = true;
        }

        wheels.drive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1);

        double liftPosition = lift.getCurrentPosition();
        if(gamepad2.dpad_up && (liftPosition < 4400 || gamepad2.back)) {
            lift.setPower(.55);
        } else if(gamepad2.dpad_down && (liftPosition > 60 || gamepad2.back)) {
            lift.setPower(-.6);
        } else {
            lift.setPower(0);
        }
        telemetry.addData("lift:", liftPosition);

        arm.moveArm(gamepad2);

        if (gamepad2.y) {
            arm.placeRelic();
        }

        if(gamepad2.x) {
            arm.grabRelic();
        } else if(gamepad2.a) {
            arm.pullArmBack();
        }

        if(gamepad1.right_trigger > .5) {
            if(!isTriggerBeingPressed) {
                isTriggerBeingPressed = true;

                if(isGrabberClosed)
                    grabber.setPosition(1);
                else
                    grabber.setPosition(0);
            }
        }
        else if (isTriggerBeingPressed) {
            if(grabber.getPosition() > .9) {
                isGrabberClosed = false;
            } else if(grabber.getPosition() < .1) {
                isGrabberClosed = true;
            }

            isTriggerBeingPressed = false;
        }


        if(gamepad1.right_bumper) {
            if(!isBumperBeingPressed) {
                isBumperBeingPressed = true;

                if(isGrabber2Closed)
                    grabber2.setPosition(1);
                else
                    grabber2.setPosition(0);
            }
        }
        else if (isBumperBeingPressed) {
            if(grabber2.getPosition() > .9) {
                isGrabber2Closed = false;
            } else if(grabber2.getPosition() < .1) {
                isGrabber2Closed = true;
            }

            isBumperBeingPressed = false;
        }


        telemetry.update();
    }
}

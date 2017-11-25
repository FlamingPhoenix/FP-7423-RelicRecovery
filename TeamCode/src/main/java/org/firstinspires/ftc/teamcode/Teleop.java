package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoControllerEx;

import org.firstinspires.ftc.teamcode.FlamingPhoenix.Arm;
import org.firstinspires.ftc.teamcode.FlamingPhoenix.Drive;

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

    DigitalChannel touchtop;
    DigitalChannel touchbot;


    double elbowPosition;
    double shoulderPosition = .5;

    Drive wheels;

    BNO055IMU imu;

    Arm arm;



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
        PwmControl.PwmRange grabberPwmRange = new PwmControl.PwmRange(1418, 2200);
        grabberController.setServoPwmRange(grabberServoPort, grabberPwmRange);

        ServoControllerEx grabber2Controller = (ServoControllerEx) grabber2.getController();
        int grabber2ServoPort = grabber2.getPortNumber();
        PwmControl.PwmRange grabber2PwmRange = new PwmControl.PwmRange(1418, 2200);
        grabber2Controller.setServoPwmRange(grabber2ServoPort, grabber2PwmRange);

        ServoControllerEx servoController = (ServoControllerEx) shoulder.getController();
        int shoulderServoPort = shoulder.getPortNumber();
        PwmControl.PwmRange shoulderPwmRange = new PwmControl.PwmRange(1015, 1776);
        servoController.setServoPwmRange(shoulderServoPort, shoulderPwmRange);

        ServoControllerEx elbowController = (ServoControllerEx) elbow.getController();
        int elbowServoPort = elbow.getPortNumber();
        PwmControl.PwmRange elbowPwmRange = new PwmControl.PwmRange(1013, 2105);
        elbowController.setServoPwmRange(elbowServoPort, elbowPwmRange);

        ServoControllerEx wristController = (ServoControllerEx) wrist.getController();
        int wristServoPort = wrist.getPortNumber();
        PwmControl.PwmRange wristPwmRange = new PwmControl.PwmRange(750, 2250);
        wristController.setServoPwmRange(wristServoPort, wristPwmRange);

        ServoControllerEx elevatorController = (ServoControllerEx) elevator.getController();
        int elevatorServoPort = elevator.getPortNumber();
        PwmControl.PwmRange elevatorPwmRange = new PwmControl.PwmRange(899, 2105);
        elevatorController.setServoPwmRange(elevatorServoPort, elevatorPwmRange);


        double shoulderInitialize = 0;

        shoulder.setPosition(shoulderInitialize);

        elbow.setPosition(.8);
        wrist.setPosition(0);
        wristation.setPosition(.5);
        finger.setPosition(1);

        elevator.setPosition(.5);

        touchtop = hardwareMap.get(DigitalChannel.class, "tt");
        touchbot = hardwareMap.get(DigitalChannel.class, "tb");

        touchtop.setMode(DigitalChannel.Mode.INPUT);
        touchbot.setMode(DigitalChannel.Mode.INPUT);

        arm = new Arm(shoulder, elbow, wrist, wristation, finger, shoulderInitialize, this);

        bl.setDirection(DcMotor.Direction.REVERSE);
        fl.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void loop() {
        wheels.drive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1);

        if(gamepad2.dpad_up) {
            lift.setPower(.4);
        } else if(gamepad2.dpad_down) {
            lift.setPower(-.4);
        } else {
            lift.setPower(0);
        }

        if(elevator.getPosition() < 0.5 && touchtop.getState() == false) {
            elevator.setPosition(.5);
        } else if(elevator.getPosition() > 0.5 && touchbot.getState() == false) {
            elevator.setPosition(.5);
        } else if(gamepad1.dpad_up) {
            elevator.setPosition(0);
        } else if(gamepad1.dpad_down) {
            elevator.setPosition(1);
        }

        arm.moveArm(gamepad2);

        if (gamepad2.y) {
            arm.placeRelic();
        }


        if(gamepad1.right_trigger > .5) {
            grabber.setPosition(0);
        } else if (gamepad1.right_bumper)
            grabber.setPosition(1);

        if(gamepad1.left_trigger > .5) {
            grabber2.setPosition(0);
        } else if (gamepad1.left_bumper)
            grabber2.setPosition(1);


        telemetry.addData("liftposition", lift.getCurrentPosition());
        telemetry.addData("elevator", elevator.getPosition());

    }
}

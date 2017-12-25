package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoControllerEx;

import org.firstinspires.ftc.teamcode.FlamingPhoenix.Direction;
import org.firstinspires.ftc.teamcode.FlamingPhoenix.Drive;
import org.firstinspires.ftc.teamcode.FlamingPhoenix.Vuforia;

/**
 * Created by HwaA1 on 10/19/2017.
 */

@Autonomous(name = "test", group = "none")
public class Test extends LinearOpMode {

    DcMotor br;
    DcMotor bl;
    DcMotor fl;
    DcMotor fr;

    Servo grabber;

    //Servo jewel;

    BNO055IMU imu;

    Drive wheels;

    ColorSensor color;

    @Override
    public void runOpMode() throws InterruptedException {

        bl = hardwareMap.dcMotor.get("backleft");
        br = hardwareMap.dcMotor.get("backright");
        fl = hardwareMap.dcMotor.get("frontleft");
        fr = hardwareMap.dcMotor.get("frontright");

        bl.setDirection(DcMotor.Direction.REVERSE);
        fl.setDirection(DcMotor.Direction.REVERSE);

        color = hardwareMap.get(ColorSensor.class, "color");
        color.enableLed(true);

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        grabber = hardwareMap.servo.get("grabber");

        //jewel = hardwareMap.servo.get("jewel");

        ServoControllerEx grabberController = (ServoControllerEx) grabber.getController();
        int grabberServoPort = grabber.getPortNumber();
        PwmControl.PwmRange grabberPwmRange = new PwmControl.PwmRange(1418, 2200);
        grabberController.setServoPwmRange(grabberServoPort, grabberPwmRange);

        //ServoControllerEx jewelController = (ServoControllerEx) jewel.getController();
        //int jewelServoPort = jewel.getPortNumber();
        //PwmControl.PwmRange jewelPwmRange = new PwmControl.PwmRange(899, 2105);
        //jewelController.setServoPwmRange(jewelServoPort, jewelPwmRange);


        grabber.setPosition(1);

        //jewel.setPosition(1);

        wheels = new Drive(fr, br, fl, bl, imu, this);

        waitForStart();

        while(opModeIsActive()) {
            telemetry.addData("color blue:", color.blue());
            telemetry.addData("color red:", color.red());
            telemetry.update();
        }
    }
}

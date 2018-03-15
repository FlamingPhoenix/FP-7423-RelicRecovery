package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.FlamingPhoenix.Direction;
import org.firstinspires.ftc.teamcode.FlamingPhoenix.Drive;
import org.firstinspires.ftc.teamcode.FlamingPhoenix.OpModeInitializer;
import org.firstinspires.ftc.teamcode.FlamingPhoenix.Vuforia;
import org.firstinspires.ftc.teamcode.FlamingPhoenix.MyIMU;

/**
 * Created by HwaA1 on 11/4/2017.
 */

@Autonomous(name = "Blue_LEFT", group = "none")
public class CraptonBlueLeft extends LinearOpMode {

    DcMotor br;
    DcMotor bl;
    DcMotor fr;
    DcMotor fl;

    Servo upperGrabber;
    Servo grabber;

    Servo jewel;
    Servo jewelbase;

    Servo shoulder;
    Servo wrist;
    Servo elbow;

    Drive wheels;

    BNO055IMU imu;
    MyIMU myImu;

    VoltageSensor vSen1;
    VoltageSensor vSen2;

    ColorSensor color;

    @Override
    public void runOpMode() throws InterruptedException {
        Vuforia vu = new Vuforia(this);

        OpModeInitializer opModeInitializer = new OpModeInitializer();

        bl = hardwareMap.dcMotor.get("backleft");
        br = hardwareMap.dcMotor.get("backright");
        fl = hardwareMap.dcMotor.get("frontleft");
        fr = hardwareMap.dcMotor.get("frontright");

        bl.setDirection(DcMotor.Direction.REVERSE);
        fl.setDirection(DcMotor.Direction.REVERSE);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        myImu = new MyIMU(imu);

        vSen1 = hardwareMap.voltageSensor.get("Expansion Hub 2");
        vSen2 = hardwareMap.voltageSensor.get("Expansion Hub 3");

        color = hardwareMap.get(ColorSensor.class, "color");
        color.enableLed(true);

        grabber = hardwareMap.servo.get("grabber");
        upperGrabber = hardwareMap.servo.get("grabber2");
        jewel = hardwareMap.servo.get("jewel");
        jewelbase = hardwareMap.servo.get("jewelbase");

        jewel.setPosition(.1);
        jewelbase.setPosition(.2);

        opModeInitializer.initializeAutoGrabbers(grabber, upperGrabber, jewel);

        shoulder = hardwareMap.servo.get("shoulder");
        wrist = hardwareMap.servo.get("wrist");
        elbow = hardwareMap.servo.get("elbow");

        wheels = new Drive(fr, br, fl, bl, imu, vSen1, vSen2, this);

        telemetry.addData("isOpModeActive", this.isStarted());
        telemetry.update();

        vu.activate();

        waitForStart();

        double startingHeading = myImu.getHeading();
        double strafingDistance = 7.5;

        //scan for image first
        if(vu.scanVuforia() == 1) {
            strafingDistance = 12;
        } else if(vu.scanVuforia() == 0) {
            strafingDistance = 6.5;
        } else if(vu.scanVuforia() == -1) {
            strafingDistance = 2;
        }

        grabber.setPosition(0);
        upperGrabber.setPosition(0);

        double distanceFromKey = vu.getZ();

        Log.d("[Phoenix-vu-strafe]", "differenceFromKey: " + distanceFromKey);
        if(distanceFromKey == -9999.0)
            distanceFromKey = -365.0;

        Log.d("[Phoenix-vu-strafe]", "differenceFromKey: " + distanceFromKey);


        double difference =(-distanceFromKey) - 370;
        difference *= 0.0394;

        Direction strafeD = Direction.RIGHT;
        if(difference > 0) {
            strafeD = Direction.LEFT;
        }

        if (strafeD == Direction.RIGHT){
            wheels.strafe(Math.abs(difference), .2, strafeD, this);
        }

        Thread.sleep(500);

        jewelbase.setPosition(.3);
        Thread.sleep(500);
        jewelbase.setPosition(.2);

        jewel.setPosition(.675);

        Thread.sleep(500);

        distanceFromKey = vu.getZ();
        if(distanceFromKey == -9999.0)
            distanceFromKey = -365.0;

        difference = (-distanceFromKey) - 370;
        difference *= 0.0394;

        if(difference > 0)
            strafeD = Direction.LEFT;

        wheels.strafe(Math.abs(difference), .2, strafeD, this);

        Log.d("[Phoenix-vu-strafe]", "distance to strafe: " + difference);

        Thread.sleep(500);

        int redValue = color.red();
        int blueValue = color.blue();

        if((blueValue - redValue) >= 10) {
            jewelbase.setPosition(.0);
        } else if ((redValue - blueValue) >= 10) {
            jewelbase.setPosition(.4);
        }
        telemetry.addData("blue", color.blue());
        telemetry.addData("red", color.red());
        telemetry.update();

        Thread.sleep(1000);

        jewel.setPosition(.1);
        Thread.sleep(200);
        jewelbase.setPosition(.2);

        Thread.sleep(1000);

        if(vu.scanVuforia() == 1) {
            strafingDistance = 12;
        } else if(vu.scanVuforia() == 0) {
            strafingDistance = 6.5;
        } else if(vu.scanVuforia() == -1) {
            strafingDistance = 2;
        }

        float imageAngle = vu.getXAngle();
        strafingDistance -= vu.getAddDistance(imageAngle, 40);

        Log.d("[Phoenix-Adjustment]", "adjustment distance: " + vu.getAddDistance(imageAngle, 22) + ". image angle: " + imageAngle + ". newstrafingDistance: " + strafingDistance);

        wheels.drive(22, Direction.FORWARD, .4, this);

        Thread.sleep(1000);

        Direction adjTurnDirection;
        wheels.strafe(strafingDistance, .5, Direction.RIGHT, this);

        double currentHeading = myImu.getHeading();

        Log.d("[Phoenix-AdjTurn]", "currentHeading: " + currentHeading + ". startHeading: " + startingHeading);

        if(Math.abs(currentHeading - startingHeading) > 2.5) {
            if (currentHeading > 0) //imu left is positive, right is negative
                adjTurnDirection = Direction.RIGHT;
            else
                adjTurnDirection = Direction.LEFT;

            wheels.adjustmentTurn(wheels.turnPower() - .025, adjTurnDirection, strafingDistance);
        }

        wheels.drive(9, Direction.FORWARD, .5, this);

        grabber.setPosition(1);
        upperGrabber.setPosition(1);
        Thread.sleep(1000);

        wheels.drive(5, Direction.BACKWARD, .4, this);

        wheels.turnByIMU(70, .5, Direction.RIGHT);
    }
}

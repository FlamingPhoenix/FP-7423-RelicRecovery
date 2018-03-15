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

@Autonomous(name = "Red_RIGHT", group = "none")
public class CraptonRedRight extends LinearOpMode {

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

    ColorSensor color;

    VoltageSensor vSen1;
    VoltageSensor vSen2;

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

        color = hardwareMap.get(ColorSensor.class, "color");
        color.enableLed(true);

        vSen1 = hardwareMap.voltageSensor.get("Expansion Hub 2");
        vSen2 = hardwareMap.voltageSensor.get("Expansion Hub 3");

        grabber = hardwareMap.servo.get("grabber");
        upperGrabber = hardwareMap.servo.get("grabber2");
        jewel = hardwareMap.servo.get("jewel");
        jewelbase = hardwareMap.servo.get("jewelbase");

        jewelbase.setPosition(.2);
        jewel.setPosition(.1);

        opModeInitializer.initializeAutoGrabbers(grabber, upperGrabber, jewel);

        shoulder = hardwareMap.servo.get("shoulder");
        wrist = hardwareMap.servo.get("wrist");
        elbow = hardwareMap.servo.get("elbow");


        opModeInitializer.initalizeAutoArm(shoulder, elbow, wrist);

        wheels = new Drive(fr, br, fl, bl, imu, vSen1, vSen2, this);

        vu.activate();

        telemetry.addData("isOpModeReady", "true");
        telemetry.addData("image angle", vu.getXAngle());
        telemetry.addData("adjustmentdistance", vu.getAddDistance(vu.getXAngle(), 22));
        telemetry.update();

        waitForStart();

        double startingHeading = myImu.getHeading();
        double strafingDistance = 8;

        Thread.sleep(1000);

        grabber.setPosition(0);
        upperGrabber.setPosition(0);

        wheels.drive(1, Direction.FORWARD, .2, this);

        Thread.sleep(1000);

        double distanceFromKey = vu.getZ();
        if(distanceFromKey == -9999)
            distanceFromKey = -370;

        double difference =(-distanceFromKey) - 370;
        difference *= 0.0394;

        Direction strafeD = Direction.LEFT;
        if(difference > 0) {
            strafeD = Direction.RIGHT;
        }

        Log.d("[Phoenix-vu-strafe]", "distance to strafe: " + difference);

        if (strafeD == Direction.LEFT){
            wheels.strafe(Math.abs(difference), .25, strafeD, this);
        }

        jewelbase.setPosition(.3);
        Thread.sleep(1000);
        jewelbase.setPosition(.2);

        jewel.setPosition(.675);

        Thread.sleep(250);

        distanceFromKey = vu.getZ();
        difference = (-distanceFromKey) - 370;
        difference *= 0.0394;

        if(difference > 0) {
            strafeD = Direction.RIGHT;
        }

        wheels.strafe(Math.abs(difference), .25, strafeD, this);

        Log.d("[Phoenix-vu-strafe]", "distance to strafe: " + difference);

        Thread.sleep(1000);

        int redValue = color.red();
        int blueValue = color.blue();

        if((redValue - blueValue) >= 10) {
            jewelbase.setPosition(0);
        } else if (blueValue - redValue >= 10) {
            jewelbase.setPosition(.4);
        }

        Thread.sleep(1000);

        jewel.setPosition(0);
        Thread.sleep(1000);
        jewelbase.setPosition(.18);

        Thread.sleep(500);

        wheels.drive(2, Direction.FORWARD, .15, this);

        Thread.sleep(2000);

        if(vu.scanVuforia() == 1) {
            strafingDistance = 2;
        } else if(vu.scanVuforia() == 0) {
            strafingDistance = 6.5;
        } else if(vu.scanVuforia() == -1) {
            strafingDistance = 12;
        }


        float imageAngle = vu.getXAngle();
        strafingDistance += vu.getAddDistance(imageAngle, 22);

        Log.d("[Phoenix-Adjustment]", "adjustment distance: " + vu.getAddDistance(imageAngle, 22) + ". image angle: " + imageAngle + ". newstrafingDistance: " + strafingDistance);

        wheels.drive(18, Direction.FORWARD, .4, this);

        Thread.sleep(1000);

        wheels.strafe(strafingDistance, .5, Direction.LEFT, myImu, this);

        Thread.sleep(250);

        Direction adjTurnDirection;
        double currentHeading = myImu.getHeading();

        Log.d("[Phoenix-AdjTurn]", "currentHeading: " + currentHeading + ". startHeading: " + startingHeading);

        if(Math.abs(currentHeading - startingHeading) > 1.5) {
            if (currentHeading > 0) //imu left is positive, right is negative
                adjTurnDirection = Direction.RIGHT;
            else
                adjTurnDirection = Direction.LEFT;

            wheels.adjustmentTurn(wheels.turnPower(), adjTurnDirection, strafingDistance);
        }

        wheels.drive(8, Direction.FORWARD, .5, this);

        grabber.setPosition(1);
        upperGrabber.setPosition(1);

        Thread.sleep(1000);

        wheels.drive(3, Direction.BACKWARD, .5, this);
    }
}
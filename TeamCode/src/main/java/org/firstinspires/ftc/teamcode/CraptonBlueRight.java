/**
 * Created by HwaA1 on 11/4/2017.
 */

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
import org.firstinspires.ftc.teamcode.FlamingPhoenix.OpModeInitializer;
import org.firstinspires.ftc.teamcode.FlamingPhoenix.Vuforia;

@Autonomous(name = "Blue_RIGHT", group = "none")
public class CraptonBlueRight extends LinearOpMode {

    DcMotor br;
    DcMotor bl;
    DcMotor fr;
    DcMotor fl;

    Servo upperGrabber;
    Servo grabber;

    Servo jewel;
    Servo jewelbase;

    Drive wheels;

    BNO055IMU imu;

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

        color = hardwareMap.get(ColorSensor.class, "color");
        color.enableLed(true);

        grabber = hardwareMap.servo.get("grabber");
        upperGrabber = hardwareMap.servo.get("grabber2");
        jewel = hardwareMap.servo.get("jewel");
        jewelbase = hardwareMap.servo.get("jewelbase");

        jewel.setPosition(.1);
        jewelbase.setPosition(.2);

        opModeInitializer.initializeAutoGrabbers(grabber, upperGrabber, jewel);

        wheels = new Drive(fr, br, fl, bl, imu, this);

        telemetry.addData("isOpModeActive", this.isStarted());
        telemetry.update();

        vu.activate();

        telemetry.addData("isOpModeActive", this.isStarted());
        telemetry.update();

        waitForStart();

        int cryptodistance = 12;

        if(vu.scanVuforia() == -1) {
            cryptodistance = 5;
        } else if (vu.scanVuforia() == 0) {
            cryptodistance = 12;
        } else if (vu.scanVuforia() == 1) {
            cryptodistance = 19;
        }

        Thread.sleep(1000);

        grabber.setPosition(0);
        upperGrabber.setPosition(0);

        double distanceFromKey = vu.getZ();

        Log.d("[Phoenix-vu-strafe]", "differenceFromKey: " + distanceFromKey);
        double oldDistance = distanceFromKey;

        if(distanceFromKey == -9999.0)
            distanceFromKey = -380.0;

        //Log.d("[Phoenix-vu-strafe]", "differenceFromKey: " + distanceFromKey);


        double difference =(-distanceFromKey) - 385;
        difference *= 0.0394;

        telemetry.addData("old distance", oldDistance);
        telemetry.addData("distanceFromKey", distanceFromKey);
        telemetry.addData("difference", difference);
        telemetry.update();

        Direction strafeD = Direction.RIGHT;
        if(difference > 0) {
            strafeD = Direction.LEFT;
        }

        if (strafeD == Direction.RIGHT){
            wheels.strafe(Math.abs(difference), .225, strafeD, this);
        }

        Thread.sleep(500);

        jewelbase.setPosition(.3);
        Thread.sleep(1000);
        jewelbase.setPosition(.2);

        jewel.setPosition(.675);

        Thread.sleep(500);

        distanceFromKey = vu.getZ();
        if(distanceFromKey == -9999.0)
            distanceFromKey = -385.0;

        difference = (-distanceFromKey) - 385;
        difference *= 0.0394;

        if(difference > 0)
            strafeD = Direction.LEFT;

        wheels.strafe(Math.abs(difference), .225, strafeD, this);

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

        Thread.sleep(500);

        wheels.drive(22, Direction.FORWARD, .3, this);

        wheels.drive(cryptodistance, Direction.FORWARD, 0.4, this);
        wheels.turnByIMU(75, .4, Direction.LEFT);
        wheels.drive(8, Direction.FORWARD, .4, this);

        grabber.setPosition(1);
        upperGrabber.setPosition(1);

        Thread.sleep(1000);

        wheels.drive(3, Direction.BACKWARD, .5, this);

        wheels.turnByIMU(160, .5, Direction.LEFT);
    }
}

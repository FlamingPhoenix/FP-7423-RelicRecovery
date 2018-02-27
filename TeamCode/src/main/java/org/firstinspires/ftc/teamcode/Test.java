package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoControllerEx;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
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

    Servo jewel;

    BNO055IMU imu;

    Drive wheels;

    ColorSensor color;

    DistanceSensor opt;

    Vuforia vu;

    @Override
    public void runOpMode() throws InterruptedException {

        vu = new Vuforia(this);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        opt = hardwareMap.get(DistanceSensor.class, "color");

        color = hardwareMap.get(ColorSensor.class, "color");
        color.enableLed(true);

        vu.activate();

        waitForStart();

        while(true && opModeIsActive()) {
            /*telemetry.addData("distance", opt.getDistance(DistanceUnit.INCH));
            telemetry.addData("alpha", color.alpha());
            telemetry.addData("red", color.red());
            telemetry.addData("blue", color.blue());
            telemetry.addData("", color.argb());*/

            telemetry.addData("Z distance", vu.getZ());

            telemetry.update();
        }
    }
}

package org.firstinspires.ftc.teamcode.Library;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.FlamingPhoenix.Direction;
import org.firstinspires.ftc.teamcode.FlamingPhoenix.Drive;

/**
 * Created by HwaA1 on 10/19/2017.
 */

@Autonomous(name = "imutest", group = "none")
public class Test extends LinearOpMode {

    DcMotor backright;
    DcMotor backleft;
    DcMotor frontleft;
    DcMotor frontright;

    BNO055IMU imu;

    Drive wheels;

    @Override
    public void runOpMode() throws InterruptedException {
        backleft = hardwareMap.dcMotor.get("backleft");
        backright = hardwareMap.dcMotor.get("backright");
        frontleft = hardwareMap.dcMotor.get("frontleft");
        frontright = hardwareMap.dcMotor.get("frontright");

        backleft.setDirection(DcMotor.Direction.REVERSE);
        frontleft.setDirection(DcMotor.Direction.REVERSE);

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        wheels = new Drive(frontright, backright, frontleft, backleft, imu, this);

        waitForStart();

        wheels.turnByIMU(45, .8, Direction.LEFT);
        Thread.sleep(1000);
        wheels.turnByIMU(90, .8, Direction.RIGHT);

    }
}

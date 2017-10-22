package org.firstinspires.ftc.teamcode.FlamingPhoenix;

import android.graphics.drawable.GradientDrawable;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * Created by HwaA1 on 10/19/2017.
 */

public class Drive {

    DcMotor fr;
    DcMotor br;
    DcMotor fl;
    DcMotor bl;
    BNO055IMU imu;
    Orientation angles;

    OpMode op;

    LinearOpMode opm;

    public Drive(DcMotor frmotor, DcMotor brmotor, DcMotor flmotor, DcMotor blmotor, BNO055IMU im, OpMode opmode) {
        fr = frmotor;
        br = brmotor;
        fl = flmotor;
        bl = blmotor;

        imu = im;

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu.initialize(parameters);

        angles = imu.getAngularOrientation();

        op = opmode;

    }

    public Drive(DcMotor frmotor, DcMotor brmotor, DcMotor flmotor, DcMotor blmotor, BNO055IMU im, LinearOpMode opmode) {
        fr = frmotor;
        br = brmotor;
        fl = flmotor;
        bl = blmotor;

        imu = im;

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu.initialize(parameters);

        angles = imu.getAngularOrientation();

        opm = opmode;

    }

    //TeleOp Methods


    //Autonomous Methods

    public void turnByIMU(int degree, double speed, Direction direction) {
        double startAngle = angles.firstAngle;
        double targetAngle = startAngle + (direction == Direction.RIGHT ? degree * -1 : degree);

        if(direction == Direction.LEFT) {
            while (angles.firstAngle < targetAngle && opm.opModeIsActive()) {
                fr.setPower(speed);
                br.setPower(speed);
                fl.setPower(-speed);
                bl.setPower(-speed);

                angles = imu.getAngularOrientation();
            }
        }
        else if(direction == Direction.RIGHT) {
            while (angles.firstAngle > targetAngle && opm.opModeIsActive()) {
                fr.setPower(-speed);
                br.setPower(-speed);
                fl.setPower(speed);
                bl.setPower(speed);

                angles = imu.getAngularOrientation();
            }
        }
        fr.setPower(0);
        br.setPower(0);
        fl.setPower(0);
        bl.setPower(0);

    }

}


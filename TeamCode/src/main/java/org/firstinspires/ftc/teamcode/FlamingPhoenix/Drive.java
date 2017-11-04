package org.firstinspires.ftc.teamcode.FlamingPhoenix;

import android.graphics.drawable.GradientDrawable;
import android.util.Log;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.internal.android.dx.cf.direct.DirectClassFile;

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

    public void drive(float x1, float y1, float x2) {
        x1 = (float) scaleInput(x1);
        y1 = (float) scaleInput(y1 * -1);

        float frontLeft = y1 + x1 + x2;
        float frontRight = y1 - x1 - x2;
        float backLeft = y1 - x1 + x2;
        float backRight = y1 + x1 - x2;

        float mv = max(Math.abs(frontLeft), Math.abs(frontRight), Math.abs(backLeft), Math.abs(backRight));
        if (Math.abs(mv) > 1) {
            frontLeft = frontLeft / mv;
            frontRight = frontRight / mv;
            backLeft = backLeft / mv;
            backRight = backRight / mv;
        }

        frontLeft = Range.clip(frontLeft, -1, 1);
        frontRight = Range.clip(frontRight, -1, 1);
        backLeft = Range.clip(backLeft, -1, 1);
        backRight = Range.clip(backRight, -1, 1);

        fl.setPower(frontLeft / 2);
        fr.setPower(frontRight / 2);
        bl.setPower(backLeft / 2);
        br.setPower(backRight / 2);
    }

    private double scaleInput(double dVal) {
        double[] scaleArray = {0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00};

        // get the corresponding index for the scaleInput array.
        int index = (int) (dVal * 16.0);
        if (index < 0) {
            index = -index;
        } else if (index > 16) {
            index = 16;
        }

        double dScale = 0.0;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }

        return dScale;
    }

    private float max(float... args) {
        float m = 0;

        for (int i = 0; i < args.length; i++) {
            if (args[i] > m)
                m = args[i];
        }

        return m;
    }

    //Autonomous Methods

    public void turnByIMU(int degree, double power, Direction direction) {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double raw = angles.firstAngle;
        double startAngle = translateAngle((float) raw, direction);
        double targetAngle = startAngle + (direction == Direction.RIGHT ? degree * -1 : degree);

        Log.d("[Phoenix]","[Phoenix]" + Double.toString(translateAngle((float) raw, direction)) + " and here is target " + Double.toString(targetAngle));

        long starttime = 0;
        long elapsedtime = 0;

        double speed = 0;

        boolean timeset = false;

        if(direction == Direction.LEFT) {
            while (translateAngle((float) raw, direction) < targetAngle && opm.opModeIsActive()) {

                if(translateAngle((float)(raw), direction) > 5) {

                    if(!timeset) {
                        starttime = System.currentTimeMillis();
                    }

                    elapsedtime = (System.currentTimeMillis() - starttime) / 1000;
                    double currentangle = translateAngle((float) raw, direction);

                    speed = (Math.abs(currentangle - startAngle)) / elapsedtime;
                }

                fr.setPower(power);
                br.setPower(power);
                fl.setPower(-power);
                bl.setPower(-power);

                Log.d("[Phoenix]","[Phoenix0]" + Double.toString(translateAngle((float) raw, direction)) + " and here is target " + Double.toString(targetAngle));

                opm.telemetry.addData("angle", translateAngle((float) raw, direction));
                opm.telemetry.update();
            }
        }
        else if(direction == Direction.RIGHT) {

            if(translateAngle((float)(raw), direction) > 5) {

                if(!timeset) {
                    starttime = System.currentTimeMillis();
                }

                elapsedtime = (System.currentTimeMillis() - starttime) / 1000;
                double currentangle = translateAngle((float) raw, direction);

                speed = (Math.abs(currentangle - startAngle)) / elapsedtime;
            }

            while (translateAngle((float) raw, direction) > targetAngle && opm.opModeIsActive()) {
                fr.setPower(-power);
                br.setPower(-power);
                fl.setPower(power);
                bl.setPower(power);

                Log.d("[Phoenix]","[Phoenix1]" + Double.toString(translateAngle((float) raw, direction)) + " and here is target " + Double.toString(targetAngle));

                opm.telemetry.addData("angle", translateAngle((float) raw, direction));
                opm.telemetry.update();
            }
        }
        fr.setPower(0);
        br.setPower(0);
        fl.setPower(0);
        bl.setPower(0);

    }

    private double translateAngle(float s, Direction d) {
        Orientation rawOrientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

         if(s > 0 && d == Direction.LEFT)
            return rawOrientation.firstAngle >= 0 ? rawOrientation.firstAngle : 360 + rawOrientation.firstAngle;

         else if ( s < 0 && d == Direction.RIGHT)
             return rawOrientation.firstAngle >= 0 ? rawOrientation.firstAngle : 360 + rawOrientation.firstAngle;

         else
            return rawOrientation.firstAngle;

    }

}


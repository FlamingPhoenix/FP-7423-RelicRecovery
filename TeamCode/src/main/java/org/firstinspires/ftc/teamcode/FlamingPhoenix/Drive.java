package org.firstinspires.ftc.teamcode.FlamingPhoenix;

import android.graphics.drawable.GradientDrawable;
import android.util.Log;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

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




    //Autonomous Methods

    public void turnByIMU(int degree, double speed, Direction direction) {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double raw = angles.firstAngle;
        double startAngle = translateAngle((float) raw, direction);
        double targetAngle = startAngle + (direction == Direction.RIGHT ? degree * -1 : degree);

        Log.d("[Phoenix]","[Phoenix]" + Double.toString(translateAngle((float) raw, direction)) + " and here is target " + Double.toString(targetAngle));

        if(direction == Direction.LEFT) {
            while (translateAngle((float) raw, direction) < targetAngle && opm.opModeIsActive()) {
                fr.setPower(speed);
                br.setPower(speed);
                fl.setPower(-speed);
                bl.setPower(-speed);

                Log.d("[Phoenix]","[Phoenix0]" + Double.toString(translateAngle((float) raw, direction)) + " and here is target " + Double.toString(targetAngle));

                opm.telemetry.addData("angle", translateAngle((float) raw, direction));
                opm.telemetry.update();
            }
        }
        else if(direction == Direction.RIGHT) {
            while (translateAngle((float) raw, direction) > targetAngle && opm.opModeIsActive()) {
                fr.setPower(-speed);
                br.setPower(-speed);
                fl.setPower(speed);
                bl.setPower(speed);

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


package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.FlamingPhoenix.Drive;
import org.firstinspires.ftc.teamcode.FlamingPhoenix.Vuforia;
import org.firstinspires.ftc.teamcode.FlamingPhoenix.MyIMU;

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

    Vuforia vu;

    VoltageSensor vSen1;
    VoltageSensor vSen2;

    MyIMU myImu;

    @Override
    public void runOpMode() throws InterruptedException {

        bl = hardwareMap.dcMotor.get("backleft");
        br = hardwareMap.dcMotor.get("backright");
        fl = hardwareMap.dcMotor.get("frontleft");
        fr = hardwareMap.dcMotor.get("frontright");

        bl.setDirection(DcMotor.Direction.REVERSE);
        fl.setDirection(DcMotor.Direction.REVERSE);

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        vSen1 = hardwareMap.voltageSensor.get("Expansion Hub 2");
        vSen2 = hardwareMap.voltageSensor.get("Expansion Hub 3");

        wheels = new Drive(fr, br, fl, bl, imu, vSen1, vSen2, this);

        vu = new Vuforia(this);

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        myImu = new MyIMU(imu);

        color = hardwareMap.get(ColorSensor.class, "color");
        color.enableLed(true);

        vu.activate();

        waitForStart();

        /*double turnPower = wheels.turnPower((vSen1.getVoltage() + vSen2.getVoltage()) / 2);

        Log.d("[Phoenix-AdjTurn]", "turnPower: " + turnPower + ". vSen1: " + vSen1.getVoltage() + ". vSen2: " + vSen2.getVoltage());

        wheels.adjustmentTurn(turnPower, Direction.LEFT);*/

        while(opModeIsActive()) {
            telemetry.addData("distance", vu.getZ());
            telemetry.update();
        }

    }
}

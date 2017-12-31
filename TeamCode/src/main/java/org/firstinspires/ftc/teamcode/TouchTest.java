package org.firstinspires.ftc.teamcode;

import android.text.method.Touch;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.TouchSensor;

/**
 * Created by HwaA1 on 11/21/2017.
 */

public class TouchTest extends OpMode {

    DigitalChannel touch;

    @Override
    public void init() {
        touch = hardwareMap.get(DigitalChannel.class ,"touch");

        touch.setMode(DigitalChannel.Mode.INPUT);
    }

    @Override
    public void loop() {

        if(touch.getState() == false) {
            telemetry.addData("touch sensor", "is Pressed");
        } else {
            telemetry.addData("touch sensor", "is not Pressed");
        }

        telemetry.update();
    }
}

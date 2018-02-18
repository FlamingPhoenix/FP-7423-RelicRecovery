package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.FlamingPhoenix.Vuforia;

import static com.sun.tools.javac.util.Constants.format;

/**
 * Created by HwaA1 on 11/21/2017.
 */

@Autonomous(name = "VuTest", group = "none")
public class VuforiaTest extends LinearOpMode {

    OpenGLMatrix lastLocation = null;

    VuforiaLocalizer vuforia;

    @Override
    public void runOpMode() throws InterruptedException {
        Vuforia vu = new Vuforia(this);
        vu.activate();

        waitForStart();

        while (opModeIsActive()) {
            double z = vu.getZ();
            double columnId = vu.getXAngle();

            telemetry.addData("z:", z);
            telemetry.addData("Column Id:", columnId);
            telemetry.update();
        }
    }
}

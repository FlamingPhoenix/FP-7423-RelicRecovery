package org.firstinspires.ftc.teamcode.FlamingPhoenix;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.vuforia.HINT;

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

/**
 * Created by HwaA1 on 11/25/2017.
 */

public class Vuforia {

    OpenGLMatrix lastLocation = null;

    VuforiaLocalizer vuforia;

    int cameraMonitorViewId;

    VuforiaTrackable relicTemplate;

    VuforiaLocalizer.Parameters parameters;
    VuforiaTrackables relicTrackables;

    public Vuforia(OpMode op) {
        cameraMonitorViewId = op.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", op.hardwareMap.appContext.getPackageName());
        parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = "AbYPrgD/////AAAAGbvKMH3NcEVFmPLgunQe4K0d1ZQi+afRLxricyooCq+sgY9Yh1j+bBrd0CdDCcoieA6trLCKBzymC515+Ps/FECtXv3+CTW6fg3/3+nvKZ6QA18h/cNZHg5HYHmghlcCgVUmSzOLRvdOpbS4S+0Y/sWGXwFK0PbuGPSN82w8XPDBoRYSWjAf8GXeitmNSlm9n4swrMoYNpMDuWCDjSm1kWnoErjFA9NuNoFzAgO+C/rYzoYjTJRk40ETVcAsahzatRlP7PJCvNNXiBhE6iVR+x7lFlTZ841xifOIOPkfVc54olC5XYe4A5ZmQ6WFD03W5HHdQrnmKPmkgcr1yqXAJ3rLTK8FZK3KVgbxz3Eeqps0";

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        com.vuforia.Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 4);

        relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate");
    }

    public void activate() {
       relicTrackables.activate();
    }

    public int scanVuforia() {
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);

        if(vuMark == RelicRecoveryVuMark.CENTER) {
            return 0;
        } else if(vuMark == RelicRecoveryVuMark.LEFT) {
            return -1;
        } else if(vuMark == RelicRecoveryVuMark.RIGHT) {
            return 1;
        } else if(vuMark == RelicRecoveryVuMark.UNKNOWN) {
            return 255;
        } else {
            return 255;
        }
    }

    public double getZ() {
        OpenGLMatrix pose = ((VuforiaTrackableDefaultListener)relicTemplate.getListener()).getPose();

        if (pose != null) {
            VectorF trans = pose.getTranslation();
            Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

            double tZ = trans.get(2);

            return tZ;
        }

        return 0;
    }

    public float getXAngle() {
        OpenGLMatrix pose = ((VuforiaTrackableDefaultListener)relicTemplate.getListener()).getPose();

        if (pose != null) {
            VectorF trans = pose.getTranslation();
            Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

            return rot.secondAngle;
        }

        return -9999;//return -9999 indicate we can't read the value
    }

    public double getAddDistance(float degrees, double distanceFromBalancePad) {
        float distance;

        if(degrees != -9999) {
            float radians = (float) Math.toRadians(degrees); //originally multiply by 2 because the angle that we recieved was inaccurate and was often smaller than what we would like. THIS WAS PUT IN AFTERWARDS

            if (Math.abs(radians) > 1.2)
                radians *= 2;

            float multiplier = (float) Math.sin(radians);

            distance = (float) (distanceFromBalancePad * multiplier);

            Log.d("[Phoenix-Adjustment]", "radians: " + radians + " multiplier: " + multiplier + " distance: " + distance);

            return distance;
        }

        return 0;
    }

}

/**
 * Created by Steve on 10/7/2017.
 */
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.*;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

//@TeleOp(name="Tank Drive", group="demo")
public class ScuttleBot extends LinearOpMode {

    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor backLeft;
    public DcMotor backRight;

    private void initialize() {
        frontLeft = hardwareMap.dcMotor.get("frontleft");
        frontRight = hardwareMap.dcMotor.get("frontright");
        backLeft = hardwareMap.dcMotor.get("backleft");
        backRight = hardwareMap.dcMotor.get("backright");

        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void runOpMode() {
        initialize();
        waitForStart();

        while (this.opModeIsActive()) {
            
        }
    }
}

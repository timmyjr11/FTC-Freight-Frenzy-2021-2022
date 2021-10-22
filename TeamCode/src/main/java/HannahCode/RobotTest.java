package HannahCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class RobotTest extends LinearOpMode {
    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;

    @Override
    public void runOpMode() throws InterruptedException {
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class,"frontRight");
        backLeft = hardwareMap.get(DcMotor.class,"backLeft");
        backRight = hardwareMap.get(DcMotor.class,"backRight");

        waitForStart();

        while (opModeIsActive()) {
            frontLeft.setPower(-gamepad1.left_stick_y);
            frontRight.setPower(-gamepad1.left_stick_y);
            backLeft.setPower(-gamepad1.left_stick_y);
            backRight.setPower(-gamepad1.left_stick_y);
        }

    }
}

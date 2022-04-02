package testCodes.robotTests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
@TeleOp
public class intakePositionTest extends LinearOpMode {

    DcMotorEx intake;

    int intakePosition;

    double error;

    double degreeCount = 360/384.5;

    double currentDegree;



    @Override
    public void runOpMode() throws InterruptedException {

        intake = hardwareMap.get(DcMotorEx.class, "intake");

        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intakePosition = intake.getCurrentPosition();
        currentDegree =  intakePosition * degreeCount;


        waitForStart();

        while (opModeIsActive()) {
            if (gamepad2.right_trigger >= 0.5) {
                intake.setPower(gamepad2.right_trigger * 0.8);
            } else if (gamepad2.left_trigger >= 0.5) {
                intake.setPower(-gamepad2.left_trigger * 0.95);
            } else {
                if (currentDegree % 180 == 0) {
                    intake.setPower(0);
                } else {
                    error = currentDegree - 180;

                    intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                    intake.setTargetPosition((int) Math.round((currentDegree - error)));

                    intake.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    intake.setPower(0.5);
                }
            }
        }
    }
}

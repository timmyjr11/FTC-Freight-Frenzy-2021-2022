package testCodes.robotTests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
@Config
@TeleOp
public class intakePositionTest extends LinearOpMode {

    DcMotorEx intake;

    int intakePosition;

    double halfWayPosition = 192.25;

    double error;
    private final FtcDashboard dashboard = FtcDashboard.getInstance();




    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        intake = hardwareMap.get(DcMotorEx.class, "intake");

        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intakePosition = intake.getCurrentPosition();

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad2.right_trigger >= 0.5) {
                intake.setPower(gamepad2.right_trigger * 0.8);
            } else if (gamepad2.left_trigger >= 0.5) {
                intake.setPower(-gamepad2.left_trigger * 0.95);
            } else {
                if (intakePosition % halfWayPosition == 0) {
                    intake.setPower(0);
                } else {
                    // If this does not work, create an enum that makes the error only update once
                    error = intakePosition - halfWayPosition;

                    intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                    intake.setTargetPosition((int) Math.round((intakePosition - error)));

                    intake.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    intake.setPower(0.5);
                }
            }
            telemetry.addData("error", error);
            telemetry.addData("position", intakePosition);
            telemetry.update();
        }
    }
}

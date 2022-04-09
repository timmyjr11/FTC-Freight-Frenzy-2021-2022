package testCodes.robotTests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
@Disabled
@TeleOp
@Config
public class intakeEncoder extends LinearOpMode {
    DcMotorEx intake;

    private final FtcDashboard dashboard = FtcDashboard.getInstance();


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        intake = hardwareMap.get(DcMotorEx.class, "intake");

        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        waitForStart();

        while (opModeIsActive()) {
            if (gamepad2.right_trigger >= 0.5) {
                intake.setPower(gamepad2.right_trigger * 0.8);
            } else if (gamepad2.left_trigger >= 0.5) {
                intake.setPower(-gamepad2.left_trigger * 0.95);
            }

            telemetry.addData("Positon", intake.getCurrentPosition());
            telemetry.update();
        }


    }
}

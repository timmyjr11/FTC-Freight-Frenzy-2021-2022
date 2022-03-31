package testCodes.PIDTuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
@Disabled
@Config
@Autonomous
public class myPersonalPIDTuner extends LinearOpMode {
    DcMotorEx testMotor;

    private double integral = 0;

    public static PIDCoefficients testPID = new PIDCoefficients(0, 0, 0);
    FtcDashboard dashboard;

    public static double TARGET_POS = 100;

    ElapsedTime PIDTimer = new ElapsedTime();

    private double lastError = 0;
    public double error = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        testMotor = hardwareMap.get(DcMotorEx.class, "testMotor");
        testMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        testMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        dashboard = FtcDashboard.getInstance();

        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        waitForStart();

        while (opModeIsActive()) {
            double power = PIDControl(TARGET_POS, testMotor.getCurrentPosition());
            testMotor.setPower(power);
            telemetry.addData("Target", TARGET_POS);
            telemetry.addData("testMotor position", testMotor.getCurrentPosition());
            telemetry.addData("error", error);
        }

    }
    public double PIDControl(double reference, double state) {
        double error = reference - state;
        integral += error * PIDTimer.time();
        double derivative = (error - lastError) / PIDTimer.time();
        lastError = error;
        PIDTimer.reset();

        return (error * testPID.p) + (integral * testPID.i) + (derivative * testPID.d);
    }
}

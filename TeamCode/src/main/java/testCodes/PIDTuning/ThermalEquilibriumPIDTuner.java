package testCodes.PIDTuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
@Config
@Autonomous
public class ThermalEquilibriumPIDTuner extends LinearOpMode {

    private final FtcDashboard dashboard = FtcDashboard.getInstance();


    DcMotorEx rightLiftMotor;
    DcMotorEx leftLiftMotor;

    double integralSum = 0;
   public static double Kp = 0;
   public static double Ki = 0;
   public static double Kd = 0;

   public static int reference = 300;

   ElapsedTime timer = new ElapsedTime();

    private double lastError = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        rightLiftMotor = hardwareMap.get(DcMotorEx.class, "rightLiftMotor");
        leftLiftMotor = hardwareMap.get(DcMotorEx.class, "leftLiftMotor");

        rightLiftMotor.setDirection(DcMotorSimple.Direction.REVERSE);


        rightLiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftLiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();
        while (opModeIsActive()) {
            double powerRight = PIDControl(300, rightLiftMotor.getCurrentPosition());
            double powerLeft = PIDControl(300, leftLiftMotor.getCurrentPosition());
            rightLiftMotor.setPower(powerRight);
            leftLiftMotor.setPower(powerLeft);
            telemetry.addData("right position", rightLiftMotor.getCurrentPosition());
            telemetry.addData("left position", leftLiftMotor.getCurrentPosition());
            telemetry.addData("reference", reference);
            telemetry.update();
        }
    }

    public double PIDControl(double reference, double state) {
        double error = reference - state;
        integralSum += error * timer.seconds();
        double derivative = (error - lastError) / timer.seconds();
        lastError = error;
        timer.reset();

        double output = (error * Kp) + (derivative * Kd) + (integralSum * Ki);
        return output;
    }
}

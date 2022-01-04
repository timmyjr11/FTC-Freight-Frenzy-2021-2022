package testCodes.robotTests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class liftTest extends LinearOpMode {
    DcMotorEx rightLiftMotor;
    DcMotorEx leftLiftMotor;


    @Override
    public void runOpMode() throws InterruptedException {
        rightLiftMotor = hardwareMap.get(DcMotorEx.class, "rightLiftMotor");
        leftLiftMotor = hardwareMap.get(DcMotorEx.class, "leftLiftMotor");

        rightLiftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leftLiftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        leftLiftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            if(gamepad1.dpad_up){
                rightLiftMotor.setVelocity(750);
                leftLiftMotor.setVelocity(750);
            }

            if (gamepad1.dpad_down){
                rightLiftMotor.setVelocity(-750);
                leftLiftMotor.setVelocity(-750);
            }

            if (!gamepad1.dpad_up && !gamepad1.dpad_down){
                rightLiftMotor.setVelocity(0);
                leftLiftMotor.setVelocity(0);
            }

            telemetry.addData("left motor", leftLiftMotor.getCurrentPosition());
            telemetry.addData("right motor", rightLiftMotor.getCurrentPosition());
            telemetry.update();
        }
    }
}

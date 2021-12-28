package testCodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class holdPosLiftTest<state> extends LinearOpMode {
    DcMotorEx rightLiftMotor;
    DcMotorEx leftLiftMotor;

    int currentLeftPosition;
    int currentRightPosition;

    @Override
    public void runOpMode() throws InterruptedException {
        rightLiftMotor = hardwareMap.get(DcMotorEx.class, "rightLiftMotor");
        leftLiftMotor = hardwareMap.get(DcMotorEx.class, "leftLiftMotor");

        rightLiftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leftLiftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        leftLiftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        currentLeftPosition = leftLiftMotor.getCurrentPosition();
        currentRightPosition = rightLiftMotor.getCurrentPosition();



        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            if(gamepad1.dpad_up){
                rightLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                leftLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightLiftMotor.setVelocity(750);
                leftLiftMotor.setVelocity(750);
            }

            if (gamepad1.dpad_down){
                rightLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                leftLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightLiftMotor.setVelocity(-750);
                leftLiftMotor.setVelocity(-750);
            }

            if (!gamepad1.dpad_up && !gamepad1.dpad_down){
                rightLiftMotor.setVelocity(0);
                leftLiftMotor.setVelocity(0);

                rightLiftMotor.setTargetPosition(currentRightPosition);
                leftLiftMotor.setTargetPosition(currentLeftPosition);

                rightLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                if (currentRightPosition < rightLiftMotor.getTargetPosition()) {
                    rightLiftMotor.setPower(0.2);
                }   else if (currentRightPosition > rightLiftMotor.getTargetPosition()) {
                        rightLiftMotor.setPower(0);
                }

                if (currentLeftPosition < leftLiftMotor.getTargetPosition()) {
                    leftLiftMotor.setPower(0.2);
                } else if (currentLeftPosition > leftLiftMotor.getTargetPosition()) {
                    leftLiftMotor.setPower(0);
                }
            }

            telemetry.addData("left motor", leftLiftMotor.getCurrentPosition());
            telemetry.addData("right motor", rightLiftMotor.getCurrentPosition());
            telemetry.update();
        }
    }
}

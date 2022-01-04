package testCodes.robotTests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class holdPosLiftTest extends LinearOpMode {
    DcMotorEx rightLiftMotor;
    DcMotorEx leftLiftMotor;

    Servo leftLinkage;
    Servo rightLinkage;

    boolean previousA = false;

    @Override
    public void runOpMode() throws InterruptedException {
        rightLiftMotor = hardwareMap.get(DcMotorEx.class, "rightLiftMotor");
        leftLiftMotor = hardwareMap.get(DcMotorEx.class, "leftLiftMotor");
        leftLinkage = hardwareMap.get(Servo.class, "leftLinkage");
        rightLinkage = hardwareMap.get(Servo.class, "rightLinkage");

        rightLiftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leftLiftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        rightLiftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        leftLinkage.setDirection(Servo.Direction.REVERSE);

        leftLinkage.setPosition(0);
        rightLinkage.setPosition(0);


        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {

            if (gamepad1.dpad_up && !gamepad1.dpad_down){
                rightLiftMotor.setVelocity(850);
                leftLiftMotor.setVelocity(850);
            } else if (gamepad1.dpad_down && !gamepad1.dpad_up){
                rightLiftMotor.setVelocity(-750);
                leftLiftMotor.setVelocity(-750);
            } else if (!gamepad1.dpad_up && !gamepad1.dpad_down) {
                leftLiftMotor.setVelocity(1);
                rightLiftMotor.setVelocity(1);
            }

            if (gamepad1.a && !previousA){
                if (leftLinkage.getPosition() == 0 && rightLinkage.getPosition() == 0) {
                    leftLinkage.setPosition(1);
                    rightLinkage.setPosition(1);
                } else if (leftLinkage.getPosition() == 1 && rightLinkage.getPosition() == 1) {
                    leftLinkage.setPosition(0);
                    rightLinkage.setPosition(0);
                }
            }

            previousA = gamepad1.a;

            telemetry.addData("D-Pad up", gamepad1.dpad_up);
            telemetry.addData("D-pad down", gamepad1.dpad_down);
            telemetry.addData("A", gamepad1.a);
            telemetry.addData("previous A", previousA);
            telemetry.update();
        }
    }
}

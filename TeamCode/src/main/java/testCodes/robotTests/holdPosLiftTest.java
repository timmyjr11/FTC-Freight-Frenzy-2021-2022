package testCodes.robotTests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
@Disabled
@TeleOp
public class holdPosLiftTest extends LinearOpMode {
    DcMotorEx rightLiftMotor;
    DcMotorEx leftLiftMotor;

    Servo leftLinkage;
    Servo rightLinkage;
    Servo rightBox;
    Servo leftBox;

    boolean previousA = false;
    boolean previousX = false;
    boolean previousY = false;

    public static double halfBox;
    public static double limitBox;

    private final FtcDashboard dashboard = FtcDashboard.getInstance();



    @Override
    public void runOpMode() throws InterruptedException {

        halfBox = 0.45;
        limitBox = 0.55;

        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());


        rightLiftMotor = hardwareMap.get(DcMotorEx.class, "rightLiftMotor");
        leftLiftMotor = hardwareMap.get(DcMotorEx.class, "leftLiftMotor");
        leftLinkage = hardwareMap.get(Servo.class, "leftLinkage");
        rightLinkage = hardwareMap.get(Servo.class, "rightLinkage");
        rightBox = hardwareMap.get(Servo.class, "rightBox");
        leftBox = hardwareMap.get(Servo.class, "leftBox");

        rightLiftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leftLiftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        rightLiftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBox.setDirection(Servo.Direction.REVERSE);
        leftLinkage.setDirection(Servo.Direction.REVERSE);

        leftLinkage.setPosition(0);
        rightLinkage.setPosition(0);
        rightBox.setPosition(0);
        leftBox.setPosition(0);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {

            if (gamepad2.dpad_up && !gamepad2.dpad_down){
                rightLiftMotor.setVelocity(850);
                leftLiftMotor.setVelocity(850);
            } else if (gamepad2.dpad_down && !gamepad2.dpad_up){
                rightLiftMotor.setVelocity(-750);
                leftLiftMotor.setVelocity(-750);
            } else if (!gamepad2.dpad_up && !gamepad2.dpad_down) {
                leftLiftMotor.setVelocity(1);
                rightLiftMotor.setVelocity(1);
            }

            if (gamepad2.a && !previousA){
                if (leftLinkage.getPosition() == 0 && rightLinkage.getPosition() == 0) {
                    leftLinkage.setPosition(1);
                    rightLinkage.setPosition(1);
                } else if (leftLinkage.getPosition() == 1 && rightLinkage.getPosition() == 1) {
                    leftLinkage.setPosition(0);
                    rightLinkage.setPosition(0);
                }
            }

            if(rightLiftMotor.getCurrentPosition() > 400 && leftLiftMotor.getCurrentPosition() > 400) {
                if (gamepad2.x && !previousX) {
                    if (rightBox.getPosition() == 1 && leftBox.getPosition() == 1) {
                        rightBox.setPosition(0);
                        leftBox.setPosition(0);
                    } else if (rightBox.getPosition() == 0 && leftBox.getPosition() == 0) {
                        rightBox.setPosition(1);
                        leftBox.setPosition(1);
                    } else if (rightBox.getPosition() <= limitBox && leftBox.getPosition() <= limitBox) {
                        rightBox.setPosition(1);
                        leftBox.setPosition(1);
                    }
                }

                if (gamepad2.y && !previousY) {
                    if ((rightBox.getPosition() == 1 && leftBox.getPosition() == 1) || (rightBox.getPosition() <= limitBox && leftBox.getPosition() <= limitBox)) {
                        rightBox.setPosition(0);
                        leftBox.setPosition(0);
                    } else if (rightBox.getPosition() == 0 && leftBox.getPosition() == 0) {
                        rightBox.setPosition(halfBox);
                        leftBox.setPosition(halfBox);
                    }
                }
            }

            previousA = gamepad1.a;
            previousX = gamepad2.x;
            previousY = gamepad2.y;

            telemetry.addData("Left lift", leftLiftMotor.getCurrentPosition());
            telemetry.addData("Right Lift", rightLiftMotor.getCurrentPosition());

            telemetry.addData("Left Servo", leftBox.getPosition());
            telemetry.addData("Right Servo", rightBox.getPosition());

            telemetry.update();
        }
    }
}

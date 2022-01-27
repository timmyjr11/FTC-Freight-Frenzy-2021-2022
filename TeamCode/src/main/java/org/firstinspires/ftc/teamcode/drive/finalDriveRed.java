package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Config
@TeleOp
public class finalDriveRed extends LinearOpMode {

    int rightLiftHeight;
    int leftLiftHeight;
    boolean previousA1;
    boolean previousA2 = false;
    boolean previousB = false;
    boolean previousX = false;
    boolean previousY = false;

    int boxState;

    SampleMecanumDrive d;

    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void runOpMode() throws InterruptedException {

        boxState = 1;

        d = new SampleMecanumDrive(hardwareMap);

        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        d.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        d.setPoseEstimate(PoseStorage.telePower);

        waitForStart();

        d.leftLinkage.setPosition(0);
        d.rightLinkage.setPosition(0);
        d.rightBox.setPosition(0);
        d.leftBox.setPosition(0);

        while (opModeIsActive() && !isStopRequested()) {
            driving();
            action();
        }
    }

    private void action() {
        if (gamepad2.dpad_up && !gamepad2.dpad_down) {
            rightLiftHeight = d.rightLiftMotor.getCurrentPosition();
            leftLiftHeight = d.leftLiftMotor.getCurrentPosition();
            d.rightLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            d.leftLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            d.rightLiftMotor.setVelocity(850);
            d.leftLiftMotor.setVelocity(850);
        } else if (gamepad2.dpad_down && !gamepad2.dpad_up) {
            rightLiftHeight = d.rightLiftMotor.getCurrentPosition();
            leftLiftHeight = d.leftLiftMotor.getCurrentPosition();
            d.rightLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            d.leftLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            d.rightLiftMotor.setVelocity(-750);
            d.leftLiftMotor.setVelocity(-750);
        } else if (!gamepad2.dpad_up && !gamepad2.dpad_down) {
            d.rightLiftMotor.setTargetPosition(rightLiftHeight);
            d.leftLiftMotor.setTargetPosition(leftLiftHeight);

            d.leftLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            d.rightLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            d.leftLiftMotor.setPower(0.25);
            d.rightLiftMotor.setPower(0.25);
        }

        if (gamepad2.a && !previousA2) {
            if (d.leftLinkage.getPosition() == 0 && d.rightLinkage.getPosition() == 0) {
                d.leftLinkage.setPosition(1);
                d.rightLinkage.setPosition(1);
            } else if (d.leftLinkage.getPosition() == 1 && d.rightLinkage.getPosition() == 1) {
                d.leftLinkage.setPosition(0);
                d.rightLinkage.setPosition(0);
            }
        }

        if (gamepad1.right_trigger >= 0.5) {
            d.rightServoWheel.setPower(1);
        } else if (gamepad1.right_trigger < 0.5) {
            d.rightServoWheel.setPower(0);
        }

        if (gamepad1.left_trigger >= 0.5) {
            d.leftServoWheel.setPower(1);
        } else if (gamepad1.left_trigger < 0.5) {
            d.leftServoWheel.setPower(0);
        }

        if (gamepad2.right_trigger >= 0.1) {
            d.intake.setPower(gamepad2.right_trigger);
        } else if (gamepad2.left_trigger >= 0.1) {
            d.intake.setPower(-gamepad2.left_trigger);
        }

        if (gamepad2.right_trigger < 0.1 && gamepad2.left_trigger < 0.1) {
            d.intake.setPower(0);
        }

        if(d.rightLiftMotor.getCurrentPosition() > 200 && d.leftLiftMotor.getCurrentPosition() > 200) {
            if (gamepad2.y && !previousY) {
                if (boxState == 1) {
                    d.rightBox.setPosition(0.6);
                    d.leftBox.setPosition(0.6);
                    boxState = 2;
                } else if (boxState == 2) {
                    d.rightBox.setPosition(1);
                    d.leftBox.setPosition(1);
                    boxState = 3;
                } else if (boxState == 3) {
                    d.rightBox.setPosition(0);
                    d.leftBox.setPosition(0);
                    boxState = 1;
                }
            }
        }

        if(gamepad1.a && !previousA1) {
            d.setPoseEstimate(PoseStorage.telePower);
        }

        previousA1 = gamepad1.a;
        previousA2 = gamepad2.a;
        previousB = gamepad2.b;
        previousX = gamepad2.x;
        previousY = gamepad2.y;

    }

    private void driving(){
        if (gamepad1.right_bumper){
            power05();
        } else if (gamepad1.left_bumper){
            power025();
        } else {
            power();
        }
    }

    private void power() {

        Pose2d poseEstimate = d.getPoseEstimate();

        // Read pose
        // Create a vector from the gamepad x/y inputs
        // Then, rotate that vector by the inverse of that heading
        Vector2d input = new Vector2d(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x
        ).rotated(-poseEstimate.getHeading());

        // Pass in the rotated input + right stick value for rotation
        // Rotation is not part of the rotated input thus must be passed in separately
        d.setWeightedDrivePower(
                new Pose2d(
                        input.getX(),
                        input.getY(),
                        -gamepad1.right_stick_x
                )
        );
        // Update everything. Optometry. Etc.
        d.update();
    }

    private void power05() {

        Pose2d poseEstimate = d.getPoseEstimate();

        Vector2d input = new Vector2d(
                -gamepad1.left_stick_y * 0.5,
                -gamepad1.left_stick_x * 0.5
        ).rotated(-poseEstimate.getHeading());

        d.setWeightedDrivePower(
                new Pose2d(
                        input.getX(),
                        input.getY(),
                        -gamepad1.right_stick_x * 0.5
                )
        );

        d.update();
    }

    private void power025() {

        Pose2d poseEstimate = d.getPoseEstimate();

        Vector2d input = new Vector2d(
                -gamepad1.left_stick_y * 0.25,
                -gamepad1.left_stick_x * 0.25
        ).rotated(-poseEstimate.getHeading());

        d.setWeightedDrivePower(
                new Pose2d(
                        input.getX(),
                        input.getY(),
                        -gamepad1.right_stick_x * 0.25
                )
        );

        d.update();
    }
}

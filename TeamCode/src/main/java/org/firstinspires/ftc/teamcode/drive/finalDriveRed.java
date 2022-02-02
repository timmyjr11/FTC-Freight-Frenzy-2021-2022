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

    //Variable that allows the lift to hold in place
    int rightLiftHeight;
    int leftLiftHeight;

    //Booleans that allow the an action to happen once and not cycle if pressed
    boolean previousA1 = false;
    boolean previousA2 = false;
    boolean previousRightStick = false;
    boolean previousX = false;
    boolean previousY = false;
    boolean previousStart = false;
    boolean previousBack = false;

    //Finite state machine that allows the box to work
    int boxState;

    //Creates SampleMecanumDrive to be used for Roadrunner
    SampleMecanumDrive d;

    //Creates the FtcDashboard that is sued for debugging
    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void runOpMode() throws InterruptedException {
        //Initial state of the box
        boxState = 1;

        //Hardware maps the SampleMecanumDrive
        d = new SampleMecanumDrive(hardwareMap);

        //Allows telemetry to be used on the dashboard
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        //Does not allow encoders to directly affect the driving of the robot
        d.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Set the pose estimate the robot knows what orientation is for field centric driving
        d.setPoseEstimate(PoseStorage.telePowerRed);

        waitForStart();

        //After the robot starts, set positions for the start
        d.leftLinkage.setPosition(0);
        d.rightLinkage.setPosition(0);
        d.rightBox.setPosition(0);
        d.leftBox.setPosition(0);

        //The loop that allows the code to keep driving and do actions
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
            d.rightLiftMotor.setVelocity(1000);
            d.leftLiftMotor.setVelocity(1000);
        } else if (gamepad2.dpad_down && !gamepad2.dpad_up) {
            rightLiftHeight = d.rightLiftMotor.getCurrentPosition();
            leftLiftHeight = d.leftLiftMotor.getCurrentPosition();
            d.rightLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            d.leftLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            d.rightLiftMotor.setVelocity(-900);
            d.leftLiftMotor.setVelocity(-900);
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

        if ((gamepad1.start && !previousStart) && (gamepad1.back && !previousBack)) {
            d.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            d.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
                    d.rightBox.setPosition(0.5);
                    d.leftBox.setPosition(0.5);
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

        if (gamepad2.right_stick_button && !previousRightStick) {
            d.rightLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            d.leftLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            d.rightLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            d.leftLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        if(gamepad1.a && !previousA1) {
            d.setPoseEstimate(PoseStorage.telePowerRed);
        }

        previousA1 = gamepad1.a;
        previousA2 = gamepad2.a;
        previousRightStick = gamepad2.right_stick_button;
        previousX = gamepad2.x;
        previousY = gamepad2.y;
        previousStart = gamepad1.start;
        previousBack = gamepad1.back;

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

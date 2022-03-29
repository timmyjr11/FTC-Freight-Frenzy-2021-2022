package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.ArrayList;
@Config
@TeleOp
public class finalDriveRedButtonUpgrade extends LinearOpMode {

    //Variable that allows the lift to hold in place
    int rightLiftHeight;
    int leftLiftHeight;
    int topHeight = 1100;

    int intakeIncrementer = 0;

    //Booleans that allow the an action to happen once and not cycle if pressed
    ArrayList<Boolean> booleanArray = new ArrayList<>();
    int booleanIncrementer = 0;
    boolean start1Pressed;
    boolean back1Pressed;
    boolean a2Pressed;
    boolean x2Pressed;
    boolean y2Pressed;
    boolean r3_2Pressed;
    boolean leftDpad2Pressed;

    //Finite state machine that allows the box to work
    int boxState;

    ElapsedTime outtakeTime = new ElapsedTime(0);

    ConfigurationStorage.intakeMode intakeMode = ConfigurationStorage.intakeMode.manual;
    ConfigurationStorage.runOuttake runOuttake = ConfigurationStorage.runOuttake.openToRun;
    ConfigurationStorage.liftMode liftMode = ConfigurationStorage.liftMode.manual;

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
        d.leftLiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        d.rightLiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        d.leftLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        d.rightLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

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
            colorSensor();

            if (d.colors.alpha() > 400 && intakeIncrementer == 0) {
                outtakeTime.reset();
                intakeMode = ConfigurationStorage.intakeMode.objectDetected;

                while(outtakeTime.milliseconds() < 1000) {
                    driving();
                    action();
                    d.intake.setPower(-0.95);
                }

                intakeMode = ConfigurationStorage.intakeMode.manual;

                intakeIncrementer = 1;
            }


        }
    }

    private void action() {
        if (gamepad2.dpad_up && !gamepad2.dpad_down && (d.rightLiftMotor.getCurrentPosition() < 1100 && d.leftLiftMotor.getCurrentPosition() < 1100) && liftMode == ConfigurationStorage.liftMode.manual) {
            rightLiftHeight = d.rightLiftMotor.getCurrentPosition();
            leftLiftHeight = d.leftLiftMotor.getCurrentPosition();
            d.rightLiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            d.leftLiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            d.rightLiftMotor.setPower(0.8);
            d.leftLiftMotor.setPower(0.8);
        } else if (gamepad2.dpad_down && !gamepad2.dpad_up && liftMode == ConfigurationStorage.liftMode.manual) {
            rightLiftHeight = d.rightLiftMotor.getCurrentPosition();
            leftLiftHeight = d.leftLiftMotor.getCurrentPosition();
            d.rightLiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            d.leftLiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            d.rightLiftMotor.setPower(-0.6);
            d.leftLiftMotor.setPower(-0.6);
        } else if (!gamepad2.dpad_up && !gamepad2.dpad_down && liftMode == ConfigurationStorage.liftMode.manual) {
            d.rightLiftMotor.setTargetPosition(rightLiftHeight);
            d.leftLiftMotor.setTargetPosition(leftLiftHeight);

            d.leftLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            d.rightLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            d.leftLiftMotor.setPower(0.1);
            d.rightLiftMotor.setPower(0.1);
        }



        if (a2Pressed && d.rightLiftMotor.getCurrentPosition() > 200 && d.leftLiftMotor.getCurrentPosition() > 200){
            if (d.leftLinkage.getPosition() == 0 && d.rightLinkage.getPosition() == 0) {
                d.leftLinkage.setPosition(1);
                d.rightLinkage.setPosition(1);
            } else if ((d.leftLinkage.getPosition() == 1 && d.rightLinkage.getPosition() == 1) || (d.leftLinkage.getPosition() == 0.5 && d.rightLinkage.getPosition() == 0.5)) {
                d.leftLinkage.setPosition(0);
                d.rightLinkage.setPosition(0);
            }
        } else if (a2Pressed && d.rightLiftMotor.getCurrentPosition() < 200 && d.leftLiftMotor.getCurrentPosition() < 200) {
            if (d.leftLinkage.getPosition() == 0 && d.rightLinkage.getPosition() == 0) {
                d.leftLinkage.setPosition(0.5);
                d.rightLinkage.setPosition(0.5);
            } else if (d.leftLinkage.getPosition() == 0.5 && d.rightLinkage.getPosition() == 0.5) {
                d.leftLinkage.setPosition(0);
                d.rightLinkage.setPosition(0);
            }
        }

        if ((start1Pressed) && (back1Pressed)) {
            d.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            d.setPoseEstimate(PoseStorage.telePowerRed);
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

        if (gamepad2.right_trigger >= 0.1 && intakeMode == ConfigurationStorage.intakeMode.manual) {
            d.intake.setPower(gamepad2.right_trigger * 0.8);
        } else if (gamepad2.left_trigger >= 0.1 && intakeMode == ConfigurationStorage.intakeMode.manual) {
            d.intake.setPower(-gamepad2.left_trigger * 0.95);
        }

        if (gamepad2.right_trigger < 0.1 && gamepad2.left_trigger < 0.1) {
            d.intake.setPower(0);
        }

        if(d.rightLiftMotor.getCurrentPosition() > 200 && d.leftLiftMotor.getCurrentPosition() > 200) {
            if (gamepad2.dpad_up) {
                d.rightBox.setPosition(0.4);
                d.leftBox.setPosition(0.4);
                boxState = 2;
            }

            if ((y2Pressed && d.rightLiftMotor.getCurrentPosition() > 200 && d.leftLiftMotor.getCurrentPosition() > 200) || (y2Pressed && d.leftLinkage.getPosition() == 0.5 && d.rightLinkage.getPosition() == 0.5)) {
                if (boxState == 1) {
                    d.rightBox.setPosition(0.4);
                    d.leftBox.setPosition(0.4);
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

        if (r3_2Pressed) {
            d.rightLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            d.leftLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            d.rightLiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            d.leftLiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }


        start1Pressed = ifPressed(gamepad1.start);
        back1Pressed = ifPressed(gamepad1.back);
        a2Pressed = ifPressed(gamepad2.a);
        x2Pressed = ifPressed(gamepad2.x);
        y2Pressed = ifPressed(gamepad2.y);
        r3_2Pressed = ifPressed(gamepad2.right_stick_button);
        leftDpad2Pressed = ifPressed(gamepad2.dpad_left);
        booleanIncrementer = 0;
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
        // Update everything. Optometry. Etc.
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

    private boolean ifPressed(boolean button) {
        boolean output = false;
        if (booleanArray.size() == booleanIncrementer) {
            booleanArray.add(false);
        }

        boolean buttonWas = booleanArray.get(booleanIncrementer);

        //noinspection PointlessBooleanExpression
        if (button != buttonWas && button == true) {
            output = true;
        }
        booleanArray.set(booleanIncrementer, button);

        booleanIncrementer = booleanIncrementer + 1;
        return output;
    }

    private void colorSensor() {
        if (d.colors.alpha() > 400 && intakeMode == ConfigurationStorage.intakeMode.manual && runOuttake == ConfigurationStorage.runOuttake.openToRun ) {
            outtakeTime.reset();
            intakeMode = ConfigurationStorage.intakeMode.objectDetected;

            while(outtakeTime.milliseconds() < 1000) {
                driving();
                action();
                d.intake.setPower(-0.95);
            }

            intakeMode = ConfigurationStorage.intakeMode.manual;
            runOuttake = ConfigurationStorage.runOuttake.doNotRunAgain;
        }

        if (d.colors.alpha() < 400) {
            runOuttake = ConfigurationStorage.runOuttake.openToRun;
    }
    }
}

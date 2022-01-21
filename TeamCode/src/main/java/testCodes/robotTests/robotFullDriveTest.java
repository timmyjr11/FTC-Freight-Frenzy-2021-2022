package testCodes.robotTests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
@Disabled
@TeleOp
public class robotFullDriveTest extends LinearOpMode {
    DcMotorEx frontRight;
    DcMotorEx frontLeft;
    DcMotorEx backLeft;
    DcMotorEx backRight;
    DcMotorEx rightLiftMotor;
    DcMotorEx leftLiftMotor;
    DcMotorEx intake;

    Servo leftLinkage;
    Servo rightLinkage;
    Servo rightBox;
    Servo leftBox;

    CRServo rightServoWheel;
    CRServo leftServoWheel;

    boolean previousA = false;
    boolean previousB = false;

    @Override
    public void runOpMode() throws InterruptedException {
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        rightLiftMotor = hardwareMap.get(DcMotorEx.class, "rightLiftMotor");
        leftLiftMotor = hardwareMap.get(DcMotorEx.class, "leftLiftMotor");
        intake = hardwareMap.get(DcMotorEx.class, "intake");

        leftLinkage = hardwareMap.get(Servo.class, "leftLinkage");
        rightLinkage = hardwareMap.get(Servo.class, "rightLinkage");
        rightBox = hardwareMap.get(Servo.class, "rightBox");
        leftBox = hardwareMap.get(Servo.class, "leftBox");

        rightServoWheel = hardwareMap.get(CRServo.class, "rightServoWheel");
        leftServoWheel = hardwareMap.get(CRServo.class, "leftServoWheel");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        rightLiftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftLinkage.setDirection(Servo.Direction.REVERSE);
        leftBox.setDirection(Servo.Direction.REVERSE);
        leftServoWheel.setDirection(DcMotorSimple.Direction.REVERSE);


        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightLiftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leftLiftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


        waitForStart();

        leftLinkage.setPosition(0);
        rightLinkage.setPosition(0);
        rightBox.setPosition(0);
        leftBox.setPosition(0);

        while (opModeIsActive() && !isStopRequested()) {
            Driving();
            Action();
            zeroPowerBehavior();
            teleBoi();
        }
    }

    private void Action() {
        if (gamepad2.dpad_up && !gamepad2.dpad_down) {
            rightLiftMotor.setVelocity(850);
            leftLiftMotor.setVelocity(850);
        } else if (gamepad2.dpad_down && !gamepad2.dpad_up) {
            rightLiftMotor.setVelocity(-750);
            leftLiftMotor.setVelocity(-750);
        } else if (!gamepad2.dpad_up && !gamepad2.dpad_down) {
            leftLiftMotor.setVelocity(1);
            rightLiftMotor.setVelocity(1);
        }

        if (gamepad2.a && !previousA) {
            if (leftLinkage.getPosition() == 0 && rightLinkage.getPosition() == 0) {
                leftLinkage.setPosition(1);
                rightLinkage.setPosition(1);
            } else if (leftLinkage.getPosition() == 1 && rightLinkage.getPosition() == 1) {
                leftLinkage.setPosition(0);
                rightLinkage.setPosition(0);
            }
        }

        if (gamepad1.right_trigger >= 0.5) {
            rightServoWheel.setPower(1);
        } else if (gamepad1.right_trigger < 0.5) {
            rightServoWheel.setPower(0);
        }

        if (gamepad1.left_trigger >= 0.5) {
            leftServoWheel.setPower(1);
        } else if (gamepad1.left_trigger < 0.5) {
            leftServoWheel.setPower(0);
        }

        if (gamepad2.right_trigger >= 0.5) {
            intake.setPower(1);
        } else if (gamepad2.left_trigger >= 0.5) {
            intake.setPower(-1);
        }

        if (gamepad2.right_trigger < 0.5 && gamepad2.left_trigger < 0.5) {
            intake.setPower(0);
        }

        if (gamepad2.b && !previousB) {
            if (rightBox.getPosition() == 1 && leftBox.getPosition() == 1) {
                rightBox.setPosition(0);
                leftBox.setPosition(0);
            } else if (rightBox.getPosition() == 0 && leftBox.getPosition() == 0) {
                rightBox.setPosition(1);
                leftBox.setPosition(1);
            }
        }

        previousA = gamepad2.a;
        previousB = gamepad2.b;
    }

    private void Driving() {
        if (gamepad1.right_bumper) {
            setPower05(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        } else if (gamepad1.left_bumper) {
            setPower025(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        } else {
            setPower(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        }
    }

    private void setPower(double y, double x, double rot) {
        double frontLeftPower = y + x + rot;
        double backLeftPower = y - x + rot;
        double frontRightPower = y - x - rot;
        double backLRightPower = y + x - rot;


        //Sets the power of the motors to the motors, which allows variable speed and movement in every direction
        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backLRightPower);
    }

    private void setPower05(double y, double x, double rot) {
        double frontLeftPower = y + x + rot;
        double backLeftPower = y - x + rot;
        double frontRightPower = y - x - rot;
        double backLRightPower = y + x - rot;


        //Sets the power of the motors to the motors, which allows variable speed and movement in every direction
        frontLeft.setPower(frontLeftPower * 0.5);
        frontRight.setPower(frontRightPower * 0.5);
        backLeft.setPower(backLeftPower * 0.5);
        backRight.setPower(backLRightPower * 0.5);
    }

    private void setPower025(double y, double x, double rot) {
        double frontLeftPower = y + x + rot;
        double backLeftPower = y - x + rot;
        double frontRightPower = y - x - rot;
        double backLRightPower = y + x - rot;


        //Sets the power of the motors to the motors, which allows variable speed and movement in every direction
        frontLeft.setPower(frontLeftPower * 0.25);
        frontRight.setPower(frontRightPower * 0.25);
        backLeft.setPower(backLeftPower * 0.25);
        backRight.setPower(backLRightPower * 0.25);
    }

    private void zeroPowerBehavior() {
        if (gamepad1.left_stick_button) {
            frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } else {
            frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
    }

    private void teleBoi() {
        telemetry.addData("Right lift position", rightLiftMotor.getCurrentPosition());
        telemetry.addData("Left lift position", leftLiftMotor.getCurrentPosition());
    }
}

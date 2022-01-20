package teachingCodes.hannahCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class bob extends LinearOpMode {
    DcMotorEx frontRight;
    DcMotorEx backRight;
    DcMotorEx frontLeft;
    DcMotorEx backLeft;
    DcMotorEx backShooter;
    DcMotorEx frontShooter;
    DcMotorEx convey;
    DcMotorEx intake;

    Servo rightPivot;
    Servo leftPivot;
    Servo armPivot;
    Servo Gripper;
    Servo stopper;
    Servo tapper;



    @Override
    public void runOpMode() throws InterruptedException {
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backShooter = hardwareMap.get(DcMotorEx.class, "backShooter");
        frontShooter = hardwareMap.get(DcMotorEx.class, "frontShooter");
        convey = hardwareMap.get(DcMotorEx.class, "convey");
        intake = hardwareMap.get(DcMotorEx.class, "intake");

        rightPivot = hardwareMap.get(Servo.class, "rightPivot");
        leftPivot = hardwareMap.get(Servo.class, "leftPivot");
        armPivot = hardwareMap.get(Servo.class, "armPivot");
        Gripper = hardwareMap.get(Servo.class, "Gripper");
        stopper = hardwareMap.get(Servo.class, "stopper");
        tapper = hardwareMap.get(Servo.class, "tapper");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        backShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        leftPivot.setDirection(Servo.Direction.REVERSE);
        frontShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        rightPivot.setPosition(0);
        leftPivot.setPosition(0);
        armPivot.setPosition(1);
        Gripper.setPosition(1);
        stopper.setPosition(1);
        tapper.setPosition(0);

        while (opModeIsActive() && !isStopRequested()) {
            if (gamepad2.a){
                armPivot.setPosition(0);
            }
            if (gamepad2.y){
                armPivot.setPosition(1);
            }
            if (gamepad2.left_bumper){
                    Gripper.setPosition(0);
            }
            if (gamepad2.right_bumper){
                    Gripper.setPosition(1);
            }
            if (gamepad2.left_trigger > 0.5){
                frontShooter.setPower(0.75);
                backShooter.setPower(0.75);
                stopper.setPosition(0);

            }
            if (gamepad2.right_trigger > 0.5){
                tapper.setPosition(1);

            }
            if (gamepad2.x){
                tapper.setPosition(0);
                stopper.setPosition(1);
            }
            if (gamepad2.dpad_down){
                rightPivot.setPosition(1);
                leftPivot.setPosition(1);
            }
            if (gamepad2.dpad_up){
                rightPivot.setPosition(0);
                leftPivot.setPosition(0);
            }
            if (gamepad2.b){
                intake.setPower(1);
            } else {
                intake.setPower(0);
            }

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
}


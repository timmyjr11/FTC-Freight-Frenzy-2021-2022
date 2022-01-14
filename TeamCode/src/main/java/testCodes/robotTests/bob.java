package testCodes.robotTests;

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
        armPivot.setPosition(0);
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


        }


    }
}


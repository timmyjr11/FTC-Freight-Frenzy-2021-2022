package testCodes.robotTests

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.*
import kotlin.Throws

@TeleOp
class boo : LinearOpMode() {
    var frontRight: DcMotorEx = null
    var frontLeft: DcMotorEx? = null
    var backLeft: DcMotorEx? = null
    var backRight: DcMotorEx? = null
    var rightLiftMotor: DcMotorEx? = null
    var leftLiftMotor: DcMotorEx? = null
    var intake: DcMotorEx? = null
    var leftLinkage: Servo? = null
    var rightLinkage: Servo? = null
    var rightServoWheel: CRServo? = null
    var leftServoWheel: CRServo? = null
    var previousA = false
    @Throws(InterruptedException::class)
    override fun runOpMode() {
        frontLeft = hardwareMap.get(DcMotorEx::class.java, "frontLeft")
        frontRight = hardwareMap.get(DcMotorEx::class.java, "frontRight")
        backLeft = hardwareMap.get(DcMotorEx::class.java, "backLeft")
        backRight = hardwareMap.get(DcMotorEx::class.java, "backRight")
        rightLiftMotor = hardwareMap.get(DcMotorEx::class.java, "rightLiftMotor")
        leftLiftMotor = hardwareMap.get(DcMotorEx::class.java, "leftLiftMotor")
        intake = hardwareMap.get(DcMotorEx::class.java, "intake")
        leftLinkage = hardwareMap.get(Servo::class.java, "leftLinkage")
        rightLinkage = hardwareMap.get(Servo::class.java, "rightLinkage")
        rightServoWheel = hardwareMap.get(CRServo::class.java, "rightServoWheel")
        leftServoWheel = hardwareMap.get(CRServo::class.java, "leftServoWheel")
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE)
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE)
        rightLiftMotor.setDirection(DcMotorSimple.Direction.REVERSE)
        leftLinkage.setDirection(Servo.Direction.REVERSE)
        leftServoWheel.setDirection(DcMotorSimple.Direction.REVERSE)
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
        rightLiftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER)
        leftLiftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER)
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT)
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT)
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT)
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT)
        waitForStart()
        leftLinkage.setPosition(0.0)
        rightLinkage.setPosition(0.0)
        while (opModeIsActive() && !isStopRequested) {
            Driving()
            Action()
            zeroPowerBehavior()
        }
    }

    private fun Action() {
        if (gamepad2.dpad_up && !gamepad2.dpad_down) {
            rightLiftMotor!!.velocity = 850.0
            leftLiftMotor!!.velocity = 850.0
        } else if (gamepad2.dpad_down && !gamepad2.dpad_up) {
            rightLiftMotor!!.velocity = -750.0
            leftLiftMotor!!.velocity = -750.0
        } else if (!gamepad2.dpad_up && !gamepad2.dpad_down) {
            leftLiftMotor!!.velocity = 1.0
            rightLiftMotor!!.velocity = 1.0
        }
        if (gamepad2.a && !previousA) {
            if (leftLinkage!!.position == 0.0 && rightLinkage!!.position == 0.0) {
                leftLinkage!!.position = 1.0
                rightLinkage!!.position = 1.0
            } else if (leftLinkage!!.position == 1.0 && rightLinkage!!.position == 1.0) {
                leftLinkage!!.position = 0.0
                rightLinkage!!.position = 0.0
            }
        }
        if (gamepad1.right_trigger >= 0.5) {
            rightServoWheel!!.power = 1.0
        } else if (gamepad1.right_trigger < 0.5) {
            rightServoWheel!!.power = 0.0
        }
        if (gamepad1.left_trigger >= 0.5) {
            leftServoWheel!!.power = 1.0
        } else if (gamepad1.left_trigger < 0.5) {
            leftServoWheel!!.power = 0.0
        }
        if (gamepad2.right_trigger >= 0.5) {
            intake!!.power = 1.0
        } else if (gamepad2.left_trigger >= 0.5) {
            intake!!.power = -1.0
        }
        if (gamepad2.right_trigger < 0.5 && gamepad2.left_trigger < 0.5) {
            intake!!.power = 0.0
        }
        previousA = gamepad2.a
    }

    private fun Driving() {
        if (gamepad1.right_bumper) {
            setPower05(-gamepad1.left_stick_y.toDouble(), gamepad1.left_stick_x.toDouble(), gamepad1.right_stick_x.toDouble())
        } else if (gamepad1.left_bumper) {
            setPower025(-gamepad1.left_stick_y.toDouble(), gamepad1.left_stick_x.toDouble(), gamepad1.right_stick_x.toDouble())
        } else {
            setPower(-gamepad1.left_stick_y.toDouble(), gamepad1.left_stick_x.toDouble(), gamepad1.right_stick_x.toDouble())
        }
    }

    private fun setPower(y: Double, x: Double, rot: Double) {
        val frontLeftPower = y + x + rot
        val backLeftPower = y - x + rot
        val frontRightPower = y - x - rot
        val backLRightPower = y + x - rot


        //Sets the power of the motors to the motors, which allows variable speed and movement in every direction
        frontLeft!!.power = frontLeftPower
        frontRight!!.power = frontRightPower
        backLeft!!.power = backLeftPower
        backRight!!.power = backLRightPower
    }

    private fun setPower05(y: Double, x: Double, rot: Double) {
        val frontLeftPower = y + x + rot
        val backLeftPower = y - x + rot
        val frontRightPower = y - x - rot
        val backLRightPower = y + x - rot


        //Sets the power of the motors to the motors, which allows variable speed and movement in every direction
        frontLeft!!.power = frontLeftPower * 0.5
        frontRight!!.power = frontRightPower * 0.5
        backLeft!!.power = backLeftPower * 0.5
        backRight!!.power = backLRightPower * 0.5
    }

    private fun setPower025(y: Double, x: Double, rot: Double) {
        val frontLeftPower = y + x + rot
        val backLeftPower = y - x + rot
        val frontRightPower = y - x - rot
        val backLRightPower = y + x - rot


        //Sets the power of the motors to the motors, which allows variable speed and movement in every direction
        frontLeft!!.power = frontLeftPower * 0.25
        frontRight!!.power = frontRightPower * 0.25
        backLeft!!.power = backLeftPower * 0.25
        backRight!!.power = backLRightPower * 0.25
    }

    private fun zeroPowerBehavior() {
        if (gamepad1.left_stick_button) {
            frontLeft!!.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
            frontRight!!.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
            backLeft!!.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
            backRight!!.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        } else {
            frontLeft!!.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT
            frontRight!!.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT
            backRight!!.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT
            backLeft!!.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT
        }
    }
}
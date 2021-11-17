package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImpl;
import com.qualcomm.robotcore.util.ElapsedTime;

public class PID_Tuner extends LinearOpMode {

    DcMotorEx motor;

    double integralSum;
    double Kp;
    double Ki;
    double Kd;
    double Kf;

    ElapsedTime timer = new ElapsedTime();
    private double lastError = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        motor = hardwareMap.get(DcMotorEx.class, "motor");

        waitForStart();
        while (opModeIsActive()) {
            double power = PIDControl(100, motor.getCurrentPosition());
            motor.setPower(power);
        }
    }
    public double PIDControl(double reference, double state) {
        double error = reference - state;
        integralSum += error * timer.seconds();
        double derivative = (error - lastError) / timer.seconds();
        lastError = error;


        timer.reset();

        double output = (error * Kd) + (derivative * Kd) + (integralSum * Ki) + (reference * Kf);
        return output;
    }

}

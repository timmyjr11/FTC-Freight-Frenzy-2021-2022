package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
@Autonomous
public class dumbAutoBlue extends LinearOpMode {

    int startingPosition;

    //Creates the dashboard that is used for debugging
    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    //Creates SampleMecanumDrive which allows the use of roadRunner
    SampleMecanumDrive d;


    Pose2d start;


    @Override
    public void runOpMode() throws InterruptedException {
        //Declare the hardware map using 'SampleMecanumDrive'
        d = new SampleMecanumDrive(hardwareMap);

        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        startingPosition();

        //Sets up servos for the proper positions
        d.leftLinkage.setPosition(0);
        d.rightLinkage.setPosition(0);
        d.rightBox.setPosition(0);
        d.leftBox.setPosition(0);

        TrajectorySequence goForDuck = d.trajectorySequenceBuilder(start)
                .lineToLinearHeading(new Pose2d(-59.5, 58.5, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(-60, 59, Math.toRadians(180)))
                .waitSeconds(3)
                .UNSTABLE_addTemporalMarkerOffset(-5, () -> d.rightServoWheel.setPower(1))
                .UNSTABLE_addTemporalMarkerOffset(5, () -> d.rightServoWheel.setPower(0))
                .build();

        TrajectorySequence parkLeft = d.trajectorySequenceBuilder(start)
                .lineToLinearHeading(new Pose2d(10, 45, Math.toRadians(0)))
                .lineToConstantHeading(new Vector2d(70, 45))
                .lineToConstantHeading(new Vector2d(70, 37))
                .lineToConstantHeading(new Vector2d(90, 37))
                .build();

        TrajectorySequence parkRight = d.trajectorySequenceBuilder(goForDuck.end())
                .lineToLinearHeading(new Pose2d(-58, 43))
                .build();

        waitForStart();

        if (startingPosition == -1) {
            d.followTrajectorySequence(goForDuck);
            d.followTrajectorySequence(parkRight);
        } else if (startingPosition == 1) {
            d.followTrajectorySequence(parkLeft);
        }
    }
    private void startingPosition() {
        telemetry.addLine("Welcome to Tim's auto selector!");
        telemetry.addLine("Choose side, left on D-pad for left, right on D-pad for right");
        telemetry.update();
        while (true) {
            if (gamepad1.dpad_left) {
                startingPosition = 1;
                break;

            } else if (gamepad1.dpad_right) {
                startingPosition = -1;
                break;
            }
            if (isStopRequested()) return;
        }

        if (startingPosition == 1) {
            start = PoseStorage.leftAutoBlue;
        } else {
            start = PoseStorage.rightAutoBlue;
        }
    }
}
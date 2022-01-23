package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;
@Config
@Autonomous
public class autoRed extends LinearOpMode {

    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    SampleMecanumDrive d;

    static int position;

    OpenCvWebcam cam;

    Pose2d start;


    int startingPosition;

    public static int rectLeftx = 65;
    public static int rectLefty = 385;
    public static int rectLeftWidth = 55;
    public static int rectLeftHeight = 55;

    public static int rectRightx = 570;
    public static int rectRighty = 390;
    public static int rectRightWidth = 55;
    public static int rectRightHeight = 55;

    public static int rectCenterx = 330;
    public static int rectCentery = 385;
    public static int rectCenterWidth = 55;
    public static int rectCenterHeight = 55;

    @Override
    public void runOpMode() throws InterruptedException {
        d = new SampleMecanumDrive(hardwareMap);

        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        cam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        cam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                cam.setPipeline(new duckDetector());
                cam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                cam.closeCameraDevice();
                telemetry.addData("errorCode:", errorCode);
                telemetry.update();
            }
        });

        FtcDashboard.getInstance().startCameraStream(cam, 30);


        telemetry.addLine("Which side? Press right on D-pad for right and left on D-pad for left.");
        telemetry.update();

        startingPosition();

        d.setPoseEstimate(start);



        d.leftLinkage.setPosition(0);
        d.rightLinkage.setPosition(0);
        d.rightBox.setPosition(0);
        d.leftBox.setPosition(0);

        if (startingPosition == 1) {
            telemetry.addData("Starting side: Right", startingPosition);
            if (position == 1) {
                telemetry.addLine("Duck position is on the right side");
            } else if (position == 0) {
                telemetry.addLine("Duck position is in the center");
            } else if (position == -1) {
                telemetry.addLine("Duck position is on the left side");
            }
            telemetry.update();
        } else if (startingPosition == -1) {
            telemetry.addData("starting side: Left", startingPosition);
            if (position == 1) {
                telemetry.addLine("Duck position is on the right side");
            } else if (position == 0) {
                telemetry.addLine("Duck position is in the center");
            } else if (position == -1) {
                telemetry.addLine("Duck position is on the left side");
            }
            telemetry.update();
        }

        if (isStopRequested()) return;

        Trajectory allianceLeftSideRightDuck = d.trajectoryBuilder(start)
                .lineToLinearHeading(new Pose2d(0, 0, Math.toRadians(0)))
                .addTemporalMarker(0.1, () -> {
                    d.leftLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    d.rightLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                    d.leftLiftMotor.setTargetPosition(1100);
                    d.rightLiftMotor.setTargetPosition(1100);

                    d.leftLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.rightLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    d.rightLiftMotor.setPower(0.75);
                    d.leftLiftMotor.setPower(0.75);
                })
                .build();


        waitForStart();

        cam.stopStreaming();

        d.followTrajectory(allianceLeftSideRightDuck);



    }

     class duckDetector extends OpenCvPipeline {
        //Creates the YCbCr color space as a mat
        Mat HSV = new Mat();

        //Creates output as a mat
        Mat outPut = new Mat();

        // Creates the rectangles as a mat
        Mat cropLeft = new Mat();

        Mat cropRight = new Mat();

        Mat cropCenter = new Mat();

        @Override
        public Mat processFrame(Mat input) {

            Imgproc.cvtColor(input, HSV, Imgproc.COLOR_RGB2HSV);

            input.copyTo(outPut);

            Rect rectLeft = new Rect(rectLeftx, rectLefty, rectLeftWidth, rectLeftHeight);
            Rect rectRight = new Rect(rectRightx, rectRighty, rectRightWidth, rectRightHeight);
            Rect rectCenter = new Rect(rectCenterx,rectCentery, rectCenterWidth, rectCenterHeight);

            Scalar rectangleColor = new Scalar(0,0, 255);

            Imgproc.rectangle(outPut, rectLeft, rectangleColor, 2);
            Imgproc.rectangle(outPut, rectRight, rectangleColor, 2);
            Imgproc.rectangle(outPut, rectCenter, rectangleColor, 2);

            cropLeft = HSV.submat(rectLeft);
            cropRight = HSV.submat(rectRight);
            cropCenter = HSV.submat(rectCenter);

            Core.extractChannel(cropLeft, cropLeft, 2);
            Core.extractChannel(cropRight, cropRight, 2);
            Core.extractChannel(cropCenter, cropCenter, 2);

            Scalar leftAverage = Core.mean(cropLeft);
            Scalar rightAverage = Core.mean(cropRight);
            Scalar centerAverage = Core.mean(cropCenter);

            double finalLeftAverage = leftAverage.val[0];
            double finalRightAverage = rightAverage.val[0];
            double finalCenterAverage = centerAverage.val[0];


            if(finalCenterAverage > finalRightAverage && finalCenterAverage > finalLeftAverage) {
                position = 0;
            } else if (finalLeftAverage > finalCenterAverage && finalLeftAverage > finalRightAverage) {
                position = -1;
            } else if (finalRightAverage > finalCenterAverage && finalRightAverage > finalCenterAverage) {
                position = 1;
            }

            return outPut;
            }
            public Mat getTheResults() { return outPut; }

    }


    private void startingPosition() {
        telemetry.addLine("Choose side, left on D-pad for left, right on D-pad for right");
        telemetry.update();
        while (true) {
            if (gamepad1.dpad_left) {
                startingPosition = -1;
                break;

            } else if(gamepad1.dpad_right) {
                startingPosition = 1;
                break;
            }
            if (isStopRequested()) return;
        }

        if (startingPosition == 1){
            start = PoseStorage.rightAutoRed;
        } else {
            start = PoseStorage.leftAutoRed;
        }
    }
}

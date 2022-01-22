package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

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

@Autonomous
public class autoRed extends LinearOpMode {

    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    SampleMecanumDrive d;

    OpenCvWebcam cam;

    duckDetector pipeline;

    Pose2d start;


    int startingPosition;

    public static int rectLeftx = 179;
    public static int rectLefty = 107;

    public static int rectRighty;
    public static int rectRightx;

    public static int rectCentery;
    public static int rectCenterx;

    @Override
    public void runOpMode() throws InterruptedException {
        d = new SampleMecanumDrive(hardwareMap);

        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        cam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        cam.setPipeline(pipeline);

        cam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                cam.startStreaming(640, 480, OpenCvCameraRotation.UPSIDE_DOWN);
                cam.setPipeline(pipeline);
            }

            @Override
            public void onError(int errorCode) {
                cam.closeCameraDevice();
                telemetry.addData("errorCode:", errorCode);
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
            telemetry.addData("Duck position:", pipeline.getDuckPosition());
            telemetry.update();
        } else if (startingPosition == -1) {
            telemetry.addData("starting side: Left", startingPosition);
            telemetry.addData("Duck position:", pipeline.getDuckPosition());
            telemetry.update();
        }

        Trajectory moveToAllianceHub = d.trajectoryBuilder(start)
                .

        waitForStart();



    }

    public static class duckDetector extends OpenCvPipeline {
        //Creates the YCbCr color space as a mat
        Mat HSV = new Mat();

        //Creates output as a mat
        Mat outPut = new Mat();

        // Creates the rectangles as a mat
        Mat cropLeft = new Mat();

        Mat cropRight = new Mat();

        Mat cropCenter = new Mat();

        public enum DuckPosition {
            Left,
            Center,
            Right
        }

        private volatile DuckPosition position =  DuckPosition.Center;

        @Override
        public Mat processFrame(Mat input) {

            Imgproc.cvtColor(input, HSV, Imgproc.COLOR_RGB2HSV);

            input.copyTo(outPut);

            Rect rectLeft = new Rect(rectLeftx, rectLefty, 37, 30);
            Rect rectRight = new Rect(rectRightx, rectRighty, 37, 30);
            Rect rectCenter = new Rect(rectCenterx,rectCentery, 37, 30);

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
               position = DuckPosition.Center;
            } else if (finalLeftAverage > finalCenterAverage && finalLeftAverage > finalRightAverage) {
                position = DuckPosition.Left;
            } else if (finalRightAverage > finalCenterAverage && finalRightAverage > finalCenterAverage) {
                position = DuckPosition.Right;
            }

            return outPut;
            }

            public DuckPosition getDuckPosition() {
            return position;
        }
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
        }

        if (startingPosition == 1){
            start = PoseStorage.rightAutoRed;
        } else {
            start = PoseStorage.leftAutoRed;
        }
    }
}

package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
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

    static double duck;

    duckDetector pipeline;


    int startingPosition;

    int rect1x = 179;
    int rect1y = 107;

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
                cam.startStreaming(1280, 720, OpenCvCameraRotation.UPSIDE_DOWN);
            }

            @Override
            public void onError(int errorCode) {
                cam.closeCameraDevice();

            }
        });

        FtcDashboard.getInstance().startCameraStream(cam, 30);


        telemetry.addLine("Which side? Press right on D-pad for right and left on D-pad for left.");
        telemetry.update();

        startingPosition();
        if (startingPosition == 1){
            d.setPoseEstimate(PoseStorage.rightAutoRed);
        } else if (startingPosition == -1) {
            d.setPoseEstimate(PoseStorage.leftAutoRed);
        }

        d.leftLinkage.setPosition(0);
        d.rightLinkage.setPosition(0);
        d.rightBox.setPosition(0);
        d.leftBox.setPosition(0);

        telemetry.addData("Starting side", startingPosition);

        waitForStart();



    }

    class duckDetector extends OpenCvPipeline {
        //Creates the YCbCr color space as a mat
        Mat HSV = new Mat();

        //Creates output as a mat
        Mat outPut = new Mat();

        // Creates the lower part of the rectangle as a mat
        Mat cropLeft = new Mat();

        Mat cropRight = new Mat();

        Mat cropCenter = new Mat();

        @Override
        public Mat processFrame(Mat input) {

            Imgproc.cvtColor(input, HSV, Imgproc.COLOR_RGB2HSV);

            input.copyTo(outPut);

            Rect rect1 = new Rect(rect1x, rect1y, 47, 40);

            Scalar rectangleColor = new Scalar(0,0, 255);

            Imgproc.rectangle(outPut, rect1, rectangleColor, 2);

            cropLeft = HSV.submat(rect1);

            Core.extractChannel(cropLeft, cropLeft, 2);

            Scalar lowerAverageOrange = Core.mean(cropLeft);

            double finalAverage = lowerAverageOrange.val[0];

            if (finalAverage > 109) {
                duck = 0;
            } else if (finalAverage > 97) {
                duck = 1;
            } else {
                duck = 2;
            }
            telemetry.addData("Average", finalAverage);
            telemetry.addLine("There are" + duck + "ducks.");
            telemetry.update();

            return outPut;
            }
;        }


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
    }
}

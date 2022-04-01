package testCodes.robotTests;

import android.os.strictmode.ImplicitDirectBootViolation;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera2;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.List;

@Autonomous
public class followDaDuckUsingOpenCVRoadRunner extends LinearOpMode {

    public static double centerOfCam = 120;

    //SampleMecanumDrive d;

    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    contourPipe pipeline;

    OpenCvInternalCamera2 cam;

    static double bounding;

    @Override
    public void runOpMode() throws InterruptedException {
        //d = new SampleMecanumDrive(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id",
                        hardwareMap.appContext.getPackageName());

        /*cam = OpenCvCameraFactory.getInstance()
                .createWebcam(hardwareMap.get
                        (WebcamName.class, "Webcam 1"), cameraMonitorViewId);
           */
        cam = OpenCvCameraFactory.getInstance().createInternalCamera2(OpenCvInternalCamera2.CameraDirection.BACK, cameraMonitorViewId);


        //Opens the camera and sets the openCV code to the webcam
        cam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                cam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                pipeline = new contourPipe();
                cam.setPipeline(pipeline);
                ///cam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);

            }

            //Runs if the camera fails to open
            @Override
            public void onError(int errorCode) {
                cam.closeCameraDevice();
                telemetry.addData("errorCode:", errorCode);
                telemetry.update();
            }
        });

        //Allows the dashboard to see what the camera sees
        FtcDashboard.getInstance().startCameraStream(cam, 30);

        telemetry.setMsTransmissionInterval(20);

        contourPipe pipeline = new contourPipe();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            sleep(20);
            telemetry.addData("yes", pipeline.getPosition());
            telemetry.update();

        }
    }


    static class contourPipe extends OpenCvPipeline {
        // Default is 80

        final int CB_CHAN_MASK_THRESHOLD = 70;
        final Scalar TEAL = new Scalar(3, 148, 252);
        final Scalar PURPLE = new Scalar(158, 52, 235);
        final Scalar BLUE = new Scalar(0, 0, 255);

        final int CONTOUR_LINE_THICKNESS = 2;
        final int CB_IDX = 2;


        private Size frameSize = new Size();


        Mat cbMat = new Mat();
        Mat thresholdMat = new Mat();
        Mat morphedThreshold = new Mat();
        Mat contoursOnPlainImageMat = new Mat();

        Mat erodeElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3));
        Mat dilateElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(6, 6));

        private volatile int position = 0;
        int centerX;
        int centerY;
        Size size;


        @Override
        public Mat processFrame(Mat input) {
            frameSize = input.size();
            ArrayList<MatOfPoint> contoursList = new ArrayList<>();

            // Convert the input image to YCrCb color space, then extract the Cb channel
            Imgproc.cvtColor(input, cbMat, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(cbMat, cbMat, CB_IDX);

            // Threshold the Cb channel to form a mask, then run some noise reduction
            Imgproc.threshold(cbMat, thresholdMat, CB_CHAN_MASK_THRESHOLD, 255, Imgproc.THRESH_BINARY_INV);
            morphMask(thresholdMat, morphedThreshold);

            // Ok, now actually look for the contours! We only look for external contours.
            Imgproc.findContours(morphedThreshold, contoursList, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);

            if (contoursList.size() == 0) {
                position = (int) (input.size().width / 2);
                return input;
            }

            double maxSize = 0;
            int maxSizeIndex = 0;
            for (int i = 0; i < contoursList.size(); i++) {
                if (contoursList.get(i).size().width * contoursList.get(i).size().height > maxSize) {
                    maxSize = (contoursList.get(i).size().width * contoursList.get(i).size().height);
                    maxSizeIndex = i;
                }
            }
            Rect boundingBox = Imgproc.boundingRect(new MatOfPoint(contoursList.get(maxSizeIndex).toArray()));
            centerX = (boundingBox.x + boundingBox.x + boundingBox.width) / 2;
            centerY = (boundingBox.y + boundingBox.y + boundingBox.height) / 2;
            position = boundingBox.x;
            size = boundingBox.size();

            Imgproc.drawContours(input, contoursList, maxSizeIndex, BLUE, CONTOUR_LINE_THICKNESS, 8);
            Imgproc.rectangle(input, boundingBox, PURPLE);


            return input;
        }

        public int getPosition() {
            return (int) (position - (frameSize.width / 2));
        }

        void morphMask(Mat input, Mat output) {
            /*
             * Apply some erosion and dilation for noise reduction
             */
            Imgproc.erode(input, output, erodeElement);
            Imgproc.erode(output, output, erodeElement);

            Imgproc.dilate(output, output, dilateElement);
            Imgproc.dilate(output, output, dilateElement);
        }
    }
}
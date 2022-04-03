package testCodes.robotTests.contours;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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

import java.util.ArrayList;
@Disabled
@Autonomous
public class duckOpenCvTest extends LinearOpMode {


    //SampleMecanumDrive d;
    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    contourPipe pipeline;

    static double bounding;

    OpenCvInternalCamera2 cam;

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
                pipeline = new contourPipe();
                cam.setPipeline(pipeline);
                cam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

                //cam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            //Runs if the camera fails to open
            @Override
            public void onError(int errorCode) {
                cam.closeCameraDevice();
                telemetry.addData("errorCode: ", errorCode);
                telemetry.update();
            }
        });

        //Allows the dashboard to see what the camera sees
        FtcDashboard.getInstance().startCameraStream(cam, 30);

        telemetry.setMsTransmissionInterval(20);


        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            sleep(20);
            ArrayList<contourPipe.analyzedDuck> ducks = pipeline.getDuckCords();

            if (ducks.isEmpty()) {
                telemetry.addLine("so sad, no ducks :(");
            } else {
                for (contourPipe.analyzedDuck duck : ducks) {
                    telemetry.addLine(String.format("Size: =%f", bounding));
                }
            }


            /* if (ducks.isEmpty()) {
                telemetry.addLine("so sad, no ducks :(");
            } else {
                for (contourPipe.analyzedDuck duck : ducks) {
                    telemetry.addData("single Duck", ducks.get(0));

                    telemetry.addLine(String.format("Duck: X=%f, Y=%f", duck.cordX, duck.cordY));

                }
            }
            */
            telemetry.update();
        }
    }

    static class contourPipe extends OpenCvPipeline {
        // Default is 80

        static final int CB_CHAN_MASK_THRESHOLD = 70;
        static final Scalar TEAL = new Scalar(3, 148, 252);
        static final Scalar PURPLE = new Scalar(158, 52, 235);
        static final Scalar BLUE = new Scalar(0, 0, 255);

        static final int CONTOUR_LINE_THICKNESS = 2;
        static final int CB_IDX = 2;

        static class analyzedDuck {
            double cordX;
            double cordY;
        }

        ArrayList<analyzedDuck> internalDuckList = new ArrayList<>();
        volatile ArrayList<analyzedDuck> clientDuckList = new ArrayList<>();

        Mat cbMat = new Mat();
        Mat thresholdMat = new Mat();
        Mat morphedThreshold = new Mat();
        Mat contoursOnPlainImageMat = new Mat();

        Mat erodeElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3));
        Mat dilateElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(6, 6));

        @Override
        public Mat processFrame(Mat input) {

            internalDuckList.clear();

            ArrayList<MatOfPoint> contours = findContours(input);

            for (int i = 0; i < contours.size(); i++) {
                Rect boundingRect = Imgproc.boundingRect(new MatOfPoint(contours.get(i).toArray()));
                analyzeContour(contours.get(i), input);
                bounding = boundingRect.size().area();
            }
            clientDuckList = new ArrayList<>(internalDuckList);

            return input;
        }

        public ArrayList<analyzedDuck> getDuckCords() { return clientDuckList; }

        ArrayList<MatOfPoint> findContours(Mat input) {
            // A list we'll be using to store the contours we find
            ArrayList<MatOfPoint> contoursList = new ArrayList<>();

            // Convert the input image to YCrCb color space, then extract the Cb channel
            Imgproc.cvtColor(input, cbMat, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(cbMat, cbMat, CB_IDX);

            // Threshold the Cb channel to form a mask, then run some noise reduction
            Imgproc.threshold(cbMat, thresholdMat, CB_CHAN_MASK_THRESHOLD, 255, Imgproc.THRESH_BINARY_INV);
            morphMask(thresholdMat, morphedThreshold);

            // Ok, now actually look for the contours! We only look for external contours.
            Imgproc.findContours(morphedThreshold, contoursList, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);

            // We do draw the contours we find, but not to the main input buffer.
            input.copyTo(contoursOnPlainImageMat);
            Imgproc.drawContours(contoursOnPlainImageMat, contoursList, -1, BLUE, CONTOUR_LINE_THICKNESS, 8);

            return contoursList;
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

        void analyzeContour(MatOfPoint contour, Mat input){
            // Transform the contour to a different format
            //Point[] points = contour.toArray();
            MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());

            // Do a rect fit to the contour, and draw it on the screen
            RotatedRect rotatedRectFitToContour = Imgproc.minAreaRect(contour2f);
            drawRotatedRect(rotatedRectFitToContour, input);

            Point center = rotatedRectFitToContour.center;
            double centerX = rotatedRectFitToContour.center.x;
            double centerY = rotatedRectFitToContour.center.y;

            drawTagTextX(rotatedRectFitToContour, "X: " + Double.toString(Math.round(center.x)), input);
            drawTagTextY(rotatedRectFitToContour, "Y: " + Double.toString(Math.round(center.y)), input);

            analyzedDuck analyzedDuck = new analyzedDuck();
            analyzedDuck.cordX = centerX;
            analyzedDuck.cordY = centerY;
            internalDuckList.add(analyzedDuck);
        }



        static void drawRotatedRect(RotatedRect rect, Mat drawOn){
            /*
             * Draws a rotated rect by drawing each of the 4 lines individually
             */
            Point[] points = new Point[4];
            rect.points(points);

            for(int i = 0; i < 4; ++i)
            {
                Imgproc.line(drawOn, points[i], points[(i+1)%4], PURPLE, 2);
            }
        }

        static void drawTagTextX(RotatedRect rect, String text, Mat mat) {
            Imgproc.putText(
                    mat, // The buffer we're drawing on
                    text, // The text we're drawing
                    new Point( // The anchor point for the text
                            rect.center.x-50,  // x anchor point
                            rect.center.y+25), // y anchor point
                    Imgproc.FONT_HERSHEY_PLAIN, // Font
                    1, // Font size
                    TEAL, // Font color
                    1); // Font thickness
        }

        static void drawTagTextY(RotatedRect rect, String text, Mat mat) {
            Imgproc.putText(
                    mat, // The buffer we're drawing on
                    text, // The text we're drawing
                    new Point( // The anchor point for the text
                            rect.center.x+50,  // x anchor point
                            rect.center.y+25), // y anchor point
                    Imgproc.FONT_HERSHEY_PLAIN, // Font
                    1, // Font size
                    TEAL, // Font color
                    1);
        }
    }
}
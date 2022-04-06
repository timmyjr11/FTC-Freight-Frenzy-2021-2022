package testCodes.robotTests.contours;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
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
import java.util.Collection;
import java.util.Collections;
import java.util.function.DoubleBinaryOperator;

@Autonomous
public class reversedVersionOfOpenCV extends LinearOpMode {

    public static double centerOfCam = 120;


    SampleMecanumDrive d;
    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    contourPipe pipeline;

    OpenCvWebcam cam;

    @Override
    public void runOpMode() throws InterruptedException {
        d = new SampleMecanumDrive(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id",
                        hardwareMap.appContext.getPackageName());

        cam = OpenCvCameraFactory.getInstance()
                .createWebcam(hardwareMap.get
                        (WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        //cam = OpenCvCameraFactory.getInstance().createInternalCamera2(OpenCvInternalCamera2.CameraDirection.BACK, cameraMonitorViewId);

        //Opens the camera and sets the openCV code to the webcam
        cam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                cam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                pipeline = new contourPipe();
                cam.setPipeline(pipeline);
                //cam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
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


        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            sleep(20);

            ArrayList<contourPipe.analyzedDuck> ducks = pipeline.getDuckCords();

            if (ducks.isEmpty()) {
                telemetry.addLine("so sad, no ducks :(");
            } else {
                telemetry.addData("X cords of Max Size: ", pipeline.getCordsX());
                telemetry.addData("Y cords of Max Size: ", pipeline.getCordsY());
                telemetry.addData("Max Size", pipeline.getSize());
                if (pipeline.getCordsX() > centerOfCam) {
                    //Turn until it works rather than a determined angle
                    //Get cords x - center of cam but make it negative when needed
                    d.turn(Math.toRadians(-pipeline.getCordsX() - centerOfCam) - 1e-6);
                } else if (pipeline.getCordsX() < centerOfCam) {
                    d.turn(Math.toRadians(pipeline.getCordsX() - centerOfCam) + 1e-6);
                }
            }
            telemetry.update();
        }
    }

    static class contourPipe extends OpenCvPipeline {
        static final int CB_CHAN_MASK_THRESHOLD = 80;
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

        ArrayList<Double> internalXCords = new ArrayList<>();
        volatile ArrayList<Double> clientXCords = new ArrayList<>();

        ArrayList<Double> internalYCords = new ArrayList<>();
        volatile ArrayList<Double> clientYCords = new ArrayList<>();

        ArrayList<Double> internalSize = new ArrayList<>();
        volatile ArrayList<Double> clientSize = new ArrayList<>();

        Mat cbMat = new Mat();
        Mat thresholdMat = new Mat();
        Mat morphedThreshold = new Mat();
        Mat contoursOnPlainImageMat = new Mat();

        Mat erodeElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3));
        Mat dilateElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(6, 6));

        @Override
        public Mat processFrame(Mat input) {

            internalXCords.clear();
            internalYCords.clear();
            internalDuckList.clear();
            internalSize.clear();

            for(MatOfPoint contour : findContours(input)) {
                analyzeContour(contour, input);
            }

            clientDuckList = new ArrayList<>(internalDuckList);
            clientXCords = new ArrayList<>(internalXCords);
            clientYCords = new ArrayList<>(internalYCords);
            clientSize = new ArrayList<>(internalSize);

            return input;
        }

        // Get the index from the max size then collect the x coordinate from that index and align with it

        public ArrayList<analyzedDuck> getDuckCords() { return clientDuckList; }
        public double getCordsX() { return clientXCords.get(clientSize.indexOf(Collections.max(clientSize))); }
        public double getSize() { return Collections.max(clientSize); }
        public double getCordsY() { return clientYCords.get(clientSize.indexOf(Collections.max(clientSize))); }

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
            //Create array for size
            double size = rotatedRectFitToContour.size.area();



            drawTagTextX(rotatedRectFitToContour, "X: " + Double.toString(Math.round(center.x)), input);
            drawTagTextY(rotatedRectFitToContour, "Y: " + Double.toString(Math.round(center.y)), input);
            drawTagTextSize(rotatedRectFitToContour, "Size: " + Double.toString(size), input);

            analyzedDuck analyzedDuck = new analyzedDuck();
            analyzedDuck.cordX = centerX;
            analyzedDuck.cordY = centerY;
            internalDuckList.add(analyzedDuck);
            internalXCords.add(centerX);
            internalYCords.add(centerY);
            internalDuckList.add(analyzedDuck);
            internalSize.add(size);
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
                            rect.center.x+25,  // x anchor point
                            rect.center.y+25), // y anchor point
                    Imgproc.FONT_HERSHEY_PLAIN, // Font
                    1, // Font size
                    TEAL, // Font color
                    1);
        }

        static void drawTagTextSize(RotatedRect rect, String text, Mat mat) {
            Imgproc.putText(
                    mat, // The buffer we're drawing on
                    text, // The text we're drawing
                    new Point( // The anchor point for the text
                            rect.center.x-25,  // x anchor point
                            rect.center.y-25), // y anchor point
                    Imgproc.FONT_HERSHEY_PLAIN, // Font
                    1, // Font size
                    TEAL, // Font color
                    1);
        }
    }

}
package testCodes.cameras.OpenCV;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera2;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.List;
@Config
@Autonomous
public class personalContour extends LinearOpMode {
    OpenCvInternalCamera2 phoneCam;

    contourDuckFinding pipeline;

    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        // Create camera instance
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera2(OpenCvInternalCamera2.CameraDirection.BACK, cameraMonitorViewId);

        // Open async and start streaming inside opened callback
        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

                pipeline = new contourDuckFinding();
                phoneCam.setPipeline(pipeline);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });
        FtcDashboard.getInstance().startCameraStream(phoneCam, 30);


        // Tell telemetry to update faster than the default 250ms period :)
        telemetry.setMsTransmissionInterval(20);

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addLine("HA");
            telemetry.update();
        }
    }

    static class contourDuckFinding extends OpenCvPipeline {
        private final Scalar RED = new Scalar(255, 0, 0, 100);
        private final Scalar GREEN = new Scalar(0, 255, 0, 100);
        private Mat RGB = new Mat();
        private Mat YCrCb = new Mat();
        private volatile int position = 0;

        public static Scalar lowerDuck = new Scalar(108.0, 159.0, 0.0);
        public static Scalar upperDuck = new Scalar(255.0, 255.0, 70.0);

        @Override
        public Mat processFrame(Mat input) {
            RGB = input;
            Imgproc.cvtColor(RGB, YCrCb, Imgproc.COLOR_RGB2YCrCb);

            Mat mask = new Mat();
            Core.inRange(YCrCb, lowerDuck, upperDuck, mask);

            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();

            Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

            int maxSize = 0;
            int maxSizeIndex = 0;

            if (contours.size() == 0) {
                position = (int) (input.size().width / 2);
                return input;
            }

            for (int i = 0; i < contours.size(); i++) {
                if (contours.get(i).size().width * contours.get(i).size().height > maxSize) {
                    maxSize = (int) (contours.get(i).size().width * contours.get(i).size().height);
                    maxSizeIndex = i;
                }
            }
            Rect boundingBox = Imgproc.boundingRect(new MatOfPoint(contours.get(maxSizeIndex).toArray()));
            int centerX = (boundingBox.x + boundingBox.x + boundingBox.width) / 2;
            int centerY = (boundingBox.y + boundingBox.y + boundingBox.height) / 2;
            position = boundingBox.x;

            Imgproc.drawContours(input, contours, maxSizeIndex, GREEN);
            Imgproc.rectangle(input, boundingBox, RED);
            return input;

        }
    }
}

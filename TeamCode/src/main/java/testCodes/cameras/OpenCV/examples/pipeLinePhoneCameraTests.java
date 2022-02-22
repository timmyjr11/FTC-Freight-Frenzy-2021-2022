package testCodes.cameras.OpenCV.examples;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera2;
import org.openftc.easyopencv.OpenCvPipeline;
@Disabled
public class pipeLinePhoneCameraTests extends LinearOpMode {

    OpenCvInternalCamera2 robit;

    int rectx = 60;
    int recty = 160;

    private final FtcDashboard dash = FtcDashboard.getInstance();


    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, dash.getTelemetry());

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        robit = OpenCvCameraFactory.getInstance().createInternalCamera2(OpenCvInternalCamera2.CameraDirection.BACK, cameraMonitorViewId);

        robit.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                robit.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Error code:", errorCode);
            }
        });

        FtcDashboard.getInstance().startCameraStream(robit, 30);

        if (isStopRequested()) return;

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {

        }
    }

    class duckGoQuack extends OpenCvPipeline {

        Mat inputValue = new Mat();

        Mat OutPut = new Mat();

        Mat crop = new Mat();

        @Override
        public Mat processFrame(Mat input) {
            Imgproc.cvtColor(input, inputValue, Imgproc.COLOR_RGB2HSV);

            input.copyTo(OutPut);

            Rect rect1 = new Rect(rectx, recty, 119, 89);

            Scalar rectangleColor = new Scalar(0,0,255);

            Imgproc.rectangle(OutPut, rect1, rectangleColor, 2);

            return OutPut;
        }
        public Mat getDaResults() { return OutPut; }
    }

}

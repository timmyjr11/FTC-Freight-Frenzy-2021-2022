package testCodes.cameras.OpenCV;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
@Config
@Autonomous
public class dualUsbPipeline extends LinearOpMode {
    OpenCvCamera usbBoi;
    //stage pipeline lmao

    int rectx = 60;
    int recty = 160;

    private final FtcDashboard dash = FtcDashboard.getInstance();

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, dash.getTelemetry());


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        usbBoi = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        usbBoi.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                usbBoi.setPipeline(new duckGoQuack());
                usbBoi.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                usbBoi.closeCameraDevice();
                telemetry.addLine("AA1");
                telemetry.addData("error", errorCode);

                telemetry.update();
            }
        });


        FtcDashboard.getInstance().startCameraStream(usbBoi, 30);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            telemetry.update();
            dash.getTelemetry();
        }

    }


    class duckGoQuack extends OpenCvPipeline {

        Mat inputValue = new Mat();

        Mat OutPut = new Mat();

        Mat crop = new Mat();


        @Override
        public Mat processFrame(Mat input) {
            Imgproc.cvtColor(input, inputValue, Imgproc.COLOR_RGB2GRAY);

            input.copyTo(OutPut);

            Rect rect1 = new Rect(rectx, recty, 119,89);

            Scalar rectangleColor = new Scalar(0,0,255);

            Imgproc.rectangle(OutPut, rect1, rectangleColor, 2);

           // Imgproc.

            inputValue = inputValue.submat(rect1);

            return OutPut;

        }
        public Mat getDaResults() { return OutPut; }
    }
}

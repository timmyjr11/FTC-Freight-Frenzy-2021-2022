package testCodes.cameras;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
@Disabled
@Autonomous
public class dualUsbCameras extends LinearOpMode {
    OpenCvCamera usbBoi;
    OpenCvCamera usbBoi2;

    private final FtcDashboard dash = FtcDashboard.getInstance();


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, dash.getTelemetry());


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

                int[] viewportContainerIds = OpenCvCameraFactory.getInstance()
                        .splitLayoutForMultipleViewports(cameraMonitorViewId, 2,OpenCvCameraFactory.ViewportSplitMethod.VERTICALLY);

                usbBoi = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), viewportContainerIds[0]);
                usbBoi2 = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 2"), viewportContainerIds[1]);

                usbBoi.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                    @Override
                    public void onOpened() {
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

                usbBoi2.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                    @Override
                    public void onOpened() {
                        usbBoi2.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
                    }

                    @Override
                    public void onError(int errorCode) {
                        usbBoi.closeCameraDevice();
                        telemetry.addLine("AA2");
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
}

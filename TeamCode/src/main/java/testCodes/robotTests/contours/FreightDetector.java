package testCodes.robotTests.contours;

import androidx.core.math.MathUtils;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import testCodes.robotTests.contours.FreightPipeline;
@Disabled
public class FreightDetector {

    private OpenCvWebcam webcam;
    private FreightPipeline pipeline;

    public FreightDetector(WebcamName webcam) {
        this.webcam = OpenCvCameraFactory.getInstance().createWebcam(webcam);
        this.pipeline = new FreightPipeline();

        this.webcam.setPipeline(pipeline);

        this.webcam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        startStreaming();
    }

    private void startStreaming() {
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });
    }

    public double calculateTurnPower() {
        return MathUtils.clamp((pipeline.getPosition() + 80.0) / (500.0 / 3.0), -0.6, 0.6);

    }

    public int getPosition() {
        return pipeline.getPosition();
    }

    public void setMaskVisibility(boolean showMask) {
        pipeline.setMaskVisibility(showMask);
    }

}
package testCodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class followDaDuckUsingOpenCVRoadRunner extends LinearOpMode {

    double center = 360;

    SampleMecanumDrive d;
    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void runOpMode() throws InterruptedException {
        d = new SampleMecanumDrive(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        waitForStart();


    }

   private void getStraight() {

    }
}

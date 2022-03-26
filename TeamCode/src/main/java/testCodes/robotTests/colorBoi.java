package testCodes.robotTests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
@Autonomous
public class colorBoi extends LinearOpMode {
    ColorSensor colors;

    @Override
    public void runOpMode() throws InterruptedException {
        colors = hardwareMap.get(ColorSensor.class, "colors");

        waitForStart();
    while (opModeIsActive() && !isStopRequested()) {
        if (colors.alpha() > 400) {
            telemetry.addLine("LMAO I HAS BLOCK LOL");
            telemetry.addData("light stuff idk", colors.alpha());
            telemetry.update();
            } else {
                telemetry.addLine("Depression, I has no block :(");
            telemetry.addData("light stuff idk", colors.alpha());
            telemetry.update();
            }
        }
    }
}

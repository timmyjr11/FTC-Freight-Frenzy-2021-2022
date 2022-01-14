package testCodes.robotTests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class dropOffBox extends LinearOpMode {
    Servo leftDropOffBox;
    Servo rightDropOffBox;

    boolean previousA = false;



    @Override
    public void runOpMode() throws InterruptedException {
        leftDropOffBox = hardwareMap.get(Servo.class, "leftDropOffBox");
        rightDropOffBox = hardwareMap.get(Servo.class, "rightDropOffBox");

        leftDropOffBox.setDirection(Servo.Direction.REVERSE);

        waitForStart();

        leftDropOffBox.setPosition(0);
        rightDropOffBox.setPosition(0);

        while (opModeIsActive() && !isStopRequested()) {

            if (gamepad2.a && !previousA){
                if (leftDropOffBox.getPosition() == 0 && rightDropOffBox.getPosition() == 0) {
                    leftDropOffBox.setPosition(1);
                    rightDropOffBox.setPosition(1);
                } else if (rightDropOffBox.getPosition() == 1 && rightDropOffBox.getPosition() ==1) {
                    leftDropOffBox.setPosition(0);
                    rightDropOffBox.setPosition(0);
                }
            }

            previousA = gamepad2.a;
        }

;    }
}

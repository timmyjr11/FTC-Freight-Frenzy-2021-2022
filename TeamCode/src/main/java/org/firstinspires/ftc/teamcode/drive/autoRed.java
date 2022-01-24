package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

@Config
@Autonomous
public class autoRed extends LinearOpMode {
//Creates the dashboard that is used for debugging
    private final FtcDashboard dashboard = FtcDashboard.getInstance();

//Creates SampleMecanumDrive which allows the use of roadRunner
    SampleMecanumDrive d;

//Creates the integer 'position' that is used for the ducks in openCV
    int position;

//Creates the webcam
    OpenCvWebcam cam;

//The starting Pose for roadRunner
    Pose2d start;

//Creates integers that will be used for the auto configuration selector.
    int startingPosition;
    int parkingPosition;
    int warehousePosition;

//Creates the left rectangle for openCV
    public static int rectLeftx = 65;
    public static int rectLefty = 385;
    public static int rectLeftWidth = 55;
    public static int rectLeftHeight = 55;

//Creates the right rectangle for openCv
    public static int rectRightx = 570;
    public static int rectRighty = 390;
    public static int rectRightWidth = 55;
    public static int rectRightHeight = 55;

//Creates the center rectangle for openCV
    public static int rectCenterx = 330;
    public static int rectCentery = 385;
    public static int rectCenterWidth = 55;
    public static int rectCenterHeight = 55;

    @Override
    public void runOpMode() throws InterruptedException {
//Declare the hardware map using 'SampleMecanumDrive'
        d = new SampleMecanumDrive(hardwareMap);

//Allows the dashboard to record telemetry
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

//Hardware maps the webcam and create a way to view what the camera sees
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        cam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

//Opens the camera and sets the openCV code to the webcam
        cam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                cam.setPipeline(new duckDetector());
                cam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
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


//Lets roadRunner understand where the robot is on the field
        d.setPoseEstimate(start);


//Sets up servos for the proper positions
        d.leftLinkage.setPosition(0);
        d.rightLinkage.setPosition(0);
        d.rightBox.setPosition(0);
        d.leftBox.setPosition(0);

//Creating the auto configuration
        startingPosition();
        sleep(500);
        parkingPosition();
        sleep(500);
        if (parkingPosition == 1) {
            wareHousePosition();
        }

//After configuration is complete, the auto configuration is then read back to drivers to ensure the correct configuration
        telemetry.addLine("Current configuration:");
        if (startingPosition == -1) {
            telemetry.addLine("Left side");
        } else if(startingPosition == 1) {
            telemetry.addLine("Right side");
        }

        if(parkingPosition == -1) {
            telemetry.addLine("Park in storage unit");
        } else if (parkingPosition == 1) {
            if (warehousePosition == 0) {
                telemetry.addLine("Park in the warehouse on left side near the shared hub");
            } else if (warehousePosition == 1) {
                telemetry.addLine("Park in the warehouse right side near the wall");
            } else if (warehousePosition == -1) {
                telemetry.addLine("Park in the warehouse top left closest to the wall and shared hub");
            }
        }

        if (position == 0) {
            telemetry.addLine("Duck is in the center");
        } else if (position == 1) {
            telemetry.addLine("Duck is on the right side");
        } else if (position == -1) {
            telemetry.addLine("Duck is on the left side");
        }

        telemetry.addLine("Something wrong with the configuration? Just restart from the beginning!");
        telemetry.addLine("Thank you for using Tim's auto selector!");
        telemetry.update();

        if (isStopRequested()) return;

//TODO: Come back to comment what the Trajectories do

        Trajectory leftSideToCarousel = d.trajectoryBuilder(start)
                .lineToConstantHeading(new Vector2d(-58, -58))
                .build();

        Trajectory leftSideToHub = d.trajectoryBuilder(leftSideToCarousel.end())
                .lineToConstantHeading(new Vector2d (-57, -20))
                .splineToSplineHeading(new Pose2d(-35, -35, Math.toRadians(180)), Math.toRadians(0))
                .build();

        Trajectory leftGoBackToDuckpt1 = d.trajectoryBuilder(leftSideToHub.end())
                .lineToLinearHeading(new Pose2d(-61, -24, Math.toRadians(270)))
                .build();

        Trajectory leftGoBackToDuckpt2 = d.trajectoryBuilder(leftGoBackToDuckpt1.end())
                .lineToConstantHeading(new Vector2d(-58, -58))
                .build();

        Trajectory leftPlaceTheDuckpt1 = d.trajectoryBuilder(leftGoBackToDuckpt2.end())
                .lineToConstantHeading(new Vector2d(-58, -20))
                .build();
        
        Trajectory leftPlaceTheDuckpt2 = d.trajectoryBuilder(leftPlaceTheDuckpt1.end())
                .lineToSplineHeading(new Pose2d(-35, -24, Math.toRadians(180)))
                .build();

        Trajectory leftParkSetup = d.trajectoryBuilder(leftPlaceTheDuckpt2.end())
                .lineToConstantHeading(new Vector2d (-58, -24))
                .build();

        Trajectory leftParkInsideStorageUnit = d.trajectoryBuilder(leftParkSetup.end())
                .lineToConstantHeading(new Vector2d(-58, -37))
                .build();

        Trajectory leftParkInsideWarehousept1 = d.trajectoryBuilder(leftParkSetup.end())
                .lineToConstantHeading(new Vector2d(-58, -58))
                .build();

        Trajectory leftParkInsideWarehousept2 = d.trajectoryBuilder(leftParkInsideWarehousept1.end())
                .splineToConstantHeading(new Vector2d(-15, -58), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(10, -48), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(37, -48), Math.toRadians(0))
                .build();

        Trajectory leftParkWareHouseRightSide = d.trajectoryBuilder(leftParkInsideWarehousept2.end())
                .lineToConstantHeading(new Vector2d(37, -60))
                .build();

        Trajectory leftParkWareHouseLeftSide = d.trajectoryBuilder(leftParkInsideWarehousept2.end())
                .lineToConstantHeading(new Vector2d(40, -38))
                .build();

        Trajectory leftParkWareHouseTopLeft = d.trajectoryBuilder(leftParkWareHouseLeftSide.end())
                .lineToConstantHeading(new Vector2d(60, -38))
                .build();

        //TODO: Create left side


        waitForStart();

//When the robot has started, the camera stops streaming
        cam.stopStreaming();

        if(startingPosition == -1) {
            d.followTrajectory(leftSideToCarousel);
            d.followTrajectory(leftSideToHub);
            d.followTrajectory(leftGoBackToDuckpt1);
            d.followTrajectory(leftGoBackToDuckpt2);
            d.followTrajectory(leftPlaceTheDuckpt1);
            d.followTrajectory(leftPlaceTheDuckpt2);
            d.followTrajectory(leftParkSetup);
            if (parkingPosition == -1) {
                d.followTrajectory(leftParkInsideStorageUnit);
            } else if(parkingPosition == 1) {
                d.followTrajectory(leftParkInsideWarehousept1);
                d.followTrajectory(leftParkInsideWarehousept2);
                if(warehousePosition == 0) {
                    d.followTrajectory(leftParkWareHouseLeftSide);
                } else if(warehousePosition == 1) {
                    d.followTrajectory(leftParkWareHouseRightSide);
                } else if(warehousePosition == -1) {
                    d.followTrajectory(leftParkWareHouseLeftSide);
                    d.followTrajectory(leftParkWareHouseTopLeft);
                }
            }
        }


    }

//The openCV code that detects ducks
     class duckDetector extends OpenCvPipeline {
        //Creates the YCbCr color space as a mat
        Mat HSV = new Mat();

        //Creates output as a mat
        Mat outPut = new Mat();

        // Creates the rectangles as a mat
        Mat cropLeft = new Mat();

        Mat cropRight = new Mat();

        Mat cropCenter = new Mat();

        @Override
        public Mat processFrame(Mat input) {

//Converts the camera color space to HSV for better detection
            Imgproc.cvtColor(input, HSV, Imgproc.COLOR_RGB2HSV);

//Copies the input to the output
            input.copyTo(outPut);

//Creates the rectangles
            Rect rectLeft = new Rect(rectLeftx, rectLefty, rectLeftWidth, rectLeftHeight);
            Rect rectRight = new Rect(rectRightx, rectRighty, rectRightWidth, rectRightHeight);
            Rect rectCenter = new Rect(rectCenterx,rectCentery, rectCenterWidth, rectCenterHeight);

//Gives the rectangles a blue boarder
            Scalar rectangleColor = new Scalar(0,0, 255);

//Draws out the rectangles to scan for yellow
            Imgproc.rectangle(outPut, rectLeft, rectangleColor, 2);
            Imgproc.rectangle(outPut, rectRight, rectangleColor, 2);
            Imgproc.rectangle(outPut, rectCenter, rectangleColor, 2);

//Turns the images of what the rectangles see into a submat that will be used to find the average
            cropLeft = HSV.submat(rectLeft);
            cropRight = HSV.submat(rectRight);
            cropCenter = HSV.submat(rectCenter);

//Extracts the color from the submats, which will be used to find the average
            Core.extractChannel(cropLeft, cropLeft, 2);
            Core.extractChannel(cropRight, cropRight, 2);
            Core.extractChannel(cropCenter, cropCenter, 2);

//Averages each of the images that the rectangles see into a singular value
            Scalar leftAverage = Core.mean(cropLeft);
            Scalar rightAverage = Core.mean(cropRight);
            Scalar centerAverage = Core.mean(cropCenter);

//Turns the values given from the average into a variable
            double finalLeftAverage = leftAverage.val[0];
            double finalRightAverage = rightAverage.val[0];
            double finalCenterAverage = centerAverage.val[0];

//If a certain rectangle has a higher value than the other two rectangles then duck is in that certain rectangle
            if(finalCenterAverage > finalRightAverage && finalCenterAverage > finalLeftAverage) {
                position = 0;
            } else if (finalLeftAverage > finalCenterAverage && finalLeftAverage > finalRightAverage) {
                position = -1;
            } else if (finalRightAverage > finalCenterAverage && finalRightAverage > finalCenterAverage) {
                position = 1;
            }
            return outPut;
            }
    }
//A part of the auto selector that determines which side the robot is on for the red alliance
    private void startingPosition() {
        telemetry.addLine("Welcome to Tim's auto selector!");
        telemetry.addLine("Choose side, left on D-pad for left, right on D-pad for right");
        telemetry.update();
        while (true) {
            if (gamepad1.dpad_left) {
                startingPosition = -1;
                break;

            } else if(gamepad1.dpad_right) {
                startingPosition = 1;
                break;
            }
            if (isStopRequested()) return;
        }

        if (startingPosition == 1){
            start = PoseStorage.rightAutoRed;
        } else {
            start = PoseStorage.leftAutoRed;
        }
    }
//A part of the auto selector that determines where to park
    private void parkingPosition() {
        if (startingPosition == -1) {
            telemetry.addLine("Left side selected, where would you like to park?");
            telemetry.addLine("Press left on D-pad to park in the storage unit");
            telemetry.addLine("Press right on D-pad to park inside the warehouse");
        } else if (startingPosition == 1) {
            telemetry.addLine("Right side selected, where would you like to park?");
            telemetry.addLine("Press left on D-pad to park in the storage unit");
            telemetry.addLine("Press right on D-pad to park inside the warehouse");
        }
        telemetry.update();
        while (true) {
            if (gamepad1.dpad_left) {
                parkingPosition = -1;
                break;
            } else if (gamepad1.dpad_right) {
                parkingPosition = 1;
                break;
            }
            if (isStopRequested()) return;
        }
    }
//A part of the auto selector that determines where to park inside the warehouse if the warehouse is selected
    private void wareHousePosition() {
        telemetry.addLine("Warehouse selected, where would you like to park specifically?");
        telemetry.addLine("Press right on D-pad to park on the right side near the wall");
        telemetry.addLine("Press left on D-pad to park on the left side near the shared hub");
        telemetry.addLine("Press up on D-pad to park on in the top left closest to the wall and shared hub");
        telemetry.update();
        while (true) {
            if (gamepad1.dpad_up) {
                warehousePosition = -1;
                break;
            } else if (gamepad1.dpad_right) {
                warehousePosition = 1;
                break;
            } else if (gamepad1.dpad_left) {
                warehousePosition = 0;
                break;
            }
            if (isStopRequested()) return;
        }
    }
}

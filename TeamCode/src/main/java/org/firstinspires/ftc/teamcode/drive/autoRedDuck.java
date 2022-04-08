package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.Collections;

import testCodes.robotTests.contours.reversedVersionOfOpenCV;
@Deprecated
@Disabled
@Config
@Autonomous(group = "Main")
public class autoRedDuck extends LinearOpMode {

    //Creates the dashboard that is used for debugging
    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    //Creates SampleMecanumDrive which allows the use of roadRunner
    SampleMecanumDrive d;

    //Creates doubles to be used for the lift
    double leftBubLift;
    double rightBubLift;
    double liftSpeed;

    //Creates the webcam
    OpenCvWebcam cam;

    contourPipe contourPipe;

    // Timer used for duck
    ElapsedTime timer = new ElapsedTime();


    //The starting Pose for roadRunner
    Pose2d start;
    Pose2d duckPose;

    //Creates ConfigurationStorage that will be used for the auto configuration selector.
    ConfigurationStorage.capStonePosition position = ConfigurationStorage.capStonePosition.toBeDetermined;
    ConfigurationStorage.sideStart startingPosition = ConfigurationStorage.sideStart.toBeDetermined;
    ConfigurationStorage.parking parkingPosition = ConfigurationStorage.parking.toBeDetermined;
    ConfigurationStorage.warehouseParking warehousePosition = ConfigurationStorage.warehouseParking.toBeDetermined;
    ConfigurationStorage.goForDuck goForDuck = ConfigurationStorage.goForDuck.toBeDetermined;
    ConfigurationStorage.rotationForDuck rotationForDuck = ConfigurationStorage.rotationForDuck.freeToRotate;

    //Creates the left rectangle for openCV
    public static int rectLeftx = 8;
    public static int rectLefty = 250;
    public static int rectLeftWidth = 80;
    public static int rectLeftHeight = 150;

    //Creates the right rectangle for openCv
    public static int rectRightx = 530;
    public static int rectRighty = 255;
    public static int rectRightWidth = 80;
    public static int rectRightHeight = 155;

    //Creates the center rectangle for openCV
    public static int rectCenterx = 260;
    public static int rectCentery = 255;
    public static int rectCenterWidth = 80;
    public static int rectCenterHeight = 150;

    //Ranges used for the duck
    public static double outerLeftBound = 293;
    public static double outerRightBound = 373;
    public static double innerLeftBound = 313;
    public static double innerRightBound = 353;

    @Override
    public void runOpMode() throws InterruptedException {
        //Declare the hardware map using 'SampleMecanumDrive'
        d = new SampleMecanumDrive(hardwareMap);

        //Allows the dashboard to record telemetry
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        //Hardware maps the webcam and create a way to view what the camera sees
        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id",
                        hardwareMap.appContext.getPackageName());

        cam = OpenCvCameraFactory.getInstance()
                .createWebcam(hardwareMap.get
                        (WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        //Opens the camera and sets the openCV code to the webcam
        cam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                contourPipe = new contourPipe();
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

        d.leftLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        d.rightLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        //Sets up servos for the proper positions
        d.leftLinkage.setPosition(0);
        d.rightLinkage.setPosition(0);
        d.rightBox.setPosition(0);
        d.leftBox.setPosition(0);

        //Creating the auto configuration
        startingPosition();
        openCVPlacement();
        sleep(500);
        parkingPosition();
        sleep(500);
        if (parkingPosition == ConfigurationStorage.parking.warehouse) {
            wareHousePosition();
        }
        if (startingPosition == ConfigurationStorage.sideStart.leftSide) {
            goingForDuck();
        }

        if (position == ConfigurationStorage.capStonePosition.right) {
            liftSpeed = 0.6;
        } else if (position == ConfigurationStorage.capStonePosition.center) {
            liftSpeed = 0.6;
        } else if (position == ConfigurationStorage.capStonePosition.left) {
            liftSpeed = 0.6;
        }

        if (startingPosition == ConfigurationStorage.sideStart.leftSide) {
            if (position == ConfigurationStorage.capStonePosition.right) {
                leftBubLift = -36;
            } else if (position == ConfigurationStorage.capStonePosition.center) {
                leftBubLift = -33.5;
            } else if (position == ConfigurationStorage.capStonePosition.left) {
                leftBubLift = -36;
            } else {
                leftBubLift = -36;
            }
        }

        if (startingPosition == ConfigurationStorage.sideStart.rightSide) {
            if (position == ConfigurationStorage.capStonePosition.right) {
                rightBubLift = -45;
            } else if (position == ConfigurationStorage.capStonePosition.center) {
                rightBubLift = -43;
            } else if (position == ConfigurationStorage.capStonePosition.left) {
                rightBubLift = -43;
            } else {
                rightBubLift = -45;
            }
        }

        telemetry.addLine("Building your configuration, please wait...");
        telemetry.addLine("");
        telemetry.addLine("According to all known laws of aviation, there is no way a bee should be able to fly. Its wings are too small to" +
                "get its fat little body off the ground. The bee of course, flies anyway, because bees don't care what humans think is impossible");
        telemetry.addLine("- Abraham Lincoln");
        telemetry.update();

        if (isStopRequested()) return;

        //Lets roadRunner understand where the robot is on the field
        d.setPoseEstimate(start);

        /*On the right side, the robot moves to the shipping hub then places the duck on the correct
        level based on the configuration of the duck */
        @SuppressWarnings("SuspiciousNameCombination") TrajectorySequence rightSide = d.trajectorySequenceBuilder(start)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    if (position == ConfigurationStorage.capStonePosition.right) {
                        d.leftBox.setPosition(0.4);
                        d.rightBox.setPosition(0.4);
                        d.leftLiftMotor.setTargetPosition(1100);
                        d.rightLiftMotor.setTargetPosition(1100);
                        d.leftLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        d.rightLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        d.leftLiftMotor.setPower(0.8);
                        d.rightLiftMotor.setPower(0.8);
                    } else if (position == ConfigurationStorage.capStonePosition.center) {
                        d.leftBox.setPosition(0.4);
                        d.rightBox.setPosition(0.4);
                        d.leftLiftMotor.setTargetPosition(350);
                        d.rightLiftMotor.setTargetPosition(350);
                        d.leftLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        d.rightLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        d.leftLiftMotor.setPower(0.8);
                        d.rightLiftMotor.setPower(0.8);
                    }
                })
                .lineToConstantHeading(new Vector2d(-20, rightBubLift))
                .waitSeconds(1.5)

                .UNSTABLE_addTemporalMarkerOffset(-1.75, () -> {
                    if (position == ConfigurationStorage.capStonePosition.left) {
                        d.leftBox.setPosition(0.4);
                        d.rightBox.setPosition(0.4);
                        d.leftLinkage.setPosition(1);
                        d.rightLinkage.setPosition(1);
                    }
                })
                .UNSTABLE_addTemporalMarkerOffset(-1.75, () -> {
                    if (position == ConfigurationStorage.capStonePosition.center || position == ConfigurationStorage.capStonePosition.right) {
                        d.leftLinkage.setPosition(1);
                        d.rightLinkage.setPosition(1);
                    }
                })
                .UNSTABLE_addTemporalMarkerOffset(-1.25, () -> {
                    d.leftBox.setPosition(1);
                    d.rightBox.setPosition(1);
                })
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
                    d.leftBox.setPosition(0);
                    d.rightBox.setPosition(0);
                })
                .build();

        //If the storage unit is chosen, the robot will go to park fully within the storage unit
        TrajectorySequence rightSideStorageUnit = d.trajectorySequenceBuilder(rightSide.end())
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    d.leftLinkage.setPosition(0);
                    d.rightLinkage.setPosition(0);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    if (position == ConfigurationStorage.capStonePosition.right || position == ConfigurationStorage.capStonePosition.center) {
                        d.leftLiftMotor.setTargetPosition(0);
                        d.rightLiftMotor.setTargetPosition(0);
                        d.leftLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        d.rightLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        d.leftLiftMotor.setPower(liftSpeed);
                        d.rightLiftMotor.setPower(liftSpeed);
                    }
                })

                .UNSTABLE_addTemporalMarkerOffset(2, () -> {
                    d.leftLiftMotor.setPower(0);
                    d.rightLiftMotor.setPower(0);
                })

                .lineToLinearHeading(new Pose2d(-91, -38, Math.toRadians(0)))
                .lineToConstantHeading(new Vector2d(-91, -18))

                .build();

        //If warehouse right is chosen, the robot will go into the warehouse and shift to the right side
        TrajectorySequence rightSideWarehouseRight = d.trajectorySequenceBuilder(rightSide.end())
                .lineToLinearHeading(new Pose2d(10, -40, Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                    d.leftLinkage.setPosition(0);
                    d.rightLinkage.setPosition(0);
                })
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
                    if (position == ConfigurationStorage.capStonePosition.right || position == ConfigurationStorage.capStonePosition.center) {
                        d.leftLiftMotor.setTargetPosition(0);
                        d.rightLiftMotor.setTargetPosition(0);
                        d.leftLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        d.rightLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        d.leftLiftMotor.setPower(liftSpeed);
                        d.rightLiftMotor.setPower(liftSpeed);
                    }
                })

                .UNSTABLE_addTemporalMarkerOffset(2, () -> {
                    d.leftLiftMotor.setPower(0);
                    d.rightLiftMotor.setPower(0);
                })
                .lineToConstantHeading(new Vector2d(60, -42))
                .lineToConstantHeading(new Vector2d(60, -58))
                .build();

        //If warehouse left is chosen, the robot will go into the warehouse and shift to the left side
        TrajectorySequence rightSideWarehouseLeft = d.trajectorySequenceBuilder(rightSide.end())
                .lineToLinearHeading(new Pose2d(10, -40, Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    d.leftLinkage.setPosition(0);
                    d.rightLinkage.setPosition(0);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    if (position == ConfigurationStorage.capStonePosition.right || position == ConfigurationStorage.capStonePosition.center) {
                        d.leftLiftMotor.setTargetPosition(0);
                        d.rightLiftMotor.setTargetPosition(0);
                        d.leftLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        d.rightLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        d.leftLiftMotor.setPower(liftSpeed);
                        d.rightLiftMotor.setPower(liftSpeed);
                    }
                })

                .UNSTABLE_addTemporalMarkerOffset(2, () -> {
                    d.leftLiftMotor.setPower(0);
                    d.rightLiftMotor.setPower(0);
                })
                .lineToConstantHeading(new Vector2d(60, -42))
                .lineToConstantHeading(new Vector2d(60, -33))
                .build();

        //If the warehouse top is chosen, the robot will go into the warehouse and shift left then move up
        TrajectorySequence rightSideWarehouseTop = d.trajectorySequenceBuilder(rightSide.end())
                .lineToLinearHeading(new Pose2d(10, -40, Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    d.leftLinkage.setPosition(0);
                    d.rightLinkage.setPosition(0);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    if (position == ConfigurationStorage.capStonePosition.right || position == ConfigurationStorage.capStonePosition.center) {
                        d.leftLiftMotor.setTargetPosition(0);
                        d.rightLiftMotor.setTargetPosition(0);
                        d.leftLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        d.rightLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        d.leftLiftMotor.setPower(liftSpeed);
                        d.rightLiftMotor.setPower(liftSpeed);
                    }
                })

                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    d.leftLiftMotor.setPower(0);
                    d.rightLiftMotor.setPower(0);
                })

                .UNSTABLE_addTemporalMarkerOffset(-4.5, () -> {
                    d.leftBox.setPosition(0);
                    d.rightBox.setPosition(0);
                })

                .UNSTABLE_addTemporalMarkerOffset(2, () -> {
                    d.leftLiftMotor.setPower(0);
                    d.rightLiftMotor.setPower(0);
                })
                .lineToConstantHeading(new Vector2d(60, -42))
                .lineToConstantHeading(new Vector2d(60, -33))
                .lineToConstantHeading(new Vector2d(100, -33))
                .build();

        /*On the left side, the robot will move to the carousel and deliver the duck, then the robot
        will go through the storage unit to place the block on the level given by the barcode.
        Then the robot will then move back the way it came and set up for parking*/
        TrajectorySequence leftSide = d.trajectorySequenceBuilder(start)
                .back(5)
                .lineToLinearHeading(new Pose2d(-70, -45, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(-70, -52.25, Math.toRadians(180)))
                .waitSeconds(3)
                .UNSTABLE_addTemporalMarkerOffset(-3.5, () -> d.leftServoWheel.setPower(1))
                .UNSTABLE_addTemporalMarkerOffset(1, () -> d.leftServoWheel.setPower(0))
                //Front wheel
                .lineToConstantHeading(new Vector2d(-68, -18))
                .UNSTABLE_addTemporalMarkerOffset(-4.0, () -> {
                    if (position == ConfigurationStorage.capStonePosition.right) {
                        d.leftLiftMotor.setTargetPosition(1100);
                        d.rightLiftMotor.setTargetPosition(1100);
                        d.leftLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        d.rightLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        d.leftLiftMotor.setPower(0.8);
                        d.rightLiftMotor.setPower(0.8);
                    } else if (position == ConfigurationStorage.capStonePosition.center) {
                        d.leftLiftMotor.setTargetPosition(350);
                        d.rightLiftMotor.setTargetPosition(350);
                        d.leftLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        d.rightLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        d.leftLiftMotor.setPower(0.8);
                        d.rightLiftMotor.setPower(0.8);
                    }
                })
                .UNSTABLE_addTemporalMarkerOffset(-3.5, () -> {
                    if (position == ConfigurationStorage.capStonePosition.center || position == ConfigurationStorage.capStonePosition.right) {
                        d.leftBox.setPosition(0.4);
                        d.rightBox.setPosition(0.4);
                    }
                })
                .lineToLinearHeading(new Pose2d(leftBubLift, -18, Math.toRadians(180)))
                .waitSeconds(2)
                .UNSTABLE_addTemporalMarkerOffset(-2, () -> {
                    d.leftBox.setPosition(0.4);
                    d.rightBox.setPosition(0.4);
                    d.leftLinkage.setPosition(1);
                    d.rightLinkage.setPosition(1);
                })
                .UNSTABLE_addTemporalMarkerOffset(-1.5, () -> {
                    d.leftBox.setPosition(1);
                    d.rightBox.setPosition(1);
                })
                .lineToLinearHeading(new Pose2d(-57, -21, Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(-2.5, () -> {
                    d.leftLinkage.setPosition(0);
                    d.rightLinkage.setPosition(0);
                    d.leftBox.setPosition(0);
                    d.rightBox.setPosition(0);
                })
                .UNSTABLE_addTemporalMarkerOffset(-1.75, () -> {
                    if (position == ConfigurationStorage.capStonePosition.right) {
                        d.leftLiftMotor.setTargetPosition(0);
                        d.rightLiftMotor.setTargetPosition(0);
                        d.leftLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        d.rightLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        d.leftLiftMotor.setPower(liftSpeed);
                        d.rightLiftMotor.setPower(liftSpeed);
                    } else if (position == ConfigurationStorage.capStonePosition.center) {
                        d.leftLiftMotor.setTargetPosition(0);
                        d.rightLiftMotor.setTargetPosition(0);
                        d.leftLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        d.rightLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        d.leftLiftMotor.setPower(liftSpeed);
                        d.rightLiftMotor.setPower(liftSpeed);
                    }
                })
                .UNSTABLE_addTemporalMarkerOffset(2, () -> {
                    d.leftLiftMotor.setPower(0);
                    d.rightLiftMotor.setPower(0);
                })
                .build();



        TrajectorySequence leftSideGoForDuck = d.trajectorySequenceBuilder(leftSide.end())
                .turn(Math.toRadians(90) + 1e-6)
                .build(); //Remove later
                /*.lineToLinearHeading(new Pose2d(-68, -18, Math.toRadians(180)))
                .UNSTABLE_addTemporalMarkerOffset(-2.0, () -> {
                    if (position == ConfigurationStorage.capStonePosition.right) {
                        d.leftLiftMotor.setTargetPosition(1100);
                        d.rightLiftMotor.setTargetPosition(1100);
                        d.leftLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        d.rightLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        d.leftLiftMotor.setPower(0.8);
                        d.rightLiftMotor.setPower(0.8);
                    } else if (position == ConfigurationStorage.capStonePosition.center) {
                        d.leftLiftMotor.setTargetPosition(350);
                        d.rightLiftMotor.setTargetPosition(350);
                        d.leftLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        d.rightLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        d.leftLiftMotor.setPower(0.8);
                        d.rightLiftMotor.setPower(0.8);
                    }
                })
                .UNSTABLE_addTemporalMarkerOffset(-1.5, () -> d.intake.setPower(0))
                .UNSTABLE_addTemporalMarkerOffset(-1.5, () -> {
                    if (position == ConfigurationStorage.capStonePosition.center || position == ConfigurationStorage.capStonePosition.right) {
                        d.leftBox.setPosition(0.4);
                        d.rightBox.setPosition(0.4);
                    }
                })
                .lineToLinearHeading(new Pose2d(leftBubLift, -18, Math.toRadians(180)))
                .waitSeconds(2)
                .UNSTABLE_addTemporalMarkerOffset(-2, () -> {
                    d.leftBox.setPosition(0.4);
                    d.rightBox.setPosition(0.4);
                    d.leftLinkage.setPosition(1);
                    d.rightLinkage.setPosition(1);
                })
                .UNSTABLE_addTemporalMarkerOffset(-1.5, () -> {
                    d.leftBox.setPosition(1);
                    d.rightBox.setPosition(1);
                })
                .lineToLinearHeading(new Pose2d(-57, -21, Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(-2.5, () -> {
                    d.leftLinkage.setPosition(0);
                    d.rightLinkage.setPosition(0);
                    d.leftBox.setPosition(0);
                    d.rightBox.setPosition(0);
                })
                .UNSTABLE_addTemporalMarkerOffset(-1.75, () -> {
                    if (position == ConfigurationStorage.capStonePosition.right) {
                        d.leftLiftMotor.setTargetPosition(0);
                        d.rightLiftMotor.setTargetPosition(0);
                        d.leftLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        d.rightLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        d.leftLiftMotor.setPower(liftSpeed);
                        d.rightLiftMotor.setPower(liftSpeed);
                    } else if (position == ConfigurationStorage.capStonePosition.center) {
                        d.leftLiftMotor.setTargetPosition(0);
                        d.rightLiftMotor.setTargetPosition(0);
                        d.leftLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        d.rightLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        d.leftLiftMotor.setPower(liftSpeed);
                        d.rightLiftMotor.setPower(liftSpeed);
                    }
                })
                .UNSTABLE_addTemporalMarkerOffset(2, () -> {
                    d.leftLiftMotor.setPower(0);
                    d.rightLiftMotor.setPower(0);
                })
                .build(); */


        //If the storage unit is chosen, the robot will move back a bit to park fully in the storage unit
        TrajectorySequence leftSideParkStorageUnit = d.trajectorySequenceBuilder(leftSide.end())
                .lineToConstantHeading(new Vector2d(-69, -32))
                .build();

        //If the  warehouse left is chosen, the robot will go into the warehouse and shift to the right side
        TrajectorySequence leftSideParkWarehouseRight = d.trajectorySequenceBuilder(leftSide.end())
                .lineToConstantHeading(new Vector2d(-56, -45))
                .lineToLinearHeading(new Pose2d(10, -45, Math.toRadians(0)))
                .lineToConstantHeading(new Vector2d(100, -45))
                .lineToConstantHeading(new Vector2d(100, -58))
                .build();

        //If warehouse left is chosen, the robot will go into the warehouse and shift to the left side
        TrajectorySequence leftSideParkWarehouseLeft = d.trajectorySequenceBuilder(leftSide.end())
                .lineToConstantHeading(new Vector2d(-56, -45))
                .lineToLinearHeading(new Pose2d(10, -45, Math.toRadians(0)))
                .lineToConstantHeading(new Vector2d(100, -45))
                .lineToConstantHeading(new Vector2d(100, -40))
                .build();

        //If the warehouse top is chosen, the robot will go into the warehouse and shift left then move up
        TrajectorySequence leftSideParkWareHouseTop = d.trajectorySequenceBuilder(leftSide.end())
                .lineToConstantHeading(new Vector2d(-56, -45))
                .lineToLinearHeading(new Pose2d(10, -45, Math.toRadians(0)))
                .lineToConstantHeading(new Vector2d(100, -45))
                .lineToConstantHeading(new Vector2d(100, -37))
                .lineToConstantHeading(new Vector2d(130, -37))
                .build();


        TrajectorySequence leftSideFixStorageDuck = d.trajectorySequenceBuilder(leftSideParkStorageUnit.end())
                .strafeRight(5)
                .build();


        while (!isStarted()) {
            //After configuration is complete, the auto configuration is then read back to drivers to ensure the correct configuration
            telemetry.addLine("Current configuration:");
            if (startingPosition == ConfigurationStorage.sideStart.leftSide) {
                telemetry.addLine("Left side");
            } else if (startingPosition == ConfigurationStorage.sideStart.rightSide) {
                telemetry.addLine("Right side");
            }

            if (parkingPosition == ConfigurationStorage.parking.storageUnit) {
                telemetry.addLine("Park in storage unit");
            } else if (parkingPosition == ConfigurationStorage.parking.warehouse) {
                if (warehousePosition == ConfigurationStorage.warehouseParking.left) {
                    telemetry.addLine("Park in the warehouse on left side near the shared hub");
                } else if (warehousePosition == ConfigurationStorage.warehouseParking.right) {
                    telemetry.addLine("Park in the warehouse right side near the wall");
                } else if (warehousePosition == ConfigurationStorage.warehouseParking.top) {
                    telemetry.addLine("Park in the warehouse top left closest to the wall and shared hub");
                }
            }

            if (startingPosition == ConfigurationStorage.sideStart.rightSide) {
                telemetry.addLine("Cannot go get duck because you are on the right side");
            }
            if (startingPosition == ConfigurationStorage.sideStart.leftSide && goForDuck == ConfigurationStorage.goForDuck.goForDuck) {
                telemetry.addLine("Will go for duck");
            } else if (startingPosition == ConfigurationStorage.sideStart.leftSide && goForDuck == ConfigurationStorage.goForDuck.doNotGoForDuck) {
                telemetry.addLine("Will NOT go for duck");
            }

            if (position == ConfigurationStorage.capStonePosition.center) {
                telemetry.addLine("Duck is in the center");
            } else if (position == ConfigurationStorage.capStonePosition.right) {
                telemetry.addLine("Duck is on the right side");
            } else if (position == ConfigurationStorage.capStonePosition.left) {
                telemetry.addLine("Duck is on the left side");
            }

            telemetry.addLine("");
            telemetry.addLine("Thank you for using Tim's auto selector! Please give me some time to build your configuration :)");
            telemetry.addLine("Something wrong with the configuration? Just restart from the beginning!");
            telemetry.update();

            if (isStopRequested()) return;

            if (isStarted()) {
                break;
            }
        }

        //When the robot has started, the camera stops streaming
        cam.setPipeline(contourPipe);
        //The robot will then follow the path as given from the configuration
        if (startingPosition == ConfigurationStorage.sideStart.leftSide) {
            d.followTrajectorySequence(leftSide);
            if (goForDuck == ConfigurationStorage.goForDuck.goForDuck) {
                d.followTrajectorySequence(leftSideGoForDuck);
                alignToDuck();
                d.setMotorPowers(0, 0, 0, 0);
                if (duckPose.getHeading() < Math.toRadians(85)) {
                    goForDuck = ConfigurationStorage.goForDuck.doNotGoForDuck;
                    TrajectorySequence doNotGoForDuck = d.trajectorySequenceBuilder(duckPose)
                            .lineToLinearHeading(new Pose2d(-57, -21, Math.toRadians(0)))
                            .build();
                    d.followTrajectorySequence(doNotGoForDuck);
                } else if (duckPose.getHeading() >= Math.toRadians(85)) {
                    TrajectorySequence goGetDuck = d.trajectorySequenceBuilder(duckPose)
                            .turn(Math.toRadians(duckPose.getHeading() + 180))
                            .UNSTABLE_addTemporalMarkerOffset(0, () -> d.intake.setPower(0.75))
                            .forward(50)
                            .lineToLinearHeading(new Pose2d(-57, -21, Math.toRadians(0)))
                            .build();
                    d.followTrajectorySequence(goGetDuck);
                }
            }
            if (parkingPosition == ConfigurationStorage.parking.storageUnit) {
                d.followTrajectorySequence(leftSideParkStorageUnit);
                if (goForDuck == ConfigurationStorage.goForDuck.goForDuck) {
                    d.followTrajectorySequence(leftSideFixStorageDuck);
                }
            } else if (warehousePosition == ConfigurationStorage.warehouseParking.left) {
                if (goForDuck == ConfigurationStorage.goForDuck.doNotGoForDuck) {
                    d.followTrajectorySequence(leftSideParkWarehouseLeft);
                } else if (goForDuck == ConfigurationStorage.goForDuck.doNotGoForDuck) {
                }
            } else if (warehousePosition == ConfigurationStorage.warehouseParking.right) {
                d.followTrajectorySequence(leftSideParkWarehouseRight);
            } else if (warehousePosition == ConfigurationStorage.warehouseParking.top) {
                d.followTrajectorySequence(leftSideParkWareHouseTop);
            }
        }

        if (startingPosition == ConfigurationStorage.sideStart.rightSide) {
            d.followTrajectorySequence(rightSide);
            if (parkingPosition == ConfigurationStorage.parking.storageUnit) {
                d.followTrajectorySequence(rightSideStorageUnit);
            } else if (warehousePosition == ConfigurationStorage.warehouseParking.left) {
                d.followTrajectorySequence(rightSideWarehouseLeft);
            } else if (warehousePosition == ConfigurationStorage.warehouseParking.right) {
                d.followTrajectorySequence(rightSideWarehouseRight);
            } else if (warehousePosition == ConfigurationStorage.warehouseParking.top) {
                d.followTrajectorySequence(rightSideWarehouseTop);
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

        //Collects the image from the camera and then processes them
        @Override
        public Mat processFrame(Mat input) {

            //Converts the camera color space to HSV for better detection
            Imgproc.cvtColor(input, HSV, Imgproc.COLOR_RGBA2RGB);
            Imgproc.cvtColor(input, HSV, Imgproc.COLOR_RGB2HSV);

            //Copies the input to the output
            input.copyTo(outPut);

            //Creates the rectangles
            Rect rectLeft = new Rect(rectLeftx, rectLefty, rectLeftWidth, rectLeftHeight);
            Rect rectRight = new Rect(rectRightx, rectRighty, rectRightWidth, rectRightHeight);
            Rect rectCenter = new Rect(rectCenterx, rectCentery, rectCenterWidth, rectCenterHeight);

            //Gives the rectangles a blue boarder
            Scalar rectangleColor = new Scalar(0, 0, 255);

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
            if (finalCenterAverage > finalRightAverage && finalCenterAverage > finalLeftAverage) {
                //Center is 0
                position = ConfigurationStorage.capStonePosition.center;
            } else if (finalLeftAverage > finalCenterAverage && finalLeftAverage > finalRightAverage) {
                //Left is -1
                position = ConfigurationStorage.capStonePosition.left;
            } else if (finalRightAverage > finalCenterAverage && finalRightAverage > finalCenterAverage) {
                //Right is 1
                position = ConfigurationStorage.capStonePosition.right;
            }
            //Returns the output that can be used
            return outPut;
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

            for (MatOfPoint contour : findContours(input)) {
                analyzeContour(contour, input);
            }

            clientDuckList = new ArrayList<>(internalDuckList);
            clientXCords = new ArrayList<>(internalXCords);
            clientYCords = new ArrayList<>(internalYCords);
            clientSize = new ArrayList<>(internalSize);

            return input;
        }

        // Get the index from the max size then collect the x coordinate from that index and align with it

        public ArrayList<analyzedDuck> getDuckCords() {
            return clientDuckList;
        }

        public double getCordsX() {
            return clientXCords.get(clientSize.indexOf(Collections.max(clientSize)));
        }

        public double getSize() {
            return Collections.max(clientSize);
        }

        public double getCordsY() {
            return clientYCords.get(clientSize.indexOf(Collections.max(clientSize)));
        }

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

        void analyzeContour(MatOfPoint contour, Mat input) {
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

        static void drawRotatedRect(RotatedRect rect, Mat drawOn) {
            /*
             * Draws a rotated rect by drawing each of the 4 lines individually
             */

            Point[] points = new Point[4];
            rect.points(points);

            for (int i = 0; i < 4; ++i) {
                Imgproc.line(drawOn, points[i], points[(i + 1) % 4], PURPLE, 2);
            }
        }

        static void drawTagTextX(RotatedRect rect, String text, Mat mat) {
            Imgproc.putText(
                    mat, // The buffer we're drawing on
                    text, // The text we're drawing
                    new Point( // The anchor point for the text
                            rect.center.x - 50,  // x anchor point
                            rect.center.y + 25), // y anchor point
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
                            rect.center.x + 25,  // x anchor point
                            rect.center.y + 25), // y anchor point
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
                            rect.center.x - 25,  // x anchor point
                            rect.center.y - 25), // y anchor point
                    Imgproc.FONT_HERSHEY_PLAIN, // Font
                    1, // Font size
                    TEAL, // Font color
                    1);
        }
    }

    //A part of the auto selector that determines which side the robot is on for the red alliance
    private void startingPosition() {
        telemetry.addLine("Welcome to Tim's auto selector!");
        telemetry.addLine("Choose side, left on D-pad for left, right on D-pad for right");
        telemetry.update();
        while (true) {
            if (gamepad1.dpad_left) {
                //Left side is -1
                startingPosition = ConfigurationStorage.sideStart.leftSide;
                break;

            } else if (gamepad1.dpad_right) {
                //Right side is 1
                startingPosition = ConfigurationStorage.sideStart.rightSide;
                break;
            }
            if (isStopRequested()) return;
        }

        if (startingPosition == ConfigurationStorage.sideStart.rightSide) {
            start = PoseStorage.rightAutoRed;
        } else {
            start = PoseStorage.leftAutoRed;
        }
    }

    //A part of the auto selector that determines where to park
    private void parkingPosition() {
        if (startingPosition == ConfigurationStorage.sideStart.leftSide) {
            telemetry.addLine("Left side selected, where would you like to park?");
            telemetry.addLine("Press left on D-pad to park in the storage unit");
            telemetry.addLine("Press right on D-pad to park inside the warehouse");
        } else if (startingPosition == ConfigurationStorage.sideStart.rightSide) {
            telemetry.addLine("Right side selected, where would you like to park?");
            telemetry.addLine("Press left on D-pad to park in the storage unit");
            telemetry.addLine("Press right on D-pad to park inside the warehouse");
        }
        telemetry.update();
        while (true) {
            if (gamepad1.dpad_left) {
                //Storage unit is -1
                parkingPosition = ConfigurationStorage.parking.storageUnit;
                break;
            } else if (gamepad1.dpad_right) {
                //Warehouse is 1
                parkingPosition = ConfigurationStorage.parking.warehouse;
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
                //-1 is top
                warehousePosition = ConfigurationStorage.warehouseParking.top;
                break;
            } else if (gamepad1.dpad_right) {
                //1 is right
                warehousePosition = ConfigurationStorage.warehouseParking.right;
                break;
            } else if (gamepad1.dpad_left) {
                //0 is left
                warehousePosition = ConfigurationStorage.warehouseParking.left;
                break;
            }

            if (isStopRequested()) return;
        }
    }

    private void goingForDuck() {
        telemetry.addLine("Finally, since you are on the left side, would you like to pick up the duck?");
        telemetry.addLine("Decide, Up on D-pad for yes, down on D-pad for no");
        telemetry.update();

        while (true) {
            if (gamepad1.dpad_up) {
                goForDuck = ConfigurationStorage.goForDuck.goForDuck;
                break;
            } else if (gamepad1.dpad_down) {
                goForDuck = ConfigurationStorage.goForDuck.doNotGoForDuck;
                break;
            }
            if (isStopRequested()) return;
        }
    }

    private void openCVPlacement() {
        if (startingPosition == ConfigurationStorage.sideStart.rightSide) {
            rectLeftx = 70;
            rectLefty = 260;
            rectLeftWidth = 80;
            rectLeftHeight = 150;

            //Creates the right rectangle for openCv
            rectRightx = 580;
            rectRighty = 265;
            rectRightWidth = 60;
            rectRightHeight = 150;

            //Creates the center rectangle for openCV
            rectCenterx = 342;
            rectCentery = 265;
            rectCenterWidth = 80;
            rectCenterHeight = 150;
        } else if (startingPosition == ConfigurationStorage.sideStart.leftSide) {
            //Creates the left rectangle for openCV
            rectLeftx = 8;
            rectLefty = 250;
            rectLeftWidth = 80;
            rectLeftHeight = 150;

            //Creates the right rectangle for openCv
            rectRightx = 530;
            rectRighty = 255;
            rectRightWidth = 80;
            rectRightHeight = 155;

            //Creates the center rectangle for openCV
            rectCenterx = 260;
            rectCentery = 255;
            rectCenterWidth = 80;
            rectCenterHeight = 150;
        }
    }

    private void alignToDuck() {
        ArrayList<contourPipe.analyzedDuck> ducks = contourPipe.getDuckCords();
        timer.reset();
        while (true) {
            if (ducks.isEmpty()) {
                telemetry.addLine("so sad, no ducks :(");
                d.setMotorPowers(-0.15, -0.15, 0.15, 0.15);
            } else {
                telemetry.addData("X cords of Max Size: ", contourPipe.getCordsX());
                telemetry.addData("Y cords of Max Size: ", contourPipe.getCordsY());
                telemetry.addData("Max Size", contourPipe.getSize());
                telemetry.update();
                if (contourPipe.getCordsX() < outerLeftBound) {
                    d.setMotorPowers(-0.15, -0.15, 0.15, 0.15);
                } else if (contourPipe.getCordsX() > outerRightBound) {
                    d.setMotorPowers(0.15, 0.15, -0.15, -0.15);
                } else if (contourPipe.getCordsX() < outerLeftBound && contourPipe.getCordsX() < innerLeftBound) {
                    d.setMotorPowers(-0.1, -0.1, 0.1, 0.1);
                } else if (contourPipe.getCordsX() > outerRightBound && contourPipe.getCordsX() < innerRightBound) {
                    d.setMotorPowers(0.1, 0.1, -0.1, -0.1);
                } else if (contourPipe.getCordsX() > innerLeftBound && contourPipe.getCordsX() < innerRightBound) {
                    d.setMotorPowers(0, 0, 0, 0);
                    d.update();
                    duckPose = d.getPoseEstimate();
                    break;
                }
            }
            d.update();
            duckPose = d.getPoseEstimate();
        }
    }
}
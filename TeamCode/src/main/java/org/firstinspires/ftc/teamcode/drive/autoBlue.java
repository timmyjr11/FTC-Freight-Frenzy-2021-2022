package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
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
public class autoBlue extends LinearOpMode {

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

    //The starting Pose for roadRunner
    Pose2d start;

    //Creates integers that will be used for the auto configuration selector.
    ConfigurationStorage.capStonePosition position = ConfigurationStorage.capStonePosition.toBeDetermined;
    ConfigurationStorage.sideStart startingPosition = ConfigurationStorage.sideStart.toBeDetermined;
    ConfigurationStorage.parking parkingPosition = ConfigurationStorage.parking.toBeDetermined;
    ConfigurationStorage.warehouseParking warehousePosition = ConfigurationStorage.warehouseParking.toBeDetermined;

    //Creates the left rectangle for openCV
    public static int rectLeftx = 40;
    public static int rectLefty = 235;
    public static int rectLeftWidth = 80;
    public static int rectLeftHeight = 150;

    //Creates the right rectangle for openCv
    public static int rectRightx = 550;
    public static int rectRighty = 240;
    public static int rectRightWidth = 80;
    public static int rectRightHeight = 150;

    //Creates the center rectangle for openCV
    public static int rectCenterx = 273;
    public static int rectCentery = 240;
    public static int rectCenterWidth = 80;
    public static int rectCenterHeight = 150;

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


        telemetry.addLine("Building your configuration, please wait...");
        telemetry.addLine("");
        telemetry.addLine("According to all known laws of aviation, there is no way a bee should be able to fly. Its wings are too small to" +
                "get its fat little body off the ground. The bee of course, flies anyway, because bees don't care what humans think is impossible");
        telemetry.addLine("- Abraham Lincoln");
        telemetry.update();

        if (position == ConfigurationStorage.capStonePosition.right) {
            liftSpeed = 0.6;
        } else if (position == ConfigurationStorage.capStonePosition.center) {
            liftSpeed = 0.2;
        } else if (position == ConfigurationStorage.capStonePosition.left) {
            liftSpeed = 0.6;
        }

        if (startingPosition == ConfigurationStorage.sideStart.rightSide) {
            if (position == ConfigurationStorage.capStonePosition.right) {
                //37.5 for left
                leftBubLift = -36;
            } else if (position == ConfigurationStorage.capStonePosition.left) {
                leftBubLift = -38;
            }
            else if (position == ConfigurationStorage.capStonePosition.center) {
                leftBubLift = -36;
            }
        }


















        if (startingPosition == ConfigurationStorage.sideStart.leftSide) {
            if (position == ConfigurationStorage.capStonePosition.right) {
                rightBubLift = 45;
            } else if (position == ConfigurationStorage.capStonePosition.center) {
                //43
                rightBubLift = 53;
            } else if (position == ConfigurationStorage.capStonePosition.left) {
                rightBubLift = 55;
            }
        }

        if (isStopRequested()) return;

        //Lets roadRunner understand where the robot is on the field
        d.setPoseEstimate(start);

        /*On the right side, the robot moves to the shipping hub then places the duck on the correct
        level based on the configuration of the duck */
        @SuppressWarnings("SuspiciousNameCombination") TrajectorySequence leftSide = d.trajectorySequenceBuilder(start)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    if (position == ConfigurationStorage.capStonePosition.right) {
                        d.leftLiftMotor.setTargetPosition(1100);
                        d.rightLiftMotor.setTargetPosition(1100);
                        d.leftLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        d.rightLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        d.leftLiftMotor.setPower(0.8);
                        d.rightLiftMotor.setPower(0.8);
                    } else if (position == ConfigurationStorage.capStonePosition.center) {
                        d.leftLiftMotor.setTargetPosition(260);
                        d.rightLiftMotor.setTargetPosition(260);
                        d.leftLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        d.rightLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        d.leftLiftMotor.setPower(0.8);
                        d.rightLiftMotor.setPower(0.8);
                    } else if (position == ConfigurationStorage.capStonePosition.left) {
                        d.leftLiftMotor.setTargetPosition(250);
                        d.rightLiftMotor.setTargetPosition(250);
                        d.leftLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        d.rightLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        d.leftLiftMotor.setPower(0.5);
                        d.rightLiftMotor.setPower(0.5);
                    }
                })
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    d.leftBox.setPosition(0.5);
                    d.rightBox.setPosition(0.5);
                })
                .lineToConstantHeading(new Vector2d(-20, rightBubLift))
                .waitSeconds(1.5)
                .UNSTABLE_addTemporalMarkerOffset(-1.5, () -> {
                    d.leftLinkage.setPosition(1);
                    d.rightLinkage.setPosition(1);
                })
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                    d.leftBox.setPosition(1);
                    d.rightBox.setPosition(1);
                })
                .build();

        //If the storage unit is chosen, the robot will go to park fully within the storage unit
        TrajectorySequence leftSideStorageUnit = d.trajectorySequenceBuilder(leftSide.end())
                .waitSeconds(0.2)
                .UNSTABLE_addTemporalMarkerOffset(-0.2, () -> {
                    if (position == ConfigurationStorage.capStonePosition.right) {
                        d.leftLinkage.setPosition(0);
                        d.rightLinkage.setPosition(0);
                        d.leftLiftMotor.setTargetPosition(100);
                        d.rightLiftMotor.setTargetPosition(100);
                        d.leftLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        d.rightLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        d.leftLiftMotor.setPower(liftSpeed);
                        d.rightLiftMotor.setPower(liftSpeed);
                    } else if (position == ConfigurationStorage.capStonePosition.left || position == ConfigurationStorage.capStonePosition.center) {
                        d.leftLinkage.setPosition(0);
                        d.rightLinkage.setPosition(0);
                        d.leftLiftMotor.setTargetPosition(250);
                        d.rightLiftMotor.setTargetPosition(250);
                        d.leftLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        d.rightLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        d.leftLiftMotor.setPower(liftSpeed);
                        d.rightLiftMotor.setPower(liftSpeed);
                    }
                })

                .UNSTABLE_addTemporalMarkerOffset(-4.5, () -> {
                    d.leftBox.setPosition(0);
                    d.rightBox.setPosition(0);
                })

                .UNSTABLE_addTemporalMarkerOffset(2, () -> {
                    d.leftLiftMotor.setPower(0);
                    d.rightLiftMotor.setPower(0);
                })
                .lineToLinearHeading(new Pose2d(-56, 45, Math.toRadians(0)))
                .lineToConstantHeading(new Vector2d(-75, 32))

                .build();

        //If warehouse right is chosen, the robot will go into the warehouse and shift to the right side
        TrajectorySequence leftSideWarehouseLeft = d.trajectorySequenceBuilder(leftSide.end())
                .waitSeconds(0.2)
                .lineToLinearHeading(new Pose2d(10, 42, Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(-5.2, () -> {
                    if (position == ConfigurationStorage.capStonePosition.right) {
                        d.leftLinkage.setPosition(0);
                        d.rightLinkage.setPosition(0);
                        d.leftLiftMotor.setTargetPosition(250);
                        d.rightLiftMotor.setTargetPosition(250);
                        d.leftLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        d.rightLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        d.leftLiftMotor.setPower(liftSpeed);
                        d.rightLiftMotor.setPower(liftSpeed);
                    } else if (position == ConfigurationStorage.capStonePosition.left || position == ConfigurationStorage.capStonePosition.center) {
                        d.leftLinkage.setPosition(0);
                        d.rightLinkage.setPosition(0);
                        d.leftLiftMotor.setTargetPosition(250);
                        d.rightLiftMotor.setTargetPosition(250);
                        d.leftLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        d.rightLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        d.leftLiftMotor.setPower(liftSpeed);
                        d.rightLiftMotor.setPower(liftSpeed);
                    }
                })

                .UNSTABLE_addTemporalMarkerOffset(-4.5, () -> {
                    d.leftBox.setPosition(0);
                    d.rightBox.setPosition(0);
                })

                .UNSTABLE_addTemporalMarkerOffset(2, () -> {
                    d.leftLiftMotor.setPower(0);
                    d.rightLiftMotor.setPower(0);
                })
                .waitSeconds(0.5)
                .lineToConstantHeading(new Vector2d(60, 42))
                .lineToConstantHeading(new Vector2d(60, 55))
                .build();

        //If warehouse left is chosen, the robot will go into the warehouse and shift to the left side
        TrajectorySequence leftSideWarehouseRight = d.trajectorySequenceBuilder(leftSide.end())
                .waitSeconds(0.2)
                .lineToLinearHeading(new Pose2d(10, 42, Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(-5.2, () -> {
                    if (position == ConfigurationStorage.capStonePosition.right) {
                        d.leftLinkage.setPosition(0);
                        d.rightLinkage.setPosition(0);
                        d.leftLiftMotor.setTargetPosition(250);
                        d.rightLiftMotor.setTargetPosition(250);
                        d.leftLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        d.rightLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        d.leftLiftMotor.setPower(liftSpeed);
                        d.rightLiftMotor.setPower(liftSpeed);
                    } else if (position == ConfigurationStorage.capStonePosition.left || position == ConfigurationStorage.capStonePosition.center) {
                        d.leftLinkage.setPosition(0);
                        d.rightLinkage.setPosition(0);
                        d.leftLiftMotor.setTargetPosition(250);
                        d.rightLiftMotor.setTargetPosition(250);
                        d.leftLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        d.rightLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        d.leftLiftMotor.setPower(liftSpeed);
                        d.rightLiftMotor.setPower(liftSpeed);
                    }
                })

                .UNSTABLE_addTemporalMarkerOffset(-4.5, () -> {
                    d.leftBox.setPosition(0);
                    d.rightBox.setPosition(0);
                })

                .UNSTABLE_addTemporalMarkerOffset(2, () -> {
                    d.leftLiftMotor.setPower(0);
                    d.rightLiftMotor.setPower(0);
                })
                .waitSeconds(0.5)
                .lineToConstantHeading(new Vector2d(60, 42))
                .lineToConstantHeading(new Vector2d(60, 33))
                .build();

        //If the warehouse top is chosen, the robot will go into the warehouse and shift left then move up
        TrajectorySequence leftSideWarehouseTop = d.trajectorySequenceBuilder(leftSide.end())
                .waitSeconds(0.2)
                .lineToLinearHeading(new Pose2d(10, 42, Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(-5.2, () -> {
                    if (position == ConfigurationStorage.capStonePosition.right) {
                        d.leftLinkage.setPosition(0);
                        d.rightLinkage.setPosition(0);
                        d.leftLiftMotor.setTargetPosition(300);
                        d.rightLiftMotor.setTargetPosition(300);
                        d.leftLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        d.rightLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        d.leftLiftMotor.setPower(liftSpeed);
                        d.rightLiftMotor.setPower(liftSpeed);
                    } else if (position == ConfigurationStorage.capStonePosition.left || position == ConfigurationStorage.capStonePosition.center) {
                        d.leftLinkage.setPosition(0);
                        d.rightLinkage.setPosition(0);;
                        d.leftLiftMotor.setTargetPosition(300);
                        d.rightLiftMotor.setTargetPosition(300);
                        d.leftLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        d.rightLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        d.leftLiftMotor.setPower(liftSpeed);
                        d.rightLiftMotor.setPower(liftSpeed);
                    }
                })

                .UNSTABLE_addTemporalMarkerOffset(-4.5, () -> {
                    d.leftBox.setPosition(0);
                    d.rightBox.setPosition(0);
                })

                .UNSTABLE_addTemporalMarkerOffset(2, () -> {
                    d.leftLiftMotor.setPower(0);
                    d.rightLiftMotor.setPower(0);
                })
                .waitSeconds(0.5)
                .lineToConstantHeading(new Vector2d(60, 42))
                .lineToConstantHeading(new Vector2d(60, 33))
                .lineToConstantHeading(new Vector2d(80, 33))
                .build();

        /*On the left side, the robot will move to the carousel and deliver the duck, then the robot
        will go through the storage unit to place the block on the level given by the barcode.
        Then the robot will then move back the way it came and set up for parking*/
        TrajectorySequence rightSide = d.trajectorySequenceBuilder(start)
                .lineToLinearHeading(new Pose2d(-51.5, 60, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(-52, 60.5, Math.toRadians(180)))
                .waitSeconds(3)
                .UNSTABLE_addTemporalMarkerOffset(-5, () -> d.rightServoWheel.setPower(1))
                .UNSTABLE_addTemporalMarkerOffset(5, () -> d.rightServoWheel.setPower(0))
                //Front wheel
                .lineToConstantHeading(new Vector2d(-59, 28))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    if (position == ConfigurationStorage.capStonePosition.right) {
                        d.leftLiftMotor.setTargetPosition(1100);
                        d.rightLiftMotor.setTargetPosition(1100);
                        d.leftLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        d.rightLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        d.leftLiftMotor.setPower(0.8);
                        d.rightLiftMotor.setPower(0.8);
                    } else if (position == ConfigurationStorage.capStonePosition.center) {
                        d.leftLiftMotor.setTargetPosition(250);
                        d.rightLiftMotor.setTargetPosition(250);
                        d.leftLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        d.rightLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        d.leftLiftMotor.setPower(0.8);
                        d.rightLiftMotor.setPower(0.8);
                    } else if (position == ConfigurationStorage.capStonePosition.left) {
                        d.leftLiftMotor.setTargetPosition(250);
                        d.rightLiftMotor.setTargetPosition(250);
                        d.leftLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        d.rightLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        d.leftLiftMotor.setPower(0.8);
                        d.rightLiftMotor.setPower(0.8);
                    }
                })
                .UNSTABLE_addTemporalMarkerOffset(-3.5, () -> {
                    d.leftBox.setPosition(0.5);
                    d.rightBox.setPosition(0.5);
                })
                .lineToLinearHeading(new Pose2d(leftBubLift, 28, Math.toRadians(180)))
                .waitSeconds(2)
                .UNSTABLE_addTemporalMarkerOffset(-2, () -> {
                    d.leftLinkage.setPosition(1);
                    d.rightLinkage.setPosition(1);
                })
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                    d.leftBox.setPosition(1);
                    d.rightBox.setPosition(1);
                })
                .lineToLinearHeading(new Pose2d(-57, 25, Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(-2, () -> {
                    if (position == ConfigurationStorage.capStonePosition.right) {
                        d.leftLinkage.setPosition(0);
                        d.rightLinkage.setPosition(0);
                        d.leftBox.setPosition(0);
                        d.rightBox.setPosition(0);
                        d.leftLiftMotor.setTargetPosition(5);
                        d.rightLiftMotor.setTargetPosition(5);
                        d.leftLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        d.rightLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        d.leftLiftMotor.setPower(liftSpeed);
                        d.rightLiftMotor.setPower(liftSpeed);
                    } else if (position == ConfigurationStorage.capStonePosition.left || position == ConfigurationStorage.capStonePosition.center) {
                        d.leftLinkage.setPosition(0);
                        d.rightLinkage.setPosition(0);
                        d.leftBox.setPosition(0);
                        d.rightBox.setPosition(0);
                        d.leftLiftMotor.setTargetPosition(300);
                        d.rightLiftMotor.setTargetPosition(300);
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

        //If the storage unit is chosen, the robot will move back a bit to park fully in the storage unit
        TrajectorySequence rightSideParkStorageUnit = d.trajectorySequenceBuilder(rightSide.end())
                .lineToConstantHeading(new Vector2d(-60, 40))
                .build();

        //If the  warehouse left is chosen, the robot will go into the warehouse and shift to the right side
        TrajectorySequence rightSideParkWarehouseLeft = d.trajectorySequenceBuilder(rightSide.end())
                .lineToConstantHeading(new Vector2d(-58, 52))
                .lineToLinearHeading(new Pose2d(10, 52, Math.toRadians(0)))
                .lineToConstantHeading(new Vector2d(85, 52))
                .lineToConstantHeading(new Vector2d(85, 63))
                .build();

        //If warehouse left is chosen, the robot will go into the warehouse and shift to the left side
        TrajectorySequence rightSideParkWarehouseRight = d.trajectorySequenceBuilder(rightSide.end())
                .lineToConstantHeading(new Vector2d(-56, 52))
                .lineToLinearHeading(new Pose2d(10, 52, Math.toRadians(0)))
                .lineToConstantHeading(new Vector2d(85, 52))
                .lineToConstantHeading(new Vector2d(85, 33))
                .build();

        //If the warehouse top is chosen, the robot will go into the warehouse and shift left then move up
        TrajectorySequence rightSideParkWareHouseTop = d.trajectorySequenceBuilder(rightSide.end())
                .lineToConstantHeading(new Vector2d(-56, 52))
                .lineToLinearHeading(new Pose2d(10, 52, Math.toRadians(0)))
                .lineToConstantHeading(new Vector2d(85, 52))
                .lineToConstantHeading(new Vector2d(84, 33))
                .lineToConstantHeading(new Vector2d(95, 32))
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
                if (warehousePosition == ConfigurationStorage.warehouseParking.right) {
                    telemetry.addLine("Park in the warehouse on right side near the shared hub");
                } else if (warehousePosition == ConfigurationStorage.warehouseParking.left) {
                    telemetry.addLine("Park in the warehouse left side near the wall");
                } else if (warehousePosition == ConfigurationStorage.warehouseParking.top) {
                    telemetry.addLine("Park in the warehouse top left closest to the wall and shared hub");
                }
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
        cam.stopStreaming();

        //The robot will then follow the path as given from the configuration
        if (startingPosition == ConfigurationStorage.sideStart.leftSide) {
            d.followTrajectorySequence(leftSide);
            if (parkingPosition == ConfigurationStorage.parking.storageUnit) {
                d.followTrajectorySequence(leftSideStorageUnit);
            } else if (warehousePosition == ConfigurationStorage.warehouseParking.left) {
                d.followTrajectorySequence(leftSideWarehouseLeft);
            } else if (warehousePosition == ConfigurationStorage.warehouseParking.right) {
                d.followTrajectorySequence(leftSideWarehouseRight);
            } else if (warehousePosition == ConfigurationStorage.warehouseParking.top) {
                d.followTrajectorySequence(leftSideWarehouseTop);
            }
        }

        if (startingPosition == ConfigurationStorage.sideStart.rightSide) {
            d.followTrajectorySequence(rightSide);
            if (parkingPosition == ConfigurationStorage.parking.storageUnit) {
                d.followTrajectorySequence(rightSideParkStorageUnit);
            } else if (warehousePosition == ConfigurationStorage.warehouseParking.left) {
                d.followTrajectorySequence(rightSideParkWarehouseLeft);
            } else if (warehousePosition == ConfigurationStorage.warehouseParking.right) {
                d.followTrajectorySequence(rightSideParkWarehouseRight);
            } else if (warehousePosition == ConfigurationStorage.warehouseParking.top) {
                d.followTrajectorySequence(rightSideParkWareHouseTop);
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

    //A part of the auto selector that determines which side the robot is on for the red alliance
    private void startingPosition() {
        telemetry.addLine("Welcome to Tim's auto selector!");
        telemetry.addLine("Choose side, left on D-pad for left, right on D-pad for right");
        telemetry.update();
        while (true) {
            if (gamepad1.dpad_left) {
                //Left is 1
                startingPosition = ConfigurationStorage.sideStart.leftSide;
                break;

            } else if (gamepad1.dpad_right) {
                //Right is -1
                startingPosition = ConfigurationStorage.sideStart.rightSide;
                break;
            }
            if (isStopRequested()) return;
        }

        if (startingPosition == ConfigurationStorage.sideStart.leftSide) {
            start = PoseStorage.leftAutoBlue;
        } else {
            start = PoseStorage.rightAutoBlue;
        }
    }

    //A part of the auto selector that determines where to park
    private void parkingPosition() {
        if (startingPosition == ConfigurationStorage.sideStart.leftSide) {
            telemetry.addLine("Left side selected, where would you like to park?");
            telemetry.addLine("Press right on D-pad to park in the storage unit");
            telemetry.addLine("Press left on D-pad to park inside the warehouse");
        } else if (startingPosition == ConfigurationStorage.sideStart.rightSide) {
            telemetry.addLine("Right side selected, where would you like to park?");
            telemetry.addLine("Press right on D-pad to park in the storage unit");
            telemetry.addLine("Press left on D-pad to park inside the warehouse");
        }
        telemetry.update();
        while (true) {
            if (gamepad1.dpad_left) {
                //1 is warehouse
                parkingPosition = ConfigurationStorage.parking.warehouse;
                break;
            } else if (gamepad1.dpad_right) {
                //-1 is storage unit
                parkingPosition = ConfigurationStorage.parking.storageUnit;
                break;
            }
            if (isStopRequested()) return;
        }
    }

    //A part of the auto selector that determines where to park inside the warehouse if the warehouse is selected
    private void wareHousePosition() {
        telemetry.addLine("Warehouse selected, where would you like to park specifically?");
        telemetry.addLine("Press Left on D-pad to park on the left side near the wall");
        telemetry.addLine("Press right on D-pad to park on the right side near the shared hub");
        telemetry.addLine("Press up on D-pad to park on in the top right closest to the wall and shared hub");
        telemetry.update();
        while (true) {
            if (gamepad1.dpad_up) {
                //1 is up
                warehousePosition = ConfigurationStorage.warehouseParking.top;
                break;
            } else if (gamepad1.dpad_right) {
                //-1 is right
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

    private void openCVPlacement() {
        if(startingPosition == ConfigurationStorage.sideStart.leftSide) {
            rectLeftx = 40;
            rectLefty = 246;
            rectLeftWidth = 80;
            rectLeftHeight = 150;

            //Creates the right rectangle for openCv
            rectRightx = 543;
            rectRighty = 255;
            rectRightWidth = 80;
            rectRightHeight = 150;

            //Creates the center rectangle for openCV
            rectCenterx = 274;
            rectCentery = 255;
            rectCenterWidth = 80;
            rectCenterHeight = 150;
        } else if (startingPosition == ConfigurationStorage.sideStart.rightSide) {
            //Creates the left rectangle for openCV
            rectLeftx = 40;
            rectLefty = 235;
            rectLeftWidth = 80;
            rectLeftHeight = 150;

            //Creates the right rectangle for openCv
            rectRightx = 550;
            rectRighty = 240;
            rectRightWidth = 80;
            rectRightHeight = 150;

            //Creates the center rectangle for openCV
            rectCenterx = 273;
            rectCentery = 240;
            rectCenterWidth = 80;
            rectCenterHeight = 150;
        }
    }
}
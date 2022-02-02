package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class PoseStorage {
    public static Pose2d telePowerRed = new Pose2d(0,0, Math.toRadians(-90));
    public static Pose2d telePowerBlue = new Pose2d(0, 0, Math.toRadians(90));


    public static Pose2d rightAutoRed = new Pose2d(6.5, -62.5, Math.toRadians(-90));
    public static Pose2d leftAutoRed = new Pose2d(-34, -62.5, Math.toRadians(-90));

    public static Pose2d rightAutoBlue = new Pose2d(-30, 62.5, Math.toRadians(90));
    public static Pose2d leftAutoBlue = new Pose2d(2.5, 62.5, Math.toRadians(90));
}
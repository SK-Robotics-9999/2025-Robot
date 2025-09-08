// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Degrees;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

/** Add your docs here. */
public class FieldNavigation {
static double botCenterToScorerOffset = Inches.of(2.0).in(Meters);
    static double botCenterToRearX = Inches.of((30/2.0)+4+0.25+0.5+1.0).in(Meters);
    static double pidApproachOffset = Inches.of(20.25+12).in(Meters);
    static double coralY = Inches.of(15/2.0).in(Meters);
    //These are right relative from the tag's pose facing out  from the reef
    static Transform2d coralLeft = new Transform2d(new Pose2d(), new Pose2d(botCenterToRearX, Inches.of(7.0).in(Meters), new Rotation2d(Degrees.of(0))));
    static Transform2d coralRight = new Transform2d(new Pose2d(), new Pose2d(botCenterToRearX, Inches.of(-4.0).in(Meters), new Rotation2d(Degrees.of(0))));
    static Transform2d coralMid = new Transform2d(new Pose2d(), new Pose2d(botCenterToRearX, 0.0, new Rotation2d(Degrees.of(0))));
    static Transform2d reefAlgae = new Transform2d(new Pose2d(), new Pose2d(botCenterToRearX, 0, new Rotation2d(Degrees.of(0))));
    static Transform2d coralSource = new Transform2d(new Pose2d(), new Pose2d(botCenterToRearX, 0, new Rotation2d(Degrees.of(180))));
    static Transform2d coralApproachOffsetLeft = new Transform2d(new Pose2d(), new Pose2d(pidApproachOffset, Inches.of(7.0).in(Meters), new Rotation2d(Degrees.of(0))));
    static Transform2d coralApproachOffsetRight = new Transform2d(new Pose2d(), new Pose2d(pidApproachOffset, Inches.of(-4).in(Meters), new Rotation2d(Degrees.of(0))));



    public static List<Pose2d> tagsReef = new ArrayList<>(){{
        add(AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape).getTagPose(6).get().toPose2d());
        add(AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape).getTagPose(7).get().toPose2d());
        add(AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape).getTagPose(8).get().toPose2d());
        add(AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape).getTagPose(9).get().toPose2d());
        add(AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape).getTagPose(10).get().toPose2d());
        add(AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape).getTagPose(11).get().toPose2d());
        
        add(AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape).getTagPose(17).get().toPose2d());
        add(AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape).getTagPose(18).get().toPose2d());
        add(AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape).getTagPose(19).get().toPose2d());
        add(AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape).getTagPose(20).get().toPose2d());
        add(AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape).getTagPose(21).get().toPose2d());
        add(AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape).getTagPose(22).get().toPose2d());
    }};

    public static List<Pose2d> tagsSource = new ArrayList<>(){{
        add(AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape).getTagPose(1).get().toPose2d());
        add(AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape).getTagPose(2).get().toPose2d());
        add(AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape).getTagPose(13).get().toPose2d());
        add(AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape).getTagPose(12).get().toPose2d());
    }};

    public static List<Pose2d> tagsProcessor = new ArrayList<>(){{
        add(AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape).getTagPose(3).get().toPose2d());
        add(AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape).getTagPose(16).get().toPose2d());
    }};

    public Field2d field = new Field2d();

    public static Pose2d blueBarge = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape).getTagPose(14).get().toPose2d();
    public static Pose2d redBarge = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape).getTagPose(14).get().toPose2d();



    public Pose2d getNearestReef(Pose2d currentPose){
        return currentPose.nearest(tagsReef);
    }

    public static Pose2d getNearestSource(Pose2d currentPose){
        return currentPose.nearest(tagsSource);
    }

    public static Pose2d getNearestProcessor(Pose2d currentPose){
        var alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        //TODO: Filter/select this one by alliance to prevent possible driver confusion in edge cases
        return currentPose.nearest(tagsProcessor);
    }

    //FROM PERSPECTIVE OF APRIL TAG, if we look from outside it is right
    public static Pose2d getCoralLeft(Pose2d currentPose){
        var nearest = currentPose.nearest(tagsReef);
        return nearest.transformBy(coralLeft);
    }

    public static Pose2d getCoralMid(Pose2d currentPose){
        var nearest = currentPose.nearest(tagsReef);
        return nearest.transformBy(coralMid);
    }

    public static Pose2d getOffsetCoralLeft(Pose2d currentPose){
        var nearest = currentPose.nearest(tagsReef);
        return nearest.transformBy(coralApproachOffsetLeft);
    }

    //SEE ABOVE
    public static Pose2d getCoralRight(Pose2d currentPose){
        var nearest = currentPose.nearest(tagsReef);
        return nearest.transformBy(coralRight);
    }
    public static Pose2d getOffsetCoralRight(Pose2d currentPose){
        var nearest = currentPose.nearest(tagsReef);
        return nearest.transformBy(coralApproachOffsetRight);
    }

    public static Pose2d getCoralSource(Pose2d currentPose){
        var nearest = currentPose.nearest(tagsSource);
        return nearest.transformBy(coralSource);

        
    }

    public static Pose2d getReefAlgae(Pose2d currentPose){
        var nearest = currentPose.nearest(tagsReef);
        return nearest.transformBy(reefAlgae);
    }

    public enum Branch {
        A,
        B,
        C;
    }

    private static Pose2d getBranchRed(Branch target){
        switch (target) {
            case A: return AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape).getTagPose(6).get().toPose2d();//FIXME
        }
        return new Pose2d();
    }

    private static Pose2d getBranchBlue(Branch target){
        return new Pose2d();
    }

    public static Pose2d getBranch(Branch target){
        if(DriverStation.getAlliance().orElse(Alliance.Blue)==Alliance.Blue){
            return getBranchBlue(target);
        }
        else{
            return getBranchRed(target);
        }
    }



}

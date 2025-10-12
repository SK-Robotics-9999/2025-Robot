// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Degrees;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class FieldNavigation {
    public static double botCenterToRearX = Inches.of((28/2.0)+9.0).in(Meters);
    static double pidApproachOffset = Inches.of(((28/2.0)+5)+12).in(Meters);
    static double coralY = Inches.of(13/2.0).in(Meters);
    //These are right relative from the tag's pose facing out  from the reef
    static Transform2d coralLeft = new Transform2d(new Pose2d(), new Pose2d(botCenterToRearX, Inches.of(6.5).in(Meters), new Rotation2d(Degrees.of(0))));
    static Transform2d coralRight = new Transform2d(new Pose2d(), new Pose2d(botCenterToRearX, Inches.of(-6.5).in(Meters), new Rotation2d(Degrees.of(0))));
    static Transform2d coralMid = new Transform2d(new Pose2d(), new Pose2d(botCenterToRearX-Inches.of(5.5).in(Meters), 0.0, new Rotation2d(Degrees.of(0))));
    static Transform2d reefAlgae = new Transform2d(new Pose2d(), new Pose2d(botCenterToRearX, 0, new Rotation2d(Degrees.of(0))));
    static Transform2d coralSource = new Transform2d(new Pose2d(), new Pose2d(botCenterToRearX+Inches.of(30).in(Meters), 0, new Rotation2d(Degrees.of(180))));
    static Transform2d coralApproachOffsetLeft = new Transform2d(new Pose2d(), new Pose2d(pidApproachOffset, Inches.of(6.5).in(Meters), new Rotation2d(Degrees.of(0))));
    static Transform2d coralApproachOffsetRight = new Transform2d(new Pose2d(), new Pose2d(pidApproachOffset, Inches.of(-6.5).in(Meters), new Rotation2d(Degrees.of(0))));
    static Transform2d coralApproachOffsetMid = new Transform2d(new Pose2d(), new Pose2d(pidApproachOffset, 0.0, new Rotation2d(Degrees.of(0))));

    static boolean gotTooClose=false;

    static BooleanSupplier isRed = () -> {
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      return alliance.get() == DriverStation.Alliance.Red;
    }
    return false;
  };

    public static List<Pose2d> tagsReef = new ArrayList<>(){{
        add(AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded).getTagPose(6).get().toPose2d());
        add(AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded).getTagPose(7).get().toPose2d());
        add(AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded).getTagPose(8).get().toPose2d());
        add(AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded).getTagPose(9).get().toPose2d());
        add(AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded).getTagPose(10).get().toPose2d());
        add(AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded).getTagPose(11).get().toPose2d());
        
        add(AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded).getTagPose(17).get().toPose2d());
        add(AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded).getTagPose(18).get().toPose2d());
        add(AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded).getTagPose(19).get().toPose2d());
        add(AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded).getTagPose(20).get().toPose2d());
        add(AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded).getTagPose(21).get().toPose2d());
        add(AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded).getTagPose(22).get().toPose2d());
    }};

    public static List<Pose2d> tagsSource = new ArrayList<>(){{
        add(AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded).getTagPose(1).get().toPose2d());
        add(AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded).getTagPose(2).get().toPose2d());
        add(AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded).getTagPose(13).get().toPose2d());
        add(AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded).getTagPose(12).get().toPose2d());
    }};

    public static List<Pose2d> customPosesSource = new ArrayList<>(){{
        add(new Pose2d(1.9, 1.1, new Rotation2d(Math.toRadians(-170))));
        add(new Pose2d(1.9, 2*4.026-1.1, new Rotation2d(Math.toRadians(170))));
        add(new Pose2d(17.55-1.9, 1.1, new Rotation2d(Math.toRadians(-10))));
        add(new Pose2d(17.55-1.9, 2*4.026-1.1, new Rotation2d(Math.toRadians(10))));
    }};

    public static List<Pose2d> customPosesBackup = new ArrayList<>(){{
        add(new Pose2d(4.5, 2.0, new Rotation2d(Math.toRadians(-170))));
        add(new Pose2d(4.5, 2*4.026-2.0, new Rotation2d(Math.toRadians(170))));
        add(new Pose2d(17.55-4.5, 2.0, new Rotation2d(Math.toRadians(-10))));
        add(new Pose2d(17.55-4.5, 2*4.026-2.0, new Rotation2d(Math.toRadians(10))));
    }};

    public static List<Pose2d> tagsProcessor = new ArrayList<>(){{
        add(AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded).getTagPose(3).get().toPose2d());
        add(AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded).getTagPose(16).get().toPose2d());
    }};

    //although should be translations, need pose to use pose.nearest() method
    public static List<Pose2d> reefCenter = new ArrayList<>(){{
        add(new Pose2d(4.49, 4.026, new Rotation2d()));
        add(new Pose2d(13.06, 4.026, new Rotation2d()));
    }};

    //hexagon inscribed in circle, meters
    public static final double reefRadius = 0.96;
    //Ruined the original one, furthermore we want to radius of the bot, not just the x. even though is rectangle, close enough.
    public static final double botRadius = Inches.of(17).in(Meters) * 1.414;

    public Field2d field = new Field2d();

    public static Pose2d blueBarge = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded).getTagPose(14).get().toPose2d();
    public static Pose2d redBarge = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded).getTagPose(14).get().toPose2d();



    public static Pose2d getNearestReef(Pose2d currentPose){
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

    
    public static Pose2d getCoralMid(Pose2d currentPose){
        var nearest = currentPose.nearest(tagsReef);
        return nearest.transformBy(coralMid);
    }
    
    public static Pose2d getOffsetCoralMid(Pose2d currentPose){
        var nearest = currentPose.nearest(tagsReef);
        return nearest.transformBy(coralApproachOffsetMid);
    }
    
    //FROM PERSPECTIVE OF APRIL TAG, if we look from outside it is right
    public static Pose2d getCoralLeft(Pose2d currentPose){
        var nearest = currentPose.nearest(tagsReef);
        return nearest.transformBy(coralLeft);
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

    public static Pose2d getCoralTag(boolean left, int blueTagID, int redTagID){
        Pose2d tagPose = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded).getTagPose(blueTagID).get().toPose2d();
        if(isRed.getAsBoolean()){
            tagPose = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded).getTagPose(redTagID).get().toPose2d();
        }

        if(left){
            return tagPose.transformBy(coralLeft);
        }
        return tagPose.transformBy(coralRight);
    }

    public static Pose2d getOffsetCoralTag(boolean left, int blueTagID, int redTagID){
        Pose2d tagPose = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded).getTagPose(blueTagID).get().toPose2d();
        if(isRed.getAsBoolean()){
            tagPose = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded).getTagPose(redTagID).get().toPose2d();
        }

        if(left){
            return tagPose.transformBy(coralApproachOffsetLeft);
        }
        return tagPose.transformBy(coralApproachOffsetRight);
    }

    public static Pose2d getCoralSource(Pose2d currentPose){
        var nearest = currentPose.nearest(tagsSource);
        return nearest.transformBy(coralSource);

        
    }

    public static Pose2d getCustomSource(Pose2d currentPose){
        var nearest = currentPose.nearest(customPosesSource);
        return nearest;
    }

    public static Pose2d getCustomBackup(Pose2d currentPose){
        var nearest = currentPose.nearest(customPosesBackup);
        return nearest;
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
            case A: return AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded).getTagPose(6).get().toPose2d();//FIXME
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

    //Too close to reef
    public static boolean getTooCloseToTag(Pose2d currentPose){
        Pose2d nearest = getNearestReef(currentPose);

        Transform2d distance = currentPose.minus(nearest);

        boolean tooCloseX = distance.getMeasureX().in(Inches)<(16.0+17.0);//17 is for bumper edge to robot center
        // SmartDashboard.putBoolean("swerve/tooCloseX", tooCloseX);
        
        boolean tooCloseY = Math.abs(distance.getMeasureY().in(Inches))<25.0;
        // SmartDashboard.putBoolean("swerve/tooCloseY", tooCloseY);



        return tooCloseX && tooCloseY;
    }
    public static boolean getTooCloseToSource(Pose2d currentPose){
        Pose2d nearest = getNearestSource(currentPose);

        Transform2d distance = currentPose.minus(nearest);

        boolean tooCloseX = distance.getMeasureX().in(Inches)<(12.0+17.0);//17 is for bumper edge to robot center
        
        boolean tooCloseY = Math.abs(distance.getMeasureY().in(Inches))<40.0;

        gotTooClose = tooCloseX && tooCloseY;

        return tooCloseX && tooCloseY;
    }

    public static Pose2d getBargeScorePose(Pose2d currentPose){
        Pose2d scorePose = new Pose2d(7.9, currentPose.getY(), new Rotation2d(Math.toDegrees(0)));
        if(isRed.getAsBoolean()){
            scorePose = new Pose2d(17.55-7.9, currentPose.getY(), new Rotation2d(Math.toDegrees(180)));
        }
        
        return scorePose;
    }

    //circle radius (simplification) with center at reef-center
    public static boolean getCrashIntoReef(Pose2d currentPose){
        Pose2d closestReefCenter = currentPose.nearest(reefCenter);
        Translation2d delta = currentPose.getTranslation().minus(closestReefCenter.getTranslation());

        return delta.getNorm() < botRadius+reefRadius;
    }

    //chatgpt goat
    public static boolean segmentIntersectsReef(Pose2d start, Pose2d end) {
        Translation2d p1 = start.getTranslation();
        Translation2d p2 = end.getTranslation();
        Translation2d center = start.nearest(reefCenter).getTranslation();
    
        Translation2d d = p2.minus(p1);
        Translation2d f = p1.minus(center);
    
        double dx = d.getX();
        double dy = d.getY();
        double fx = f.getX();
        double fy = f.getY();
    
        double r = botRadius + reefRadius;
    
        // Compute quadratic coefficients
        double a = dx * dx + dy * dy;
        double b = 2 * (fx * dx + fy * dy);   // dot product f·d
        double c = (fx * fx + fy * fy) - (r * r);
    
        double discriminant = b * b - 4 * a * c;
        if (discriminant < 0) {
            // No intersection
            return false;
        }
    
        discriminant = Math.sqrt(discriminant);
        double t1 = (-b - discriminant) / (2 * a);
        double t2 = (-b + discriminant) / (2 * a);
    
        // Check if either intersection lies on the segment [0, 1]
        return (t1 >= 0 && t1 <= 1) || (t2 >= 0 && t2 <= 1);
    }


}

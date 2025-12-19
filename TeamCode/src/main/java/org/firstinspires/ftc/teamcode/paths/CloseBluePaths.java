package org.firstinspires.ftc.teamcode.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.PathChain;

public class CloseBluePaths {

    private Follower fol;
    public static final Pose startPose = new Pose(28, 131, Math.toRadians(144)); // STARTING POSITION was 23, 124, 144
    final Pose preScorePose = new Pose(60, 104, Math.toRadians(146)); // PRE-LOAD SCORING POSITION
    final Pose row1Line = new Pose(44.5, 84, Math.toRadians(0)); // Position
    final Pose row1Line1CP = new Pose(91,84); // CONTROL POINT
    final Pose row1Grab = new Pose(30, 84, Math.toRadians(0)); // Position
    final Pose row1Score = new Pose(61, 78, Math.toRadians(132)); // Scoring
    final Pose row2Line = new Pose(47, 60, Math.toRadians(0)); // Position
    final Pose row2LineCP = new Pose(85, 60);
    final Pose row2Grab = new Pose(30, 59.5, Math.toRadians(0));
    final Pose row2Score = new Pose(61, 78, Math.toRadians(132));

    final Pose parkPose = new Pose(50, 72, Math.toRadians(132)); // PARKING POSITION
    public PathChain pathPreScore, pathRow1Line, pathRow1Grab, pathRow1Score, pathRow2Line, pathRow2Grab, pathRow2Score, pathParkPose;

    public CloseBluePaths(Follower fol){

            pathPreScore = fol.pathBuilder()
                    .addPath(new BezierLine(startPose, preScorePose))
                    .setLinearHeadingInterpolation(startPose.getHeading(), preScorePose.getHeading())
                    .build();

            pathRow1Line = fol.pathBuilder()
                    .addPath(new BezierLine(preScorePose, row1Line))
                    .setLinearHeadingInterpolation(preScorePose.getHeading(), row1Line.getHeading())
                    //.setTangentHeadingInterpolation()
                    .build();

            pathRow1Grab = fol.pathBuilder()
                    .addPath(new BezierLine(row1Line, row1Grab))
                    .setLinearHeadingInterpolation(row1Line.getHeading(), row1Grab.getHeading())
                    .build();

            pathRow1Score = fol.pathBuilder()
                    .addPath(new BezierLine(row1Grab, row1Score))
                    .setLinearHeadingInterpolation(row1Grab.getHeading(), row1Score.getHeading())
                    .build();

            pathRow2Line = fol.pathBuilder()
                    .addPath(new BezierLine(row1Score, row2Line))
                    .setLinearHeadingInterpolation(row1Score.getHeading(), row2Line.getHeading())
                    .build();

            pathRow2Grab = fol.pathBuilder()
                    .addPath(new BezierLine(row2Line, row2Grab))
                    .setLinearHeadingInterpolation(row2Line.getHeading(), row2Grab.getHeading())
                    .build();

            pathRow2Score = fol.pathBuilder()
                    .addPath(new BezierLine(row2Grab, row2Score))
                    .setLinearHeadingInterpolation(row2Grab.getHeading(), row2Score.getHeading())
                    .build();

            pathParkPose = fol.pathBuilder()
                    .addPath(new BezierLine(row2Score, parkPose))
                    .setLinearHeadingInterpolation(row2Score.getHeading(), parkPose.getHeading())
                    .build();
        }

}


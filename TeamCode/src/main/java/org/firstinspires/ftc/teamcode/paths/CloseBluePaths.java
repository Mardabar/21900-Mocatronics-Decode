//package org.firstinspires.ftc.teamcode.paths;
//
//import com.pedropathing.follower.Follower;
//import com.pedropathing.geometry.Pose;
//import com.pedropathing.geometry.BezierLine;
//import com.pedropathing.paths.PathChain;
//
//public class CloseBluePaths {
//
//
//    /// BOT WIDTH 15.375
//    /// BOT LENGTH 15.75
//
//    /// START POSE
//    public static final Pose startPose = new Pose(27.3, 132.7, Math.toRadians(143));
//    final Pose preScorePose = new Pose(50, 115, Math.toRadians(143));
//    final Pose row1Line = new Pose(48, 84, Math.toRadians(180));
//    final Pose row1Grab = new Pose(23, 84, Math.toRadians(180));
//    final Pose row1Score = new Pose(39.5, 102, Math.toRadians(135));
//    final Pose row2Line = new Pose(48, 59.5, Math.toRadians(180));
//    final Pose row2Grab = new Pose(23, 59.5, Math.toRadians(180));
//    final Pose row2Score = new Pose(52, 88.5, Math.toRadians(135));
//    final Pose row3Line = new Pose (48, 35.5, Math.toRadians(180));
//    final Pose row3Grab = new Pose (20, 35.5, Math.toRadians(180));
//
//    /// Row 3 score and park close
//    final Pose row3ScoreClose = new Pose (57.5, 84.3, Math.toRadians(134));
//    final Pose row3ParkClose = new Pose (55, 63, Math.toRadians(134));
//
//    /// Row 3 score and park far
//    final Pose row3ScoreFar = new Pose (58, 13.5, Math.toRadians(124));
//    final Pose row3ParkFar = new Pose (55.5, 39, Math.toRadians(124));
//
//
//    public PathChain pathPreScore, pathRow1Line, pathRow1Grab, pathRow1Score, pathRow2Line, pathRow2Grab, pathRow2Score, pathRow3Line, pathRow3Grab,  pathRow3Score, pathPark;
//
//
//
//
//
//
//
//    public CloseBluePaths(Follower fol){
//
//        pathPreScore = fol.pathBuilder()
//                .addPath(new BezierLine(startPose, preScorePose))
//                .setLinearHeadingInterpolation(startPose.getHeading(), preScorePose.getHeading())
//                .build();
//
//        pathRow1Line = fol.pathBuilder()
//                .addPath(new BezierLine(preScorePose, row1Line))
//                .setLinearHeadingInterpolation(preScorePose.getHeading(), row1Line.getHeading())
//                .build();
//
//        pathRow1Grab = fol.pathBuilder()
//                .addPath(new BezierLine(row1Line, row1Grab))
//                .setLinearHeadingInterpolation(row1Line.getHeading(), row1Grab.getHeading())
//                .build();
//
//        pathRow1Score = fol.pathBuilder()
//                .addPath(new BezierLine(row1Grab, row1Score))
//                .setLinearHeadingInterpolation(row1Grab.getHeading(), row1Score.getHeading())
//                .build();
//
//        pathRow2Line = fol.pathBuilder()
//                .addPath(new BezierLine(row1Score, row2Line))
//                .setLinearHeadingInterpolation(row1Score.getHeading(), row2Line.getHeading())
//                .build();
//
//        pathRow2Grab = fol.pathBuilder()
//                .addPath(new BezierLine(row2Line, row2Grab))
//                .setLinearHeadingInterpolation(row2Line.getHeading(), row2Grab.getHeading())
//                .build();
//
//        pathRow2Score = fol.pathBuilder()
//                .addPath(new BezierLine(row2Grab, row2Score))
//                .setLinearHeadingInterpolation(row2Grab.getHeading(), row2Score.getHeading())
//                .build();
//
//        pathRow3Line = fol.pathBuilder()
//                .addPath(new BezierLine(row2Score, row3Line))
//                .setLinearHeadingInterpolation(row2Score.getHeading(), row3Line.getHeading())
//                .build();
//
//        pathRow3Grab = fol.pathBuilder()
//                .addPath(new BezierLine(row3Line, row3Grab))
//                .setLinearHeadingInterpolation(row3Line.getHeading(), row3Grab.getHeading())
//                .build();
//
//        //              Row 3 close score and shoot
//        pathRow3Score = fol.pathBuilder()
//                .addPath(new BezierLine(row3Grab, row3ScoreClose))
//                .setLinearHeadingInterpolation(row3Grab.getHeading(), row3ScoreClose.getHeading())
//                .build();
//
//        pathPark = fol.pathBuilder()
//                .addPath(new BezierLine(row3ScoreClose, row3ParkClose))
//                .setLinearHeadingInterpolation(row3ScoreClose.getHeading(), row3ParkClose.getHeading())
//                .build();
//
//        /*              Row 3 far score and shoot
//        pathRow3Score = fol.pathBuilder()
//                .addPath(new BezierLine(row3Grab, row3ScoreFar))
//                .setLinearHeadingInterpolation(row3Grab.getHeading(), row3ScoreFar.getHeading())
//                .build();
//
//        pathPark = fol.pathBuilder()
//                .addPath(new BezierLine(row3ScoreFar, row3ParkFar))
//                .setLinearHeadingInterpolation(row3ScoreFar.getHeading(), row3ParkFar.getHeading())
//                .build();  */
//
//
//    }
//
//
//}
//
//
//        /// Here for posibile code simplifying with multiple paths in one chain
//
////        public PathChain pathPreScore, pathRow1, pathRow1Score, pathRow2Line, pathRow2Grab, pathRow2Score, pathRow3Line, pathRow3Grab;
////    pathRow1 = fol.pathBuilder()
////                    .addPath(new BezierLine(preScorePose, row1Line))
////            .setLinearHeadingInterpolation(preScorePose.getHeading(), row1Line.getHeading())
////            .addPath(new BezierLine(row1Line, row1Grab))
////            .setLinearHeadingInterpolation(row1Line.getHeading(), row1Grab.getHeading())
////            .build();
////
////    pathRow1Score = fol.pathBuilder()
////                    .addPath(new BezierLine(row1Grab, row1Score))
////            .setLinearHeadingInterpolation(row1Grab.getHeading(), row1Score.getHeading())
////            .build();
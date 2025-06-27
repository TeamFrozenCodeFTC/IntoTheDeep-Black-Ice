package org.firstinspires.ftc.teamcode.blackIce.examples;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.blackIce.action.Condition;
import org.firstinspires.ftc.teamcode.blackIce.math.geometry.Pose;
import org.firstinspires.ftc.teamcode.blackIce.math.geometry.Vector;
import org.firstinspires.ftc.teamcode.blackIce.follower.Follower;
import org.firstinspires.ftc.teamcode.blackIce.paths.PathSequenceConstructor;
import org.firstinspires.ftc.teamcode.blackIce.paths.Path;
import org.firstinspires.ftc.teamcode.blackIce.paths.segments.BezierCurve;
import org.firstinspires.ftc.teamcode.blackIce.util.Logger;

@Autonomous(group="Black-Ice Examples")
public class LineTransition extends LinearOpMode {
    @Override
    public void runOpMode() {
        Follower follower = new Follower(this, new Pose(0,0,0));

        PathSequenceConstructor constructor = follower.pathSequenceConstructor();
        
//        Path path = constructor.toPoint(new Vector(36, 0))
//            .maximizeSpeed()
//            .continueMomentumAtEnd();
        
        Path path = constructor.buildSegments()
            .toPoint(new Vector(36, 0))
            .toPoint(new Vector(36, 24))
            .build()
            .stopAtEnd();

        // TODO copyBehavior(path)
//        Path path2 = constructor.toPoint(new Vector(36, 12))
//            .maximizeSpeed()
//            .stopAtEnd()
//            .holdUntil(Condition.NEVER);
        
//        Logger.debug("YY previous target pose2", constructor.buildSegments().getPreviousTargetPoint());
//
//        Logger.debug("YY END POSE", path2.getEndPose().getPosition());
//        Logger.debug("YY LENGTH", path2.length);
        
        waitForStart();
        
        follower.beginFollowing(path);
        
        while (follower.isFollowingPath() && opModeIsActive()) {
            follower.update();
        }
        
//        follower.beginFollowing(path2);
//
//        while (follower.isFollowingPath() && opModeIsActive()) {
//            follower.update();
//        }
        
        // TODO path Maker op-mode
        // TODO motor/servo opmodes (just find online)
    }
}
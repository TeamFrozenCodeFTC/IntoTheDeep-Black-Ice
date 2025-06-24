package org.firstinspires.ftc.teamcode.blackIce.follower;

import org.firstinspires.ftc.teamcode.blackIce.motion.MotionState;
import org.firstinspires.ftc.teamcode.blackIce.paths.FollowingState;
import org.firstinspires.ftc.teamcode.blackIce.paths.Path;
import org.firstinspires.ftc.teamcode.blackIce.paths.PathExecutor;
import org.firstinspires.ftc.teamcode.blackIce.util.Advancer;

public class PathSequenceExecutor {
    private final Advancer<PathExecutor> advancer;
    
    public PathSequenceExecutor(PathExecutor... executors) {
        this.advancer = new Advancer<>(executors);
    }
    
    public PathExecutor current() {
        return advancer.current();
    }
    
    public boolean update(MotionState motionState) {
        current().updateDrivePowersToFollowPath(motionState);
        if (current().getState() == FollowingState.DONE) {
            current().start();
            return advancer.advance();
        }
        return true;
    }
    
    public Path getCurrentPath() {
        return current().getPath();
    }
    
    public boolean isDone() {
        return advancer.isDone();
    }
}
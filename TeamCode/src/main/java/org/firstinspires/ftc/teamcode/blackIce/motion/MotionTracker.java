package org.firstinspires.ftc.teamcode.blackIce.motion;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.blackIce.math.geometry.Vector;
import org.firstinspires.ftc.teamcode.blackIce.localization.Localizer;

/**
 * Updates the robot's {@link MotionState} based on the {@link Localizer}'s data.
 */
public class MotionTracker {
    private final Localizer localizer;
    
    private final ElapsedTime stuckDetectedTimer = new ElapsedTime(0);

    public MotionTracker(Localizer localizer) {
        this.localizer = localizer;
    }

    private double lastTime = System.nanoTime();
    private Vector previousRobotRelativeVelocity = new Vector(0,0);
    private MotionState motionState;
    
    public void update() {
        localizer.update();

        Vector position = new Vector(
            localizer.getX(),
            localizer.getY()
        );
        Vector fieldRelativeVelocity = new Vector(
            localizer.getFieldVelocityX(), // pinpoint is not robot relative?
            localizer.getFieldVelocityY()
        );
        Vector robotRelativeVelocity = fieldRelativeVelocity.toRobotVector(localizer.getHeading());
        motionState = new MotionState(
            position,
            localizer.getHeading(),
            localizer.getAngularVelocity(),
            computeDeltaTime(),
            fieldRelativeVelocity,
            robotRelativeVelocity.computeMagnitude(),
            robotRelativeVelocity,
            previousRobotRelativeVelocity
        );
        previousRobotRelativeVelocity = robotRelativeVelocity;
    }
    
    public MotionState getMotionState() {
        return motionState;
    }
    
    private double computeDeltaTime() {
        double currentTime = System.nanoTime();
        double deltaTime = (currentTime - lastTime) / 1e9;
        lastTime = currentTime;
        return deltaTime;
    }
    
    public void resetStuckTimer() {
        stuckDetectedTimer.reset();
    }
}

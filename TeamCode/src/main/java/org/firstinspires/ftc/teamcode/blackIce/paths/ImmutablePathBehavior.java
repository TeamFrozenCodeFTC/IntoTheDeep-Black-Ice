package org.firstinspires.ftc.teamcode.blackIce.paths;

import org.firstinspires.ftc.teamcode.blackIce.paths.behavior.DrivePowerModifier;
import org.firstinspires.ftc.teamcode.blackIce.paths.behavior.HeadingInterpolator;
import org.firstinspires.ftc.teamcode.blackIce.paths.behavior.VelocityProfile;

import java.lang.reflect.Field;

public class ImmutablePathBehavior {
    public final ActionLoop actionLoop;
    
    public final boolean stopAtEnd;

    public final boolean decelerateWithoutFeedforward;
    
    public final double stoppedVelocityConstraint;
    public final double stoppedAngularVelocityConstraint;
    
    public final double stuckVelocityConstraint;
    public final double stuckTimeoutSeconds;
    public final boolean cancelPathIfStuck;
    
    public final double timeoutSeconds;

    public final VelocityProfile velocityProfile;
    public final DrivePowerModifier drivePowerModifier;
    
    public ImmutablePathBehavior(ActionLoop actionLoop, boolean stopAtEnd, boolean decelerateWithoutFeedforward,
                                 double stoppedVelocityConstraint,
                                 double stoppedAngularVelocityConstraint,
                                 double stuckVelocityConstraint, double stuckTimeoutSeconds,
                                 boolean cancelPathIfStuck, double timeoutSeconds,
                                 VelocityProfile velocityProfile, DrivePowerModifier drivePowerModifier) {
        this.actionLoop = actionLoop;
        this.stopAtEnd = stopAtEnd;
        this.decelerateWithoutFeedforward = decelerateWithoutFeedforward;
        this.stoppedVelocityConstraint = stoppedVelocityConstraint;
        this.stoppedAngularVelocityConstraint = stoppedAngularVelocityConstraint;
        this.stuckVelocityConstraint = stuckVelocityConstraint;
        this.stuckTimeoutSeconds = stuckTimeoutSeconds;
        this.cancelPathIfStuck = cancelPathIfStuck;
        this.timeoutSeconds = timeoutSeconds;
        this.velocityProfile = velocityProfile;
        this.drivePowerModifier = drivePowerModifier;
    }
    
    public PathBehavior toBuilder() {
        PathBehavior builder = new PathBehavior();
        try {
            for (Field field : ImmutablePathBehavior.class.getDeclaredFields()) {
                field.setAccessible(true);
                Object value = field.get(this);
                field.set(builder, value);
            }
        } catch (IllegalAccessException e) {
            throw new RuntimeException("Reflection error while copying behavior", e);
        }
        return builder;
    }
}

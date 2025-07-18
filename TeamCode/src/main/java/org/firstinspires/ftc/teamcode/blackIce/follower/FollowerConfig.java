package org.firstinspires.ftc.teamcode.blackIce.follower;

import org.firstinspires.ftc.teamcode.blackIce.controller.PIDController;
import org.firstinspires.ftc.teamcode.blackIce.localization.Localizer;
import org.firstinspires.ftc.teamcode.blackIce.math.kinematics.VelocityToStoppingDistanceVectorModel;
import org.firstinspires.ftc.teamcode.blackIce.paths.PathBehaviorModifier;

import org.firstinspires.ftc.teamcode.blackIce.controller.PIDFController;
import org.firstinspires.ftc.teamcode.blackIce.robot.drivetrain.Drivetrain;

/**
 * Configuration for a Follower.
 * DefaultFollower -> Follower (OpMode) -> Path
 * <p>
 * Important: Might be able to create a static follower if you don't initialize it with hardware map.
 */
public class FollowerConfig {
    public Localizer localizer;
    public Drivetrain drivetrain;
    public PIDController headingPID;
    public PIDController positionalPID;
    public PIDController translationalPID;
    public PIDFController driveVelocityPIDF;
    public PathBehaviorModifier defaultPathBehavior = path -> {};
    public VelocityToStoppingDistanceVectorModel brakingDisplacement;
    
    /**
     * Adds default behavior to the each path. Each path can override specific behaviors.
     */
    public FollowerConfig addDefaultPathBehavior(PathBehaviorModifier defaultPathBehavior) {
        this.defaultPathBehavior.combine(defaultPathBehavior);
        return this;
    }
    
    /**
     * Note: This completely replaces the previous default path behavior.
     * <p>
     * To extend the default path behavior use, {@link #addDefaultPathBehavior}.
     */
    public FollowerConfig defaultPathBehavior(PathBehaviorModifier defaultPathBehavior) {
        this.defaultPathBehavior = defaultPathBehavior;
        return this;
    }
    
    /**
     * The localizer used to determine the robot's position on the field.
     */
    public FollowerConfig localizer(Localizer localizer) {
        this.localizer = localizer;
        return this;
    }

    public FollowerConfig drivetrain(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        return this;
    }
    
    public FollowerConfig headingPIDF(PIDFController headingPIDF) {
        this.headingPID = headingPIDF;
        return this;
    }
    
    public FollowerConfig positionalPIDF(PIDFController positionalPIDF) {
        this.positionalPID = positionalPIDF;
        return this;
    }
    
    public FollowerConfig driveVelocityPIDF(PIDFController driveVelocityPIDF) {
        this.driveVelocityPIDF = driveVelocityPIDF;
        return this;
    }
    
    public FollowerConfig translationalPIDF(PIDFController translationalPIDF) {
        this.translationalPID = translationalPIDF;
        return this;
    }
    
    public FollowerConfig brakingDisplacement(VelocityToStoppingDistanceVectorModel brakingDisplacement) {
        this.brakingDisplacement = brakingDisplacement;
        return this;
    }
}

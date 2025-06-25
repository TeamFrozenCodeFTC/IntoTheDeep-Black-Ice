package org.firstinspires.ftc.teamcode.blackIce.follower;

import org.firstinspires.ftc.teamcode.blackIce.controller.PIDController;
import org.firstinspires.ftc.teamcode.blackIce.localization.Localizer;
import org.firstinspires.ftc.teamcode.blackIce.math.kinematics.VelocityToStoppingDistanceVectorModel;
import org.firstinspires.ftc.teamcode.blackIce.paths.PathBehavior;

import org.firstinspires.ftc.teamcode.blackIce.controller.PIDFController;
import org.firstinspires.ftc.teamcode.blackIce.robot.drivetrain.Drivetrain;

/**
 * Configuration for a Follower.
 * DefaultFollower -> Follower (OpMode) -> Path
 * <p>
 * Important: Might be able to create a static follower if you don't initialize it with hardware map.
 */
public class FollowerConstants {
    public final Localizer localizer;
    public final Drivetrain drivetrain;
    public final PIDController headingPID;
    public final PIDController positionalPID;
    public final PIDController translationalPID;
    public final PIDFController driveVelocityPIDF;
    public final PathBehavior defaultPathBehavior;
    public final VelocityToStoppingDistanceVectorModel brakingDisplacement;
    
    public FollowerConstants(Localizer localizer, Drivetrain drivetrain, PIDController headingPIDF,
                             PIDController positionalPIDF, PIDController translationalPID,
                             PIDFController driveVelocityPIDF, PathBehavior defaultPathBehavior,
                             VelocityToStoppingDistanceVectorModel brakingDisplacement) {
        this.localizer = localizer;
        this.drivetrain = drivetrain;
        this.headingPID = headingPIDF;
        this.positionalPID = positionalPIDF;
        this.translationalPID = translationalPID;
        this.driveVelocityPIDF = driveVelocityPIDF;
        this.defaultPathBehavior = defaultPathBehavior;
        this.brakingDisplacement = brakingDisplacement;
    }
    
    public static class Builder {
        public Localizer localizer;
        public Drivetrain drivetrain;
        public PIDController headingPIDF;
        public PIDController positionalPIDF;
        public PIDController translationalPIDF;
        public PIDFController driveVelocityPIDF;
        public PathBehavior defaultPathBehavior = path -> {};
        public VelocityToStoppingDistanceVectorModel brakingDisplacement;
        
        public FollowerConstants build() {
            return new FollowerConstants(localizer, drivetrain, headingPIDF, positionalPIDF,
                translationalPIDF, driveVelocityPIDF, defaultPathBehavior, brakingDisplacement);
        }
        
        public Builder addDefaultPathBehavior(PathBehavior defaultPathBehavior) {
            this.defaultPathBehavior.combine(defaultPathBehavior);
            return this;
        }
        
        /**
         * The localizer used to determine the robot's position on the field.
         */
        public Builder localizer(Localizer localizer) {
            this.localizer = localizer;
            return this;
        }

        public Builder drivetrain(Drivetrain drivetrain) {
            this.drivetrain = drivetrain;
            return this;
        }
        
        public Builder headingPIDF(PIDFController headingPIDF) {
            this.headingPIDF = headingPIDF;
            return this;
        }
        
        public Builder positionalPIDF(PIDFController positionalPIDF) {
            this.positionalPIDF = positionalPIDF;
            return this;
        }
        
        public Builder driveVelocityPIDF(PIDFController driveVelocityPIDF) {
            this.driveVelocityPIDF = driveVelocityPIDF;
            return this;
        }
        
        public Builder translationalPIDF(PIDFController translationalPIDF) {
            this.translationalPIDF = translationalPIDF;
            return this;
        }
        
        public Builder brakingDisplacement(VelocityToStoppingDistanceVectorModel brakingDisplacement) {
            this.brakingDisplacement = brakingDisplacement;
            return this;
        }
     }
}

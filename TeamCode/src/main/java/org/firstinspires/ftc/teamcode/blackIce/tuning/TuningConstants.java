package org.firstinspires.ftc.teamcode.blackIce.tuning;

import org.firstinspires.ftc.teamcode.blackIce.math.kinematics.QuadraticBrakingModel;
import org.firstinspires.ftc.teamcode.blackIce.math.kinematics.QuadraticLinearBrakingModel;
import org.firstinspires.ftc.teamcode.blackIce.math.kinematics.VelocityToStoppingDistanceVectorModel;

public class TuningConstants {
//    public static final SignedQuadratic FORWARD_BRAKING_DISPLACEMENT =
//        new SignedQuadratic(0.00112, 0.07316, 0.00577);
//    // TODO try removing c term because of little impact on accuracy and increase in performance
//
//    public static final SignedQuadratic LATERAL_BRAKING_DISPLACEMENT =
//        new SignedQuadratic(0.00165, 0.05054, 0.01029);

    public static final double kP = 0.025; // about 0.05 inches of error
    public static final double kD = 0.0001;

    public static final double MAX_FORWARD_VELOCITY = 60;
    public static final double MAX_LATERAL_VELOCITY = 45;

//    public static final Vector VELOCITY_SCALING_VECTOR = new Vector(
//        MAX_FORWARD_VELOCITY,
//        MAX_LATERAL_VELOCITY
//    ).normalized();

    // TODO test these with just acceleration values too

//    // instant stopping
    public static VelocityToStoppingDistanceVectorModel BRAKING_DISPLACEMENT =
        new VelocityToStoppingDistanceVectorModel(
     //Drift, braking force
            new QuadraticLinearBrakingModel(0.00112, 0.07316),
            new QuadraticLinearBrakingModel(0.00165, 0.05054)
        );

//    public static VelocityToStoppingDistanceVectorModel BRAKING_DISPLACEMENT =
//        new VelocityToStoppingDistanceVectorModel(
//            //Drift, braking force
//            new QuadraticBrakingModel(-150),
//            new QuadraticBrakingModel(-170)
//        );
//
    // decreasing the stopping distance makes the robot think it will undershoot so it goes faster
    // but if I increase the stopping distance it makes the robot think is going to overshoot so it stops faster

    
//    // the higher the number, the more braking distance and the slower the robot decelerates
//    public static VelocityToStoppingDistanceVectorModel BRAKING_DISPLACEMENT =
//        new VelocityToStoppingDistanceVectorModel(
//            //Drift, braking force
//            new QuadraticLinearBrakingModel(0.005, 0.07316),
//            new QuadraticLinearBrakingModel(0.005, 0.05054)
//        );
//
    // Solely QuadraticBrakingModel
//    public static VelocityToStoppingDistanceVectorModel BRAKING_DISPLACEMENT =
//        new VelocityToStoppingDistanceVectorModel(
//            new QuadraticLinearBrakingModel(0.005, 0),
//            new QuadraticLinearBrakingModel(0.005, 0)
//        );

//public static VelocityToStoppingDistanceVectorModel BRAKING_DISPLACEMENT =
//    new VelocityToStoppingDistanceVectorModel(
//        new QuadraticBrakingModel(-300),
//        new QuadraticBrakingModel(-300)
//    ); // in braking calc the lateral thing is off because drive powers duplicates the same affect


    // get the angle of this and multiply the drive vector by the angle

//    public static final VelocityToStoppingDistanceVectorModel FORWARD_BRAKING_DISPLACEMENT =
//        new VelocityToStoppingDistanceVectorModel(0.00112, 0.07316);
//
//    public static final VelocityToStoppingDistanceVectorModel LATERAL_BRAKING_DISPLACEMENT =
//        new VelocityToStoppingDistanceVectorModel(0.00165, 0.05054);

    public static final double PROPORTIONAL_CONSTANT = 1.5; // should be 999 or set magnitude to 1
    public static final double DRIVE_D_CONSTANT = 1.5; // should be 999 or set magnitude to 1
    public static final double HOLDING_PROPORTIONAL_CONSTANT = 1;

    // 1. Go to blackIce.drive.Drive to change names of wheel motors and their directions.
    // 2. Configure odometry hardware names and offsets in BlackIce.odometry.GoBildaOdometryTest
}

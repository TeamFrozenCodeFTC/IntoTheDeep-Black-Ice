package org.firstinspires.ftc.teamcode.blackIce.tuning;

public class TuningConstants {
    public static final SignedQuadratic FORWARD_BRAKING_DISPLACEMENT =
        new SignedQuadratic(0.00112, 0.07316, 0.00577);
    // TODO try removing c term because of little impact on accuracy and increase in performance

    public static final SignedQuadratic LATERAL_BRAKING_DISPLACEMENT =
        new SignedQuadratic(0.00165, 0.05054, 0.01029);

    public static final double PROPORTIONAL_CONSTANT = 1.5; // should be 999 or set magnitude to 1
    public static final double HOLDING_PROPORTIONAL_CONSTANT = 1;

    // 1. Go to blackIce.drive.Drive to change names of wheel motors and their directions.
    // 2. Configure odometry hardware names and offsets in BlackIce.odometry.GoBildaOdometryTest
}

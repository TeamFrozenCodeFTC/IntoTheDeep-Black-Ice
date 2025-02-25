package org.firstinspires.ftc.teamcode.blackIce.tuning;

import org.firstinspires.ftc.teamcode.blackIce.BrakingDisplacement;

public class TuningConstants {
    public static final BrakingDisplacement FORWARD_BRAKING_DISPLACEMENT =
        new BrakingDisplacement(0.00112, 0.07316, 0.00577);

    public static final BrakingDisplacement LATERAL_BRAKING_DISPLACEMENT =
        new BrakingDisplacement(0.00165, 0.05054, 0.01029);

//    public static final BrakingDisplacement BRAKING_DISPLACEMENT =
//        new BrakingDisplacement(0.00130445, 0.0644448, 0.0179835);
}
// TODO use static {} blocks
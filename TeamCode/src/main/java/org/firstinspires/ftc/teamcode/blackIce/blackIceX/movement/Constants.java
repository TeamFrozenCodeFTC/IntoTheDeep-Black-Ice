package org.firstinspires.ftc.teamcode.blackIce.blackIceX.movement;

public class Constants {
    public static class Measurement {
        static final int TILE = 24;
        static final int ROBOT = 18;
        static final int HALF_OF_ROBOT = ROBOT / 2;
        static final int EDGE_OF_TILE = TILE - ROBOT;
        static final double ROBOT_TURN_RADIUS = Math.sqrt(Math.pow(HALF_OF_ROBOT, 2) * 2);
    }

    public static class TurnCorrection {
        static final double FINISH_TURN_BY_PERCENT = 0.20;
        static final double TURN_POWER = 0.03;
    }
}
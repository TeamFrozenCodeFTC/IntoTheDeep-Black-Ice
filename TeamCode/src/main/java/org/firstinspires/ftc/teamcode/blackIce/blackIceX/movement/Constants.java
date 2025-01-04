package org.firstinspires.ftc.teamcode.blackIce.blackIceX.movement;

public class Constants {
    public static class Measurement {
        public static final int TILE = 24;
        public static final int ROBOT = 18;
        public static final int HALF_OF_ROBOT = ROBOT / 2;
        public static final int EDGE_OF_TILE = TILE - ROBOT;
        public static final double ROBOT_TURN_RADIUS = Math.sqrt(Math.pow(HALF_OF_ROBOT, 2) * 2);
    }

    public static class TurnCorrection {
        public static final double FINISH_TURN_BY_PERCENT = 0.20;
        public static final double TURN_POWER = 0.03;
    }
}
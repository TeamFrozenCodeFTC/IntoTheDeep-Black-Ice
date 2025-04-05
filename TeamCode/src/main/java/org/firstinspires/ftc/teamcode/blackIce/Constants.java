package org.firstinspires.ftc.teamcode.blackIce;

public class Constants {
    public static class Measurement {
        public static final int TILE = 24;
        public static final int ROBOT = 18;
        public static final int HALF_OF_ROBOT = ROBOT / 2;
        public static final int EDGE_OF_TILE = TILE - ROBOT;
        public static final double ROBOT_TURN_RADIUS = Math.sqrt(Math.pow(HALF_OF_ROBOT, 2) * 2);
        public static final double EXTRA_TURN_RADIUS = ROBOT_TURN_RADIUS - HALF_OF_ROBOT;
        public static final double EDGE_OF_SUBMERSIBLE = 4.25;
    }

    public static class Curve {
        public static double INCHES_PER_POINT = 2;
        public static int LOOK_AHEAD_POINTS_FOR_HEADING = 2; // Looks ahead x points to turn in that direction
    }

    public static class TurnCorrection {
        public static final double FINISH_TURN_BY_PERCENT = 0.20;
        public static final double TURN_POWER = 0.03;
    }
}
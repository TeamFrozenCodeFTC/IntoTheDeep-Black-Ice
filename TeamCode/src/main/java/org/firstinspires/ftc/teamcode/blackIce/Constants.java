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

    public static class TurnCorrection {
        public static final double FINISH_TURN_BY_PERCENT = 0.20;
        public static final double TURN_POWER = 0.03;//0.03;



        // 1. Heading Complete Percentage 20% - finishes turning by 20% before the end of the movement

        // 2. Heading Proportional - 0.03

        // 6,7,8 Error Margin, x,y,heading

        // 9, 10 x and y for braking distances (automatic)

        // Little to none arbitrary tuning variables
        // Black Ice has less than 10 tuning variables while Pedro path has more than 60

        // Only the automatic braking distance test is required to use movements that don't fully brake
    }
}
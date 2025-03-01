package org.firstinspires.ftc.teamcode.autonomous.tests.loopTimes;

import org.firstinspires.ftc.teamcode.blackIce.Follower;

public class LoopTime {
    private static final int LOOP_TIMES = 1000;

    public static void getLoopTime(Runnable function) {
        long startTime = System.nanoTime();
        for (int i = 0; i < LOOP_TIMES; i++) {
            function.run();
        }
        long endTime = System.nanoTime();
        long duration = endTime - startTime;

        Follower.telemetry.addData("Execution Time (ms)", duration / 1_000_000.0);
        Follower.telemetry.addData("Per Loop (ms)", duration / 1_000_000.0 / LOOP_TIMES);
        Follower.telemetry.update();
    }

}

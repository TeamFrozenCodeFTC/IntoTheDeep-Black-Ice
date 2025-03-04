package org.firstinspires.ftc.teamcode.development.loopTimes;

import org.firstinspires.ftc.teamcode.blackIce.Follower;

import java.util.Arrays;

//
//public class LoopTime {
//    private static final int LOOP_TIMES = 1000;
//
//    public static void getLoopTime(Runnable function) {
//        System.gc();
//
//        for (int i = 0; i < 500; i++) {
//            function.run();
//        }
//
//        Thread.currentThread().setPriority(Thread.MAX_PRIORITY);
//
//        long startTime = System.nanoTime();
//        for (int i = 0; i < LOOP_TIMES; i++) {
//            function.run();
//        }
//        long endTime = System.nanoTime();
//        long duration = endTime - startTime;
//
//        Follower.telemetry.addData("Execution Time (ms)", duration / 1_000_000.0);
//        Follower.telemetry.addData("Per Loop (ms)", duration / 1_000_000.0 / LOOP_TIMES);
//        Follower.telemetry.update();
//    }
//
//}
public class LoopTime {
    private static final int LOOP_TIMES = 1000;

    public static void getLoopTime(Runnable function) {
        System.gc();

        // Warm-up phase
        for (int i = 0; i < 500; i++) {
            function.run();
        }

        Thread.currentThread().setPriority(Thread.MAX_PRIORITY);

        // Store individual loop times
        long[] loopTimes = new long[LOOP_TIMES];

        // Measure time per iteration
        for (int i = 0; i < LOOP_TIMES; i++) {
            long startTime = System.nanoTime();
            function.run();
            loopTimes[i] = System.nanoTime() - startTime;
        }

        // Sort the times to find the median
        Arrays.sort(loopTimes);
        double medianTime = (loopTimes[LOOP_TIMES / 2 - 1] + loopTimes[LOOP_TIMES / 2]) / 2.0;

        // Convert to milliseconds
        medianTime /= 1_000_000.0;

        // Compute total duration for reference
        long totalDuration = Arrays.stream(loopTimes).sum();
        double averageTime = totalDuration / 1_000_000.0 / LOOP_TIMES;

        // Telemetry output
        Follower.telemetry.addData("Total Execution Time (ms)", totalDuration / 1_000_000.0);
        Follower.telemetry.addData("Average Per Loop (ms)", averageTime);
        Follower.telemetry.addData("Median Per Loop (ms)", medianTime);
        Follower.telemetry.update();
    }
}
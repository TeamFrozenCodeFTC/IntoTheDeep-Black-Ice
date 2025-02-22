package org.firstinspires.ftc.teamcode.autonomous.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Robot;

import java.util.concurrent.atomic.AtomicReference;

@Autonomous(group="Tests")
public class MoreAccurateStopAt extends Robot {
    @Override
    public void runOpMode() {
        initRobot();
        waitForStart();

        clawLeft.getController().pwmDisable();

        odometry.setPosition(0, 0, 0);

        AtomicReference<Double> totalXError = new AtomicReference<>((double) 0);
        AtomicReference<Double> totalYError = new AtomicReference<>((double) 0);

        movement.buildMovement(0, 48, 0)
            .stopAtPosition()
            .setDriveCorrection(() -> {
                totalXError.updateAndGet(v -> Double.valueOf(v + movement.target.xError));
                totalYError.updateAndGet(v -> Double.valueOf(v + movement.target.yError));

                return movement.driveCorrections.fieldVectorToLocalWheelPowers(new double[]{
                    (movement.target.xError - odometry.xBrakingDistance),
                    (movement.target.yError - odometry.yBrakingDistance) // * 1 higher becomes more unstable but more accurate, lower becomes more stable but less accurate
                });
            })
            .setMovementExit(() -> false)
            .runTimeout(999);
    }
}
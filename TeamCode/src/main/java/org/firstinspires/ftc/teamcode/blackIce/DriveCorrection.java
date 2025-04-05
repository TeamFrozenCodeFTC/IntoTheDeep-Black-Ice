package org.firstinspires.ftc.teamcode.blackIce;

import static org.firstinspires.ftc.teamcode.blackIce.Drive.fieldVectorToLocalWheelPowers;

import org.firstinspires.ftc.teamcode.blackIce.tuning.TuningConstants;
import org.firstinspires.ftc.teamcode.blackIce.odometry.Odometry;

public interface DriveCorrection {
    double[] calculateDrivePowers();

    DriveCorrection adjustWithBrakingDistanceWithoutStop = () -> {
        double x = Target.xError - Odometry.xBrakingDistance;
        double y = Target.yError - Odometry.yBrakingDistance;

        if (Math.abs(x) < 1) {
            x = Target.xError; // * 3?
        }
        if (Math.abs(y) < 1) {
            y = Target.yError;
        }

        return fieldVectorToLocalWheelPowers(new double[]{x, y});
    };

//    double lastError = 0; // put this in Target
//    double lastTime = 0;
    /**
     * Stop at the target using proportional control and predicting braking distance.
     * {@code (error * predictedBrakingDisplacement) * constant}
     */
    DriveCorrection velocityConstraints = () -> {
        double xPower = (Target.xError);
        double yPower = (Target.yError);

        Follower.telemetry.addData("velocity", Odometry.velocity);
        Follower.telemetry.update();
//
//        double currentTime = System.nanoTime() / 1_000_000_000.0;
//        if (lastTime == 0) { // First run, no previous time
//            lastTime = currentTime;
//            lastError = (40 - Odometry.velocity);
//            return 0;
//        }
//
//        double deltaTime = currentTime - lastTime;
//        double rateOfChange = ( (40 - Odometry.velocity) - lastError) / deltaTime;
//
//        lastTime = currentTime;
//        lastError =  (40 - Odometry.velocity);

        //Vector.setMagnitude(xPower, yPower, 30 - Odometry.velocity); // only downscale
        //Vector.setMagnitude(xPower, yPower, 30 / Odometry.velocity); // only downscale
//        Vector.scaleToMax(xPower, yPower, (30 - Odometry.xVelocity) * 0.05); // only downscale
        return fieldVectorToLocalWheelPowers(
            Vector.scaleToMax(xPower, yPower,
                Math.max(0, (40 - Odometry.velocity) * 0.5) + 40 * 0.4 + 0.1)); // only downscale
        // at 0 velocity xPower and yPower max is 30
        // at 29 velocity xPower and yPower max is 1
        // at 31 velocity xPower and yPower max is -1

        //

        // 0.5, 0


        // deceleration -30 inches/second^2
        // velocity inches/second

        // feedforward? targetVelocity^2 * c squared?
        // velocityError * c + 0.3  only when xPower is greater than 1
        //  implement acceleration constraints later

        // set magnitude to velocityError

//        double currentMag = Math.sqrt(xPower * xPower + yPower * yPower);

        // TODO add derivative term PIDF

//        if (xPower > 1) {
//            double scale = ((30 - Math.abs(Odometry.xVelocity)) * 0.05 + 0.1) / currentMag;
//                // target / xVelocity
//            xPower = xPower * scale;
//        }
//        if (yPower > 1) {
//            double scale = ((30 - Math.abs(Odometry.yVelocity)) * 0.05 + 0.1) / currentMag;
//            yPower = yPower * scale;
//        }

        // Set xError and yError vector's magnitude to velocityError

//        return fieldVectorToLocalWheelPowers(
//            new double[]{
//                xPower,
//                yPower
//            }
//        );
    };

    /**
     * Stop at the target using proportional control and predicting real-time braking distance.
     * <p>
     * {@code (error - predictedBrakingDistance) * constant}
     * (Note: brakingDistance is directional, meaning it can be negative).
     */
    DriveCorrection stopAtTarget = () -> fieldVectorToLocalWheelPowers(
        new double[]{
            (Target.xError - Odometry.xBrakingDistance) * TuningConstants.HOLDING_PROPORTIONAL_CONSTANT,
            (Target.yError - Odometry.yBrakingDistance) * TuningConstants.HOLDING_PROPORTIONAL_CONSTANT
        }
    );

    DriveCorrection proportional = () -> fieldVectorToLocalWheelPowers(
        new double[]{
            Target.xError * TuningConstants.PROPORTIONAL_CONSTANT,
            Target.yError * TuningConstants.PROPORTIONAL_CONSTANT,
        });
    //

//    /**
//     * Drives toward the target at full power.
//     * <p>
//     * Uses {@link Vector#scaleToMax} meaning at least lateral or forward power will 1.
//     */
//    DriveCorrection fullPower = () -> fieldVectorToLocalWheelPowers(
//        Vector.scaleToMax( // or normalize vector? (powers would end up like (0.447,0.894))
//            Target.xError * Math.abs(Odometry.yBrakingDistance),
//            Target.yError * Math.abs(Odometry.xBrakingDistance),
//            1
//        )
//    );

    DriveCorrection fullPowerWithBrakingCorrectionUpScale = () -> Drive.fieldVectorToLocalWheelPowers(
        Vector.scaleToMax(
            (Target.xError - Odometry.xBrakingDistance),
            (Target.yError - Odometry.yBrakingDistance),
            1
        )
    );

    DriveCorrection fullPower = () -> Drive.fieldVectorToLocalWheelPowers(
        Vector.scaleToMax( // downscale?
            (Target.xError),
            (Target.yError),
            1
        )
    );


    /**
     * Drives toward the target at full power.
     * <p>
     * The magnitude of the drive vector will be one.
     */
    DriveCorrection fullPowerNormalized = () -> fieldVectorToLocalWheelPowers(
        Vector.setMagnitudeToOne(
            Target.xError,
            Target.yError
        )
    );
}

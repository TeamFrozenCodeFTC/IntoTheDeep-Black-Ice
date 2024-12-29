//package org.firstinspires.ftc.teamcode.blackIce.blackIceX.movement;
//
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.blackIce.blackIceX.Movement;
//
//public abstract class Tuning extends Movement {
//    /**
//     * Moves the robot straight with a set power for the given seconds. (Includes turn correction).
//     */
//    public void goStraightForSeconds(double seconds, double power) {
//        ElapsedTime timer = new ElapsedTime();
//
//        timer.reset();
//        while (opModeIsActive() && timer.seconds() < seconds) {
//            updatePosition();
//            goStraight(power);
//        }
//    }
//
//    /**
//     * Moves the robot straight with a set power for the given seconds. (Includes turn correction).
//     */
//    public void slideForSeconds(double seconds, double power) {
//        ElapsedTime timer = new ElapsedTime();
//
//        timer.reset();
//        while (opModeIsActive() && timer.seconds() < seconds) {
//            updatePosition();
//            slide(power);
//        }
//    }
//
//    /**
//     * Brakes the robot for the given seconds.
//     */
//    public void brakeForSeconds(double seconds) {
//        ElapsedTime timer = new ElapsedTime();
//
//        timer.reset();
//        while (opModeIsActive() && timer.seconds() < seconds) {
//            updatePosition();
//            brake();
//        }
//    }
//
//    /**
//     * Locks the robot at the last target pose for the given seconds.
//     */
//    public void holdFor(double seconds) {
//        ElapsedTime timer = new ElapsedTime();
//
//        timer.reset();
//        while (opModeIsActive() && timer.seconds() < seconds) {
//            updatePosition();
//            moveTowardTarget();
//        }
//
//        updatePosition();
//    }
//
//
//    public void goStraight(double power) {
////        powerMult = power;
////        goTowardTarget();
////        powerMult = 1;
//        powerWheels(applyCorrection((new double[] {power, power, power, power})));
//    }
//
//    public void slide(double power) {
//        powerWheels(applyCorrection((new double[] {power, -power, -power, power})));
//    }
//}

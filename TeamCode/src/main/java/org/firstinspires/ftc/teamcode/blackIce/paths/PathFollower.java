//package org.firstinspires.ftc.teamcode.blackIce.paths;
//
//import org.firstinspires.ftc.teamcode.blackIce.Condition;
//import org.firstinspires.ftc.teamcode.blackIce.Constants;
//import org.firstinspires.ftc.teamcode.blackIce.Drive;
//import org.firstinspires.ftc.teamcode.blackIce.Follower;
//import org.firstinspires.ftc.teamcode.blackIce.movement.Movement;
//import org.firstinspires.ftc.teamcode.blackIce.Target;
//import org.firstinspires.ftc.teamcode.blackIce.odometry.Odometry;
//import org.firstinspires.ftc.teamcode.util.Util;
//
//public class PathFollower extends Follower<PathFollower> {
//    public Path path;
//
//    public PathFollower(Path path) {
//        this.path = path;
//    }
//
//    private int i = 1;
//    private boolean holding = false;
//    private boolean finished = false;
//
//    @Override
//    public PathFollower start() {
//        i = 1;
//        holding = false;
//        finished = false;
//
//        Target.setTarget(Target.previousHeading, this.path.points[0][0], this.path.points[0][1]); // ?
//
//        return this;
//    }
//
//    @Override
//    public void update() {
//        if (holding) {
//            stopAtEndPosition.update();
//            if (stopAtEndPosition.isFinished()) {
//                finished = true;
//            }
//            return;
//        }
//
//        Target.updatePosition();
//
//        double[] point = this.path.points[i];
//
//        Follower.telemetry.addData("i", i);
//        Follower.telemetry.update();
//
//        double inchesLeftOnPath = (this.path.points.length - i) * Constants.Curve.INCHES_PER_POINT;
//        if (inchesLeftOnPath < Util.getVectorMagnitude(
//            Odometry.xBrakingDistance, Odometry.yBrakingDistance) + 2) {
//            stopAndHoldEndPoint();
//            return;
//        }
//
////        double targetHeading;
////        if (this.path.isConstantHeading) {
////            targetHeading = this.path.constantHeading;
////        } else {
////            double[] pointHeading;
////            if (i + Constants.Curve.LOOK_AHEAD_POINTS_FOR_HEADING >= this.path.points.length) {
////                pointHeading = this.path.endPoint;
////            } else {
////                pointHeading = this.path.points[i + Constants.Curve.LOOK_AHEAD_POINTS_FOR_HEADING];
////            }
////
////            targetHeading = Math.toDegrees(
////                Math.atan2(pointHeading[1] - point[1], pointHeading[0] - point[0])
////            ) + this.path.headingOffset;
////        }
//
//        new Movement(point[0], point[1], this.path.headings[i])
//            .copyProperties(this)
//            .moveThrough()
//            .waitForMovement();
//
//        this.i += 1;
//        if (i >= this.path.points.length) {
//            stopAndHoldEndPoint();
//        }
//    }
//
//    @Override
//    protected PathFollower getThis() {
//        return this;
//    }
//
//    private Movement stopAtEndPosition;
//
//    private void stopAndHoldEndPoint() {
//        stopAtEndPosition = new Movement(this.path.endPoint[0], this.path.endPoint[1], this.path.endingHeading)
//            .copyProperties(this)
//            .stopAtPosition();
//        stopAtEndPosition.start();
//
//        holding = true;
//    }
//
//    /**
//     * Wait for the Movement to be completed with an extraCondition .
//     *
//     * @param extraCondition False will continue holding the position,
//     *                       True will allow exit
//     * @param loopMethod A function gets called every loop. This is useful for things like
//     *                       when a linear slide reaches a certain height, a claw opens.
//     * <p>
//     * If no loop method and no extra condition is needed see {@link Movement#waitForMovement()}
//     */
//    @Override
//    public void waitForMovement(Condition extraCondition, Runnable loopMethod) {
//        start();
//
//        while (!finished && extraCondition.condition()) {
//            update();
//            loopMethod.run();
//        }
//
//        Drive.zeroPowerBrakeMode();
//        Drive.zeroPower();
//    }
//
//    @Override
//    public boolean isFinished() {
//        return this.finished;
//    }
//}

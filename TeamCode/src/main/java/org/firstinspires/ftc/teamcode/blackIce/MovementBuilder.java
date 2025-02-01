package org.firstinspires.ftc.teamcode.blackIce;

import org.firstinspires.ftc.teamcode.Robot;


public class MovementBuilder {
    Robot robot;
    public TargetTracker target;
    public HeadingCorrections headingCorrections;
    public DriveCorrections driveCorrections;

    public MovementBuilder(Robot robot) {
        this.robot = robot;
        this.target = new TargetTracker(robot);
        this.headingCorrections = new HeadingCorrections(robot);
        this.driveCorrections = new DriveCorrections(robot);
    }

    public MovementBuild buildMovement(double heading, double x, double y) {
        return new MovementBuild(this, x, y, heading);
    }

    public void moveThrough(double heading, double x, double y) {
        buildMovement(heading, x, y).moveThrough().run(); // make error margin larger
    }

    public void moveTo(double heading, double x, double y, double brakingPercent) {
        buildMovement(heading, x, y).moveTo(brakingPercent).run();
    }

    public void stopAtPosition(double heading, double x, double y) {
        buildMovement(heading, x, y).stopAtPosition().run();
    }

    public void turnAndMoveThrough(double heading, double x, double y) {
        buildMovement(heading, x, y).turnAndMoveThrough().run();
    }


    // TO PREVENT ERRORS
    public void quickBrakeTo(double heading, double x, double y, double vel) {

    }

    public void stopAtPositionPI(double heading, double x, double y) {

    }

    public void backIntoWall(double x) {

    }

    public void movePast(double heading, double x, double y) {

    }

    public void moveTo(double heading, double x, double y) {

    }
}



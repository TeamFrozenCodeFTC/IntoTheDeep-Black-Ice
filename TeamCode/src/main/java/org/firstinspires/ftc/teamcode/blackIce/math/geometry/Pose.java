package org.firstinspires.ftc.teamcode.blackIce.math.geometry;

/**
 *
 */
public class Pose {
    private final Vector position;
    private final double heading;

    public Pose(Vector position, double heading) {
        this.position = position;
        this.heading = heading;
    }
    
    public Pose(double x, double y, double heading) {
        this(new Vector(x, y), heading);
    }
    
    public Vector getPosition() {
        return position;
    }
    
    public double getHeading() {
        return heading;
    }

    public Pose withHeading(double heading) {
        return new Pose(position, heading);
    }
    public Pose withPosition(Vector position) {
        return new Pose(position, heading);
    }
    public Pose withX(double x) {
        return new Pose(position.withX(x), heading);
    }
    public Pose withY(double y) {
        return new Pose(position.withY(y), heading);
    }
}

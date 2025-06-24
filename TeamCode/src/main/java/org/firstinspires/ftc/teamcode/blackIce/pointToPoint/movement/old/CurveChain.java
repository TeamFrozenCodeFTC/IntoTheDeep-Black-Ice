//package org.firstinspires.ftc.teamcode.blackIce.paths;
//
//import org.firstinspires.ftc.teamcode.blackIce.geometry.Vector;
//
//public class CurveChain implements Curve {
//    private final Curve[] curves;
//    private final double length;
//    private final Curve lastCurve;
//
//    /**
//     * Note: PathChain set methods overrides custom Path's configurations unless
//     */
//    public CurveChain(Curve... curves) {
//        this.curves = curves;
//        this.lastCurve = curves[curves.length - 1];
//        // copy properties from PathChain. wait pathChain
//        // doesn't have any properties until after init
//
//        double length = 0;
//        for (Curve curve : curves) {
//            length += curve.length();
//        }
//        this.length = length;
//    }
//
//    // TODO vvv
//
//    private Curve getCurrentCurve(double t) {
//        return curves[getCurveIndex(t)];
//    }
//
//    public double getLength() {
//        return length;
//    }
//
//    public int getCurveIndex(double t) {
//        return (int) (t * this.curves.length);
//    }
//
//    private double globalToLocalT(double t, int index) {
//        double accumulated = 0;
//        for (int i = 0; i < index; i++) {
//            accumulated += curves[i].length();
//        }
//        double localT = (t * this.length - accumulated) / curves[index].length();
//        return Curve.clampT(localT);
//    }
//
//    @Override
//    public CurvePoint calculateClosestPointTo(Vector robotPosition, Path path) {
//        return curves[path.curveIndex]
//            .calculateClosestPointTo(robotPosition, path);
//    }
//
//    @Override
//    public Vector calculatePointAt(double t, Path path) {
//        return curves[path.curveIndex]
//            .calculatePointAt(globalToLocalT(t, path.curveIndex), path);
//    }
//
//    @Override
//    public double length() {
//        return length;
//    }
//
//    @Override
//    public Vector getEndPoint() {
//        return lastCurve.getEndPoint();
//    }
//
//    @Override
//    public Vector getEndTangent() {
//        return lastCurve.getEndTangent();
//    }
//
//    @Override
//    public Type getCurveType() {
//        return Curve.Type.CHAIN;
//    }
//}

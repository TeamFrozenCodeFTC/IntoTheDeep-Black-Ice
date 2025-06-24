package org.firstinspires.ftc.teamcode.blackIce.robot.wheelPowers;

import org.firstinspires.ftc.teamcode.blackIce.robot.OperableComponents;

public abstract class WheelPowers
    <Subclass extends WheelPowers<Subclass>>
    extends OperableComponents<Subclass>
{
    protected WheelPowers(double... components) {
        super(components);
    }
    
    public double getMaxAbs() {
        double max = 0;
        for (double component : components) {
            max = Math.max(max, Math.abs(component));
        }
        return max;
    }

    /**
     * Add the given powers together,
     * but only add the powers if they are in the same direction and working together.
     */
    public Subclass addAlignedPowers(Subclass other) {
        ComponentPairOperator operator = (power1, power2) -> {
            double sign1 = Math.signum(power1);
            if (sign1 == 0) return power2;
            boolean isAligned =
                Math.signum(power1) == Math.signum(power2) || Math.abs(power1 - power2) < 0.5;
            if (isAligned) {
                return power1 + power2;
            } else {
                return power1;
            }
        };
        return map(other, operator);
    }
    
    /**
     * Scale the given powers until at least one of them is equal to the given max power.
     *
     * <pre><code>
     * scaleToMax({10, -5, -10, 1}, 1) -> {1, -0.5, -1, 0.1}
     * scaleToMax({-0.5, 0.25, 0.75, -0.25}, 1) -> {-0.66, 0.33, 1, -0.33}
     * </code></pre>
     *
     * @param newMax The power value this method is scaling to. Must be positive.
     */
    public Subclass scaleMaxTo(double newMax) {
        double currentMax = getMaxAbs();
        if (currentMax == 0) return getThis();
        return this.times(newMax / currentMax);
    }
    
    /**
     * Scale the given powers down so that
     * at least one of them is less than or equal to the given max power.
     *
     * <pre><code>
     * {10, -5, -10, 1}.downscaleToMax(1) -> {1, -0.5, -1, 0.1}
     * {-0.5, 0.25, 0.75, -0.25}.downscaleToMax(1) -> {-0.5, 0.25, 0.75, -0.25}
     * // Does not change the values because all of them are less than 1.
     * </code></pre>
     *
     * @param max The power value this method is scaling to. Must be positive.
     */
    public Subclass downscaleMaxTo(double max) {
        double currentMax = getMaxAbs();
        if (currentMax <= max) return getThis();
        return this.times(max / currentMax);
    }
    
    /**
     * Scale the powers so that the maximum value is equal to 1.
     *
     * <pre><code>
     * normalize({2, -4, 2, -1}) -> {0.5, -1, 0.5, -0.25}
     * normalize({-0.5, 0.25, 0, 0.5}) -> {-1, 0.5, 0, 1}
     * </code></pre>
     */
    public Subclass normalize() {
        return scaleMaxTo(1);
    }
    
    /**
     * Scale the powers so that the maximum value is less than or equal to 1.
     *
     * <pre><code>
     * normalizeDown({2, -4, 2, -1}) -> {0.5, -1, 0.5, -0.25}
     * normalizeDown({-0.5, 0.25, 0, 0.5}) -> {-0.5, 0.25, 0, 0.5}
     * </code></pre>
     *
     * Different from {@link DrivePowers#normalize} because this method does not upscale the powers
     * if the max value is less than 1.
     */
    public Subclass normalizeDown() {
        return downscaleMaxTo(1);
    }
    
    @SuppressWarnings("unchecked")
    private Subclass getThis() {
        return (Subclass) this;
    }
    
    public double[] getPowers() {
        return components.clone();
    }
}
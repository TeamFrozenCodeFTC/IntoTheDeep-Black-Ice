package org.firstinspires.ftc.teamcode.blackIce.paths;

import org.firstinspires.ftc.teamcode.blackIce.movement.MovementBuild;

/**
 * A external way to add properties to a wrapping class of Movements.
 * <p>
 * Example:
 * <pre><code>
 *     path.pathMovementProperties.setProperties((movement) -> movement.setMaxVelocity(...)))}
 * </code></pre>
 */
public interface MovementProperties {
    MovementBuild setProperties(MovementBuild build);
}

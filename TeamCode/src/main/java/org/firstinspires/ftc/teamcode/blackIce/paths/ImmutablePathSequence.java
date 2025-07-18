package org.firstinspires.ftc.teamcode.blackIce.paths;

import org.firstinspires.ftc.teamcode.blackIce.math.geometry.Pose;
import org.firstinspires.ftc.teamcode.blackIce.util.Advancer;

import java.util.Arrays;

public class ImmutablePathSequence {
    private final Path[] paths;
    public final Pose endPose;
    
    public ImmutablePathSequence(Path... paths) {
        this.paths = paths.clone();
        this.endPose = this.paths[this.paths.length - 1].endPose;
    }
    
    public static ImmutablePathSequence empty() {
        return new ImmutablePathSequence();
    }
    
    public ImmutablePathSequence withPath(Path path) {
        Path[] newList = Arrays.copyOf(paths, paths.length + 1);
        newList[paths.length] = path;
        return new ImmutablePathSequence(newList);
    }
    
    public ImmutablePathSequence withSequence(ImmutablePathSequence other) {
        Path[] newList = Arrays.copyOf(paths, paths.length + other.paths.length);
        System.arraycopy(other.paths, 0, newList, paths.length, other.paths.length);
        return new ImmutablePathSequence(newList);
    }
    
    public Advancer<Path> advancer() {
        return new Advancer<>(paths);
    }
    
    public Path[] getPaths() {
        return paths.clone();
    }
    
    public int size() {
        return paths.length;
    }
}

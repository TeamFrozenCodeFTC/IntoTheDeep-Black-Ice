package org.firstinspires.ftc.teamcode.blackIce.pointToPoint.movement;

//public interface Function<ArgumentType, ReturnType> {
//    ReturnType run(ArgumentType x);
//}

@FunctionalInterface
public interface Function<ArgumentType, ReturnType> {
    ReturnType run(ArgumentType x);
}

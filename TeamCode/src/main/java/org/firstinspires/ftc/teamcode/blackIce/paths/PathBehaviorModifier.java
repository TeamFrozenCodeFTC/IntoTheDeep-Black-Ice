package org.firstinspires.ftc.teamcode.blackIce.paths;

@FunctionalInterface
@Deprecated
public interface PathBehaviorModifier {
    void configure(Path path);
    
    default PathBehaviorModifier combine(PathBehaviorModifier configurer) {
        return path -> {
            this.configure(path);
            configurer.configure(path);
        };
    }
}

//follower.addDefaultPathSettings(p -> {
//    if (p.getName().contains("AutoPark")) {
//    p.setTargetVelocity(30);
//    }
//        });
package org.firstinspires.ftc.teamcode.blackIce.paths;

@FunctionalInterface
public interface PathBehavior {
    void configure(Path path);
    
    default PathBehavior combine(PathBehavior configurer) {
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
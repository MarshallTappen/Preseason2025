package frc.robot.subsystems.vision;

import java.util.Optional;

public interface TargetDetectorInterface {
    
    public static record Detection(double tx, double ty, double estimatedDistance) {};

    public Optional<Detection> getClosestVisibleTarget();

}

package frc.robot.commands.vision;

import java.util.Optional;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.SwerveTeleopCommand;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.vision.TargetDetectorInterface;
import frc.robot.subsystems.vision.TargetDetectorInterface.Detection;

public class AimAndDriveBase extends Command {

    private final TargetDetectorInterface targetDetector;
    private final SwerveSubsystem swerveSubsystem;
    private final PIDController pidController;
    private final double maxRotVelocity;
    private final XboxController driverController;

    public AimAndDriveBase(TargetDetectorInterface targetDetector, SwerveSubsystem swerveSubsystem,
            XboxController driverController,
            double kP, double maxRotVelocity) {

        this.targetDetector = targetDetector;
        this.swerveSubsystem = swerveSubsystem;
        this.pidController = new PIDController(kP, 0, 0);
        this.maxRotVelocity = maxRotVelocity;
        this.driverController = driverController;
    }

    @Override
    public void execute() {

        ChassisSpeeds cs = SwerveTeleopCommand.computeVelocitiesFromController(driverController);
        Optional<Detection> detection = targetDetector.getClosestVisibleTarget();

        // Set the rotational velocity to zero because it will be replaced later.
        // If there is no detection, it will drive forward
        cs.omegaRadiansPerSecond = 0;

        if(detection.isPresent()) {

            double tx;

            tx = detection.get().tx();
            double angleVelocity = pidController.calculate(tx, 0);
            angleVelocity = MathUtil.clamp(angleVelocity, -this.maxRotVelocity, this.maxRotVelocity);
            SmartDashboard.putNumber("angleVelocity", angleVelocity);
            double inputRotVelocity = angleVelocity / 180 * Math.PI;
            cs.omegaRadiansPerSecond = inputRotVelocity;
        }

        swerveSubsystem.drive(cs);
    }
}

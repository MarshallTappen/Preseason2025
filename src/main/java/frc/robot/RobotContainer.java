// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.SwerveTeleopCommand;
import frc.robot.commands.vision.AimAndDriveBase;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.vision.NoteLocatorSim;
import frc.robot.subsystems.vision.TargetDetectorInterface;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public final SwerveSubsystem swerveSubsystem = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
      "swerve/neo"));

  private final CommandXboxController driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);

  public final TargetDetectorInterface noteDetector;

  public final SwerveTeleopCommand swerveTeleopCommand = new SwerveTeleopCommand(swerveSubsystem, driverController);
  // Replace with CommandPS4Controller or CommandJoystick if needed

  private final SendableChooser<Command> autoChooser;
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    if (RobotBase.isSimulation()) {
      noteDetector = new NoteLocatorSim(swerveSubsystem);
    } else {
      noteDetector = null;
    }

    // Configure the trigger bindings
    configureBindings();

    // Need to initialize this here after vision is configured.
    // Need to clean up initialization flow to make it more clear
    autoChooser = buildAutoChooser();

  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    driverController.a().whileTrue(new AimAndDriveBase(noteDetector, swerveSubsystem,
        driverController.getHID(), 16, 360));

    swerveSubsystem.setDefaultCommand(swerveTeleopCommand);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    // return Autos.driveToNote(swerveSubsystem, noteDetector);
    return autoChooser.getSelected();

  }

  private SendableChooser<Command> buildAutoChooser() {
    SendableChooser<Command> chooser = new SendableChooser<Command>();

    chooser.setDefaultOption("None", Commands.none());
    chooser.addOption("DriveToNote", Autos.driveToNote(swerveSubsystem, noteDetector));
    SmartDashboard.putData("Auto Chooser", chooser);

    return chooser;
  }

}

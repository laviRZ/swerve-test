package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class FieldHeadingDriveCommand extends CommandBase {

  private final DoubleSupplier xSupplier;
  private final DoubleSupplier ySupplier;
  private final DoubleSupplier omegaXSupplier;
  private final DoubleSupplier omegaYSupplier;
  private final DrivetrainSubsystem drivetrainSubsystem;

  private final ProfiledPIDController thetaController;

  public FieldHeadingDriveCommand(
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaXSupplier,
      DoubleSupplier omegaYSupplier,
      DrivetrainSubsystem drivetrainSubsystem) {

    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    this.omegaXSupplier = omegaXSupplier;
    this.omegaYSupplier = omegaYSupplier;
    this.drivetrainSubsystem = drivetrainSubsystem;

    // FIXME set theta constraints
    TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(Math.PI, Math.PI);
    
    // FIXME set theta PID values
    thetaController = new ProfiledPIDController(1, 0, 0, kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(drivetrainSubsystem);
  }

  @Override
  public void initialize() {
    thetaController.reset(drivetrainSubsystem.getGyroscopeRotation().getRadians());
  }

  @Override
  public void execute() {
    
    double heading;
    if (omegaXSupplier.getAsDouble() == 0 && omegaYSupplier.getAsDouble() == 0) {
      // Hold heading when stick is centered
      heading = drivetrainSubsystem.getGyroscopeRotation().getRadians();
    } else {
      // Use X and Y and calculate an angular heading
      heading = Math.atan2(omegaYSupplier.getAsDouble(), omegaXSupplier.getAsDouble()) + (Math.PI / 2.0);
      heading = Math.IEEEremainder(heading, 2*Math.PI);
    }
    SmartDashboard.putNumber("Heading", Units.radiansToDegrees(heading));

    var omega = thetaController.calculate(drivetrainSubsystem.getGyroscopeRotation().getRadians(), heading);

    drivetrainSubsystem.drive(new ChassisSpeeds(xSupplier.getAsDouble(), ySupplier.getAsDouble(), omega));
  }

  @Override
  public void end(boolean interrupted) {
    drivetrainSubsystem.stop();
  }
  
}
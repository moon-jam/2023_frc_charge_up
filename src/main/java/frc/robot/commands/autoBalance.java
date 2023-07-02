package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.intake_position;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.RackSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class autoBalance extends CommandBase {
  private final SwerveSubsystem swerve;
  RackSubsystem rack;
  IntakeSubsystem intake;
  double pitch_kP = 0.08, roll_kP = 0.05, rack_kP = 0.03;
  double xSpeed = 3, ySpeed = 3;
  boolean left_ground = false;
  boolean isFinished = false;
  SlewRateLimiter filter_x = new SlewRateLimiter(2);
  SlewRateLimiter filter_y = new SlewRateLimiter(2);

  /**
   * Creates a new autoBalance.
   *
   * @param subsystem The subsystem used by this command.
   */
  public autoBalance(SwerveSubsystem swerve, RackSubsystem rack, IntakeSubsystem intake) {
    this.swerve = swerve;
    this.rack = rack;
    this.intake = intake;

    addRequirements(swerve, intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putNumber("autoBlance pitch_kP", pitch_kP);
    SmartDashboard.putNumber("autoBlance roll_kP", roll_kP);
    left_ground = false;
    isFinished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // pitch_kP = SmartDashboard.getNumber("autoBlance pitch_kP", 0);
    // roll_kP = SmartDashboard.getNumber("autoBlance roll_kP", 0);
    double Pitch_error = -swerve.getGyroPitch();
    double Roll_error = swerve.getGyroRoll();
    // if(Math.abs(Pitch_error)<14.3) Pitch_error = 0;
    // if(Math.abs(Roll_error)<10 && !left_ground) Roll_error = 30;
    // else if(Math.abs(Roll_error)<14.3) {
    //   left_ground = true;
    // }
    if(Roll_error < 14) {
      rack.rack_open();
      // intake.set_lift_floor();
    }
    if(Roll_error <3){
      Roll_error = 0;
    }
     if(Pitch_error <3){
      Pitch_error = 0;
    }

    ySpeed = filter_y.calculate(Pitch_error * pitch_kP);
    xSpeed = filter_x.calculate(Roll_error * roll_kP);

    // SmartDashboard.putNumber("autoBlance pitch", Pitch_error);
    SmartDashboard.putNumber("autoBlance roll", Roll_error);
    
    xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0.0;
    ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? ySpeed : 0.0;

    SmartDashboard.putNumber("Xspeed", xSpeed);

    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xSpeed, 0, 0);
    SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
    swerve.setModuleStates(moduleStates);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.stopModules();
    intake.lift_stop();
    rack.rack_stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.intake_position;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.RackSubsystem;

public class autoEat extends CommandBase {

  IntakeSubsystem intake;
  ElevatorSubsystem elevator;
  RackSubsystem rack;
  boolean isFinished = false;
  eatMode mode;

  /**
   * Creates a new autoEat.t
   *
   * @param subsystem The subsystem used by this command.
   */
  public autoEat(IntakeSubsystem intake, ElevatorSubsystem elevator, RackSubsystem rack, eatMode mode) {
    this.intake = intake;
    this.elevator = elevator;
    this.rack = rack;
    this.mode = mode;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isFinished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    rack.rack_close();
    if (mode == eatMode.double_substation_con) {
      elevator.elevator_up();
      intake.set_lift_straight();
      intake.eatCon();
    }
    if (mode == eatMode.single_substation_cube) {
      elevator.elevator_down();
      intake.set_lift_position(intake_position.intake_substation_cube);
      intake.eatCube();
    }
    if (mode == eatMode.floor_con) {
      elevator.elevator_down();
      intake.set_lift_floor();
      intake.eatCon();
    }
    if (mode == eatMode.floor_cube) {
      elevator.elevator_down();
      intake.set_lift_floor();
      intake.eatCube();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.lift_stop();
    intake.suckStop();
    elevator.elevator_stop();
  }

  public static enum eatMode {
    single_substation_cube,
    double_substation_con,
    floor_cube,
    floor_con;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return isFinished;
  }

  public void stop() {
    isFinished = true;
  }
}

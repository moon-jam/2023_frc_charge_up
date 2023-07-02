package frc.robot.commands;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.intake_position;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.RackSubsystem;

public class autoPut extends CommandBase {

    IntakeSubsystem intake;
    RackSubsystem rack;
    ElevatorSubsystem elevator;
    putMode Mode;
    boolean isFinished = false;
    Timer timer = new Timer();

    /**
     * Creates a new autoPut.
     *
     * @param subsystem The subsystem used by this command.
     */
    public autoPut(IntakeSubsystem intake, RackSubsystem rack, ElevatorSubsystem elevator, putMode Mode) {
        this.intake = intake;
        this.rack = rack;
        this.elevator = elevator;
        this.Mode = Mode;

        addRequirements(intake, rack, elevator);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        isFinished = false;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (Mode == putMode.down_cube) {
            elevator.elevator_down();
            rack.rack_close();
            intake.set_lift_straight();
            if (elevator.elevator_down() && rack.rack_close()
                    && intake.set_lift_straight()) {
                rack.rack_stay_close();
                intake.putCube();
            }
        }
        if (Mode == putMode.down_con) {
            elevator.elevator_down();
            rack.rack_close();
            intake.set_lift_straight();
            if (elevator.elevator_down() && rack.rack_close()
                    && intake.set_lift_straight()) {
                rack.rack_stay_close();
                intake.putCon();
            }
        }

        if (Mode == putMode.middle_cube) {
            elevator.elevator_middle();
            rack.rack_close();
            if (elevator.elevator_middle() && rack.rack_close()
                    && intake.set_lift_straight()) {
                rack.rack_stay_close();
                intake.putCube();
            }
        }
        if (Mode == putMode.middle_con) {
            elevator.elevator_middle();
            rack.rack_close();
            if (elevator.elevator_middle() && rack.rack_close()
                    && intake.set_lift_straight()) {
                rack.rack_stay_close();
                intake.putCon();
            }
        }

        if (Mode == putMode.upper_cube) {
            elevator.elevator_up();
            rack.rack_open();
            // intake.set_lift_straight();x
            if (elevator.elevator_up() && rack.rack_open()) {
                if(Math.abs(intake.getLiftPosition()-intake_position.intake_substation_cube.value)>200)
                    intake.set_lift_position(intake_position.intake_substation_cube);
                else if(Math.abs(intake.getLiftPosition()-intake_position.intake_straight.value)>100)
                    intake.set_lift_position(intake_position.intake_straight);
                else
                    intake.putCon();
            }
        }
        if (Mode == putMode.upper_con) {
            elevator.elevator_up();
            rack.rack_open();
            // intake.set_lift_straight();x
            if (elevator.elevator_up() && rack.rack_open()) {
                intake.set_lift_position(intake_position.intake_straight_con);
            }
        }
        if (Mode == putMode.reset) {
            intake.set_lift_position(intake_position.intake_origin);
            rack.rack_close();
            if (intake.getLiftPosition()<150 && rack.rack_close() && elevator.elevator_down()) {
                rack.rack_stay_close();
                elevator.elevator_down();
            }
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        intake.lift_stop();
        // intake.suckStop();
        if (Mode == putMode.reset)
            rack.rack_stay_close();
        else
            rack.rack_stop();
        elevator.elevator_stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return isFinished;
    }

    public static enum putMode {
        down_cube, down_con, middle_cube,
        middle_con, upper_cube, upper_con,
        reset;
    }

    public void stop() {
        isFinished = true;
    }
}

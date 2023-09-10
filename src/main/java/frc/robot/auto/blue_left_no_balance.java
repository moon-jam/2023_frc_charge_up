package frc.robot.auto;

import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.robot.commands.aimViewer;
import frc.robot.commands.autoAim;
import frc.robot.commands.autoBalance;
import frc.robot.commands.autoEat;
import frc.robot.commands.autoPut;
import frc.robot.commands.autoEat.eatMode;
import frc.robot.commands.autoPut.putMode;
import frc.robot.modules.PathPlannerModule;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.RackSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.Constants.intake_position;

public class blue_left_no_balance extends PathPlannerModule {

    IntakeSubsystem intake;
    RackSubsystem rack;

    autoAim AimCube;
    autoAim AimAprilTag;
    // aimViewer aim_Viewer;
    autoEat autoEatCube;
    autoPut Put_up_con;
    autoPut Put_up_cube;
    autoPut reset;
    autoPut reset2;
    autoPut reset3;
    autoBalance AutoBalance;

    Timer timer = new Timer();

    public blue_left_no_balance(SwerveSubsystem swerve, IntakeSubsystem intake, RackSubsystem rack,
            ElevatorSubsystem elevator, Limelight limelight) {
        super(swerve);
        this.intake = intake;
        this.rack = rack;

        AimCube = new autoAim(swerve, limelight, autoAim.Mode.floor, autoAim.subMode.cube);
        AimAprilTag = new autoAim(swerve, limelight, autoAim.Mode.grid, autoAim.subMode.cube);
        // aim_Viewer = new aimViewer(limelight);
        autoEatCube = new autoEat(intake, elevator, rack, eatMode.floor_cube);
        AutoBalance = new autoBalance(swerve, rack, intake);
        reset = new autoPut(intake, rack, elevator, putMode.reset);
        reset2 = new autoPut(intake, rack, elevator, putMode.reset);
        reset3 = new autoPut(intake, rack, elevator, putMode.reset);
        Put_up_con = new autoPut(intake, rack, elevator, putMode.upper_con);
        Put_up_cube = new autoPut(intake, rack, elevator, putMode.upper_cube);

        List<PathPlannerTrajectory> trajectory = PathPlanner.loadPathGroup("blue_left_no_balance",
                new PathConstraints(4, 3),
                new PathConstraints(4, 3),
                new PathConstraints(4, 3));
        PathPlannerTrajectory traj1 = trajectory.get(0);
        PathPlannerTrajectory traj2 = trajectory.get(1);
        PathPlannerTrajectory traj3 = trajectory.get(2);
        PPSwerveControllerCommand autoDrive1 = baseSwerveCommand(traj1);
        PPSwerveControllerCommand autoDrive2 = baseSwerveCommand(traj2);
        PPSwerveControllerCommand autoDrive3 = baseSwerveCommand(traj3);

        addCommands(new InstantCommand(() -> swerve.zeroHeading()),
                new InstantCommand(() -> swerve.gyro_pitch_offset = 180), 
                new InstantCommand(() -> swerve.resetOdometry(traj1.getInitialHolonomicPose())),
                // Put_up_con.withTimeout(3.5),
                // new InstantCommand(() -> this.intake.set_lift_position(intake_position.intake_straight_con)).withTimeout(1.5),
                // // new InstantCommand(() -> this.intake.suckStop()).withTimeout(1.5),
                // new InstantCommand(() -> this.intake.putCube()).withTimeout(4),
                // reset.withTimeout(4),
                // // new InstantCommand(() -> this.intake.suckStop()).withTimeout(1.5),
                // new InstantCommand(() -> this.intake.suckStop()),
                // autoDrive1.withTimeout(2.3),
                // AimCube.withTimeout(2),
                // new InstantCommand(() -> this.intake.eatCube()),
                // new RunCommand(() -> {
                //         if (this.intake.getLiftPosition() > 2500)
                //                 this.intake.lift.setPercentOutput(0.1);
                //         else if (this.intake.getLiftPosition() > 1000)
                //                 this.intake.lift.setPercentOutput(-0.3);
                //         else
                //                 this.intake.lift.setPercentOutput(-0.95);
                // }).withTimeout(2),
                // reset2.withTimeout(1),
                // autoDrive2,
                // AimAprilTag.withTimeout(2),
                // new RunCommand(() -> {
                //         this.intake.set_lit_position(intake_position.intake_straight);
                // }).withTimeout(1),
                // new InstantCommand(() -> this.intake.putCube()).withTimeout(1.5),
                // reset3.withTimeout(1),
                // autoDrive3.alongWith(reset3.withTimeout(2)),
                new InstantCommand(() -> swerve.stopModules()));
    }
}

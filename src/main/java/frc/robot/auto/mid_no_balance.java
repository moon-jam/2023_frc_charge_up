// package frc.robot.auto;

// import java.util.List;

// import com.pathplanner.lib.PathConstraints;
// import com.pathplanner.lib.PathPlanner;
// import com.pathplanner.lib.PathPlannerTrajectory;
// import com.pathplanner.lib.commands.PPSwerveControllerCommand;

// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.robot.commands.aimViewer;
// import frc.robot.commands.autoAim;
// import frc.robot.commands.autoBalance;
// import frc.robot.commands.autoEat;
// import frc.robot.commands.autoPut;
// import frc.robot.commands.autoEat.eatMode;
// import frc.robot.commands.autoPut.putMode;
// import frc.robot.modules.PathPlannerModule;
// import frc.robot.subsystems.ElevatorSubsystem;
// import frc.robot.subsystems.IntakeSubsystem;
// import frc.robot.subsystems.Limelight;
// import frc.robot.subsystems.RackSubsystem;
// import frc.robot.subsystems.SwerveSubsystem;

// public class mid_no_balance extends PathPlannerModule {

//     SwerveSubsystem swerve;
//     IntakeSubsystem intake;
//     RackSubsystem rack;

//     autoAim AimCube;
//     aimViewer aim_Viewer;
//     autoEat autoEatCube;
//     autoPut Put_up_con;
//     autoPut Put_up_con2;
//     autoPut reset;
//     autoPut reset2;
//     autoPut reset3;
//     autoBalance AutoBalance;

//     Timer timer = new Timer();

//     public mid_no_balance(SwerveSubsystem swerve, IntakeSubsystem intake, RackSubsystem rack,
//             ElevatorSubsystem elevator, Limelight limelight) {
//         super(swerve);
//         this.intake = intake;
//         this.rack = rack;

//         AimCube = new autoAim(swerve, limelight, autoAim.Mode.floor, autoAim.subMode.cube);
//         aim_Viewer = new aimViewer(limelight);
//         autoEatCube = new autoEat(intake, elevator, rack, eatMode.floor_cube);
//         AutoBalance = new autoBalance(swerve, rack, intake);
//         reset = new autoPut(intake, rack, elevator, putMode.reset);
//         reset2 = new autoPut(intake, rack, elevator, putMode.reset);
//         reset3 = new autoPut(intake, rack, elevator, putMode.reset);
//         Put_up_con = new autoPut(intake, rack, elevator, putMode.upper_con);
//         Put_up_con2 = new autoPut(intake, rack, elevator, putMode.upper_con);

//         List<PathPlannerTrajectory> trajectory = PathPlanner.loadPathGroup("middle_no_balance",
//                 new PathConstraints(4, 3),
//                 new PathConstraints(4, 3));
//         PathPlannerTrajectory traj1 = trajectory.get(0);
//         PathPlannerTrajectory traj2 = trajectory.get(1);
//         PPSwerveControllerCommand autoDrive1 = baseSwerveCommand(traj1);
//         PPSwerveControllerCommand autoDrive2 = baseSwerveCommand(traj2);

//         addCommands(new InstantCommand(() -> swerve.zeroHeading()),
//                 new InstantCommand(() -> swerve.resetOdometry(traj1.getInitialHolonomicPose())),
//                 Put_up_con.withTimeout(2),
//                 new InstantCommand(() -> this.intake.putCon()).withTimeout(1),
//                 autoDrive1.deadlineWith(reset.withTimeout(2.5)),
//                 // AimCube.until(() -> Math.abs(aim_Viewer.get_x_Error()) < 0.5 && Math.abs(aim_Viewer.get_y_Error()) < 0.5),
//                 autoEatCube.withTimeout(2),
//                 autoDrive2.deadlineWith(reset2.withTimeout(1)),
//                 Put_up_con2.withTimeout(2),
//                 new InstantCommand(() -> this.intake.putCube()).withTimeout(1),
//                 reset3,
//                 endCommand());
//     }
// }
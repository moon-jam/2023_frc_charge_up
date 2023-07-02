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

// public class blue_right_balance extends PathPlannerModule{

//     SwerveSubsystem swerve;
//     IntakeSubsystem intake;
//     RackSubsystem rack;

//     autoAim AimCube;
//     aimViewer aim_Viewer;
//     autoEat autoEatCube;
//     autoPut Put_up_con;
//     autoPut reset;
//     autoPut reset2;
//     autoBalance AutoBalance;

//     Timer timer = new Timer();

//     public blue_right_balance(SwerveSubsystem swerve, IntakeSubsystem intake, RackSubsystem rack,
//                 ElevatorSubsystem elevator, Limelight limelight) {
//         super(swerve);
//         this.intake = intake;
//         this.rack = rack;

//         AimCube = new autoAim(swerve, limelight, autoAim.Mode.floor, autoAim.subMode.cube);
//         aim_Viewer = new aimViewer(limelight);
//         autoEatCube = new autoEat(intake, elevator, rack, eatMode.floor_cube);
//         AutoBalance = new autoBalance(swerve, rack, intake);
//         reset = new autoPut(intake, rack, elevator, putMode.reset);
//         reset2 = new autoPut(intake, rack, elevator, putMode.reset);
//         Put_up_con = new autoPut(intake, rack, elevator, putMode.upper_con);

//         List<PathPlannerTrajectory> trajectory = PathPlanner.loadPathGroup("blue_right_balance",
//             new PathConstraints(4, 3),
//             new PathConstraints(4, 3),
//             new PathConstraints(5, 4));
//         PathPlannerTrajectory traj1 = trajectory.get(0);
//         PathPlannerTrajectory traj2 = trajectory.get(1);
//         PathPlannerTrajectory traj3 = trajectory.get(2);
//         PPSwerveControllerCommand autoDrive1 = baseSwerveCommand(traj1);
//         PPSwerveControllerCommand autoDrive2 = baseSwerveCommand(traj2);
//         PPSwerveControllerCommand autoDrive3 = baseSwerveCommand(traj3);

//         addCommands(new InstantCommand(() -> swerve.zeroHeading()),
//                         new InstantCommand(() -> swerve.resetOdometry(traj1.getInitialHolonomicPose())),
//                         Put_up_con.withTimeout(2),
//                         new InstantCommand(() -> this.intake.putCon()).withTimeout(1),
//                         autoDrive1.deadlineWith(reset.withTimeout(2.5)),
//                         // AimCube.until(()->Math.abs(aim_Viewer.get_x_Error())<0.5 && Math.abs(aim_Viewer.get_y_Error())<0.5),
//                         autoEatCube.withTimeout(2),
//                         // reset2.withTimeout(0.5),
//                         autoDrive2.deadlineWith(reset2.withTimeout(1)),
//                         autoDrive3.deadlineWith(new InstantCommand(()->{this.rack.rack_open();})),
//                         new InstantCommand(()->rack.rack_stop()),
//                         endCommand());
//     }
// }

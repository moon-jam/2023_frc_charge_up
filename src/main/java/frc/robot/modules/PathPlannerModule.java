package frc.robot.modules;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * Parent class for all autonomous commands
 */
public class PathPlannerModule extends SequentialCommandGroup {
    public static SwerveSubsystem swerve;
    public static final ProfiledPIDController profiledthetaController =
        new ProfiledPIDController(0, 0, 0,
            Constants.AutoConstants.kThetaControllerConstraints);
    public static final PIDController thetaController =
        new PIDController(0, 0, 0);

    /**
     * Autonomous that aligns limelight then executes a trajectory.
     *
     * @param swerve swerve subsystem
     */
    public PathPlannerModule(SwerveSubsystem swerve) {
        PathPlannerModule.swerve = swerve;
        addRequirements(swerve);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
    }

    /**
     * Creates a SwerveControllerCommand from a Trajectory
     *
     * @param trajectory Trajectory to run
     * @return A SwerveControllerCommand for the robot to move
     */
    public static SwerveControllerCommand baseSwerveCommand(Trajectory trajectory) {
        SwerveControllerCommand command = new SwerveControllerCommand(trajectory, swerve::getPose,
            Constants.DriveConstants.kDriveKinematics,
            new PIDController(0, 0, 0),
            new PIDController(0, 0, 0), profiledthetaController,
            swerve::setModuleStates, swerve);
        return command;
    }

    /**
     * Creates a SwerveController Command using a Path Planner Trajectory
     *
     * @param trajectory a Path Planner Trajectory
     * @return A SwerveControllerCommand for the robot to move
     */
    public static PPSwerveControllerCommand baseSwerveCommand(PathPlannerTrajectory trajectory) {
        PPSwerveControllerCommand command = new PPSwerveControllerCommand(trajectory,
            swerve::getPose, Constants.DriveConstants.kDriveKinematics,
            new PIDController(0, 0, 0),
            new PIDController(0, 0, 0), thetaController,
            swerve::setModuleStates, true, swerve);
        return command;
    }

    public static Command endCommand(){
       return new SequentialCommandGroup(    
        new InstantCommand(() -> swerve.stopModules()));
    
    }
}
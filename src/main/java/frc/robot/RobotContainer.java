
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.List;
import java.util.spi.AbstractResourceBundleProvider;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import com.pathplanner.lib.PathConstraints;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.intake_position;
import frc.robot.Constants.limelight_pipeline;
// import frc.robot.auto.blue_left_balance;
import frc.robot.auto.blue_left_no_balance;
// import frc.robot.auto.mid_balance;
// import frc.robot.auto.mid_no_balance;
// import frc.robot.auto.blue_right_balance;
// import frc.robot.auto.blue_right_no_balance;
import frc.robot.commands.SwerveJoystick_Cmd;
// import frc.robot.commands.aimViewer;
import frc.robot.commands.autoAim;
import frc.robot.commands.autoBalance;
import frc.robot.commands.autoEat;
import frc.robot.commands.autoPut;
import frc.robot.modules.PathPlannerModule;
import frc.robot.modules.LimelightModule.LightMode;
// import frc.robot.commands.autoAim;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.RackSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...

    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
    public final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    public final RackSubsystem rackSubsystem = new RackSubsystem();
    private final Limelight limelight = new Limelight();
    PathPlannerModule pathPlanner = new PathPlannerModule(swerveSubsystem);
    private final autoEat eat_double_substation_con = new autoEat(intakeSubsystem, elevatorSubsystem, rackSubsystem,
            autoEat.eatMode.double_substation_con);
    private final autoEat eat_single_substation_cube = new autoEat(intakeSubsystem, elevatorSubsystem, rackSubsystem,
            autoEat.eatMode.single_substation_cube);
    private final autoEat eat_floor_con = new autoEat(intakeSubsystem, elevatorSubsystem, rackSubsystem,
            autoEat.eatMode.floor_cube);
    private final autoEat eat_floor_cube = new autoEat(intakeSubsystem, elevatorSubsystem, rackSubsystem,
            autoEat.eatMode.floor_con);
    private final autoPut putDown_Con = new autoPut(intakeSubsystem, rackSubsystem, elevatorSubsystem,
            autoPut.putMode.down_con);
    private final autoPut putDown_Cube = new autoPut(intakeSubsystem, rackSubsystem, elevatorSubsystem,
            autoPut.putMode.down_cube);
    private final autoPut putMiddle_Con = new autoPut(intakeSubsystem, rackSubsystem, elevatorSubsystem,
            autoPut.putMode.middle_con);
    private final autoPut putMiddle_Cube = new autoPut(intakeSubsystem, rackSubsystem, elevatorSubsystem,
            autoPut.putMode.middle_cube);
    private final autoPut putUp_Con = new autoPut(intakeSubsystem, rackSubsystem, elevatorSubsystem,
            autoPut.putMode.upper_con);
    private final autoPut putUp_Cube = new autoPut(intakeSubsystem, rackSubsystem, elevatorSubsystem,
            autoPut.putMode.upper_cube);
    private final autoPut putReset = new autoPut(intakeSubsystem, rackSubsystem, elevatorSubsystem,
            autoPut.putMode.reset);
    private final autoAim Aim_floor_con = new autoAim(swerveSubsystem, limelight, autoAim.Mode.floor,
            autoAim.subMode.con);
    private final autoAim Aim_grid_con = new autoAim(swerveSubsystem, limelight, autoAim.Mode.grid,
            autoAim.subMode.con);
    private final autoAim Aim_floor_cube = new autoAim(swerveSubsystem, limelight, autoAim.Mode.floor,
            autoAim.subMode.cube);
    private final autoAim Aim_grid_cube = new autoAim(swerveSubsystem, limelight, autoAim.Mode.grid,
            autoAim.subMode.cube);

    // private final autoAim Aim_blue_right_right = new autoAim(swerveSubsystem, limelight, autoAim.Mode.blue_right,
    //         autoAim.subMode.right);
    // private final autoAim Aim_blue_right_mid = new autoAim(swerveSubsystem, limelight, autoAim.Mode.blue_right,
    //         autoAim.subMode.middle);
    // private final autoAim Aim_blue_right_left = new autoAim(swerveSubsystem, limelight, autoAim.Mode.blue_right,
    //         autoAim.subMode.left);
    // private final autoAim Aim_blue_mid_right = new autoAim(swerveSubsystem, limelight, autoAim.Mode.blue_middle,
    //         autoAim.subMode.right);
    // private final autoAim Aim_blue_mid_mid = new autoAim(swerveSubsystem, limelight, autoAim.Mode.blue_middle,
    //         autoAim.subMode.middle);
    // private final autoAim Aim_blue_mid_left = new autoAim(swerveSubsystem, limelight, autoAim.Mode.blue_middle,
    //         autoAim.subMode.left);
    // private final autoAim Aim_blue_left_right = new autoAim(swerveSubsystem, limelight, autoAim.Mode.blue_left,
    //         autoAim.subMode.right);
    // private final autoAim Aim_blue_left_mid = new autoAim(swerveSubsystem, limelight, autoAim.Mode.blue_left,
    //         autoAim.subMode.middle);
    // private final autoAim Aim_blue_left_left = new autoAim(swerveSubsystem, limelight, autoAim.Mode.blue_left,
    //         autoAim.subMode.left);

    private final autoAim Aim_aprilTag_right = new autoAim(swerveSubsystem, limelight, autoAim.Mode.aprilTag,
            autoAim.subMode.right);
    private final autoAim Aim_aprilTag_mid = new autoAim(swerveSubsystem, limelight, autoAim.Mode.aprilTag,
            autoAim.subMode.middle);
    private final autoAim Aim_aprilTag_left = new autoAim(swerveSubsystem, limelight, autoAim.Mode.aprilTag,
            autoAim.subMode.left);
    private final autoAim Aim_aprilTag_doubleSub_left = new autoAim(swerveSubsystem, limelight, autoAim.Mode.doubleSub, null);
    private final autoAim Aim_aprilTag_doubleSub_right = new autoAim(swerveSubsystem, limelight, autoAim.Mode.doubleSub, null);
    // aimViewer AimViewer = new aimViewer(limelight);

    private final Joystick driverJoystick = new Joystick(0);
    private final Joystick customJoystick = new Joystick(3);
    private final CommandXboxController copilotJoystick = new CommandXboxController(2);

    autoBalance auto_balance = new autoBalance(swerveSubsystem, rackSubsystem, intakeSubsystem);

    SendableChooser<Integer> autoChooser = new SendableChooser<>();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // try {
        autoChooser.setDefaultOption("Do Nothing", 0);
        autoChooser.addOption("blue left balance", 1);
        autoChooser.addOption("blue left no balance", 2);
        autoChooser.addOption("red left balance", 3);
        autoChooser.addOption("red left no balance", 4);
        autoChooser.addOption("middle balance", 5);
        autoChooser.addOption("middle no balance", 6);
        autoChooser.addOption("blue right balance", 7);
        autoChooser.addOption("blue right no balance", 8);
        autoChooser.addOption("red right balance", 9);
        autoChooser.addOption("red right no balance", 10);
        SmartDashboard.putData("Auto chooser", autoChooser);
        // } catch (Exception e) {
        // // TODO: handle exception
        // }

        swerveSubsystem.setDefaultCommand(new SwerveJoystick_Cmd(
                swerveSubsystem,
                () -> -driverJoystick.getRawAxis(OIConstants.kDriverYAxis),
                () -> driverJoystick.getRawAxis(OIConstants.kDriverXAxis),
                () -> driverJoystick.getRawButton(9)? 0 : driverJoystick.getRawAxis(OIConstants.kDriverRotAxis),
                () -> driverJoystick.getRawAxis(OIConstants.kDriverSpeedAxis),
                () -> copilotJoystick.getRawAxis(XboxController.Axis.kLeftTrigger.value),
                () -> copilotJoystick.getRawAxis(XboxController.Axis.kRightTrigger.value),
                () -> !driverJoystick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx),
                () -> driverJoystick.getRawButton(OIConstants.kDriverBrakeButtonIdx),
                () -> driverJoystick.getRawButton(OIConstants.kAimCon)
                        || driverJoystick.getRawButton(OIConstants.kAimCube) ||
                        driverJoystick.getRawButton(OIConstants.kAimAprilTag)
                        || driverJoystick.getRawButton(OIConstants.kAimReflective)));

        // limelight.setDefaultCommand(new aimViewer(limelight));

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
     * it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        new JoystickButton(driverJoystick, OIConstants.kResetHeading)
                .onTrue(Commands.runOnce(() -> swerveSubsystem.zeroHeading()));
        new JoystickButton(driverJoystick, 7)
                .onTrue(Commands.runOnce(() -> limelight.setLightCode(LightMode.Blink)));
        new JoystickButton(driverJoystick, 8)
                .onTrue(Commands.runOnce(() -> limelight.setLightCode(LightMode.On)));
        new JoystickButton(driverJoystick, 10)
                .onTrue(Commands.runOnce(() -> limelight.setLightCode(LightMode.Off)));
        // new JoystickButton(driverJoystick, 9)
                // .whileTrue(auto_balance);
        new JoystickButton(driverJoystick, 12)
                .onTrue(Commands.runOnce(() -> limelight.setDriverMode()));
        new POVButton(driverJoystick, 0)
                .onTrue(Commands.runOnce(() -> limelight.get_tag_data(limelight_pipeline.aprilTag)));
        new POVButton(driverJoystick, 270)
                .onTrue(Commands.runOnce(() -> limelight.get_tag_data(limelight_pipeline.con)));
        new POVButton(driverJoystick, 90)
                .onTrue(Commands.runOnce(() -> limelight.get_tag_data(limelight_pipeline.cube)));
        new POVButton(driverJoystick, 180)
                .onTrue(Commands.runOnce(() -> limelight.get_tag_data(limelight_pipeline.reflective)));

        new JoystickButton(driverJoystick, 3).whileTrue(Aim_floor_con);
        new JoystickButton(driverJoystick, 5).whileTrue(Aim_grid_con);
        new JoystickButton(driverJoystick, 4).whileTrue(Aim_floor_cube);
        new JoystickButton(driverJoystick, 6).whileTrue(Aim_grid_cube);

        new JoystickButton(customJoystick, 1).whileTrue(Aim_aprilTag_left);
        new JoystickButton(customJoystick, 2).whileTrue(Aim_aprilTag_mid);
        new JoystickButton(customJoystick, 3).whileTrue(Aim_aprilTag_right);
        // new JoystickButton(customJoystick, 4).whileTrue(Aim_blue_mid_right);
        // new JoystickButton(customJoystick, 5).whileTrue(Aim_blue_mid_mid);
        // new JoystickButton(customJoystick, 6).whileTrue(Aim_blue_mid_left);
        // new JoystickButton(customJoystick, 7).whileTrue(Aim_blue_left_right);
        new JoystickButton(customJoystick, 8).whileTrue(Aim_aprilTag_doubleSub_left);
        new JoystickButton(customJoystick, 9).whileTrue(Aim_aprilTag_doubleSub_right);

        new JoystickButton(customJoystick, 10).onTrue(Commands.runOnce(() -> {
            elevatorSubsystem.stay_pid_control = true;
            intakeSubsystem.stay_pid_control = true;
        }));
        new JoystickButton(customJoystick, 10).onFalse(Commands.runOnce(() -> {
            elevatorSubsystem.stay_pid_control = false;
            intakeSubsystem.stay_pid_control = false;
        }));

        copilotJoystick.leftStick().onTrue(Commands.runOnce(() -> limelight.setDriverMode()));
        copilotJoystick.rightStick().onTrue(Commands.runOnce(() -> limelight.setDriverMode()));

        // 半全自動
        // copilotJoystick.a().whileTrue(putDown_Cube);
        // copilotJoystick.a().onFalse(Commands.runOnce(() -> putDown_Cube.stop()));
        // copilotJoystick.y().whileTrue(putUp_Cube);
        // copilotJoystick.y().onFalse(Commands.runOnce(() -> putUp_Cube.stop()));
        // copilotJoystick.b().whileTrue(putMiddle_Cube);
        // copilotJoystick.b().onFalse(Commands.runOnce(() -> putMiddle_Cube.stop()));
        // copilotJoystick.povDown().whileTrue(putDown_Con);
        // copilotJoystick.povDown().onFalse(Commands.runOnce(() ->
        // putDown_Con.stop()));
        // copilotJoystick.povUp().whileTrue(putUp_Con);
        // copilotJoystick.povUp().onFalse(Commands.runOnce(() -> putUp_Con.stop()));
        // copilotJoystick.povRight().whileTrue(putMiddle_Con);
        // copilotJoystick.povRight().onFalse(Commands.runOnce(() ->
        // putMiddle_Con.stop()));
        copilotJoystick.povLeft().whileTrue(putReset);
        copilotJoystick.povLeft().onFalse(Commands.runOnce(() -> putReset.stop()));

        // copilotJoystick.start().whileTrue(eat_double_substation_con);
        // copilotJoystick.start().onFalse(Commands.runOnce(() ->
        // eat_double_substation_con.stop()));
        // copilotJoystick.back().whileTrue(eat_single_substation_cube);
        // copilotJoystick.back().onFalse(Commands.runOnce(() ->
        // eat_single_substation_cube.stop()));
        // copilotJoystick.leftBumper().whileTrue(eat_floor_cube);
        // copilotJoystick.leftBumper().onFalse(Commands.runOnce(() ->
        // eat_floor_cube.stop()));
        // copilotJoystick.leftStick().whileTrue(eat_floor_con);
        // copilotJoystick.leftStick().onFalse(Commands.runOnce(() ->
        // eat_floor_con.stop()));

        // 半自動
        // copilotJoystick.povUp().whileTrue(Commands.run(() ->
        // intakeSubsystem.addPoint(-3)));
        copilotJoystick.b().whileTrue(
                Commands.run(() -> intakeSubsystem.set_lift_position(intake_position.intake_substation_cube)));
        copilotJoystick.b().onFalse(Commands.runOnce(() -> intakeSubsystem.lift_stop()));
        copilotJoystick.povRight()
                .whileTrue(Commands.run(() -> intakeSubsystem.set_lift_position(intake_position.intake_straight)));
        copilotJoystick.povRight().onFalse(Commands.runOnce(() -> intakeSubsystem.lift_stop()));
        // copilotJoystick.povDown().whileTrue(Commands.run(() ->
        // intakeSubsystem.addPoint(+3)));

        copilotJoystick.back().whileTrue(Commands.run(() -> rackSubsystem.rack_close()));
        copilotJoystick.back().onFalse(Commands.runOnce(() -> rackSubsystem.rack_stop_close()));
        copilotJoystick.start().whileTrue(Commands.run(() -> rackSubsystem.rack_open()));
        copilotJoystick.start().onFalse(Commands.runOnce(() -> rackSubsystem.rack_stop()));

        copilotJoystick.rightBumper().whileTrue(Commands.run(() -> elevatorSubsystem.elevator_up()));
        // copilotJoystick.rightBumper()
        //         .onFalse(Commands.run(() -> elevatorSubsystem.setPoint(elevatorSubsystem.elevator_get_height())));
        copilotJoystick.rightBumper().onFalse(Commands.run(() -> elevatorSubsystem.elevator_stop()));
        copilotJoystick.a().whileTrue(Commands.run(() -> elevatorSubsystem.elevator_middle()));
        copilotJoystick.a().onFalse(Commands.runOnce(() -> elevatorSubsystem.elevator_stop()));
        copilotJoystick.leftBumper().whileTrue(Commands.run(() -> elevatorSubsystem.elevator_down()));
        // copilotJoystick.leftBumper()
        //         .onFalse(Commands.runOnce(() -> elevatorSubsystem.setPoint(elevatorSubsystem.elevator_get_height())));
        copilotJoystick.leftBumper().onFalse(Commands.runOnce(() -> elevatorSubsystem.elevator_stop()));

        copilotJoystick.y().onTrue(Commands.runOnce(() -> intakeSubsystem.eatCon()));
        copilotJoystick.y().onFalse(Commands.runOnce(() -> intakeSubsystem.suckStop()));
        copilotJoystick.x().onTrue(Commands.runOnce(() -> intakeSubsystem.eatCube()));
        copilotJoystick.x().onFalse(Commands.runOnce(() -> intakeSubsystem.suckStop()));

        // 全手動
        copilotJoystick.povDown().whileTrue(Commands.run(() -> {
            intakeSubsystem.lift.setPercentOutput(-0.7);
        }));
        copilotJoystick.povDown().onFalse(Commands.runOnce(() -> intakeSubsystem.lift_stop()));
        copilotJoystick.povUp().whileTrue(Commands.run(() -> {
               
                    intakeSubsystem.lift.setPercentOutput(0.5);
            }));
        copilotJoystick.povUp().onFalse(Commands.runOnce(() -> intakeSubsystem.lift_stop()));

        // copilotJoystick.back().onTrue(Commands.runOnce(() -> {
        // rackSubsystem.elongation.setPercentOutput(0.5);
        // }));
        // copilotJoystick.back().onFalse(Commands.runOnce(() -> {
        // rackSubsystem.rack_stop();
        // }));
        // copilotJoystick.start().onTrue(Commands.runOnce(() -> {
        // rackSubsystem.elongation.setPercentOutput(-0.5);
        // }));
        // copilotJoystick.start().onFalse(Commands.runOnce(() -> {
        // rackSubsystem.rack_stop();
        // }));

        // copilotJoystick.rightBumper().whileTrue(Commands.run(() ->
        // elevatorSubsystem.elevator_stupid_up()));
        // copilotJoystick.rightBumper().onFalse(Commands.runOnce(() ->
        // elevatorSubsystem.elevator_stop()));
        // copilotJoystick.leftBumper().whileTrue(Commands.run(() ->
        // elevatorSubsystem.elevator_stupid_down()));
        // copilotJoystick.leftBumper().onFalse(Commands.runOnce(() ->
        // elevatorSubsystem.elevator_stop()));

        // copilotJoystick.y().onTrue(Commands.runOnce(() -> {
        // intakeSubsystem.eatCon();
        // }));
        // copilotJoystick.y().onFalse(Commands.runOnce(() -> {
        // intakeSubsystem.suckStop();
        // }));
        // copilotJoystick.x().onTrue(Commands.runOnce(() -> {
        // intakeSubsystem.eatCube();
        // }));
        // copilotJoystick.x().onFalse(Commands.runOnce(() -> {
        // intakeSubsystem.suckStop();
        // }));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return (new blue_left_no_balance(swerveSubsystem, intakeSubsystem, rackSubsystem, elevatorSubsystem,
                    limelight));
        // rackSubsystem, elevatorSubsystem, limelight));
        // if (autoChooser.getSelected() == 1)
        //     return (new blue_left_balance(swerveSubsystem, intakeSubsystem, rackSubsystem, elevatorSubsystem,
        //             limelight));
        // else if (autoChooser.getSelected() == 2)
        //     return (new blue_left_no_balance(swerveSubsystem, intakeSubsystem, rackSubsystem, elevatorSubsystem,
        //             limelight));
        // else if (autoChooser.getSelected() == 3)
        //     return (new blue_right_balance(swerveSubsystem, intakeSubsystem, rackSubsystem, elevatorSubsystem,
        //             limelight));
        // else if (autoChooser.getSelected() == 4)
        //     return (new blue_right_no_balance(swerveSubsystem, intakeSubsystem, rackSubsystem, elevatorSubsystem,
        //             limelight));
        // else if (autoChooser.getSelected() == 5)
        //     return (new mid_balance(swerveSubsystem, intakeSubsystem, rackSubsystem, elevatorSubsystem, limelight));
        // else if (autoChooser.getSelected() == 6)
        //     return (new mid_no_balance(swerveSubsystem, intakeSubsystem, rackSubsystem, elevatorSubsystem,
        //             limelight));
        // else if (autoChooser.getSelected() == 7)
        //     return (new blue_right_balance(swerveSubsystem, intakeSubsystem, rackSubsystem, elevatorSubsystem,
        //             limelight));
        // else if (autoChooser.getSelected() == 8)
        //     return (new blue_right_no_balance(swerveSubsystem, intakeSubsystem, rackSubsystem, elevatorSubsystem,
        //             limelight));
        // else if (autoChooser.getSelected() == 9)
        //     return (new blue_left_balance(swerveSubsystem, intakeSubsystem, rackSubsystem, elevatorSubsystem,
        //             limelight));
        // else if (autoChooser.getSelected() == 10)
        //     return (new blue_left_no_balance(swerveSubsystem, intakeSubsystem, rackSubsystem, elevatorSubsystem,
        //             limelight));
        // return null;

        // List<PathPlannerTrajectory> trajectory = PathPlanner.loadPathGroup("test",
        // new PathConstraints(4, 3));
        // PathPlannerTrajectory traj1 = trajectory.get(0);
        // PPSwerveControllerCommand autoDrive1 =
        // PathPlannerModule.baseSwerveCommand(traj1);

        // 5. Add some init and wrap-up, and return everything
        // return new SequentialCommandGroup(
        // // putMiddle_Cube,
        // new InstantCommand(() ->
        // swerveSubsystem.resetOdometry(traj1.getInitialHolonomicPose())),
        // // new InstantCommand(() -> limelight.setLightCode(LightMode.On)),
        // autoDrive1,
        // // new InstantCommand(() -> limelight.setLightCode(LightMode.Off)),
        // // autoDrive2,
        // // new InstantCommand(() -> limelight.setLightCode(LightMode.Blink)),
        // // autoDrive3,
        // // new InstantCommand(() -> limelight.setLightCode(LightMode.Off)),
        // new InstantCommand(() -> swerveSubsystem.stopModules()));
    }
}

// package frc.robot.commands;

// import frc.robot.Constants.DriveConstants;
// import frc.robot.Constants.limelight_pipeline;
// import frc.robot.modules.LimelightModule;
// import frc.robot.subsystems.Limelight;
// import frc.robot.subsystems.SwerveSubsystem;
// import frc.robot.Constants.AutoAimConstants;

// import java.io.Console;
// import java.util.PropertyResourceBundle;
// import java.util.function.Supplier;

// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.math.kinematics.SwerveModuleState;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.CommandBase;

// public class aimViewer extends CommandBase {
//      Limelight limelight;
//      limelight_pipeline pipeline;
//      double tx = LimelightModule.getBasicData("tx");
//      double ty = LimelightModule.getBasicData("ty");
//      Mode mode;
//      subMode submode;
//      double x_Error = 0, y_Error = 0;
//      boolean allowance_x = false;
//      boolean allowance_y = false;

//      public static enum Mode {
//           floor, substation,
//           blue_left, blue_right, blue_middle,
//           red_left, red_right, red_middle;
//      }

//      public static enum subMode {
//           right, middle, left, cube, con;
//      }

//      public aimViewer(Limelight limelight) {
//           allowance_x = false;
//           allowance_y = false;
//           this.limelight = limelight;

//           addRequirements(limelight);
//      }

//      @Override
//      public void initialize() {

//      }

//      @Override
//      public void execute() {

//           if (limelight.get_pipeline() == limelight_pipeline.con.value)
//                pipeline = limelight_pipeline.con;
//           if (limelight.get_pipeline() == limelight_pipeline.cube.value)
//                pipeline = limelight_pipeline.cube;
//           if (limelight.get_pipeline() == limelight_pipeline.aprilTag.value)
//                pipeline = limelight_pipeline.aprilTag;

//           double[] data = limelight.get_tag_data(pipeline);
//           double field_x = data[5];
//           double field_y = data[6];
//           double field_z = data[7];
//           double field_Roll = data[8];
//           double field_Pitch = data[9];
//           double field_Yaw = data[10];

//           if (pipeline == limelight_pipeline.con) {
//                double heading_x_error = data[1];
//                double heading_y_error = 0;
//                if (mode == Mode.floor)
//                     heading_y_error = data[2] + AutoAimConstants.con_floor_target_y;
//                // if(mode == Mode.substation) heading_y_error =
//                // data[2]+AutoAimConstants.con_substation_target_y;

//                y_Error = AutoAimConstants.kP_con_x * heading_x_error * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
//                x_Error = AutoAimConstants.kP_con_y * heading_y_error * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;

//           }
//           if (pipeline == limelight_pipeline.cube) {
//                double heading_x_error = data[1];
//                double heading_y_error = 0;
//                if (mode == Mode.floor)
//                     heading_y_error = data[2] + AutoAimConstants.cube_floor_target_y;
//                // if(mode == Mode.substation) heading_y_error =
//                // data[2]+AutoAimConstants.cube_substation_target_y;

//                y_Error = AutoAimConstants.kP_cube_x * heading_x_error
//                          * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
//                x_Error = AutoAimConstants.kP_cube_y * heading_y_error
//                          * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;

//           }
//           if (pipeline == limelight_pipeline.aprilTag) {
//                double heading_x_error = 0;
//                double heading_y_error = 0;
//                double heading_turn_error = 0;
//                double heading_x_target = 0;
//                double heading_y_target = 0;
//                double heading_turn_target = 180;

//                if (mode == Mode.blue_left) {
//                     if (submode == subMode.left) {
//                          heading_x_target = AutoAimConstants.AprilTag_blue_left_left_target_x;
//                          heading_y_target = AutoAimConstants.AprilTag_blue_left_left_target_y;
//                     }
//                     if (submode == subMode.middle) {
//                          heading_x_target = AutoAimConstants.AprilTag_blue_left_middle_target_x;
//                          heading_y_target = AutoAimConstants.AprilTag_blue_left_middle_target_y;
//                     }
//                     if (submode == subMode.right) {
//                          heading_x_target = AutoAimConstants.AprilTag_blue_left_right_target_x;
//                          heading_y_target = AutoAimConstants.AprilTag_blue_left_right_target_y;
//                     }
//                }
//                if (mode == Mode.blue_middle) {
//                     if (submode == subMode.left) {
//                          heading_x_target = AutoAimConstants.AprilTag_blue_middle_left_target_x;
//                          heading_y_target = AutoAimConstants.AprilTag_blue_middle_left_target_y;
//                     }
//                     if (submode == subMode.middle) {
//                          heading_x_target = AutoAimConstants.AprilTag_blue_middle_middle_target_x;
//                          heading_y_target = AutoAimConstants.AprilTag_blue_middle_middle_target_y;
//                     }
//                     if (submode == subMode.right) {
//                          heading_x_target = AutoAimConstants.AprilTag_blue_middle_right_target_x;
//                          heading_y_target = AutoAimConstants.AprilTag_blue_middle_right_target_y;
//                     }
//                }
//                if (mode == Mode.blue_right) {
//                     if (submode == subMode.left) {
//                          heading_x_target = AutoAimConstants.AprilTag_blue_right_left_target_x;
//                          heading_y_target = AutoAimConstants.AprilTag_blue_right_left_target_y;
//                     }
//                     if (submode == subMode.middle) {
//                          heading_x_target = AutoAimConstants.AprilTag_blue_right_middle_target_x;
//                          heading_y_target = AutoAimConstants.AprilTag_blue_right_middle_target_y;
//                     }
//                     if (submode == subMode.right) {
//                          heading_x_target = AutoAimConstants.AprilTag_blue_right_right_target_x;
//                          heading_y_target = AutoAimConstants.AprilTag_blue_right_right_target_y;
//                     }
//                }

//                if (mode == Mode.red_left) {
//                     if (submode == subMode.left) {
//                          heading_x_target = AutoAimConstants.AprilTag_red_left_left_target_x;
//                          heading_y_target = AutoAimConstants.AprilTag_red_left_left_target_y;
//                     }
//                     if (submode == subMode.middle) {
//                          heading_x_target = AutoAimConstants.AprilTag_red_left_middle_target_x;
//                          heading_y_target = AutoAimConstants.AprilTag_red_left_middle_target_y;
//                     }
//                     if (submode == subMode.right) {
//                          heading_x_target = AutoAimConstants.AprilTag_red_left_right_target_x;
//                          heading_y_target = AutoAimConstants.AprilTag_red_left_right_target_y;
//                     }
//                }
//                if (mode == Mode.red_middle) {
//                     if (submode == subMode.left) {
//                          heading_x_target = AutoAimConstants.AprilTag_red_middle_left_target_x;
//                          heading_y_target = AutoAimConstants.AprilTag_red_middle_left_target_y;
//                     }
//                     if (submode == subMode.middle) {
//                          heading_x_target = AutoAimConstants.AprilTag_red_middle_middle_target_x;
//                          heading_y_target = AutoAimConstants.AprilTag_red_middle_middle_target_y;
//                     }
//                     if (submode == subMode.right) {
//                          heading_x_target = AutoAimConstants.AprilTag_red_middle_right_target_x;
//                          heading_y_target = AutoAimConstants.AprilTag_red_middle_right_target_y;
//                     }
//                }
//                if (mode == Mode.red_right) {
//                     if (submode == subMode.left) {
//                          heading_x_target = AutoAimConstants.AprilTag_red_right_left_target_x;
//                          heading_y_target = AutoAimConstants.AprilTag_red_right_left_target_y;
//                     }
//                     if (submode == subMode.middle) {
//                          heading_x_target = AutoAimConstants.AprilTag_red_right_middle_target_x;
//                          heading_y_target = AutoAimConstants.AprilTag_red_right_middle_target_y;
//                     }
//                     if (submode == subMode.right) {
//                          heading_x_target = AutoAimConstants.AprilTag_red_right_right_target_x;
//                          heading_y_target = AutoAimConstants.AprilTag_red_right_right_target_y;
//                     }
//                }

//                heading_x_error = heading_x_target - field_x;
//                heading_y_error = heading_y_target - field_y;
//                heading_turn_error = heading_turn_target - field_Yaw;

//                y_Error = -AutoAimConstants.kP_AprilTag_y * heading_y_error
//                          * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
//                x_Error = AutoAimConstants.kP_AprilTag_x * heading_x_error
//                          * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;

//                if (x_Error > 0)
//                     x_Error = Math.min(x_Error, 1.5);
//                if (x_Error < 0)
//                     x_Error = Math.max(x_Error, -1.5);
//                if (y_Error > 0)
//                     y_Error = Math.min(y_Error, 1.5);
//                if (y_Error < 0)
//                     y_Error = Math.max(y_Error, -1.5);
//           }

//           if (Math.abs(x_Error) < 0.1 * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond)
//                x_Error = 0;
//           if (Math.abs(y_Error) < 0.1 * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond)
//                y_Error = 0;
//           if (data[0] == 1) {
//                allowance_y = y_Error == 0;
//                allowance_x = x_Error == 0;
//                SmartDashboard.putNumber("X_Error", x_Error);
//                SmartDashboard.putNumber("Y_Error", y_Error);
//           } else {
//                SmartDashboard.putNumber("X_Error", 404);
//                SmartDashboard.putNumber("Y_Error", 404);
//           }
//           SmartDashboard.putBoolean("X Allowance", allowance_x);
//           SmartDashboard.putBoolean("Y Allowance", allowance_y);
//      }

//      public double get_x_Error() {
//           return x_Error;
//      }

//      public double get_y_Error() {
//           return y_Error;
//      }

//      @Override
//      public void end(boolean interrupted) {
//      }
// }

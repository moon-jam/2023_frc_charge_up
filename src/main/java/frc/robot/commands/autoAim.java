package frc.robot.commands;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.limelight_pipeline;
import frc.robot.modules.LimelightModule;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.Constants.AutoAimConstants;

import java.io.Console;
import java.util.PropertyResourceBundle;
import java.util.function.Supplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class autoAim extends CommandBase {
    Limelight limelight;
    SwerveSubsystem swerve;
    public double xSpeed = 0, ySpeed = 0, turnSpeed = 0;
    limelight_pipeline pipeline;
    double tx = LimelightModule.getBasicData("tx");
    double ty = LimelightModule.getBasicData("ty");
    ChassisSpeeds chassisSpeeds;
    Mode mode;
    subMode submode;
    public boolean allowance_x = false;
    public boolean allowance_y = false;
    
    public static enum Mode {
        floor, substation, grid,
        blue_left, blue_right, blue_middle,
        red_left, red_right, red_middle, aprilTag, doubleSub;
    }
    
    public static enum subMode {
        right, middle, left, cube, con;
    }
 
    public autoAim(SwerveSubsystem swerve, Limelight limelight, Mode mode, subMode submode) {
        this.limelight = limelight;
        this.swerve = swerve;
        this.mode = mode;
        this.submode = submode;
 
        addRequirements(limelight, swerve);
    }
 
    @Override
    public void initialize(){
        allowance_x = false;
        allowance_y = false;
        xSpeed = 0;
        ySpeed = 0;
        turnSpeed = 0;
    }

    @Override
    public void execute() {

        if(submode == subMode.con)  pipeline = limelight_pipeline.con;
        if(submode == subMode.cube)  pipeline = limelight_pipeline.cube;
        if(mode == Mode.grid && submode == subMode.con)  
            pipeline = limelight_pipeline.reflective;
        if(mode == Mode.grid && submode == subMode.cube)  
            pipeline = limelight_pipeline.aprilTag;

        double[] data = limelight.get_tag_data(pipeline);
        double field_x = data[5];
        double field_y = data[6];
        double field_z = data[7];
        double field_Roll = data[8];
        double field_Pitch = data[9];
        double field_Yaw = data[10];
        
        
        SmartDashboard.putNumber("steering_adjust", turnSpeed);

        if(pipeline == limelight_pipeline.con){
            double heading_x_error= data[1];
            double heading_y_error = 0;
            heading_y_error = data[2]-AutoAimConstants.con_floor_target_y;
            
            ySpeed = AutoAimConstants.kP_con_x*heading_x_error* DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
            xSpeed = AutoAimConstants.kP_con_y*heading_y_error* DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
                // SmartDashboard.putNumber("Xspeed", xSpeed);
                // turnSpeed = -AutoAimConstants.kP_con_turn*heading_x_error*DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;   
        }
        if(pipeline == limelight_pipeline.cube){
            double heading_x_error = data[1];
            double heading_y_error = 0;
            heading_y_error = data[2]-AutoAimConstants.cube_floor_target_y;

            ySpeed = AutoAimConstants.kP_cube_x*heading_x_error* DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
            xSpeed = AutoAimConstants.kP_cube_y*heading_y_error* DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
            // turnSpeed = AutoAimConstants.kP_cube_turn*heading_x_error*DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;
        }
        if(pipeline == limelight_pipeline.aprilTag){
            double heading_x_error = data[1];
            double heading_y_error = 0;
            heading_y_error = data[2]-AutoAimConstants.AprilTag_target_y;
            if(heading_y_error >10) heading_y_error = 10;
            ySpeed = 0.015*heading_x_error* DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
            xSpeed = 0.015*heading_y_error* DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
            // turnSpeed = AutoAimConstants.kP_cube_turn*heading_x_error*DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;
        }
        if(pipeline == limelight_pipeline.reflective){
            double heading_x_error = data[1];
            double heading_y_error = 0;
            heading_y_error = data[2]-AutoAimConstants.Reflective_target_y;

            ySpeed = 0.015*heading_x_error* DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
            xSpeed = 0.015*heading_y_error* DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
            // turnSpeed = AutoAimConstants.kP_cube_turn*heading_x_error*DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;
        }

        if(Math.abs(xSpeed) < 0.05*DriveConstants.kTeleDriveMaxSpeedMetersPerSecond) xSpeed = 0;
        if(Math.abs(ySpeed) < 0.05*DriveConstants.kTeleDriveMaxSpeedMetersPerSecond) ySpeed = 0;
        if(Math.abs(turnSpeed) < 0.05*DriveConstants.kTeleDriveMaxSpeedMetersPerSecond) turnSpeed = 0;
        if(data[0] == 1){
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turnSpeed);
            SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
            swerve.setModuleStates(moduleStates);

            allowance_y = Math.abs(ySpeed) < 1;
            allowance_x = Math.abs(xSpeed) < 1;
            SmartDashboard.putNumber("X_Error", xSpeed);
            SmartDashboard.putNumber("Y_Error", ySpeed);
        }else{
            SmartDashboard.putNumber("X_Error", 404);
            SmartDashboard.putNumber("Y_Error", 404);
            swerve.stopModules();
        }
        SmartDashboard.putBoolean("X Allowance", allowance_x);
        SmartDashboard.putBoolean("Y Allowance", allowance_y);
    }
    
 
    @Override
    public void end(boolean interrupted) {
        swerve.stopModules();
    }
}
/**
 *             // if(mode == Mode.substation) {
            //     pipeline = limelight_pipeline.aprilTag;
            //     data = limelight.get_tag_data(pipeline);
                
            //     heading_x_error = AutoAimConstants.con_substation_target_x-field_x;
            //     heading_y_error = AutoAimConstants.con_substation_target_y-field_y;
            //     double heading_turn_error = 0-field_Yaw;

            //     ySpeed = -AutoAimConstants.kP_AprilTag_y*heading_y_error* DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
            //     xSpeed = AutoAimConstants.kP_AprilTag_x*heading_x_error* DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
            //     turnSpeed = -AutoAimConstants.kP_AprilTag_turn*heading_turn_error*DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;

            //     if(xSpeed>0)    xSpeed = Math.min(xSpeed, 1.5);
            //     if(xSpeed<0)    xSpeed = Math.max(xSpeed, -1.5);
            //     if(ySpeed>0)    ySpeed = Math.min(ySpeed, 1.5);
            //     if(ySpeed<0)    ySpeed = Math.max(ySpeed, -1.5);
            //     if(turnSpeed>0)    turnSpeed = Math.min(turnSpeed, 1);
            //     if(turnSpeed<0)    turnSpeed = Math.max(turnSpeed, -1);
            //     if(Math.abs(xSpeed) < 0.1*DriveConstants.kTeleDriveMaxSpeedMetersPerSecond) xSpeed = 0;
            //     if(Math.abs(ySpeed) < 0.1*DriveConstants.kTeleDriveMaxSpeedMetersPerSecond) ySpeed = 0;
            //     if(Math.abs(turnSpeed) < 0.1*DriveConstants.kTeleDriveMaxSpeedMetersPerSecond) turnSpeed = 0;
            //     if(data[0] == 1){
            //         if(pipeline == limelight_pipeline.aprilTag){
            //             chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            //                     xSpeed, ySpeed, turnSpeed, swerve.getRotation2d());
            //         }
            //         else{
            //             chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turnSpeed);
            //         }
            //         SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
            //         swerve.setModuleStates(moduleStates);
            //     }else{
            //         swerve.stopModules();
            //     }
                
            // }   

            
        if(pipeline == limelight_pipeline.aprilTag){
            double heading_x_error = 0;
            double heading_y_error = 0;
            double heading_turn_error = 0;
            double heading_x_target = 0;
            double heading_y_target = 0;
            double heading_turn_target = 180;

            if(mode == Mode.blue_left){
                if(submode == subMode.left) {
                     heading_x_target = AutoAimConstants.AprilTag_blue_left_left_target_x;
                     heading_y_target = AutoAimConstants.AprilTag_blue_left_left_target_y;
                }
                if(submode == subMode.middle) {
                     heading_x_target = AutoAimConstants.AprilTag_blue_left_middle_target_x;
                     heading_y_target = AutoAimConstants.AprilTag_blue_left_middle_target_y;
                }
                if(submode == subMode.right) {
                     heading_x_target = AutoAimConstants.AprilTag_blue_left_right_target_x;
                     heading_y_target = AutoAimConstants.AprilTag_blue_left_right_target_y;
                }
            }
            if(mode == Mode.blue_middle){
                if(submode == subMode.left) {
                     heading_x_target = AutoAimConstants.AprilTag_blue_middle_left_target_x;
                     heading_y_target = AutoAimConstants.AprilTag_blue_middle_left_target_y;
                }
                if(submode == subMode.middle) {
                     heading_x_target = AutoAimConstants.AprilTag_blue_middle_middle_target_x;
                     heading_y_target = AutoAimConstants.AprilTag_blue_middle_middle_target_y;
                }
                if(submode == subMode.right) {
                     heading_x_target = AutoAimConstants.AprilTag_blue_middle_right_target_x;
                     heading_y_target = AutoAimConstants.AprilTag_blue_middle_right_target_y;
                }
            }
            if(mode == Mode.blue_right){
                if(submode == subMode.left) {
                     heading_x_target = AutoAimConstants.AprilTag_blue_right_left_target_x;
                     heading_y_target = AutoAimConstants.AprilTag_blue_right_left_target_y;
                }
                if(submode == subMode.middle) {
                     heading_x_target = AutoAimConstants.AprilTag_blue_right_middle_target_x;
                     heading_y_target = AutoAimConstants.AprilTag_blue_right_middle_target_y;
                }
                if(submode == subMode.right) {
                     heading_x_target = AutoAimConstants.AprilTag_blue_right_right_target_x;
                     heading_y_target = AutoAimConstants.AprilTag_blue_right_right_target_y;
                }
            }

            if(mode == Mode.red_left){
                if(submode == subMode.left) {
                     heading_x_target = AutoAimConstants.AprilTag_red_left_left_target_x;
                     heading_y_target = AutoAimConstants.AprilTag_red_left_left_target_y;
                }
                if(submode == subMode.middle) {
                     heading_x_target = AutoAimConstants.AprilTag_red_left_middle_target_x;
                     heading_y_target = AutoAimConstants.AprilTag_red_left_middle_target_y;
                }
                if(submode == subMode.right) {
                     heading_x_target = AutoAimConstants.AprilTag_red_left_right_target_x;
                     heading_y_target = AutoAimConstants.AprilTag_red_left_right_target_y;
                }
            }
            if(mode == Mode.red_middle){
                if(submode == subMode.left) {
                     heading_x_target = AutoAimConstants.AprilTag_red_middle_left_target_x;
                     heading_y_target = AutoAimConstants.AprilTag_red_middle_left_target_y;
                }
                if(submode == subMode.middle) {
                     heading_x_target = AutoAimConstants.AprilTag_red_middle_middle_target_x;
                     heading_y_target = AutoAimConstants.AprilTag_red_middle_middle_target_y;
                }
                if(submode == subMode.right) {
                     heading_x_target = AutoAimConstants.AprilTag_red_middle_right_target_x;
                     heading_y_target = AutoAimConstants.AprilTag_red_middle_right_target_y;
                }
            }
            if(mode == Mode.red_right){
                if(submode == subMode.left) {
                     heading_x_target = AutoAimConstants.AprilTag_red_right_left_target_x;
                     heading_y_target = AutoAimConstants.AprilTag_red_right_left_target_y;
                }
                if(submode == subMode.middle) {
                     heading_x_target = AutoAimConstants.AprilTag_red_right_middle_target_x;
                     heading_y_target = AutoAimConstants.AprilTag_red_right_middle_target_y;
                }
                if(submode == subMode.right) {
                     heading_x_target = AutoAimConstants.AprilTag_red_right_right_target_x;
                     heading_y_target = AutoAimConstants.AprilTag_red_right_right_target_y;
                }
            }

            heading_x_error = heading_x_target-field_x;
            heading_y_error = heading_y_target-field_y;
            heading_turn_error = heading_turn_target-field_Yaw;

            ySpeed = -AutoAimConstants.kP_AprilTag_y*heading_y_error* DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
            xSpeed = AutoAimConstants.kP_AprilTag_x*heading_x_error* DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
            turnSpeed = -AutoAimConstants.kP_AprilTag_turn*heading_turn_error*DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;

            if(xSpeed>0)    xSpeed = Math.min(xSpeed, 1.5);
            if(xSpeed<0)    xSpeed = Math.max(xSpeed, -1.5);
            if(ySpeed>0)    ySpeed = Math.min(ySpeed, 1.5);
            if(ySpeed<0)    ySpeed = Math.max(ySpeed, -1.5);
            if(turnSpeed>0)    turnSpeed = Math.min(turnSpeed, 1);
            if(turnSpeed<0)    turnSpeed = Math.max(turnSpeed, -1);
        }


        
        if(mode == Mode.aprilTag){
            double heading_x_error = 0;
            double heading_y_error = data[2]-AutoAimConstants.AprilTag_target_y;
            if(submode == subMode.left)     heading_x_error = data[1]-AutoAimConstants.AprilTag_left_target_x;
            if(submode == subMode.middle)   heading_x_error = data[1]-AutoAimConstants.AprilTag_middle_target_x;
            if(submode == subMode.right)    heading_x_error = data[1]-AutoAimConstants.AprilTag_right_target_x;
            ySpeed = AutoAimConstants.kP_cube_x*heading_x_error* DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
            xSpeed = AutoAimConstants.kP_cube_y*heading_y_error* DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
            turnSpeed = AutoAimConstants.kP_cube_turn*heading_x_error*DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;
        }

        if(mode == Mode.doubleSub){
            double heading_x_error = data[1];
            double heading_y_error = data[2]-AutoAimConstants.AprilTag_doubleSub_target_y;
            ySpeed = AutoAimConstants.kP_cube_x*heading_x_error* DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
            xSpeed = AutoAimConstants.kP_cube_y*heading_y_error* DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
            turnSpeed = AutoAimConstants.kP_cube_turn*heading_x_error*DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;
        }
 */

 // AutoAimConstants.kP_con_x = SmartDashboard.getNumber("kP_con_x", 0);
        // AutoAimConstants.kP_con_y = SmartDashboard.getNumber("kP_con_y", 0);
        // AutoAimConstants.kP_con_turn = SmartDashboard.getNumber("kP_con_turn", 0);
        // AutoAimConstants.kP_cube_x = SmartDashboard.getNumber("kP_cube_x", 0);
        // AutoAimConstants.kP_cube_y = SmartDashboard.getNumber("kP_cube_y", 0);
        // AutoAimConstants.kP_cube_turn = SmartDashboard.getNumber("kP_cube_turn", 0);
        // AutoAimConstants.kP_AprilTag_x = SmartDashboard.getNumber("kP_AprilTag_x", 0);
        // AutoAimConstants.kP_AprilTag_y = SmartDashboard.getNumber("kP_AprilTag_y", 0);
        // AutoAimConstants.kP_AprilTag_turn = SmartDashboard.getNumber("kP_AprilTag_turn", 0);
        // AutoAimConstants.kP_Reflective_x = SmartDashboard.getNumber("kP_Reflective_x", 0);
        // AutoAimConstants.kP_Reflective_y = SmartDashboard.getNumber("kP_Reflective_y", 0);
        // AutoAimConstants.kP_Reflective_turn = SmartDashboard.getNumber("kP_Reflective_turn", 0);

        // SmartDashboard.putNumber("kP_con_x", AutoAimConstants.kP_con_x);
        // SmartDashboard.putNumber("kP_con_y", AutoAimConstants.kP_con_y);
        // SmartDashboard.putNumber("kP_con_turn", AutoAimConstants.kP_con_turn);
        // SmartDashboard.putNumber("kP_cube_x", AutoAimConstants.kP_cube_x);
        // SmartDashboard.putNumber("kP_cube_y", AutoAimConstants.kP_cube_y);
        // SmartDashboard.putNumber("kP_cube_turn", AutoAimConstants.kP_cube_turn);
        // SmartDashboard.putNumber("kP_AprilTag_x", AutoAimConstants.kP_AprilTag_x);
        // SmartDashboard.putNumber("kP_AprilTag_y", AutoAimConstants.kP_AprilTag_y);
        // SmartDashboard.putNumber("kP_AprilTag_turn", AutoAimConstants.kP_AprilTag_turn);
        // SmartDashboard.putNumber("kP_Reflective_x", AutoAimConstants.kP_Reflective_x);
        // SmartDashboard.putNumber("kP_Reflective_y", AutoAimConstants.kP_Reflective_y);
        // SmartDashboard.putNumber("kP_Reflective_turn", AutoAimConstants.kP_Reflective_turn);
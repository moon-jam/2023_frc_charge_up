// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static enum PortID {
        intake_suck_neo550(22, true, 0, 0, 0, 0, 0, 0.2, 0),

        intake_lift_falcon500(12, false, 0.03, 0, 0.02, -0.007, 0, 0, 500),
        rack_elongation_falcon500(13, false, 0, 0, 0, 0, 0, 0, 0),
        elevator_elongation_r_falcon500(14, false, 0, 0, 0, 0, 0, 0, 0),
        elevator_elongation_l_falcon500(15, false, 0, 0, 0, 0, 0, 0, 0),

        intake_encoder(16, false, 0, 0, 0, 0, 0, 0, 0),

        // yellow
        rack_opne_limit_switch(2, true, 0, 0, 0, 0, 0, 0, 0),
        // green
        rack_close_limit_switch(3, true, 0, 0, 0, 0, 0, 0, 0),
        // white
        elevator_opne_limit_switch(5, true, 0, 0, 0, 0, 0, 0, 0),
        // red
        elevator_close_limit_switch(4, true, 0, 0, 0, 0, 0, 0, 0);

        public final int port;
        public final boolean reversed;
        public final double kP;
        public final double kI;
        public final double kD;
        public final double kF;
        public final int I_Zone;
        public final double ramp_rate;
        public final int allow_error;

        /**
         * port, reversed, kP, kI, kD, kF, I_Zone, ramp_rate, allow_error
         */
        PortID(int port, boolean reversed, double kP, double kI, double kD, double kF, int I_Zone, double ramp_rate,
                int allow_error) {
            this.port = port;
            this.reversed = reversed;
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
            this.kF = kF;
            this.I_Zone = I_Zone;
            this.ramp_rate = ramp_rate;
            this.allow_error = allow_error;
        }
    }

    public static enum limelight_pipeline {
        aprilTag(0),
        reflective(1),
        cube(2),
        con(3),
        driver(4);

        public final int value;

        /**
         * port, reversed, kP, kI, kD, kF, I_Zone, ramp_rate, allow_error
         */
        limelight_pipeline(int value) {
            this.value = value;
        }
    }

    public final static int kTIMEOUT = 10;
    public final static double lift_open = 75000;
    public final static double lift_open_max = 50000;
    public final static double rack_open = -4;
    public final static double elevator_open = -6.3;
    public final static double elevator_middle = -3;
    public final static double elevator_tick2rotate = 5 * 2048;
    public final static double rack_tick2rotate = 3 * 2048;
    public final static double intakeAbsoluteEncoderOffsetTicks = 0;

    public static enum intake_position {
        intake_origin(0),
        intake_substation_cube(850),
        intake_straight(1250),
        intake_straight_con(1450),
        intake_floor(2250);

        public final double value;

        intake_position(double value) {
            this.value = value;
        }
    }

    public static final class ModuleConstants {
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
        public static final double kTurningEncoderTicksPerPulse2RPS = 1.0 / 409.6;
        public static final double kDriveEncoderTicksPerPulse2RPS = 1.0 / 204.8;
        public static final double kTurningEncoderTicks2Rot = 1.0 / 4096;
        public static final double kDriveMotorGearRatio = 1 / 6.55;
        public static final double kTurningMotorGearRatio = 1;
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
        public static final double kDriveEncoderRPS2MeterPerSec = kDriveEncoderRot2Meter;
        public static final double kTurningEncoderRPS2RadPerSec = kTurningEncoderRot2Rad;
        public static final double kPTurning = 0.3;

        public static final double kP=0.15;
        public static final double kI=0.0001;
        public static final double kD=3;
        public static final double kF=0.06;
        public static final double I_Zone=0;
        public static final double allow_error=3000;
    }

    public static final class DriveConstants {

        public static final double kTrackWidth = Units.inchesToMeters(21.25);
        // Distance between right and left wheels
        public static final double kWheelBase = Units.inchesToMeters(25.25);
        // Distance between front and back wheels
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2));

        public static final int kFrontLeftDriveMotorPort = 9;
        public static final int kBackLeftDriveMotorPort = 6;
        public static final int kFrontRightDriveMotorPort = 3;
        public static final int kBackRightDriveMotorPort = 0;

        public static final int kFrontLeftTurningMotorPort = 10;
        public static final int kBackLeftTurningMotorPort = 7;
        public static final int kFrontRightTurningMotorPort = 4;
        public static final int kBackRightTurningMotorPort = 1;

        public static final int kFrontLeftTurningAbsoluteEncoderPort = 11;
        public static final int kBackLeftTurningAbsoluteEncoderPort = 8;
        public static final int kFrontRightTurningAbsoluteEncoderPort = 5;
        public static final int kBackRightTurningAbsoluteEncoderPort = 2;

        public static final boolean kFrontLeftTurningReversed = false;
        public static final boolean kBackLeftTurningReversed = false;
        public static final boolean kFrontRightTurningReversed = false;
        public static final boolean kBackRightTurningReversed = false;

        public static final boolean kFrontLeftDriveReversed = false;
        public static final boolean kBackLeftDriveReversed = true;
        public static final boolean kFrontRightDriveReversed = true;
        public static final boolean kBackRightDriveReversed = false;//

        public static final boolean kFrontLeftTurningAbsoluteEncoderReversed = true;
        public static final boolean kBackLeftTurningAbsoluteEncoderReversed = true;
        public static final boolean kFrontRightTurningAbsoluteEncoderReversed = true;
        public static final boolean kBackRightTurningAbsoluteEncoderReversed = true;

        public static final double kFrontLeftTurningAbsoluteEncoderOffsetTicks = 3811;
        public static final double kBackLeftTurningAbsoluteEncoderOffsetTicks = 3860;
        public static final double kFrontRightTurningAbsoluteEncoderOffsetTicks = 2658.5;
        public static final double kBackRightTurningAbsoluteEncoderOffsetTicks = 3020.5;

        public static final double kPhysicalMaxSpeedMetersPerSecond = 4;
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 4 * 2 * Math.PI;

        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = //
                kPhysicalMaxAngularSpeedRadiansPerSecond / 4;
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;

    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 4;
        public static final double kMaxAngularSpeedRadiansPerSecond = //
                DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 10;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;
        public static final double kPXController = 1.5;
        public static final double kPYController = 1.5;
        public static final double kPThetaController = 2;

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //
                new TrapezoidProfile.Constraints(
                        kMaxAngularSpeedRadiansPerSecond,
                        kMaxAngularAccelerationRadiansPerSecondSquared);
    }

    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;

        public static final int kDriverYAxis = 1;
        public static final int kDriverXAxis = 0;
        public static final int kDriverRotAxis = 2;
        public static final int kDriverSpeedAxis = 3;
        public static final int kDriverFieldOrientedButtonIdx = 2;
        public static final int kResetHeading = 11;
        public static final int kDriverBrakeButtonIdx = 1;

        public static final int kAimCon = 3;
        public static final int kAimCube = 4;
        public static final int kAimAprilTag = 5;
        public static final int kAimReflective = 6;

        public static final double kDeadband = 0.1;
    }

    public static /* final */ class AutoAimConstants {
        public static /* final */ double kP_con_turn = 0.025;
        public static /* final */ double kP_con_x = 0.025;
        public static /* final */ double kP_con_y = 0.025;
        public static /* final */ double kP_cube_turn = 0.02;
        public static /* final */ double kP_cube_x = 0.025;
        public static /* final */ double kP_cube_y = 0.03;
        public static /* final */ double kP_AprilTag_turn = 0.01;
        public static /* final */ double kP_AprilTag_x = 0.2;
        public static /* final */ double kP_AprilTag_y = 0.2;
        public static /* final */ double kP_Reflective_turn = 0.025;
        public static /* final */ double kP_Reflective_x = 0.025;
        public static /* final */ double kP_Reflective_y = 0.025;

        public static /* final */ double con_floor_target_y = -22;
        public static /* final */ double con_substation_target_x = 0;
        public static /* final */ double con_substation_target_y = 0;
        public static /* final */ double cube_floor_target_y = -25;
        // public static /* final */ double cube_substation_target_y = 0;

        public static /* final */ double AprilTag_target_y = -19;
        public static /* final */ double Reflective_target_y = -6.5;
    }

    public static final class AprilTagConstants {
        public static final int red_left = 1;
        public static final int red_middle = 2;
        public static final int red_right = 3;
        public static final int blue_substation = 4;
        public static final int red_substation = 5;
        public static final int blue_left = 6;
        public static final int blue_middle = 7;
        public static final int blue_right = 8;
    }
}

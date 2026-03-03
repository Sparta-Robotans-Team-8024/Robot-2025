import math
import typing

import navx
import wpilib

from pathplannerlib.auto import AutoBuilder
from pathplannerlib.controller import PPHolonomicDriveController
from pathplannerlib.config import RobotConfig, PIDConstants

from commands2 import Subsystem
from wpilib import SPI, SmartDashboard
from wpilib import DriverStation
from wpimath import kinematics
from wpimath.filter import SlewRateLimiter
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.kinematics import (
    ChassisSpeeds,
    SwerveModuleState,
    SwerveDrive4Kinematics,
    SwerveDrive4Odometry,
)

from constants import DriveConstants
from subsystems.maxswervemodule import MAXSwerveModule
from Utils import swerveutils


class DriveSubsystem(Subsystem):
    def __init__(self) -> None:
        super().__init__()

        # Create MAXSwerveModules
        self.front_left = MAXSwerveModule(
            DriveConstants.kFrontLeftDrivingCanId,
            DriveConstants.kFrontLeftTurningCanId,
            DriveConstants.kFrontLeftChassisAngularOffset,
        )

        self.front_right = MAXSwerveModule(
            DriveConstants.kFrontRightDrivingCanId,
            DriveConstants.kFrontRightTurningCanId,
            DriveConstants.kFrontRightChassisAngularOffset,
        )

        self.rear_left = MAXSwerveModule(
            DriveConstants.kBackLeftDrivingCanId,
            DriveConstants.kBackLeftTurningCanId,
            DriveConstants.kBackLeftChassisAngularOffset,
        )

        self.rear_right = MAXSwerveModule(
            DriveConstants.kBackRightDrivingCanId,
            DriveConstants.kBackRightTurningCanId,
            DriveConstants.kBackRightChassisAngularOffset,
        )

        # The gyro sensor
        self.gyro = navx.AHRS(navx.AHRS.NavXComType.kMXP_SPI)

        # Slew rate filter variables for controlling lateral acceleration
        self.current_rotation = 0.0
        self.current_translation_dir = 0.0
        self.current_translation_mag = 0.0

        self.mag_limiter = SlewRateLimiter(DriveConstants.kMagnitudeSlewRate)
        self.rot_limiter = SlewRateLimiter(DriveConstants.kRotationalSlewRate)
        self.prev_time = wpilib.Timer.getFPGATimestamp()

        # Odometry class for tracking robot pose
        self.odometry = SwerveDrive4Odometry(
            DriveConstants.kDriveKinematics,
            Rotation2d.fromDegrees(self.gyro.getAngle()),
            (
                self.front_left.getPosition(),
                self.front_right.getPosition(),
                self.rear_left.getPosition(),
                self.rear_right.getPosition(),
            ),
        )

        config = RobotConfig.fromGUISettings()

        # Configure the AutoBuilder last
        AutoBuilder.configure(
            self.get_pose, # Robot pose supplier
            self.reset_odometry, # Method to reset odometry (will be called if your auto has a starting pose)
            self.get_chassis_speeds, # ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            lambda speeds, feedforwards: self.drive_chassis_speeds(speeds), # Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also outputs individual module feedforwards
            PPHolonomicDriveController( # PPHolonomicController is the built in path following controller for holonomic drive trains
                PIDConstants(5.0, 0.0, 0.0), # Translation PID constants
                PIDConstants(5.0, 0.0, 0.0) # Rotation PID constants
            ),
            config, # The robot configuration
            self.shouldFlipPath, # Supplier to control path flipping based on alliance color
            self # Reference to this subsystem to set requirements
        )


    def periodic(self) -> None:
        # Update the odometry in the periodic block
        self.odometry.update(
            Rotation2d.fromDegrees(self.gyro.getAngle()),
            (
                self.front_left.getPosition(),
                self.front_right.getPosition(),
                self.rear_left.getPosition(),
                self.rear_right.getPosition(),
            ),
        )

    def get_pose(self) -> Pose2d:
        """Returns the currently-estimated pose of the robot.

        :returns: The pose.
        """
        return self.odometry.getPose()
    
    def shouldFlipPath(self) -> bool:
        # Boolean supplier that controls when the path will be mirrored for the red alliance
        # This will flip the path being followed to the red side of the field.
        # THE ORIGIN WILL REMAIN ON THE BLUE SIDE
        return DriverStation.getAlliance() == DriverStation.Alliance.kRed

    def reset_odometry(self, pose: Pose2d) -> None:
        """Resets the odometry to the specified pose.

        :param pose: The pose to which to set the odometry.

        """
        self.odometry.resetPosition(
            Rotation2d.fromDegrees(self.gyro.getAngle()),
            (
                self.front_left.getPosition(),
                self.front_right.getPosition(),
                self.rear_left.getPosition(),
                self.rear_right.getPosition(),
            ),
            pose,
        )

    def drive(
        self,
        x_speed: float,
        y_speed: float,
        rot: float,
        field_relative: bool,
        rate_limit: bool,
    ) -> None:
        """Method to drive the robot using joystick info.

        :param x_speed:        Speed of the robot in the x direction (forward).
        :param y_speed:        Speed of the robot in the y direction (sideways).
        :param rot:           Angular rate of the robot.
        :param field_relative: Whether the provided x and y speeds are relative to the
                              field.
        :param rate_limit:     Whether to enable rate limiting for smoother control.
        """

        x_speed_commanded = x_speed
        y_speed_commanded = y_speed

        if rate_limit:
            # Convert XY to polar for rate limiting
            input_translation_dir = math.atan2(y_speed, x_speed)
            input_translation_mag = math.hypot(x_speed, y_speed)

            # Calculate the direction slew rate based on an estimate of the lateral acceleration
            if self.current_translation_mag != 0.0:
                direction_slew_rate = abs(
                    DriveConstants.kDirectionSlewRate / self.current_translation_mag
                )
            else:
                direction_slew_rate = 500.0
                # some high number that means the slew rate is effectively instantaneous

            current_time = wpilib.Timer.getFPGATimestamp()
            elapsed_time = current_time - self.prev_time
            angle_dif = swerveutils.angleDifference(
                input_translation_dir, self.current_translation_dir
            )
            if angle_dif < 0.45 * math.pi:
                self.current_translation_dir = swerveutils.stepTowardsCircular(
                    self.current_translation_dir,
                    input_translation_dir,
                    direction_slew_rate * elapsed_time,
                )
                self.current_translation_mag = self.mag_limiter.calculate(
                    input_translation_mag
                )

            elif angle_dif > 0.85 * math.pi:
                # some small number to avoid floating-point errors with equality checking
                # keep currentTranslationDir unchanged
                if self.current_translation_mag > 1e-4:
                    self.current_translation_mag = self.mag_limiter.calculate(0.0)
                else:
                    self.current_translation_dir = swerveutils.wrapAngle(
                        self.current_translation_dir + math.pi
                    )
                    self.current_translation_mag = self.mag_limiter.calculate(
                        input_translation_mag
                    )

            else:
                self.current_translation_dir = swerveutils.stepTowardsCircular(
                    self.current_translation_dir,
                    input_translation_dir,
                    direction_slew_rate * elapsed_time,
                )
                self.current_translation_mag = self.mag_limiter.calculate(0.0)

            self.prev_time = current_time

            x_speed_commanded = self.current_translation_mag * math.cos(
                self.current_translation_dir
            )
            y_speed_commanded = self.current_translation_mag * math.sin(
                self.current_translation_dir
            )
            self.current_rotation = self.rot_limiter.calculate(rot)

        else:
            self.current_rotation = rot

        # Convert the commanded speeds into the correct units for the drivetrain
        x_speed_delivered = x_speed_commanded * DriveConstants.kMaxSpeedMetersPerSecond
        y_speed_delivered = y_speed_commanded * DriveConstants.kMaxSpeedMetersPerSecond
        rot_delivered = self.current_rotation * DriveConstants.kMaxAngularSpeed

        swerve_module_states = DriveConstants.kDriveKinematics.toSwerveModuleStates(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                x_speed_delivered,
                y_speed_delivered,
                rot_delivered,
                Rotation2d.fromDegrees(self.gyro.getAngle()),
            )
            if field_relative
            else ChassisSpeeds(x_speed_delivered, y_speed_delivered, rot_delivered)
        )
        fl, fr, rl, rr = SwerveDrive4Kinematics.desaturateWheelSpeeds(
            swerve_module_states, DriveConstants.kMaxSpeedMetersPerSecond
        )
        self.front_left.setDesiredState(fl)
        self.front_right.setDesiredState(fr)
        self.rear_left.setDesiredState(rl)
        self.rear_right.setDesiredState(rr)

    def get_chassis_speeds(self) -> ChassisSpeeds:
        return DriveConstants.kDriveKinematics.toChassisSpeeds(
            [self.front_left.getState(),
             self.front_right.getState(),
             self.rear_left.getState(),
             self.rear_right.getState()]
        )

    def drive_chassis_speeds(self, chassis_speeds: ChassisSpeeds):
        # SmartDashboard.putNumber("Swerve/Translation X", translation.x)
        # SmartDashboard.putNumber("Swerve/Translation Y", translation.y)
        # SmartDashboard.putNumber("Swerve/Rotation", rotation)
        # SmartDashboard.putBoolean("Swerve/With PID", False)
        module_states = DriveConstants.kDriveKinematics.toSwerveModuleStates(
            chassis_speeds
        )
        fl, fr, rl, rr = SwerveDrive4Kinematics.desaturateWheelSpeeds(
            module_states, DriveConstants.kMaxSpeedMetersPerSecond
        )

        self.front_left.setDesiredState(fl)
        self.front_right.setDesiredState(fr)
        self.rear_left.setDesiredState(rl)
        self.rear_right.setDesiredState(rr)

    def set_x(self) -> None:
        """Sets the wheels into an X formation to prevent movement."""
        self.front_left.setDesiredState(SwerveModuleState(0, Rotation2d.fromDegrees(45)))
        self.front_right.setDesiredState(
            SwerveModuleState(0, Rotation2d.fromDegrees(-45))
        )
        self.rear_left.setDesiredState(SwerveModuleState(0, Rotation2d.fromDegrees(-45)))
        self.rear_right.setDesiredState(SwerveModuleState(0, Rotation2d.fromDegrees(45)))

    def set_module_states(
        self,
        desired_states: typing.Tuple[
            SwerveModuleState, SwerveModuleState, SwerveModuleState, SwerveModuleState
        ],
    ) -> None:
        """Sets the swerve ModuleStates.

        :param desired_states: The desired SwerveModule states.
        """
        fl, fr, rl, rr = SwerveDrive4Kinematics.desaturateWheelSpeeds(
            desired_states, DriveConstants.kMaxSpeedMetersPerSecond
        )
        self.front_left.setDesiredState(fl)
        self.front_right.setDesiredState(fr)
        self.rear_left.setDesiredState(rl)
        self.rear_right.setDesiredState(rr)

    def reset_encoders(self) -> None:
        """Resets the drive encoders to currently read a position of 0."""
        self.front_left.resetEncoders()
        self.rear_left.resetEncoders()
        self.front_right.resetEncoders()
        self.rear_right.resetEncoders()

    def zero_heading(self) -> None:
        """Zeroes the heading of the robot."""
        self.gyro.reset()

    def get_heading(self) -> float:
        """Returns the heading of the robot.

        :returns: the robot's heading in degrees, from -180 to 180
        """
        return Rotation2d.fromDegrees(self.gyro.getYaw()).degrees()

    def get_turn_rate(self) -> float:
        """Returns the turn rate of the robot.

        :returns: The turn rate of the robot, in degrees per second
        """
        return self.gyro.getRate() * (-1.0 if DriveConstants.kGyroReversed else 1.0)
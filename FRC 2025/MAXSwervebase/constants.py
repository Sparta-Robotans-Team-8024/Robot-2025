import math

from wpimath import units
from wpimath.geometry import Translation2d
from wpimath.kinematics import SwerveDrive4Kinematics
from wpimath.trajectory import TrapezoidProfileRadians
import robotpy_apriltag
import wpimath.geometry
from photonlibpy.photonPoseEstimator import PoseStrategy

from rev import SparkMax, SparkMaxConfig, SparkBaseConfig

class NeoMotorConstants:
    kFreeSpeedRPM = 5676 # RPM = Rotations per minute

class VisionConstants:
    kcameraname = "Cam1"
    #kfield = robotpy_apriltag.AprilTagFieldLayout(robotpy_apriltag.AprilTagField.k2025Reefscape)
    krotation = wpimath.geometry.Rotation3d()
    krotation2 = wpimath.geometry.Rotation2d()
    ktransform = wpimath.geometry.Transform3d(0,0,0,krotation)
    kstrategy = PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR

class DeepHangConstants:
    kPressCanId1 = 3


class AlgaeConstants:
    kIntakeCanId = 4
    kPivotCanId = 5

    kForward = 0.5
    kReverse = -0.6
    kHold = 0.25

    kStow = 18.5
    kHoldPower = 0.25
    kDown = -0.1
    kUp = 0.2

class CoralConstants:
    kIntakeCanId = 6

    kForward = 0.1
    kReverse = -0.3

    kAutoI = 0.1
    kAutoR = -0.2

    kStop = 0
    
class DriveConstants:

    # Define allowed maxium capable speed
    kMaxSpeedMetersPerSecond = 4.8
    kMaxAngularSpeed = math.tau # Radians per second

    kDirectionSlewRate = 1.2 # Radians per second
    kMagnitudeSlewRate = 1.8 # Radians per second
    kRotationalSlewRate = 2.0 # Radians per second

    # Chassis Config
    kTrackWidth = units.inchesToMeters(26.5)
    
    # Distance between front and back wheels on the robot
    kWheelBase = units.inchesToMeters(26.5)

    # Distance betweem front and back wheels on the robot
    kModulePositions = [
        Translation2d(kWheelBase / 2, kTrackWidth / 2),
        Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
    ]
    kDriveKinematics = SwerveDrive4Kinematics(*kModulePositions)

    # Angular offsets of the module relative to the chassis in radians
    kFrontLeftChassisAngularOffset = 0 #5.811 
    kFrontRightChassisAngularOffset = 0 #4.322 
    kBackLeftChassisAngularOffset = 0 #2.850 
    kBackRightChassisAngularOffset = 0 #1.538 

    # SPARK MAX CAN IDs
    kFrontLeftDrivingCanId = 11 # w/ Front Left Turning Motor
    kBackLeftDrivingCanId = 13 # w/ Back Left Turning Motor
    kFrontRightDrivingCanId = 15 # w/ Front Right Turning Motor 
    kBackRightDrivingCanId = 17 # w/ Back Right Turning Motor

    kFrontLeftTurningCanId = 10 
    kBackLeftTurningCanId = 12
    kFrontRightTurningCanId = 14
    kBackRightTurningCanId = 16

    kGyroReversed = False

class ModuleConstants:
    # The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    # This changes the drive speed of the module (a pinion gear with more teeth will result in a
    # robot that drives faster).
    kDrivingMotorPinionTeeth = 14

    # Invert the turning encoder, since the output shaft rotates in the opposite direction of
    # the steering motor in the MAXSwerve Module.
    kTurningEncoderInverted = True

    # Calculations required for driving motor conversion factors and feed forward
    kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRPM / 60 # Rps = Rotations per minute
    kWheelDiameters = 0.0762
    kWheelCircumferenceMeters = kWheelDiameters * math.pi
    
    # 45 Teeth on wheel's bevel gear, 222 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
    kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15)
    kDriveWheelFreeSpeedRps  = (
        kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters
    ) / kDrivingMotorReduction

    kDrivingEncoderPositionFactor = (
        kWheelDiameters * math.pi
    ) / kDrivingMotorReduction # meters
    kDrivingEncoderVelocityFactor = (
        (kWheelDiameters * math.pi) / kDrivingMotorReduction
    ) / 60.0 # Meters per second

    kTurningEncoderPositionFactor = math.tau # Radian
    kTurningEncoderVelocityFactor = math.tau / 60.0 # Radian

    kTurningEncoderPositionPIDMinInput = 0 # Radian
    kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor # Radian

    kDrivingP = 0.04
    kDrivingI = 0
    kDrivingD = 0
    kDrivingFF = 1 / kDriveWheelFreeSpeedRps
    kDrivingMinOutput = -1
    kDrivingMaxOutput = 1

    kDrivingMotorCurrentLimit = 60  # amp 
    kTurningMotorCurrentLimit = 40  # amp

    kDrivingP = 0.04
    kDrivingI = 0
    kDrivingD = 0
    kDrivingFF = 1 / kDriveWheelFreeSpeedRps
    kDrivingMinOutput = -1
    kDrivingMaxOutput = 1

    kTurningP = 1
    kTurningI = 0
    kTurningD = 0
    kTurningFF = 0
    kTurningMinOutput = -1
    kTurningMaxOutput = 1

    kDrivingMotorIdleMode = SparkBaseConfig.IdleMode.kBrake
    kTurningMotorIdleMode = SparkBaseConfig.IdleMode.kBrake

    kDrivingMotorCurrentLimit = 50 
    kTurningMotorCurrentLimit = 20

class OIConstants:
    kDriverControllerPort = 0 
    kButtonControllerPort = 1 
    kDriverDeadband = 0.05

    kTriggerButtonThreshold = 0.2

class AutoConstants:
    kMaxSpeedMetersPerSecond = 3
    kMaxAccelerationMetersPerSecond = 3
    kMaxAngularSpeedRadiansPerSecond = math.pi
    kMaxAngularSpeedRadiansPerSecondSquared = math.pi

    kPXController = 2
    kPYController = 2
    kPThetaController = 2
    
    # Contraint for the motion profiled robot angle controller
    kPThetaControllerContraints = TrapezoidProfileRadians.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared
    )

    

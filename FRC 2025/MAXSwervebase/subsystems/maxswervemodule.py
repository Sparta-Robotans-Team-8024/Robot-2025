from rev import SparkMax, SparkMaxConfig, SparkBase, ClosedLoopConfig
from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModuleState, SwerveModulePosition

from constants import ModuleConstants


class MAXSwerveModule:
    def __init__(
        self, drivingCANId: int, turningCANId: int, chassisAngularOffset: float
    ) -> None:
        """Constructs a MAXSwerveModule and configures the driving and turning motor,
        encoder, and PID controller. This configuration is specific to the REV
        MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
        Encoder.
        """
        self.desiredState = SwerveModuleState(0.0, Rotation2d())

        self.drivingSparkMax = SparkMax(
            drivingCANId, SparkMax.MotorType.kBrushless
            )
        self.turningSparkMax = SparkMax(
            turningCANId, SparkMax.MotorType.kBrushless
        )
        self.driving_config = SparkMaxConfig()
        self.turning_config = SparkMaxConfig()

        # Factory reset, so we get the SPARKS MAX to a known state before configuring
        # them. This is useful in case a SPARK MAX is swapped out.
        # self.drivingSparkMax.restoreFactoryDefaults()
        # self.turningSparkMax.restoreFactoryDefaults()

        # Setup encoders and PID controllers for the driving and turning SPARKS MAX.
        self.driving_encoder = self.drivingSparkMax.getEncoder()
        self.turning_encoder = self.turningSparkMax.getAbsoluteEncoder(
            # SparkMaxAbsoluteEncoder.Type.kDutyCycle
        )
        self.driving_pid_controller = self.drivingSparkMax.getClosedLoopController()
        self.turning_pid_controller = self.turningSparkMax.getClosedLoopController()
        self.driving_config.closedLoop.setFeedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
        self.turning_config.closedLoop.setFeedbackSensor(ClosedLoopConfig.FeedbackSensor.kAbsoluteEncoder)

        # Apply position and velocity conversion factors for the driving encoder. The
        # native units for position and velocity are rotations and RPM, respectively,
        # but we want meters and meters per second to use with WPILib's swerve APIs.
        self.driving_config.encoder.positionConversionFactor(
            ModuleConstants.kDrivingEncoderPositionFactor
        )
        self.driving_config.encoder.velocityConversionFactor(
            ModuleConstants.kDrivingEncoderVelocityFactor
        )

        # Apply position and velocity conversion factors for the turning encoder. We
        # want these in radians and radians per second to use with WPILib's swerve
        # APIs.
        self.turning_config.absoluteEncoder.positionConversionFactor(
            ModuleConstants.kTurningEncoderPositionFactor
        )
        self.turning_config.absoluteEncoder.velocityConversionFactor(
            ModuleConstants.kTurningEncoderVelocityFactor
        )

        # Invert the turning encoder, since the output shaft rotates in the opposite direction of
        # the steering motor in the MAXSwerve Module.
        self.turning_config.absoluteEncoder.inverted(ModuleConstants.kTurningEncoderInverted)

        # Enable PID wrap around for the turning motor. This will allow the PID
        # controller to go through 0 to get to the setpoint i.e. going from 350 degrees
        # to 10 degrees will go through 0 rather than the other direction which is a
        # longer route.
        self.turning_config.closedLoop.positionWrappingEnabled(True)
        self.turning_config.closedLoop.positionWrappingMinInput(
            ModuleConstants.kTurningEncoderPositionPIDMinInput
        )
        self.turning_config.closedLoop.positionWrappingMaxInput(
            ModuleConstants.kTurningEncoderPositionPIDMaxInput
        )

        # Set the PID gains for the driving motor. Note these are example gains, and you
        # may need to tune them for your own robot!
        self.driving_config.closedLoop.P(ModuleConstants.kDrivingP)
        self.driving_config.closedLoop.I(ModuleConstants.kDrivingI)
        self.driving_config.closedLoop.D(ModuleConstants.kDrivingD)
        self.driving_config.closedLoop.velocityFF(ModuleConstants.kDrivingFF)
        self.driving_config.closedLoop.outputRange(
            ModuleConstants.kDrivingMinOutput, ModuleConstants.kDrivingMaxOutput
        )

        # Set the PID gains for the turning motor. Note these are example gains, and you
        # may need to tune them for your own robot!
        self.turning_config.closedLoop.P(ModuleConstants.kTurningP)
        self.turning_config.closedLoop.I(ModuleConstants.kTurningI)
        self.turning_config.closedLoop.D(ModuleConstants.kTurningD)
        self.turning_config.closedLoop.velocityFF(ModuleConstants.kTurningFF)
        self.turning_config.closedLoop.outputRange(
            ModuleConstants.kTurningMinOutput, ModuleConstants.kTurningMaxOutput
        )

        self.driving_config.setIdleMode(ModuleConstants.kDrivingMotorIdleMode)
        self.turning_config.setIdleMode(ModuleConstants.kTurningMotorIdleMode)
        self.driving_config.smartCurrentLimit(
            ModuleConstants.kDrivingMotorCurrentLimit
        )
        self.turning_config.smartCurrentLimit(
            ModuleConstants.kTurningMotorCurrentLimit
        )

        # Save the SPARK MAX configurations. If a SPARK MAX browns out during
        # operation, it will maintain the above configurations.
        self.drivingSparkMax.configure(self.driving_config,
                                          SparkBase.ResetMode.kResetSafeParameters,
                                          SparkBase.PersistMode.kPersistParameters)
        self.turningSparkMax.configure(self.turning_config,
                                         SparkBase.ResetMode.kResetSafeParameters,
                                         SparkBase.PersistMode.kPersistParameters)

        self.chassis_angular_offset = chassisAngularOffset
        self.desiredState.angle = Rotation2d(self.turning_encoder.getPosition())
        self.driving_encoder.setPosition(0)

    def getState(self) -> SwerveModuleState:
        """Returns the current state of the module.

        :returns: The current state of the module.
        """
        # Apply chassis angular offset to the encoder position to get the position
        # relative to the chassis.
        return SwerveModuleState(
            self.driving_encoder.getVelocity(),
            Rotation2d(self.turning_encoder.getPosition() - self.chassis_angular_offset),
        )
    
    def getPosition(self) -> SwerveModulePosition:
        """Returns the current position of the module.

        :returns: The current position of the module.
        """
        # Apply chassis angular offset to the encoder position to get the position
        # relative to the chassis.
        return SwerveModulePosition(
            self.driving_encoder.getPosition(),
            Rotation2d(self.turning_encoder.getPosition() - self.chassis_angular_offset),
        )
    
    def setDesiredState(self, desiredState: SwerveModuleState) -> None:
        """Sets the desired state for the module.

        :param desiredState: Desired state with speed and angle.

        """
        # Apply chassis angular offset to the desired state.
        correctedDesiredState = SwerveModuleState()
        correctedDesiredState.speed = desiredState.speed
        correctedDesiredState.angle = desiredState.angle + Rotation2d(
            self.chassis_angular_offset
        )

        # Optimize the reference state to avoid spinning further than 90 degrees.
        optimizedDesiredState = SwerveModuleState.optimize(
            correctedDesiredState, Rotation2d(self.turning_encoder.getPosition())
        )

        # Command driving and turning SPARKS MAX towards their respective setpoints.
        self.driving_pid_controller.setReference(
            correctedDesiredState.speed, SparkMax.ControlType.kVelocity
        )
        self.turning_pid_controller.setReference(
            correctedDesiredState.angle.radians(), SparkMax.ControlType.kPosition
        )

        self.desiredState = desiredState

    def resetEncoders(self) -> None:
        """
        Zeroes all the SwerveModule encoders.
        """
        self.driving_encoder.setPosition(0)

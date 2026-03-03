import math

import commands2
import wpilib
import wpimath

from constants import OIConstants
from subsystems.drivesubsystem import DriveSubsystem


class DriveCommand(commands2.Command):

    def __init__(self, drive_sub: DriveSubsystem) -> None:
        super().__init__()

        self.driver_controller = commands2.button.CommandXboxController(OIConstants.kDriverControllerPort)
        self.drive_sub = drive_sub
        self.addRequirements(self.drive_sub)

    def execute(self) -> None:
        forward = self.driver_controller.getLeftY()
        strafe = self.driver_controller.getLeftX()

        gyro_degrees = self.drive_sub.get_heading()
        gyro_radians = gyro_degrees * math.pi/180
        temp = forward * math.cos(gyro_radians) + strafe * math.sin(gyro_radians)
        strafe = -forward * math.sin(gyro_radians) + strafe * math.cos(gyro_radians)
        fwd = temp

        '''self.drive_sub.drive(
            -wpimath.applyDeadband(
                (-self.driver_controller.getRawAxis(1) * math.cos(self.drive_sub.get_heading() * (math.pi / 180))) +
                (self.driver_controller.getRawAxis(0) * math.sin(self.drive_sub.get_heading() * (math.pi / 180))),
                OIConstants.kDriverDeadband
            ),
            -wpimath.applyDeadband(
                (self.driver_controller.getRawAxis(1) * math.sin(self.drive_sub.get_heading() * (math.pi / 180))) +
                (self.driver_controller.getRawAxis(0) * math.cos(self.drive_sub.get_heading() * (math.pi / 180))),
                OIConstants.kDriverDeadband
            ),
            -wpimath.applyDeadband(
                self.driver_controller.getRawAxis(4), OIConstants.kDriverDeadband
            ),
            True,
            True,
        )'''

        self.drive_sub.drive(
            -wpimath.applyDeadband(
                self.driver_controller.getLeftY(), OIConstants.kDriverDeadband
            ),
            -wpimath.applyDeadband(
                self.driver_controller.getLeftX(), OIConstants.kDriverDeadband
            ),
            -wpimath.applyDeadband(
                self.driver_controller.getRightX(), OIConstants.kDriverDeadband
            ),
            False,
            True,
        )

    def isFinished(self) -> bool:
        return False

    def end(self, interrupted: bool) -> None:
        self.drive_sub.drive(
            0,
            0,
            0,
            True,
            True,
        )
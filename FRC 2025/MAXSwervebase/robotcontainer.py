import math

import commands2
import wpilib

from pathplannerlib.auto import AutoBuilder

from commands2 import cmd
from wpilib import SmartDashboard
from wpimath.controller import PIDController, ProfiledPIDControllerRadians, HolonomicDriveController
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.trajectory import TrajectoryConfig, TrajectoryGenerator

from commands.drivecommand import DriveCommand
from constants import OIConstants, AutoConstants, DriveConstants, AlgaeConstants, CoralConstants
from subsystems.drivesubsystem import DriveSubsystem
from subsystems.deephangsubsystem import DeepHangSubsystem
from subsystems.algaemanipulator import AlgaeManipulator
from subsystems.algaepivot import AlgaePivot
from subsystems.coralsubsystem import CoralSubsystem
from pathplannerlib.auto import NamedCommands



class RobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.
    """

    def __init__(self) -> None:
        # The robot's subsystems
        self.drive_subsystem = DriveSubsystem()
        self.deepHang = DeepHangSubsystem()
        self.Algae = AlgaeManipulator()
        self.Pivot = AlgaePivot()
        self.Coral = CoralSubsystem()


        # The driver's controller
        self.driver_controller = commands2.button.CommandXboxController(OIConstants.kDriverControllerPort)
        self.buttoncontroller = commands2.button.CommandXboxController(OIConstants.kButtonControllerPort)


        # named commands
        NamedCommands.registerCommand("Score", self.Coral.run_reverse_auto())
        NamedCommands.registerCommand("Stop", self.Coral.stop())

        # Configure the button bindings
        self.configure_button_bindings()

        # Configure default commands
        self.drive_subsystem.setDefaultCommand(
            DriveCommand(self.drive_subsystem)
        )

        # Build Your Autos Here

        # Leave Top
        '''self.TopL1 = AutoBuilder.buildAuto("Leave Auto 1 (Top)")
        self.TopL2 = AutoBuilder.buildAuto("Leave Auto 2 (Top)")
        self.TopL3 = AutoBuilder.buildAuto("Leave Auto 3 (Top)")
        self.TopL4 = AutoBuilder.buildAuto("Leave Auto 4 (Top)")
        self.TopL5 = AutoBuilder.buildAuto("Leave Auto 5 (Top)")

        # Leave Bottom
        self.BottomL1 = AutoBuilder.buildAuto("Leave Auto 1 (Bottom)")
        self.BottomL2 = AutoBuilder.buildAuto("Leave Auto 2 (Bottom)")
        self.BottomL3 = AutoBuilder.buildAuto("Leave Auto 3 (Bottom)")
        self.BottomL4 = AutoBuilder.buildAuto("Leave Auto 4 (Bottom)")
        self.BottomL5 = AutoBuilder.buildAuto("Leave Auto 5 (Bottom)")
        
        # Leave Center
        self.CenterL1 = AutoBuilder.buildAuto("Leave Auto 1 (Center)")
        self.CenterL2 = AutoBuilder.buildAuto("Leave Auto 2 (Center)")
        self.CenterL3 = AutoBuilder.buildAuto("Leave Auto 3 (Center)")
        self.CenterL4 = AutoBuilder.buildAuto("Leave Auto 4 (Center)")
        self.CenterL5 = AutoBuilder.buildAuto("Leave Auto 5 (Center)")

        # 1 Coral 
        self.CoralCenter1 = AutoBuilder.buildAuto("1 Coral Auto (Center)")
        self.CoralTop1 = AutoBuilder.buildAuto("1 Coral Auto (Top)")
        self.CoralBottom1 = AutoBuilder.buildAuto("1 Coral Auto (Bottom)")

        # 2 Coral
        self.CoralTop2 = AutoBuilder.buildAuto("2 Coral Auto (Top)")
        self.CoralCenter2 = AutoBuilder.buildAuto("2 Coral Auto 1 (Center)")
        self.CoralCenter3 = AutoBuilder.buildAuto("2 Coral Auto 2 (Center)")'''

        self.CoralCenter1 = AutoBuilder.buildAuto("1 Coral Auto (Center)")

        self.autoChooser = AutoBuilder.buildAutoChooser()

        SmartDashboard.putData("Auto Chooser", self.autoChooser)

        # Auto Options
        ''''self.autoChooser.setDefaultOption("Leave Auto 1 (Top)", self.TopL1)
        self.autoChooser.addOption("Leave Auto 2 (Top)", self.TopL2)
        self.autoChooser.addOption("Leave Auto 3 (Top)", self.TopL3)
        self.autoChooser.addOption("Leave Auto 4 (Top)", self.TopL4)
        self.autoChooser.addOption("Leave Auto 5 (Top)", self.TopL5)

        self.autoChooser.addOption("Leave Auto 1 (Bottom)", self.BottomL1)
        self.autoChooser.addOption("Leave Auto 2 (Bottom)", self.BottomL2)
        self.autoChooser.addOption("Leave Auto 3 (Bottom)", self.BottomL3)
        self.autoChooser.addOption("Leave Auto 4 (Bottom)", self.BottomL4)
        self.autoChooser.addOption("Leave Auto 5 (Bottom)", self.BottomL5)

        self.autoChooser.addOption("Leave Auto 1 (Center)", self.CenterL1)
        self.autoChooser.addOption("Leave Auto 2 (Center)", self.CenterL2)
        self.autoChooser.addOption("Leave Auto 3 (Center)", self.CenterL3)
        self.autoChooser.addOption("Leave Auto 4 (Center)", self.CenterL4)
        self.autoChooser.addOption("Leave Auto 5 (Center)", self.CenterL5)

        self.autoChooser.addOption("1 Coral Auto (Center)", self.CoralCenter1)
        self.autoChooser.addOption("1 Coral Auto (Top)", self.CoralTop1)
        self.autoChooser.addOption("1 Coral Auto (Bottom)", self.CoralBottom1)

        self.autoChooser.addOption("2 Coral Auto (Top)", self.CoralTop2)
        self.autoChooser.addOption("2 Coral Auto (Center to Bottom)", self.CoralCenter2)
        self.autoChooser.addOption("2 Coral Auto (Center to Top)", self.CoralCenter3)'''

        self.autoChooser.setDefaultOption("1 Coral Auto (Center)", self.CoralCenter1)

    def configure_button_bindings(self) -> None:
        """
        Use this method to define your button->command mappings. Buttons can be created by
        instantiating a :GenericHID or one of its subclasses (Joystick or XboxController),
        and then passing it to a JoystickButton.
        """

        # Deep Hang
        climb_up_command = commands2.StartEndCommand(lambda: self.deepHang.set_motor_voltage(2.5), self.deepHang.stop_motor, self.deepHang)
        climb_down_command = commands2.StartEndCommand(lambda: self.deepHang.set_motor_voltage(-5.5), self.deepHang.stop_motor, self.deepHang)
        
        self.driver_controller.x().whileTrue(climb_up_command)
        self.driver_controller.y().whileTrue(climb_down_command)

        # Pivot arm manual control
        algae_up_command = commands2.StartEndCommand(lambda: self.Pivot.armMotor.set(0.6), lambda: self.Pivot.armMotor.set(0), self.Pivot)
        algae_down_command = commands2.StartEndCommand(lambda: self.Pivot.armMotor.set(-0.4), lambda: self.Pivot.armMotor.set(0), self.Pivot)

        self.driver_controller.a().whileTrue(algae_up_command)
        self.driver_controller.b().whileTrue(algae_down_command)

        # Right trigger -> Intake forward while held (Collect)
        self.driver_controller.rightTrigger(OIConstants.kTriggerButtonThreshold).whileTrue(
            commands2.StartEndCommand(
            lambda: self.Algae.set_intake_power(AlgaeConstants.kForward),
            lambda: self.Algae.set_intake_power(0.0),
            self.Algae
            )
        )

        # Left trigger -> Intake reverse while held (Score)
        self.driver_controller.leftTrigger(OIConstants.kTriggerButtonThreshold).whileTrue(
            commands2.StartEndCommand(
            lambda: self.Algae.set_intake_power(AlgaeConstants.kReverse),
            lambda: self.Algae.set_intake_power(0.0),
            self.Algae
            )
        )

        # Right Bumber -> Score Coral
        self.driver_controller.rightBumper().whileTrue(
            commands2.StartEndCommand(
                lambda: self.Coral.set_intake_power(CoralConstants.kReverse),
                lambda: self.Coral.set_intake_power(0.0),
                self.Coral
            )
        )

        # Left Bumber -> Eject
        self.driver_controller.leftBumper().whileTrue(
            commands2.StartEndCommand(
                lambda: self.Coral.set_intake_power(CoralConstants.kForward),
                lambda: self.Coral.set_intake_power(0.0),
                self.Coral
            )
        )

    def disable_pid_subsystems(self) -> None:
        """Disables all ProfiledPIDSubsystem and PIDSubsystem instances.
        This should be called on robot disable to prevent integral windup."""

    def get_autonomous_command(self) -> commands2.Command:
        """Use this to pass the autonomous command to the main {@link Robot} class.

        :returns: the command to run in autonomous
        """
        return self.autoChooser.getSelected()
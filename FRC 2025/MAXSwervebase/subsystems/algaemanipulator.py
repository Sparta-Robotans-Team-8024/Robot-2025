import wpilib
import rev
import commands2

from wpilib import SmartDashboard

from constants import AlgaeConstants



class AlgaeManipulator(commands2.Subsystem):
    def __init__(self):
        super().__init__()

        self.intakeMotor = rev.SparkFlex(AlgaeConstants.kIntakeCanId, rev.SparkFlex.MotorType.kBrushless)
        intakeConfig = rev.SparkFlexConfig()
        self.intakeMotor.configure(intakeConfig, rev.SparkBase.ResetMode.kResetSafeParameters, rev.SparkBase.PersistMode.kPersistParameters)
        
        self.intakeController = self.intakeMotor.getClosedLoopController()
        self.intakeEncoder = self.intakeMotor.getEncoder()

        self.intakeMotor.IdleMode(rev.SparkFlex.IdleMode.kBrake)

        self.stow_when_idle = True
        #self.was_reset = False

        self.mechanism2d = wpilib.Mechanism2d(50, 50)
        self.mechanism_root = self.mechanism2d.getRoot('Ball Intake Root', 28, 3)

        SmartDashboard.putData('Algae Subsystem', self.mechanism2d)

    def zero_on_user_button(self):
        if not self.was_reset and wpilib.RobotController.getUserButton():
            self.was_reset = True
        elif not wpilib.RobotController.getUserButton():
            self.was_reset = False

    def set_intake_power(self, power):
        self.intakeMotor.set(power)

    def set_intake_position(self, position):
        self.armController.setReference(position, rev.SparkFlex.ControlType.kPosition)

    def run_intake_command(self):
        def execute():
            self.set_intake_power(AlgaeConstants.kForward)
            self.set_intake_position(AlgaeConstants.kDown)

        return commands2.InstantCommand(execute, self)

    def reverse_intake_command(self):
        def execute():
            self.set_intake_power(AlgaeConstants.kReverse)
            self.set_intake_position(AlgaeConstants.kHold)

        return commands2.InstantCommand(execute, self)
    

    def idle_command(self):
        def execute():
            if self.stow_when_idle:
                self.set_intake_power(0.0)
                self.set_intake_position(AlgaeConstants.kStow)
            else:
                self.set_intake_power(AlgaeConstants.kHold)
                self.set_intake_position(AlgaeConstants.kHold)

        return commands2.InstantCommand(execute, self)

    def periodic(self):
        #self.zero_on_user_button()

        #SmartDashboard.putNumber('Algae/Arm/Position', self.armEncoder.getPosition())
        SmartDashboard.putNumber('Algae/Intake/Applied Output', self.intakeMotor.getAppliedOutput())


   

import wpilib
import rev
import commands2

from constants import CoralConstants as CC

class CoralSubsystem(commands2.Subsystem):
    def __init__(self):
        super().__init__()

        self.intakeMotor = rev.SparkMax(CC.kIntakeCanId, rev.SparkMax.MotorType.kBrushless)
        intakeConfig = rev.SparkMaxConfig()

        self.intakeMotor.configure(intakeConfig, rev.SparkBase.ResetMode.kResetSafeParameters, rev.SparkBase.PersistMode.kPersistParameters)
        
        self.intakeController = self.intakeMotor.getClosedLoopController()
        self.intakeEncoder = self.intakeMotor.getEncoder()

        self.intakeMotor.IdleMode(rev.SparkFlex.IdleMode.kBrake)

    def set_intake_power(self, power):
        self.intakeMotor.set(power)

    def run_intake_command(self):
        def execute():
            self.set_intake_power(CC.kForward)

        return commands2.InstantCommand(execute, self)
    
    def run_reverse_command(self):
        def execute():
            self.set_intake_power(CC.kReverse)

        return commands2.InstantCommand(execute, self)
    
    def run_intake_auto(self):
        def execute():
            self.set_intake_power(CC.kAutoI)

        return commands2.InstantCommand(execute, self)
    
    def run_reverse_auto(self):
        def execute():
            self.set_intake_power(CC.kAutoR)

        return commands2.InstantCommand(execute, self)
    
    def stop(self):
        def execute():
            self.set_intake_power(CC.kStop)

        return commands2.InstantCommand(execute, self)
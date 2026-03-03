import wpilib
import rev
import commands2

from wpilib import SmartDashboard

from constants import AlgaeConstants

class AlgaePivot(commands2.Subsystem):
    def __init__(self):
        super().__init__()

        self.armMotor = rev.SparkMax(AlgaeConstants.kPivotCanId, rev.SparkMax.MotorType.kBrushless)
        armConfig = rev.SparkFlexConfig()
        self.armMotor.configure(armConfig, rev.SparkBase.ResetMode.kResetSafeParameters, rev.SparkBase.PersistMode.kPersistParameters)
        
        self.armController = self.armMotor.getClosedLoopController()
        self.armEncoder = self.armMotor.getEncoder()

        self.armMotor.IdleMode(rev.SparkMax.IdleMode.kBrake)

        self.was_reset = False
        self.armEncoder.setPosition(0.0)

    def zero_on_user_button(self):
        if not self.was_reset and wpilib.RobotController.getUserButton():
            self.was_reset = True
            self.armEncoder.setPosition(0.0)
        elif not wpilib.RobotController.getUserButton():
            self.was_reset = False

    def set_arm_power(self, power):
        if power > 0:  # If the power is positive (moving up), use kUpPower
            self.armMotor.set(AlgaeConstants.kUpPower)  # 5% power to move the arm up
            self.armMotor.setIdleMode(rev.SparkMax.IdleMode.kBrake)  # Apply brake mode while moving up
        else:  # If the power is negative (moving down), use kDownPower
            self.armMotor.set(AlgaeConstants.kDownPower)  # 10% power to move the arm down
            self.armMotor.setIdleMode(rev.SparkMax.IdleMode.kBrake)

    def algae_up_command(self):
        def execute():
            self.set_arm_power(AlgaeConstants.kUp)

        return commands2.InstantCommand(execute, self)
            
    def algae_down_command(self):
        def execute():
            self.set_arm_power(AlgaeConstants.kDown)

        return commands2.InstantCommand(execute, self)
    
    def periodic(self):
        self.zero_on_user_button()

        SmartDashboard.putNumber('Algae/Arm/Position', self.armEncoder.getPosition())
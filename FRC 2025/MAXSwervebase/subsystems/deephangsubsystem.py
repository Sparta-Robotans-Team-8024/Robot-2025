import commands2
import rev 

import constants

class DeepHangSubsystem(commands2.Subsystem):
    MOTOR_GEAR_RATIO = 375 


    def __init__(self) -> None:
        super().__init__()
        self.press = rev.SparkFlex(constants.DeepHangConstants.kPressCanId1, rev.SparkFlex.MotorType.kBrushless)
        config = rev.SparkFlexConfig()
        self.press.configure(config, rev.SparkBase.ResetMode.kResetSafeParameters, rev.SparkBase.PersistMode.kPersistParameters)

        self.press.setCANTimeout(250)
        self.press.IdleMode(rev.SparkFlex.IdleMode.kBrake)

        self.pressEncoder = self.press.getEncoder()
        self.pressEncoderConfig = rev.AlternateEncoderConfig()

        self.pressEncoderConfig.positionConversionFactor(1 / self.MOTOR_GEAR_RATIO)

        self.stop_motor()


    def set_motor_voltage(self, volts):
        self.press.setVoltage(volts)

    def stop_motor(self):
        self.press.stopMotor()


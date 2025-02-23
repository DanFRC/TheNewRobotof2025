package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmPivotConstants;

public class ArmSubsystem extends SubsystemBase {

    //Definitions
    private final VictorSPX _liftMotor = new VictorSPX(ArmPivotConstants.kArmPivotMotorPort);
    // Elevator confirmed using a REV through bore encoder
    private final DutyCycleEncoder _encoder = new DutyCycleEncoder(ArmPivotConstants.kEncoderPort);

    public ArmSubsystem() {
        // do smth once
    }

    public void driveArm(double speed) {
        SmartDashboard.putNumber("liftSpeed", speed);
        if (speed < 0 && _encoder.get() >= ArmPivotConstants.kArmPivotDeadZoneMax) {
            _liftMotor.set(ControlMode.PercentOutput, speed);
            SmartDashboard.putBoolean("Up", true);
        } else if (speed > 0 && _encoder.get() <= ArmPivotConstants.kArmPivotDeadZoneMin) {
            _liftMotor.set(ControlMode.PercentOutput, speed);
        } else {
            _liftMotor.set(ControlMode.PercentOutput, 0);
        }

    }

    public double getEncoder() {
        return _encoder.get();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm Encoder Output", getEncoder());
    }

    @Override
    public void simulationPeriodic() {
    }
}
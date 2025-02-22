package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {

    //Definitions
    private final VictorSPX _liftMotor = new VictorSPX(ElevatorConstants.kElevatorMotorPort);

    private final DigitalInput homingSwitch = new DigitalInput(ElevatorConstants.kLimitSwitchPort);
    // Elevator confirmed using a REV through bore encoder
    private final Encoder _encoder = new Encoder(ElevatorConstants.kENC_A,ElevatorConstants.kENC_B);

    public ElevatorSubsystem() {
        // do smth once
        _encoder.reset();
    }

    public void driveElevator(double speed) {
        _liftMotor.set(ControlMode.PercentOutput, -speed);
    }

    public void HomeElevator() {
        if (homingSwitch.get() != true) {
            _liftMotor.set(ControlMode.PercentOutput, 0.4);
        } else {
            _liftMotor.set(ControlMode.PercentOutput, 0);
            _encoder.reset();
        }
    }

    public double getEncoder() {
        return _encoder.get();
    }

    public boolean getLimitSwitch() {
        return homingSwitch.get();
    }

    public void resetEncoder() {
        _encoder.reset();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Encoder Output", _encoder.get());
        SmartDashboard.putBoolean("Switch Static", homingSwitch.get());
    }

    @Override
    public void simulationPeriodic() {
    }
}
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {

    //Definitions
    private final VictorSPX _liftMotor = new VictorSPX(ElevatorConstants.kElevatorMotorPort);
    // Elevator confirmed using a REV through bore encoder
    private final DutyCycleEncoder _encoder = new DutyCycleEncoder(ElevatorConstants.kEncoderPort);

    public ElevatorSubsystem() {
        // do smth once
    }

    public void driveElevator(double speed) {
        _liftMotor.set(ControlMode.PercentOutput, speed);
    }

    public double getEncoder() {
        return _encoder.get();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Encoder Output", getEncoder());
    }

    @Override
    public void simulationPeriodic() {
    }
}
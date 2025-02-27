package frc.robot.subsystems;

import java.lang.reflect.Parameter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {

    //Definitions
    private PIDController normalPID = new PIDController(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD);
    private SlewRateLimiter smoother = new SlewRateLimiter(1.4);



    private double point;

    private boolean homed = false;
    //private PIDController slowPID = new PIDController(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD);
    private final VictorSPX _liftMotor = new VictorSPX(ElevatorConstants.kElevatorMotorPort);

    private final DigitalInput homingSwitch = new DigitalInput(ElevatorConstants.kLimitSwitchPort);
    // Elevator confirmed using a REV through bore encoder
    private final Encoder _encoder = new Encoder(ElevatorConstants.kENC_A,ElevatorConstants.kENC_B);

    public ElevatorSubsystem() {
        // do smth once
        homed = false;
        point = _encoder.get();
    }

    public void driveElevator(double speed) {
        _liftMotor.set(ControlMode.PercentOutput, -speed);
    }

    public void setElevator(double setPoint) {
        point = setPoint;
        normalPID.reset();
        smoother.reset(0);
        normalPID.setIZone(1050);
    }

    public void resetHome() {
        homed = false;
    }

    public double getElevatorError() {
        return normalPID.getError();
    }

    public void HomeElevator() {
        if (homingSwitch.get() == true) {
            _liftMotor.set(ControlMode.PercentOutput, -0.075);
        } else {
            _liftMotor.set(ControlMode.PercentOutput, 0);
            _encoder.reset();
            homed = true;
            normalPID.reset();
            smoother.reset(0);
            point = 8100;
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
        //_liftMotor.set(ControlMode.PercentOutput, -0.3);
        double output = normalPID.calculate(_encoder.get(), point);
        if (homed == true) {
            _liftMotor.set(ControlMode.PercentOutput, smoother.calculate(output));
        } else {
            HomeElevator();
        }
        SmartDashboard.putNumber("Encoder Output", _encoder.get());
        SmartDashboard.putBoolean("Switch Static", homingSwitch.get());
        SmartDashboard.putBoolean("Homed", homed);
        SmartDashboard.putNumber("ele POint", point);
    }

    @Override
    public void simulationPeriodic() {
    }
}
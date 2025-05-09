package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmPivotConstants;
import frc.robot.Constants.ElevatorConstants;

public class ArmSubsystem extends SubsystemBase {

    //Definitions
    private final VictorSPX _liftMotor = new VictorSPX(ArmPivotConstants.kArmPivotMotorPort);
    private PIDController normalPID = new PIDController(ArmPivotConstants.kP, ArmPivotConstants.kI, ArmPivotConstants.kD);
    private PIDController quickPID = new PIDController(4,2.65,0.475);
    private PIDController slowPID = new PIDController(1.6, 2.4, 0.5);
    private SlewRateLimiter smoother = new SlewRateLimiter(3);

    private double MAXSPEED = 0.4;

    private double point;
    private boolean slowed;
    private boolean homed;
    private boolean runnablle = false;
    private boolean quicked = false;
    double NormSpeed;
    double FASTSpeed;

    // Elevator confirmed using a REV through bore encoder
    private final DutyCycleEncoder _encoder = new DutyCycleEncoder(ArmPivotConstants.kEncoderPort);


    public ArmSubsystem() {
        // do smth once
        slowPID.reset();
        smoother.reset(0);
        slowPID.setIZone(0.2);
        normalPID.reset();
        normalPID.setIZone(0.125);
        quickPID.reset();
        quickPID.setIZone(0.125);
        homed = false;
        point = _encoder.get();
        slowed = false;
        NormSpeed = 0;
    }

    public void enableIt() {
        runnablle = false;
        slowPID.reset();
        smoother.reset(0);
        slowPID.setIZone(0.2);
        normalPID.reset();
        normalPID.setIZone(0.125);
        homed = false;
        point = _encoder.get();
        slowed = false;
        NormSpeed = 0;
        runnablle = true;
    }

    // public void driveArm(double speed) {
    //     SmartDashboard.putNumber("liftSpeed", speed);
    //     if (speed < 0 && _encoder.get() >= ArmPivotConstants.kArmPivotDeadZoneMax) {
    //         _liftMotor.set(ControlMode.PercentOutput, speed);
    //         SmartDashboard.putBoolean("Up", true);
    //     } else if (speed > 0 && _encoder.get() <= ArmPivotConstants.kArmPivotDeadZoneMin) {
    //         _liftMotor.set(ControlMode.PercentOutput, speed);
    //     } else {
    //         _liftMotor.set(ControlMode.PercentOutput, 0);
    //     }

    // }

    public void setArm(double setPoint, boolean slow, boolean quick) {
        if (slow == true) {
            slowPID.reset();
            smoother.reset(0);
            slowPID.setIZone(0.2);
            quicked = false;
        } else {
            if (quick == true) {
                quickPID.reset();
                quickPID.setIZone(0.125);
                quicked = true;
            } else {
                normalPID.reset();
                smoother.reset(0);
                normalPID.setIZone(0.125);
                quicked = false;
            }

        }
        point = setPoint;
        slowed = slow;
    }

    public double getArmError() {
        if (slowed == true) {
            return slowPID.getError();
        } else {
            return normalPID.getError();
        }
    }

    public double getEncoder() {
        return _encoder.get();
    }

    @Override
    public void periodic() {

        SmartDashboard.putBoolean("Runnable", runnablle);

        if (runnablle == true) {
            if (slowed == true) {
                double slowSpeed = slowPID.calculate(_encoder.get(), point);

                if (slowSpeed < 0 && _encoder.get() >= ArmPivotConstants.kArmPivotDeadZoneMax) {
                    _liftMotor.set(ControlMode.PercentOutput, smoother.calculate(slowSpeed));
                } else if (slowSpeed > 0 && _encoder.get() <= ArmPivotConstants.kArmPivotDeadZoneMin) {
                    _liftMotor.set(ControlMode.PercentOutput, smoother.calculate(slowSpeed));
                } else {
                    _liftMotor.set(ControlMode.PercentOutput, 0);
                }
                
            } else {
                if (quicked == true) {
                    FASTSpeed = quickPID.calculate(_encoder.get(), point);
    
                    if (NormSpeed < 0 && _encoder.get() >= ArmPivotConstants.kArmPivotDeadZoneMax) {
                        _liftMotor.set(ControlMode.PercentOutput, FASTSpeed);
                    } else if (NormSpeed > 0 && _encoder.get() <= ArmPivotConstants.kArmPivotDeadZoneMin) {
                        _liftMotor.set(ControlMode.PercentOutput, FASTSpeed);
                    } else {
                        _liftMotor.set(ControlMode.PercentOutput, 0);
                    }
                } else {
                    NormSpeed = normalPID.calculate(_encoder.get(), point);
    
                    if (NormSpeed < 0 && _encoder.get() >= ArmPivotConstants.kArmPivotDeadZoneMax) {
                        _liftMotor.set(ControlMode.PercentOutput, smoother.calculate(NormSpeed));
                    } else if (NormSpeed > 0 && _encoder.get() <= ArmPivotConstants.kArmPivotDeadZoneMin) {
                        _liftMotor.set(ControlMode.PercentOutput, smoother.calculate(NormSpeed));
                    } else {
                        _liftMotor.set(ControlMode.PercentOutput, 0);
                    }
                }
            }
        }

        
        SmartDashboard.putNumber("Arm Encoder Output", getEncoder());
        SmartDashboard.putNumber("Pos", point);
    }

    @Override
    public void simulationPeriodic() {
    }
}
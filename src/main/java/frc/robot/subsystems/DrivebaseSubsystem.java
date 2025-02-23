package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivebaseContants;

public class DrivebaseSubsystem extends SubsystemBase {

    final AHRS gyro = new AHRS(NavXComType.kUSB1);

    private Rotation2d rotation = new Rotation2d();

  private String side = "left";

  private final MecanumDrive _robotDrive;

  public double getGyroYaw() {
    return -gyro.getYaw();
    }

  public String getReefSide() {
    return side;
  }

  public void setReefSide(String givenSide) {
    if (givenSide == "left" || givenSide == "right" || givenSide == "Left" || givenSide == "Right")
    this.side = givenSide.toLowerCase();
  }

  // Define the drive base with 4 victor SPXs
  public DrivebaseSubsystem() {
    VictorSPX frontLeft = new VictorSPX(DrivebaseContants.kLeftMotorPort1);
    VictorSPX rearLeft = new VictorSPX(DrivebaseContants.kLeftMotorPort2);
    VictorSPX frontRight = new VictorSPX(DrivebaseContants.kRightMotorPort1);
    VictorSPX rearRight = new VictorSPX(DrivebaseContants.kRightMotorPort2);
  
    // Invert the right side motors.
    // You may need to change or remove this to match your robot.
    frontRight.setInverted(true);
    rearRight.setInverted(true);
    
    // Define the drivebase's speeds, the drivebase takes in 4 values as doubles.
    _robotDrive = new MecanumDrive((double speedFL) -> frontLeft.set(ControlMode.PercentOutput, speedFL),
    (double speedRL) -> rearLeft.set(ControlMode.PercentOutput, speedRL),
    (double speedFR) -> frontRight.set(ControlMode.PercentOutput, speedFR),
    (double speedRR) -> rearRight.set(ControlMode.PercentOutput, speedRR));
  }

    public void drive(double speedX, double speedY, double PIDoutput, boolean AUTOTurning, boolean fieldOriented) { 

      if (PIDoutput > 0 && AUTOTurning == false) {
        PIDoutput *= PIDoutput;
        if (fieldOriented == true) {
          _robotDrive.driveCartesian(speedX, speedY, PIDoutput, rotation.fromDegrees(-gyro.getYaw()));
        } else if (fieldOriented == false) {
          _robotDrive.driveCartesian(speedX, speedY, PIDoutput);
        }
      } else if (PIDoutput < 0 && AUTOTurning == false) {
        PIDoutput *= -1 * PIDoutput;
        _robotDrive.driveCartesian(speedX, speedY, PIDoutput);
      }

    }

    public void zeroGyro() {
        // Reset Gyro for field orentation
        gyro.reset();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Yaw", gyro.getYaw());
    }
}

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
// import com.studica.frc.AHRS.NavXComType;
// import com.studica.frc.AHRS;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivebaseContants;

public class MecanumDrivebase extends SubsystemBase {

    final AHRS gyro = new AHRS(NavXComType.kUSB1);

  // gyro calibration constant, may need to be adjusted;
  // gyro value of 360 is set to correspond to one full revolution
  private double finalDriveSpeed = 0;
  private double thisDriveSpeedx = 0;
  private double thisDriveSpeedy = 0;
  private double thisTurnspeed = 0;

  private Rotation2d rotation = new Rotation2d(gyro.getYaw());
  // Probably won't need this because mecanum drive is field oriented using the NavX
  private double invert = 1;

  //private final AHRS _gyro = new AHRS(NavXComType.kMXP_SPI);
  private final MecanumDrive _robotDrive;

  //Define wheel locations
  // Locations of the wheels relative to the robot center.
//   Translation2d m_frontLeftLocation = new Translation2d(0.381, 0.381);
//   Translation2d m_frontRightLocation = new Translation2d(0.381, -0.381);
//   Translation2d m_backLeftLocation = new Translation2d(-0.381, 0.381);
//   Translation2d m_backRightLocation = new Translation2d(-0.381, -0.381);
//   // Creating my kinematics object using the wheel locations.
//   MecanumDriveKinematics m_kinematics = new MecanumDriveKinematics(
//       m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation
//   );

  public double getGyroYaw() {
    return gyro.getYaw();
    }

  // Define the drive base with 4 victor SPXs
  public MecanumDrivebase() {
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

    // public double getGyroAngle() {
    //     // Get the angle of the gyro
    //     //return _gyro.getAngle();
    //     return 1; // This is temporary, because AHRS doesn't currently work
    // }

    public void drive(double driveSpeedx, double driveSpeedy, double turnSpeed) {

        
        // 2 given inputs drive the robot
        thisDriveSpeedx = driveSpeedx * invert;
        thisDriveSpeedy = driveSpeedy * -invert;
        thisTurnspeed = turnSpeed;
        _robotDrive.driveCartesian(thisDriveSpeedx, thisDriveSpeedy, thisTurnspeed, new Rotation2d(-gyro.getYaw())); // this actually takes 4 values, x speed, y speed, turns speed, and the gyro angle, potentially for field centric driving!

        // Help visualise data
        finalDriveSpeed = driveSpeedx;
    }
    
    public void invertDrive() {
        // Invert the drive
        invert *= -1;
    }

    public void zeroGyro() {
        // Reset Gyro for field orentation
        gyro.reset();
    }

    @Override
    public void periodic() {

        SmartDashboard.putNumber("Yaw", gyro.getYaw());
          
        // ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        //     2.0, 2.0, Math.PI / 2.0, Rotation2d.fromDegrees(45.0));
        //       // This is for field orented driving.
        //     MecanumDriveWheelSpeeds wheelSpeeds = m_kinematics.toWheelSpeeds(speeds);

              // Individual speesd
        // This method will be called once per scheduler run
        if (invert == 1) {
            SmartDashboard.putBoolean("Inverted", false);
        } else
        SmartDashboard.putBoolean("Inverted", true);

            // print stuff to the dashboard
        // SmartDashboard.putNumber("Drive Speed", finalDriveSpeed);
        // SmartDashboard.putNumber("Drive Speed X", thisDriveSpeedx);
        // SmartDashboard.putNumber("Drive Speed Y", thisDriveSpeedy);
        // SmartDashboard.putNumber("Turn Speed", thisTurnspeed);
    }
}

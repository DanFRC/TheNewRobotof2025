package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class FrontFacingCameraSubsystem extends SubsystemBase {

    final PhotonCamera frontCamera = new PhotonCamera("OV2311-10316"); // This will be the name of the OV2311 hooked up to the OPi5+ once it's setup (IF I REMEMBER XD)
    
    private double y;
    private double distance;

    public FrontFacingCameraSubsystem() {
    }


    // call for this to get left right distance from the april tag. Commonly used for aligning the robot with april tags
    // intended use, lining up the robot with the april tag to allow for easy line ups with the reef.
    public double getLeftRightDistanceFromTag() {
            return y;
    }

    // call for this to get the camera's calculated distance from the april tag. It returns metres.
    // This will likely go up to 4-5+ metres. and also really close thanks to the nature of the OV2311.
    public double getDistanceToTag() {
            return distance;
    }

    private double periodicDistance;

    @Override
    public void periodic() {

        var result = frontCamera.getLatestResult();

        if (result.hasTargets()) {
            PhotonTrackedTarget target = result.getBestTarget();
            Transform3d bestCamera = target.getBestCameraToTarget();
            // Scale for rounding
            double scale = Math.pow(10, 2);
            periodicDistance = Math.round(bestCamera.getX() * scale) / scale - .5;

            // Returned values
            distance = periodicDistance;
            y = bestCamera.getY();
        }
        // For better visualisation on the SmartDashboard.
        SmartDashboard.putNumber("Found tag distance", periodicDistance);
    }

    @Override
    public void simulationPeriodic() {
    }
}
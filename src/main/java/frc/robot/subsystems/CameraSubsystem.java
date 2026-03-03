package frc.robot.subsystems;

import java.util.Optional;
import java.util.function.BiConsumer;
import java.util.function.Consumer;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IDs;

public class CameraSubsystem extends SubsystemBase {
    private final PhotonCamera camera;
    private final PhotonPoseEstimator poseEstimator;
    private final Consumer<EstimatedRobotPose> consumer;

    public static final String cameraName = "robinvision"; // TODO: degis bunlari svp pls plsp lspo lsokap lds 
    public static final Transform3d robotToCam = new Transform3d(new Translation3d(0.5, 0.5 ,0.5), new Rotation3d(0, 0 ,0));
    public static final AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);


    public CameraSubsystem(Consumer<EstimatedRobotPose> consumer) 
    {
        this.consumer = consumer;
        this.camera = new PhotonCamera(cameraName);
        this.poseEstimator = new PhotonPoseEstimator(layout, robotToCam);
    }

    @Override
    public void periodic()
    {
        if (!camera.isConnected())
        {
            return; 
        }
        
        for (PhotonPipelineResult result : camera.getAllUnreadResults())
        {
            if (!result.hasTargets())
            {
                continue;
            }

            // wtf bu ne niye bunu returnluyo rusta donduk hani java yazicaktik...
            Optional<EstimatedRobotPose> estimated = poseEstimator.estimateCoprocMultiTagPose(result);

            if (estimated.isEmpty())
            {
                estimated = poseEstimator.estimateLowestAmbiguityPose(result);
            }

            estimated.ifPresent(consumer);
        }
    }
}

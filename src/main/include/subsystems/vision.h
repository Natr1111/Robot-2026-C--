#include "Robot.h" 
#include <math.h>

std::vector<LimelightHelpers::RawFiducial> getRawFiducials(std::string limelight_id)
{
    LimelightHelpers::PoseEstimate BotPoseEstimate = LimelightHelpers::getBotPoseEstimate(limelight_id, "botpose", false);
    return BotPoseEstimate.rawFiducials;
}
bool tagTargeting(int tagId,double* distance,double* angle){
    std::vector<LimelightHelpers::RawFiducial> aprilTagResults = getRawFiducials("limelight-b");
    for (LimelightHelpers::RawFiducial aprilTag : aprilTagResults){
        if(aprilTag.id==tagId){
            *distance=aprilTag.distToCamera;
            *angle=aprilTag.txnc;
            return true;
        }
    }
    return false;
}
bool rotationalValues(int tagId,double* distance,double* angle, double XCameraOffsetFromCenter, double YCameraOffsetFromCenter){
    double angleFromCamera;
    double horizontalFromCamera;
    double verticalFromCamera;
    double horizontalFromRobot;
    double verticalFromRobot;
    std::vector<LimelightHelpers::RawFiducial> aprilTagResults = getRawFiducials("limelight-b");
    for (LimelightHelpers::RawFiducial aprilTag : aprilTagResults){
        if(aprilTag.id == tagId){
            // *distance = aprilTag.distToCamera;
            angleFromCamera = aprilTag.txnc;
            horizontalFromCamera = aprilTag.tync;
            horizontalFromRobot = horizontalFromCamera + XCameraOffsetFromCenter;
            verticalFromCamera = std::sqrt(std::pow(aprilTag.distToCamera, 2) - std::pow(horizontalFromCamera, 2));
            verticalFromRobot = verticalFromCamera + YCameraOffsetFromCenter;

            *distance = std::sqrt(std::pow(verticalFromRobot, 2) + std::pow(horizontalFromRobot, 2));
            *angle = std::atan(horizontalFromRobot / verticalFromRobot);
            return true;
        }
    }

    //double cameraHypotonuse=sqrt(pow(distance*,2)/pow(ty,2));
    return false;
}
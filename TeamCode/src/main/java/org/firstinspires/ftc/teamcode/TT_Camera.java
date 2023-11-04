package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.CameraAprilTag.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

// Author:  John Batchelder
// Team: 8947 - Techtonics
// Camera class uses April Tags to read sleeve for FTC PowerPlay.

 public class TT_Camera {
     OpenCvCamera camera;
     AprilTagDetectionPipeline aprilTagDetectionPipeline;

     static final double FEET_PER_METER = 3.28084;

     // Lens intrinsics; UNITS ARE PIXELS
     // NOTE: this calibration is for the C920 webcam at 800x448.
     // You will need to do your own calibration for other configurations!
     double fx = 578.272;
     double fy = 578.272;
     double cx = 402.145;
     double cy = 221.506;

     // UNITS ARE METERS
     double tagsize = 0.166;

     AprilTagDetection tagOfInterest = null;

     //this is the constructor for the Camera Class
     public TT_Camera(HardwareMap hardwareMap) {
         int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
         camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
         aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

         camera.setPipeline(aprilTagDetectionPipeline);
         camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
         {
             @Override
             public void onOpened()
             {
                 camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
             }

             @Override
             public void onError(int errorCode)
             {

             }
         });

     }
     public int scanSignal() {
         // Read detections, this givers a list of detetion
         ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();
         //  If size is > than 0, we detected something
         if(currentDetections.size() > 0){
             tagOfInterest = currentDetections.get(0);
         }
         // if we have a tag of interest than return ID
         if(tagOfInterest == null ){
             return 0;
         } else {
             return tagOfInterest.id;
         }
     }
 }




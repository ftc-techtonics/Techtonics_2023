package org.firstinspires.ftc.teamcode.CoachTesting;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

/*
 * Op mode for preliminary tuning of the follower PID coefficients (located in the drive base
 * classes). The robot drives back and forth in a straight line indefinitely. Utilization of the
 * dashboard is recommended for this tuning routine. To access the dashboard, connect your computer
 * to the RC's WiFi network. In your browser, navigate to https://192.168.49.1:8080/dash if you're
 * using the RC phone or https://192.168.43.1:8080/dash if you are using the Control Hub. Once
 * you've successfully connected, start the program, and your robot will begin moving forward and
 * backward. You should observe the target position (green) and your pose estimate (blue) and adjust
 * your follower PID coefficients such that you follow the target position as accurately as possible.
 * If you are using SampleMecanumDrive, you should be tuning TRANSLATIONAL_PID and HEADING_PID.
 * If you are using SampleTankDrive, you should be tuning AXIAL_PID, CROSS_TRACK_PID, and HEADING_PID.
 * These coefficients can be tuned live in dashboard.
 *
 * This opmode is designed as a convenient, coarse tuning for the follower PID coefficients. It
 * is recommended that you use the FollowerPIDTuner opmode for further fine tuning.
 */
@Disabled
@Config
@Autonomous(name ="Coach Auto Camera", group = "Coach Testing")
public class CoachAutonomousCamera extends LinearOpMode {

    public static double DISTANCE = 20;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;
/*
        Trajectory traj = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(64, 0), 0)
                .build();

        drive.followTrajectory(traj);
        sleep(2000);

        drive.followTrajectory(
                drive.trajectoryBuilder(traj.end(), true)
                        .splineTo(new Vector2d(44, 0), Math.toRadians(180))
                        .build()
        );

        sleep(2000);
        drive.followTrajectory(
                drive.trajectoryBuilder(traj.end(), true)
                        .splineTo(new Vector2d(44, 20), Math.toRadians(180))
                        .build()
        );

 */

        Pose2d startPose2d = new Pose2d();
        Trajectory traj = drive.trajectoryBuilder(new Pose2d())
                .forward(52).build();
        drive.followTrajectory(traj);

        sleep(2000);
        traj = drive.trajectoryBuilder(traj.end())
                .back(10).build();
        drive.followTrajectory(traj);

        sleep(2000);
        traj = drive.trajectoryBuilder(traj.end())
                .strafeRight(16).build();
        drive.followTrajectory(traj);

        sleep(2000);

        traj = drive.trajectoryBuilder(traj.end())
                .strafeLeft(16).build();
        drive.followTrajectory(traj);

        sleep(2000);

        traj = drive.trajectoryBuilder(traj.end())
                .back(41).build();
        drive.followTrajectory(traj);

        sleep(2000);


/*
        Trajectory trajectoryLeft = drive.trajectoryBuilder(trajectoryForward.end())
                .strafeLeft(DISTANCE).build();

        Trajectory trajectoryBackward = drive.trajectoryBuilder(trajectoryLeft.end())
                .back(DISTANCE).build();

        Trajectory trajectoryRight = drive.trajectoryBuilder(trajectoryBackward.end())
                .strafeRight(DISTANCE).build();
*/
        //while (opModeIsActive() && !isStopRequested()) {
            //drive.followTrajectory(trajectoryForward);
            //drive.followTrajectory(trajectoryLeft);
            //drive.followTrajectory(trajectoryBackward);
            //drive.followTrajectory(trajectoryRight);
        //}

    }
}
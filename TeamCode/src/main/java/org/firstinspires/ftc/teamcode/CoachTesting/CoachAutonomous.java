package org.firstinspires.ftc.teamcode.CoachTesting;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import com.qualcomm.robotcore.hardware.DistanceSensor;
@Disabled
@Config
@Autonomous(name ="Coach Full Auto", group = "Coach Testing")
public class CoachAutonomous extends LinearOpMode {

    public static double DISTANCE = 20;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

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
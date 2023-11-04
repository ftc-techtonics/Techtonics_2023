package org.firstinspires.ftc.teamcode.CoachTesting;

import static java.lang.Thread.sleep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RRMecanumDrive;
@Disabled
@TeleOp(group = "Coach", name = "Test Distance")
public class CoachDistanceTesting_Teleop extends OpMode {

    private RRMecanumDrive mecanumDrive;

    private boolean wasDown, wasUp;
    private int distance = 24;

    /**
     * initializes all the testing features and adds them to a test list
     */
    @Override
    public void init() {
        mecanumDrive = new RRMecanumDrive(hardwareMap);
    }

    // Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP

    /**
     * allows the user to cycle through tests and press A to switch on to true
     */
    @Override
    public void loop() {

        //Put instructions on the telemetry
        telemetry.addLine("Use D-pad to change distance by 12 inches");
        telemetry.addLine("Press A for Forward / B for backward");
        telemetry.addData("Last Test Target: %d", distance);
        telemetry.addData("Estimate X:%d",mecanumDrive.getPoseEstimate().getX());
        telemetry.addLine();
        telemetry.update();

        if (gamepad1.dpad_up ) {
            distance = distance + 12;

            try {
                sleep(500);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }else if (gamepad1.dpad_down ) {
            distance = distance - 12;

            try {
                sleep(500);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
        else if (gamepad1.a ) {
            Trajectory traj = mecanumDrive.trajectoryBuilder(new Pose2d())
                    .forward(distance).build();
            mecanumDrive.followTrajectory(traj);

            try {
                sleep(500);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        } else if (gamepad1.b ) {
            Trajectory traj = mecanumDrive.trajectoryBuilder(new Pose2d())
                    .back(distance).build();
            mecanumDrive.followTrajectory(traj);

            try {
                sleep(500);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }

    }
}
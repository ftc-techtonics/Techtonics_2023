package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@Autonomous(group = "Techtonics", name = "TT_Autonomous")
public class TT_Autonomous extends LinearOpMode {

    // Distances made public so we can change in testing
    public static double DISTANCESTRAIGHT = 51;
    public static double DISTANCEBACK = 10;
    public static double DISTANCERIGHT = 17.5;
    public static double DISTANCELEFT = 17.5;

    // Motors, Servos, and Sensors
    private RRMecanumDrive drive;
    private TT_Camera camera;
    private TT_TheAmazingLift lift;
    private TT_ArmCode01 arm;
    private Servo hand;
    private Servo gripper;

    private int signalDetected = 0;
    private ElapsedTime timer = new ElapsedTime();
    private double gripperClosed = 0.7;
    private double gripperOpen = .35;

    @Override
    public void runOpMode() throws InterruptedException {
        // This enables viewing of the telemetry data through the browser Dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // define variables and run the constructors
        drive = new RRMecanumDrive(hardwareMap);
        camera = new TT_Camera(hardwareMap);
        lift = new TT_TheAmazingLift(hardwareMap);
        arm = new TT_ArmCode01(hardwareMap);
        hand = hardwareMap.get(Servo.class, "hand");
        gripper = hardwareMap.get(Servo.class, "gripper");

        //  After initilize is pressed, but before Play...  show what the camera detects.
        while (!isStarted()) {
            signalDetected = camera.scanSignal();
            telemetry.addData("Initialize tagID ", signalDetected);
            telemetry.update();
        }

        gripper.setPosition(gripperClosed);
        sleep(1000);
        //hand.setPosition(0);

        Pose2d startPose = new Pose2d(-DISTANCESTRAIGHT / 2, -DISTANCESTRAIGHT / 2, 0);

        drive.setPoseEstimate(startPose);
        // Play button pressed - starting...
        waitForStart();
        lift.SetMode(true);
        arm.SetMode(true);
        arm.SetPower(.5);
        timer.startTime();

        telemetry.addData("Running tagID ", camera.scanSignal());
        telemetry.update();

        // Start Autonomous steps
        // 1) Detect Signal... Already Done!
        // 2) Grab Cone
        gripper.setPosition(gripperClosed);
        //hand.setPosition(0);
        sleep(100);
        arm.SetPosition(TT_ArmCode01.ScoringPosition.UP, TT_ArmCode01.ScoringDirection.LEFT);
        // 3) ....

        //Move forward
        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                .forward(DISTANCESTRAIGHT)
                .build();
        drive.followTrajectorySequence(trajSeq);

        // ----------------   Drop Cone     ------- ----------------------------
        lift.SetPosition(TT_TheAmazingLift.ScoringPosition.HIGH);
        sleep(1000);
        arm.SetPosition(TT_ArmCode01.ScoringPosition.HIGH, TT_ArmCode01.ScoringDirection.LEFT);
        sleep(1000);
        hand.setPosition(1);
        sleep(5000);
        lift.SetEncoderPosition(2600);
        sleep(3000);
        gripper.setPosition(gripperOpen);
        sleep(3000);
        arm.SetPosition(TT_ArmCode01.ScoringPosition.UP, TT_ArmCode01.ScoringDirection.LEFT);
        sleep(3000);
        lift.SetPosition(TT_TheAmazingLift.ScoringPosition.START);

        sleep(3000);

        //While statement start here
        //Do while loop till 8 seconds left
        while (getRuntime() < 22) {

            trajSeq = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .back(DISTANCEBACK)
                    .strafeRight(DISTANCERIGHT)
                    .build();
            drive.followTrajectorySequence(trajSeq);

            // pickup cone

            trajSeq = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .strafeLeft(DISTANCELEFT)
                    .forward(DISTANCEBACK)
                    .build();
            drive.followTrajectorySequence(trajSeq);
            // drop
        }
        // 8 seconds to park, v  , this is for parking
        // Use camera signal to park the robot
        if (signalDetected == 1) {
            //Park in zone one
            trajSeq = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .back(DISTANCEBACK)
                    .strafeLeft(DISTANCERIGHT)

                    .build();
            drive.followTrajectorySequence(trajSeq);
        } else if (signalDetected == 2) {
            // Park in zone 2
            trajSeq = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .back(DISTANCEBACK)
                    .build();
            drive.followTrajectorySequence(trajSeq);
        } else if (signalDetected == 3) {
            // Park in zone 3
            trajSeq = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .back(DISTANCEBACK)
                    .strafeRight(DISTANCERIGHT)

                    .build();
            drive.followTrajectorySequence(trajSeq);
        }
    }
}

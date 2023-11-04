package org.firstinspires.ftc.teamcode.CoachTesting;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RRMecanumDrive;
@Config
@TeleOp(name ="Coach Teleop", group = "Coach Testing")
public class CoachTeleop extends OpMode {

    private RRMecanumDrive drive;
    private double xPower;
    private double yPower;
    private Rev2mDistanceSensor sensorRange;
    private boolean distanceTesting = true;
    private static double maxPower = .6;
    private static enum ScoringPosition {START, LOW, MEDIUM, HIGH, UP};

    @Override
    public void init() {
        if (distanceTesting) {
            sensorRange = (Rev2mDistanceSensor)hardwareMap.get(DistanceSensor.class, "distance");
        }
        drive = new RRMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop() {
        yPower = gamepad1.left_stick_y;
        xPower = gamepad1.left_stick_x;

        drive.setWeightedDrivePower(
                new Pose2d(
                        -Math.min(maxPower, yPower),
                        -Math.min(maxPower, xPower),
                        -Math.min(maxPower, gamepad1.right_stick_x)
                )
        );
        drive.update();
    }
}

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(group = "Techtonics")
public class TT_Teleop extends OpMode {
    // Define class members
    Servo hand;
    Servo gripper;
    RRMecanumDrive drive;
    TT_TheAmazingLift lift;
    TT_ArmCode01 arm;
    double gripperClosed = 0.5;
    double gripperOpen = .35;

    TT_ArmCode01.ScoringDirection direction = TT_ArmCode01.ScoringDirection.LEFT;

    int liftCurrentPosition = 0;
    double liftPower = 0;
    int armCurrentPosition = 0;
    double armPower = 0;
    double maxPower = 0.5;
    boolean automaticPositioning = false;
    double gripperCurrentPosition = 0;
    double busyRunTime;

    @Override
    public void init() {
        busyRunTime = getRuntime();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addLine("Initializing...");
        telemetry.update();
        drive = new RRMecanumDrive(hardwareMap);
        lift = new TT_TheAmazingLift(hardwareMap);
        arm = new TT_ArmCode01(hardwareMap);
        hand = hardwareMap.get(Servo.class, "hand");
        hand.setPosition(0);
        gripper = hardwareMap.get(Servo.class, "gripper");
        gripper.setPosition(gripperClosed);
        // Positioning of gripper still has to be figured out.
        //gripper.setPosition(gripperCurrentPosition);
        telemetry.addLine("Initialization Done!");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Drive Train - Gamepad 1
        drive.setWeightedDrivePower(
                new Pose2d(
                        gamepad1.left_stick_y,
                        gamepad1.left_stick_x,
                        gamepad1.right_stick_x
                )
        );
        drive.update();

        // Gripper
        if (gamepad2.a) {
            //Close hand
            gripper.setPosition(gripperClosed);
        } else if (gamepad2.b) {
            //Open hand
            gripper.setPosition(gripperOpen);
        }

        if (gamepad1.y) {
            //Can now drive
            //Now use D-pad to position arm and lift
            automaticPositioning = true;
            arm.SetMode(automaticPositioning);
            //lift.SetMode(automaticPositioning);
        } else if (gamepad1.x) {
            //When hit X no longer use D-pad
            //Now use joystick to move arm and lift and can no longer drive
            automaticPositioning = false;
            arm.SetMode(automaticPositioning);
            //lift.SetMode(automaticPositioning);
        }


        if (gamepad2.left_bumper) {
            //Switch D-pad to left scoring
            direction = TT_ArmCode01.ScoringDirection.LEFT;
        } else if (gamepad2.right_bumper) {
            //Switch D-pad to right scoring
            direction = TT_ArmCode01.ScoringDirection.RIGHT;
        }

        liftPower = gamepad2.left_stick_y;
        liftCurrentPosition = lift.Move(liftPower);

        if (busyRunTime > getRuntime()) {
            // do nothing
        } else {
            // Prepare for pickup
            if (gamepad2.left_stick_button) {
                //Close hand
                // lift down, arm out, hand positioned and open
                arm.SetPosition(TT_ArmCode01.ScoringPosition.READY_TO_PICKUP, direction);
                hand.setPosition(.8);
                gripper.setPosition(gripperOpen);
            } else if (gamepad2.right_stick_button) {
                //Open hand
                //lift.SetPosition(TT_TheAmazingLift.ScoringPosition.PICKUP_CONE);
                gripper.setPosition(.7);
                //lift.SetPosition(TT_TheAmazingLift.ScoringPosition.UP);
                arm.SetPosition(TT_ArmCode01.ScoringPosition.PICKUP_CONE, direction);
                //hand.setPosition(.7);
            }

            if (automaticPositioning) {
                AutomaticProcessing();
            } else {

                armPower = gamepad2.right_stick_y;
                armCurrentPosition = arm.Move(armPower);

                if (gamepad1.dpad_up) {
                    gripperCurrentPosition = gripperCurrentPosition + .02;
                    gripper.setPosition(gripperCurrentPosition);
                } else if (gamepad1.dpad_down) {
                    gripperCurrentPosition = gripperCurrentPosition - .02;
                    gripper.setPosition(gripperCurrentPosition);
                }
            }
        }
        TelemetryUpdate();
    }

    public void TelemetryUpdate() {
        String autoMode;
        if (automaticPositioning) {
            if (direction == TT_ArmCode01.ScoringDirection.LEFT)
                autoMode = "Automatic Positioning - LEFT Side";
            else autoMode = "Automatic Positioning - RIGHT Side";
        } else {
            autoMode = "Manual Positioning";
        }
        String teleArmInfo = String.format("ARM   Position: %4d   Power: %1.4f", arm.GetCurrentPosition(), armPower);
        String teleLiftInfo = String.format("LIFT   Position: %4d   Power: %1.4f", lift.GetCurrentPosition(), liftPower);
        String teleHandInfo = String.format("Hand   Position: %1.2f   Gripper: %1.2f", hand.getPosition(), gripper.getPosition());

        telemetry.addLine(autoMode);
        telemetry.addLine(teleArmInfo);
        telemetry.addLine(teleLiftInfo);
        telemetry.addLine(teleHandInfo);

        telemetry.update();
    }

    public boolean AutomaticProcessing() {
        if (gamepad1.dpad_up) {
            armCurrentPosition = arm.SetPosition(TT_ArmCode01.ScoringPosition.HIGH, direction);
            liftCurrentPosition = lift.SetPosition(TT_TheAmazingLift.ScoringPosition.HIGH);
            if (direction == TT_ArmCode01.ScoringDirection.RIGHT)
                hand.setPosition(.05);
            else
                hand.setPosition(1);
        } else if (gamepad1.dpad_right) {
            armCurrentPosition = arm.SetPosition(TT_ArmCode01.ScoringPosition.MEDIUM, direction);
            liftCurrentPosition = lift.SetPosition(TT_TheAmazingLift.ScoringPosition.MEDIUM);
            if (direction == TT_ArmCode01.ScoringDirection.RIGHT)
                hand.setPosition(.05);
            else
                hand.setPosition(1);
        } else if (gamepad1.dpad_down) {
            armCurrentPosition = arm.SetPosition(TT_ArmCode01.ScoringPosition.LOW, direction);
            liftCurrentPosition = lift.SetPosition(TT_TheAmazingLift.ScoringPosition.LOW);
            if (direction == TT_ArmCode01.ScoringDirection.RIGHT)
                hand.setPosition(.05);
            else
                hand.setPosition(1);
        } else if (gamepad1.dpad_left) {
            armCurrentPosition = arm.SetPosition(TT_ArmCode01.ScoringPosition.UP, direction);
            liftCurrentPosition = lift.SetPosition(TT_TheAmazingLift.ScoringPosition.UP);
            hand.setPosition(.5);
        } else if (gamepad1.left_stick_button) {
            hand.setPosition(0);
            armCurrentPosition = arm.SetPosition(TT_ArmCode01.ScoringPosition.UP, TT_ArmCode01.ScoringDirection.LEFT);
            liftCurrentPosition = lift.SetPosition(TT_TheAmazingLift.ScoringPosition.UP);

            armCurrentPosition = arm.SetPosition(TT_ArmCode01.ScoringPosition.START, TT_ArmCode01.ScoringDirection.LEFT);
            liftCurrentPosition = lift.SetPosition(TT_TheAmazingLift.ScoringPosition.START);
        }
        return true;
    }
}

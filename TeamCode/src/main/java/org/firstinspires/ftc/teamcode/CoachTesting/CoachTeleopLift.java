package org.firstinspires.ftc.teamcode.CoachTesting;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Disabled
@Config
@TeleOp(name ="Coach Teleop Lift", group = "Coach Testing")
public class CoachTeleopLift extends OpMode {
    private boolean testing = true;
    private CoachLift lift;

    @Override
    public void init() {
        lift = new CoachLift(hardwareMap);

        // Check for needing a reset
        if (!lift.LiftInStartPosition()) {
            // prompt to press A to reset or B to continue
            lift.ResetLift();
        }
        if (lift.LiftInStartPosition())
            telemetry.addLine("Lift is ready");

        else {
            telemetry.addLine("Lift not verified");
            lift.SwitchToManual();
        }
    }

    @Override
    public void loop() {
        telemetry.addLine("----Motor Power and Position----");

        if (lift.IsAutomaticOn()) {

        } else {
                telemetry.addData("Lift Position: ", lift.Move(-gamepad1.left_stick_y));
                telemetry.addLine();
        }
        telemetry.update();
    }
}

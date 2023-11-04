package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.ArrayList;
import java.util.List;
@TeleOp(group = "Testing", name = "Test Wiring")
public class QQ_TestWiring_Teleop extends OpMode {
    private final QQ_MecanumDrive mecanumDrive = new QQ_MecanumDrive();
    private List<QQ_Test> tests;
    private boolean wasDown, wasUp;
    private int testNum;
    /**
     * initializes all the testing features and adds them to a test list
     */
    @Override
    public void init() {
        mecanumDrive.init(hardwareMap);
        tests = new ArrayList<>();
        tests.addAll(mecanumDrive.getTests());
        // To add more tests here, do this: tests.addAll(sampleMechanism.getTests());
    }

    // Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP

    /**
     * allows the user to cycle through tests and press A to switch on to true
     */
    @Override
    public void loop() {
        // move up in the list of test
        if (gamepad1.dpad_up && !wasUp) {
            testNum--;
            if (testNum < 0) {
                testNum = tests.size() - 1;
            }
        }
        wasUp = gamepad1.dpad_up;

        // move down in the list of tests
        if (gamepad1.dpad_down && !wasDown) {
            testNum++;
            if (testNum >= tests.size()) {
                testNum = 0;
            }
        }
        wasDown = gamepad1.dpad_down;

        //Put instructions on the telemetry
        telemetry.addLine("Use Up and Down on D-pad to cycle through choices");
        telemetry.addLine("Press A to run test");
        //put the test on the telemetry
        QQ_Test currTest = tests.get(testNum);
        telemetry.addData("Test:", currTest.getDescription());
        //run or don't run based on a
        currTest.run(gamepad1.a, telemetry);
    }
}
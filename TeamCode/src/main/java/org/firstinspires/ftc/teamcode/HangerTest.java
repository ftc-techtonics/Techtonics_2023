/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/*
 * This OpMode ramps a single motor speed up and down repeatedly until Stop is pressed.
 * The code is structured as a LinearOpMode
 *
 * This code assumes a DC motor configured with the name "hanger" as is found on a Robot.
 *
 * INCREMENT sets how much to increase/decrease the power each cycle
 * CYCLE_MS sets the update period.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */
@TeleOp(name = "HangerTest", group = "Concept")
//@Disabled
public class HangerTest extends LinearOpMode {

    static final int    CYCLE_MS    =   50;     // period of each cycle
    @Override
    public void runOpMode() {

        TT_Hanger hanger = new TT_Hanger(hardwareMap);
        hanger.setSavedPower(0.35);

        // Wait for the start button
        telemetry.addData("Hanger Power", "%5.2f", hanger.getPower());
        telemetry.addData("Hanger Position", "%d", hanger.getCurrentPosition());
        telemetry.addData(">", "V008 - Press Start to run Hanger." );
        telemetry.update();
        waitForStart();

        while(opModeIsActive()) {

            if (gamepad1.dpad_up) {
                hanger.close();
            } else if (gamepad1.dpad_down) {
                hanger.open();
            } else if (gamepad1.a && !gamepad1.start) {
                hanger.moveTo(TT_Hanger.PositionTag.FULL_OPEN);
            } else if (gamepad1.y) {
                hanger.moveTo(TT_Hanger.PositionTag.FULL_CLOSED);
            } else if (!hanger.isBusy()) {
                hanger.stop();
            }

            // Display the current values
            telemetry.addData("Hanger Power", "%5.2f", hanger.getPower());
            telemetry.addData("Hanger Position", "%d", hanger.getCurrentPosition());
            telemetry.addData(">", "Press Stop to end test." );
            telemetry.update();

            sleep(CYCLE_MS);
            idle();
        }

        // Turn off motor and signal done;
        hanger.stop();
        telemetry.addData(">", "Done");
        telemetry.update();

    }
}

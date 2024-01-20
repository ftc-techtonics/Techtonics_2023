/* Copyright (c) 2023 FIRST. All rights reserved.
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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Wiring Testing", group = "Techtonics ")
public class Motor_Wiring_Testing extends LinearOpMode {
    // Adjust these numbers to suit your robot.
    final double DESIRED_DISTANCE = 12.0; //  this is how close the camera should get to the target (inches)

    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    final double SPEED_GAIN = 0.02;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN = 0.015;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    final double TURN_GAIN = 0.01;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)

    private DcMotor leftFrontDrive = null;  //  Used to control the left front drive wheel
    private DcMotor rightFrontDrive = null;  //  Used to control the right front drive wheel
    private DcMotor leftBackDrive = null;  //  Used to control the left back drive wheel
    private DcMotor rightBackDrive = null;  //  Used to control the right back drive wheel
    private DcMotor intakeleft = null;  //  Used to control the left front drive wheel
    private DcMotor intakeright = null;  //  Used to control the right front drive wheel
    private DcMotor liftmotor = null;  //  Used to control the left back drive wheel

    private Servo pixelSlide = null;
    private Servo pixelRelease = null;
    private Servo arm = null;

    @Override
    public void runOpMode() {
        double drive = 0;        // Desired forward power/speed (-1 to +1)
        double strafe = 0;        // Desired strafe power/speed (-1 to +1)
        double turn = 0;        // Desired turning power/speed (-1 to +1)
        double pixelslidemaxheight = 0.3;
        int maxLiftHeight = -1000;
        int minLiftHeight = 0;
        int intakestate = 0;
        double intakePower = .9;
        double maxLiftPower = 0.3;
        double minArmHeight = 0;
        double maxArmHeight = 0.6;
        boolean dpadPressedUp = false;
        boolean dpadPressedDown = false;
        boolean dpadPressedLeft = false;
        boolean dpadPressedRight = false;
        double maxReleasePos = 0.5;

        boolean releaseOpen = true;
        boolean armUp = false;

        boolean buttonPressedLBumper = false;
        boolean buttonPressedB = false;
        boolean buttonPressedA = false;
        int mode = 1;
        double powerDisplay = 0;
        int positionDisplay = 0;
        String modeDisplay = "";

        // Initialize the Apriltag Detection process

        // initAprilTag();
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must match the names assigned during the robot configuration.
        // step (using the FTC Robot Controller app on the phone).
        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftfront_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightfront_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "leftback_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightback_drive");
        //leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        //leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        //rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        //rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        intakeleft = hardwareMap.get(DcMotor.class, "intakeleft");
        intakeright = hardwareMap.get(DcMotor.class, "intakeright");
        intakeleft.setDirection(DcMotorSimple.Direction.REVERSE);
        liftmotor = hardwareMap.get(DcMotor.class, "liftmotor");
        liftmotor.setDirection(DcMotorSimple.Direction.REVERSE);

        pixelSlide = hardwareMap.get(Servo.class, "pixelslide");
        pixelRelease = hardwareMap.get(Servo.class, "pixelrelease");
        arm = hardwareMap.get(Servo.class, "arm");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips


        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            sleep(10);

            if (gamepad1.a)
                modeDisplay = "Back Left";
            else if (gamepad1.b) modeDisplay = "Back Right";
            else if (gamepad1.x) modeDisplay = "Front Left";
            else if (gamepad1.y) modeDisplay = "Front Right";

            powerDisplay = gamepad1.left_stick_y;
            if (modeDisplay == "Back Left") {
                leftBackDrive.setPower(powerDisplay);
                positionDisplay = leftBackDrive.getCurrentPosition();
            } else {leftBackDrive.setPower(0);
            }
            if (modeDisplay == "Back Right") {
                rightBackDrive.setPower(powerDisplay);
                positionDisplay = rightBackDrive.getCurrentPosition();
            } else {rightBackDrive.setPower(0);
            }
            if (modeDisplay == "Front Left") {
                leftFrontDrive.setPower(powerDisplay);
                positionDisplay = leftFrontDrive.getCurrentPosition();
            } else {leftFrontDrive.setPower(0);
            }
            if (modeDisplay == "Front Right") {
                rightFrontDrive.setPower(powerDisplay);
                positionDisplay = rightFrontDrive.getCurrentPosition();
            } else { rightFrontDrive.setPower(0);}

            //telemetry stuff
            try {
                telemetry.addData("Mode:     %s", modeDisplay);
                telemetry.addData("Power:    %f", powerDisplay);
                telemetry.addData("Position: %d ", positionDisplay);
                telemetry.update();
            } catch (Exception ex) {
                telemetry.addLine("failed");
                telemetry.update();
            }

            sleep(10);
        }
    }

    /**
     * Move robot according to desired axes motions
     * <p>
     * Positive X is forward
     * <p>
     * Positive Y is strafe left
     * <p>
     * Positive Yaw is counter-clockwise
     */
    public void moveRobot(double axial, double lateral, double yaw) {
        // Calculate wheel powers.
        double leftFrontPower = axial + lateral + yaw;
        double rightFrontPower = axial - lateral - yaw;
        double leftBackPower = axial - lateral + yaw;
        double rightBackPower = axial + lateral - yaw;


        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send powers to the wheels.
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }
}

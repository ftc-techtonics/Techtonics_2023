package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

// Author:  Sam and Catie
// Team: 8947 - Techtonics
// The amazing lift...

public class TT_TheAmazingLift {

    private DcMotorEx motor;

    private double maxPower = .75;
    private double minPower = -0.75;
    public static enum ScoringPosition {START, LOW, MEDIUM, HIGH, UP, READY_TO_PICKUP, PICKUP_CONE};

    public static enum ScoringDirection {LEFT, RIGHT}    ;
    public ScoringPosition currentScoringPosition = ScoringPosition.START;
    public ScoringPosition newScoringPosition = ScoringPosition.START;

    // Range from from one side to the other is just about 1000
    public static int MaxPosition = 2650;
    public int startPosition = 0;
    public int lowPosition = 100;
    public int mediumPosition = 900;
    public int highPosition = 2700;
    public int upPosition = 0;

    public TT_TheAmazingLift(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, "lift");
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void SetMode(boolean automaticPositioning) {
        if (automaticPositioning) {
            motor.setTargetPosition(-100);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(maxPower);
        } else {
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setPower(0);
        }
    }

    public int GetCurrentPosition() {
        return motor.getCurrentPosition();
    }
    public void SetEncoderPosition(int encoderValue) {
        motor.setTargetPosition(-encoderValue);
    }
    public int SetPosition(ScoringPosition position) {
        int calcPosition = startPosition;

        switch (position) {
            case START:
                calcPosition = startPosition;
                break;
            case LOW:
                calcPosition = lowPosition;
                break;
            case MEDIUM:
                calcPosition = mediumPosition;
                break;
            case HIGH:
                calcPosition = highPosition;
                break;
            case UP:
                calcPosition = upPosition;
                break;
            case PICKUP_CONE:
                calcPosition = 100;
                break;
            case READY_TO_PICKUP:
                calcPosition = 2000;
                break;
        }
        // Encoder values go negative so we need to reverse the sign.
        motor.setTargetPosition(-1 * calcPosition);
        currentScoringPosition = position;
        return motor.getCurrentPosition();
    }

    public int Move(double power) {
        if (power > 0) {
            if (motor.getCurrentPosition() >= 0) {
                power = 0;
            } else {
                power = Math.min(maxPower, power);
            }

        } else if (power < 0) {
            if (motor.getCurrentPosition() <= -MaxPosition) {
                power = 0;
            }
            else {
                power = Math.max(minPower, power);
            }
        }
        motor.setPower(power);
        return motor.getCurrentPosition();
    }
}


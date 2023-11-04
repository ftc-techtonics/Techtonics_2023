package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.apache.commons.math3.analysis.function.Max;
import org.opencv.core.Mat;

// Author:  me
// Team: 8947 - Techtonics
//bryce
//should work
//if doesnt work, please refer to https://www.youtube.com/watch?v=vR_E91r1Ya4
public class TT_ArmCode01 {
    private DcMotorEx motor;
    private double maxPower = 0.5;
    private double minPower = -0.5;
    public static enum ScoringPosition {START, LOW, MEDIUM, HIGH, UP,READY_TO_PICKUP, PICKUP_CONE};
    public static enum ScoringDirection {LEFT, RIGHT};
    public ScoringPosition currentScoringPosition = ScoringPosition.START;
    public ScoringPosition newScoringPosition = ScoringPosition.START;
    // Range from from one side to the other is just about 1000
    public static int MaxPosition = 1800;
    public int startPosition = 0;
    public int lowPosition = 700; // D-Pad Down
    public int mediumPosition = 700;  // D-Pad Right
    public int highPosition = 700;    // D-Pad Up
    public int upPosition = 800;

    public TT_ArmCode01(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, "arm");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public int GetCurrentPosition() {
        return motor.getCurrentPosition();
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

    public void SetPower (double power) {
        motor.setPower(power);
    }
    public void SetEncoderPosition (int encoderValue) {
        motor.setTargetPosition(encoderValue);
    }
    public int SetPosition (ScoringPosition position, ScoringDirection side) {
        int calcPosition = motor.getCurrentPosition();

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
                calcPosition = upPosition;
                break;
            case READY_TO_PICKUP:
                calcPosition = 150;
                break;
        }
        if (side == ScoringDirection.RIGHT && position != ScoringPosition.START) {
            calcPosition = MaxPosition - calcPosition;
        }
        // Encoder values go negative so we need to reverse the sign.
        motor.setTargetPosition(-1 * calcPosition);
        return motor.getCurrentPosition();
    }

    public int Move(double power) {
        if (power > 0 ) {
            if (motor.getCurrentPosition() >= -100) {
                power = 0;
            }
            else{
                power = Math.min(maxPower, power);
            }
        } else if (power < 0)
        {
            if (motor.getCurrentPosition() <= -900) {
                power = 0;
            }
            else {
                power = Math.max(minPower, power);
            }
        }

        // This assists in ensuring ZeroPowerBehavior is performed.
        if (Math.abs(power) < .1) power = 0;
        motor.setPower(power);
        return motor.getCurrentPosition();
    }
}


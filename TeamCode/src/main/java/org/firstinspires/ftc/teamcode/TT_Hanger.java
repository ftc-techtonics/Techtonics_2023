package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class TT_Hanger {

    public static enum PositionTag {FULL_CLOSED, START, FULL_OPEN};
    static final int DEFAULT_FULL_OPEN = 1000;
    static final int DEFAULT_FULL_CLOSED = 0;
    static final int DEFAULT_START = DEFAULT_FULL_OPEN;


    private int position_full_open = DEFAULT_FULL_OPEN;
    private int position_full_closed = DEFAULT_FULL_CLOSED;
    private int position_start = DEFAULT_START;
    private int position_target= DEFAULT_START;

    static final double INCREMENT   = 0.01;     // amount to ramp motor each CYCLE_MS cycle
    static final int    CYCLE_MS    =   50;     // period of each cycle
    static final double MAX_FWD     =  1.0;     // Maximum FWD power applied to motor
    static final double MAX_REV     = -1.0;     // Maximum REV power applied to motor

    // Define class members
    public DcMotor motor;

    public TT_Hanger(HardwareMap hardwareMap) {

        // Change the text in quotes to match any motor name on your robot.
        motor = hardwareMap.get(DcMotor.class, "hanger");

        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        save_current_position(PositionTag
.START);

    }

    public int save_current_position(PositionTag position_tag) {
        return set_position_tag_value(position_tag, motor.getCurrentPosition());
    }

    public int get_current_position() {
        return motor.getCurrentPosition();
    }

    public int set_position_tag_value(PositionTag position_tag, int value) {

        switch (position_tag) {
            case START:
                position_start = value;
                break;
            case FULL_OPEN:
                position_full_open = value;
                break;
            case FULL_CLOSED:
                position_full_closed = value;
                break;
        }

        return value;
    }

    public int set_target_position(PositionTag
 position_tag) {

        position_target = motor.getCurrentPosition();

        switch (position_tag) {
            case START:
                position_target = position_start;
                break;
            case FULL_OPEN:
                position_target = position_full_open;
                break;
            case FULL_CLOSED:
                position_target = position_full_closed;
                break;
        }

        return set_target_position(position_target);
    }

    public int set_target_position(int target) {
        position_target = target;
        motor.setTargetPosition(position_target);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        return motor.getCurrentPosition();
    }

    public int move(double power) {
        if (power > 0) {
            if (motor.getCurrentPosition() >= position_target) {
                power = 0;
            } else {
                power = Math.min(MAX_FWD, power);
            }

        } else if (power < 0) {
            if (motor.getCurrentPosition() <= position_target) {
                power = 0;
            }
            else {
                power = Math.max(MAX_REV, power);
            }
        }
        motor.setPower(power);
        return motor.getCurrentPosition();

    }


}

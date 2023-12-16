package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class TT_Hanger {

    public final double DEFAULT_POWER = 0.25;
    public static enum PositionTag {FULL_CLOSED, START, FULL_OPEN};

    static final int DEFAULT_START = 0;
    static final int DEFAULT_FULL_OPEN = 2000;
    static final int DEFAULT_FULL_CLOSED = -2000;

    private DcMotor motor;
    private double saved_power = DEFAULT_POWER;
    private int position_start = DEFAULT_START;
    private int position_full_open = DEFAULT_FULL_OPEN;
    private int position_full_closed = DEFAULT_FULL_CLOSED;

    TT_Hanger(HardwareMap hardwareMap) {

        // Change the text in quotes to match any motor name on your robot.
        motor = hardwareMap.get(DcMotor.class, "hanger");

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    public double getPower() {
        double power = motor.getPower();
/*
        if (motor.getMode() == DcMotor.RunMode.RUN_TO_POSITION) {
            if (motor.getTargetPosition() < motor.getCurrentPosition()) {
                power = 0 - power;
            }
        }
 */
        return power;
    }

    public void setSavedPower(double power) {
        saved_power = power;
    }

    public int getCurrentPosition() {
        return motor.getCurrentPosition();
    }

    public int getPositionByTag(PositionTag position_tag) {
        switch (position_tag) {
            case START:
                return position_start;
            case FULL_OPEN:
                return position_full_open;
            case FULL_CLOSED:
                return position_full_closed;
        }
        return 0;
    }
    public void moveTo(PositionTag position_tag) {

        int target_position = getPositionByTag(position_tag);
        motor.setTargetPosition(target_position);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(saved_power);

    }

    public void move(double power) {
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        int current_position = getCurrentPosition();
        if ((current_position > position_full_closed && power < 0) ||
                (current_position < position_full_open && power > 0)) {
            motor.setPower(power);
        } else {
            stop();
        }
    }

    public void open() {
        move(Math.abs(saved_power)); // move in positive direction
    }

    public void close() {
        move(0-Math.abs(saved_power)); // move in negative direction
    }

    public boolean isBusy() {
        return motor.isBusy();
    }

    public void stop() {
        motor.setPower(0);
    }
}


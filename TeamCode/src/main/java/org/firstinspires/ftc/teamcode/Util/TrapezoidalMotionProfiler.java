package org.firstinspires.ftc.teamcode.Util;

import com.arcrobotics.ftclib.controller.PIDController;
import com.pedropathing.util.PIDFController;

public class TrapezoidalMotionProfiler {
    public State m_current;

    private State m_goal;
    private State m_target;
    private int direction;
    private double max_acceleration;
    private double max_velocity;
    private double end_accel;
    private double cruise_speed;
    private double end_deccel;
    private double min_input;
    private double max_input;
    private PIDController controller;

    public TrapezoidalMotionProfiler(double max_a, double max_v, double pos, double vel, PIDController pid) {
        this.max_acceleration = max_a;
        this.max_velocity = max_v;
        this.m_current = new State(pos, vel);
        this.controller = pid;
    }

    public static class State {
        public double position;
        public double velocity;

        public State(double position, double velocity) {
            this.position = position;
            this.velocity = velocity;
        }

        public boolean equals(State other) {
            return this.position == other.position && this.velocity == other.velocity;
        }

    }

    public State calculate(double t, State current, State target) {
        direction = shouldFlipAcceleration(current, target) ? -1 : 1;
        m_current = direct(current);
        target = direct(target);

        if (m_current.velocity > max_velocity) {
            m_current.velocity = max_velocity;
        }

        double cutoffBegin = m_current.velocity / max_acceleration;
        double cutoffDistBegin = cutoffBegin * cutoffBegin * max_acceleration / 2.0;

        double cutoffEnd = target.velocity / max_acceleration;
        double cutoffDistEnd = cutoffEnd * cutoffEnd * max_acceleration / 2.0;

        double fullTrapezoidDist =
            cutoffDistBegin + (target.position - m_current.position) + cutoffDistEnd;
        double accelerationTime = max_velocity / max_acceleration;

        double fullSpeedDist =
            fullTrapezoidDist - accelerationTime * accelerationTime * max_acceleration;

        if (fullSpeedDist < 0) {
            accelerationTime = Math.sqrt(fullTrapezoidDist / max_acceleration);
            fullSpeedDist = 0;
        }

        end_accel = accelerationTime - cutoffDistBegin;
        cruise_speed = end_accel + fullSpeedDist / max_velocity;
        end_deccel = cruise_speed + accelerationTime - cutoffEnd;
        State result = new State(m_current.position, m_current.velocity);

        if (t < end_accel) {
            result.velocity += t * max_acceleration;
            result.position += (m_current.velocity + t * max_acceleration / 2.0) * t;
        } else if (t < cruise_speed) {
            result.velocity = max_velocity;
            result.position +=
                    (m_current.velocity + end_accel * max_acceleration / 2.0) * end_accel
                        + max_velocity * (t - end_accel);
        } else if (t <= end_deccel) {
            result.velocity = target.velocity + (end_deccel - t) * max_acceleration;
            double timeLeft = end_deccel - t;
            result.position = target.position - (target.position + timeLeft * max_acceleration / 2.0) * timeLeft;
        } else {
            result = target;
        }
        return direct(result);
    }

    public double calculatePID(double measurement) {
        m_target = this.calculate(getPeriod(), m_target, m_goal);
        return controller.calculate(measurement, m_target.position);
    }

    public double calculateProfiledPID(double measurement, double goal) {
        setGoal(goal);
        return calculatePID(measurement);
    }

    public void reset(State measurement) {
        controller.reset();
        m_target = measurement;
    }

    private static boolean shouldFlipAcceleration(State initial, State target) {
        return initial.position > target.position;
    }

    private State direct(State in) {
        State result = new State(in.position, in.velocity);
        result.position = result.position * direction;
        result.velocity = result.velocity * direction;
        return result;
    }

    public void setPID(double kP, double kI, double kD) {
        controller.setPID(kP, kI, kD);
    }

    public double getPeriod() {
        return controller.getPeriod();
    }

    public void setGoal(State goal) {
        m_goal = goal;
    }

    public void setGoal(double goal) {
        m_goal = new State(goal, 0);
    }

    public State getGoal() {
        return m_goal;
    }

    public boolean atGoal() {
        return atSetpoint() && m_goal.equals(m_target);
    }
    public boolean atSetpoint() {
        return controller.atSetPoint();
    }

    public State getSetpoint() {
        return m_target;
    }
}

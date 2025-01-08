package org.firstinspires.ftc.teamcode.OpModes.Autonomous.Subsystems;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDController {
    double Kp;
    double Ki;
    double Kd;
    double setpoint;

    double integralSum;

    double lastError;

    ElapsedTime timer = new ElapsedTime();

    public double calculate(double desiredPos, double processVariable, double dt) {
        setpoint = desiredPos;
        //Calculate error
        double error = setpoint - processVariable;
        //process variable = current motor encoder position

        //proportional term
        double pTerm = Kp * error;

        //integral term
        integralSum += error * dt;
        double iTerm = Ki * integralSum;

        //derivative term
        double derivative = (error - lastError) / dt;
        double dTerm = Kd * derivative;

        //update last error
        lastError = error;

        //compute PID output
        return pTerm + iTerm + dTerm;
    }
}

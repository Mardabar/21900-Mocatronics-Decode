package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class FlyWheelTuner {
    private DcMotorEx cannon;
    private double kS, kV, kP;

    public FlyWheelTuner(DcMotorEx cannon, double kS, double kV, double kP){
        this.cannon = cannon;
        this.kS = kS;
        this.kV = kV;
        this.kP = kP;

        this.cannon.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void update(double targetRPM, double currentVoltage){
        // converts rpm to ticks per seccond
        double targetTicksPerSecond = (targetRPM * 28) / 60.0;

        // Gets the velocity of the motor
        double currentTicksPerSecond = cannon.getVelocity();

        // FeedForward portion, F unit
        double ff = (kV * targetTicksPerSecond) + (kS * Math.signum(targetTicksPerSecond));

        // Feedback portion, Proportinal Unit
        double error = targetTicksPerSecond - currentTicksPerSecond;
        double fb = kP * error;

        // this should be a equation i got online for volatge compensation
        // Will assume the voltage is 12 for simplicity
        double compensationFactor = 12 / currentVoltage;


        // This should set motor to power correctly after accounting for the voltage comp equa
        cannon.setPower((ff + fb) * compensationFactor);

    }
}

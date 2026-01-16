package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

@Disabled
@TeleOp(name = "FlyWheelShooter")
public class FlyWheelShooter extends OpMode {
    public static double kP = 0.001;
    public static double kS = 0.02;
    public static double kV = 0.00043;



    private DcMotorEx cannon;
    private VoltageSensor battery;

    @Override
    public void init() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        battery = hardwareMap.voltageSensor.iterator().next();
        //battery = hardwareMap.get(VoltageSensor.class, "")

        cannon.setDirection(DcMotorEx.Direction.REVERSE);
    }

    @Override
    public void loop() {
        double currentTPS = cannon.getVelocity();
        double currentVoltage = battery.getVoltage();

        telemetry.addData("Actual TPS", currentTPS);
        telemetry.addData("Voltage", currentVoltage);
        telemetry.addData("kP", kP);
        telemetry.update();
    }

    public double GetCalculatedPower(double targetTPS, double currentTPS){
        double error = targetTPS - currentTPS;
        double currentVoltage = battery.getVoltage();

        double ff = (kV * targetTPS) + (kS * Math.signum(targetTPS));
        double fb = kP * error;

        return (ff + fb) * (12.0 / currentVoltage);
    }

}
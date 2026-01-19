package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class FlyWheelShooter{
    public static double kP = 0.001;
    public static double kS = 0.02;
    public static double kV = 0.00043;



    private DcMotorEx cannon;
    private VoltageSensor battery;

    public FlyWheelShooter(HardwareMap hardwareMap, Telemetry telemetry) {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        battery = hardwareMap.voltageSensor.iterator().next();
        //battery = hardwareMap.get(VoltageSensor.class, "")

        cannon.setDirection(DcMotorEx.Direction.REVERSE);


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
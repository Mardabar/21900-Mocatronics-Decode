package org.firstinspires.ftc.teamcode.teleop;




import static org.firstinspires.ftc.teamcode.subsystems.ShootSystem.closePos;
import static org.firstinspires.ftc.teamcode.subsystems.ShootSystem.openPos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.panels.Panels;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.subsystems.FeedBackShootSystem;
import org.firstinspires.ftc.teamcode.subsystems.ShootSystem;



@Configurable
@TeleOp(name = "FlyWheelTuner")
public class FlyWheelTuner extends OpMode {

    private FeedBackShootSystem shooter;


    public static double kP = 0.001;
    public static double kS = 0.02;
    public static double kV = 0.00043; // was 0.00035
    public static double targetTPS = 1200;  // was 1200


    private DcMotorEx cannon, belt;
    private VoltageSensor battery;

    @Override
    public void init() {

        shooter = new FeedBackShootSystem(hardwareMap, telemetry);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        cannon = hardwareMap.get(DcMotorEx.class, "cannon");
        belt = hardwareMap.get(DcMotorEx.class, "belt");
        battery = hardwareMap.voltageSensor.iterator().next();

        cannon.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        cannon.setDirection(DcMotorEx.Direction.REVERSE);
        cannon.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
    }

    @Override
    public void loop() {

        double currentTPS = cannon.getVelocity();
        double currentVoltage = battery.getVoltage();

        double ff = (kV * targetTPS) + (kS * Math.signum(targetTPS));

        double error = targetTPS - currentTPS;
        double fb = kP * error;


        double power = (ff + fb) * (12.0 / currentVoltage);

        cannon.setPower(power);


        if (gamepad1.b)
            belt.setPower(1);
        else if (gamepad1.x)
            belt.setPower(-1);
        else
            belt.setPower(0);


        if (gamepad1.yWasPressed()){
            shooter.feeder.setPosition(closePos);
        } else if (gamepad1.yWasReleased())
            shooter.feeder.setPosition(openPos);

        telemetry.addData("Target TPS", targetTPS);
        telemetry.addData("Actual TPS", currentTPS);
        telemetry.addData("Error", error);
        telemetry.addData("Voltage", currentVoltage);
        telemetry.addData("kP", kP);
        telemetry.update();
    }
}

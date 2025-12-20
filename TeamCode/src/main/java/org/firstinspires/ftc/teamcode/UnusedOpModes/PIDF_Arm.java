package org.firstinspires.ftc.teamcode.UnusedOpModes;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;


@Configurable
@Disabled
@TeleOp
public class PIDF_Arm extends OpMode {

  public PIDController controller;

  public static double p = 0, i = 0, d = 0;
  public static double f = 0;

  public static double elbowTarget;

  public void setElbowTarget(double b) {
    elbowTarget = b;
  }

  private final double ticks_in_degrees = 751.8 / 180;

  private DcMotor elbow;

  @Override
  public void init() {
    controller = new PIDController(p, i, d);
    telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

    elbow = hardwareMap.get(DcMotor.class, "elbow");

    elbow.setDirection(DcMotor.Direction.REVERSE);

  }


  @Override
  public void loop() {
    controller.setPID(p, i, d);
    int elPos = elbow.getCurrentPosition();
    double pid = controller.calculate(elPos, elbowTarget);
    double ff = Math.cos(Math.toRadians(elbowTarget / ticks_in_degrees)) * f;

    elbow.setPower(pid + ff);

    telemetry.addData("Elbow Pos", elPos);
    telemetry.addData("Elbow Target", elbowTarget);
    telemetry.update();
  }
}




//package org.firstinspires.ftc.teamcode.subsystems;
//import com.pedropathing.util.Timer;
//import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
//import com.qualcomm.robotcore.util.ElapsedTime;
//import com.seattlesolvers.solverslib.command.SubsystemBase;
//import com.seattlesolvers.solverslib.command.Command;
//import com.seattlesolvers.solverslib.command.InstantCommand;
//
//
//public class ShootSubsystem {
//
//    private DcMotorEx ls;
//    private DcMotorEx rs;
//    private DcMotor belt;
//    private DcMotor elbow;
//    private CRServo bl;
//    private CRServo br;
//    private CRServo ascension;
//    private Servo blocker;
//
//
//    private final double OVERSHOOT_VEL_MULT = 1.68; // was 1.68
//    private final double OVERSHOOT_ANG_MULT = 1;
//    private final double ANGLE_CONST = 2.08833333;
//    private final int ELBOW_GEAR_RATIO = 4;
//    private final double MAX_HEIGHT = 1.4;
//    private double shootVel;
//    private double shootAngle;
//
//
//    // INTAKE VARS
//    private double elbowSpeed = 0.5;
//    private double beltSpeed = 1;
//
//
//    // SERVO VARS
//    private double openPos = 0.53;
//    private double feedPos = 0.02;
//
//
//    // TIMER VARS
//    private ElapsedTime feedTimer;
//    private double feedDur = 450; // was 400
//    private double retDur = 600; // was 1000
//    private double beltDur = 600; // was 500, 300
//    private ElapsedTime shootTimer;
//    private int shootTimerCount = -1;
//    private int feeding = 0;
//    private int fcount = 0;
//
//    private double fx = 10;
//    private double fy = 136.5;
//
//    // Game timers
//    private Timer pathTimer, opmodeTimer;
//    public ShootSubsystem(final HardwareMap hMap, final String name){
//
//        ls = hMap.get(DcMotorEx.class, "ls");
//        MotorConfigurationType configLs = ls.getMotorType().clone();
//        configLs.setAchieveableMaxRPMFraction(1.0);
//        ls.setMotorType(configLs);
//        ls.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        ls.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        ls.setDirection(DcMotorSimple.Direction.FORWARD);
//
//        rs = hMap.get(DcMotorEx.class, "rs");
//        MotorConfigurationType configRs = rs.getMotorType().clone();
//        configRs.setAchieveableMaxRPMFraction(1.0);
//        rs.setMotorType(configRs);
//        rs.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rs.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rs.setDirection(DcMotorSimple.Direction.REVERSE);
//
//
//        elbow = hMap.get(DcMotor.class, "elbow");
//        elbow.setDirection(DcMotor.Direction.REVERSE);
//        elbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//    }
//
//    public void setShootPos(double ix, double iy, double fx, double fy){
//
//    /* dist is the total distance the ball will travel until it hits the ground
//       It's divided by 40 to turn the field units into meters
//       Then, it's multiplied by 1.3 because the ball will hit the goal first, so using
//       equation, it'll be about 1 meter high (the height of the goal) when it hit our r
//     */
//        double dist = (Math.sqrt(Math.pow(fx - ix, 2) + Math.pow(fy - iy, 2)) / 40) * 1.3;
//
//        // The angle and velocity are both calculated using the distance we found
//        shootAngle = ((distToAngle(dist) * OVERSHOOT_ANG_MULT) - 53.5);
//        shootVel = angleToVel(distToAngle(dist)) * OVERSHOOT_VEL_MULT;
//
////        telemetry.addData("Distance", dist);
////        telemetry.addData("Angle", shootAngle);
////        telemetry.addData("Real Angle", distToAngle(dist));
////        telemetry.addData("Velocity", shootVel);
////        telemetry.addData("Real Velocity", angleToVel(distToAngle(dist)));
////        telemetry.update();
//
//        setElbowTarget(angleToEncoder(shootAngle));
//    }
//
//    public double distToAngle(double dist){
//        return Math.toDegrees(Math.atan(54.88 / (9.8 * dist)));
//    }
//
//    // This function translates angle to velocity using the already set maximum height
//    public double angleToVel(double angle){
//        return Math.sqrt((MAX_HEIGHT * 19.6) / Math.pow(Math.sin(Math.toRadians(angle)), 2));
//    }
//
//    // This function translates velocity to motor power specifically for 6000 RPM motors co
//    public double velToPow(double vel){
//        return (vel / (7.2 * Math.PI)) * 2800;
//    }
//
//
//    // This function translates an angle in degrees to an encoder value on 223 RPM motors
//    public double angleToEncoder(double angle){
//        return angle * ANGLE_CONST * ELBOW_GEAR_RATIO;
//    }
//
//
//    // Do i change these to public because it takes position of the elbow,
//    // The elbow is called and intitalised in the subsystem tho so I shouldnt need to right
//    public void setElbowTarget(double angle){
//        elbow.setTargetPosition((int) angleToEncoder(angle));
//        elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//    }
//
//    public void shootball(){
//        if (shootTimerCount == -1) {
//            shootTimer.reset();
//            shootTimerCount = 0;
//        }
//
//        if (shootTimer.milliseconds() < 1200 && shootTimerCount == 0 /* && shootPos == 1*/){
//            ls.setVelocity(velToPow(shootVel));
//            rs.setVelocity(velToPow(shootVel));
//        }
//        else if (shootTimerCount == 0){
//            shootTimer.reset();
//            feedTimer.reset();
//            shootTimerCount = 1;
//        }
//        // Changed the multiplier to 2 because we are grabbing 2 balls instead of 3
//        if (shootTimer.milliseconds() < 13000 && fcount <= 6 && shootTimerCount == 1){
//            feedLauncher();
//        }
//        else if (shootTimerCount == 1)
//            shootTimerCount = 2;
//
//        if (shootTimerCount == 2){
//            ls.setVelocity(0);
//            rs.setVelocity(0);
//            feeding = 2;
//            fcount = 0;
//            ascension.setPower(0);
//            stopBelt();
//            blocker.setPosition(1);
//        }
//    }
//
//    //
//    public void belt(double speed){
//        belt.setPower(-speed);
//        br.setPower(-speed);
//        bl.setPower(speed);
//    }
//
//
//    // Ball to launcher function
//    public void feedLaunch(){
//        if (feedTimer.milliseconds() < feedDur && feeding == 0){
//            blocker.setPosition(0);
//            ascension.setPower(1);
//            stopBelt();
//        }
//        else if (feedTimer.milliseconds() < retDur && feeding == 1){
//            blocker.setPosition(1);
//        }
//        else if (feedTimer.milliseconds() < beltDur && feeding == 2) {
//            blocker.setPosition(1);
//            ascension.setPower(1);
//            runBelt();
//        }
//        else {
//            if (ls.getVelocity() >= velToPow(shootVel) - 30 && rs.getVelocity() >= velToPow(shootVel) - 30) {
//                if (feeding == 2)
//                    feeding = 0;
//                else
//                    feeding++;
//                fcount++;
//            }
//            feedTimer.reset();
//        }
//    }
//
//    /// COMMANDS HERE
//    // Ball to launcher command
//    public Command feedLauncher(){
//        return new InstantCommand(() -> feedLaunch());
//    }
//
//    // Turns on intake
//    public Command runBelt(){
//        return new InstantCommand(() -> belt(1));
//    }
//
//    // Turns off intake
//    public Command stopBelt(){
//        return new InstantCommand(() -> belt(0));
//    }
//
//    // Shoots balls
//    public Command shoot(){
//        return new InstantCommand(() -> shootball());
//    }
//
//
//
//}

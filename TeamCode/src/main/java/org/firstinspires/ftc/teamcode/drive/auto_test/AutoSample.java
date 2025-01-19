package org.firstinspires.ftc.teamcode.drive.auto_test;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Config
@Autonomous(name = "AUTObasket", group = "Autonomous")

public class AutoSample extends LinearOpMode{

    private PIDController controller;

    public static double p = 0.02, i = 0, d = 0.0005;

    public static double f = 0.001;
    public static int target = 0;

    private DcMotorEx AL;
    private DcMotorEx AR;
    private final double ticks_in_degree = 700 / 180.0;

    private Servo H_angleL;

    private Servo H_angleR;

    public class H_factor {
        private Servo H_wristL;
        private Servo H_wristR;

        private Servo H_angleR;
        private Servo H_angleL;

        private Servo H_length;



        private Servo V_wristL;

        //setting default var
        int target = 0;

        boolean first_count = false;

        double V_Grip_OPEN = 0.35;
        double V_Grip_CLOSE = 0.63;

        double H_Grip_OPEN = 0.55;
        double H_Grip_CLOSE = 0.85;

        boolean HG_OPEN = true;
        boolean VG_OPEN = true;

        //TODO: find Horizon Griper value

        int Low_basket = 2400;
        int High_basket = 4200;
        int clip_pick = 0;

        int High_chamber = 2200;
        int High_chamber_hang = 1100;

        //TODO: make rigging mechanism and find tick
        int Low_rigging = 0;

        double V_wrist_outside_90degree = 0.78;
        double V_wrist_clip_pickup = 0.77;
        double V_wrist_trans = 0.03;
        double V_wrist_trans_temp = 0.2;
        double V_wrist_basket = 0.65;


        int trans_status = 0;

        int chamber_status = 0;

        int pick_status = 0;

        double H_wristL_POS90 = 0.5;
        double H_wristR_POS90 = 0.5;

        double H_wristL_POS180 = 0.85;
        double H_wristR_POS180 = 0.85;

        double H_length_IN = 0.88;
        double H_length_OUT = 0.5;

        double H_angle_Ready = 0.3;
        double H_angle_pickup = 0.16;
        double H_angle_trans = 0.68;

        public H_factor(HardwareMap hardwareMap) {
            H_wristL = hardwareMap.servo.get("H_wristL");
            H_wristR = hardwareMap.servo.get("H_wristR");

            V_wristL = hardwareMap.servo.get("V_wristL");

            H_angleR = hardwareMap.servo.get("H_angleR"); // Wrist right Servo
            H_angleL = hardwareMap.servo.get("H_angleL"); // Wrist left Servo

            H_length = hardwareMap.servo.get("H_length");

            H_wristL.setDirection(Servo.Direction.REVERSE);
            H_angleL.setDirection(Servo.Direction.REVERSE);
        }

        public class H_out implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                H_length.setPosition(H_length_OUT);
                H_wristL.setPosition(H_wristL_POS90);
                H_wristR.setPosition(H_wristR_POS90);
                H_angleL.setPosition(0.3);
                H_angleR.setPosition(0.3);
                return false;
            }
        }
        public Action H_OUT() {
            return new H_out();
        }

        public class H_in implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                H_length.setPosition(H_length_IN);
                H_angleL.setPosition(0.68);
                H_angleR.setPosition(0.68);
                H_wristL.setPosition(H_wristL_POS180);
                H_wristR.setPosition(H_wristR_POS180);

                return false;
            }
        }
        public Action H_IN() {
            return new H_in();
        }

        public class H_pick implements Action {


            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                    H_length.setPosition(H_length_OUT);
                    H_wristL.setPosition(H_wristL_POS90);  //trans
                    H_wristR.setPosition(H_wristR_POS90);  //trans

                    H_angleL.setPosition(0.16);  //ready
                    H_angleR.setPosition(0.16);  //ready

                    V_wristL.setPosition(V_wrist_trans);  //

                return false;
            }
        }

        public Action H_Pick() {
            return new H_pick();
        }

        public class H_up implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {


                H_angleL.setPosition(H_angle_trans);      //trans
                H_angleR.setPosition(H_angle_trans);      //trans

                H_wristL.setPosition(H_wristL_POS180);      //trans
                H_wristR.setPosition(H_wristR_POS180);      //trans

                return false;
            }
        }
        public Action H_UP() {
            return new H_up();
        }
    }

    public class V_factor {
        private DcMotorEx AL;
        private DcMotorEx AR;
        private Servo V_wristL;

        //setting default var

        int target = 0;



        //TODO: find Horizon Griper value

        int Low_basket = 2400;
        int High_basket = 4200;
        int clip_pick = 0;

        int High_chamber = 2200;
        int High_chamber_hang = 1100;

        //TODO: make rigging mechanism and find tick
        int Low_rigging = 0;

        double V_wrist_outside_90degree = 0.78;
        double V_wrist_clip_pickup = 0.77;
        double V_wrist_trans = 0.07;
        double V_wrist_trans_temp = 0.2;
        double V_wrist_basket = 0.76;


        int trans_status = 0;

        int chamber_status = 0;

        int pick_status = 0;

        double H_wristL_POS90 = 0.5;
        double H_wristR_POS90 = 0.5;

        double H_wristL_POS180 = 0.85;
        double H_wristR_POS180 = 0.85;

        double H_length_IN = 0.88;
        double H_length_OUT = 0.5;

        double H_angle_Ready = 0.3;
        double H_angle_pickup = 0.16;
        double H_angle_trans = 0.68;


        public V_factor(HardwareMap hardwareMap) {
            AL = hardwareMap.get(DcMotorEx.class, "AL");
            AR = hardwareMap.get(DcMotorEx.class, "AR");

            AR.setDirection(DcMotorSimple.Direction.REVERSE);

            AL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            AR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            V_wristL = hardwareMap.servo.get("V_wristL");
        }

        public class V_basket implements Action {
            private boolean init = false;

            @Override
            public boolean run (@NonNull TelemetryPacket packet) {
                if (!init) {
                    AL.setPower(1);
                    AR.setPower(1);
                    V_wristL.setPosition(V_wrist_basket);


                    init = true;
                }

                target = High_basket;

                double pos = AL.getCurrentPosition();
                packet.put("AL_POS", pos);
                if (pos < 4100) {
                    return true;
                } else {
                    AL.setPower(0);
                    AR.setPower(0);
                    return false;
                }

            }
        }

        public Action V_Basket() {
            return new V_basket();
        }

        public class V_ground implements Action {
            private boolean init = false;

            @Override
            public boolean run (@NonNull TelemetryPacket packet) {
                if (!init) {
                    AL.setPower(-1);
                    AR.setPower(-1);
                    V_wristL.setPosition(V_wrist_trans);

                    init = true;
                }

                target = 0;


                double pos = AL.getCurrentPosition();
                packet.put("AL_POS", pos);
                if (pos >= 30) {
                    return true;
                } else {
                    AL.setPower(0);
                    AR.setPower(0);
                    return false;
                }

            }
        }

        public Action V_Ground() {
            return new V_ground();
        }

        public class V_chamber_high implements Action {
            private boolean init = false;

            @Override
            public boolean run (@NonNull TelemetryPacket packet) {
                if (!init) {

                    AL.setPower(1);
                    AR.setPower(1);

                    init = true;
                }

                target = High_chamber;



                double pos = AL.getCurrentPosition();
                packet.put("AL_POS", pos);
                if (pos <= High_chamber) {
                    return true;
                } else {
                    AL.setPower(0);
                    AR.setPower(0);
                    return false;
                }

            }
        }

        public Action V_Chamber_High() {
            return new V_chamber_high();
        }

        public class V_chamber_hang implements Action {
            private boolean init = false;

            @Override
            public boolean run (@NonNull TelemetryPacket packet) {
                if (!init) {
                    AL.setPower(-1);
                    AR.setPower(-1);
                    init = true;
                }

                target = High_chamber_hang;

                double pos = AL.getCurrentPosition();
                packet.put("AL_POS", pos);
                if (pos >= High_chamber_hang) {
                    return true;
                } else {
                    AL.setPower(0);
                    AR.setPower(0);
                    return false;
                }

            }
        }

        public Action V_Chamber_Hang() {
            return new V_chamber_hang();
        }

        public class clip_pick implements Action {
            private boolean init = false;

            @Override
            public boolean run (@NonNull TelemetryPacket packet) {
                V_wristL.setPosition(V_wrist_clip_pickup);
                return false;

            }
        }

        public Action Clip_PICK() {
            return new clip_pick();
        }





        public class chamber_hang implements Action {
            private boolean init = false;

            @Override
            public boolean run (@NonNull TelemetryPacket packet) {
                V_wristL.setPosition(V_wrist_outside_90degree);
                return false;

            }
        }

        public Action V_wrist_Chamber_Hang() {
            return new chamber_hang();
        }



    }

    public class Grip_factor {
        private Servo H_grip;
        private Servo V_grip;


        double V_Grip_OPEN = 0.35;
        double V_Grip_CLOSE = 0.63;

        double H_Grip_OPEN = 0.55;
        double H_Grip_CLOSE = 0.85;

        public Grip_factor(HardwareMap hardwareMap) {
            H_grip = hardwareMap.servo.get("H_grip");
            V_grip = hardwareMap.servo.get("V_grip");
        }

        public class V_grip_close implements Action {
            private boolean init = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                V_grip.setPosition(V_Grip_CLOSE);
                return false;
            }
        }
        public Action V_grip_CLOSE() {
            return new V_grip_close();
        }

        public class V_grip_open implements  Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                V_grip.setPosition(V_Grip_OPEN);
                return false;
            }
        }
        public Action V_grip_OPEN() {
            return new V_grip_open();
        }

        public class H_grip_close implements  Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                H_grip.setPosition(H_Grip_CLOSE);
                return false;
            }
        }
        public Action H_grip_CLOSE() {
            return new H_grip_close();
        }

        public class H_grip_open implements  Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                H_grip.setPosition(H_Grip_OPEN);
                return false;
            }
        }
        public Action H_grip_OPEN() {
            return new H_grip_open();
        }

    }



   @Override
    public void runOpMode() throws InterruptedException {

       Pose2d initialPose = new Pose2d(-6.7, -62.4, 3*Math.PI / 2);
       MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
       H_factor h_factor = new H_factor(hardwareMap);
       V_factor v_factor = new V_factor(hardwareMap);
       Grip_factor grip_factor = new Grip_factor(hardwareMap);

       controller = new PIDController(p, i, d);

       AL = hardwareMap.get(DcMotorEx.class, "AL");
       AR = hardwareMap.get(DcMotorEx.class, "AR");
       AR.setDirection(DcMotorSimple.Direction.REVERSE);



       TrajectoryActionBuilder traj = drive.actionBuilder(initialPose)
               .afterTime(0, v_factor.V_Chamber_High())
               .afterTime(0, v_factor.V_wrist_Chamber_Hang())
               .lineToY(-36)
               .stopAndAdd(() -> Actions.runBlocking(v_factor.V_Chamber_Hang()))
               .stopAndAdd(() -> Actions.runBlocking(grip_factor.V_grip_OPEN()))
               .waitSeconds(0.1)
               .afterTime(0.5, v_factor.V_Ground())
               .afterTime(1, h_factor.H_OUT())
               .setTangent(Math.PI/2)
               .splineToLinearHeading(new Pose2d(-46.7, -46, Math.PI / 2), Math.PI / 2)
               .afterTime(0, grip_factor.H_grip_OPEN())
               .stopAndAdd(() -> Actions.runBlocking(h_factor.H_Pick()))
               .stopAndAdd(() -> Actions.runBlocking(grip_factor.H_grip_CLOSE()))
               .stopAndAdd(() -> Actions.runBlocking(h_factor.H_IN()))
               .stopAndAdd(() -> Actions.runBlocking(grip_factor.V_grip_CLOSE()))
               .afterTime(0, grip_factor.H_grip_OPEN());
               /*
               .afterTime(0, grip_factor.H_grip_OPEN())
               .afterTime(0, grip_factor.V_grip_CLOSE())
               .setTangent(Math.PI/4)
               .splineToLinearHeading(new Pose2d(-56, -58.5,  Math.PI / 4), Math.PI / 4)
              // .splineToConstantHeading(new Vector2d(-60.5,-52.0),3*Math.PI/2)
               .stopAndAdd(() -> Actions.runBlocking(v_factor.V_Basket()))
               .lineToY(-60)
               .stopAndAdd(() -> Actions.runBlocking(grip_factor.V_grip_OPEN()))
               .afterTime(1.8, v_factor.V_Ground())
               .afterTime(2, h_factor.H_OUT())
               .setTangent(Math.PI/2)
               .splineToLinearHeading(new Pose2d(-48.1, -45, Math.PI / 2), Math.PI / 2)
               //.splineToConstantHeading(new Vector2d(-48.1,-45),3*Math.PI/2)
               .stopAndAdd(() -> Actions.runBlocking(h_factor.H_Pick()))
               .stopAndAdd(() -> Actions.runBlocking(grip_factor.H_grip_CLOSE()))
               .stopAndAdd(() -> Actions.runBlocking(h_factor.H_IN()))
               .stopAndAdd(() -> Actions.runBlocking(grip_factor.V_grip_CLOSE()))
               .afterTime(1, grip_factor.H_grip_OPEN())
               .setTangent(Math.PI/4)
               .splineToLinearHeading(new Pose2d(-56, -58.5,  Math.PI / 4), Math.PI / 4)
               //.splineToConstantHeading(new Vector2d(-60.5,-52.0),3*Math.PI/2)
               .stopAndAdd(() -> Actions.runBlocking(v_factor.V_Basket()))
               .stopAndAdd(() -> Actions.runBlocking(grip_factor.V_grip_OPEN()))
               .afterTime(0, grip_factor.H_grip_OPEN())
               .afterTime(1.8, v_factor.V_Ground())
               .afterTime(2, h_factor.H_OUT())
               .setTangent(Math.PI/2)
               .splineToLinearHeading(new Pose2d(-55.6, -45, Math.PI / 2), Math.PI / 2)
               //.splineToConstantHeading(new Vector2d(-55.6,-45),3*Math.PI/2)
               .stopAndAdd(() -> Actions.runBlocking(h_factor.H_Pick()))
               .stopAndAdd(() -> Actions.runBlocking(grip_factor.H_grip_CLOSE()))
               .stopAndAdd(() -> Actions.runBlocking(h_factor.H_IN()))
               .stopAndAdd(() -> Actions.runBlocking(grip_factor.V_grip_CLOSE()))
               .afterTime(1, grip_factor.H_grip_OPEN())
               .setTangent(Math.PI/4)
               .splineToLinearHeading(new Pose2d(-56, -58.5,  Math.PI / 4), Math.PI / 4)
               //.splineToConstantHeading(new Vector2d(-60.5,-52.0),3*Math.PI/2)
               .stopAndAdd(() -> Actions.runBlocking(v_factor.V_Basket()))
               .stopAndAdd(() -> Actions.runBlocking(grip_factor.V_grip_OPEN()))
               .afterTime(0, grip_factor.H_grip_OPEN())
               .afterTime(1.8, v_factor.V_Ground())
               .afterTime(2, h_factor.H_OUT())
               .setTangent(Math.PI/2)
               .splineToLinearHeading(new Pose2d(-62.1, -45, Math.PI / 2), Math.PI / 2)
               //.splineToConstantHeading(new Vector2d(-62.1,-45),3*Math.PI/2)
               .stopAndAdd(() -> Actions.runBlocking(h_factor.H_Pick()))
               .stopAndAdd(() -> Actions.runBlocking(grip_factor.H_grip_CLOSE()))
               .stopAndAdd(() -> Actions.runBlocking(h_factor.H_IN()))
               .stopAndAdd(() -> Actions.runBlocking(grip_factor.V_grip_CLOSE()))
               .setTangent(Math.PI/4)
               .splineToLinearHeading(new Pose2d(-56, -58.5,  Math.PI / 4), Math.PI / 4)
               //.splineToConstantHeading(new Vector2d(-60.5,-52.0),3*Math.PI/2)
               .stopAndAdd(() -> Actions.runBlocking(v_factor.V_Basket()))
               .stopAndAdd(() -> Actions.runBlocking(grip_factor.V_grip_OPEN()))
               .afterTime(1, v_factor.V_Ground());
*/





       Action traj_END = traj.endTrajectory().fresh()
                       .build();


               Actions.runBlocking(grip_factor.H_grip_CLOSE());
               Actions.runBlocking(grip_factor.V_grip_CLOSE());
               Actions.runBlocking(h_factor.H_IN());


       waitForStart();

       Actions.runBlocking(
               new SequentialAction(
                       traj.build(),





                       traj_END
               )
       );

       while (opModeIsActive()) {
           controller.setPID(p, i, d);
           int armPos = AL.getCurrentPosition();
           double pid = controller.calculate(armPos, target);
           double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;

           double power = pid + ff;

           AL.setPower(power);
           AR.setPower(power);



           telemetry.addData("armpower ", power);
           telemetry.addData("armtarget ", target);
           telemetry.update();

           if (isStopRequested()) return;
       }


   }


}

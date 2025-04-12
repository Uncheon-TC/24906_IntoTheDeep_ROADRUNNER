package org.firstinspires.ftc.teamcode.drive.auto_test;

import androidx.annotation.NonNull;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

// Non-RR imports
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Config
@Autonomous(name = "AUTOhang_v2", group = "Autonomous")

public class AUTOHANG_V2 extends LinearOpMode{

    private PIDController controller;

    public static double p = 0.02, i = 0, d = 0.0005;

    public static double f = 0.001;
    public static int target = 0;

    private DcMotorEx AL;
    private DcMotorEx AR;

    private Servo V_wristL;
    private Servo V_wristR;
    private Servo H_lengthL;
    private Servo H_lengthR;
    private Servo H_wristL;
    private Servo H_wristR;
    private Servo H_angleL;
    private Servo H_angleR;
    private Servo V_angleL;
    private Servo V_angleR;
    private final double ticks_in_degree = 700 / 180.0;

    public class H_factor {


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

        int clip_pick = 0;
        int High_backet = 2700;

        int High_chamber_hang = 640;


        //TODO: make rigging mechanism and find tick


        double V_wrist_L_pick = 0.48;
        double V_wrist_R_pick = 0.50;
        double V_wrist_L_hang = 0.36;
        double V_wrist_R_hang = 0.92;
        double V_wrist_L_trans = 0.92;
        double V_wrist_R_trans =0.94;
        double V_wrist_L_backet = 0.45;
        double V_wrist_R_backet = 0.45;

        double V_angle_pick = 0.84;
        double V_angle_up = 0.77;
        double V_angle_hang = 0.22;
        double V_angle_hang_down = 0.14;
        double V_angle_backet =0.3;
        double V_angle_trans_ready =0.54;
        double V_angle_trans = 0.43;



        int chamber_status = 0;



        double H_wristL_Ready = 0.45;
        double H_wristR_Ready = 0.45;

        double H_wrist_L_hide = 0.5;
        double H_wrist_R_hide = 0.49;
        double H_wrist_L_trans = 1;
        double H_wrist_R_trans = 1;

        double H_angleL_back = 0.47;
        double H_angleR_back = 0.47;
        double H_length_L_IN = 0.4;
        double H_length_R_IN = 0.4;
        double H_length_L_OUT = 0.1;
        double H_length_R_OUT = 0.1;
        double H_length_L_trans = 0.60;
        double H_length_R_trans = 0.58;

        double H_angle_Ready = 0.5;
        double H_angle_pickup = 0.69;
        double H_angle_hide = 0.76;
        double H_angle_trans = 0.46;

        public H_factor(HardwareMap hardwareMap) {
            V_wristL = hardwareMap.servo.get("V_wristL");
            V_wristR = hardwareMap.servo.get("V_wristR");//Bucket Wrist left Servo
            V_angleL = hardwareMap.servo.get("V_angleL");
            V_angleR = hardwareMap.servo.get("V_angleR");
            H_angleR = hardwareMap.servo.get("H_angleR");
            H_lengthL = hardwareMap.servo.get("H_lengthL");//Slide right Servo
            H_lengthR = hardwareMap.servo.get("H_lengthR");
            H_wristR = hardwareMap.servo.get("H_wristR"); // Ground Gripper right Servo
            H_wristL = hardwareMap.servo.get("H_wristL"); // Ground Gripper Left Servo
            H_angleR = hardwareMap.servo.get("H_angleR"); // Wrist right Servo
            H_angleL = hardwareMap.servo.get("H_angleL"); // Wrist left Servo

            H_wristL.setDirection(Servo.Direction.REVERSE);
            V_wristL.setDirection(Servo.Direction.REVERSE);

            H_lengthL.setDirection(Servo.Direction.REVERSE);

            H_angleL.setDirection(Servo.Direction.REVERSE);
            V_angleL.setDirection(Servo.Direction.REVERSE);
        }

        public class H_out implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                H_lengthL.setPosition(H_length_L_OUT);
                H_lengthR.setPosition(H_length_R_OUT);
                return false;
            }
        }
        public Action H_OUT() {
            return new H_out();
        }

        public class H_in implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                H_lengthL.setPosition(H_length_L_IN);
                H_lengthR.setPosition(H_length_R_IN);
                return false;
            }
        }
        public Action H_IN() {
            return new H_in();
        }

        public class H_pick implements Action {


            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                H_lengthL.setPosition(H_length_L_OUT);
                H_lengthR.setPosition(H_length_R_OUT);
                H_angleL.setPosition(H_angle_Ready);
                H_angleR.setPosition(H_angle_Ready);



                H_wristL.setPosition(H_wristL_Ready);
                H_wristR.setPosition(H_wristR_Ready);

                return false;
            }
        }

        public Action H_Pick() {
            return new H_pick();
        }

        public class H_up implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {


                H_angleL.setPosition(H_angle_trans);
                H_angleR.setPosition(H_angle_trans);

                H_wristL.setPosition(H_wrist_L_trans);
                H_wristR.setPosition(H_wrist_R_trans);

                H_lengthL.setPosition(H_length_L_IN);
                H_lengthR.setPosition(H_length_R_IN);

                V_angleL.setPosition(V_angle_trans);
                V_angleR.setPosition(V_angle_trans);

                V_wristL.setPosition(V_wrist_L_trans);
                V_wristR.setPosition(V_wrist_R_trans);


                return false;
            }
        }
        public Action H_UP() {
            return new H_up();
        }
    }

    public class V_factor {

        //setting default var

        int target = 0;



        //TODO: find Horizon Griper value

        int clip_pick = 0;
        int High_backet = 2450;

        int High_chamber_hang = 800;


        //TODO: make rigging mechanism and find tick


        double V_wrist_L_pick = 0.5;
        double V_wrist_R_pick = 0.5;
        double V_wrist_L_hang = 97;
        double V_wrist_R_hang = 0.34;
        double V_wrist_L_trans = 0.82;
        double V_wrist_R_trans =0.83;
        double V_wrist_L_backet = 0.45;
        double V_wrist_R_backet = 0.45;

        double V_angle_pick = 0.16;
        double V_angle_up = 0.77;
        double V_angle_hang = 0.82;
        double V_angle_hang_down = 0.14;
        double V_angle_backet =0.3;
        double V_angle_trans_ready =0.54;
        double V_angle_trans = 0.60;



        int chamber_status = 0;



        double H_wristL_Ready = 0.45;
        double H_wristR_Ready = 0.45;

        double H_wrist_L_hide = 0.5;
        double H_wrist_R_hide = 0.5;
        double H_wrist_L_trans = 1;
        double H_wrist_R_trans = 1;

        double H_angleL_back = 0.47;
        double H_angleR_back = 0.47;
        double H_length_L_IN = 0.4;
        double H_length_R_IN = 0.4;
        double H_length_L_OUT = 0.32;
        double H_length_R_OUT = 0.30;
        double H_length_L_trans = 0.60;
        double H_length_R_trans = 0.58;

        double H_angle_Ready = 0.62;
        double H_angle_pickup = 0.69;
        double H_angle_hide = 0.76;
        double H_angle_trans = 0.46;


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

                    init = true;
                }

                target = High_backet;

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
                    AL.setTargetPosition(clip_pick);
                    AR.setTargetPosition(clip_pick);

                    AL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    AR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    // 목표 위치로 이동할 때의 전력 설정
                    AL.setPower(1);
                    AR.setPower(1);
                    // 서보 모터 동시 동작 추가
                    H_lengthL.setPosition(H_length_L_IN);
                    H_lengthR.setPosition(H_length_R_IN);
                    H_angleL.setPosition(H_angle_hide);
                    H_angleR.setPosition(H_angle_hide);
                    H_wristL.setPosition(H_wrist_L_hide);
                    H_wristR.setPosition(H_wrist_R_hide);
                    V_angleL.setPosition(V_angle_pick);
                    V_angleR.setPosition(V_angle_pick);
                    V_wristL.setPosition(V_wrist_L_pick);
                    V_wristR.setPosition(V_wrist_R_pick);
                    init = true;
                }

                target =clip_pick;
                double pos = AL.getCurrentPosition();
                packet.put("AL_POS", pos);

                if (pos > clip_pick) {
                    return true;
                } else {
                    // 현재 위치에 모터 고정
                    AL.setTargetPosition((int)pos);
                    AR.setTargetPosition((int)pos);

                    AL.setPower(0.2);
                    AR.setPower(0.2);

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

                target = High_chamber_hang;



                double pos = AL.getCurrentPosition();
                packet.put("AL_POS", pos);
                if (pos <= High_chamber_hang) {
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

        /*        public class V_chamber_hang implements Action {
                    private boolean init = false;

                    @Override
                    public boolean run (@NonNull TelemetryPacket packet) {
                        if (!init) {
                            AL.setPower(1);
                            AR.setPower(1);
                            init = true;
                        }

                        target = High_chamber_hang;




                        double pos = AL.getCurrentPosition();
                        packet.put("AL_POS", pos);
                        if (pos < High_chamber_hang) {
                            return true;
                        } else {
                            AL.setPower(0);
                            AR.setPower(0);
                            return false;
                        }

                    }
                }*/
        public class V_chamber_hang implements Action {
            private boolean init = false;

            @Override
            public boolean run (@NonNull TelemetryPacket packet) {
                if (!init) {
                    // RUN_TO_POSITION 모드로 설정
                    AL.setTargetPosition(High_chamber_hang);
                    AR.setTargetPosition(High_chamber_hang);

                    AL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    AR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    // 목표 위치로 이동할 때의 전력 설정
                    AL.setPower(1);
                    AR.setPower(1);
                    // 서보 모터 동시 동작 추가
                    V_angleL.setPosition(V_angle_hang);
                    V_angleR.setPosition(V_angle_hang);
                    V_wristL.setPosition(V_wrist_L_hang);
                    V_wristR.setPosition(V_wrist_R_hang);
                    init = true;
                }

                target = High_chamber_hang;
                double pos = AL.getCurrentPosition();
                packet.put("AL_POS", pos);

                if (pos < High_chamber_hang) {
                    return true;
                } else {
                    // 현재 위치에 모터 고정
                    AL.setTargetPosition((int)pos);
                    AR.setTargetPosition((int)pos);

                    AL.setPower(0.2);
                    AR.setPower(0.2);

                    return false;
                }
            }
        }
        public Action V_Chamber_Hang() {
            return new V_chamber_hang();
        }

        public class V_chamber_down implements Action {
            private boolean init = false;

            @Override
            public boolean run (@NonNull TelemetryPacket packet) {
                if (!init) {
                    // RUN_TO_POSITION 모드로 설정
                    AL.setTargetPosition(High_chamber_hang);
                    AR.setTargetPosition(High_chamber_hang);

                    AL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    AR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    // 목표 위치로 이동할 때의 전력 설정
                    AL.setPower(1);
                    AR.setPower(1);
                    // 서보 모터 동시 동작 추가
                    V_angleL.setPosition(V_angle_hang+0.03);
                    V_angleR.setPosition(V_angle_hang+0.03);
                    V_wristL.setPosition(V_wrist_L_hang);
                    V_wristR.setPosition(V_wrist_R_hang);
                    init = true;
                }

                target = High_chamber_hang;
                double pos = AL.getCurrentPosition();
                packet.put("AL_POS", pos);

                if (pos < High_chamber_hang) {
                    return true;
                } else {
                    // 현재 위치에 모터 고정
                    AL.setTargetPosition((int)pos);
                    AR.setTargetPosition((int)pos);

                    AL.setPower(0.2);
                    AR.setPower(0.2);

                    return false;
                }
            }
        }
        public Action V_Chamber_Down() {
            return new V_chamber_down();
        }

        public class clip_pick implements Action {
            private boolean init = false;

            @Override
            public boolean run (@NonNull TelemetryPacket packet) {
                H_lengthL.setPosition(H_length_L_IN);
                H_lengthR.setPosition(H_length_R_IN);
                H_angleL.setPosition(H_angle_hide);
                H_angleR.setPosition(H_angle_hide);
                H_wristL.setPosition(H_wrist_L_hide);
                H_wristR.setPosition(H_wrist_R_hide);
                V_angleL.setPosition(V_angle_pick);
                V_angleR.setPosition(V_angle_pick);
                V_wristL.setPosition(V_wrist_L_pick);
                V_wristR.setPosition(V_wrist_R_pick);
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
                V_angleL.setPosition(V_angle_hang);
                V_angleR.setPosition(V_angle_hang);
                V_wristL.setPosition(V_wrist_L_hang);
                V_wristR.setPosition(V_wrist_R_hang);
                return false;

            }
        }

        public Action chamber() {
            return new chamber_hang();
        }



    }

    public class Grip_factor {
        private Servo H_grip;
        private Servo V_grip;


        double V_Grip_OPEN = 0.43;
        double V_Grip_CLOSE = 0.65;

        double H_Grip_OPEN = 0.47;
        double H_Grip_CLOSE = 0.25;

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

        Pose2d initialPose = new Pose2d(-4, 63, Math.PI /2);
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        H_factor h_factor = new H_factor(hardwareMap);
        V_factor v_factor = new V_factor(hardwareMap);
        Grip_factor grip_factor = new Grip_factor(hardwareMap);

        controller = new PIDController(p, i, d);

        AL = hardwareMap.get(DcMotorEx.class, "AL");
        AR = hardwareMap.get(DcMotorEx.class, "AR");
        AR.setDirection(DcMotorSimple.Direction.REVERSE);



        TrajectoryActionBuilder traj = drive.actionBuilder(initialPose)







                //.stopAndAdd(() -> Actions.runBlocking(v_factor.V_Chamber_Hang()))
                //.stopAndAdd(() -> Actions.runBlocking(grip_factor.V_grip_OPEN()))
                .afterTime(0, v_factor.V_Chamber_Hang())
                .afterTime(0, grip_factor.V_grip_CLOSE())
                .setTangent(Math.PI*3/2)
                .splineToConstantHeading(new Vector2d(-4,27.5),Math.PI*3/2 )
                .stopAndAdd(() -> Actions.runBlocking(grip_factor.V_grip_OPEN()))
                .afterTime(0.2, v_factor.V_Ground())
                .splineToConstantHeading(new Vector2d(-4,30),Math.PI/2 )





                .splineToConstantHeading(new Vector2d(-27.5,30),Math.PI*4/4 )
                //.setTangent(Math.PI*3/2)
                .splineToConstantHeading(new Vector2d(-31,18),Math.PI*3/2)
                .splineToConstantHeading(new Vector2d(-44,13),Math.PI*4/4 )
                .setTangent(Math.PI/2)

                //.splineToConstantHeading(new Vector2d(-43, 13),  Math.PI *3/2)
                //.waitSeconds(0.01)
                //.waitSeconds(0.1)
                //.lineToY(53)
                .splineToConstantHeading(new Vector2d(-44, 42),  Math.PI /2)
                //.waitSeconds(0.1)
                //.lineToY(25)
                .splineToConstantHeading(new Vector2d(-41.5, 24),  Math.PI *3/2)
                .splineToConstantHeading(new Vector2d(-53, 12),  Math.PI *2/2)
                .setTangent(Math.PI/2)
                //.waitSeconds(0.010)



                .splineToConstantHeading(new Vector2d(-53, 41),  Math.PI / 2)

                .splineToConstantHeading(new Vector2d(-50, 15.5),  Math.PI *3/ 2)


                //.setTangent(3 * Math.PI / 2)
                //.splineToConstantHeading(new Vector2d(-50, 46),  Math.PI / 2)


                .splineToConstantHeading(new Vector2d(-62,12),Math.PI*2/ 2)
                .setTangent(Math.PI/2)
                //.waitSeconds(0.01)
                //첫번째 잡기 위치 이동
                .splineToConstantHeading(new Vector2d(-60,60),Math.PI/ 2)
                //.splineToConstantHeading(new Vector2d(-60.5, 56), Math.PI *3/ 2)
                .stopAndAdd(() -> Actions.runBlocking(grip_factor.V_grip_CLOSE()))
                .afterTime(0.1, v_factor.V_Chamber_Hang())
                .setTangent(Math.PI*3/2)

                //.splineTo(new Vector2d(-4, 18), Math.PI *3/ 2)
                //              .splineToConstantHeading(new Vector2d(-5,36),Math.PI*3 / 2)

                .splineTo(new Vector2d(1,32.5),Math.PI*3 / 2)
                .stopAndAdd(() -> Actions.runBlocking(grip_factor.V_grip_OPEN()))
                .afterTime(0.6, v_factor.V_Ground())
                .setTangent(Math.atan2(29.5,-42))

                //.splineTo(new Vector2d(-38, 62), Math.PI / 2)
                .splineToConstantHeading(new Vector2d(-41, 62), Math.atan2(29.5,-42))
                //.splineToConstantHeading(new Vector2d(-40.5, 54.5), Math.PI *3/ 2)
                //.afterTime(0.25, grip_factor.V_grip_CLOSE())
                .stopAndAdd(() -> Actions.runBlocking(grip_factor.V_grip_CLOSE()))
                .afterTime(0.15, v_factor.V_Chamber_Hang())

                //.setTangent(Math.PI*3/2)
                //.splineToConstantHeading(new Vector2d(-25,45),Math.PI/2)
                //.splineTo(new Vector2d(-4, 17), Math.PI *3/ 2)

                //        .splineToConstantHeading(new Vector2d(-5,28),Math.PI *3/ 2)
                .setTangent(Math.toRadians(180)+Math.atan2(30,-44))
                .splineToConstantHeading(new Vector2d(3.5,32),Math.toRadians(180)+Math.atan2(30,-44.5))
                .stopAndAdd(() -> Actions.runBlocking(grip_factor.V_grip_OPEN()))
                .afterTime(0.6, v_factor.V_Ground())

                .setTangent( Math.atan2(30,-44.5))
                .splineToConstantHeading(new Vector2d(-41, 62), Math.atan2(30,-44.5))
                //.splineToConstantHeading(new Vector2d(-40.5, 54.5), Math.PI *3/ 2)
                //.afterTime(0.2, grip_factor.V_grip_CLOSE())
                .stopAndAdd(() -> Actions.runBlocking(grip_factor.V_grip_CLOSE()))
                .afterTime(0.15, v_factor.V_Chamber_Hang())

                //.setTangent(Math.PI*3/2)
                //.splineTo(new Vector2d(-4, 17), Math.PI *3/ 2)
                //        .splineToConstantHeading(new Vector2d(-5,28),Math.PI*3 / 2)
                .setTangent(Math.toRadians(180)+Math.atan2(30,-44.5))
                .splineToConstantHeading(new Vector2d(3.5,32),Math.toRadians(180)+Math.atan2(30,-44.5))
                .stopAndAdd(() -> Actions.runBlocking(grip_factor.V_grip_OPEN()))
                .afterTime(0.6, v_factor.V_Ground())
                .setTangent(Math.atan2(30,-44.5))



                .splineToConstantHeading(new Vector2d(-41, 62), Math.atan2(30,-44.5))
                //.splineToConstantHeading(new Vector2d(-40.5, 54.5), Math.PI *3/ 2)
                //.afterTime(0.2, grip_factor.V_grip_CLOSE())
                .stopAndAdd(() -> Actions.runBlocking(grip_factor.V_grip_CLOSE()))
                .afterTime(0.15, v_factor.V_Chamber_Hang())

                //.setTangent(Math.PI*3/2)

                //.splineTo(new Vector2d(-4, 17), Math.PI *3/ 2)
                //        .splineToConstantHeading(new Vector2d(-5,28),Math.PI *3/ 2)
                .setTangent(Math.toRadians(180)+Math.atan2(31,-44.5))
                .splineToConstantHeading(new Vector2d(3.5,31),Math.toRadians(180)+Math.atan2(31,-44.5))
                .stopAndAdd(() -> Actions.runBlocking(grip_factor.V_grip_OPEN()))
                .afterTime(0.5, v_factor.V_Ground())
                //.setTangent(Math.PI/2)
                //.splineTo(new Vector2d(-40.5, 54.5), Math.PI / 2);
                .setTangent(Math.atan2(31,-44.5))
                .splineToConstantHeading(new Vector2d(-41, 62), Math.atan2(31 ,-44.5));

        //.splineToConstantHeading(new Vector2d(-60, 46),  Math.PI / 2)
        //.setTangent(Math.PI / 2)







        // .lineToY(17)
        //  .splineToConstantHeading(new Vector2d(-61,10),Math.PI/2)
        // .lineToY(55)
        //    .lineToY(-10)
               /*
       .setTangent(3 * Math.PI / 2)
       .splineToLinearHeading(new Pose2d(-12, 35, Math.PI / 2), 3 * Math.PI / 2) //1st chamb
       .waitSeconds(0.5)
       .splineToSplineHeading(new Pose2d(-45, 10,  Math.PI / 2),  Math.PI / 2)
       .splineToSplineHeading(new Pose2d(-36, 8, 3 * Math.PI / 2), 3 * Math.PI / 2)
       .splineToLinearHeading(new Pose2d(-46, 11, 3 * Math.PI / 2), Math.PI / 2)
       .lineToY(50)
       .setTangent(3 * Math.PI / 2)
       .splineToConstantHeading(new Vector2d(-46, 11), 3 * Math.PI / 2)
       .splineToConstantHeading(new Vector2d(-54, 16), Math.PI / 2)
       .lineToY(50)
       .setTangent(3 * Math.PI / 2)
       .splineToConstantHeading(new Vector2d(-54, 11), 3 * Math.PI / 2)
       .splineToConstantHeading(new Vector2d(-60, 16), Math.PI / 2)
       .lineToY(40)
       .splineToSplineHeading(new Pose2d(-47, 60, 3 * Math.PI / 2), Math.PI / 2) //clip pickup
       .waitSeconds(0.5)
       .setTangent(3 * Math.PI / 2)
       .splineToSplineHeading(new Pose2d(-8, 35, Math.PI / 2), 3 * Math.PI / 2) //2nd chamber
       .waitSeconds(0.5)
       .splineToLinearHeading(new Pose2d(-47, 60, 3 * Math.PI / 2), Math.PI / 2)
       .waitSeconds(0.5)
       .splineToLinearHeading(new Pose2d(-4, 35, Math.PI / 2), 3 * Math.PI / 2) //3rd chamber
       .waitSeconds(0.5)
       .splineToLinearHeading(new Pose2d(-47, 60, 3 * Math.PI / 2), Math.PI / 2)
       .waitSeconds(0.5)
       .splineToLinearHeading(new Pose2d(0, 35, Math.PI / 2), 3 * Math.PI / 2) //4th chamber
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

package frc.robot.config;

import org.ejml.simple.SimpleMatrix;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import frc.robot.config.Config.ShooterConfig.BallManagementConfig;
import frc.robot.utils.control.pidf.PIDF;

public class Config {
    public String name = "Shawty";

    //////////////////////////////////////////////////////////////////////////////
    // Subsystem Enablers

    public boolean enableShooterSubsystem = true;
    public boolean enableBallManagementSubsystem = true;
    public boolean enableDriveSubsystem = true;
    public boolean enableClimbSubsystem = true;
    public boolean enableIntakeSubsystem = true;
    public boolean enableSpinnyboiSubsystem = true;
    public boolean enablePIDHelper = false;

    //////////////////////////////////////////////////////////////////////////////
    // Motor IDs

    // Shooter
    public int AZIMUTH_MOTOR_ID = 6;
    public int ELEVATION_MOTOR_ID = 7;
    public int SHOOTER_MOTOR_ID = 15;
    public int SHOOTER_FOLLOWER_MOTOR_ID = 16;
    public int FEEDER_MOTOR_ID = 9;

    public int BALLMANAGEMENT_MOTOR_ID = 14;

    // Intake
    public int INTAKE_MOTOR_ID = 10;

    // Climb
    public int CLIMB_LEFT_MOTOR_ID = 12;
    public int CLIMB_RIGHT_MOTOR_ID = 8;

    // Drive
    public int LEFT_DRIVE_IDS[] = { 2, 3 };
    public int RIGHT_DRIVE_IDS[] = { 1, 4 };

    // SpinnyBoi
    public int SPINNYBOI_MOTOR_ID = 11;

    //////////////////////////////////////////////////////////////////////////////
    // Vision

    //////////////////////////////////////////////////////////////////////////////
    // Motor Configs
    public static class ShooterConfig {
        public float azimuthGearRatio = 28f / 130f;

        public float elevationGearRatio = 40f / 70f;

        public float shooterGearRatio = .45f / 1f;

        public float defaultAzimuthTurnVelocity_deg = 10;
        public float defaultElevationTurnVelocity_deg = 10;

        public double manualAzimuthDeadband = 0.2;
        public double manualElevationDeadband = 0.2;

        public float leftAzimuthSoftLimit_deg = 90;
        public float rightAzimuthSoftLimit_deg = 180;
        
        public float forwardElevationSoftLimit_deg = 60;
        public float backwardElevationSoftLimit_deg = 0;

        public float feederSpinUpDeadband_ticks = 50;

        public MotorConfig azimuth = new MotorConfig();
        public MotorConfig elevation = new MotorConfig();
        public MotorConfig feeder = new MotorConfig();
        public MotorConfig shooter = new MotorConfig();
        public MotorConfig shooterFollower = new MotorConfig();

        // meters as all good things are
        // relative to center of robot
        // not necessarily CoM of robot but like, same thing
        public double xTurretCenter = 0.17181322;
        public double yTurretCenter = 0.01766824;
        // distance from center of turret to LL camera
        public double rLL = 0.19368008;

        public float[] elevationPositions_deg = new float[] { //
                0, /// Comment so that VSCode doesn't ruin my format.
                10, // Comment so that VSCode doesn't ruin my format.
                40, // Comment so that VSCode doesn't ruin my format.
                50, // Comment so that VSCode doesn't ruin my format.
                60 /// Comment so that VSCode doesn't ruin my format.
        };

        public static class BallManagementConfig {
            public MotorConfig spinner = new MotorConfig();

            public BallManagementConfig() {
                spinner.encoderType = MotorConfig.EncoderType.None;
            }
        }

        public ShooterConfig() {
            shooter.encoderType = MotorConfig.EncoderType.Integrated;
            shooter.inverted = true;
            shooterFollower.inverted = true;

            azimuth.motionMagicAcceleration = 1350;
            azimuth.motionMagicCruiseVelocity = 1350;

            elevation.motionMagicAcceleration = 1350;
            elevation.motionMagicCruiseVelocity = 1350;

            azimuth.inverted = true;
            elevation.inverted = false;
        }
    }

    public static class IntakeConfig {
        public boolean intakePivotEnabled = true;

        public MotorConfig intake = new MotorConfig();
    }

    public static class ClimbConfig {

        public MotorConfig climbLeft = new MotorConfig();
        public MotorConfig climbRight = new MotorConfig();

        ClimbConfig(){
            climbLeft.inverted = true;
        }
    }

    public static class SpinnyBoiConfig {
        public MotorConfig spinner = new MotorConfig();
    }

    public static class DriveConfig {
        public double maxAllowedSpeed_ips = 14 * 12.0;//39.3701;
        public double maxAllowedTurn_degps = 180;

        public int MOTORS_PER_SIDE = 2;
        public MotorConfig leftMotors[];
        public MotorConfig rightMotors[];

        public int[] leftIDs;
        public int[] rightIDs;

        public MotorConfig leftLeader;
        public MotorConfig rightLeader;

        public boolean leftInverted = false;
        public boolean rightInverted = false;
        public boolean invertLeftCommand = false;
        public boolean invertRightCommand = true;

        public MotorConfig.EncoderType encoderType = MotorConfig.EncoderType.Integrated;

        /** Gear ratio from encoder to wheel. gearRatio encoder turns = 1 wheel turn */
        public double gearRatio = (10 + 8.0 / 9);
        public double ticksPerRevolution = 2048;
        public double wheelRadius_in = 3;
        public double trackWidth_in = 23.90069263162104;

        public double ROTATION_DRIVE_KP = 5 * 2 * Math.PI / 360;

        public SimpleMotorFeedforward characterization = new SimpleMotorFeedforward(0.163, 2.46, 0.251);



        public boolean useFancyOdometry = false;

        // these are matrices used for describing the time evolution of our drive base's velocities
        // CODE REVIEW PARTY: feel free to ignore these
        // the state is x = [left velocity, right velocity]'
        // the input is u = [left voltage,  right voltage]'
        // in discrete time: x(t + 20ms) = Ad*x(t) + Bd*u(t)
        // in continuous time: x' = Ac*x(t) + Bc*u(t)
        
        // discrete time
        public SimpleMatrix Ad_char = new SimpleMatrix(2, 2, true, new double[] {
            0.8678678537600736, 0.0071917452743623786,
            0.005903949588909425, 0.8723072041767262
        });

        public SimpleMatrix Bd_char = new SimpleMatrix(2, 2, true, new double[] {
            0.050392385308988936, -0.004313538469407867,
            -0.002206154164037924, 0.047675029476658845
        });

        // continuous time
        public SimpleMatrix Ac_char = new SimpleMatrix(2, 2, true, new double[] {
            -7.087195478354283, 0.413285738104402,
            0.339280393075371, -6.832080740045777
        });

        // continuous time system noise
        public SimpleMatrix QVc_char = new SimpleMatrix(2, 2, true, new double[] {
            0.359432446406913, 0.349630210052408,
            0.349630210052408, 0.366367970306715
        });

        public SimpleMatrix Bc_char = new SimpleMatrix(2, 2, true, new double[] {
            2.702895517197959, -0.241632861263366,
            -0.126961060623545, 2.551095849721741
        });



        public DriveConfig() {
        }

        /**
         * Based on the MOTORS_PER_SIDE
         */
        public void initMotorConfigArrays() {
            leftMotors = new MotorConfig[MOTORS_PER_SIDE];
            rightMotors = new MotorConfig[MOTORS_PER_SIDE];
            for (int i = 0; i < MOTORS_PER_SIDE; i++) {
                leftMotors[i] = new MotorConfig();
                leftMotors[i].id = leftIDs[i];
                leftMotors[i].encoderType = encoderType;
                leftMotors[i].inverted = leftInverted;

                rightMotors[i] = new MotorConfig();
                rightMotors[i].id = rightIDs[i];
                rightMotors[i].encoderType = encoderType;
                rightMotors[i].inverted = rightInverted;

                if (i > 0) {
                    leftMotors[i].followingID = leftIDs[0];
                    rightMotors[i].followingID = rightIDs[0];
                }
            }
            leftLeader = leftMotors[0];
            rightLeader = rightMotors[0];
        }
    }

    public static class AutoConfig {
        public double cruiseSpeed_mps = 0.3*14;
        public double maxAcceleration_mps = 2;

        public double b = 2;
        public double zeta = 0.35;

        public double kP = 0.911;
    }

    public ShooterConfig shooter = new ShooterConfig();
    public BallManagementConfig ballManagement = new BallManagementConfig();
    public DriveConfig drive = new DriveConfig();
    public IntakeConfig intake = new IntakeConfig();
    public SpinnyBoiConfig spinnyboi = new SpinnyBoiConfig();
    public ClimbConfig climb = new ClimbConfig();
    public AutoConfig auto = new AutoConfig();

    public Config() {

        //////////////////////////////////////////////////////////////////////////////
        // IDs (Again)

        // Shooter
        shooter.azimuth.id = AZIMUTH_MOTOR_ID;
        shooter.elevation.id = ELEVATION_MOTOR_ID;
        shooter.feeder.id = FEEDER_MOTOR_ID;
        shooter.shooter.id = SHOOTER_MOTOR_ID;
        shooter.shooterFollower.id = SHOOTER_FOLLOWER_MOTOR_ID;
        ballManagement.spinner.id = BALLMANAGEMENT_MOTOR_ID;

        // Intake
        intake.intake.id = INTAKE_MOTOR_ID;

        // Climb
        climb.climbRight.id = CLIMB_RIGHT_MOTOR_ID;
        climb.climbLeft.id = CLIMB_LEFT_MOTOR_ID;

        // SpinnyBoi
        spinnyboi.spinner.id = SPINNYBOI_MOTOR_ID;

        // Drive
        drive.leftIDs = LEFT_DRIVE_IDS;
        drive.rightIDs = RIGHT_DRIVE_IDS;

        //////////////////////////////////////////////////////////////////////////////
        // PIDFs

        // Shooter
        shooter.azimuth.positionPIDF = new PIDF(//
                0.1 * 1023f / 176 * 2 * 2 * 2 * 2, // P
                0, // I
                10 * 0.1 * 1023f / 176 * 2 * 2 * 2 * 2, // D
                1023f / 2650 /// F
        );
        shooter.elevation.positionPIDF = new PIDF(//
                0.1 * 1023f / 176 * 2 * 2 * 2 * 2, // P
                0.0001, // I
                10 * 0.1 * 1023f / 176 * 2 * 2 * 2 * 2, // D
                1023f / 2650, /// F
                300
        );
        shooter.shooter.velocityPIDF = new PIDF(//
                1023.0, // P
                0, // I
                0, // D
                0, /// F,
                0
        );

        // Climb
        climb.climbLeft.positionPIDF = new PIDF(//
                0.1 * 1023f / 176 * 2 * 2 * 2 * 2, // P
                0, // I
                10 * 0.1 * 1023f / 176 * 2 * 2 * 2 * 2, // D
                1023f / 2650 /// F
        );
        
        climb.climbRight.positionPIDF = new PIDF(//
                0.1 * 1023f / 176 * 2 * 2 * 2 * 2, // P
                0, // I
                10 * 0.1 * 1023f / 176 * 2 * 2 * 2 * 2, // D
                1023f / 2650 /// F
        );

        // SpinnyBoi
        spinnyboi.spinner.positionPIDF = new PIDF(//
                0.05, // P
                0, // I
                0, // D
                0 /// F
        );

        // Drive
        drive.initMotorConfigArrays();

        drive.leftLeader.velocityPIDF = new PIDF(//
                0.00539*2, // P
                0.001,//0.0, // I
                0.0539, // D
                1023 / 21740f, /// F,
                300
        );
        drive.leftLeader.positionPIDF = new PIDF(//
                0, // P
                0, // I
                0, // D
                0 /// F
        );
        drive.rightLeader.velocityPIDF = new PIDF(//
                0.00539, // P
                0.001,//0.0, // I
                0.0539, // D
                1023 / 21340f, /// F
                300
        );
        drive.rightLeader.positionPIDF = new PIDF(//
                0, // P
                0, // I
                0, // D
                0 /// F
        );

        //////////////////////////////////////////////////////////////////////////////
        // Ticks Per Revolution
        shooter.azimuth.ticksPerRevolution = 4096;
        shooter.elevation.ticksPerRevolution = 8192;
        shooter.shooter.ticksPerRevolution = 2048;
        
        //////////////////////////////////////////////////////////////////////////////
        // Follower IDs
        shooter.shooterFollower.followingID = SHOOTER_MOTOR_ID;
    }

}
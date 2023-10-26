package com.team254.frc2022;

import java.net.NetworkInterface;
import java.net.SocketException;
import java.util.Enumeration;

import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.swerve.SwerveDriveKinematics;
import com.team254.lib.swerve.SwerveSetpointGenerator.KinematicLimits;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public class Constants {
    public static final double kLooperDt = 0.01;

    // CAN
    public static final int kCANTimeoutMs = 10; // use for important on the fly updates
    public static final int kLongCANTimeoutMs = 100; // use for constructors
    public static final String kRioCANBusName = "rio";
    public static final String kCANivoreCANBusName = "canivore";
    public static final double kCancoderBootAllowanceSeconds = 10.0;

    // Swerve Config
    public static final String kPracticeBotMACAddress = "00:80:2F:33:D1:5F";
    public static final boolean kPracticeBot = getMACAddress().equals(kPracticeBotMACAddress);
    public static final boolean kUseVelocityDrive = true;

    // Controls
    public static final boolean kForceDriveGamepad = false;
    public static final boolean kUseHeadingController = false;
    public static final int kDriveGamepadPort = 0;
    public static final int kMainThrottleJoystickPort = 0;
    public static final int kMainTurnJoystickPort = 1;
    public static final int kOperatorControllerPort = 2;
    public static final double kDriveJoystickThreshold = 0.075;
    public static final double kJoystickThreshold = 0.1;
    public static final boolean kLimelightSnapshotMode = false;

    public static final int kPigeonIMUId = 20;

    // Drive constants
    public static final double kDriveReduction = (14.0 / 50.0) * (28.0 / 16.0) * (15.0 / 45.0);
    public static final double kSteerReduction = (14.0 / 50.0) * (10.0 / 60.0);
    public static final double kDriveWheelDiameter = 0.10033 * 81.0 / 84.213; /// meters, TODO measure
    public static final double kDriveTrackwidthMeters = 0.61595; // DONE Measure and set trackwidth
    public static final double kDriveWheelbaseMeters = 0.61595; // DONE Measure and set wheelbase

    public static final double kMaxDriveVoltage = 12.0;

    // Measure the drivetrain's maximum velocity or calculate the theoretical.
    //  The formula for calculating the theoretical maximum velocity is:
    //   <Motor free speed RPM> / 60 * <Drive reduction> * <Wheel diameter meters> * pi
    // TODO measure this
    public static final double kMaxVelocityMetersPerSecond = 4.959668;

    // Robot constants
    public static final double kMaxDriveAcceleration = 1867 * 0.8;   // m/s^2 tuned 2/18 practice bot
    public static final double kTrackScrubFactor = 1;


    /**
     * The maximum angular velocity of the robot in radians per second.
     * <p>
     * This is a measure of how fast the robot can rotate in place.
     */
    // Here we calculate the theoretical maximum angular velocity. You can also replace this with a measured amount.
    public static final double kMaxAngularVelocityRadiansPerSecond = 11.386413;

    public static final double kScaleTranslationInputs = 0.5;
    public static final double kScaleRotationInputs = 0.2;

    public static final KinematicLimits kUncappedKinematicLimits = new KinematicLimits();
    static {
        kUncappedKinematicLimits.kMaxDriveVelocity = Constants.kMaxVelocityMetersPerSecond;
        kUncappedKinematicLimits.kMaxDriveAcceleration = Double.MAX_VALUE;
        kUncappedKinematicLimits.kMaxSteeringVelocity = Double.MAX_VALUE;
    }

    public static final KinematicLimits kAzimuthOnlyKinematicLimits = new KinematicLimits();
    static {
        kAzimuthOnlyKinematicLimits.kMaxDriveVelocity = Constants.kMaxVelocityMetersPerSecond;
        kAzimuthOnlyKinematicLimits.kMaxDriveAcceleration = Double.MAX_VALUE;
        kAzimuthOnlyKinematicLimits.kMaxSteeringVelocity = Units.degreesToRadians(1500.0);
    }

    public static final KinematicLimits kTeleopKinematicLimits = new KinematicLimits();
    static {
        kTeleopKinematicLimits.kMaxDriveVelocity = Constants.kMaxVelocityMetersPerSecond;
        kTeleopKinematicLimits.kMaxDriveAcceleration = kTeleopKinematicLimits.kMaxDriveVelocity / 0.1;
        kTeleopKinematicLimits.kMaxSteeringVelocity = Units.degreesToRadians(1500.0);
    }

    public static final KinematicLimits kFastKinematicLimits = new KinematicLimits();
    static {
        kFastKinematicLimits.kMaxDriveVelocity = Constants.kMaxVelocityMetersPerSecond;
        kFastKinematicLimits.kMaxDriveAcceleration = kFastKinematicLimits.kMaxDriveVelocity / 0.2;
        kFastKinematicLimits.kMaxSteeringVelocity = Units.degreesToRadians(1000.0);
    }

    public static final KinematicLimits kSmoothKinematicLimits = new KinematicLimits();
    static {
        kSmoothKinematicLimits.kMaxDriveVelocity = Constants.kMaxVelocityMetersPerSecond * .7;
        kSmoothKinematicLimits.kMaxDriveAcceleration = kSmoothKinematicLimits.kMaxDriveVelocity / 1.0;
        kSmoothKinematicLimits.kMaxSteeringVelocity = Units.degreesToRadians(750.0);
    }

    public static final SwerveDriveKinematics kKinematics = new SwerveDriveKinematics(
            // Front left
            new Translation2d(Constants.kDriveTrackwidthMeters / 2.0, Constants.kDriveWheelbaseMeters / 2.0),
            // Front right
            new Translation2d(Constants.kDriveTrackwidthMeters / 2.0, -Constants.kDriveWheelbaseMeters / 2.0),
            // Back left
            new Translation2d(-Constants.kDriveTrackwidthMeters / 2.0, Constants.kDriveWheelbaseMeters / 2.0),
            // Back right
            new Translation2d(-Constants.kDriveTrackwidthMeters / 2.0, -Constants.kDriveWheelbaseMeters / 2.0)
    );



    // TODO rename these to be steer/drive
    public static final double kMk4AziKp = 0.75;
    public static final double kMk4AziKi = 0;
    public static final double kMk4AziKd = 15;

    public static final double kMk4DriveVelocityKp = 0.1;
    public static final double kMk4DriveVelocityKi = 0.0;
    public static final double kMk4DriveVelocityKd = 0.01;
    public static final double kMk4DriveVelocityKf = 1023 / (kMaxVelocityMetersPerSecond / (Math.PI * Constants.kDriveWheelDiameter * Constants.kDriveReduction / 2048.0 * 10));

    public static final double kMaxAngularSpeedRadiansPerSecond = kMaxVelocityMetersPerSecond /
            Math.hypot(kDriveTrackwidthMeters / 2.0, kDriveWheelbaseMeters / 2.0);
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = kMaxVelocityMetersPerSecond /
            Math.hypot(kDriveTrackwidthMeters / 2.0, kDriveWheelbaseMeters / 2.0);
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);




    public static final double kShooterKp = 0.03;
    public static final double kShooterKi = 0.0003;
    public static final double kShooterIZoneRPM = 100.0;
    public static final double kShooterKd = 0.0;
    public static final double kShooterKf = 0.04782;
    public static final double kShooterTicksPerRevolution = 2048.0 / (14.0 / 72.0); // based on gear reduction between encoder and output shaft, and encoder port
    public static final double kShooterAllowablePercentError = 0.96;

    // CANdle
    public static final int kCANdleId = 30;

    // Shoot on move
    public static final boolean kAllowShootOnMove = true;
    public static final double kMaxTurretAdjustmentForShootOnMove = 45.0; // degrees

    // Pneumatics
    public static final int kAnalogSensorChannel = 0;

    // Turret
    public static final boolean kTurretHomingUseReverseLimit = kPracticeBot ? false : true;


    // Swerve Heading Controller
    public static final double kSwerveHeadingControllerErrorTolerance = 1.5; // degree error

    //TODO tune heading controller snap PID
    public static final double kSnapSwerveHeadingKp = 0.05;
    public static final double kSnapSwerveHeadingKi = 0.0;
    public static final double kSnapSwerveHeadingKd = 0.0075;

    //TODO tune heading controller maintain PID
    public static final double kMaintainSwerveHeadingKp = 0.01;
    public static final double kMaintainSwerveHeadingKi = 0.0;
    public static final double kMaintainSwerveHeadingKd = 0.0;

    //TODO tune radius controller snap PID
    public static final double kSnapRadiusKp = 2.0;
    public static final double kSnapRadiusKi = 0.0;
    public static final double kSnapRadiusKd = 0.0;

    //TODO tune radius controller maintain PID
    public static final double kMaintainRadiusKp = 1.5;
    public static final double kMaintainRadiusKi = 0.0;
    public static final double kMaintainRadiusKd = 0.0;



    // Camera Calibration
    public static final double kHorizontalFOV = 29.8 * 2; //degrees
    public static final double kVerticalFOV = 24.85 * 2; //degrees
    public static final double kResolutionWidthHalf = 960.0 / 2.0; //pixels
    public static final double kResolutionHeightHalf = 720.0 / 2.0; //pixels
    public static final double kLensHeight = 0.9190187; // Sensor to floor
    public static final Rotation2d kHorizontalPlaneToLens = Rotation2d.fromDegrees(35.7); // Measured on comp bot
    public static final double kVisionTargetHeight = 2.64;
    public static final double kTurretRotationFudgeFactor = -1.5;
    public static final Pose2d kTurretToLens =  new Pose2d(new Translation2d(-0.1882556, 0.0), Rotation2d.fromDegrees(kTurretRotationFudgeFactor)); // Turret center to sensor

    public static final double kVisionTargetToGoalCenter = 0.67786; //meters
    public static final double kLimelightTransmissionTimeLatency = 0.0 / 1000.0; // seconds
    public static final double kImageCaptureLatency = 11.0 / 1000.0; // seconds
    public static final double kMaxValidAziVelocity = 50.0;

    // Hood Tuning
    public static final double kDefaultShooterRPM = 600;
    public static final boolean kIsShooterTuning = false;

    // Positive direction means bias shots away from robot.
    // This number is adjustable in SmartDashboard / Shuffleboard as "Shooter range offset"
    public static final double kRangeOffset = -0.15; // meters

    public static final boolean kUseSinMap = true;
    // Sin Map Parameters
    public static final double kVerticalRpmPerMeter = 30.0;
    public static final double kVerticalRpmOffset = 625.0;
    public static final double kHorizontalRpmPerMeter = 93.142;
    public static final double kHorizontalRpmOffset = 20.0;
    // Exit angle of ball (in degrees) when hood is at "0" as measured on video
    public static final double kHoodAngleOffset = 12.0;




    // Climber
    public static final double kClimberKp = 0.13;
    public static final double kClimberKi = 0;
    public static final double kClimberKd = 0;

    public static final double kClimberTraversalTransferKp = 0.13 / 8;
    public static final double kClimberTraversalTransferKi = 0;
    public static final double kClimberTraversalTransferKd = 0;

    public static final double kClimberTicksToInches =
            1.0 / 2048.0 * 11.0 / 40.0 * 22.0 / 38.0 * 14.0 / 48.0          // ticks to sprocket revs
                    * 0.25 / Math.sin(Math.toRadians(180.0 / 22.0)) * Math.PI;  // sprocket revs to inches of travel



    public static final Translation2d goalToTruss = new Translation2d(-7, 2.5);
    public static final Translation2d goalToMiddleDriverStation = new Translation2d(-7, 0);
    public static final boolean kTrussEject = true;


    /**
     * @return the MAC address of the robot
     */
    public static String getMACAddress() {
        try {
            Enumeration<NetworkInterface> nwInterface = NetworkInterface.getNetworkInterfaces();
            StringBuilder ret = new StringBuilder();
            while (nwInterface.hasMoreElements()) {
                NetworkInterface nis = nwInterface.nextElement();
                System.out.println("NIS: " + nis.getDisplayName());
                if (nis != null && "eth0".equals(nis.getDisplayName())) {
                    byte[] mac = nis.getHardwareAddress();
                    if (mac != null) {
                        for (int i = 0; i < mac.length; i++) {
                            ret.append(String.format("%02X%s", mac[i], (i < mac.length - 1) ? ":" : ""));
                        }
                        String addr = ret.toString();
                        System.out.println("NIS " + nis.getDisplayName() + " addr: " + addr);
                        return addr;
                    } else {
                        System.out.println("Address doesn't exist or is not accessible");
                    }
                } else {
                    System.out.println("Skipping adaptor: " + nis.getDisplayName());
                }
            }
        } catch (SocketException | NullPointerException e) {
            e.printStackTrace();
        }

        return "";
    }

    //Pure Pursuit Constants
    public static final double kPathLookaheadTime = 0.25; // From 1323 (2019)
    public static final double kPathMinLookaheadDistance = 12.0; //From 1323 (2019)
    public static final double kAdaptivePathMinLookaheadDistance = 6.0;
    public static final double kAdaptivePathMaxLookaheadDistance = 24.0;
    public static final double kAdaptiveErrorLookaheadCoefficient = 0.01;

}

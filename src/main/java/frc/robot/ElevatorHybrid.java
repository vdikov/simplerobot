package frc.robot;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Rotations;

// Note: In your code, this class will likely be named "Elevator" instead.
public class ElevatorHybrid extends SubsystemBase {

    /* These are the CAN IDs that will be used for the motors. */
    private static int TOP_ELEVATOR_CAN_ID = 16;
    private static int BOTTOM_ELEVATOR_CAN_ID = 17;

    /* Set up the TalonFX objects so we can actually control the motors. */
    private final TalonFX topMotor = new TalonFX(TOP_ELEVATOR_CAN_ID);
    private final TalonFX bottomMotor = new TalonFX(BOTTOM_ELEVATOR_CAN_ID);

    /* How many inches the elevator rises per one motor revolution. */
    private final double ELEVATOR_TRAVEL_INCHES_PER_MOTOR_ROTATION = 0.94;

    /* How high the motion profiling routine should raise the elevator. */
    private Distance elevatorHeightSetpoint = Inches.of(0);

    /*
    This will configure the PID (Feedback) gains, and Feedforward gains of the motors,
    alongside the direction of the shaft rotation.
     */
    private final TalonFXConfiguration motorConfiguration =
        new TalonFXConfiguration()
            .withSlot0(
                new Slot0Configs()
                    .withKP(0)
                    .withKI(0)
                    .withKD(0)
                    .withKS(0)
                    .withKV(0)
                    .withKA(0)
                    .withKG(0)
            ).withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.Clockwise_Positive));

    private PositionVoltage motorPositionalControl = new PositionVoltage(0);

    /* These are used to put data on NetworkTables so we can read the data from AdvantageScope. */
    private final DoublePublisher profiledSetpointRotationsPublisher;
    private final DoublePublisher currentPositionRotationsPublisher;

    // The constants that define the motion profile.
    private final TrapezoidProfile.Constraints motionProfilingConstants = new TrapezoidProfile.Constraints(3, 3);
    // The motion profile itself; this is what we use to actually run calculations for the motion profile
    private final TrapezoidProfile motionProfile;
    // This is the current "next step" of the motion profile that gets calculated. The motor tries to reach this position, though it changes every loop.
    private TrapezoidProfile.State motionProfileSetpoint = new TrapezoidProfile.State(0,0);

    public ElevatorHybrid(NetworkTableInstance nt) {
        /* Apply the settings to the motors. */
        topMotor.getConfigurator().apply(motorConfiguration);
        bottomMotor.getConfigurator().apply(motorConfiguration);

        /*
        Set up the Publishers to publish to a networktable (i.e. a folder where it will be published).
        */
        profiledSetpointRotationsPublisher = nt.getDoubleTopic("/Elevator/Motion Profile Setpoint (rotations)").publish();
        currentPositionRotationsPublisher = nt.getDoubleTopic("/Elevator/Current Elevator Position (rotations)").publish();

        /* Make the bottom motor spin in the same direction and copy the output of the top motor. */
        bottomMotor.setControl(new Follower(TOP_ELEVATOR_CAN_ID, MotorAlignmentValue.Aligned));

        motionProfile = new TrapezoidProfile(motionProfilingConstants);

    }

    /**
     * @return The current height of the elevator calculated from the current motor positions.
     */
    private Angle turnElevatorHeightToMotorRotations(Distance elevatorHeight) {
        return Rotations.of(elevatorHeight.in(Inches) / ELEVATOR_TRAVEL_INCHES_PER_MOTOR_ROTATION);
    }

    /**
     * This code essentially runs in a loop. It will run once every 20 ms, or 50 times a second.
     * This code will run the motion profiling routine to drive the motor to a new position when
     * {@code elevatorHeightSetpoint} changes.
     */
    @Override
    public void periodic() {
        motionProfilePeriodic();
    }

    public void motionProfilePeriodic() {
        motionProfileSetpoint = motionProfile.calculate(0.02, motionProfileSetpoint, new TrapezoidProfile.State(turnElevatorHeightToMotorRotations(elevatorHeightSetpoint).in(Rotations), 0.0));

        profiledSetpointRotationsPublisher.accept(motionProfileSetpoint.position);
        currentPositionRotationsPublisher.accept(topMotor.getRotorPosition().getValueAsDouble());

        topMotor.setControl(motorPositionalControl.withPosition(Rotations.of(motionProfileSetpoint.position)));


    }

}
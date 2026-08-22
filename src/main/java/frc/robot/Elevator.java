package frc.robot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
    private static String RIO_CAN_LOOP_NAME = "rio";
    private static int TOP_ELEVATOR_CAN_ID = 16;
    private static int BOTTOM_ELEVATOR_CAN_ID = 17;

    private final TalonFX topElevator = new TalonFX(TOP_ELEVATOR_CAN_ID, RIO_CAN_LOOP_NAME);
    private final TalonFX bottomElevator = new TalonFX(BOTTOM_ELEVATOR_CAN_ID, RIO_CAN_LOOP_NAME);

    private VoltageOut elevatorVoltage = new VoltageOut(0.0);

    // Voltage based routine variables.
    ///////////////////////////////////

    // Voltage based routine timer. Used to determine when to change from one
    // phase of the routine to another.
    private final Timer autoTimer = new Timer();

    // Motion profile based routine variables.
    //////////////////////////////////////////

    // Motion profile constraints: max velocity (m/s) and max acceleration (m/s^2)
    private final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(3, 3);
    // WPILib Motion Profiler
    private final TrapezoidProfile profile = new TrapezoidProfile(constraints);
    // Target goal setpoint and current calculated profile state.
    private TrapezoidProfile.State goal = new TrapezoidProfile.State(0, 0);
    private TrapezoidProfile.State setpoint = new TrapezoidProfile.State(0, 0);

    // Feedback and feedforward controllers driving the elevator, following the
    // motion profile
    // setpoints.
    private final PIDController pid = new PIDController(4, 0.0, 0.0);
    private final ElevatorFeedforward feedforward = new ElevatorFeedforward(0.05, 0.28, 4.5, 3);
    // ks = 0.138
    // kg = 0.20163

    // Publishers.
    //////////////
    private final DoublePublisher profiledSetPointMetersPublisher;
    private final DoublePublisher currentPositionMetersPublisher;
    private final DoublePublisher voltagePublisher;

    public Elevator(NetworkTableInstance nt) {
        // Configure the motors.
        TalonFXConfiguration configs = new TalonFXConfiguration()
                .withMotorOutput(
                        new MotorOutputConfigs()
                                .withInverted(InvertedValue.Clockwise_Positive)
                                .withNeutralMode(NeutralModeValue.Brake))
                .withCurrentLimits(
                        new CurrentLimitsConfigs()
                                .withSupplyCurrentLimit(Amps.of(17)));
        topElevator.getConfigurator().apply(configs);
        bottomElevator.getConfigurator().apply(configs);

        bottomElevator.setControl(
                new Follower(TOP_ELEVATOR_CAN_ID, MotorAlignmentValue.Aligned));

        var elevatorTable = nt.getTable("Robot/Elevator");
        profiledSetPointMetersPublisher = elevatorTable.getDoubleTopic("profiledSetPointMeters").publish();
        currentPositionMetersPublisher = elevatorTable.getDoubleTopic("currentPositionMeters").publish();
        voltagePublisher = elevatorTable.getDoubleTopic("voltage").publish();
    }

    // Call from Robot.autonomousInit() to initialize auto states.
    public void autonomousInit() {
        // Resets timer to 0 and starts running
        autoTimer.reset();
        autoTimer.start();

        // Reset the encoder position to 0 at the start of autonomous
        topElevator.setPosition(0.0);
        goal = new TrapezoidProfile.State(0.4, 0.0);
    }

    // Helper function to compute elevator vertical position from
    // the rotation position of the motors.
    private Distance curPosition() {
        return Inches.of(topElevator.getPosition().getValueAsDouble());
    }

    // Implements a simple auto procedure with three fixed voltage/duration stages
    // Must be called from Robot.autonomousPeriodic().
    public void autonomousVoltagePeriodic() {
        double elapsedTime = autoTimer.get();
        Voltage voltageOutput;
        if (elapsedTime < 2.0) {
            // Run motors at 50% for 2 seconds
            voltageOutput = Volts.of(1.0);
        } else if (elapsedTime < 6.0) {
            // Stop motors after 2 seconds
            voltageOutput = Volts.of(-0.4);
        } else {
            // Stop motors after that
            voltageOutput = Volts.of(0.0);
        }

        topElevator.setControl(
                elevatorVoltage.withOutput(voltageOutput));

        currentPositionMetersPublisher.set(curPosition().in(Meters));
        voltagePublisher.set(voltageOutput.in(Volts));
    }

    // Implements an auto procedure lifting the elevator according to a motion
    // profile.
    // Must be called from Robot.autonomousPeriodic().
    public void autonomousProfiledPeriodic() {
        // 1. Calculate the next profile state step (dT is typically 0.02s for periodic)
        setpoint = profile.calculate(0.02, setpoint, goal);

        // 2. Get current position reading from the motor (in rotations)
        // Since for this mechanism, one rotation ~ 1 inch, we can use these as inches.
        double currentPosition = curPosition().in(Meters);

        // 3. Calculate PID output based on setpoint position vs current position
        double pidOutputVolts = pid.calculate(currentPosition, setpoint.position);

        // 4. Calculate Feedforward voltage using position setpoint velocity &
        // acceleration
        double ffOutputVolts = feedforward.calculate(setpoint.velocity);

        // 5. Apply total output voltage to the TalonFX via VoltageOut
        double totalVoltage = pidOutputVolts + ffOutputVolts;
        topElevator.setControl(elevatorVoltage.withOutput(totalVoltage));

        profiledSetPointMetersPublisher.set(setpoint.position);
        currentPositionMetersPublisher.set(currentPosition);
        voltagePublisher.set(totalVoltage);
    }
}

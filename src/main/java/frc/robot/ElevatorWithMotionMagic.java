package frc.robot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorWithMotionMagic extends SubsystemBase {
    private static String RIO_CAN_LOOP_NAME = "rio";
    private static int TOP_ELEVATOR_CAN_ID = 16;
    private static int BOTTOM_ELEVATOR_CAN_ID = 17;

    private final CANBus bus = new CANBus(RIO_CAN_LOOP_NAME);
    private final TalonFX topMotor = new TalonFX(TOP_ELEVATOR_CAN_ID, bus);
    private final TalonFX bottomMotor = new TalonFX(BOTTOM_ELEVATOR_CAN_ID, bus);
    // Reusable control request for Motion Magic position control.
    private final MotionMagicVoltage elevatorPositionRequest = new MotionMagicVoltage(0);
    // Reusable control request for duty cycle output.
    private final DutyCycleOut elevatorDutyCycleRequest = new DutyCycleOut(0);

    // Voltage based routine timer. Used to determine when to change from one
    // phase of the routine to another.
    private final Timer autoTimer = new Timer();
    private double timerWatermark = 0.0;

    // Simulation-only definitions
    private final TalonFXSimState topMotorSim = topMotor.getSimState();
    private final ElevatorSim elevatorSim;
    private final StatusSignal<Double> topMotorSetpointRotationsSignal = topMotor.getClosedLoopReference();
    private final StatusSignal<Angle> topMotorPositionRotationsSignal = topMotor.getPosition();

    // Constants for the mechanism simulation parameters
    // 70:12 gearbox; corresponds to motor-rotations/sprocket-rotations conversion
    private static final double kGearReduction = 70.0 / 12.0;
    private static final Mass kCarriageMass = Kilograms.of(5.0); // 5 kg elevator carriage
    private static final Distance kSprocketPitchRadius = Inches.of(1.751 / 2.0); // 1" radius pulley
    private static final Distance kMinHeight = Meters.of(0.0);
    private static final Distance kMaxHeight = Meters.of(0.5);

    private final DoublePublisher profiledSetPointMetersPublisher;
    private final DoublePublisher currentPositionMetersPublisher;
    private final DoublePublisher voltagePublisher;
    private final BooleanPublisher isNewSetpointPublisher;

    public ElevatorWithMotionMagic(NetworkTableInstance nt) {
        // Configure the motors.
        TalonFXConfiguration configs = new TalonFXConfiguration()
                .withMotorOutput(
                        new MotorOutputConfigs()
                                .withNeutralMode(NeutralModeValue.Brake))
                .withCurrentLimits(
                        new CurrentLimitsConfigs()
                                .withSupplyCurrentLimit(Amps.of(17)))
                .withSlot0(
                        new Slot0Configs()
                                .withKP(1)
                                .withKS(0.03)
                                .withKV(0.12)
                                .withKG(0.15))
                .withMotionMagic(
                        new MotionMagicConfigs()
                                .withMotionMagicCruiseVelocity(
                                        RotationsPerSecond.of(50))
                                .withMotionMagicAcceleration(
                                        RotationsPerSecondPerSecond.of(40))
                                .withMotionMagicExpo_kV(Volts.per(RotationsPerSecond)
                                        .ofNative(0.12))
                                .withMotionMagicExpo_kA(Volts
                                        .per(RotationsPerSecondPerSecond)
                                        .ofNative(0.10)));
        topMotor.getConfigurator().apply(configs);
        bottomMotor.getConfigurator().apply(configs);

        bottomMotor.setControl(
                new Follower(TOP_ELEVATOR_CAN_ID, MotorAlignmentValue.Aligned));

        // 2. Initialize the WPILib elevator physics simulation plant
        elevatorSim = new ElevatorSim(
                DCMotor.getKrakenX60(2), // Modeling 2 motors total on the gearbox
                kGearReduction,
                kCarriageMass.in(Kilograms),
                kSprocketPitchRadius.in(Meters),
                kMinHeight.in(Meters),
                kMaxHeight.in(Meters),
                true, // Simulate gravity
                0.0 // Starting height
        );

        var elevatorTable = nt.getTable("Robot/Elevator");
        profiledSetPointMetersPublisher = elevatorTable.getDoubleTopic("profiledSetPointMeters").publish();
        currentPositionMetersPublisher = elevatorTable.getDoubleTopic("currentPositionMeters").publish();
        voltagePublisher = elevatorTable.getDoubleTopic("voltage").publish();
        isNewSetpointPublisher = elevatorTable.getBooleanTopic("isNewSetpoint").publish();
    }

    /**
     * Convert a linear travel distance in meters to the equivalent number of motor
     * rotations, taking into account the sprocket radius and gear reduction.
     * 
     * Can be applied to both position (m) and velocity (m/s) conversions, as the
     * units are consistent.
     * 
     * @param linearTravelMeters
     * @return The equivalent number of motor rotations for the given linear travel
     *         distance.
     */
    private static double linearTravelToMotorRotations(double linearTravelMeters) {
        return (linearTravelMeters / (2 * Math.PI * kSprocketPitchRadius.in(Meters))) * kGearReduction;
    }

    /**
     * Convert a number of motor rotations to the equivalent linear travel distance
     * in meters, taking into account the sprocket radius and gear reduction.
     * 
     * @param motorRotations
     * @return The equivalent linear travel distance in meters for the given number
     *         of motor rotations.
     */
    private static double motorRotationsToLinearTravelMeters(double motorRotations) {
        return (motorRotations / kGearReduction) * (2 * Math.PI * kSprocketPitchRadius.in(Meters));
    }

    @Override
    public void simulationPeriodic() {
        // Feed the simulated battery voltage into the motor controller simulation state
        topMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage()); //

        // Fetch the motor's requested output voltage from the code
        double motorVoltage = topMotorSim.getMotorVoltage(); //

        // Pass the voltage into the physical simulation environment
        elevatorSim.setInput(motorVoltage);

        // Step the simulation forward by 20ms (standard robot loop)
        elevatorSim.update(0.020);

        // Convert the linear height back to motor rotations to update internal sensor
        // data
        // Rotations = Linear distance / Circumference * Gear Reduction
        double linearPosition = elevatorSim.getPositionMeters();
        double linearVelocity = elevatorSim.getVelocityMetersPerSecond();

        // Force the simulated motor's sensor to read our simulated physical mechanism
        // values
        topMotorSim.setRawRotorPosition(linearTravelToMotorRotations(linearPosition));
        topMotorSim.setRotorVelocity(linearTravelToMotorRotations(linearVelocity));
    }

    // Call from Robot.autonomousInit() to initialize auto states.
    public void autonomousInit() {
        // Resets timer to 0 and starts running
        autoTimer.reset();
        autoTimer.start();

        // Reset the encoder position to 0 at the start of autonomous.
        // That assumes that the elevator is physically at the bottom of its travel at
        // the start of autonomous.
        topMotor.setPosition(0.0);
    }

    /**
     * Drives the elevator using open-loop duty cycle output (-1.0 to 1.0).
     * 
     * @param output Normalized duty cycle output
     */
    private void setDutyCycle(double output) {
        ControlModeValue controlMode = topMotor.getControlMode().getValue();
        if (controlMode == ControlModeValue.MotionMagicVoltageFOC && output == 0.0)
            return; // Don't override Motion Magic control with 0.0 output.

        output *= 0.10; // cap the output of full power to avoid overspeeding the elevator.
        if (output > 0.0) {
            topMotor.setControl(elevatorDutyCycleRequest.withOutput(output));
        } else {
            // Negative values are driving the motor down;
            // reduce the output to 80% to avoid overspeeding the elevator downwards.
            topMotor.setControl(elevatorDutyCycleRequest.withOutput(0.8 * output));
        }
    }

    /**
     * Sets the elevator desired position using Motion Magic control.
     * 
     * @param position The desired elevator position in meters.
     */
    private void setPositionSetpoint(Distance position) {
        topMotor.setControl(elevatorPositionRequest
                .withPosition(Rotations.of(linearTravelToMotorRotations(position.in(Meters)))));
    }

    /**
     * An autoroutine frame specifying the time at which the autoroutine gets a new
     * setpoint position.
     */
    private record AutoroutineSetpoint(double time, Distance position) {
    };

    /** The autoroutine sequence. */
    private final ArrayList<AutoroutineSetpoint> autoRoutine = new ArrayList<>(Arrays.asList(
            new AutoroutineSetpoint(0.0, Meters.of(0.0)), // Start at bottom
            new AutoroutineSetpoint(4.0, Meters.of(0.4)), // Move to top
            new AutoroutineSetpoint(6.0, Meters.of(0.2)), // Move to midway
            new AutoroutineSetpoint(8.0, Meters.of(0.0)), // Move to bottom
            new AutoroutineSetpoint(10.0, Meters.of(0.4)), // Move to top
            new AutoroutineSetpoint(12.0, Meters.of(0.0)) // Move to bottom
    ));

    public void autonomousProfiledPeriodic() {
        double elapsedTime = autoTimer.get();
        boolean isNewSetpoint = false;
        for (var setpoint : autoRoutine) {
            if (timerWatermark <= setpoint.time() && setpoint.time() < elapsedTime) {
                // Update the setpoint to the next one in the routine.
                setPositionSetpoint(setpoint.position());
                isNewSetpoint = true;
                break;
            }
        }
        isNewSetpointPublisher.set(isNewSetpoint);
        timerWatermark = elapsedTime;
    }

    @Override
    public void periodic() {
        // Refresh motor signals to get the latest data from the CAN bus
        topMotorSetpointRotationsSignal.refresh();
        topMotorPositionRotationsSignal.refresh();

        // Update the NetworkTables values for the current setpoint and position in
        // meters.
        profiledSetPointMetersPublisher
                .set(motorRotationsToLinearTravelMeters(topMotorSetpointRotationsSignal.getValueAsDouble()));
        currentPositionMetersPublisher
                .set(motorRotationsToLinearTravelMeters(topMotorPositionRotationsSignal.getValueAsDouble()));
        voltagePublisher.set(topMotor.getMotorVoltage().getValueAsDouble());
    }

    /**
     * Creates a command to run manual open-loop control from a supplier (e.g.
     * joystick axis).
     * Stops the motor when interrupted.
     *
     * @param speedSupplier Supplier providing values between -1.0 (up) and 1.0
     *                      (down)
     * @return The default command
     */
    public Command runManual(DoubleSupplier speedSupplier) {
        return run(() -> setDutyCycle(speedSupplier.getAsDouble()))
                .finallyDo(() -> setDutyCycle(0.0))
                .withName("ElevatorManualDutyCycle");
    }
}

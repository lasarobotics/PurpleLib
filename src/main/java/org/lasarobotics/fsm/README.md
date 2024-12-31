# State Machine template

First, make sure to import the following packages so that the State Machine works as expected:
* `import frc.robot.fsm.StateMachine;`
* `import frc.robot.fsm.SystemState;`

Simply extend the [StateMachine](src\main\java\org\lasarobotics\fsm\StateMachine.java) and implement [AutoCloseable] for your subsystem, and call the `super()` constructor with the appropriate arguments.

Use the [ExampleStateMachine](https://github.com/lasarobotics/PurpleLibExamples/tree/master/ExampleStateMachine) repo as an example

## Creating States

In order to create states for the respective subsystem, you MUST create an `enum` of States that implements the [SystemState](src\main\java\org\lasarobotics\fsm\SystemState.java) class.

To transition away from solely using the provided command-based infrastructure, each state should contain a `initialize()` and `nextState()` method. Within each of these methods, you can include the appropriate logic to establish states and transition between them.

 For example, here's how to create an IDLE state for a climber subsystem.
 ```
  public enum State implements SystemState {
    IDLE {
      @Override
      public void initialize() {
        s_instance.stop();
      }

      @Override
      public State nextState() {
        if (s_climbButton.getAsBoolean() && !s_retractButton.getAsBoolean()) return RELEASING;
        if (s_retractButton.getAsBoolean() && !s_climbButton.getAsBoolean()) return RETRACTING;
        return this;
      }
    }
  }
```

Additional code to provide more insight into variables used above:
```
  private static ClimberSubsystem s_instance;
  private static Trigger s_climbButton = new Trigger(() -> false);
  private static Trigger s_retractButton = new Trigger(() -> false);
```

In the above example, you can see how we have defined a State enum, which is being used to create our numerous states. In the IDLE state, we have defined our `initialize()` and `nextState()` methods. In the `initialize()` method, we are simply stoping the instance, since we are in the IDLE state of the climber. When the `nextState()` method is invoked, we have coded logic when determining what state to travel to next. It is this decision making process that allows us to replicate the State Machine structure in the real world. As team, we decided that it would be best to code all of our logic (conditonals, loops, etc.) in the `nextState()` method to improve readability and code accessibility for our team members. 

Specifically, from the above example, we can observe the creation of two new states: RELEASING an RETRACTING. Keep in mind that these are these are the only additional states needed for the CLIMBER subsystem. For a more complicated subsystem such as the DRIVE or SHOOTER subsytem, more states would need to be created, following the logic provided above.

## Integrating Motor Movement into States

 It is important to keep in mind that the methods to perform the movement of the motors are actually defined in the rest of the class.

 For example, for the CLIMBER subsytem example, we have defined our `runClimber()`, `retractClimber()`, and `stop()` methods.

```
  /**
   * Runs the climber during a match
   */
  private void runClimber() {
    m_lClimberMotor.set(CLIMBER_VELOCITY.in(Units.Percent), ControlType.kDutyCycle);
    m_rClimberMotor.set(CLIMBER_VELOCITY.in(Units.Percent), ControlType.kDutyCycle);
  }

  /**
   * Retracts the climber during a match
   */
  private void retractClimber() {
    m_lClimberMotor.set(-CLIMBER_VELOCITY.in(Units.Percent), ControlType.kDutyCycle);
    m_rClimberMotor.set(-CLIMBER_VELOCITY.in(Units.Percent), ControlType.kDutyCycle);
  }

  /**
   * Stop both motors
   */
  private void stop() {
    m_lClimberMotor.stopMotor();
    m_rClimberMotor.stopMotor();
  }
```

The methods defined above are then used in the creation of additional states. Below is the code for the RELEASING State:
```
  RELEASING {
    @Override
    public void initialize() {
      s_instance.runClimber();
    }

    @Override
    public State nextState() {
      if (!s_climbButton.getAsBoolean()) return IDLE;
      return this;
    }
  },
```
Instead of stopping the instance in the `initialize()` as done in the State example above, we are NOW calling the `runClimber()` method introduced above.

We also want to highlight how we have chosen to return to the IDLE state after each State, as our logic to transition to the next state is contained in the IDLE State seen above.

## Additional Changes with Our State Machine Structure

After adding this State Machine Structure, we have begun to bind our buttons to specific functions in the class itself, alongside the `RobotContainer` class. For example, we have binded buttons to retract and expand the climber:
```
    /**
   * Set climb button
   * @param climbButtonTrigger Button to use
   */
  public void bindClimbButton(Trigger climbButtonTrigger) {
    s_climbButton = climbButtonTrigger;
  }
```
Additionally, we are no longer using the `periodic()` function for all of our repetitive needs in the CLIMBER subsytem. It is NOW only being used for logging States and outputs in the CLIMBER subsystem, so that this process of transitioning between States can occur as needed. 
```
  @Override
  public void periodic() {
    Logger.recordOutput(getName() + "/State", getState().toString());
  }
```
## Closing Comments
Feel free to add more States and logic as needed in the `nextState()` methods. This State Machine structure has been created to provide additional autonomy and support for PurpleHaze and other FRC teams.

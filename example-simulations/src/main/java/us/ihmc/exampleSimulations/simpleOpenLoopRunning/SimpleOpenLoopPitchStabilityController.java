package us.ihmc.exampleSimulations.simpleOpenLoopRunning;

import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.State;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateMachine;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class SimpleOpenLoopPitchStabilityController implements RobotController
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getName());
   private final SimpleOpenLoopPitchStabilityRobot robot;
   private final StateMachine<SimplePitchStabilityState> stateMachine;
   
   private final YoDouble stanceDuration = new YoDouble("stanceDuration", registry);
   private final YoDouble flightDuration = new YoDouble("flightDuration", registry);
   private final YoDouble strideDuration = new YoDouble("strideDuration", registry);

   private final YoDouble zSetPoint = new YoDouble("zSetPoint", registry);
   private final YoDouble zSpring = new YoDouble("zSpring", registry);
   private final YoDouble zDamping = new YoDouble("zDamping", registry);

   private final YoDouble pitchGainK1 = new YoDouble("pitchGainK1", registry);
   private final YoDouble pitchGainK2 = new YoDouble("pitchGainK2", registry);

   private final YoDouble radiusOfGyration = new YoDouble("radiusOfGyration", registry);

   private final YoDouble wnPitch = new YoDouble("wnPitch", registry);
   private final YoDouble wnPitchHertz = new YoDouble("wnPitchHertz", registry);
   private final YoDouble zetaPitch = new YoDouble("zetaPitch", registry);

   private final YoDouble theta = new YoDouble("theta", registry);
   private final YoDouble thetaDot = new YoDouble("thetaDot", registry);
   private final YoDouble phiDot = new YoDouble("phiDot", registry);
   private final YoDouble discreteNondimensionalGainK = new YoDouble("discreteNondimensionalGainK", registry);
   private final YoDouble discreteNondimensionalGainB = new YoDouble("discreteNondimensionalGainB", registry);

   private final YoDouble discreteStabilityCriterionMustBeLessThanZero = new YoDouble("discreteStabilityCriterionMustBeLessThanZero", registry);
   private final YoDouble discreteNondimensionalGainBMax = new YoDouble("discreteNondimensionalGainBMax", registry);
   
   private final YoDouble nextExpectedTheta = new YoDouble("nextExpectedTheta", registry);
   private final YoDouble nextExpectedThetaDot = new YoDouble("nextExpectedThetaDot", registry);
   private final YoDouble nextExpectedPhiDot = new YoDouble("nextExpectedPhiDot", registry);

   private final YoDouble xOffsetForPitchStabilityDamping = new YoDouble("xOffsetForPitchStabilityDamping", registry);
   private final YoDouble xOffsetForPitchStabilitySpring = new YoDouble("xOffsetForPitchStabilitySpring", registry);
   
   private final YoDouble groundContactLength = new YoDouble("groundContactLength", registry);

   private enum SimplePitchStabilityState
   {
      STANCE, FLIGHT;
   }
   
   public SimpleOpenLoopPitchStabilityController(SimpleOpenLoopPitchStabilityRobot robot)
   {
      this.robot = robot;
      
      stateMachine = new StateMachine<>("state", "switchTime", SimplePitchStabilityState.class, robot.getYoTime(), registry);
      
      stanceDuration.set(0.1);
      flightDuration.set(0.2);
      
      zSetPoint.set(0.4);
      zSpring.set(0.4);
      zDamping.set(0.1);
      
      pitchGainK1.set(0.2);
      pitchGainK2.set(0.02);
      
      groundContactLength.set(0.0); //0.5);
      
      StanceState stanceState = new StanceState();
      FlightState flightState = new FlightState();
      
      stateMachine.addState(stanceState);
      stateMachine.addState(flightState);
      
      flightState.setDefaultNextState(SimplePitchStabilityState.STANCE);
      stanceState.setDefaultNextState(SimplePitchStabilityState.FLIGHT);

      stateMachine.setCurrentState(SimplePitchStabilityState.STANCE);
      
      this.doControl();
      stanceState.doTransitionIntoAction();
   }
   
   @Override
   public void initialize()
   {      
   }

   @Override
   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   @Override
   public String getName()
   {
      return getClass().getSimpleName();
   }

   @Override
   public String getDescription()
   {
      return getName();
   }

   @Override
   public void doControl()
   {
      double gravity = -robot.getGravityZ();
      strideDuration.set(stanceDuration.getDoubleValue() + flightDuration.getDoubleValue());
      radiusOfGyration.set(robot.getRadiusOfGyration());
      
      wnPitch.set(Math.sqrt(gravity * pitchGainK1.getDoubleValue() / (radiusOfGyration.getDoubleValue() * radiusOfGyration.getDoubleValue())));
      wnPitchHertz.set(wnPitch.getDoubleValue() / (2.0 * Math.PI));
      zetaPitch.set(0.5 * pitchGainK2.getDoubleValue() / radiusOfGyration.getDoubleValue() * Math.sqrt(gravity/pitchGainK1.getDoubleValue()));

      discreteNondimensionalGainK.set(gravity * strideDuration.getDoubleValue() * strideDuration.getDoubleValue() * pitchGainK1.getDoubleValue() / (radiusOfGyration.getDoubleValue() * radiusOfGyration.getDoubleValue()));
      discreteNondimensionalGainB.set(gravity * strideDuration.getDoubleValue() * pitchGainK2.getDoubleValue() / (radiusOfGyration.getDoubleValue() * radiusOfGyration.getDoubleValue()));
      
      double kPlusB = discreteNondimensionalGainK.getDoubleValue() + discreteNondimensionalGainB.getDoubleValue();
      discreteStabilityCriterionMustBeLessThanZero.set(kPlusB * kPlusB - 4.0 * discreteNondimensionalGainK.getDoubleValue());
      discreteNondimensionalGainBMax.set(2.0 * Math.sqrt(discreteNondimensionalGainK.getDoubleValue()) - discreteNondimensionalGainK.getDoubleValue());
      
      theta.set(robot.getPitch());
      thetaDot.set(robot.getPitchRate());
      phiDot.set(robot.getPitchRate() * strideDuration.getDoubleValue());
      
      stateMachine.doAction();
      stateMachine.checkTransitionConditions();
   }

   
   private class StanceState extends State<SimplePitchStabilityState>
   {
      public StanceState()
      {
         super(SimplePitchStabilityState.STANCE);
      }

      @Override
      public void doAction()
      {
         xOffsetForPitchStabilitySpring.set(pitchGainK1.getDoubleValue() * robot.getPitch());

         double xGroundPosition = groundContactLength.getDoubleValue() * (-0.5 + getTimeInCurrentState() / stanceDuration.getDoubleValue());
         robot.setGroundContactPositionXWithRespectToBody(xGroundPosition + xOffsetForPitchStabilityDamping.getDoubleValue() + xOffsetForPitchStabilitySpring.getDoubleValue());
         
         double groundForce = -robot.getMass() * robot.getGravityZ() * strideDuration.getDoubleValue() / stanceDuration.getDoubleValue();
         double springDamperForce = (zSpring.getDoubleValue() * (zSetPoint.getDoubleValue() - robot.getZ()) - zDamping.getDoubleValue() * robot.getZVelocity()) / stanceDuration.getDoubleValue();
         
         robot.setGroundContactForceZ(groundForce + springDamperForce);
         
         if (this.getTimeInCurrentState() > stanceDuration.getDoubleValue())
         {
            this.transitionToDefaultNextState();
         }
      }

      @Override
      public void doTransitionIntoAction()
      {         
         nextExpectedTheta.set((1.0 - discreteNondimensionalGainK.getDoubleValue()) * theta.getDoubleValue() + (1.0 - discreteNondimensionalGainB.getDoubleValue()) * phiDot.getDoubleValue());
         nextExpectedPhiDot.set((-discreteNondimensionalGainK.getDoubleValue()) * theta.getDoubleValue() + (1.0 - discreteNondimensionalGainB.getDoubleValue()) * phiDot.getDoubleValue());
         
         double gravity = -robot.getGravityZ();
         
         double kk = gravity * strideDuration.getDoubleValue() * pitchGainK1.getDoubleValue() / (robot.getRadiusOfGyration() * robot.getRadiusOfGyration());
         double bb = gravity * strideDuration.getDoubleValue() * pitchGainK2.getDoubleValue() / (robot.getRadiusOfGyration() * robot.getRadiusOfGyration());
         
         nextExpectedThetaDot.set(-kk * theta.getDoubleValue() + (1.0 - bb) * thetaDot.getDoubleValue());
         
         xOffsetForPitchStabilityDamping.set(pitchGainK2.getDoubleValue() * robot.getPitchRate());
      }

      @Override
      public void doTransitionOutOfAction()
      {         
      }
      
   }
   
   
   private class FlightState extends State<SimplePitchStabilityState>
   {
      public FlightState()
      {
         super(SimplePitchStabilityState.FLIGHT);
      }

      @Override
      public void doAction()
      {
         if (this.getTimeInCurrentState() > flightDuration.getDoubleValue())
         {
            this.transitionToDefaultNextState();
         }
      }

      @Override
      public void doTransitionIntoAction()
      {         
         robot.setGroundContactForceZ(0.0);
      }

      @Override
      public void doTransitionOutOfAction()
      {         
      }
      
   }

}

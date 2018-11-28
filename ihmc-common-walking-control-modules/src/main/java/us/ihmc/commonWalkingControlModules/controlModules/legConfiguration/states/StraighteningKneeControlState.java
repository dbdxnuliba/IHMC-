package us.ihmc.commonWalkingControlModules.controlModules.legConfiguration.states;

import us.ihmc.commonWalkingControlModules.controlModules.legConfiguration.LegConfigurationControlToolbox;
import us.ihmc.commonWalkingControlModules.controlModules.legConfiguration.LegControlWeight;
import us.ihmc.commonWalkingControlModules.controlModules.legConfiguration.gains.LegConfigurationGainsReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class StraighteningKneeControlState implements LegControlState
{
   private static final boolean ONLY_MOVE_PRIV_POS_IF_NOT_BENDING = true;

   private final YoDouble yoStraighteningAcceleration;

   private double startingPosition;
   private double previousKneePitchAngle;

   private double timeUntilStraight;
   private double straighteningVelocity;
   private double straighteningAcceleration;

   private double dwellTime;
   private double desiredPrivilegedPosition;

   private double previousTime;

   private final LegConfigurationControlToolbox toolbox;
   private final OneDoFJointBasics kneePitchJoint;

   private final DoubleProvider desiredStraightLegAngle;

   public StraighteningKneeControlState(String namePrefix, LegConfigurationControlToolbox toolbox, YoVariableRegistry registry)
   {
      yoStraighteningAcceleration = new YoDouble(namePrefix + "SupportKneeStraighteningAcceleration", registry);
      yoStraighteningAcceleration.set(toolbox.getParameters().getAccelerationForSupportKneeStraightening());
      this.toolbox = toolbox;
      this.kneePitchJoint = toolbox.getKneePitchJoint();
      this.desiredStraightLegAngle = toolbox.getDesiredStraightLegAngle();
   }

   @Override
   public boolean isDone(double timeInState)
   {
      return timeInState > (timeUntilStraight + dwellTime);
   }

   @Override
   public void doAction(double timeInState)
   {
      double estimatedDT = estimateDT(timeInState);
      double currentPosition = kneePitchJoint.getQ();

      if (ONLY_MOVE_PRIV_POS_IF_NOT_BENDING)
      {
         if (currentPosition > previousKneePitchAngle && currentPosition > startingPosition) // the knee is bending
         {
            dwellTime += estimatedDT;
         }
         else
         {
            straighteningVelocity += estimatedDT * straighteningAcceleration;
            desiredPrivilegedPosition -= estimatedDT * straighteningVelocity;
         }
      }
      else
      {
         straighteningVelocity += estimatedDT * straighteningAcceleration;
         desiredPrivilegedPosition -= estimatedDT * straighteningVelocity;
      }

      desiredPrivilegedPosition = Math.max(desiredStraightLegAngle.getValue(), desiredPrivilegedPosition);

      previousKneePitchAngle = currentPosition;
   }

   @Override
   public void onEntry()
   {
      startingPosition = kneePitchJoint.getQ();
      previousKneePitchAngle = kneePitchJoint.getQ();

      straighteningVelocity = 0.0;
      straighteningAcceleration = yoStraighteningAcceleration.getDoubleValue();

      timeUntilStraight = Math.sqrt(2.0 * (startingPosition - desiredStraightLegAngle.getValue()) / straighteningAcceleration);
      timeUntilStraight = Math.max(timeUntilStraight, 0.0);

      desiredPrivilegedPosition = startingPosition;

      previousTime = 0.0;
      dwellTime = 0.0;

      if (toolbox.useBracingAngle())
         toolbox.setLegControlWeight(LegControlWeight.MEDIUM);
   }

   @Override
   public void onExit()
   {
   }

   private double estimateDT(double timeInState)
   {
      double estimatedDT = timeInState - previousTime;
      previousTime = timeInState;

      return estimatedDT;
   }

   @Override
   public LegConfigurationGainsReadOnly getLegConfigurationGains()
   {
      return toolbox.getStraightLegGains();
   }

   @Override
   public double getKneePitchPrivilegedConfiguration()
   {
      return desiredPrivilegedPosition;
   }
}

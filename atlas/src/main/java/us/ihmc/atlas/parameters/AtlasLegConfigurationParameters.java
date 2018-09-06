package us.ihmc.atlas.parameters;

import us.ihmc.commonWalkingControlModules.configurations.LegConfigurationParameters;
import us.ihmc.commonWalkingControlModules.controlModules.legConfiguration.LegConfigurationGains;

public class AtlasLegConfigurationParameters extends LegConfigurationParameters
{
   private final boolean runningOnRealRobot;

   public AtlasLegConfigurationParameters(boolean runningOnRealRobot)
   {
      this.runningOnRealRobot = runningOnRealRobot;
   }

   @Override
   /** {@inheritDoc} */
   public boolean attemptToStraightenLegs()
   {
      return false;
   }

   @Override
   /** {@inheritDoc} */
   public LegConfigurationGains getBentLegGains()
   {
      LegConfigurationGains gains = new LegConfigurationGains();
      gains.setJointSpaceKp(runningOnRealRobot ? 40.0 : 100.0);
      gains.setJointSpaceKd(6.0);

      return gains;
   }

   /** {@inheritDoc} */
   @Override
   public LegConfigurationGains getStraightLegGains()
   {
      LegConfigurationGains gains = new LegConfigurationGains();
      gains.setActuatorSpaceKp(750.0);
      gains.setJointSpaceKd(runningOnRealRobot ? 3.0 : 10.0);

      return gains;
   }

   /** {@inheritDoc} */
   @Override
   public double getLegPrivilegedHighWeight()
   {
      return runningOnRealRobot ? 50.0 : 5.0;
   }

   @Override
   public double getDesiredFractionOfMidrangeForCollapsedAngle()
   {
      return 0.3;
      //      return 1.8; // for big step down
   }

   /** {@inheritDoc} */
   @Override
   public double getFractionOfSwingToStraightenLeg()
   {
      return runningOnRealRobot ? 0.6 : 0.4;
   }

   @Override
   public double getFractionOfTransferToCollapseLeg()
   {
      return 0.7;
      //      return 1.0;
   }

   @Override
   public double getFractionOfSwingToCollapseStanceLeg()
   {
      return 0.55;
      //      return 1.0;
      //      return 0.3; //for big step down
   }


   /** {@inheritDoc} */
   @Override
   public double getKneeAngleWhenExtended()
   {
      return runningOnRealRobot ? 0.2 : 0.9;
   }

   /** {@inheritDoc} */
   @Override
   public double getKneeAngleWhenStraight()
   {
      return runningOnRealRobot ? 0.35 : 1.17819 ;
   } //1.17
}

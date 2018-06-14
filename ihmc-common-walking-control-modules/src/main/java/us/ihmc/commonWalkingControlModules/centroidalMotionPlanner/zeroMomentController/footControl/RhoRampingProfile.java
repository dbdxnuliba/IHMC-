package us.ihmc.commonWalkingControlModules.centroidalMotionPlanner.zeroMomentController.footControl;

import us.ihmc.commons.MathTools;
import us.ihmc.robotics.math.trajectories.Trajectory;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class RhoRampingProfile
{
   private static final int numberOfTrajectoryCoefficients = 2;
   private final Trajectory rhoTrajectory = new Trajectory(numberOfTrajectoryCoefficients);
   private final YoDouble t0;
   private final YoDouble rho0;
   private final YoDouble tF;
   private final YoDouble rhoF;

   public RhoRampingProfile(String namePrefix, YoVariableRegistry registry)
   {
      namePrefix += "RhoRamp";
      t0 = new YoDouble(namePrefix + "InitialTime", registry);
      rho0 = new YoDouble(namePrefix + "InitialWeight", registry);
      tF = new YoDouble(namePrefix + "FinalTime", registry);
      rhoF = new YoDouble(namePrefix + "FinalWeight", registry);
   }

   public double compute(double time)
   {
      double clippedTime = MathTools.clamp(time, t0.getDoubleValue(), tF.getDoubleValue());
      rhoTrajectory.compute(clippedTime);
      return rhoTrajectory.getPosition();
   }

   public void createRhoRamp(double t0, double tF, double rhoInitial, double rhoFinal)
   {
      this.t0.set(t0);
      this.tF.set(tF);
      this.rho0.set(rhoInitial);
      this.rhoF.set(rhoFinal);
      rhoTrajectory.setLinear(t0, tF, rhoInitial, rhoFinal);
   }
}

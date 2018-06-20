package us.ihmc.commonWalkingControlModules.centroidalMotionPlanner.decoupledMotionGeneration;

import java.util.function.Supplier;

import us.ihmc.robotics.math.trajectories.Trajectory;

public class CentroidalMotionPlannerTools
{
   // Hidden constructor to prevent creation of objects
   private CentroidalMotionPlannerTools()
   {
      
   }
   
   public static Supplier<Trajectory> getTrajectoryBuilder(int maxNumberOfCoefficients)
   {
      return new Supplier<Trajectory>()
      {
         @Override
         public Trajectory get()
         {
            return new Trajectory(maxNumberOfCoefficients);
         }
      };
   }
}

package us.ihmc.avatar.networkProcessor.rrtToolboxModule;

import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.HumanoidKinematicsSolver;
import us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace.SpatialNode;

public class InitialGuessManager
{
   private final SpatialNode initialGuess;
      
   private final HumanoidKinematicsSolver kinematicsSolver;   
   
   public InitialGuessManager(HumanoidKinematicsSolver kinematicsSolver)
   {
      this.initialGuess = 
      this.kinematicsSolver = kinematicsSolver;
   }
   
   public void updateManager(SpatialNode guess)
   {
      
   }
}

package us.ihmc.avatar.networkProcessor.rrtToolboxModule;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace.ExploringDefinition;
import us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace.SpatialData;
import us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace.SpatialNode;

public class ExpandingManager extends WholeBodyTrajectoryToolboxManager
{
   // Temp : for efficient debug.
   private final Random random = new Random(1);

   private List<SpatialNode> validNodes = new ArrayList<SpatialNode>();
   private List<SpatialNode> inValidNodes = new ArrayList<SpatialNode>();

   private SpatialNode recentlyAddedNode = null;

   private final static int maximumCountForWating = 100;

   public ExpandingManager(ExploringDefinition exploringDefinition, int maximumNumberOfUpdate)
   {
      super(exploringDefinition, maximumNumberOfUpdate);
   }

   public void getInitialGuesses(List<SpatialNode> initialGuesses)
   {
      validNodes.addAll(initialGuesses);
   }

   @Override
   public void initialize()
   {
      super.initialize();
      validNodes = new ArrayList<SpatialNode>();
      inValidNodes = new ArrayList<SpatialNode>();
      recentlyAddedNode = null;
   }

   @Override
   public SpatialNode createDesiredNode()
   {
      // create random node.
      SpatialData randomSpatialData = exploringDefinition.getRandomSpatialData();

//      double nextDouble = random.nextDouble();
//      double randomTime = nextDouble * (1.0 + WholeBodyTrajectoryToolboxSettings.timeCoefficient * tree.getMostAdvancedTime());

      SpatialNode randomNode = new SpatialNode(randomSpatialData);
      // find closest one.
      // if no, re-try.

      return randomNode;
   }

   @Override
   public boolean isDone()
   {
      return isExceedMaximumNumberOfUpdate() || getProgress() == 1.0;
   }

   @Override
   public void putDesiredNode(SpatialNode desiredNode)
   {
      if (desiredNode.isValid())
      {
         validNodes.add(desiredNode);
         recentlyAddedNode = desiredNode;
      }
      else
      {
         inValidNodes.add(desiredNode);
      }
   }

   @Override
   public boolean hasFail()
   {
      if (getProgress() == 1.0)
         return false;
      else
         return true;
   }

   private double getProgress()
   {
      return exploringDefinition.getExploringProgress(recentlyAddedNode);
   }
}

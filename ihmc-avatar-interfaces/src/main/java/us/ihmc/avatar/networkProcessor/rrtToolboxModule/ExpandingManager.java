package us.ihmc.avatar.networkProcessor.rrtToolboxModule;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commons.PrintTools;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.ConfigurationSpaceName;
import us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace.ExploringDefinition;
import us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace.SpatialData;
import us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace.SpatialNode;

public class ExpandingManager extends WholeBodyTrajectoryToolboxManager
{
   private List<SpatialNode> validNodes = new ArrayList<SpatialNode>();
   private List<SpatialNode> inValidNodes = new ArrayList<SpatialNode>();

   private double advancedProgress = 0.0;

   private final static int maximumCountForWating = 100;
   private final static double timeCoefficient = 2.5;

   private final double timeWeight = 0.5;
   private final double positionWeight = 1.0;
   private final double orientationWeight = 1.0;

   private final double maxTimeInterval = 1 / 2.0;
   private final double maxPositionDistance = 0.05;
   private final double maxOrientationDistance = Math.toRadians(10.0);

   public ExpandingManager(ExploringDefinition exploringDefinition, int maximumNumberOfUpdate)
   {
      super(exploringDefinition, maximumNumberOfUpdate);
   }

   public void setInitialGuesses(List<SpatialNode> initialGuesses)
   {
      validNodes.addAll(initialGuesses);
   }

   public List<SpatialNode> getPath()
   {
      List<SpatialNode> path = new ArrayList<>();
      path.clear();

      List<SpatialNode> revertedPath = new ArrayList<SpatialNode>();
      SpatialNode currentNode = validNodes.get(validNodes.size() - 1);
      revertedPath.add(currentNode);

      while (true)
      {
         SpatialNode parentNode = currentNode.getParent();
         if (parentNode != null)
         {
            revertedPath.add(parentNode);
            currentNode = parentNode;
         }
         else
            break;
      }

      int revertedPathSize = revertedPath.size();

      for (int i = 0; i < revertedPathSize; i++)
      {
         path.add(revertedPath.get(revertedPathSize - 1 - i));
      }

      // TODO : filtering out too short time gap between path nodes.
      // TODO : set every parent nodes.
//    path.add(new SpatialNode(revertedPath.get(revertedPathSize - 1)));
//
//    // filtering out too short time gap node.
//    // test ahead adding and attach only valid filtering.
//    // TODO : but still has problem. To test ahead whole shortcut would be too heavy.      
//    int currentIndex = 0;
//    int latestAddedIndexOfRevertedPath = revertedPathSize - 1;
//    for (int j = revertedPathSize - 2; j > 0; j--)
//    {
//       double timeGapWithLatestNode = revertedPath.get(j).getTime() - path.get(currentIndex).getTime();
//
//       if (timeGapWithLatestNode > minTimeInterval)
//       {
//          SpatialNode dummyNode = new SpatialNode(revertedPath.get(j));
//          dummyNode.setParent(path.get(currentIndex));
//          updateValidity(dummyNode);
//          if (dummyNode.isValid())
//          {
//             path.add(new SpatialNode(revertedPath.get(j)));
//             latestAddedIndexOfRevertedPath = j;
//             currentIndex++;
//          }
//          else
//          {
//             j = latestAddedIndexOfRevertedPath - 1;
//
//             path.add(new SpatialNode(revertedPath.get(j)));
//             latestAddedIndexOfRevertedPath = j;
//
//             currentIndex++;
//          }
//       }
//    }
//    path.add(new SpatialNode(revertedPath.get(0)));
//
//    // set every parent nodes.
//    for (int i = 0; i < path.size() - 1; i++)
//       path.get(i + 1).setParent(path.get(i));


      return path;
   }

   @Override
   public void initialize()
   {
      super.initialize();
      validNodes = new ArrayList<SpatialNode>();
      inValidNodes = new ArrayList<SpatialNode>();
   }

   @Override
   public SpatialNode createDesiredNode()
   {
      // create random node.
      SpatialData randomSpatialData = exploringDefinition.getRandomSpatialData();

      double nextDouble = ConfigurationSpaceName.random.nextDouble();
      double randomTime = exploringDefinition.getTrajectoryTime() * nextDouble * (1.0 + timeCoefficient * advancedProgress);

      SpatialNode randomNode = new SpatialNode(randomTime, randomSpatialData);

      // find closest one.
      SpatialNode nearestNode = null;
      for (int i = 0; i < maximumCountForWating; i++)
      {
         nearestNode = findNearestValidNodeToRandomNode(randomNode);
         if (nearestNode != null)
         {
            break;
         }
      }
      if (nearestNode == null)
      {
         System.out.println("could not find nearest node");
         return null;
      }

      // create desired node.
      // TODO : clamping for reaching manifold.
      double timeGap = nearestNode.getTimeGap(randomNode);

      double alpha = 1.0;

      double timeStepToward;

      timeStepToward = Math.min(exploringDefinition.getTrajectoryTime() - nearestNode.getTime(), maxTimeInterval);

      alpha = Math.min(alpha, timeStepToward / timeGap);

      SpatialNode desiredNode = new SpatialNode(randomNode);
      desiredNode.interpolate(nearestNode, randomNode, alpha);
      desiredNode.setParent(nearestNode);

      return desiredNode;
   }

   @Override
   public boolean isDone()
   {
      return isExceedMaximumNumberOfUpdate() || advancedProgress >= 1.0;
   }

   @Override
   public void putDesiredNode(SpatialNode desiredNode)
   {
      if (desiredNode.isValid())
      {
         validNodes.add(desiredNode);
         advancedProgress = Math.max(advancedProgress, exploringDefinition.getExploringProgress(desiredNode));
         
         // TODO : implement for reaching.
         //               else if (manifoldCommands != null)
         //               {
         //                  Pose3D testFrame = toolboxData.getTestFrame(tree.getLastNodeAdded());
         //
         //                  testFramePose.setPosition(testFrame.getPosition());
         //                  testFramePose.setOrientation(testFrame.getOrientation());
         //                  testFrameViz.setVisible(true);
         //                  testFrameViz.update();
         //
         //                  // TODO : terminal condition for manifold command.
         //                  double maximumDistanceFromManifolds = toolboxData.getMaximumDistanceFromManifolds(tree.getLastNodeAdded());
         //                  minimumDistanceFromManifold.set(maximumDistanceFromManifolds);
         //                  if (maximumDistanceFromManifolds < 0.05)
         //                     isExpandingTerminalCondition = true;
         //               }
      }
      else
      {
         inValidNodes.add(desiredNode);
      }
   }

   @Override
   public boolean hasFail()
   {
      if (advancedProgress == 1.0)
         return false;
      else
         return true;
   }

   private SpatialNode findNearestValidNodeToRandomNode(SpatialNode randomNode)
   {
      double distanceToNearestNode = Double.MAX_VALUE;
      SpatialNode nearestNode = null;

      for (int i = 0; i < validNodes.size(); i++)
      {
         if (randomNode.getTime() < validNodes.get(i).getTime())
            continue;

         double distance;

         if (exploringDefinition.getTrajectoryTime() == 0.0)
         {
            distance = validNodes.get(i).computeDistanceWithinMaxDistance(0.0, positionWeight, orientationWeight, randomNode, maxTimeInterval,
                                                                          maxPositionDistance, maxOrientationDistance);
         }
         else
         {
            distance = validNodes.get(i).computeDistanceWithinMaxDistance(timeWeight, positionWeight, orientationWeight, randomNode, maxTimeInterval,
                                                                          maxPositionDistance, maxOrientationDistance);
         }

         if (distance < distanceToNearestNode)
         {
            distanceToNearestNode = distance;
            nearestNode = validNodes.get(i);
         }
      }

      return nearestNode;
   }
}

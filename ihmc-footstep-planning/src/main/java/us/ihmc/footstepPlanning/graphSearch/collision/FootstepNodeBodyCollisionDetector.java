package us.ihmc.footstepPlanning.graphSearch.collision;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNodeTools;
import us.ihmc.footstepPlanning.graphSearch.graph.LatticeNode;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.robotics.geometry.PlanarRegionsList;

import java.util.HashMap;

public class FootstepNodeBodyCollisionDetector
{
   private final BoundingBoxCollisionDetector collisionDetector;
   private final FootstepPlannerParameters parameters;
   private final HashMap<LatticeNode, BodyCollisionData> collisionDataHolder = new HashMap<>();
   private final RigidBodyTransform transform = new RigidBodyTransform();

   public FootstepNodeBodyCollisionDetector(FootstepPlannerParameters parameters)
   {
      this.collisionDetector = new BoundingBoxCollisionDetector(parameters);
      this.parameters = parameters;
   }

   public void setPlanarRegionsList(PlanarRegionsList planarRegionsList)
   {
      if(planarRegionsList != null)
         collisionDetector.setPlanarRegionsList(planarRegionsList);
      collisionDataHolder.clear();
   }

   public BodyCollisionData checkForCollision(FootstepNode footstepNode, double snappedNodeHeight)
   {
      FootstepNodeTools.getNodeTransform(footstepNode, transform);
      double tx = parameters.getBodyBoxBaseX();
      double ty = footstepNode.getRobotSide().negateIfLeftSide(parameters.getBodyBoxBaseY());
      transform.appendTranslation(tx, ty, snappedNodeHeight);
      LatticeNode latticeNode = new LatticeNode(transform.getTranslationX(), transform.getTranslationY(), footstepNode.getYaw());

      if (collisionDataHolder.containsKey(latticeNode))
      {
         return collisionDataHolder.get(latticeNode);
      }
      else
      {
         collisionDetector.setBoxPose(transform.getTranslationX(), transform.getTranslationY(), transform.getTranslationZ() + parameters.getBodyBoxBaseZ(), footstepNode.getYaw());
         BodyCollisionData collisionData = collisionDetector.checkForCollision();
         collisionDataHolder.put(latticeNode, collisionData);
         return collisionData;
      }
   }
}

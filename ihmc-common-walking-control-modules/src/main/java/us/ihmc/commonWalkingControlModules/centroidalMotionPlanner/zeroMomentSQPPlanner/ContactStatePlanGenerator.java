package us.ihmc.commonWalkingControlModules.centroidalMotionPlanner.zeroMomentSQPPlanner;

import java.util.ArrayList;

import us.ihmc.commonWalkingControlModules.controlModules.flight.ContactState;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose2DReadOnly;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

/**
 * A helper class to help with generation of contact state plans 
 * for the {@code CollinearForceBasedCoMMotionPlanner}. Methods are provided to generate 
 * contact state plans given some basic details of the footstep plans as well as 
 * convert more complex footstep plans and generate the corresponding contact state plans
 * Note : Can only handle flat ground cases 
 * @author Apoorv S
 * 
 */
public class ContactStatePlanGenerator
{
   private FrameConvexPolygon2d tempPolygon = new FrameConvexPolygon2d();
   private ArrayList<FramePoint2D> tempVertexList = new ArrayList<>();

   public ContactStatePlanGenerator(int maxNumberOfSupportPolygonVertices)
   {
      for (int i = 0; i < maxNumberOfSupportPolygonVertices; i++)
         tempVertexList.add(new FramePoint2D());
   }
   
   /**
    * Populates the contact states based on a generated footstep plan. A footstep containing {@code Double.NaN} or {@code null} is considered 
    * as undefined and is not included in the contact state
    * @param contactStateListToPopulate list of contact states that will be set from the footstep plan. Should be populated with elements
    * @param footstepLocations list of footsteps that will make up the contact state list
    * @param footSupportPolygonInAnkleFrame  default shape of the support polygon. Assumed to be the same for both feet
    */
   public void computeAndSetSupportPolygon(ContactState contactStateToPopulate, ReferenceFrame supportPolygonReferenceFrame, 
                                        SideDependentList<? extends FramePose2DReadOnly> anklePoses,
                                        SideDependentList<ConvexPolygon2D> footSupportPolygonInAnkleFrame)
   {
      computeAndSetSupportPolygon(contactStateToPopulate, supportPolygonReferenceFrame, anklePoses.get(RobotSide.LEFT), anklePoses.get(RobotSide.RIGHT),
                               footSupportPolygonInAnkleFrame.get(RobotSide.LEFT), footSupportPolygonInAnkleFrame.get(RobotSide.RIGHT));
   }

   public void computeAndSetSupportPolygon(ContactState contactStateToPopulate, ReferenceFrame desiredSupportPolygonReferenceFrame,
                                        FramePose2DReadOnly leftAnklePose, FramePose2DReadOnly rightAnklePose, ConvexPolygon2D leftFootSupportPolygon,
                                        ConvexPolygon2D rightFootSupportPolygon)
   {
      boolean isLeftFootSupported = leftAnklePose != null && !leftAnklePose.containsNaN() && leftFootSupportPolygon.getNumberOfVertices() > 0;
      boolean isRightFootSupported = rightAnklePose != null && !rightAnklePose.containsNaN() && rightFootSupportPolygon.getNumberOfVertices() > 0;
      int numberOfVertices = 0;
      if (isRightFootSupported)
      {
         addVerticesToListFromPoseAndPolygon(desiredSupportPolygonReferenceFrame, rightAnklePose, rightFootSupportPolygon, numberOfVertices);
         numberOfVertices += rightFootSupportPolygon.getNumberOfVertices();
      }
      if (isLeftFootSupported)
      {
         addVerticesToListFromPoseAndPolygon(desiredSupportPolygonReferenceFrame, leftAnklePose, leftFootSupportPolygon, numberOfVertices);
         numberOfVertices += leftFootSupportPolygon.getNumberOfVertices();
      }

      tempPolygon.clear();
      generateMinimalVertexSupportPolygon(tempPolygon, tempVertexList, numberOfVertices);
      contactStateToPopulate.setPoseToZero(desiredSupportPolygonReferenceFrame);
      contactStateToPopulate.setSupportPolygon(tempPolygon.getGeometryObject());
   }

   public void generateMinimalVertexSupportPolygon(FrameConvexPolygon2d polygonToSet, ArrayList<FramePoint2D> vertexList, int numberOfVertices)
   {
      if (numberOfVertices == 0)
      {
         polygonToSet.clear();
         polygonToSet.update();
         return;
      }
      // Generate the minimal vertex polygon. New gift wrapping algorithm
      // Get the max X max Y element. 
      int candidateVertexIndex = 0;
      for (int i = 1; i < numberOfVertices; i++)
      {
         if (vertexList.get(i).getX() > vertexList.get(candidateVertexIndex).getX())
            candidateVertexIndex = i;
         else if (vertexList.get(i).getX() == vertexList.get(candidateVertexIndex).getX()
               && vertexList.get(i).getY() > vertexList.get(candidateVertexIndex).getY())
            candidateVertexIndex = i;
      }
      // Place the top right vertex at the beginning of list
      FramePoint2D topRightVertex = vertexList.get(candidateVertexIndex);
      FramePoint2D firstVertex = vertexList.get(0);
      vertexList.set(0, topRightVertex);
      vertexList.set(candidateVertexIndex, firstVertex);
      // Start the marching
      for (int i = 1; i < numberOfVertices; i++)
      {
         FramePoint2D lastComputedPoint = vertexList.get(i - 1);
         FramePoint2D candidatePoint = vertexList.get(i);
         // Find the next one 
         for (int j = i + 1; j < numberOfVertices; j++)
         {
            FramePoint2D pointUnderConsideration = vertexList.get(j);
            double det = (pointUnderConsideration.getY() - lastComputedPoint.getY()) * (candidatePoint.getX() - lastComputedPoint.getX())
                  - (pointUnderConsideration.getX() - lastComputedPoint.getX()) * (candidatePoint.getY() - lastComputedPoint.getY());
            boolean swap = det > 0.0 || (det == 0.0 && lastComputedPoint.distance(pointUnderConsideration) > lastComputedPoint.distance(candidatePoint));
            if (swap)
            {
               vertexList.set(j, candidatePoint);
               vertexList.set(i, pointUnderConsideration);
               candidatePoint = pointUnderConsideration;
            }
         }
         double det2 = (topRightVertex.getY() - lastComputedPoint.getY()) * (candidatePoint.getX() - lastComputedPoint.getX())
               - (topRightVertex.getX() - lastComputedPoint.getX()) * (candidatePoint.getY() - lastComputedPoint.getY());
         boolean terminate = det2 > 0.0 || (det2 == 0.0 && lastComputedPoint.distance(candidatePoint) < lastComputedPoint.distance(topRightVertex));
         if (terminate)
         {
            numberOfVertices = i;
            break;
         }
      }
      polygonToSet.setAndUpdate(vertexList, numberOfVertices);
   }

   private void addVerticesToListFromPoseAndPolygon(ReferenceFrame desiredReferenceFrame, FramePose2DReadOnly anklePose,
                                                    ConvexPolygon2D footSupportPolygonInAnkleFrame, int firstIndex)
   {
      for (int i = 0; i < footSupportPolygonInAnkleFrame.getNumberOfVertices(); i++)
      {
         FramePoint2D vertexToSet = tempVertexList.get(i + firstIndex);
         vertexToSet.setIncludingFrame(anklePose.getPosition());
         vertexToSet.add(footSupportPolygonInAnkleFrame.getVertex(i));
         vertexToSet.changeFrameAndProjectToXYPlane(desiredReferenceFrame);
      }
   }
}

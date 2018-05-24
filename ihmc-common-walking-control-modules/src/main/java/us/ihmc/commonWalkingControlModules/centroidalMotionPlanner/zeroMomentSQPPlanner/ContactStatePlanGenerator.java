package us.ihmc.commonWalkingControlModules.centroidalMotionPlanner.zeroMomentSQPPlanner;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commonWalkingControlModules.controlModules.flight.ContactState;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePose2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose2DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
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
   private FramePose2D tempPose = new FramePose2D();
   private FrameConvexPolygon2d tempPolygon = new FrameConvexPolygon2d();
   private ArrayList<FramePoint2D> tempVertexList = new ArrayList<>();
   private RigidBodyTransform tempTransform = new RigidBodyTransform();
   private double defaultPrecision = 1e-4;

   public ContactStatePlanGenerator(int maxNumberOfSupportPolygonVertices, double defaultPrecision)
   {
      for (int i = 0; i < maxNumberOfSupportPolygonVertices; i++)
         tempVertexList.add(new FramePoint2D());
      this.defaultPrecision = defaultPrecision;
   }

   public ContactStatePlanGenerator(int maxNumberOfSupportPolygonVertices)
   {
      for (int i = 0; i < maxNumberOfSupportPolygonVertices; i++)
         tempVertexList.add(new FramePoint2D());
   }

   public void computeAndSetSupportPolygon(ContactState contactStateToPopulate, ReferenceFrame supportPolygonReferenceFrame,
                                           SideDependentList<? extends FramePose2DReadOnly> anklePoses,
                                           SideDependentList<ConvexPolygon2D> footSupportPolygonsInAnkleFrame)
   {
      tempPose.setToZero(supportPolygonReferenceFrame);
      computeAndSetSupportPolygon(contactStateToPopulate, tempPose, anklePoses.get(RobotSide.LEFT), anklePoses.get(RobotSide.RIGHT),
                                  footSupportPolygonsInAnkleFrame.get(RobotSide.LEFT), footSupportPolygonsInAnkleFrame.get(RobotSide.RIGHT));
   }

   public void computeAndSetSupportPolygon(ContactState contactStateToPopulate, ReferenceFrame supportPolygonReferenceFrame, FramePose2DReadOnly leftAnklePose,
                                           FramePose2DReadOnly rightAnklePose, ConvexPolygon2D leftFootSupportPolygon, ConvexPolygon2D rightFootSupportPolygon)
   {
      tempPose.setToZero(supportPolygonReferenceFrame);
      computeAndSetSupportPolygon(contactStateToPopulate, tempPose, leftAnklePose, rightAnklePose, leftFootSupportPolygon, rightFootSupportPolygon);
   }

   public void computeAndSetSupportPolygon(ContactState contactStateToPopulate, FramePose2DReadOnly poseToGenerateSupportPolygon,
                                           FramePose2DReadOnly leftAnklePose, FramePose2DReadOnly rightAnklePose, ConvexPolygon2D leftFootSupportPolygon,
                                           ConvexPolygon2D rightFootSupportPolygon)
   {
      computeAndSetSupportPolygon(contactStateToPopulate, poseToGenerateSupportPolygon, leftAnklePose, rightAnklePose, leftFootSupportPolygon,
                                  rightFootSupportPolygon, defaultPrecision);
   }

   public void computeAndSetSupportPolygons(List<ContactState> contactStateList, ReferenceFrame supportPolygonReferenceFrame,
                                            List<SideDependentList<? extends FramePose2DReadOnly>> anklePoses,
                                            SideDependentList<ConvexPolygon2D> footSupportPolygonsInAnkleFrame)
   {
      if (contactStateList.size() < anklePoses.size())
         throw new IllegalArgumentException("Contact state list does not have enough elements to store all processed support polygons");
      for (int i = 0; i < anklePoses.size(); i++)
         computeAndSetSupportPolygon(contactStateList.get(i), supportPolygonReferenceFrame, anklePoses.get(i), footSupportPolygonsInAnkleFrame);
   }

   public void computeAndSetSupportPolygon(ContactState contactStateToPopulate, FramePose2DReadOnly poseToGenerateSupportPolygon,
                                           FramePose2DReadOnly leftAnklePose, FramePose2DReadOnly rightAnklePose, ConvexPolygon2D leftFootSupportPolygon,
                                           ConvexPolygon2D rightFootSupportPolygon, double precision)
   {
      boolean isLeftFootSupported = leftAnklePose != null && !leftAnklePose.containsNaN() && leftFootSupportPolygon.getNumberOfVertices() > 0;
      boolean isRightFootSupported = rightAnklePose != null && !rightAnklePose.containsNaN() && rightFootSupportPolygon.getNumberOfVertices() > 0;
      int numberOfVertices = 0;
      if (isRightFootSupported && isLeftFootSupported)
      {
         addVerticesInCentroidalPoseFrameToListFromAnklePoseAndPolygon(poseToGenerateSupportPolygon, rightAnklePose, rightFootSupportPolygon, numberOfVertices);
         numberOfVertices += rightFootSupportPolygon.getNumberOfVertices();
         addVerticesInCentroidalPoseFrameToListFromAnklePoseAndPolygon(poseToGenerateSupportPolygon, leftAnklePose, leftFootSupportPolygon, numberOfVertices);
         numberOfVertices += leftFootSupportPolygon.getNumberOfVertices();
         roundVertexCoordinatesToPrecision(precision, numberOfVertices);
         generateMinimalVertexSupportPolygon(tempPolygon, tempVertexList, numberOfVertices);
      }
      else if (isLeftFootSupported)
         contactStateToPopulate.setSupportPolygon(leftFootSupportPolygon);
      else if (isRightFootSupported)
         contactStateToPopulate.setSupportPolygon(rightFootSupportPolygon);
      else
      {
         tempPolygon.clearAndUpdate(poseToGenerateSupportPolygon.getReferenceFrame());
         contactStateToPopulate.setSupportPolygon(tempPolygon.getGeometryObject());
      }
      contactStateToPopulate.setPose(poseToGenerateSupportPolygon);
      contactStateToPopulate.setSupportPolygon(tempPolygon.getGeometryObject());
   }

   private void roundVertexCoordinatesToPrecision(double precision, int numberOfVertices)
   {
      for (int i = 0; i < numberOfVertices; i++)
      {
         FramePoint2D vertexToRound = tempVertexList.get(i);
         double newX = MathTools.roundToPrecision(vertexToRound.getX(), precision);
         double newY = MathTools.roundToPrecision(vertexToRound.getY(), precision);
         vertexToRound.set(newX, newY);
      }
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

   private void addVerticesInCentroidalPoseFrameToListFromAnklePoseAndPolygon(FramePose2DReadOnly desiredPose, FramePose2DReadOnly anklePose,
                                                                              ConvexPolygon2D footSupportPolygonInAnkleFrame, int firstIndex)
   {
      for (int i = 0; i < footSupportPolygonInAnkleFrame.getNumberOfVertices(); i++)
      {
         FramePoint2D vertexToSet = tempVertexList.get(i + firstIndex);
         vertexToSet.setIncludingFrame(anklePose.getPosition());
         vertexToSet.add(footSupportPolygonInAnkleFrame.getVertex(i));
         vertexToSet.changeFrame(desiredPose.getReferenceFrame());
         desiredPose.get(tempTransform);
         vertexToSet.applyInverseTransform(tempTransform);
      }
   }
}

package us.ihmc.commonWalkingControlModules.centroidalMotionPlanner.zeroMomentSQPPlanner;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commonWalkingControlModules.controlModules.flight.ContactState;
import us.ihmc.commonWalkingControlModules.controlModules.flight.TransformHelperTools;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.Pose2DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePose2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose2DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
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
   private double defaultPrecision = 1e-4;

   private FramePose2D tempPoseForLeftFoot = new FramePose2D();
   private FramePose2D tempPoseForRightFoot = new FramePose2D();
   private FramePose2D tempPose = new FramePose2D();
   private ConvexPolygon2D tempPolygon = new ConvexPolygon2D();
   private ArrayList<Point2D> tempVertexList = new ArrayList<>();
   private RigidBodyTransform tempTransform = new RigidBodyTransform();

   public ContactStatePlanGenerator(int maxNumberOfSupportPolygonVertices, double defaultPrecision)
   {
      this(maxNumberOfSupportPolygonVertices);
      this.defaultPrecision = defaultPrecision;
   }

   public ContactStatePlanGenerator(int maxNumberOfSupportPolygonVertices)
   {
      for (int i = 0; i < maxNumberOfSupportPolygonVertices; i++)
         tempVertexList.add(new Point2D());
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

   public void computeAndSetSupportPolygon(ContactState contactStateToPopulate, FramePose2DReadOnly supportPolygonPose,
                                           FramePose2DReadOnly leftAnklePose, FramePose2DReadOnly rightAnklePose, ConvexPolygon2D leftFootSupportPolygon,
                                           ConvexPolygon2D rightFootSupportPolygon, double precision)
   {
      boolean isLeftFootSupported = leftAnklePose != null && !leftAnklePose.containsNaN() && leftFootSupportPolygon.getNumberOfVertices() > 0;
      boolean isRightFootSupported = rightAnklePose != null && !rightAnklePose.containsNaN() && rightFootSupportPolygon.getNumberOfVertices() > 0;
      int numberOfVertices = 0;
      if (isRightFootSupported && isLeftFootSupported)
      {
         addVerticesInCentroidalPoseFrameToListFromAnklePoseAndPolygon(supportPolygonPose, rightAnklePose, rightFootSupportPolygon, numberOfVertices);
         numberOfVertices += rightFootSupportPolygon.getNumberOfVertices();
         addVerticesInCentroidalPoseFrameToListFromAnklePoseAndPolygon(supportPolygonPose, leftAnklePose, leftFootSupportPolygon, numberOfVertices);
         numberOfVertices += leftFootSupportPolygon.getNumberOfVertices();
         roundVertexCoordinatesToPrecision(precision, numberOfVertices);
         generateMinimalVertexSupportPolygon(tempPolygon, tempVertexList, numberOfVertices);
      }
      else if (isLeftFootSupported)
      {
         tempPolygon.set(leftFootSupportPolygon);
         TransformHelperTools.transformFromPoseToPose(leftAnklePose, supportPolygonPose, tempPolygon);
      }
      else if (isRightFootSupported)
      {
         tempPolygon.set(rightFootSupportPolygon);
         TransformHelperTools.transformFromPoseToPose(leftAnklePose, supportPolygonPose, tempPolygon);
      }
      else
      {
         tempPolygon.clearAndUpdate();
      }
      contactStateToPopulate.setPose(supportPolygonPose);
      contactStateToPopulate.setSupportPolygon(tempPolygon);
   }

   private void roundVertexCoordinatesToPrecision(double precision, int numberOfVertices)
   {
      for (int i = 0; i < numberOfVertices; i++)
      {
         Point2D vertexToRound = tempVertexList.get(i);
         double newX = MathTools.roundToPrecision(vertexToRound.getX(), precision);
         double newY = MathTools.roundToPrecision(vertexToRound.getY(), precision);
         vertexToRound.set(newX, newY);
      }
   }

   public void generateMinimalVertexSupportPolygon(ConvexPolygon2D polygonToSet, ArrayList<Point2D> vertexList, int numberOfVertices)
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
      Point2D topRightVertex = vertexList.get(candidateVertexIndex);
      Point2D firstVertex = vertexList.get(0);
      vertexList.set(0, topRightVertex);
      vertexList.set(candidateVertexIndex, firstVertex);
      // Start the marching
      for (int i = 1; i < numberOfVertices; i++)
      {
         Point2D lastComputedPoint = vertexList.get(i - 1);
         Point2D candidatePoint = vertexList.get(i);
         // Find the next one 
         for (int j = i + 1; j < numberOfVertices; j++)
         {
            Point2D pointUnderConsideration = vertexList.get(j);
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
         Point2D vertexToSet = tempVertexList.get(i + firstIndex);
         vertexToSet.set(footSupportPolygonInAnkleFrame.getVertex(i));
         TransformHelperTools.transformFromPoseToPose(anklePose, desiredPose, vertexToSet);
      }
   }

   
   // AS: From this point on all functions are specific to the kind of behavior wanted from the contact state plan

   /**
    * Generates a contact state plan that move the robot by the value specified in {@code pelvisPoseChangePerJump}. The feet are held at a constant 
    * pose offset from the pelvis pose. 
    * @param contactStates the list of contact states to be populated. Should be of size (2 * {@code numberOfJumps} + 1) 
    * @param numberOfJumps the number of jumps to be planned
    * @param initialPelvisPose initial pose of the pelvis. The reference frame of this pose is used as the reference frame in which the plan is generated
    * @param pelvisPoseChangePerJump the change in the pelvis pose after every jump
    * @param leftAnklePoseOffset the offset of the left ankle frame
    * @param rightAnklePoseOffset the offset of the right ankle frame
    * @param flightDuration the duration of the flight phase
    * @param groundDuration the duration of the double support phase
    * @param footSupportPolygon the support polygon in ankle frames for both feet
    */
   public void generateContactStatePlanForJumping(List<ContactState> contactStates, int numberOfJumps, FramePose2DReadOnly initialPelvisPose,
                                                  Pose2DReadOnly pelvisPoseChangePerJump, Pose2DReadOnly leftAnklePoseOffset,
                                                  Pose2DReadOnly rightAnklePoseOffset, double flightDuration, double groundDuration, ConvexPolygon2D footSupportPolygon)
   {
      if (contactStates.size() < numberOfJumps * 2 + 1)
         throw new RuntimeException("Contact state list does not contain enough elements to store contact states for " + numberOfJumps + " jumps");
      
      tempPose.setIncludingFrame(initialPelvisPose);
      addPose(tempPoseForLeftFoot, tempPose, leftAnklePoseOffset);
      addPose(tempPoseForRightFoot, tempPose, rightAnklePoseOffset);
      
      ContactState firstGroundState = contactStates.get(0);
      computeAndSetSupportPolygon(firstGroundState, tempPose, tempPoseForLeftFoot, tempPoseForRightFoot, footSupportPolygon, footSupportPolygon);
      firstGroundState.setDuration(groundDuration);
      for (int i = 0; i < numberOfJumps; i++)
      {
         ContactState flightState = contactStates.get(2 * i + 1);
         flightState.setPose(tempPose);
         tempPolygon.clear();
         flightState.setSupportPolygon(tempPolygon);
         flightState.setDuration(flightDuration);
         
         addPose(tempPose, tempPose, pelvisPoseChangePerJump);
         
         ContactState groundState = contactStates.get(2 * i + 2);
         groundState.setPose(tempPose);
         firstGroundState.getSupportPolygon(tempPolygon);
         groundState.setSupportPolygon(tempPolygon);
         groundState.setDuration(groundDuration);
      }
   }

   public void generateContactStatePlanForRunning(List<ContactState> contactStates, int numberOfSteps, FramePose2DReadOnly initialLeftFootPose,
                                                  FramePose2DReadOnly rightFootPose, FramePose2DReadOnly poseDelta, RobotSide firstStepSide,
                                                  boolean useLastStepToEndRun)
   {
      
   }

   public void generateContactStatePlanForWalking(List<ContactState> contactStates, int numberOfSteps, FramePose2DReadOnly initialLeftFootPose,
                                                  FramePose2DReadOnly rightFootPose, double xStep, double yStep, double yawStep, RobotSide firstStepSide,
                                                  boolean useLastStepToEndWalk)
   {

   }

   private void addPose(FramePose2D poseToSet, FramePose2DReadOnly pose1, Pose2DReadOnly pose2)
   {
      poseToSet.setIncludingFrame(pose1);
      poseToSet.appendTranslation(pose2.getPosition());
      poseToSet.appendRotation(pose2.getOrientation());
   }
}

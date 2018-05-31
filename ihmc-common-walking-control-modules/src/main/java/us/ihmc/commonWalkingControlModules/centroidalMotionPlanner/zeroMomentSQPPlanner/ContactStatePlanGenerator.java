package us.ihmc.commonWalkingControlModules.centroidalMotionPlanner.zeroMomentSQPPlanner;

import java.nio.file.FileVisitResult;
import java.util.ArrayList;
import java.util.List;

import com.jme3.math.Plane.Side;

import boofcv.abst.geo.fitting.GenerateMotionPnP;
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
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
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

   private final FramePose2D tempPoseForLeftFoot = new FramePose2D();
   private final FramePose2D tempPoseForRightFoot = new FramePose2D();
   private final FramePose2D tempPose = new FramePose2D();
   private final ConvexPolygon2D tempPolygon = new ConvexPolygon2D();
   private final ArrayList<Point2D> tempVertexList = new ArrayList<>();

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

   public void computeAndSetSupportPolygon(ContactState contactStateToPopulate, FramePose2DReadOnly supportPolygonPose, FramePose2DReadOnly leftAnklePose,
                                           FramePose2DReadOnly rightAnklePose, ConvexPolygon2D leftFootSupportPolygon, ConvexPolygon2D rightFootSupportPolygon,
                                           double precision)
   {
      boolean isLeftFootSupported = leftAnklePose != null && !leftAnklePose.containsNaN() && leftFootSupportPolygon.getNumberOfVertices() > 0;
      boolean isRightFootSupported = rightAnklePose != null && !rightAnklePose.containsNaN() && rightFootSupportPolygon.getNumberOfVertices() > 0;
      if (isRightFootSupported && isLeftFootSupported)
         changePoseAndMergePolygons(tempPolygon, supportPolygonPose, leftAnklePose, leftFootSupportPolygon, rightAnklePose, rightFootSupportPolygon, precision);
      else if (isLeftFootSupported)
         getPolygonInDesiredPose(tempPolygon, supportPolygonPose, leftAnklePose, leftFootSupportPolygon);
      else if (isRightFootSupported)
         getPolygonInDesiredPose(tempPolygon, supportPolygonPose, rightAnklePose, rightFootSupportPolygon);
      else
         tempPolygon.clearAndUpdate();
      contactStateToPopulate.setPose(supportPolygonPose);
      contactStateToPopulate.setSupportPolygon(tempPolygon);
   }

   public void setSupportPolygon(ContactState contactStateToPopulate, FramePose2DReadOnly supportPolygonPose, ConvexPolygon2D supportPolygon)
   {
      contactStateToPopulate.setPose(supportPolygonPose);
      contactStateToPopulate.setSupportPolygon(supportPolygon);
   }

   public void changePoseAndSetSupportPolygon(ContactState contactStateToPopulate, FramePose2DReadOnly desiredPose, FramePose2DReadOnly supportPolygonPose,
                                              ConvexPolygon2D supportPolygon)
   {
      tempPolygon.set(supportPolygon);
      TransformHelperTools.transformFromPoseToPose(supportPolygonPose, desiredPose, tempPolygon);
      setSupportPolygon(contactStateToPopulate, desiredPose, tempPolygon);
   }

   public void getPolygonInDesiredPose(ConvexPolygon2D polygonToSet, FramePose2DReadOnly desiredPose, FramePose2DReadOnly poseForPolygon1,
                                       ConvexPolygon2D polygon1)
   {
      polygonToSet.set(polygon1);
      TransformHelperTools.transformFromPoseToPose(poseForPolygon1, desiredPose, polygonToSet);
   }

   public void mergePolygons(ConvexPolygon2D polygonToSet, ConvexPolygon2D polygon1, ConvexPolygon2D polygon2, double precision)
   {
      int maxNumberOfVertices = polygon1.getNumberOfVertices() + polygon2.getNumberOfVertices();
      if (maxNumberOfVertices > tempVertexList.size())
         throw new RuntimeException("Not enough vertices to combine the polygons");
      for (int i = 0; i < polygon1.getNumberOfVertices(); i++)
         tempVertexList.get(i).set(polygon1.getVertex(i));
      for (int i = 0; i < polygon2.getNumberOfVertices(); i++)
         tempVertexList.get(i + polygon1.getNumberOfVertices()).set(polygon2.getVertex(i));
      roundVertexCoordinatesToPrecision(precision, maxNumberOfVertices);
      generateMinimalVertexSupportPolygon(polygonToSet, tempVertexList, maxNumberOfVertices);
   }

   public void changePoseAndMergePolygons(ConvexPolygon2D polygonToSet, FramePose2DReadOnly desiredPose, FramePose2DReadOnly poseForPolygon1,
                                          ConvexPolygon2D polygon1, FramePose2DReadOnly poseForPolygon2, ConvexPolygon2D polygon2, double precision)
   {
      int maxNumberOfVertices = polygon1.getNumberOfVertices() + polygon2.getNumberOfVertices();
      if (maxNumberOfVertices > tempVertexList.size())
         throw new RuntimeException("Not enough vertices to combine the polygons");
      tempPolygon.set(polygon1);
      TransformHelperTools.transformFromPoseToPose(poseForPolygon1, desiredPose, tempPolygon);
      for (int i = 0; i < polygon1.getNumberOfVertices(); i++)
         tempVertexList.get(i).set(tempPolygon.getVertex(i));
      tempPolygon.set(polygon2);
      TransformHelperTools.transformFromPoseToPose(poseForPolygon2, desiredPose, tempPolygon);
      for (int i = 0; i < polygon2.getNumberOfVertices(); i++)
         tempVertexList.get(i + polygon1.getNumberOfVertices()).set(tempPolygon.getVertex(i));
      roundVertexCoordinatesToPrecision(precision, maxNumberOfVertices);
      generateMinimalVertexSupportPolygon(polygonToSet, tempVertexList, maxNumberOfVertices);
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
                                                  Pose2DReadOnly rightAnklePoseOffset, double flightDuration, double groundDuration,
                                                  ConvexPolygon2D footSupportPolygon)
   {
      generateContactStatePlanForJumping(contactStates, numberOfJumps, initialPelvisPose, pelvisPoseChangePerJump, leftAnklePoseOffset, rightAnklePoseOffset,
                                         flightDuration, groundDuration, footSupportPolygon, footSupportPolygon);
   }

   public void generateContactStatePlanForJumping(List<ContactState> contactStates, int numberOfJumps, FramePose2DReadOnly initialPelvisPose,
                                                  Pose2DReadOnly pelvisPoseChangePerJump, Pose2DReadOnly leftAnklePoseOffset,
                                                  Pose2DReadOnly rightAnklePoseOffset, double flightDuration, double groundDuration,
                                                  ConvexPolygon2D leftFootSupportPolygon, ConvexPolygon2D rightFootSupportPolygon)
   {
      if (contactStates.size() < numberOfJumps * 2 + 1)
         throw new RuntimeException("Contact state list does not contain enough elements to store contact states for " + numberOfJumps + " jumps");

      tempPose.setIncludingFrame(initialPelvisPose);
      addPose(tempPoseForLeftFoot, tempPose, leftAnklePoseOffset);
      addPose(tempPoseForRightFoot, tempPose, rightAnklePoseOffset);

      ContactState firstGroundState = contactStates.get(0);
      changePoseAndMergePolygons(tempPolygon, initialPelvisPose, tempPoseForLeftFoot, leftFootSupportPolygon, tempPoseForRightFoot, rightFootSupportPolygon,
                                 defaultPrecision);
      setSupportPolygon(firstGroundState, initialPelvisPose, tempPolygon);
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

   public void generateContactStatePlanForWalking(List<ContactState> contactStates, int numberOfSteps, FramePose2DReadOnly initialLeftAnklePose,
                                                  FramePose2DReadOnly initialRightAnklePose, Point2DReadOnly stepSize, RobotSide firstStepSide,
                                                  double singleSupportDuration, double transferDuration, double initialDoubleSupportDuration,
                                                  double finalDoubleSupportDuration, boolean useLastStepToEndWalk, ConvexPolygon2D leftFootSupportPolygon,
                                                  ConvexPolygon2D rightFootSupportPolygon)
   {
      if (contactStates.size() < numberOfSteps * 2 + 1)
         throw new RuntimeException("Contact state list does not contain enough elements to store contact states for " + numberOfSteps + " steps");
      ContactState initialDoubleSupportState = contactStates.get(0);
      initialDoubleSupportState.setDuration(initialDoubleSupportDuration);
      tempPoseForLeftFoot.set(initialLeftAnklePose);
      tempPoseForRightFoot.set(initialRightAnklePose);
      computeAveragePose(tempPose, initialLeftAnklePose, initialRightAnklePose);
      changePoseAndMergePolygons(tempPolygon, tempPose, tempPoseForLeftFoot, leftFootSupportPolygon, tempPoseForRightFoot, rightFootSupportPolygon,
                                 defaultPrecision);
      setSupportPolygon(initialDoubleSupportState, tempPose, tempPolygon);
      RobotSide stepSide = firstStepSide;
      double halfStepX = stepSize.getX() / 2.0;
      double halfStepY = stepSize.getY() / 2.0;
      for (int i = 0; i < numberOfSteps - 1; i++)
      {
         ContactState singleSupportState = contactStates.get(2 * i + 1);
         singleSupportState.setDuration(singleSupportDuration);
         if (stepSide == RobotSide.LEFT)
         {
            setSupportPolygon(singleSupportState, tempPoseForRightFoot, rightFootSupportPolygon);
            tempPoseForLeftFoot.appendTranslation(halfStepX, halfStepY);
         }
         else
         {
            setSupportPolygon(singleSupportState, tempPoseForLeftFoot, leftFootSupportPolygon);
            tempPoseForRightFoot.appendTranslation(halfStepX, halfStepY);
         }
         ContactState transferState = contactStates.get(2 * i + 2);
         transferState.setDuration(transferDuration);
         computeAveragePose(tempPose, initialLeftAnklePose, initialRightAnklePose);
         changePoseAndMergePolygons(tempPolygon, tempPose, tempPoseForLeftFoot, leftFootSupportPolygon, tempPoseForRightFoot, rightFootSupportPolygon,
                                    defaultPrecision);
         setSupportPolygon(transferState, tempPose, tempPolygon);
         stepSide = stepSide.getOppositeSide();
         if (stepSide == RobotSide.LEFT)
            tempPoseForLeftFoot.appendTranslation(halfStepX, halfStepY);
         else
            tempPoseForRightFoot.appendTranslation(halfStepX, halfStepY);
      }
      ContactState singleSupportState = contactStates.get(2 * numberOfSteps - 1);
      singleSupportState.setDuration(singleSupportDuration);
      if (stepSide == RobotSide.LEFT)
      {
         setSupportPolygon(singleSupportState, tempPoseForRightFoot, rightFootSupportPolygon);
         if (!useLastStepToEndWalk)
            tempPoseForLeftFoot.appendTranslation(halfStepX, halfStepY);
      }
      else
      {
         setSupportPolygon(singleSupportState, tempPoseForLeftFoot, leftFootSupportPolygon);
         if (!useLastStepToEndWalk)
            tempPoseForRightFoot.appendTranslation(halfStepX, halfStepY);
      }
      ContactState finalDoubleSupportState = contactStates.get(2 * numberOfSteps);
      finalDoubleSupportState.setDuration(finalDoubleSupportDuration);
      computeAveragePose(tempPose, initialLeftAnklePose, initialRightAnklePose);
      changePoseAndMergePolygons(tempPolygon, tempPose, tempPoseForLeftFoot, leftFootSupportPolygon, tempPoseForRightFoot, rightFootSupportPolygon,
                                 defaultPrecision);
      setSupportPolygon(finalDoubleSupportState, tempPose, tempPolygon);
   }

   public void generateContactStatePlanForRunning(List<ContactState> contactStates, int numberOfSteps, FramePose2DReadOnly initialLeftAnklePose,
                                                  FramePose2DReadOnly initialRightAnklePose, Point2DReadOnly stepSize, RobotSide firstStepSide,
                                                  double flightDuration, double singleSupportDuration, double initialDoubleSupportDuration,
                                                  double finalDoubleSupportDuration, boolean useLastStepToEndWalk, ConvexPolygon2D leftFootSupportPolygon,
                                                  ConvexPolygon2D rightFootSupportPolygon)
   {
      if (contactStates.size() < numberOfSteps * 2 + 1)
         throw new RuntimeException("Contact state list does not contain enough elements to store contact states for " + numberOfSteps + " steps");

      ContactState initialDoubleSupportState = contactStates.get(0);
      initialDoubleSupportState.setDuration(initialDoubleSupportDuration);
      tempPoseForLeftFoot.set(initialLeftAnklePose);
      tempPoseForRightFoot.set(initialRightAnklePose);
      computeAveragePose(tempPose, initialLeftAnklePose, initialRightAnklePose);
      changePoseAndMergePolygons(tempPolygon, tempPose, tempPoseForLeftFoot, leftFootSupportPolygon, tempPoseForRightFoot, rightFootSupportPolygon,
                                 defaultPrecision);
      setSupportPolygon(initialDoubleSupportState, tempPose, tempPolygon);
      RobotSide stepSide = firstStepSide;
      double halfStepX = stepSize.getX() / 2.0;
      double halfStepY = stepSize.getY() / 2.0;

      for (int i = 0; i < numberOfSteps - 1; i++)
      {
         ContactState singleSupportState = contactStates.get(2 * i + 1);
         singleSupportState.setDuration(singleSupportDuration);

         ContactState flightState = contactStates.get(2 * i + 2);
         flightState.setDuration(flightDuration);
         if (stepSide == RobotSide.LEFT)
         {
            tempPoseForLeftFoot.appendTranslation(halfStepX, halfStepY);
            setSupportPolygon(singleSupportState, tempPoseForRightFoot, rightFootSupportPolygon);
            tempPolygon.clear();
            tempPoseForRightFoot.appendTranslation(halfStepX, halfStepY);
            computeAveragePose(tempPose, tempPoseForLeftFoot, tempPoseForRightFoot);
            setSupportPolygon(flightState, tempPose, tempPolygon);
         }
         else
         {
            tempPoseForRightFoot.appendTranslation(halfStepX, halfStepY);
            setSupportPolygon(singleSupportState, tempPoseForLeftFoot, leftFootSupportPolygon);
            tempPolygon.clear();
            tempPoseForLeftFoot.appendTranslation(halfStepX, halfStepY);
            computeAveragePose(tempPose, tempPoseForLeftFoot, tempPoseForRightFoot);
            setSupportPolygon(flightState, tempPose, tempPolygon);
         }
         stepSide = stepSide.getOppositeSide();
      }
      ContactState singleSupportState = contactStates.get(2 * numberOfSteps - 1);
      singleSupportState.setDuration(flightDuration);
      if (stepSide == RobotSide.LEFT)
         setSupportPolygon(singleSupportState, tempPoseForRightFoot, rightFootSupportPolygon);
      else
         setSupportPolygon(singleSupportState, tempPoseForLeftFoot, leftFootSupportPolygon);
      ContactState finalDoubleSupportState = contactStates.get(2 * numberOfSteps);
      finalDoubleSupportState.setDuration(finalDoubleSupportDuration);
      if (!useLastStepToEndWalk)
      {
         if (stepSide == RobotSide.LEFT)
            tempPoseForLeftFoot.appendTranslation(halfStepX, halfStepY);
         else
            tempPoseForRightFoot.appendTranslation(halfStepX, halfStepY);
         computeAveragePose(tempPose, tempPoseForLeftFoot, tempPoseForRightFoot);
      }
      changePoseAndMergePolygons(tempPolygon, tempPose, tempPoseForLeftFoot, leftFootSupportPolygon, tempPoseForRightFoot, rightFootSupportPolygon,
                                 defaultPrecision);
      setSupportPolygon(finalDoubleSupportState, tempPose, tempPolygon);
   }

   private void computeAveragePose(FramePose2D poseToSet, FramePose2DReadOnly pose1, FramePose2DReadOnly pose2)
   {
      pose1.checkReferenceFrameMatch(pose2);
      poseToSet.setReferenceFrame(pose1.getReferenceFrame());
      double x = 0.5 * (pose1.getX() + pose2.getX());
      double y = 0.5 * (pose1.getY() + pose2.getY());
      double yaw = 0.5 * (pose1.getYaw() + pose2.getYaw());
      poseToSet.set(x, y, yaw);
   }

   private void addPose(FramePose2D poseToSet, FramePose2DReadOnly pose1, Pose2DReadOnly pose2)
   {
      poseToSet.setIncludingFrame(pose1);
      poseToSet.appendTranslation(pose2.getPosition());
      poseToSet.appendRotation(pose2.getOrientation());
   }
}

package us.ihmc.commonWalkingControlModules.centroidalMotionPlanner.zeroMomentController.footControl;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactPointInterface;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.ReferenceFrameHolder;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactableFoot;

public class ContactPointLabelHolder
{
   private double heelX;
   private double toeX;
   private final List<Boolean> isToeVertexList = new ArrayList<>();
   private final List<Boolean> isHeelVertexList = new ArrayList<>();

   // For any non standard point annotations use a HashMap<String propertyName, List<Boolean>>. Currently not used since. 
   public ContactPointLabelHolder(int numberOfVertices)
   {
      for (int i = 0; i < numberOfVertices; i++)
      {
         isHeelVertexList.add(false);
         isToeVertexList.add(false);
      }
   }

   public void setVertexLabels(int index, boolean isHeelVertex, boolean isToeVertex)
   {
      isHeelVertexList.set(index, isHeelVertex);
   }

   public boolean isHeelVertex(int index)
   {
      return isHeelVertexList.get(index);
   }

   public boolean isToeVertex(int index)
   {
      return isToeVertexList.get(index);
   }

   public void getIsToeVertexFlag(List<Boolean> toeVertexFlagsToPopulate)
   {
      toeVertexFlagsToPopulate.clear();
      for (int i = 0; i < isToeVertexList.size(); i++)
         toeVertexFlagsToPopulate.add(isToeVertex(i));
   }

   public void getIsHeelVertexFlag(List<Boolean> heelVertexFlagsToPopulate)
   {
      heelVertexFlagsToPopulate.clear();
      for (int i = 0; i < isHeelVertexList.size(); i++)
         heelVertexFlagsToPopulate.add(isHeelVertex(i));
   }

   public void getToeVertices(List<? extends Point2DReadOnly> vertexList, List<Point2DReadOnly> toeVertices)
   {
      for (int i = 0; i < vertexList.size(); i++)
      {
         if (isToeVertexList.get(i))
            toeVertices.add(vertexList.get(i));
      }
   }

   public void getHeelVertices(List<? extends Point2DReadOnly> vertexList, List<Point2DReadOnly> heelVertices)
   {
      for (int i = 0; i < vertexList.size(); i++)
      {
         if (isHeelVertexList.get(i))
            heelVertices.add(vertexList.get(i));
      }
   }

   public static ContactPointLabelHolder createLabelsFromContactPointList(List<? extends ContactPointInterface> contactPoints)
   {
      checkReferenceFrameMatch(contactPoints);
      ContactPointLabelHolder contactPointLabelHolder = new ContactPointLabelHolder(contactPoints.size());
      contactPointLabelHolder.toeX = contactPoints.get(0).getPosition().getX();
      contactPointLabelHolder.heelX = contactPointLabelHolder.toeX;
      for (int i = 1; i < contactPoints.size(); i++)
      {
         double polygonVertexX = contactPoints.get(i).getPosition().getX();
         if (polygonVertexX > contactPointLabelHolder.toeX)
            contactPointLabelHolder.toeX = polygonVertexX;
         if (polygonVertexX < contactPointLabelHolder.heelX)
            contactPointLabelHolder.heelX = polygonVertexX;
      }

      for (int i = 0; i < contactPoints.size(); i++)
      {
         FramePoint3DReadOnly vertex = contactPoints.get(i).getPosition();
         contactPointLabelHolder.setVertexLabels(i, vertex.getX() == contactPointLabelHolder.toeX, vertex.getX() == contactPointLabelHolder.heelX);
      }
      return contactPointLabelHolder;

   }

   private static void checkReferenceFrameMatch(List<? extends ReferenceFrameHolder> referenceFrameHolderList)
   {
      ReferenceFrame referenceFrame = referenceFrameHolderList.get(0).getReferenceFrame();
      for (int i = 1; i < referenceFrameHolderList.size(); i++)
         referenceFrameHolderList.get(i).checkReferenceFrameMatch(referenceFrame);
   }

   public static ContactPointLabelHolder createLabelsFromFramePointList(List<? extends FramePoint2DReadOnly> contactPoints)
   {
      checkReferenceFrameMatch(contactPoints);
      ContactPointLabelHolder contactPointLabelHolder = new ContactPointLabelHolder(contactPoints.size());
      contactPointLabelHolder.toeX = contactPoints.get(0).getX();
      contactPointLabelHolder.heelX = contactPointLabelHolder.toeX;
      for (int i = 1; i < contactPoints.size(); i++)
      {
         double polygonVertexX = contactPoints.get(i).getX();
         if (polygonVertexX > contactPointLabelHolder.toeX)
            contactPointLabelHolder.toeX = polygonVertexX;
         if (polygonVertexX < contactPointLabelHolder.heelX)
            contactPointLabelHolder.heelX = polygonVertexX;
      }

      for (int i = 0; i < contactPoints.size(); i++)
      {
         FramePoint2DReadOnly vertex = contactPoints.get(i);
         contactPointLabelHolder.setVertexLabels(i, vertex.getX() == contactPointLabelHolder.toeX, vertex.getX() == contactPointLabelHolder.heelX);
      }
      return contactPointLabelHolder;

   }

   public static ContactPointLabelHolder createLabelsFromContactableFoot(ContactableFoot contactableFoot)
   {
      return createLabelsFromFramePointList(contactableFoot.getContactPoints2d());
   }

   public double getHeelX()
   {
      return heelX;
   }

   public double getToeX()
   {
      return toeX;
   }
}

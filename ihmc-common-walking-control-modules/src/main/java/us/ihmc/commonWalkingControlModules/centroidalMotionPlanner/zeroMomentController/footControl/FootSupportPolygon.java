package us.ihmc.commonWalkingControlModules.centroidalMotionPlanner.zeroMomentController.footControl;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactableFoot;

public class FootSupportPolygon
{
   private final List<Point2DReadOnly> verticesInSoleFrame = new ArrayList<>();
   private final List<Boolean> isToeVertex = new ArrayList<>();
   private final List<Boolean> isHeelVertex = new ArrayList<>();

   public void addVertex(Point2DReadOnly vertex, boolean isToeVertex, boolean isHeelVertex)
   {
      verticesInSoleFrame.add(new Point2D(vertex));
      this.isToeVertex.add(isToeVertex);
      this.isHeelVertex.add(isHeelVertex);
   }

   public void addVertex(double x, double y, boolean isToeVertex, boolean isHeelVertex)
   {
      verticesInSoleFrame.add(new Point2D(x, y));
      this.isToeVertex.add(isToeVertex);
      this.isHeelVertex.add(isHeelVertex);
   }

   public void addVertices(List<Point2D> vertices, List<Boolean> isToeVertex, List<Boolean> isHeelVertex)
   {
      if (vertices.size() != isHeelVertex.size() || vertices.size() != isToeVertex.size())
         throw new RuntimeException("Mismatch in vertice list size and vertex list type");
      for (int i = 0; i < vertices.size(); i++)
         addVertex(vertices.get(i), isToeVertex.get(i), isHeelVertex.get(i));
   }

   public void getVertices(List<Point2DReadOnly> vertices)
   {
      for (int i = 0; i < verticesInSoleFrame.size(); i++)
         vertices.add(verticesInSoleFrame.get(i));
   }

   public void getToeVertices(List<Point2DReadOnly> toeVertices)
   {
      for (int i = 0; i < verticesInSoleFrame.size(); i++)
      {
         if (isToeVertex.get(i))
            toeVertices.add(verticesInSoleFrame.get(i));
      }
   }

   public void getHeelVertices(List<Point2DReadOnly> heelVertices)
   {
      for (int i = 0; i < verticesInSoleFrame.size(); i++)
      {
         if (isHeelVertex.get(i))
            heelVertices.add(verticesInSoleFrame.get(i));
      }
   }

   public static FootSupportPolygon createSupportPolygonFromContactableFoot(ContactableFoot contactableFoot)
   {
      FootSupportPolygon supportPolygonToSet = new FootSupportPolygon();
      List<FramePoint2D> contactPoints = contactableFoot.getContactPoints2d();
      double maxX = contactPoints.get(0).getX();
      double minX = maxX;
      for (int i = 1; i < contactPoints.size(); i++)
      {
         double polygonVertexX = contactPoints.get(i).getX();
         if(polygonVertexX > maxX)
            maxX = polygonVertexX;
         if(polygonVertexX < minX)
            minX = polygonVertexX;
      }

      for (int i = 0; i < contactPoints.size(); i++)
      {
         FramePoint2D vertex = contactPoints.get(i);
         supportPolygonToSet.addVertex(vertex, vertex.getX() == maxX, vertex.getX() == minX);
      }
      return supportPolygonToSet;
   }
}

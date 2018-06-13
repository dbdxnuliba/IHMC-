package us.ihmc.commonWalkingControlModules.centroidalMotionPlanner.zeroMomentController;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;

public class FootSupportPolygon
{
   private final List<Point2DReadOnly> verticesInSoleFrame = new ArrayList<>();
   private final List<Boolean> isToeVertex = new ArrayList<>();
   private final List<Boolean> isHeelVertex = new ArrayList<>();

   public void addVertex(Point2D vertex, boolean isToeVertex, boolean isHeelVertex)
   {
      verticesInSoleFrame.add(new Point2D(vertex));
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
}

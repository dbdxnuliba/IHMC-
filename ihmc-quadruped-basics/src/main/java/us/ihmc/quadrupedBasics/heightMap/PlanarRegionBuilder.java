package us.ihmc.quadrupedBasics.heightMap;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.linearAlgebra.PrincipalComponentAnalysis3D;

import java.util.*;
import java.util.stream.Collectors;

public class PlanarRegionBuilder implements Iterable<Point3DReadOnly>
{
   private int id = PlanarRegion.NO_REGION_ID;

   private final PrincipalComponentAnalysis3D pca = new PrincipalComponentAnalysis3D();
   private final VectorMean normal = new VectorMean();
   private final PointMean origin = new PointMean();
   private final Vector3D standardDeviationPrincipalValues = new Vector3D();
   private final Quaternion orientation = new Quaternion();

   private final RigidBodyTransform transformFromLocalToWorld = new RigidBodyTransform();

   private final List<Point3DReadOnly> nodes = new ArrayList<>();
   private final Set<Point3DReadOnly> nodeSet = new HashSet<>();

   public PlanarRegionBuilder()
   {
   }

   public PlanarRegionBuilder(int id)
   {
      this.id = id;
   }

   public PlanarRegionBuilder(int id, Collection<Point3DReadOnly> nodes)
   {
      this(id);
      addNodes(nodes);
   }

   public void setId(int id)
   {
      this.id = id;
   }

   public void reset()
   {
      nodes.clear();
      nodeSet.clear();
   }

   public boolean addNode(Point3DReadOnly node)
   {
      boolean isRegionModified = nodeSet.add(node);
      if (isRegionModified)
      {
         nodes.add(node);
         computeNormalAndOrigin();
      }
      return isRegionModified;
   }

   public boolean addNodes(Point3DReadOnly... nodes)
   {
      boolean isRegionModified = false;
      for (Point3DReadOnly node : nodes)
      {
         if (nodeSet.add(node))
            isRegionModified = true;
      }
      if (isRegionModified)
      {
         for (Point3DReadOnly node : nodes)
            this.nodes.add(node);
         computeNormalAndOrigin();
      }

      return isRegionModified;
   }


   public boolean addNodes(Collection<Point3DReadOnly> nodes)
   {
      boolean isRegionModified = nodeSet.addAll(nodes);
      if (isRegionModified)
      {
         this.nodes.addAll(nodes);
         computeNormalAndOrigin();
      }

      return isRegionModified;
   }

   public boolean contains(Point3DReadOnly node)
   {
      return nodeSet.contains(node);
   }

   private void computeNormalAndOrigin()
   {
      pca.clear();
      nodes.stream().forEach(node -> pca.addPoint(node.getX(), node.getY(), node.getZ()));
      pca.compute();

      Point3D mean = new Point3D();
      pca.getMean(mean);

      origin.clear();
      origin.update(mean, getNumberOfNodes());

      Vector3D thirdVector = new Vector3D();
      pca.getThirdVector(thirdVector);
      pca.getStandardDeviation(standardDeviationPrincipalValues);

      if (thirdVector.dot(normal) < 0.0)
         thirdVector.negate();
      normal.clear();
      normal.update(thirdVector, getNumberOfNodes());
      orientation.set(EuclidGeometryTools.axisAngleFromZUpToVector3D(normal));

      transformFromLocalToWorld.set(orientation, origin);
   }

   public PlanarRegion buildPlanarRegion()
   {
      ConvexPolygon2D convexPolygon = new ConvexPolygon2D();
      List<Point2D> pointsInPlane = toPointsInPlane(nodes, origin, orientation);
      for (Point2D pointInPlane : pointsInPlane)
         convexPolygon.addVertex(pointInPlane);
      PlanarRegion planarRegion = new PlanarRegion(transformFromLocalToWorld, convexPolygon);
      planarRegion.setRegionId(id);
      return planarRegion;
   }

   public int getId()
   {
      return id;
   }

   public void getPoint(int index, Point3DBasics pointToPack)
   {
      pointToPack.set(nodes.get(index));
   }

   public Point3DReadOnly getNode(int index)
   {
      return nodes.get(index);
   }

   public double angle(Vector3DReadOnly normal)
   {
      return this.normal.angle(normal);
   }

   public double absoluteAngle(Vector3DReadOnly normal)
   {
      return Math.abs(angle(normal));
   }

   public Vector3D getNormal()
   {
      return normal;
   }

   public Point3D getOrigin()
   {
      return origin;
   }

   public Vector3D getStandardDeviationPrincipalValues()
   {
      return standardDeviationPrincipalValues;
   }

   public boolean isEmpty()
   {
      return nodes.isEmpty();
   }

   public int getNumberOfNodes()
   {
      return nodes.size();
   }

   private static List<Point2D> toPointsInPlane(List<? extends Point3DReadOnly> pointsToTransform, Point3DReadOnly planeOrigin,
                                               Orientation3DReadOnly planeOrientation)
   {
      return pointsToTransform.stream().map(point -> toPointInPlane(point, planeOrigin, planeOrientation)).collect(Collectors.toList());
   }

   private static Point2D toPointInPlane(Point3DReadOnly pointToTransform, Point3DReadOnly planeOrigin, Orientation3DReadOnly planeOrientation)
   {
      return toPointInPlane(pointToTransform.getX(), pointToTransform.getY(), pointToTransform.getZ(), planeOrigin, planeOrientation);
   }

   private static Point2D toPointInPlane(double xToTransform, double yToTransform, double zToTransform, Point3DReadOnly planeOrigin,
                                        Orientation3DReadOnly planeOrientation)
   {
      Point3D point3DInPlane = new Point3D(xToTransform, yToTransform, zToTransform);
      point3DInPlane.sub(planeOrigin);
      planeOrientation.inverseTransform(point3DInPlane);
      return new Point2D(point3DInPlane);
   }

   @Override
   public Iterator<Point3DReadOnly> iterator()
   {
      return nodes.iterator();
   }

   @Override
   public String toString()
   {
      String ret = "Region ID: " + id;
      ret += ", origin: " + origin + ", normal: " + normal;
      ret += ", size: " + getNumberOfNodes();
      return ret;
   }
}

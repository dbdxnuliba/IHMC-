package us.ihmc.robotEnvironmentAwareness.fusion.tools;

import org.apache.commons.lang3.mutable.MutableDouble;
import org.apache.commons.lang3.mutable.MutableInt;
import org.apache.commons.math3.stat.descriptive.moment.Variance;
import org.ejml.alg.dense.decomposition.svd.SvdImplicitQrDecompose_D64;
import org.ejml.data.DenseMatrix64F;
import org.ejml.interfaces.decomposition.SingularValueDecomposition;
import org.ejml.ops.MatrixFeatures;
import org.ejml.ops.SingularOps;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.jOctoMap.tools.IncrementalCovariance3D;
import us.ihmc.robotEnvironmentAwareness.fusion.data.SuperPixelData;
import us.ihmc.robotEnvironmentAwareness.fusion.parameters.SuperPixelNormalEstimationParameters;
import us.ihmc.robotics.linearAlgebra.PrincipalComponentAnalysis3D;

import java.util.List;
import java.util.Random;
import java.util.concurrent.ThreadLocalRandom;
import java.util.stream.Stream;

public class SuperPixelNormalEstimationTools
{
   public static void updateUsingPCA(SuperPixelData superPixel, List<Point3DReadOnly> points, boolean addPointsInParallel)
   {
      PrincipalComponentAnalysis3D pca = new PrincipalComponentAnalysis3D();
      Stream<Point3DReadOnly> pointStream = addPointsInParallel ? points.parallelStream() : points.stream();
      pointStream.forEach(pca::addDataPoint);

      pca.compute();

      Point3D center = new Point3D();
      Vector3D normal = new Vector3D();
      Vector3D standardDeviation = new Vector3D();

      pca.getMean(center);
      pca.getThirdVector(normal);
      pca.getStandardDeviation(standardDeviation);

      if (normal.getZ() < 0.0)
         normal.negate();

      superPixel.setCenter(center);
      superPixel.setNormal(normal);
      superPixel.setStandardDeviation(standardDeviation);
   }

   public static void updateUsingRansac(SuperPixelData currentPixel, List<Point3DReadOnly> points, SuperPixelNormalEstimationParameters parameters)
   {
      if (points.size() < 2)
         return;

      double maxDistanceFromPlane = parameters.getMaxDistanceFromPlane();
      // assume vertical
      Vector3D currentNormal = new Vector3D(0.0, 0.0, 1.0);
      if (currentPixel.isNormalSet())
         currentNormal.set(currentPixel.getNormal());
      Point3D currentCenterLocation = new Point3D();
      if (currentPixel.isCenterSet())
         currentCenterLocation.set(currentPixel.getCenter());
      else
         currentCenterLocation.set(getRandomStartPoint(points));

      // Need to be recomputed as the neighbors may have changed
      MutableInt currentConsensus = new MutableInt();
      MutableDouble currentVariance = new MutableDouble();
      computeNormalConsensusAndVariance(currentCenterLocation, currentNormal, points, maxDistanceFromPlane, currentVariance, currentConsensus);

      for (int iteration = 0; iteration < parameters.getNumberOfIterations(); iteration++)
      {
         Vector3D candidateNormal = computeNormalFromTwoRandomPoints(points, currentCenterLocation);

         if (candidateNormal == null)
            continue;

         if (parameters.isLeastSquaresEstimationEnabled())
            candidateNormal = refineNormalWithLeastSquares(currentCenterLocation, candidateNormal, maxDistanceFromPlane, points);

         if (candidateNormal == null)
            continue;

         MutableInt candidateConsensus = new MutableInt();
         MutableDouble candidateVariance = new MutableDouble();
         computeNormalConsensusAndVariance(currentCenterLocation, candidateNormal, points, maxDistanceFromPlane, candidateVariance, candidateConsensus);

         peekBestNormal(currentPixel, currentNormal, currentVariance, currentConsensus, candidateNormal, candidateVariance, candidateConsensus, parameters);
      }
   }

   private static Point3DReadOnly getRandomStartPoint(List<Point3DReadOnly> points)
   {
      return points.get(RandomNumbers.nextInt(ThreadLocalRandom.current(), 0, points.size() - 1));
   }

   private static boolean peekBestNormal(SuperPixelData superPixel, Vector3DReadOnly currentNormal, MutableDouble currentVariance, MutableInt currentConsensus,
                                         Vector3DBasics candidateNormal, MutableDouble candidateVariance, MutableInt candidateConsensus,
                                         SuperPixelNormalEstimationParameters parameters)
   {
      if (isCandidateNormalBetter(currentVariance, currentConsensus, candidateVariance, candidateConsensus, parameters))
      {
         if (currentNormal.dot(candidateNormal) < 0.0)
            candidateNormal.negate();

         superPixel.setNormal(candidateNormal);
         superPixel.setNormalQuality(candidateVariance.floatValue(), candidateConsensus.intValue());
         currentConsensus.setValue(candidateConsensus);
         currentVariance.setValue(candidateVariance);
         return true;
      }
      return false;
   }

   private static boolean isCandidateNormalBetter(MutableDouble currentVariance, MutableInt currentConsensus, MutableDouble candidateVariance,
                                                  MutableInt candidateConsensus, SuperPixelNormalEstimationParameters parameters)
   {
      double minConsensusRatio = parameters.getMinConsensusRatio();
      double maxAverageDeviationRatio = parameters.getMaxAverageDeviationRatio();

      boolean isBetter = candidateConsensus.intValue() >= currentConsensus.intValue() && candidateVariance.doubleValue() <= currentVariance.doubleValue();
      if (isBetter)
         return true;

      boolean hasSmallerConsensusButIsMuchBetter = candidateConsensus.intValue() >= (int) (minConsensusRatio * currentConsensus.intValue())
            && candidateVariance.doubleValue() <= maxAverageDeviationRatio * currentVariance.doubleValue();
      return hasSmallerConsensusButIsMuchBetter;
   }

   private static Vector3D computeNormalFromTwoRandomPoints(List<Point3DReadOnly> neighbors, Point3DReadOnly currentPixelCenter)
   {
      Random random = ThreadLocalRandom.current();

      int maxNumberOfAttempts = 5;

      int iteration = 0;
      Vector3D normalCandidate = null;

      while (normalCandidate == null && iteration++ < maxNumberOfAttempts)
      {
         Point3D[] randomHitLocations = random.ints(0, neighbors.size())
                                              .distinct()
                                              .limit(2)
                                              .mapToObj(neighbors::get)
                                              .map(Point3D::new)
                                              .toArray(Point3D[]::new);

         normalCandidate = EuclidGeometryTools.normal3DFromThreePoint3Ds(currentPixelCenter, randomHitLocations[0], randomHitLocations[1]);
      }
      return normalCandidate;
   }

   private static Vector3D refineNormalWithLeastSquares(Point3DReadOnly pointOnPlane, Vector3DReadOnly ransacNormal, double maxDistanceFromPlane,
                                                        List<Point3DReadOnly> points)
   {
      IncrementalCovariance3D covarianceCalculator = new IncrementalCovariance3D();

      Vector3D toNeighborHitLocation = new Vector3D();

      for (Point3DReadOnly point: points)
      {
         if (point == pointOnPlane)
            continue;

         toNeighborHitLocation.sub(point, pointOnPlane);
         double distanceFromPlane = Math.abs(ransacNormal.dot(toNeighborHitLocation));
         if (distanceFromPlane <= maxDistanceFromPlane)
            covarianceCalculator.addDataPoint(point.getX(), point.getY(), point.getZ());
      }

      if (covarianceCalculator.getSampleSize() <= 2)
         return null;

      SingularValueDecomposition<DenseMatrix64F> svd = new SvdImplicitQrDecompose_D64(true, false, true, false);
      svd.decompose(covarianceCalculator.getCovariance());
      DenseMatrix64F v = svd.getV(null, false);
      if (MatrixFeatures.hasNaN(v))
         return null;
      SingularOps.descendingOrder(null, false, svd.getW(null), v, false);

      Vector3D refinedNormal = new Vector3D(v.get(0, 2), v.get(1, 2), v.get(2, 2));
      refinedNormal.normalize();
      return refinedNormal;
   }

   private static void computeNormalConsensusAndVariance(Point3DReadOnly pointOnPlane, Vector3DReadOnly planeNormal, Iterable<Point3DReadOnly> points,
                                                         double maxDistanceFromPlane, MutableDouble varianceToPack, MutableInt consensusToPack)
   {
      Variance variance = new Variance();
      consensusToPack.setValue(0);

      Vector3D toNeighborHitLocation = new Vector3D();

      for (Point3DReadOnly point : points)
      {
         if (point == pointOnPlane)
            continue;

         toNeighborHitLocation.set(point);
         toNeighborHitLocation.sub(pointOnPlane);
         double distanceFromPlane = Math.abs(planeNormal.dot(toNeighborHitLocation));
         if (distanceFromPlane <= maxDistanceFromPlane)
         {
            variance.increment(distanceFromPlane);
            consensusToPack.increment();
         }
      }

      if (consensusToPack.intValue() == 0)
         varianceToPack.setValue(Double.POSITIVE_INFINITY);
      else
         varianceToPack.setValue(variance.getResult());
   }
}

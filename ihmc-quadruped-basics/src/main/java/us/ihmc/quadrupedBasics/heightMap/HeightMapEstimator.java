package us.ihmc.quadrupedBasics.heightMap;

import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoInteger;

import java.util.ArrayList;
import java.util.List;

public class HeightMapEstimator
{
   private static final int defaultMaxBufferSize = 50;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final YoInteger maxBufferSize = new YoInteger("MaxPointBufferSize", registry);
   private final YoInteger currentPointBufferSize = new YoInteger("CurrentPointBufferSize", registry);

   private final List<PlanarRegion> planarRegionsBuffer = new ArrayList<>();
   private final List<HeightMapListener> heightMapListeners = new ArrayList<>();

   public HeightMapEstimator(YoVariableRegistry parentRegistry)
   {
      maxBufferSize.set(defaultMaxBufferSize);

      parentRegistry.addChild(registry);
   }

   public void processIncomingPoint(Point3DReadOnly pointInWorld)
   {
      List<PlanarRegion> regionsToAdd = new ArrayList<>();

      PlanarRegion containingRegion = getContainingRegion(pointInWorld);
      if (containingRegion == null)
      {
         PlanarRegion nearestRegion = getNearestRegion(pointInWorld);
         regionsToAdd.add(createFromNearbyRegion(pointInWorld, nearestRegion));
      }
      else
      {
         regionsToAdd.addAll(createByDividingRegion(pointInWorld, containingRegion));
      }

      addRegionsToBuffer(regionsToAdd);
      reportUpdatedRegions();
   }

   public void setMaxBufferSize(int maxBufferSize)
   {
      this.maxBufferSize.set(maxBufferSize);
   }

   public List<PlanarRegion> getHeightMapRegionBuffer()
   {
      return planarRegionsBuffer;
   }

   public void attachHeightMapListener(HeightMapListener heightMapListener)
   {
      heightMapListeners.add(heightMapListener);
   }

   private void reportUpdatedRegions()
   {
      PlanarRegionsList planarRegionsList = new PlanarRegionsList(planarRegionsBuffer);
      for (HeightMapListener heightMapListener : heightMapListeners)
         heightMapListener.reportNewHeightMap(planarRegionsList);
   }
}

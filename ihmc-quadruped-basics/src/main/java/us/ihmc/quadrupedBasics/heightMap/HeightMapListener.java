package us.ihmc.quadrupedBasics.heightMap;

import us.ihmc.robotics.geometry.PlanarRegionsList;

public interface HeightMapListener
{
   void reportNewHeightMap(PlanarRegionsList planarRegionsList);
}

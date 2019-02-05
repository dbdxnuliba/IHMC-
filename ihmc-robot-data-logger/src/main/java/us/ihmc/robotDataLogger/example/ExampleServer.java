package us.ihmc.robotDataLogger.example;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.log.LogTools;
import us.ihmc.robotDataLogger.YoVariableServer;
import us.ihmc.robotDataLogger.logger.DataServerSettings;
import us.ihmc.robotDataLogger.util.PeriodicGCFreeNonRealtimeThreadSchedulerFactory;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoInteger;
import us.ihmc.yoVariables.variable.YoLong;
import us.ihmc.yoVariables.variable.YoVariable;

public class ExampleServer
{
   public enum SomeEnum
   {
      A, B, C, D, E, F;
   }

   private static final int variablesPerType = 1000;
   private static final double dt = 0.001;
   private static final DataServerSettings logSettings = new DataServerSettings(true);

   private final Random random = new Random(127L);
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final YoVariableServer yoVariableServer;
   
   private final List<YoVariable<?>> allChangingVariables = new ArrayList<>();

   private long timestamp = 0;

   public ExampleServer()
   {
      createVariables();

      PeriodicGCFreeNonRealtimeThreadSchedulerFactory schedulerFactory = new PeriodicGCFreeNonRealtimeThreadSchedulerFactory();
      yoVariableServer = new YoVariableServer(getClass(), schedulerFactory, null, logSettings, dt);
      yoVariableServer.setMainRegistry(registry , null, null);
      
      
   }

   public void start()
   {
      yoVariableServer.start();

      LogTools.info("Starting to loop.");
      while (true)
      {
         updateVariables();
         yoVariableServer.update(timestamp++);
         ThreadTools.sleepSeconds(dt);
      }
   }

   private void createVariables()
   {
      for (int i = 0; i < variablesPerType; i++)
      {
         new YoBoolean("Boolean" + i, registry);
         new YoDouble("Double" + i, registry);
         new YoInteger("Integer" + i, registry);
         new YoLong("Long" + i, registry);
         new YoEnum<>("Enum" + i, registry, SomeEnum.class, random.nextBoolean());
      }
      
      
      allChangingVariables.addAll(registry.getAllVariablesIncludingDescendants());
      
      
      YoDouble input = new YoDouble("input", registry);
      YoDouble output = new YoDouble("output", registry);
      input.addVariableChangedListener((v) -> output.set(input.getValue()));
      
   }

   private void updateVariables()
   {
      for (int varIdx = 0; varIdx < allChangingVariables.size(); varIdx++)
      {
         updateVariable(allChangingVariables.get(varIdx));
      }
   }

   private void updateVariable(YoVariable<?> variable)
   {
      if (variable instanceof YoBoolean)
      {
         ((YoBoolean) variable).set(random.nextBoolean());
      }
      else if (variable instanceof YoDouble)
      {
         ((YoDouble) variable).set(random.nextDouble());
      }
      else if (variable instanceof YoInteger)
      {
         ((YoInteger) variable).set(random.nextInt());
      }
      else if (variable instanceof YoLong)
      {
         ((YoLong) variable).set(random.nextLong());
      }
      else if (variable instanceof YoEnum<?>)
      {
         int enumSize = ((YoEnum<?>) variable).getEnumSize();
         ((YoEnum<?>) variable).set(random.nextInt(enumSize));
      }
      else
      {
         throw new RuntimeException("Implement this case for " + variable.getClass().getSimpleName() + ".");
      }
   }

   public static void main(String[] args)
   {
      LogTools.info("Starting " + ExampleServer.class.getSimpleName());
      ExampleServer exampleServer = new ExampleServer();
      exampleServer.start();
   }
}

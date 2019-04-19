package us.ihmc.robotEnvironmentAwareness.fusion;

import java.util.concurrent.atomic.AtomicReference;
import controller_msgs.msg.dds.ImageMessage;
import javafx.animation.AnimationTimer;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.javaFXToolkit.messager.SharedMemoryJavaFXMessager;
import java.net.Socket;
import java.nio.file.Files;
import java.util.Arrays;
import java.io.*;

public class FusionObjectDetector
{

    private JavaFXMessager messager;
    private final AnimationTimer timer;

    protected final AtomicReference<ImageMessage> newImageMessageToSend;
    private final AtomicReference<Boolean> detectDoorMessage;
    private String host = "127.0.0.1";
    private int port = 1111;
    public FusionObjectDetector(SharedMemoryJavaFXMessager messager)
    {
        this.messager = messager;
        detectDoorMessage = messager.createInput(LidarImageFusionAPI.DetectDoor, false);
        newImageMessageToSend = messager.createInput(LidarImageFusionAPI.ImageState);

        timer = new AnimationTimer()
        {
            @Override
            public void handle(long now)
            {
                if (detectDoorMessage.getAndSet(false))
                {
                    detectDoor();
                }
            }
        };

        messager.registerMessagerStateListener(isMessagerOpen ->
        {
            if (isMessagerOpen)
                start();
            else
                sleep();
        });
    }

    public void start()
    {
        timer.start();
    }

    public void sleep()
    {
        timer.stop();
    }

    private void detectDoor()
    {
        try
        {
            Socket soc = new Socket(host, port);
            System.out.println("Connected");

            File f = new File("C:/Users/buzz7/OneDrive/Desktop/door8.jpg");
            byte[] fileByte = Files.readAllBytes(f.toPath());

            DataOutputStream dos = new DataOutputStream(soc.getOutputStream());
            dos.write(fileByte);

            BufferedReader br = new BufferedReader(new InputStreamReader(soc.getInputStream()));
            int[] result = Arrays.stream(br.readLine().split(",")).mapToInt(Integer::parseInt).toArray();
            System.out.println(Arrays.toString(result));
        }
        catch (Exception e)
        {
            e.printStackTrace();
        }
    }
}
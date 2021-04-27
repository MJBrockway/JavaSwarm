import java.util.*;
import java.io.*;
import org.json.*; //Json support: org.json.jar needs to be in classpath
import java.nio.file.*; //Path, Files

public class Flatten {
  public static void main(String[] args) throws IOException, JSONException {
    SwarmModel m = SwarmModel.loadSwarmJson(args[0]);
    m.saveSwarm("flat.txt");
  }  
}


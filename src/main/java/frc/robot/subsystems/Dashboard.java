package frc.robot.subsystems;
import java.util.HashMap;
import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;


public class Dashboard {
    public static void parse(HashMap<String, Object> fields, HashMap<String, Object> button)throws IOException{
        //BufferedReader reader = new BufferedReader(new InputStreamReader(System.in));
            for(String a : fields.keySet()){
                System.out.println(a + ", " + fields.get(a));
            }
        }
}

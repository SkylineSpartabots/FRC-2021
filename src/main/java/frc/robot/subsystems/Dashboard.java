package frc.robot.subsystems;
import java.util.HashMap;
import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;

public class Dashboard {
    public static void parse(HashMap<String, Object> fields, HashMap<String, Button> button)throws IOException{
        BufferedReader reader = new BufferedReader(new InputStreamReader(System.in));
        if (reader.readLine().equals("y")){
            for(String a : fields.keySet()){
                System.out.println(a + ": " + fields.get(a));
            }
        } else{
            System.out.println("No output requested.");
        }
    }
}
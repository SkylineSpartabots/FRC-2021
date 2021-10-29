package frc.robot.subsystems;
import java.util.HashMap;
import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;

/**
 * Parses telemetry maps and prints them out to console for now, might add to dashboard later?
 * @see Subsystem#outputTelemetry() usage
 */

public class Dashboard {

    /**
     * Prints out buttons and fields to console for rn
     * @param fields config field and values to print
     * @param button buttons to print (should work better when actually added to console)
     */
    public static void parse(HashMap<String, Object> fields, HashMap<String, Button> button) {
        //BufferedReader reader = new BufferedReader(new InputStreamReader(System.in));
            System.out.println("Printing out field values");
            for(String a : fields.keySet()){
                System.out.println(a + ", " + fields.get(a));
            }
            System.out.println("Printing out Buttons");
            button.forEach((s, o) -> System.out.println("Button Name: " + s));
        }
}

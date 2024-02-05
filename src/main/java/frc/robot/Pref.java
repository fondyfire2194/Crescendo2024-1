/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.Collection;
/**
 * Add your docs here.
 */
import java.util.Enumeration;
import java.util.HashMap;
import java.util.Iterator;
import java.util.Map;
import java.util.Vector;

import edu.wpi.first.wpilibj.Preferences;

public class Pref {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private static Collection<String> v;
  private static Enumeration<String> e;
  private static String tempString;
  private static double tempDouble;

  public static HashMap<String, Double> prefDict = new HashMap<>();

  static {

    // drive tune
    prefDict.put("DriveKp", .001);
    prefDict.put("DriveFF", .5);

    // angle tune

    prefDict.put("AngleKp", .01);

    //shooter

    prefDict.put("LeftRPM", 500.);
    prefDict.put("RightRPM", 500.);
    prefDict.put("FeedRPM", 500.);

    prefDict.put("ShooterKp", .01);
    prefDict.put("FeederKp", .01);
    prefDict.put("ShooterAngleKp", .01);


    prefDict.put("IntakeKp", .01);
    prefDict.put("NoteHoldKp", .01);

    prefDict.put("ElevatorKp", .01);




    //intake / holdnote

    prefDict.put("IntakeRPM", 500.);
    prefDict.put("HoldNoteInRPM", 500.);
    prefDict.put("HoldNoteShootRPM", 500.);
    prefDict.put("HoldNoteAmpRPM", 500.);
    


  }

  public static void ensureRioPrefs() {
    // init();
    deleteUnused();
    addMissing();
  }

  public static void deleteUnused() {
    v = new Vector<String>();
    v = Preferences.getKeys();
    // v = (Vector<String>) RobotContainer.prefs.getKeys();
    String[] myArray = v.toArray(new String[v.size()]);

    for (int i = 0; i < v.size(); i++) {
      boolean doNotDelete = myArray[i].equals(".type");

      if (!doNotDelete && !prefDict.containsKey(myArray[i]) && Preferences.containsKey(myArray[i])) {
        Preferences.remove(myArray[i]);
      }
    }

  }

  public static void addMissing() {

    Iterator<Map.Entry<String, Double>> it = prefDict.entrySet().iterator();
    while (it.hasNext()) {
      Map.Entry<String, Double> pair = it.next();
      tempString = pair.getKey();
      tempDouble = pair.getValue();
      if (!Preferences.containsKey((tempString)))
        Preferences.setDouble(tempString, tempDouble);
    }
  }

  public static double getPref(String key) {
    if (prefDict.containsKey(key))
      return Preferences.getDouble(key, prefDict.get(key));
    else
      return 0;
  }

  public static void deleteAllPrefs(Preferences Preferences) {
    edu.wpi.first.wpilibj.Preferences.removeAll();
  }

}

package frc.robot;

import java.io.FileOutputStream;
import java.io.IOException;
import java.io.ObjectOutputStream;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.io.FileInputStream;
import java.io.ObjectInputStream;

public class JoyReadWrite {

    private static JoyStorage joyStorage[];
    JoyReadWrite() {
        

    }

    public static void writeObject(JoyStorage joyStorage[], String fileName) {
        try (FileOutputStream fos = new FileOutputStream("/home/lvuser/" + fileName + ".dat");
                ObjectOutputStream oos = new ObjectOutputStream(fos)) {

            // create a new user object
            // User user = new User("John Doe", "john.doe@example.com",
            // new String[] { "Member", "Admin" }, true);

            // write object to file
            oos.writeObject(joyStorage);
            oos.flush();
            oos.close();
            fos.flush();
            fos.close();

        } catch (IOException ex) {
            ex.printStackTrace();
        }
    }

    public static JoyStorage[] readObject(String fileName) {
      //  JoyStorage joyStorage[];// = new JoyStorage();
        try (FileInputStream fis = new FileInputStream("/home/lvuser/" + fileName + ".dat");
                ObjectInputStream ois = new ObjectInputStream(fis)) {

            // read object from file
           // User user = (User) ois.readObject();
           joyStorage = (JoyStorage[]) ois.readObject();

           SmartDashboard.putString("In readobject", "in");
            // print object
            System.out.println(joyStorage);

        } catch (IOException | ClassNotFoundException ex) {
            ex.printStackTrace();
        }
        return joyStorage;
    }

}

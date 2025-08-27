package frc.robot.common;

import java.io.IOException;
import java.io.PrintWriter;
import java.net.Socket;

public class RoboTalker {

    public static void backtalk(){
        Socket socket = null;
        try {
            socket = new Socket("192.168.12.193", 5810);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
        PrintWriter out = null;
        try {
            out = new PrintWriter(socket.getOutputStream(), true);
        } catch (IOException e) {
            try {
                socket.close();
            } catch (IOException error) {
                throw new RuntimeException(error);
            }
            throw new RuntimeException(e);
        }
        out.println("Hello, I am speaking now!");
        try {
            socket.close();
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }
}

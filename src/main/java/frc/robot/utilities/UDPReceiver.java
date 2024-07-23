package frc.robot.utilities;

import java.io.*;
import java.math.BigInteger;
import java.net.*;

//import com.google.gson.Gson;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class UDPReceiver extends Thread {
    public static String lastDataReceived = "";

    protected DatagramSocket socket = null;
    protected BufferedReader in = null;
    protected boolean moreQuotes = true;

    public UDPReceiver() throws IOException {
        this("udpReceiver");
    }

    public UDPReceiver(String name) throws IOException {
        super(name);
        socket = new DatagramSocket(3620);
    }

    public void run() {
        Util.log("Start UDP thread");
        byte[] buf = new byte[256];
        DatagramPacket packet = new DatagramPacket(buf, buf.length);
        while (moreQuotes) {
            try {

                // receive request
                packet.setLength(buf.length);
                socket.receive(packet);
                byte[] data = packet.getData();
                lastDataReceived = new String(data, 0, packet.getLength());
                SmartDashboard.putString("last Data Received", lastDataReceived);
                Util.logf("Data from UPD:%s\n", lastDataReceived);
            } catch (IOException e) {
                e.printStackTrace();
                moreQuotes = false;
            }
        }
        socket.close();
    }

    public String toHex(String arg) {
        return String.format("%040x", new BigInteger(1, arg.getBytes()));
    }

}
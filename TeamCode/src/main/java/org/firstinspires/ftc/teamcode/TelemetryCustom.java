package org.firstinspires.ftc.teamcode;

import android.os.Environment;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;

public class TelemetryCustom {
    private Telemetry telemetry;
    private FileWriter fileWriter;
    private SimpleDateFormat timeFormat;
    private ElapsedTime runtime;

    public TelemetryCustom(Telemetry telemetry) {
        this.telemetry = telemetry;
        this.timeFormat = new SimpleDateFormat("HH:mm:ss.SSS", Locale.US);
        this.runtime = new ElapsedTime();
        try {
            // Define the folder path: /sdcard/FIRST/logs/
            String path = Environment.getExternalStorageDirectory().getAbsolutePath() + "/FIRST/logs";
            File dir = new File(path);

            // Create directory if it doesn't exist
            if (!dir.exists()) {
                dir.mkdirs();
            }

            // Create a unique file name based on the start time
            String fileName = "Log_" + new SimpleDateFormat("yyyyMMdd_HHmm", Locale.US).format(new Date()) + ".txt";
            File file = new File(dir, fileName);

            // Initialize the writer
            fileWriter = new FileWriter(file, true);

            // Write a header
            WriteToFile("SYSTEM", "Log file created successfully");

        } catch (IOException e) {
            telemetry.addData("Error", "File IO Exception: " + e.getMessage());
            telemetry.update();
            fileWriter = null;
        }
    }

    public void Log(String caption, Object value) {
        // 1. Show on Driver Station Screen
        if (telemetry != null) {
            String displayValue = (value != null) ? value.toString() : "null";
            telemetry.addData(caption, displayValue);
            // telemetry.update(); // is slow to update every time, update once per loop instead
            WriteToFile(caption, displayValue);
        }
    }

    // Helper method to handle the actual file writing
    private void WriteToFile(String caption, String value) {
        if (fileWriter != null) {
            try {
                String timestamp = timeFormat.format(new Date());
                String logEntry = String.format("%s, %s| %s: %s\n", timestamp, runtime.seconds(), caption, value);
                fileWriter.write(logEntry);
            } catch (IOException e) {
                // Silently fail or handle error
            }
        }
    }
    public void Update(){
        if (telemetry != null) {
            telemetry.update();
        }
    }

    // IMPORTANT: You must call this when the OpMode stops to save the data!
    public void close() {
        if (fileWriter != null) {
            try {
                fileWriter.close();
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
    }
}
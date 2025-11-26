# downloading wirelessly using adb (for later)
# from gemini search result
To download code wirelessly to the Control Hub in FTC, connect your computer to the Control Hub's Wi-Fi network, then use the adb connect 192.168.43.1:5555 command in your terminal or command prompt. This uses Android Debug Bridge (ADB) to establish a wireless connection, allowing you to build and deploy your code directly from Android Studio.
This video shows how to connect the Control Hub and the Driver Station through Wi-Fi:
Related video thumbnail
58s


Step 1: Connect to the Control Hub's Wi-Fi
Power on the Control Hub. The LED will turn solid green when it's ready.
On your computer, find the Wi-Fi network broadcast by the Control Hub (it usually starts with "FTC_") and connect to it. It is normal for it to say "connected, no internet".
Step 2: Open a terminal and connect with ADB
Open the Terminal (macOS/Linux) or Command Prompt (Windows).
If you want to be absolutely sure no other devices are connected, first type adb disconnect and press Enter.
Then, type adb connect 192.168.43.1:5555 and press Enter. This command connects your computer to the Control Hub on port 5555.
If the command is successful, you should see a confirmation message.


Step 3: Deploy your code from Android Studio
Once connected, open your FTC project in Android Studio.
The Control Hub should now appear as a connected device.
Click the green play button to build and wirelessly deploy your code to the Control Hub.
Troubleshooting tips
Ensure your computer is connected to the correct Control Hub Wi-Fi network.
If you can't connect, try rebooting the Control Hub and your computer, then restart the process.
Some users find it helpful to use an additional USB Wi-Fi adapter on their computer so they can be connected to the internet and the Control Hub simultaneously, as shown in this Reddit post and this forum post.
The ADB executable must be in your system's PATH environment variable to be run directly. You may need to find the correct path to the adb.exe file within your Android Studio installation. 

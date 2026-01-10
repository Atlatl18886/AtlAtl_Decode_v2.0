https://developer.android.com/tools/releases/platform-tools
Setting up the Android Debug Bridge (ADB) for FTC on a Mac involves installing Android Studio, locating the ADB executable, and connecting to the robot's Control Hub, typically wirelessly via its Wi-Fi network.
Prerequisites
A Mac computer with Android Studio installed.
A powered-on REV Robotics Control Hub (or an Android device running the FTC Robot Controller app).
A USB-C cable for the initial connection (recommended for the first setup).
Step 1: Install Android Studio and ADB
Download and Install Android Studio from the Android developer's website.
During the initial setup, Android Studio will install the necessary Android SDK Platform-Tools, which include ADB.
Verify ADB installation by opening the Terminal in Android Studio (viewable in the bottom panel) and running adb devices. If it returns a list of devices, ADB is working.
Step 2: Connect to the Control Hub via Wi-Fi
The most common method for FTC development is a wireless connection, using the Control Hub's built-in Wi-Fi network. The Control Hub automatically runs the ADB server on port 5555.
Connect to the Control Hub's Wi-Fi network from your Mac's Wi-Fi settings. The network name is usually FTC_#### or TEAMNUMBER-RC. The default password is on the hub's label.
Open the Terminal within Android Studio.
Connect ADB to the Control Hub by running the following command:
bash
adb connect 192.168.43.1:5555
Note: The IP address for a REV Control Hub is always 192.168.43.1 over its Wi-Fi network.
Verify the connection by running adb devices. Your device's serial number should appear as a connected device.
Alternative Method: Using the ADB Wi-Fi Plugin
For convenience, you can use a plugin to manage connections directly within the Android Studio interface.
In Android Studio, go to Preferences (on macOS: Android Studio > Preferences, or press ⌘,).
Navigate to Plugins > Marketplace, and search for the "ADB Wi-fi" plugin (by Yury Polek).
Install the plugin and restart Android Studio.
After restarting and connecting to the Control Hub's Wi-Fi network, a Wi-Fi icon will appear on the right-hand side of the screen. Click it to add and manage connections.
Once connected, the Control Hub will be available as a target device in Android Studio, allowing you to build and deploy your code wirelessly using the "Run" button. 


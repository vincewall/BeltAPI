![Cover banner](.github/cover.png)

# Source code for my Fusion 360 Timing Belt Generator

This repo contains all the source code for my Fusion 360 script. The script is pretty robust, but far from perfect. It can generate most timing belt drives successfully, but there are edge cases that it cannot handle due to simplifying assumptions I made during development. 

The belt generation happens entirely within the Python backend, based on my previous projects: `beltpy` and `pybeltsolver`. This is the main limiting factor of the script, as it can only generate belts with a specific set of properties. See the `pybeltsolver` repo for more details regarding this. There are also lower bound limits set for the tooth count and smooth pulley diameter in the `config.py` file to avoid issues during generation.

The script currently supports the following profiles:

* GT2 profiles with a 2 mm pitch
* GT3 profiles with a 3 mm pitch

More may be added in the future based on popular demand. You may also add your own profiles if you want. The procedure to do so is identical to the one described in the documentation for `pybeltsolver`.

# Documentation
Below you can find some useful information and instructions about the script.

## General Usage Instructions

### 1. Launching the script

Once installed, you can access the script from the `Scripts and Add-Ins` dialog box inside the Utilities tab.

**Step by step instructions**

1. Open Fusion 360 and load your project.
2. Navigate to the `UTILITIES` tab at the top of the window.
3. Click the `ADD-INS` (or Scripts and Add-Ins) button.
4. Inside the newly opened dialog box, select the `Scripts` tab and scroll down the list until you find `TimingBeltGenerator`.
5. Launch the script by selecting it and clicking the `Run` button.


### 2. Creating a belt
In the generator dialog box, you will need to specify multiple parameters. Below are some basic instructions. In addition to this, I highly recommend you watch the example video, as some of the steps are hard to convey by text.

**Step by step instructions**

1. **Specify the belt configuration**
    1. Specify the belt thickness in mm and select a belt profile from the dropdown list.
    2. Select a construction plane. This is the plane that the belt will be symmetrically extruded from.

2. **Select pulley positions and specify the topology**
    1. Add a new row in the table for every pulley or idler you desire to include in your system.
    2. For each entry in the table, you need to specify the type, the diameter / teeth count, and center coordinate. You specify a center coordinate by selecting a row in the table, then clicking on the `Select pulley position` box above the table. Now you may select vertices, sketch points, and circular features anywhere inside your project. Just note that all coordinates will be orthogonally projected onto your selected construction plane!

3. **Make a selection for the tensioner pulley**
    1. You will need to select at least one tensioner pulley to ensure that the midline path of the belt is divisible by the pitch of the selected tooth profile. Start by specifying the `Tensioner row index`. The pulley / idler in this row will act as your tensioner.
    2. Now, `Select pull direction`, i.e., select an additional point your tensioner will slide towards in order to adjust the belt's midline length. Consider this as a vector from point A (tensioner pulley) to point B (selected point) that the tensioner pulley may slide along.
    3. By this point, a preview should have rendered inside the viewport of your project. The preview will render green if your specified values and topology are valid. Otherwise, it will render red. Use the `Slide tensioner pulley` slider to get your desired belt length.

## Installation/Uninstallation

### Installing from the Fusion Marketplace

1. Visit the Fusion Marketplace and search for "Timing Belt Generator".
2. Click on "Get", then download the installer. Run it and follow the on-screen instructions.
3. Once installed, restart Fusion to make the script avaliable.


### Installing from the github source code

1. Download the repository as a ZIP file from GitHub and extract it.
2. Rename the extracted folder to exactly `TimingBeltGenerator.bundle`.
3. Move this renamed folder into your Fusion 360 plugins directory: `%APPDATA%\Autodesk\ApplicationPlugins`
4. Restart Fusion to make the script avaliable.

### Uninstalling
1. Navigate to `%APPDATA%\Autodesk\ApplicationPlugins`.
2. Remove the `TimingBeltGenerator.bundle` folder along with all its contents. 

## Support Information
To get additional help or to report bugs, feel free to contact me directly via my email: vincent.wallsten03@gmail.com, and I will try to reach back to you.

# Example Video

Below is a video demonstrating the script in action. Note that the value entered for "toothed" pulleys is the physical tooth count, while the value entered for the "smooth" tensioner pulley is the desired diameter in mm.

<video src="https://github.com/user-attachments/assets/c4fbc2e5-757e-42ea-b348-9093ce6b3d49" width="100%" autoplay loop muted playsinline></video>




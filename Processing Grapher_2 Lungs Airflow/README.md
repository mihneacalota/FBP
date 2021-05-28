[![GPLv3 License](https://img.shields.io/badge/License-GPL%20v3-yellow.svg)](https://opensource.org/licenses/)
[![Issues](https://img.shields.io/github/issues-raw/chillibasket/processing-grapher.svg?maxAge=25000)](https://github.com/chillibasket/processing-grapher/issues)
[![GitHub last commit](https://img.shields.io/github/last-commit/chillibasket/processing-grapher.svg?style=flat)](https://github.com/chillibasket/processing-grapher/commits/master)

# Serial Monitor and Real-time Graphing Program
This project is a Processing-based serial terminal and graphing program for the analysis and recording of data from serial devices, such as Arduinos. This program is designed as a replacement for the serial monitor contained within the Arduino IDE. The program contains easy-to-use tools to record data received from serial devices, and to plot numerical data on up to 4 separate graphs in real-time. This makes it useful for quickly analysing sensor data from a micro-controller. 

This is still a work in progress; please let me know if you come across any issues or bugs which need to be fixed!
A full description and set of instructions can be found on my website: [https://wired.chillibasket.com/processing-grapher/](https://wired.chillibasket.com/processing-grapher/)
</br>
</br>

![](/Images/LiveGraph_tab.jpg)
</br>
*Live graph tab, illustrating how real-time data can be plotted on multiple graphs*
</br>
</br>

## Features
1. Easy UI scale and colour theme changing 
1. Serial terminal monitor
	1. Connect to any serial port at any baud rate
	1. Send and receive serial communication
	1. Record the communication as a text file
	1. Change the colour of lines containing specific tags
1. Live Graphing
	1. Plot real-time data obtained from serial device on a graph
	1. Can display data on up to 4 separate graphs
	1. Supports comma delimited numbers only (example: 12,24,-15.4)
	1. Apply different colours and names to each input
	1. Record the real-time data as a comma delimited file
1. File Graphing
	1. Opens comma delimited files for analysis
	1. Apply different colours and names to each input
	1. Supports zooming into sections of the waveforms
	1. Add vertical markers/labels to the data
	1. Apply filters to remove noise and transform the data
</br>
</br>


## Installation/Setup Guide

### Basic Usage in the Processing IDE
1. Download and install the Processing IDE from [https://processing.org/](https://processing.org/).
1. Clone or download all files in this repository.
1. Open the main program file `ProcessingGrapher.pde` in the Processing editor. All the other files should automatically open in separate tabs in the Processing IDE.
1. Press the `Run` button in the top-left of the Processing editor to start the program.
</br>
</br>

### Creating a Stand-alone Program
It is possible to create a stand-alone version of the program, which does not require the Processing IDE to run.

1. Open the code in the Processing IDE, as described in the previous steps.
1. In the top bar, click on `File > Export Application...`
1. In the *Export Options* window that pops up, select the platform you want to export for and make sure that the *Embed Java* option is ticked. Finally, click *Export*.
1. This will create an application folder which will include either an `*.exe` file (Windows), shell script (Linux) or `*.app` launch file (OS X) which you can use to run the program.
</br>
</br>

## Getting Started
1. To connect to an Arduino:
	1. Ensure Arduino is plugged into your computer
	1. Go to the "Serial" tab of the program
	1. In the right-hand sidebar, press on `Port: None` button
	1. A list of all available ports should appear. Click on the port you want to connect to
	1. Press on the `Baud: 9600` button and select the baud rate of the serial connection
	1. Finally, click on the `Connect` button to initiate the connection with the Arduino
1. To plot real-time data received from the Arduino:
	1. Make sure that the data consists of numbers being separated by a comma
	1. For example the message `12,25,16` could be sent using Arduino code:
		```cpp
		Serial.print(dataPoint1); Serial.print(",");
		Serial.print(dataPoint2); Serial.print(",");
		Serial.println(dataPoint3);
		```
	1. Go to the "Live Graph" tab of the program. The data should automatically be plotted on the graph.
	1. To plot different signals on separate graphs, click on the number of graphs (1 to 4) in the "Split" section of the right-hand sidebar.
	1. You can then press the up or down buttons on each signal in the sidebar to move it to a different graph.
	1. To change options (such as graph type, x-axis and y-axis scaling) for a specific graph, click on the graph you want to edit. The options for that graph are then shown in the sidebar.

A full set of instructions and documentation can be found on my website at: [https://wired.chillibasket.com/processing-grapher/](https://wired.chillibasket.com/processing-grapher/)
</br>
</br>

![](/Images/SerialMonitor_tab.jpg) 
</br>
*Serial monitor tab, showing the communication with an Arduino*
</br>
</br>

![](/Images/FileGraph_tab.jpg)
</br>
*File graph tab, showing how information from a CSV file can be plotted on a graph*
</br>
</br>

## Changelog
1. (1st April 2021) Version 1.2.3
	1. Fixed export bug when trying to save CSV data to a file.
1. (6th March 2021) Version 1.2.2
	1. Fixed bug where the first and last data-point on the Live Graph were not being displayed.
	1. Improved mouse-wheel scrolling speed to better match the content.
1. (1st March 2021) Version 1.2.1 [Release]
	1. When adding labels to the graph in the "File Graph" tab, they are now stored as a new signal.
	1. Changes to the chart in the "File Graph" tab can now be saved to a new file.
	1. Added option to apply filters to the signals in the "File Graph" tab.
	1. Advanced serial port settings can now be modified in the "Settings" menu.
1. (9th February 2021) Version 1.2.0
	1. Updated to use the JavaFX (FX2D) renderer, which significantly reduces the processor usage of the program.
	1. Implemented native JavaFX pop-up dialogues while maintaining backwards compatibility with the default renderer.
1. (29th November 2020) Version 1.1.1
	1. Added button on "Live Graph" tab to toggle automatic y-axis scaling on/off.
	2. Added a "Hidden" section on the "Live Graph" tab so that unwanted signals can be hidden from the real-time graphs.
	3. Added a "Settings" sidebar menu to make changing program preferences easier.
	4. Added two additional program colour schemes.
1. (7th September 2020) Version 1.1.0 [Release]
	1. Updated all version number to differentiate between minor updates and larger releases.
	1. Improved the way in which the axis labels are displayed on the graphs.
	1. Constrain strings which are too long so they appear correctly in the right-hand menu.
1. (23rd August 2020) Version 1.0.8
	1. Added a file recording error recovery function, which deals with all possible error scenarios.
	1. Optimised the Serial monitor text rendering to improve the frame rate.
	1. Added menu button to turn off the automatic scrolling of the serial monitor.
	1. Improved the menu UI to make it clearer when a button is disabled.
	1. Menu clicks are now handled in a separate thread, meaning pop-ups etc. are no longer blocking.
	1. Added a new thread which updates the COM port list at regular intervals.
	1. Overhauled the "Graph" drawing class to make it more robust to displaying a variety of data.
1. (6th August 2020) Version 1.0.7
	1. Revamped file saving process so that new entries are written to the output file the moment they are received.
	1. When recording data for a long time, the entries are now automatically split into multiple files of maximum 100,000 rows.
	1. Updated live graphs to show a continuous running data stream, rather than drawing the data from left to right.
1. (3rd August 2020) Version 1.0.6
	1. Added "Getting Started" boxes to help new users.
	1. Updated error-handling of the CSV file saving process.
	1. Added variable which can be used to change the serial comms line-ending character.
	1. File open/save path strings are now properly cleaned to prevent backslash "\" errors.
1. (20th July 2020) Version 1.0.5
	1. Streamlined the Serial devices connection process by adding COM port and Baud rate options into sub menus on the right-hand sidebar (instead of blocking pop-ups).
	1. Removed the duplicate serial connection code from the "Live Graph" tab, so that future improvements to the serial connection process will be easier.
	1. Added loading screen on start-up.
	1. CTRL+ and CTRL- keyboard combinations can be used to increase and decrease the UI scaling factor.
1. (19th July 2020) Version 1.0.4
	1. Added scrolling support to the right-hand menu bar using the mouse scroll wheel or up/down arrow keys. This solves the issue where menu items could be hidden when the window size is too small.
	1. Finished updating all code function commenting to a more consistent Doxygen-style format.
1. (18th July 2020) Version 1.0.3
	1. Added usage instructions which appear in the serial monitor on start-up
	1. Added the "Inconsolata" font which is used in the serial monitor
	1. Fixed issue where serial messages which are longer than the window width would not appear. Now the visible portion of the text is shown, and a double arrow ">>" icon is used to show that some text is hidden. However, there is not any way to see that text yet, other than resizing the entire window.
	1. Added support for "Page Up" and "Page Down" keys to quickly scroll through text in the serial monitor.
1. (17th July 2020) Version 1.0.2
	1. Fixed live graph bug which plotted erroneous data when graph y-axis was resized.
	1. Added code which reset the live graph signal list when serial device is disconnected.
1. (19th April 2020) Version 1.0.1
	1. Added ability to display live serial data on up to four separate graphs.
	1. Graphs now support the display of linecharts, dotcharts and barcharts.
	1. Updated zooming options on the "Live Graph" and "File Graph" tabs.
	1. Fixed some of the bugs in displaying the live graph data.
	1. Changed method used to plot live serial data, so that the maximum frequency which can be displayed is no longer limited by th frame rate (60Hz).

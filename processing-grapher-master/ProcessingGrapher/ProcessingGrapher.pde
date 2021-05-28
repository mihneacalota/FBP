/* * * * * * * * * * * * * * * * * * * * * * *
 * PROCESSING GRAPHER
 *
 * @file      ProcessingGrapher.pde
 * @brief     Serial monitor and real-time graphing program
 * @author    Simon Bluett
 * @website   https://wired.chillibasket.com/processing-grapher/
 *
 * @copyright GNU General Public License v3
 * @date      1st April 2021
 * @version   1.2.3
 * * * * * * * * * * * * * * * * * * * * * * */

/*
 * Copyright (C) 2021 - Simon Bluett <hello@chillibasket.com>
 *
 * This file is part of ProcessingGrapher 
 * <https://github.com/chillibasket/processing-grapher>
 * 
 * ProcessingGrapher is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <https://www.gnu.org/licenses/>.
 */

String versionNumber = "1.2.3";

// Swing for input popups
import static javax.swing.JOptionPane.*;

// Serial port handling
import processing.serial.*;

// Advanced key inputs
import java.awt.event.KeyEvent;

// File dialogue
import java.io.File;

// Resizeable windows
import javax.swing.JFrame;
import java.awt.Dimension;
import processing.awt.PSurfaceAWT.SmoothCanvas;

// Java FX imports
import javafx.stage.FileChooser;
import javafx.stage.Stage;
import javafx.scene.canvas.Canvas;
import javafx.application.Platform;
import java.lang.reflect.*;


// -------- UI APPEARANCE SETTINGS ---------------------------------------------
// UI Scaling Options (eg. 0.6 = tiny, 1.0 = normal, 1.4 = huge)
float uimult = 1.0;

// Fonts
final String programFont = "Lucida Sans";
final String terminalFont = "Inconsolata-SemiBold.ttf";

// Predefined colors
final color c_white = color(255, 255, 255);
final color c_blue1 = color(96, 200, 220);
final color c_blue2 = color(120, 200, 220);
final color c_purple = color(147, 111, 212);
final color c_red = color(208, 38, 98);
final color c_yellow = color(215, 196, 96);
final color c_green = color(35, 205, 65);
final color c_orange = color(230, 85, 37);
final color c_lightgrey = color(134, 134, 138);
final color c_grey = color(111, 108, 90);
final color c_darkgrey = color(49, 50, 44);

// Select current colour scheme
// 0 = light (Celeste)
// 1 = dark  (One Dark Gravity)
// 2 = dark  (Monokai) - default
int colorScheme = 2;

// Graph colour list
color[] c_colorlist = {c_blue1, c_green, c_red, c_red};

// Default Window Size
int lastWidth = 1000;
int lastHeight = 700;

// Size Values
final int tabWidth = 90;
final int tabHeight = 30;
final int sidebarWidth = 150;
final int sideItemHeight = 30;
final int bottombarHeight = 22;
// -----------------------------------------------------------------------------

// Define UI colour variables
color c_background;
color c_tabbar;
color c_tabbar_h;
color c_idletab;
color c_tabbar_text;
color c_idletab_text;
color c_sidebar;
color c_sidebar_h;
color c_sidebar_heading;
color c_sidebar_text;
color c_sidebar_button;
color c_sidebar_divider;
color c_sidebar_accent;
color c_terminal_text;
color c_message_text;
color c_graph_axis;
color c_graph_gridlines;
color c_graph_border;
color c_serial_message_box;
color c_message_box_outline;
color c_alert_message_box;
color c_info_message_box;
color c_status_bar;

// Serial Port Variables
Serial myPort;
int portNumber = 0;
int baudRate = 9600;
char lineEnding = '\n';
boolean serialConnected = false;
String currentPort = "";
String[] portList;
char serialParity = 'N';
int serialDatabits = 8;
float serialStopbits = 1.0;

/**
 * Class containing all relevant info for a serial port connection
 * @TODO - Use this class to replace the variables being used above
 *//*
class SerialPortItem {
	public Serial port;
	public int portNumber;
	public String portName;
	public int baudRate;
	public char lineEnding;
	public boolean connected;

	public SerialPortItem() {
		portNumber = 0;
		portName = "";
		baudRate = 9600;
		lineEnding = '\n';
		connected = false;
	} 
}
ArrayList<SerialPortItem> serialObjects = new ArrayList<SerialPortItem>();
*/

// Drawing Booleans
boolean redrawUI = true;
boolean redrawAlert = false;
boolean redrawContent = true;
boolean drawNewData = false;
boolean drawFPS = false;
boolean preventDrawing = false;
boolean settingsMenuActive = false;
boolean showInstructions = true;
boolean programActive = true;
int state = 0;

// Interaction Booleans
boolean textInput = false;
boolean controlKey = false;
boolean scrollingActive = false;

// Tab Bar
ArrayList<TabAPI> tabObjects = new ArrayList<TabAPI>();
TabAPI settings;
int currentTab = 0;
int tabTop = round(tabHeight * uimult);

// Fonts
PFont base_font;
PFont mono_font;

// Alert Messages
final int alertWidth = 300;
final int alertHeight = 150;
String alertHeading = "";
boolean alertActive = false;

// Exit handler
DisposeHandler dh;

// JavaFX pop-up dialogues
Stage stage;
FileChooser fileChooser;
String userInputString = null;
int startTime;
PGraphics mainCanvas;

// Options are: FX2D (Recommended), JAVA2D
final String activeRenderer = FX2D;



/******************************************************//**
 * SETUP FUNCTIONS
 *
 * @info  Methods used to initialise all parts of the 
 *        GUI and set the values of required variables
 *********************************************************/

/**
 * Processing Setup function
 *
 * This sets up the window and rendering engine.
 * All other loading is done later from the draw() 
 * function after the loading screen is drawn.
 */
void setup() {
	println("'processing.awt.PSurfaceAWT': This warning is a known issue and doesn't affect the program");

	// Set up the window and rendering engine
	size(1000, 700, activeRenderer);
	smooth();

	loadColorScheme(colorScheme);
	background(c_background);

	// Set the desired frame rate (FPS)
	frameRate(60);
}


/**
 * Program Setup Function
 *
 * Initialise all screen drawing parameters and 
 * instantiate the classes for each of the tabs. 
 */
void setupProgram() {
	startTime = millis();

	// Java FX specific setup
	if (activeRenderer == FX2D) {
		stage = (Stage) ((Canvas) surface.getNative()).getScene().getWindow();
		fileChooser = new FileChooser(); 

		// Minimum Window Size
		stage.setMinWidth(600);
    	stage.setMinHeight(350);

    // Java default renderer specific setup
	} else if (activeRenderer == JAVA2D) {
		// Minimum Window Size
		SmoothCanvas sc = (SmoothCanvas) getSurface().getNative();
		JFrame jf = (JFrame) sc.getFrame();
		Dimension d = new Dimension(600, 350);
		jf.setMinimumSize(d);
	}

    // Ensure window close event is called properly
	dh = new DisposeHandler(this);

	// Add window title and icon
	surface.setIcon(loadImage("icon-48.png"));
	surface.setTitle("Processing Grapher");
	mainCanvas = g;

	// All window to be resized
	surface.setResizable(true);

	// Initialise the fonts
	base_font = createFont(programFont, int(13*uimult), true);
	mono_font = createFont(terminalFont, int(14*uimult), true);
	textFont(base_font);

	// Calculate initial screen size of the tab content area
	int tabWidth2 = round(width - (sidebarWidth * uimult));
	int tabBottom = round(height - (bottombarHeight * uimult));

	// Load the settings menu and read user preferences file
	settings = new Settings("Settings", 0, tabWidth2, tabTop, tabBottom);
	settings.drawContent();

	// Recalculate now that UI scaling preference has been loaded
	tabWidth2 = round(width - (sidebarWidth * uimult));
	tabBottom = round(height - (bottombarHeight * uimult));

	// Define all the tabs here
	tabObjects.add(new SerialMonitor("Serial", 0, tabWidth2, tabTop, tabBottom));
	tabObjects.add(new LiveGraph("Live Graph", 0, tabWidth2, tabTop, tabBottom));
	tabObjects.add(new FileGraph("File Graph", 0, tabWidth2, tabTop, tabBottom));

	// Start serial port checking thread
	portList = Serial.list();
	thread("checkSerialPortList");
}


/**
 * Resize scaling of all UI elements
 *
 * @param  amount The quantity by which to change the scaling multiplier
 */
void uiResize(float amount) {
	// Resize UI scaler
	uimult += amount;

	uiResize();
}


/**
 * Resize scaling of all UI elements
 */
void uiResize() {

	// Resize fonts
	base_font = createFont(programFont, int(13*uimult), true);
	mono_font = createFont(terminalFont, int(14*uimult), true);
	tabTop = round(tabHeight * uimult);

	// Update sizing on all tabs
	for (TabAPI curTab : tabObjects) {
		curTab.changeSize(0, round(width - (sidebarWidth * uimult)), round(tabHeight * uimult), round(height - (bottombarHeight * uimult)));
	}

	// Settings menu
	settings.changeSize(0, round(width - (sidebarWidth * uimult)), round(tabHeight * uimult), round(height - (bottombarHeight * uimult)));

	// Redraw all content
	redrawUI = true;
	redrawContent = true;
}


/**
 * Dispose handler which is called when the "close"
 * window button is pressed. It makes sure that the
 * exit() function properly is called in this case.
 */
public class DisposeHandler {   
	DisposeHandler(PApplet pa) {
		pa.registerMethod("dispose", this);
	}
   
	public void dispose()
	{
		exit();
	}
}

/**
 * Function to properly exit the application
 */
public void exit() {
	for (TabAPI curTab : tabObjects) {
		curTab.performExit();
	}

	/*
	boolean safeToExit = true;
	for (TabAPI curTab : tabObjects) {
		safeToExit &= curTab.checkSafeExit();
		println(safeToExit);
	}

	if (!safeToExit) {
		println("Showing dialogue");
		myShowInputDialog("Are you sure you want to exit?", "There are still recordings/tasks running.","Nope");
		delay(5000);
	}*/

	programActive = false;
	println("Exiting program - exit()");
	exitActual();
}



/******************************************************//**
 * WINDOW DRAWING FUNCTIONS
 *
 * @info  Functions used to manage the drawing of all
 *        elements shown in the program window
 *********************************************************/


/**
 * Processing Draw Function
 *
 * This function manages all the screen drawing operations, 
 * and is looped at a frequency up to 60Hz (this will drop 
 * to a lower FPS under load). After the program has 
 * finished the setup process, this function will always 
 * call the drawProgram() method
 *
 * @see void drawProgram()
 */
void draw() {

	switch (state) {
		// Draw loading screen
		case 0: 
			drawLoadingScreen();
			if (millis() > startTime + 2000) state++;
			break;

		// Setup the program
		case 1: 
			setupProgram();
			state++;
			break;

		// Normal drawing operations of the program
		default:
			drawProgram();
			break;
	}
}


/**
 * Main Program Draw Function
 *
 * The function only updates the areas of the program window 
 * which need to be redrawn, as dictated by these boolean variables:
 *
 * @info redrawContent  - Draw the entire tab content area
 * @info drawNewData    - Only redraw parts affected by new serial data
 * @info redrawUI       - Draw the tab-bar, menu area and bottom status bar
 * @info drawFPS        - Draw a frame rate indicator at the top-right
 * @info redrawAlert    - Draw the alert message box
 * @info preventDrawing - Overrides above variables, preventing the screen
 *                        from being redrawn (eg. when window is too small)
 */
void drawProgram() {
	// Some logic to allow the screen to be updated behind alerts
	if (alertActive && (redrawContent || drawNewData || redrawUI)) {
		redrawContent = true;
		redrawUI = true;
		redrawAlert = true;
	}

	// If the window is resized, redraw all elements at the new size
	if ((lastWidth != width) || (lastHeight != height)){
		if (width < 400 || height < 300) {
			background(c_background);
			textAlign(CENTER, CENTER);
			text("Window Size too small !", width / 2, height / 2);
			lastWidth = width;
			lastHeight = height;
			preventDrawing = true;
		} else {
			redrawUI = true;
			redrawContent = true;
			preventDrawing = false;
			lastWidth = width;
			lastHeight = height;
			if (alertActive) redrawAlert = true;
			for (TabAPI curTab : tabObjects) {
				curTab.changeSize(0, round(width - (sidebarWidth * uimult)), round(tabHeight * uimult), round(height - (bottombarHeight * uimult)));
			}
			settings.changeSize(0, round(width - (sidebarWidth * uimult)), round(tabHeight * uimult), round(height - (bottombarHeight * uimult)));
		}
	}

	if (!preventDrawing) {
		// Redraw the content area elements
		if (redrawContent){
			if (tabObjects.size() > currentTab) {
				TabAPI curTab = tabObjects.get(currentTab);
				curTab.drawContent();
				redrawContent = false;
			} else currentTab = 0;
		}

		// Draw new data in the content area
		if (drawNewData) {
			if (tabObjects.size() > currentTab) {
				TabAPI curTab = tabObjects.get(currentTab);
				curTab.drawNewData();
				drawNewData = false;
			} else currentTab = 0;
		}
		
		// Redraw the UI elements (right and top bars)
		if (redrawUI){
			drawTabs(currentTab);
			drawSidebar();
			redrawUI = false;
		}

		// Draw an FPS indicator
		if (drawFPS) {
			rectMode(CORNERS);
			noStroke();
			textAlign(CENTER, CENTER);
			String frameRateText = "FPS: " + round(frameRate);
			fill(c_tabbar);
			rect(width - ((20 + sidebarWidth) * uimult) - textWidth(frameRateText), height - (bottombarHeight * uimult), width - (sidebarWidth * uimult), height);
			fill(c_idletab_text);
			text(frameRateText, width - ((20 + sidebarWidth) * uimult) - textWidth(frameRateText), height - (bottombarHeight * uimult), width - (sidebarWidth * uimult), height - 3);
			if (alertActive && !redrawAlert) {
				fill(c_white, 80);
				rect(width - ((20 + sidebarWidth) * uimult) - textWidth(frameRateText), height - (bottombarHeight * uimult), width - (sidebarWidth * uimult), height);
			}
		}

		// Redraw the alert message
		if(redrawAlert){
			drawAlert();
			redrawAlert = false;
		}
	}
}


/**
 * Draw the top Tab navigation bar
 *
 * @param  highlight The current active tab
 */
void drawTabs (int highlight) {

	// Setup drawing parameters
	rectMode(CORNER);
	noStroke();
	textAlign(CENTER, CENTER);

	// Tab Bar
	fill(c_tabbar);
	rect(0, 0, width, tabHeight * uimult);
	fill(c_tabbar_h);
	rect(0, (tabHeight - 1) * uimult, width, 1 * uimult);

	// Tab Buttons
	int i = 0;
	final int calcWidth = int((tabWidth - 1) * uimult);

	for(TabAPI curTab : tabObjects){
		int calcXpos = int(i * tabWidth * uimult);

		if(highlight == i){
			fill(c_background);
			rect(calcXpos, 0, calcWidth, tabHeight * uimult);
			fill(c_red);
			rect(calcXpos, 0, calcWidth, 4 * uimult);
			fill(c_tabbar_text);
		} else {
			fill(c_idletab);
			rect(calcXpos, 0, calcWidth, (tabHeight - 1) * uimult);
			fill(c_idletab_text);
		}

		text(curTab.getName(), calcXpos, 0, calcWidth, tabHeight * uimult);
		i++;
	}

	// Settings button
	if (!settingsMenuActive) {
		fill(c_idletab);
		rect(width - int(40 * uimult), 0, 40 * uimult, (tabHeight - 1) * uimult);
		fill(c_idletab_text);
		rect(width - int(28 * uimult), tabHeight * uimult * 2 / 6 - 1 * uimult, 10 * uimult, 1.5 * uimult);
		circle(width - int(14 * uimult), tabHeight * uimult * 2 / 6, 4 * uimult);
		rect(width - int(28 * uimult), tabHeight * uimult * 3 / 6 - 1 * uimult, 2 * uimult, 1.5 * uimult);
		circle(width - int(22 * uimult), tabHeight * uimult * 3 / 6, 4 * uimult);
		rect(width - int(18 * uimult), tabHeight * uimult * 3 / 6 - 1 * uimult, 6 * uimult, 1.5 * uimult);
		rect(width - int(28 * uimult), tabHeight * uimult * 4 / 6 - 1 * uimult, 10 * uimult, 1.5 * uimult);
		circle(width - int(14 * uimult), tabHeight * uimult * 4 / 6, 4 * uimult);
	} else {
		fill(c_background);
		rect(round(width - (sidebarWidth * uimult) + 1), 0, sidebarWidth * uimult, (tabHeight - 1) * uimult);
		fill(c_tabbar_text);
		//text("X", width - int(20 * uimult), (tabHeight - 1) / 2 * uimult);
		text("Settings", width - ((sidebarWidth + 20) * uimult) / 2, (tabHeight - 1) / 2 * uimult);
		fill(c_background);
		rect(width - int(40 * uimult), 0, 40 * uimult, (tabHeight - 1) * uimult);
		fill(c_tabbar_text);
		
		stroke(c_tabbar_text);
		strokeWeight(1.5 * uimult);
		line(width - int(25 * uimult), tabHeight * uimult / 3, width - int(15 * uimult), tabHeight * uimult * 2 / 3);
		line(width - int(15 * uimult), tabHeight * uimult / 3, width - int(25 * uimult), tabHeight * uimult * 2 / 3);
	}
}


/**
 * Draw the Side Bar and Bottom Bar
 *
 * This function draws the right-side menu area
 * for the current tab and the bottom status bar
 */
void drawSidebar () {

	// Setup drawing parameters
	rectMode(CORNER);
	noStroke();
	textAlign(CENTER, CENTER);

	// Calculate sizing of sidebar
	final int sT = round(tabHeight * uimult);
	final int sL = round(width - (sidebarWidth * uimult) + 1);
	final int sW = round(sidebarWidth * uimult);
	final int sH = height - sT;

	// Bottom info area
	fill(c_tabbar);
	rect(0, height - (bottombarHeight * uimult), width - sW + 1, bottombarHeight * uimult);
	fill(c_tabbar_h);
	rect(0, height - (bottombarHeight * uimult), width - sW + 1, 1 * uimult);

	// Sidebar
	fill(c_sidebar);
	rect(sL, sT, sW, sH);
	fill(c_sidebar_h);
	rect(sL - 1, sT, 1 * uimult, sH);
	
	// If settings menu is open, draw it
	if (settingsMenuActive) {
		settings.drawSidebar();
	// Otherwise draw the Tab-specific sidebar elements
	} else {
		if (tabObjects.size() > currentTab) {
			TabAPI curTab = tabObjects.get(currentTab);
			curTab.drawSidebar();
		} else currentTab = 0;
	}
}


/**
 * Draw the loading screen which is shown during start-up
 */
void drawLoadingScreen() {
	// Clear the background
	background(c_background);

	rectMode(CENTER);
	noStroke();
	fill(c_tabbar);
	rect(width / 2, height / 2 - int(15 * uimult), int(400 * uimult), int(340 * uimult));
	fill(c_tabbar_h);
	rect(width / 2, height / 2 - int(10 * uimult), int(400 * uimult), int(2 * uimult));

	// Set up text drawing parameters
	textAlign(CENTER, CENTER);
	textSize(int(20 * uimult));
	fill(c_tabbar_text);

	// Draw icon
	image(loadImage("icon-48.png"), (width / 2) - 24, (height / 2) - int(130 * uimult));

	// Draw text
	text("Processing Grapher", width / 2, (height / 2) - int(50 * uimult));
	textSize(int(14 * uimult));
	text("Loading v" + versionNumber, width / 2, (height / 2) + int(20 * uimult));
	fill(c_terminal_text);
	text("(C) Copyright 2018-2021 - Simon Bluett", width / 2, (height / 2) + int(60 * uimult));
	text("Free Software - GNU General Public License v3", width / 2, (height / 2) + int(90 * uimult));
}



/******************************************************//**
 * SIDEBAR MENU DRAWING FUNCTIONS
 *
 * @info  Functions used to draw the text, buttons and
 *        data-boxes on the sidebar menu of each tab
 *********************************************************/

/**
 * Draw sidebar text
 *
 * @param  text      The text to display
 * @param  textcolor The colour of the text
 * @param  lS        Left X-coordinate
 * @param  tS        Top Y-coordinate
 * @param  iW        Width of text box area
 * @param  tH        Height of text box area
 */
void drawText(String text, color textcolor, float lS, float tS, float iW, float tH) {
	if (tS >= tabTop && tS <= height) {
		textAlign(LEFT, CENTER);
		textFont(base_font);
		fill(textcolor);
		text(text, lS, tS, iW, tH);
	}
}


/**
 * Draw sidebar heading text
 *
 * @param  text The text to display
 * @param  lS   Left X-coordinate
 * @param  tS   Top Y-coordinate
 * @param  iW   Width of text box area
 * @param  tH   Height of text box area
 */
void drawHeading(String text, float lS, float tS, float iW, float tH){
	if (tS >= tabTop && tS <= height) {
		textAlign(CENTER, CENTER);
		textFont(base_font);
		fill(c_sidebar_heading);
		text(text, lS, tS, iW, tH);
	}
}


/**
 * Draw a sidebar button (overload function)
 *
 * @see drawButton(String, color, color, float, float, float, float, float)
 */
void drawButton(String text, color boxcolor, float lS, float tS, float iW, float iH, float tH){
	drawButton(text, c_sidebar_text, boxcolor, lS, tS, iW, iH, tH);
}


/**
 * Draw a sidebar button
 *
 * @param  text      The text to display on the button
 * @param  textcolor The colour of the text
 * @param  boxcolor  Background fill colour of the button
 * @param  lS        Top-left X-coordinate of the button
 * @param  tS        Top-left Y-coordinate of the button
 * @param  iW        Width of the button
 * @param  iH        Height of the button
 * @param  tH        Height of the text area on the button
 */
void drawButton(String text, color textcolor, color boxcolor, float lS, float tS, float iW, float iH, float tH){
	if (tS >= tabTop && tS <= height) {
		rectMode(CORNER);
		noStroke();
		textAlign(CENTER, CENTER);
		textFont(base_font);
		fill(boxcolor);
		rect(lS, tS, iW, iH);
		fill(textcolor);
		text(text, lS, tS, iW, tH);
	}
}


/**
 * Draw a sidebar databox (overload function)
 *
 * @see drawDatabox(String, color, float, float, float, float, float)
 */
void drawDatabox(String text, float lS, float tS, float iW, float iH, float tH){
	drawDatabox(text, c_sidebar_text, lS, tS, iW, iH, tH);
}


/**
 * Draw a sidebar databox - this is a button with no fill but just an outline
 *
 * @param  text      The text to display on the button
 * @param  textcolor The colour of the text
 * @param  lS        Top-left X-coordinate of the button
 * @param  tS        Top-left Y-coordinate of the button
 * @param  iW        Width of the button
 * @param  iH        Height of the button
 * @param  tH        Height of the text area on the button
 */
void drawDatabox(String text, color textcolor, float lS, float tS, float iW, float iH, float tH){
	if (tS >= tabTop && tS <= height) {
		rectMode(CORNER);
		noStroke();
		textAlign(CENTER, CENTER);
		textFont(base_font);
		fill(c_sidebar_button);
		rect(lS, tS, iW, iH);
		fill(c_sidebar);
		rect(lS + (1 * uimult), tS + (1 * uimult), iW - (2 * uimult), iH - (2 * uimult));
		fill(textcolor);
		text(constrainString(text, iW - (10 * uimult)), lS, tS, iW, tH);
	}
}


/**
 * Draw a simple rectangle on the sidebar
 *
 * @param  boxcolor Background fill colour of the rectangle
 * @param  lS        Top-left X-coordinate of the button
 * @param  tS        Top-left Y-coordinate of the button
 * @param  iW        Width of the button
 * @param  iH        Height of the button
 */       
void drawRectangle(color boxcolor, float lS, float tS, float iW, float iH){
	if (tS >= tabTop && tS <= height) {
		rectMode(CORNER);
		noStroke();
		fill(boxcolor);
		rect(lS, tS, iW, iH);
	}
}


/**
 * Check in mouse clicked on a sidebar menu item
 *
 * @param  yPos    Mouse Y-coordinate
 * @param  topPos  Top Y-coordinate of menu area
 * @param  unitH   Height of a standard menu item unit
 * @param  itemH   Height of menu item
 * @param  n       Number of units the current item is from the top
 * @return True if mouse Y-coordinate is on the menu item, false otherwise
 */
boolean menuYclick(int yPos, int topPos, int unitH, int itemH, float n) {
	return ((yPos > topPos + (unitH * n)) && (yPos < topPos + (unitH * n) + itemH));
}


/**
 * Draw a message box on the screen
 *
 * @param  heading  Title text of the message box
 * @param  text     Array of strings to be displayed (each item is a new line)
 * @param  lS       Left X-coordinate of the display area
 * @param  rS       Right X-coordinate of the display area
 * @param  tS       Top Y-coordinate of the display area
 * @param  alert    Whether or not this is an alert message (reduce opacity of background)
 */ 

void drawMessageArea(String heading, String[] text, float lS, float rS, float tS, boolean alert) {
	// Setup drawing parameters
	rectMode(CORNER);
	noStroke();
	textAlign(CENTER, CENTER);
	textFont(base_font);

	// Get text width
	final int border = int(uimult * 15);

	// Approximate how many rows of text are needed
	int boxHeight = int(30 * uimult) + 2 * border;
	int boxWidth = int(rS - lS);
	int[] itemHeight = new int[text.length];
	int largestWidth = 0;

	for (int i = 0; i < text.length; i++) {
		itemHeight[i] = int(22 * uimult);
		boxHeight += int(22 * uimult);
		int textW = int(textWidth(text[i]));

		if ((textW + 2 * border > largestWidth) && (textW + 2 * border < boxWidth)) largestWidth = int(textW + 2 * border + 2 * uimult);
		else if (textW + 2 * border > boxWidth) {
			largestWidth = boxWidth;
			boxHeight += int(22 * uimult * (ceil(textW / (boxWidth - 2 * border))));
			itemHeight[i] += int(22 * uimult * (ceil(textW / (boxWidth - 2 * border))));
		}
	}

	boxWidth = largestWidth;
	int verticalSum = int(tS + border + 25 * uimult);

	// Slightly lighten the background content
	if (alert) {
		fill(c_white, 80);
		rect(0, 0, width, height);
		if (boxWidth < alertWidth * uimult) boxWidth = int(alertWidth * uimult);
		if (boxHeight < alertHeight * uimult) {
			verticalSum += int(((alertHeight * uimult) - boxHeight) / 2.0);
			boxHeight = int(alertHeight * uimult);
		}
	}

	// Draw the box
	fill(c_message_box_outline);
	rect(int((lS + rS) / 2.0 - (boxWidth) / 2.0 - uimult * 2), tS - int(uimult * 2), boxWidth + int(uimult * 4), boxHeight + int(uimult * 4));
	if (alert) fill(c_alert_message_box);
	else fill(c_info_message_box);
	rect((lS + rS) / 2.0 - (boxWidth) / 2.0, tS, boxWidth, boxHeight);

	// Draw the text
	rectMode(CORNER);
	fill(c_sidebar_heading);
	text(heading, int((lS + rS) / 2.0 - boxWidth / 2.0 + border), int(tS + border), boxWidth - 2 * border, 20 * uimult);

	fill(c_sidebar_text);

	for (int i = 0; i < text.length; i++) {
		if (alert && i == text.length - 1) fill(c_lightgrey);
		text(text[i],  int((lS + rS) / 2.0 - (boxWidth) / 2.0 + border), verticalSum, boxWidth - 2 * border, itemHeight[i]);
		verticalSum += itemHeight[i];
	}
}


/**
 * Draw a message box on the screen (overload function)
 *
 * This is a simplified version of the function call
 * when the message is not an alert
 *
 * @see void drawMessageArea(String, String[], float, float, float, boolean)
 */
void drawMessageArea(String heading, String[] text, float lS, float rS, float tS) {
	drawMessageArea(heading, text, lS, rS, tS, false);
}


/**
 * Draw an Alert message box
 *
 * This is a special instance of the message box which
 * is drawn when there is an error/notification message
 * which needs to be shown to the user
 *
 * @see void drawMessageArea(String, String[], float, float, float, boolean)
 */
void drawAlert () {
	alertActive = true;

	String heading = "Info Message";
	String[] messages = split(alertHeading, '\n');

	if (messages.length > 1) {
		heading = messages[0];
		messages = remove(messages, 0);
	}

	messages = append(messages, "");
	messages = append(messages, "[Click to dismiss]");
	
	drawMessageArea(heading, messages, 50 * uimult, width - 50 * uimult, (height / 2.5) - (alertHeight * uimult / 2), true);
}


/**
 * Set and show a new alert message
 *
 * @param  message The alert message text
 */
void alertMessage(String message) {
	if (message != null) {
		alertHeading = message;
		redrawAlert = true;
	}
}



/******************************************************//**
 * KEYBOARD AND MOUSE INTERACTION FUNCTIONS
 *
 * @info  Functions to manage user input and interaction
 *        using the keyboard or mouse
 *********************************************************/

/**
 * Mouse click event handler
 *
 * This function figures out which region of the
 * screen was clicked and passes the information 
 * on to the current tab
 */
void mousePressed(){ 
	if (!alertActive) {

		// If mouse is hovering over the content area
		if ((mouseX > 0) && (mouseX < int(width - (sidebarWidth * uimult))) 
			&& (mouseY > int(tabHeight * uimult)) && (mouseY < int(height - (bottombarHeight * uimult))))
		{
			if (tabObjects.size() > currentTab) {
				TabAPI curTab = tabObjects.get(currentTab);
				curTab.contentClick(mouseX, mouseY);
			} else currentTab = 0;
		} else cursor(ARROW);

		// If mouse is hovering over a tab button
		if ((mouseY > 0) && (mouseY < tabHeight*uimult)) {
			
			// Open settings menu
			if (!settingsMenuActive && mouseX > width - int(40 * uimult)) {
				settingsMenuActive = true;
				redrawUI = true;
			}

			// Close settings menu
			else if (settingsMenuActive && mouseX > width - int(sidebarWidth * uimult)) {
				settingsMenuActive = false;
				settings.drawNewData();
				redrawUI = true;
			}

			// Other tab buttons
			else {
				for (int i = 0; i < tabObjects.size(); i++) {
					if ((mouseX > i*tabWidth*uimult) && (mouseX < (i+1)*tabWidth*uimult)) {
						currentTab = i;
						redrawUI = redrawContent = true;
					}
				}
			}
		}

		// If mouse is hovering over the side bar
		else if ((mouseX > width - (sidebarWidth * uimult)) && (mouseX < width)) {
			thread("menuClickEvent");
		}

	// If an alert is active, any mouse click hides the notification
	} else {
		alertActive = false;
		redrawUI = true;
		redrawContent = true;
	}
}


/**
 * Thread to manages clicks on the menu
 *
 * This thread asynchronously deals with mouse
 * clicks on the right-hand menu. Running this 
 * in a separate thread allows the main functions
 * of the program to continue working even when
 * blocking user-input pop-up dialogs are used.
 */
void menuClickEvent() {
	if (settingsMenuActive) {
		settings.menuClick(mouseX, mouseY);
	} else {
		if (tabObjects.size() > currentTab) {
			TabAPI curTab = tabObjects.get(currentTab);
			curTab.menuClick(mouseX, mouseY);
		} else currentTab = 0;
	}
}


/**
 * Mouse release handler
 *
 * This is only used to track and figure out the
 * end of mouse drag and drop operations
 */
void mouseReleased() {
	if (scrollingActive) scrollingActive = false;
}


/**
 * Mouse Wheel Scroll Handler
 *
 * @param  event Details of the mouse-scroll event
 */
void mouseWheel(MouseEvent event) {
  int e = event.getCount();
  
  if (abs(e) > 0) {
	
	// If mouse is hovering over the content area
	if ((mouseX > 0) && (mouseX < round(width - (sidebarWidth * uimult))) 
		&& (mouseY > round(tabHeight * uimult)) && (mouseY < round(height - (bottombarHeight * uimult))))
	{
		if (tabObjects.size() > currentTab) {
			TabAPI curTab = tabObjects.get(currentTab);
			curTab.scrollWheel(e);
		} else currentTab = 0;
	}

	// If mouse is hovering over the side bar
	if ((mouseX > width - (sidebarWidth * uimult)) && (mouseX < width)) {
		if (settingsMenuActive) {
			settings.scrollWheel(e);
		} else {
			if (tabObjects.size() > currentTab) {
				TabAPI curTab = tabObjects.get(currentTab);
				curTab.scrollWheel(e);
			} else currentTab = 0;
		}
	}
  }
}


/**
 * Thread to manage scrollbar drag operations
 *
 * This function is not currently in use
 */
void scrollBarEvent() {
	/*
	while (scrollingActive) {
		TabAPI curTab = tabObjects.get(currentTab);
		curTab.scrollBarUpdate(mouseX, mouseY);
		delay(20);
	}
	*/
}


/**
 * Keyboard Button Typed Handler
 * 
 * This function returns non-coded ASCII keys
 * which were typed. It does not track modifier
 * keys such as SHIFT and CONTROL
 */
void keyTyped() {
	if (!controlKey) {
		if (settingsMenuActive && (mouseX >= width - (sidebarWidth * uimult))) {
			settings.keyboardInput(key, (keyCode == 0)? key: keyCode, false);
		} else {
			if (tabObjects.size() > currentTab) {
				TabAPI curTab = tabObjects.get(currentTab);
				curTab.keyboardInput(key, (keyCode == 0)? key: keyCode, false);
			} else currentTab = 0;
		}
	}
}


/**
 * Keyboard Button Pressed Handler
 *
 * This function deals with button presses for
 * all coded keys, which are not handled by the
 * keyTyped() function
 */
void keyPressed() {
	
	// Check for control key
	if (key == CODED && keyCode == CONTROL) {
		controlKey = true;

	// Decrease UI scaling (CTRL and -)
	} else if (controlKey && (key == '-' || key == '_' || keyCode == 45)) {
		if (uimult > 0.5) uiResize(-0.1);
	// Increase UI scaling (CTRL and +)
	} else if (controlKey && (key == '=' || key == '+' || keyCode == 61)) {
		if (uimult < 2.0) uiResize(0.1);

	// For all other keys, send them on to the active tab
	} else if (key == CODED) {
		if (settingsMenuActive && (mouseX >= width - (sidebarWidth * uimult))) {
			settings.keyboardInput(key, keyCode, true);
		} else {
			if (tabObjects.size() > currentTab) {
				TabAPI curTab = tabObjects.get(currentTab);
				curTab.keyboardInput(key, keyCode, true);
			} else currentTab = 0;
		}
	}
	
	// Prevent the escape key from closing the application
	if (key == ESC) {
		key = 0;
		if (alertActive) {
			alertActive = false;
			redrawUI = true;
			redrawContent = true;
		}
	}
}


/**
 * Keyboard Button Release Handler
 *
 * This is mainly used for tracking the use
 * of key combinations using the Control key
 */
void keyReleased() {
	if (key == CODED && keyCode == CONTROL) {
		controlKey = false;
	}
}



/******************************************************//**
 * SERIAL COMMUNICATION FUNCTIONS
 *
 * @info  Functions to manage the serial communications
 *        with UART devices and micro-controllers
 *********************************************************/

/**
 * Setup Serial Communication
 *
 * This function manages the serial device 
 * connection and disconnection process 
 */
void setupSerial () {

	if (!serialConnected) {

		// Get a list of the available serial ports
		String[] ports = Serial.list();

		// If no ports are available
		if(ports.length == 0) {

			alertMessage("Error\nNo serial ports available");

		// If the port number we want to use is not in the list
		} else if((portNumber < 0) || (ports.length <= portNumber)) {

			alertMessage("Error\nInvalid port number selected");

		// Try to connet to the serial port
		} else {
			try {
				// Connect to the port
				myPort = new Serial(this, Serial.list()[portNumber], baudRate, serialParity, serialDatabits, serialStopbits);
				currentPort = Serial.list()[portNumber];

				// Trigger serial event once a line-ending is reached in the buffer
				if (lineEnding != 0) {
					myPort.bufferUntil(lineEnding);
				// Else if no line-ending is set, trigger after any byte is received
				} else {
					myPort.buffer(1);
				}

				serialConnected = true;

				// Tell all the tabs that a serial device has connected
				for (TabAPI curTab : tabObjects) {
					curTab.connectionEvent(true);
				}

				redrawUI = true;
			} catch (Exception e){
				alertMessage("Error\nUnable to connect to the port:\n" + e);
			}
		}

	// Disconnect from serial port
	} else {
		myPort.clear();
		myPort.stop();
		currentPort = "";
		serialConnected = false;

		// Tell all the tabs that a device has disconnected
		for (TabAPI curTab : tabObjects) {
			curTab.connectionEvent(false);
		}

		redrawUI = true;
	}
}


/**
 * Receive Serial Message Handler
 *
 * @param  myPort The selected serial COMs port
 */

void serialEvent (Serial myPort) {
	try {
		String inString;
		if (lineEnding != 0) {
			inString = myPort.readStringUntil(lineEnding);
		} else {
			inString = myPort.readString();
		}

		inString = trim(inString);

		// Remove line ending characters... is this needed?
		//inString = inString.replace("\n", "");
		//inString = inString.replace("\r", "");

		// Check if data is graphable
		boolean graphable = numberMessage(inString);

		// Send the data over to all the tabs
		for (TabAPI curTab : tabObjects) {
			curTab.parsePortData(inString, graphable);
		}

	} catch (Exception e) {
		alertMessage("Error\nUnable to read data from serial port:\n" + e);
	}
}


/**
 * Send Serial Message
 *
 * @param  message The message to be sent
 */
void serialSend (String message) {
	if (serialConnected) {
		try {
			myPort.write(message + lineEnding);
		} catch (Exception e) {
			alertMessage("Error\nUnable to write to the serial port:\n" + e);
		}
	}
}


/**
 * Serial Message Pop-up Dialog
 *
 * Pop-up dialog to allow user to send a serial
 * message from the live graph tab
 */
void serialSendDialog() {
	final String message = showInputDialog("Serial Message:");
	if (message != null){
		serialSend(message);
	}
}


/**
 * Serial Port List Update Thread
 *
 * This thread is started in the setup() method
 * and checks the serial list every 1 second
 * to see if there are any changes which need
 * to be taken into account
 */
void checkSerialPortList() {
	while (programActive) {
		try {
			boolean different = false;
			boolean currentPortExists = false;

			String[] currentList = Serial.list();
			if (currentList.length != portList.length) different = true;

			for (int i = 0; i < currentList.length; i++) {
				if (i < portList.length) {
					if (!currentList[i].equals(portList[i])) different = true;
				}

				if (currentPort.equals(currentList[i])) {
					currentPortExists = true;
					portNumber = i;
				}
			}

			if (serialConnected && !currentPortExists) {
				setupSerial();
				alertMessage("Error\nThe serial port has been disconnected");
			}

			if (different) {
				redrawUI = true;
			}

			portList = currentList;

		} catch (Exception e) {
			println("Error in checkSerialPortList: " + e);
		}

		delay(1000);
	}
}



/******************************************************//**
 * USER INPUT AND FILE SELECTION FUNCTIONS
 *
 * @info  Functions to deal with the display and callback
 *        of user input and file selection pop-up dialogs
 *********************************************************/


/**
 * Show a pop-up user input dialogue
 *
 * @param  heading     The heading text
 * @param  message     The message text
 * @param  defaultText Default value to show in the input area
 * @return The user input data
 */
String myShowInputDialog(final String heading, final String message, final String defaultText) {
	if (activeRenderer == FX2D) {
		userInputString = "";
		Platform.runLater(new Runnable() {
			@Override
			public void run() {
				userInputString = trim(FxDialogs.showTextInput(heading, message, defaultText));
			}
		});

		// Wait for the user response
		while (programActive && userInputString == "" && userInputString != null) {
			delay(200);
		}

		return userInputString;
		
	} else {
		return trim(showInputDialog(heading + "\n" + message, defaultText));
	}
}


/**
 * Get the File Selected in the Input/Output Dialogue
 *
 * @param  selection The selected file path
 */
 void fileSelected(File selection) {

	// If a file was actually selected
	if (selection != null) {

		// Send it over to the tabs that require it
		for (TabAPI curTab : tabObjects) {
			if(curTab.getOutput() == "") {
				// Get absolute path of file and convert backslashes into normal slashes
				String newFile = join(split(selection.getAbsolutePath(), '\\'), "/");
				curTab.setOutput(newFile);
			}
		}

	} else {
		for (TabAPI curTab : tabObjects) {
			if(curTab.getOutput() == "") {
				curTab.setOutput("No File Set");
			}
		}
	}
	redrawUI = true;
}


/**
 * Override the Processing "selectOutput" function
 * 
 * This function opens the Save File Dialogue, overriding
 * the default behaviour to use the native JavaFX file dialogue
 * when using the FX2D renderer.
 *
 * @param  message        The title of the Save File Dialogue
 * @param  callbackMethod Function to call when dialogue is submitted
 */
@Override
void selectOutput(final String message, final String callbackMethod) {
	if (activeRenderer == FX2D) {
		fileChooser.setTitle(message);

		fileChooser.getExtensionFilters().clear();
		if (message.contains("CSV")) {
			fileChooser.getExtensionFilters().add(new FileChooser.ExtensionFilter("Comma Separated", "*.csv"));
		} else if (message.contains("TXT")) {
			fileChooser.getExtensionFilters().add(new FileChooser.ExtensionFilter("Text File", "*.txt"));
		}

		Platform.runLater(new Runnable() {
			@Override
			public void run() {
				File file = fileChooser.showSaveDialog(stage); 
				mySelectCallback(file, callbackMethod);
			}
		});
	} else {
		selectOutput(message, callbackMethod, null);
	}
}


/**
 * Override the Processing "selectInput" function
 * 
 * This function opens the Open File Dialogue, overriding
 * the default behaviour to use the native JavaFX file dialogue
 * when using the FX2D renderer.
 *
 * @param  message        The title of the Open File Dialogue
 * @param  callbackMethod Function to call when dialogue is submitted
 */
@Override
void selectInput(final String message, final String callbackMethod) {
	if (activeRenderer == FX2D) {
		fileChooser.setTitle(message);
		
		fileChooser.getExtensionFilters().clear();
		if (message.contains("CSV")) {
			fileChooser.getExtensionFilters().add(new FileChooser.ExtensionFilter("Comma Separated", "*.csv"));
		} else if (message.contains("TXT")) {
			fileChooser.getExtensionFilters().add(new FileChooser.ExtensionFilter("Text File", "*.txt"));
		}

		Platform.runLater(new Runnable() {
			@Override
			public void run() {
				File file = fileChooser.showOpenDialog(stage); 
				mySelectCallback(file, callbackMethod);
			}
		});
	} else {
		selectInput(message, callbackMethod, null);
	}
}


/**
 * Function used to run callback when a file selection dialogue is submitted
 *
 * A copy of the protected method "selectCallback" in the Processing Core
 *
 * @param  selectedFile   The file selected in the dialogue
 * @param  callbackMethod Name of the function to be called
 */ 
void mySelectCallback(File selectedFile, String callbackMethod) {
    try {
      Class<?> callbackClass = this.getClass();
      Method selectMethod = callbackClass.getMethod(callbackMethod, new Class[] { File.class });
      selectMethod.invoke(this, new Object[] { selectedFile });

    } catch (IllegalAccessException iae) {
      System.err.println(callbackMethod + "() must be public");

    } catch (InvocationTargetException ite) {
      ite.printStackTrace();

    } catch (NoSuchMethodException nsme) {
      System.err.println(callbackMethod + "() could not be found");
    }
}


public class ValidateInput {
	private String inputString;
	private String errorMessage;
	private int intValue;
	private double doubleValue;

	static public final int NONE = 0;
	static public final int GT = 1;
	static public final int GTE = 2;
	static public final int LT = 3;
	static public final int LTE = 4;

	public ValidateInput(final String heading, final String message, final String defaultText) {
		inputString = myShowInputDialog(heading, message, defaultText);
	}

	public void setErrorMessage(String error) {
		errorMessage = error;
	}

	public boolean checkFloat() {
		return checkDouble(NONE, 0, NONE, 0);
	}

	public boolean checkFloat(int operator1, float value1) {
		return checkDouble(operator1, value1, NONE, 0);
	}

	public boolean checkFloat(int operator1, float value1, int operator2, float value2) {
		return checkDouble(operator1, value1, operator2, value2);
	}

	public boolean checkDouble() {
		return checkDouble(NONE, 0, NONE, 0);
	}

	public boolean checkDouble(int operator1, double value1) {
		return checkDouble(operator1, value1, NONE, 0);
	}

	public boolean checkDouble(int operator1, double value1, int operator2, double value2) {
		if (inputString != null) {
			try {
				doubleValue = Double.parseDouble(inputString);
				if (!doubleConstraint(operator1, value1) || !doubleConstraint(operator2, value2)) {
					alertMessage(errorMessage);
					return false;
				}
				return true;
			} catch (Exception e) {
				alertMessage(errorMessage);
				return false;
			}
		}
		return false;
	}

	public boolean checkInt() {
		return checkInt(NONE, 0, NONE, 0);
	}

	public boolean checkInt(int operator1, int value1) {
		return checkInt(operator1, value1, NONE, 0);
	}

	public boolean checkInt(int operator1, int value1, int operator2, int value2) {
		if (inputString != null) {
			try {
				intValue = Integer.parseInt(inputString);
				if (!intConstraint(operator1, value1) || !intConstraint(operator2, value2)) {
					alertMessage(errorMessage);
					return false;
				}
				return true;
			} catch (Exception e) {
				alertMessage(errorMessage);
				return false;
			}
		}
		return false;
	}


	private boolean doubleConstraint(int operators, double value1) {
		switch (operators) {
			case GT:
				if (doubleValue <= value1) return false;
				break;
			case GTE:
				if (doubleValue < value1) return false;
				break;
			case LT:
				if (doubleValue >= value1) return false;
				break;
			case LTE:
				if (doubleValue > value1) return false;
				break;
		}
		return true;
	}

	private boolean intConstraint(int operators, int value1) {
		switch (operators) {
			case GT:
				if (intValue <= value1) return false;
				break;
			case GTE:
				if (intValue < value1) return false;
				break;
			case LT:
				if (intValue >= value1) return false;
				break;
			case LTE:
				if (intValue > value1) return false;
				break;
		}
		return true;
	}

	public double getDouble() {
		return doubleValue;
	}

	public float getFloat() {
		return (float) doubleValue;
	}

	public int getInt() {
		return intValue;
	}
}


/******************************************************//**
 * UTILITY AND DATA OPERATION FUNCTIONS
 *
 * @info  Functions used to perform common data
 *        manipulation operations
 *********************************************************/

/**
 * Ceil number up to 'n' significant figure (overload function)
 *
 * @see double ceilToSigFig(double, int)
 */
float ceilToSigFig(float num, int n) {
	return (float) ceilToSigFig((double) num, n);
}


/**
 * Ceil number up to 'n' significant figure
 *
 * @param  num The number to be rounded
 * @param  n   The number of significant figures to keep
 * @return The number rounded up to 'n' significant figures
 */
double ceilToSigFig(double num, int n) {
	if(num == 0) {
		return 0;
	}

	final double d = Math.ceil(Math.log10(num < 0 ? -num: num));
	final int power = n - (int) d;

	final double magnitude = Math.pow(10, power);
	final long shifted = (long) Math.ceil(num*magnitude);
	return shifted/magnitude;
}


/**
 * Floor number down to 'n' significant figure (overload function)
 *
 * @see double floorToSigFig(double, int)
 */
float floorToSigFig(float num, int n) {
	return (float) floorToSigFig((double) num, n);
}


/**
 * Floor number down to 'n' significant figure
 *
 * @param  num The number to be rounded
 * @param  n   The number of significant figures to keep
 * @return The number rounded down to 'n' significant figures
 */
double floorToSigFig(double num, int n) {
	if(num == 0) {
		return 0;
	}

	final double d = Math.ceil(Math.log10(num < 0 ? -num: num));
	final int power = n - (int) d;

	final double magnitude = Math.pow(10, power);
	final long shifted = (long) Math.floor(num*magnitude);
	return shifted/magnitude;
}


/**
 * Round number to 'n' significant figures (overload function)
 *
 * @see double roundToSigFig(double, int)
 */
float roundToSigFig(float num, int n) {
	return (float) roundToSigFig((double) num, n);
}


/**
 * Round number up/down to 'n' significant figure
 *
 * @param  num The number to be rounded
 * @param  n   The number of significant figures to keep
 * @return The number rounded up/down to 'n' significant figures
 */
double roundToSigFig(double num, int n) {
	if(num == 0) {
		return 0;
	}

	final double d = Math.ceil(Math.log10(num < 0 ? -num: num));
	final int power = n - (int) d;

	final double magnitude = Math.pow(10, power);
	final long shifted = Math.round(num*magnitude);
	return shifted/magnitude;
}


/**
 * Remove an element from a string array
 *
 * @param  a     The String array
 * @param  index The index of the String to be removed
 * @return The string with the specified item removed
 */
String[] remove(String[] a, int index){
	// Move the specified item to the end of the array
	for (int i = index + 1; i < a.length; i++) {
		a[i-1] = a[i];
	}

	// Remove the last item from the array
	return shorten(a);
}


/**
 * Test whether a character is a number/digit
 *
 * @param  c The character to be tested
 * @return True if the character is a number
 */
//boolean charIsNum(char c) {
//	return 48 <= c && c <= 57;
//}


/**
 * Test whether a String follows correct format to be displayed on live graph
 *
 * @param  msg The string to be tested
 * @return True if the string doesn't contain any invalid characters
 */
boolean numberMessage(String msg) {
	for (int i = 0; i < msg.length() - 1; i++) {
		final char j = msg.charAt(i);
		if ((j < 43 && j != ' ') || j > 57 || j == 47) {
			return false;
		}
	}
	return true;
}


/**
 * Constrain text length to fit within a certain width
 *
 * The function removes characters from the front of the string
 * until it fits into the designated width
 *
 * @param  inputText The text to be constrained
 * @param  maxWidth  The maximum width in pixels of the text
 * @return The shortened string
 */
String constrainString(String inputText, float maxWidth) {
	if (textWidth(inputText) > maxWidth) {
		while (textWidth(".." + inputText) > maxWidth && inputText.length() > 1) {
			inputText = inputText.substring(1, inputText.length());
		}
		inputText = ".." + inputText;
	}
	return inputText;
}


/**************************************************************************************//**
 * Abstracted TAB API Interface
 *
 * These are functions which each "Tab" in the GUI
 * need to have. This makes it easier to add new tabs
 * later on which use the same interface and serial
 * features available from the core program.
 ******************************************************************************************/
interface TabAPI {
	// Name of the tab
	String getName();
	
	// Draw functions
	void drawContent();
	void drawNewData();
	void drawSidebar();
	
	// Mouse clicks
	void menuClick (int xcoord, int ycoord);
	void contentClick (int xcoord, int ycoord);
	void scrollWheel(float e);
	void scrollBarUpdate(int xcoord, int ycoord);

	// Keyboard input
	void keyboardInput(char keyChar, int keyCodeInt, boolean codedKey);
	
	// Change content area size
	void changeSize(int newL, int newR, int newT, int newB);
	
	// Getting new file paths
	String getOutput();
	void setOutput(String newoutput);
	
	// Serial communication
	void connectionEvent(boolean status);
	void parsePortData(String inputData, boolean graphable);

	// Exit function
	boolean checkSafeExit();
	void performExit();
}

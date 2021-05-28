/* * * * * * * * * * * * * * * * * * * * * * *
 * LIVE GRAPH PLOTTER CLASS
 * implements TabAPI for Processing Grapher
 *
 * @file     LiveGraph.pde
 * @brief    Real-time serial data plotter tab
 * @author   Simon Bluett
 *
 * @license  GNU General Public License v3
 * @class    LiveGraph
 * @see      TabAPI <ProcessingGrapher.pde>
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

class LiveGraph implements TabAPI {

  int cL, cR, cT, cB;     // Content coordinates (left, right, top bottom)
  Graph graphA, graphB, graphC, graphD;
  int menuScroll;
  int menuHeight;
  String name;

  String outputfile;

  String[] dataColumns = {};
  int[] graphAssignment = {};
  int graphMode;
  CustomTable dataTable;
  boolean recordData;
  int recordCounter;
  int fileCounter;
  int maxFileRows = 100000;
  int drawFrom;
  float xRate=50;
  int selectedGraph;
  boolean autoAxis;
  int maxSamples;
  int[] sampleWindow = {1000, 1000, 1000, 1000};
  int signalListChange;


  /**
   	 * Constructor
   	 *
   	 * @param  setname Name of the tab
   	 * @param  left    Tab area left x-coordinate
   	 * @param  right   Tab area right x-coordinate
   	 * @param  top     Tab area top y-coordinate
   	 * @param  bottom  Tab area bottom y-coordinate
   	 */
  LiveGraph (String setname, int left, int right, int top, int bottom) {
    name = setname;

    cL = left;
    cR = right;
    cT = top;
    cB = bottom;

    graphA = new Graph(cL, cR, cT, cB, 0, 10, 0, 1, "Air Pressure Left Lung (cmH20)");
    graphB = new Graph(cL, cR, (cT + cB) / 2, cB, 0, 10, 0, 1, "Air Flow Left Lung (ml/s)");
    graphC = new Graph((cL + cR) / 2, cR, cT, (cT + cB) / 2, 0, 10, 0, 1, "Air Pressure Right Lung (cmH20)");
    graphD = new Graph((cL + cR) / 2, cR, (cT + cB) / 2, cB, 0, 10, 0, 1, "Air Flow Right Lung (ml/s)");
    graphA.setHighlight(true);
    graphA.setXaxisName("Time (s)");
    graphB.setXaxisName("Time (s)");
    graphC.setXaxisName("Time (s)");
    graphD.setXaxisName("Time (s)");
    
    //variables
    int time=7;
    int pressureMax=40;
    int pressureMin=-10;
    int airflowMax = 75;
    int airflowMin = -75;
    
    //graph A manual
    graphA.setMinY(pressureMin);  
    graphA.setMaxY(pressureMax);
    graphA.setMaxX(time);


    //graph B manual
    graphB.setMinY(airflowMin);  
    graphB.setMaxY(airflowMax); 
    graphB.setMaxX(time);
    
    //graph C manual
    graphC.setMinY(pressureMin);  
    graphC.setMaxY(pressureMax);
    graphC.setMaxX(time);
    
    //graph D manual
    graphD.setMinY(airflowMin);  
    graphD.setMaxY(airflowMax); 
    graphD.setMaxX(time);

    graphMode = 1;
    selectedGraph = 1;

    outputfile = "No File Set";
    recordData = false;
    recordCounter = 0;
    fileCounter = 0;

    xRate = 50;
    autoAxis = false;

    drawFrom = 0;
    maxSamples = 10;
    signalListChange = 0;

    dataTable = new CustomTable();

    menuScroll = 0;
    menuHeight = cB - cT - 1;
  }


  /**
   	 * Get the name of the current tab
   	 *
   	 * @return Tab name
   	 */
  String getName () {
    return name;
  }


  /**
   	 * Redraw all tab content
   	 */
  void drawContent () {
    graphA.drawGrid();
    graphA.resetGraph();
    if (graphMode > 1) {
      graphB.drawGrid();
      graphB.resetGraph();
    }
    if (graphMode > 2) {
      graphC.drawGrid();
      graphC.resetGraph();
    }
    if (graphMode > 3) {
      graphD.drawGrid();
      graphD.resetGraph();
    }

    // Show message if no serial device is connected
    if (!serialConnected) {
      if (showInstructions) {
        String[] message = {"1. In the 'Serial' tab, use the right-hand menu to connect to a serial device", 
          "2. Each line sent by the device should contain only numbers separated with commas", 
          "3. The signals/numbers can be displayed in real-time on up to 4 separate graphs"};
        drawMessageArea("Getting Started", message, cL + 60 * uimult, cR - 60 * uimult, cT + 30 * uimult);
      }
    } else if (dataTable.getRowCount() > 0) {
      drawNewData();
    }
  }


  /**
   	 * Draw new tab data
   	 */
  void drawNewData () {
    int currentCount = dataTable.getRowCount();

    // If there is content to draw
    if (currentCount > 0) {

      int samplesA = currentCount - sampleWindow[0] - 1;
      int samplesB = currentCount - sampleWindow[1] - 1;
      int samplesC = currentCount - sampleWindow[2] - 1;
      int samplesD = currentCount - sampleWindow[3] - 1;

      drawFrom = samplesA;
      graphA.clearGraph();
      if (graphMode >= 2) {
        graphB.clearGraph();
        if (samplesB < drawFrom) drawFrom = samplesB;
      }
      if (graphMode >= 3) {
        graphC.clearGraph();
        if (samplesC < drawFrom) drawFrom = samplesC;
      }
      if (graphMode >= 4) {
        graphD.clearGraph();
        if (samplesD < drawFrom) drawFrom = samplesD;
      }

      maxSamples = currentCount - drawFrom;
      if (drawFrom < 0) drawFrom = 0;

      for (int j = drawFrom; j < currentCount; j++) {
        for (int i = 0; i < dataTable.getColumnCount(); i++) {
          try {
            float dataPoint = (float) dataTable.getDouble(j, i);
            if (dataPoint != dataPoint) dataPoint = 99999999;
            if (graphAssignment[i] == 2 && graphMode >= 2 && samplesB <= drawFrom) {
              checkGraphSize(dataPoint, graphB);
              graphB.plotData(dataPoint, i);
            } else if (graphAssignment[i] == 3 && graphMode >= 3 && samplesC <= drawFrom) {
              checkGraphSize(dataPoint, graphC);
              graphC.plotData(dataPoint, i);
            } else if (graphAssignment[i] == 4 && graphMode >= 4 && samplesD <= drawFrom) {
              checkGraphSize(dataPoint, graphD);
              graphD.plotData(dataPoint, i);
            } else if (graphAssignment[i] == 1 && samplesA <= drawFrom) {
              checkGraphSize(dataPoint, graphA);
              graphA.plotData(dataPoint, i);
              println("Plotting: " + drawFrom + ", " + dataPoint);
            }
          } 
          catch (Exception e) {
            println("LiveGraph::drawNewData() - drawFrom: " + drawFrom + ", currentCount: " + currentCount + ", Error: " + e);
          }
        }
        drawFrom++;
      }
    }
  }


  /**
   	 * Resize graph y-axis if data point is out of bounds
   	 *
   	 * @param  dataPoint   Y-coordinate of new data point
   	 * @param  graphSelect Which if the 4 graphs to check
   	 */
  void checkGraphSize (float dataPoint, Graph currentGraph) {

    // If data exceeds graph size, resize the graph
    if (autoAxis && dataPoint !=  99999999) {

      if (dataPoint < currentGraph.getMinY()) {
        currentGraph.setMinY(floorToSigFig(dataPoint, 1));
        currentGraph.drawGrid();
        redrawUI = true;
      } else if (dataPoint > currentGraph.getMaxY()) {
        currentGraph.setMaxY(ceilToSigFig(dataPoint, 1));
        currentGraph.drawGrid();
        redrawUI = true;
      }
    }
  }


  /**
   	 * Change tab content area dimensions
   	 *
   	 * @param  newL New left x-coordinate
   	 * @param  newR New right x-coordinate
   	 * @param  newT New top y-coordinate
   	 * @param  newB new bottom y-coordinate
   	 */
  void changeSize (int newL, int newR, int newT, int newB) {
    cL = newL;
    cR = newR;
    cT = newT;
    cB = newB;

    if (graphMode == 2) {
      graphA.changeSize(cL, cR, cT, (cT + cB) / 2);
      graphB.changeSize(cL, cR, (cT + cB) / 2, cB);
    } else if (graphMode == 3) {
      graphA.changeSize(cL, (cL + cR) / 2, cT, (cT + cB) / 2);
      graphB.changeSize(cL, cR, (cT + cB) / 2, cB);
      graphC.changeSize((cL + cR) / 2, cR, cT, (cT + cB) / 2);
    } else if (graphMode == 4) {
      graphA.changeSize(cL, (cL + cR) / 2, cT, (cT + cB) / 2);
      graphB.changeSize(cL, (cL + cR) / 2, (cT + cB) / 2, cB);
      graphC.changeSize((cL + cR) / 2, cR, cT, (cT + cB) / 2);
      graphD.changeSize((cL + cR) / 2, cR, (cT + cB) / 2, cB);
    } else {
      graphA.changeSize(cL, cR, cT, cB);
    }
    //drawContent();
  }


  /**
   	 * Change CSV data file location
   	 *
   	 * @param  newoutput Absolute path to the new file location
   	 */
  void setOutput (String newoutput) {
    if (newoutput != "No File Set") {
      // Ensure file type is *.csv
      int dotPos = newoutput.lastIndexOf(".");
      if (dotPos > 0) newoutput = newoutput.substring(0, dotPos);
      newoutput = newoutput + ".csv";

      // Test whether this file is actually accessible
      if (saveFile(newoutput) == null) {
        alertMessage("Error\nUnable to access the selected output file location; perhaps this location is write-protected?\n" + newoutput);
        newoutput = "No File Set";
      }
    }
    outputfile = newoutput;
  }


  /**
   	 * Get the current CSV data file location
   	 *
   	 * @return Absolute path to the data file
   	 */
  String getOutput () {
    return outputfile;
  }


  /** 
   	 * Start recording new serial data points to file
   	 */
  void startRecording () {
    // Ensure table is empty
    dataTable = new CustomTable();
    drawFrom = 0;

    // Add columns to the table
    while (dataTable.getColumnCount() < dataColumns.length) dataTable.addColumn(dataColumns[dataTable.getColumnCount()]);

    // Open up the CSV output stream
    if (!dataTable.openCSVoutput(outputfile)) {
      alertMessage("Error\nUnable to create the output file; perhaps the location no longer exists?\n" + outputfile);
    } else {
      recordCounter = 0;
      fileCounter = 0;
      recordData = true;
      redrawUI = true;
    }
  }


  /**
   	 * Stop recording data points to file
   	 */
  void stopRecording() {
    recordData = false;
    if (dataTable.closeCSVoutput()) {
      alertMessage("Success\nRecorded " + ((fileCounter * 10000) + recordCounter) + " samples to " + (fileCounter + 1) + " CSV file(s)");
    } else {
      emergencyOutputSave(false);
    }
    outputfile = "No File Set";
    redrawUI = true;
  }


  /**
   	 * Recover from an rrror when recording data to file
   	 *
   	 * @param  continueRecording If we want to continue recording after dealing with the error
   	 */
  void emergencyOutputSave(boolean continueRecording) {
    dataTable.closeCSVoutput();

    // Figure out name for new backup file
    String[] tempSplit = split(outputfile, '/');
    int dotPos = tempSplit[tempSplit.length - 1].lastIndexOf(".");
    String nextoutputfile = tempSplit[tempSplit.length - 1].substring(0, dotPos);
    outputfile = nextoutputfile + "-backup.csv";

    String emergencysavefile = nextoutputfile + "-backup-" + (fileCounter + 1) + ".csv";

    try {
      // Backup the existing data
      saveTable(dataTable, emergencysavefile);

      // If we want to continue recording, try setting up a new output file
      if (continueRecording) {
        fileCounter++;
        nextoutputfile = nextoutputfile + "-backup-" + (fileCounter + 1) + ".csv";

        // If new output file was successfully opened, only show a Warning message
        if (dataTable.openCSVoutput(nextoutputfile)) {
          alertMessage("Warning\nAn issue occurred when trying to save new data to the ouput file.\n1. A backup of all the data has been created\n2. Data is still being recorded (to a new file)\n3. The files are in the same directory as ProcessingGrapher.exe");

          // If not, show an error message that the recording has stopped
        } else {
          recordData = false;
          redrawUI = true;
          alertMessage("Error - Recording Stopped\nAn issue occurred when trying to save new data to the ouput file.\n1. A backup of all the data has been created\n2. The files are in the same directory as ProcessingGrapher.exe");
        }

        // If we don't want to continue, show a simple error message
      } else {
        recordData = false;
        alertMessage("Error\nAn issue occurred when trying to save new data to the ouput file.\n1. Data recording has been stopped\n2. A backup of all the data has been created\n3. The backup is in the same directory as ProcessingGrapher.exe");
      }

      // If something went wrong in the error recovery process, show a critical error message
    } 
    catch (Exception e) {
      dataTable.closeCSVoutput();
      recordData = false;
      alertMessage("Critical Error\nAn issue occurred when trying to save new data to the ouput file.\nData backup was also unsuccessful, so some data may have been lost...\n" + e);
    }
  }


  /**
   	 * Function called when a serial device has connected/disconnected
   	 *
   	 * @param  status True if a device has connected, false if disconnected
   	 */
  void connectionEvent (boolean status) {

    // If port has disconnected
    if (!status) {
      // Stop recording any data
      if (recordData) stopRecording();

      // Reset the signal list
      dataTable.clearRows();

      while (dataColumns.length > 0) {
        dataColumns = shorten(dataColumns);
        graphAssignment = shorten(graphAssignment);
      }
      while (dataTable.getColumnCount() > 0) dataTable.removeColumn(0);
      drawFrom = 0;
      redrawContent = true;
    }
  }


  /**
   	 * Parse new data points received from serial port
   	 *
   	 * @param  inputData String containing data points separated by commas
   	 * @param  graphable True if data in message can be plotted on a graph
   	 */
  void parsePortData (String inputData, boolean graphable) {

    // Check that the starts with a number
    if (graphable) {
      String[] dataArray = trim(split(inputData, ','));

      // If data column does not exist, add it to the list
      while (dataColumns.length < dataArray.length) {
        dataColumns = append(dataColumns, "Signal-" + (dataColumns.length + 1));
        graphAssignment = append(graphAssignment, 1);
        dataTable.addColumn("Signal-" + (dataColumns.length + 1), CustomTable.STRING);
        redrawUI = true;
      }

      // Only remove extra columns if not recording and
      // the last 10 input data samples didn't contain the signal
      if (dataColumns.length > dataArray.length) {
        signalListChange++;
        if (signalListChange >= 10 && !recordData) {
          dataColumns = shorten(dataColumns);
          graphAssignment = shorten(graphAssignment);
          dataTable.removeColumn(dataColumns.length);
          signalListChange = 0;
          redrawUI = true;
        }
      }

      // --- Data Recording ---
      TableRow newRow = dataTable.addRow();
      //float[] newData = new float[dataArray.length];

      // Go through each data column, and try to parse and add to file
      for (int i = 0; i < dataArray.length; i++) {
        try {
          double dataPoint = Double.parseDouble(dataArray[i]);
          newRow.setDouble(i, dataPoint);
          //newData[i] = dataPoint;
          //checkGraphSize(dataPoint, 0);
        } 
        catch (Exception e) {
          print(e);
          println(" - When parsing live graph data");
        }
      }

      //graphA.bufferNewData(newData);

      // Record data to file
      if (recordData) {
        recordCounter++;
        if (!dataTable.saveCSVentries(dataTable.lastRowIndex(), dataTable.lastRowIndex())) {
          emergencyOutputSave(true);
        }

        // Separate data into files once the max number of rows has been reached
        if (recordCounter >= maxFileRows) {
          dataTable.closeCSVoutput();
          fileCounter++;
          recordCounter = 0;

          int dotPos = outputfile.lastIndexOf(".");
          String nextoutputfile = outputfile.substring(0, dotPos);
          nextoutputfile = nextoutputfile + "-" + (fileCounter + 1) + ".csv";
          if (!dataTable.openCSVoutput(nextoutputfile)) {
            emergencyOutputSave(true);
          }

          // Ensure table is empty
          dataTable = new CustomTable();
          drawFrom = 0;
        }
      } else {
        // Remove rows from table which don't need to be shown on the graphs anymore
        while (dataTable.getRowCount() > maxSamples) {
          dataTable.removeRow(0);
          drawFrom--;
          if (drawFrom < 0) drawFrom = 0;
        }
      }

      drawNewData = true;
    }
  }


  /**
   	 * Draw the sidebar menu for the current tab
   	 */
  void drawSidebar () {

    // Calculate sizing of sidebar
    // Do this here so commands below are simplified
    int sT = cT;
    int sL = cR;
    int sW = width - cR;
    int sH = height - sT;

    int uH = round(sideItemHeight * uimult);
    int tH = round((sideItemHeight - 8) * uimult);
    int iH = round((sideItemHeight - 5) * uimult);
    int iL = round(sL + (10 * uimult));
    int iW = round(sW - (20 * uimult));
    menuHeight = round((12.5 + dataColumns.length + ((graphMode + 1) * 0.75)) * uH);

    // Figure out if scrolling of the menu is necessary
    if (menuHeight > sH) {
      if (menuScroll == -1) menuScroll = 0;
      else if (menuScroll > menuHeight - sH) menuScroll = menuHeight - sH;

      // Draw left bar
      fill(c_serial_message_box);
      rect(width - round(15 * uimult) / 2, sT, round(15 * uimult) / 2, sH);

      // Figure out size and position of scroll bar indicator
      int scrollbarSize = sH - round(sH * float(menuHeight - sH) / menuHeight);
      if (scrollbarSize < uH) scrollbarSize = uH;
      int scrollbarOffset = round((sH - scrollbarSize) * (menuScroll / float(menuHeight - sH)));
      fill(c_terminal_text);
      rect(width - round(15 * uimult) / 2, sT + scrollbarOffset, round(15 * uimult) / 2, scrollbarSize);

      sT -= menuScroll;
      sL -= round(15 * uimult) / 4;
      iL -= round(15 * uimult) / 4;
    } else {
      menuScroll = -1;
    }

    // Save to File
    drawHeading("Record Graph Data", iL, sT + (uH * 0), iW, tH);
    if (outputfile == "No File Set" || outputfile == "") {
      drawButton("Set Output File", c_sidebar_button, iL, sT + (uH * 1), iW, iH, tH);
      drawDatabox("Start Recording", c_idletab_text, iL, sT + (uH * 2), iW, iH, tH);
    } else {
      String[] fileParts = split(outputfile, '/');
      String fileName = fileParts[fileParts.length - 1];

      if (recordData) {
        drawDatabox(fileName, c_idletab_text, iL, sT + (uH * 1), iW, iH, tH);
        drawButton("Stop Recording", c_sidebar_accent, iL, sT + (uH * 2), iW, iH, tH);
      } else {
        drawDatabox(fileName, c_sidebar_text, iL, sT + (uH * 1), iW, iH, tH);
        drawButton("Start Recording", c_sidebar_button, iL, sT + (uH * 2), iW, iH, tH);
      }
    }

    // Graph options
    Graph currentGraph;
    if (selectedGraph == 2) currentGraph = graphB;
    else if (selectedGraph == 3) currentGraph = graphC;
    else if (selectedGraph == 4) currentGraph = graphD;
    else currentGraph = graphA;

    drawHeading("Graph " + selectedGraph + " - Options", iL, sT + (uH * 3.5), iW, tH);
    drawButton("Line", (currentGraph.getGraphType() == "linechart")? c_sidebar_accent:c_sidebar_button, iL, sT + (uH * 4.5), iW / 3, iH, tH);
    drawButton("Dots", (currentGraph.getGraphType() == "dotchart")? c_sidebar_accent:c_sidebar_button, iL + (iW / 3), sT + (uH * 4.5), iW / 3, iH, tH);
    drawButton("Bar", (currentGraph.getGraphType() == "barchart")? c_sidebar_accent:c_sidebar_button, iL + (iW * 2 / 3), sT + (uH * 4.5), iW / 3, iH, tH);
    drawRectangle(c_sidebar_divider, iL + (iW / 3), sT + (uH * 4.5) + (1 * uimult), 1 * uimult, iH - (2 * uimult));
    drawRectangle(c_sidebar_divider, iL + (iW * 2 / 3), sT + (uH * 4.5) + (1 * uimult), 1 * uimult, iH - (2 * uimult));

    drawDatabox(str(currentGraph.getMinX()).replaceAll("[0]+$", "").replaceAll("[.]+$", ""), c_idletab_text, iL, sT + (uH * 5.5), (iW / 2) - (6 * uimult), iH, tH);
    drawButton("x", c_sidebar_button, iL + (iW / 2) - (6 * uimult), sT + (uH * 5.5), 12 * uimult, iH, tH);
    drawDatabox(str(currentGraph.getMaxX()).replaceAll("[0]+$", "").replaceAll("[.]+$", ""), iL + (iW / 2) + (6 * uimult), sT + (uH * 5.5), (iW / 2) - (6 * uimult), iH, tH);
    drawDatabox(str(currentGraph.getMinY()).replaceAll("[0]+$", "").replaceAll("[.]+$", ""), iL, sT + (uH * 6.5), (iW / 2) - (6 * uimult), iH, tH);
    drawButton("y", c_sidebar_button, iL + (iW / 2) - (6 * uimult), sT + (uH * 6.5), 12 * uimult, iH, tH);
    drawDatabox(str(currentGraph.getMaxY()).replaceAll("[0]+$", "").replaceAll("[.]+$", ""), iL + (iW / 2) + (6 * uimult), sT + (uH * 6.5), (iW / 2) - (6 * uimult), iH, tH);
    drawButton((autoAxis)? "Scale: Auto":"Scale: Manual", c_sidebar_button, iL, sT + (uH * 7.5), iW, iH, tH);

    // Input Data Columns
    drawHeading("Data Format", iL, sT + (uH * 9), iW, tH);
    drawDatabox("Rate: " + xRate + "Hz", iL, sT + (uH * 10), iW, iH, tH);
    //drawButton("Add Column", c_sidebar_button, iL, sT + (uH * 13.5), iW, iH, tH);
    drawDatabox("Split", c_idletab_text, iL, sT + (uH * 11), iW - (80 * uimult), iH, tH);
    drawButton("1", (graphMode == 1)? c_sidebar_accent:c_sidebar_button, iL + iW - (80 * uimult), sT + (uH * 11), 20 * uimult, iH, tH);
    drawButton("2", (graphMode == 2)? c_sidebar_accent:c_sidebar_button, iL + iW - (60 * uimult), sT + (uH * 11), 20 * uimult, iH, tH);
    drawButton("3", (graphMode == 3)? c_sidebar_accent:c_sidebar_button, iL + iW - (40 * uimult), sT + (uH * 11), 20 * uimult, iH, tH);
    drawButton("4", (graphMode == 4)? c_sidebar_accent:c_sidebar_button, iL + iW - (20 * uimult), sT + (uH * 11), 20 * uimult, iH, tH);
    drawRectangle(c_sidebar_divider, iL + iW - (60 * uimult), sT + (uH * 11) + (1 * uimult), 1 * uimult, iH - (2 * uimult));
    drawRectangle(c_sidebar_divider, iL + iW - (40 * uimult), sT + (uH * 11) + (1 * uimult), 1 * uimult, iH - (2 * uimult));
    drawRectangle(c_sidebar_divider, iL + iW - (20 * uimult), sT + (uH * 11) + (1 * uimult), 1 * uimult, iH - (2 * uimult));

    float tHnow = 12;

    for (int j = 0; j < graphMode + 1; j++) {
      if (j < graphMode) drawText("Graph " + (j + 1), c_idletab_text, iL, sT + (uH * tHnow), iW, iH * 3 / 4);
      else drawText("Hidden", c_idletab_text, iL, sT + (uH * tHnow), iW, iH * 3 / 4);
      tHnow += 0.75;
      int itemCount = 0;

      // List of Data Columns
      for (int i = 0; i < dataColumns.length; i++) {

        if (graphAssignment[i] == j + 1) {
          // Column name
          drawDatabox(dataColumns[i], iL, sT + (uH * tHnow), iW - (40 * uimult), iH, tH);

          // Up button
          color buttonColor = c_colorlist[i-(c_colorlist.length * floor(i / c_colorlist.length))];
          drawButton((graphAssignment[i] > 1)? "▲":"", c_sidebar, buttonColor, iL + iW - (40 * uimult), sT + (uH * tHnow), 20 * uimult, iH, tH);

          // Down button
          drawButton((graphAssignment[i] < graphMode + 1)? "▼":"", c_sidebar, buttonColor, iL + iW - (20 * uimult), sT + (uH * tHnow), 20 * uimult, iH, tH);

          drawRectangle(c_sidebar_divider, iL + iW - (20 * uimult), sT + (uH * tHnow) + (1 * uimult), 1 * uimult, iH - (2 * uimult));
          tHnow++;
          itemCount++;
        }
      }

      if (itemCount == 0) drawText("Empty", c_idletab_text, iL + iW / 2, sT + (uH * (tHnow - itemCount - 0.75)), iW / 2, iH * 3 / 4);
    }

    textAlign(LEFT, TOP);
    textFont(base_font);
    fill(c_status_bar);
    text("Output: " + constrainString(outputfile, width - sW - round(30 * uimult) - textWidth("Output: ")), round(5 * uimult), height - round(bottombarHeight * uimult) + round(2*uimult));
  }


  /**
   	 * Keyboard input handler function
   	 *
   	 * @param  key The character of the key that was pressed
   	 */
  void keyboardInput (char keyChar, int keyCodeInt, boolean codedKey) {
    if (!codedKey && key == 's' && serialConnected) {
      thread("serialSendDialog");
    } else if (codedKey) {
      switch (keyCodeInt) {
      case UP:
        // Scroll menu bar
        if (mouseX >= cR && menuScroll != -1) {
          menuScroll -= (12 * uimult);
          if (menuScroll < 0) menuScroll = 0;
        }
        redrawUI = true;
        break;

      case DOWN:
        // Scroll menu bar
        if (mouseX >= cR && menuScroll != -1) {
          menuScroll += (12 * uimult);
          if (menuScroll > menuHeight - (height - cT)) menuScroll = menuHeight - (height - cT);
        }
        redrawUI = true;
        break;

      case KeyEvent.VK_PAGE_UP:
        // Scroll menu bar
        if (mouseX >= cR && menuScroll != -1) {
          menuScroll -= height - cT;
          if (menuScroll < 0) menuScroll = 0;
          redrawUI = true;
        }
        break;

      case KeyEvent.VK_PAGE_DOWN:
        // Scroll menu bar
        if (mouseX >= cR && menuScroll != -1) {
          menuScroll += height - cT;
          if (menuScroll > menuHeight - (height - cT)) menuScroll = menuHeight - (height - cT);
          redrawUI = true;
        }
        break;

      case KeyEvent.VK_END:
        // Scroll menu bar
        if (mouseX >= cR && menuScroll != -1) {
          menuScroll = menuHeight - (height - cT);
          redrawUI = true;
        }
        break;

      case KeyEvent.VK_HOME:
        // Scroll menu bar
        if (mouseX >= cR && menuScroll != -1) {
          menuScroll = 0;
          redrawUI = true;
        }
        break;
      }
    }
  }


  /**
   	 * Content area mouse click handler function
   	 *
   	 * @param  xcoord X-coordinate of the mouse click
   	 * @param  ycoord Y-coordinate of the mouse click
   	 */
  void contentClick (int xcoord, int ycoord) {
    if ((graphMode == 1 || ycoord <= (cT + cB) / 2) && (graphMode < 3 || xcoord <= (cL + cR) / 2)) {
      selectedGraph = 1;
      graphA.setHighlight(true);
      graphB.setHighlight(false);
      graphC.setHighlight(false);
      graphD.setHighlight(false);
      redrawUI = true;
      redrawContent = true;
    } else if ((ycoord > (cT + cB) / 2 && graphMode > 1) && (xcoord <= (cL + cR) / 2 || graphMode < 4)) {
      selectedGraph = 2;
      graphA.setHighlight(false);
      graphB.setHighlight(true);
      graphC.setHighlight(false);
      graphD.setHighlight(false);
      redrawUI = true;
      redrawContent = true;
    } else if ((ycoord <= (cT + cB) / 2 && graphMode > 2) && (xcoord > (cL + cR) / 2)) {
      selectedGraph = 3;
      graphA.setHighlight(false);
      graphB.setHighlight(false);
      graphC.setHighlight(true);
      graphD.setHighlight(false);
      redrawUI = true;
      redrawContent = true;
    } else if ((ycoord > (cT + cB) / 2 && graphMode > 3) && (xcoord > (cL + cR) / 2)) {
      selectedGraph = 4;
      graphA.setHighlight(false);
      graphB.setHighlight(false);
      graphC.setHighlight(false);
      graphD.setHighlight(true);
      redrawUI = true;
      redrawContent = true;
    }
  }


  /**
   	 * Scroll wheel handler function
   	 *
   	 * @param  amount Multiplier/velocity of the latest mousewheel movement
   	 */
  void scrollWheel (float amount) {
    // Scroll menu bar
    if (mouseX >= cR && menuScroll != -1) {
      menuScroll += (sideItemHeight * amount * uimult);
      if (menuScroll < 0) menuScroll = 0;
      else if (menuScroll > menuHeight - (height - cT)) menuScroll = menuHeight - (height - cT);
    }

    redrawUI = true;
  }


  /**
   	 * Scroll bar handler function
   	 *
   	 * @param  xcoord Current mouse x-coordinate position
   	 * @param  ycoord Current mouse y-coordinate position
   	 */
  void scrollBarUpdate (int xcoord, int ycoord) {
  }


  /**
   	 * Sidebar mouse click handler function
   	 *
   	 * @param  xcoord X-coordinate of the mouse click
   	 * @param  ycoord Y-coordinate of the mouse click
   	 */
  void menuClick (int xcoord, int ycoord) {

    // Coordinate calculation
    int sT = cT;
    if (menuScroll > 0) sT -= menuScroll;
    int sL = cR;
    int sW = width - cR;
    int sH = height - sT;

    int uH = round(sideItemHeight * uimult);
    int tH = round((sideItemHeight - 8) * uimult);
    int iH = round((sideItemHeight - 5) * uimult);
    int iL = round(sL + (10 * uimult));
    int iW = int(sW - (20 * uimult));

    // Select output file name and directory
    if ((mouseY > sT + (uH * 1)) && (mouseY < sT + (uH * 1) + iH)) {
      if (!recordData) {
        outputfile = "";
        selectOutput("Select a location and name for the output *.CSV file", "fileSelected");
      }
    }

    // Start recording data and saving it to a file
    else if ((mouseY > sT + (uH * 2)) && (mouseY < sT + (uH * 2) + iH)) {
      if (recordData) {
        stopRecording();
      } else if (outputfile != "" && outputfile != "No File Set") {
        startRecording();
      }
      //else {
      //	alertMessage("Error\nPlease set an output file path.");
      //}
    }

    // Change graph type
    else if ((mouseY > sT + (uH * 4.5)) && (mouseY < sT + (uH * 4.5) + iH)) {
      Graph currentGraph;
      if (selectedGraph == 2) currentGraph = graphB;
      else if (selectedGraph == 3) currentGraph = graphC;
      else if (selectedGraph == 4) currentGraph = graphD;
      else currentGraph = graphA;

      // Line
      if ((mouseX > iL) && (mouseX <= iL + iW / 3)) {
        currentGraph.setGraphType("linechart");
        redrawContent = redrawUI = true;
      }

      // Dot
      else if ((mouseX > iL + (iW / 3)) && (mouseX <= iL + (iW * 2 / 3))) {
        currentGraph.setGraphType("dotchart");
        redrawContent = redrawUI = true;
      }

      // Bar
      else if ((mouseX > iL + (iW * 2 / 3)) && (mouseX <= iL + iW)) {
        currentGraph.setGraphType("barchart");
        redrawContent = redrawUI = true;
      }
    }

    // Update X axis scaling
    else if ((mouseY > sT + (uH * 5.5)) && (mouseY < sT + (uH * 5.5) + iH)) {
      Graph currentGraph;
      if (selectedGraph == 2) currentGraph = graphB;
      else if (selectedGraph == 3) currentGraph = graphC;
      else if (selectedGraph == 4) currentGraph = graphD;
      else currentGraph = graphA;

      // Change X axis minimum value [DISABLED]
      /*
			if ((mouseX > iL) && (mouseX < iL + (iW / 2) - (6 * uimult))) {
       				final String xMin = showInputDialog("Please enter new X-axis minimum value:");
       				if (xMin != null){
       					try {
       						currentGraph.setMinMax(Float.parseFloat(xMin), 0);
       					} catch (Exception e) {}
       				} 
       				redrawContent = redrawUI = true;
       			}
       			*/

      // Change X axis maximum value
      if ((mouseX > iL + (iW / 2) + (6 * uimult)) && (mouseX < iL + iW)) {
        ValidateInput userInput = new ValidateInput("Set the X-axis Maximum Value", "Maximum:", str(currentGraph.getMaxX()));
        userInput.setErrorMessage("Error\nInvalid x-axis maximum value entered.\nPlease input a number greater than 0.");
        if (userInput.checkFloat(userInput.GT, 0)) {
          currentGraph.setMaxX(userInput.getFloat());
          sampleWindow[selectedGraph - 1] = int(xRate * abs(currentGraph.getMaxX() - currentGraph.getMinX()));
        } 
        redrawContent = redrawUI = true;
      }
    }

    // Update Y axis scaling
    else if ((mouseY > sT + (uH * 6.5)) && (mouseY < sT + (uH * 6.5) + iH)) {
      Graph currentGraph;
      if (selectedGraph == 2) currentGraph = graphB;
      else if (selectedGraph == 3) currentGraph = graphC;
      else if (selectedGraph == 4) currentGraph = graphD;
      else currentGraph = graphA;

      // Change Y axis minimum value
      if ((mouseX > iL) && (mouseX < iL + (iW / 2) - (6 * uimult))) {
        ValidateInput userInput = new ValidateInput("Set the Y-axis Minimum Value", "Minimum:", str(currentGraph.getMinY()));
        userInput.setErrorMessage("Error\nInvalid y-axis minimum value entered.\nThe number should be smaller the the maximum value.");
        if (userInput.checkFloat(userInput.LT, currentGraph.getMaxY())) {
          currentGraph.setMinY(userInput.getFloat());
        } 
        redrawContent = redrawUI = true;
      }

      // Change Y axis maximum value
      else if ((mouseX > iL + (iW / 2) + (6 * uimult)) && (mouseX < iL + iW)) {
        ValidateInput userInput = new ValidateInput("Set the Y-axis Maximum Value", "Maximum:", str(currentGraph.getMaxY()));
        userInput.setErrorMessage("Error\nInvalid y-axis maximum value entered.\nThe number should be larger the the minimum value.");
        if (userInput.checkFloat(userInput.GT, currentGraph.getMinY())) {
          currentGraph.setMaxY(userInput.getFloat());
        } 
        redrawContent = redrawUI = true;
      }
    }

    // Turn auto-scaling on/off
    else if ((mouseY > sT + (uH * 7.5)) && (mouseY < sT + (uH * 7.5) + iH)) {
      autoAxis = !autoAxis;
      redrawUI = true;
    }

    // Change the input data rate
    else if ((mouseY > sT + (uH * 10)) && (mouseY < sT + (uH * 10) + iH)) {
      ValidateInput userInput = new ValidateInput("Received Data Update Rate", "Frequency (Hz):", str(graphA.getXrate()));
      userInput.setErrorMessage("Error\nInvalid frequency entered.\nThe rate can only be a number between 0 - 10,000 Hz");
      if (userInput.checkFloat(userInput.GT, 0, userInput.LTE, 10000)) {
        xRate = userInput.getFloat();
        graphA.setXrate(xRate);
        graphB.setXrate(xRate);
        graphC.setXrate(xRate);
        graphD.setXrate(xRate);
        sampleWindow[0] = int(xRate * abs(graphA.getMaxX() - graphA.getMinX()));
        sampleWindow[1] = int(xRate * abs(graphB.getMaxX() - graphB.getMinX()));
        sampleWindow[2] = int(xRate * abs(graphC.getMaxX() - graphC.getMinX()));
        sampleWindow[3] = int(xRate * abs(graphD.getMaxX() - graphD.getMinX()));

        redrawContent = true;
        redrawUI = true;
      }
    }

    // Add a new input data column
    else if ((mouseY > sT + (uH * 11)) && (mouseY < sT + (uH * 11) + iH)) {

      // Graph mode 1
      if ((mouseX >= iL + iW - (80 * uimult)) && (mouseX < iL + iW - (60 * uimult))) {
        graphMode = 1;
        graphA.changeSize(cL, cR, cT, cB);
        redrawUI = true;
        redrawContent = true;
        if (selectedGraph > 1) {
          selectedGraph = 1;
          graphA.setHighlight(true);
          graphB.setHighlight(false);
          graphC.setHighlight(false);
          graphD.setHighlight(false);
        }
        for (int i = 0; i < graphAssignment.length; i++) {
          if (graphAssignment[i] > graphMode + 1) graphAssignment[i] = graphMode + 1;
        }

        // Graph mode 2
      } else if ((mouseX >= iL + iW - (60 * uimult)) && (mouseX < iL + iW - (40 * uimult))) {
        graphMode = 2;
        redrawUI = true;
        redrawContent = true;
        graphA.changeSize(cL, cR, cT, (cT + cB) / 2);
        graphB.changeSize(cL, cR, (cT + cB) / 2, cB);
        if (selectedGraph > 2) {
          selectedGraph = 2;
          graphA.setHighlight(false);
          graphB.setHighlight(true);
          graphC.setHighlight(false);
          graphD.setHighlight(false);
        }
        for (int i = 0; i < graphAssignment.length; i++) {
          if (graphAssignment[i] > graphMode + 1) graphAssignment[i] = graphMode + 1;
        }

        // Graph mode 3
      } else if ((mouseX >= iL + iW - (40 * uimult)) && (mouseX < iL + iW - (20 * uimult))) {
        graphMode = 3;
        redrawUI = true;
        redrawContent = true;
        graphA.changeSize(cL, (cL + cR) / 2, cT, (cT + cB) / 2);
        graphB.changeSize(cL, cR, (cT + cB) / 2, cB);
        graphC.changeSize((cL + cR) / 2, cR, cT, (cT + cB) / 2);
        if (selectedGraph > 3) {
          selectedGraph = 3;
          graphA.setHighlight(false);
          graphB.setHighlight(false);
          graphC.setHighlight(true);
          graphD.setHighlight(false);
        }
        for (int i = 0; i < graphAssignment.length; i++) {
          if (graphAssignment[i] > graphMode + 1) graphAssignment[i] = graphMode + 1;
        }

        // Graph mode 4
      } else if ((mouseX >= iL + iW - (20 * uimult)) && (mouseX < iL + iW)) {
        graphMode = 4;
        redrawUI = true;
        redrawContent = true;
        graphA.changeSize(cL, (cL + cR) / 2, cT, (cT + cB) / 2);
        graphB.changeSize(cL, (cL + cR) / 2, (cT + cB) / 2, cB);
        graphC.changeSize((cL + cR) / 2, cR, cT, (cT + cB) / 2);
        graphD.changeSize((cL + cR) / 2, cR, (cT + cB) / 2, cB);
        for (int i = 0; i < graphAssignment.length; i++) {
          if (graphAssignment[i] > graphMode + 1) graphAssignment[i] = graphMode + 1;
        }
      }

      //final String colname = showInputDialog("Column Name:");
      //if (colname != null){
      //    dataColumns = append(dataColumns, colname);
      //    dataTable.addColumn("Untitled-" + dataColumns.length);
      //    graphAssignment = append(graphAssignment, 1);
      //    redrawUI = true;
      //}
    } else {
      float tHnow = 12;

      for (int j = 0; j < graphMode + 1; j++) {
        tHnow += 0.75;

        // List of Data Columns
        for (int i = 0; i < dataColumns.length; i++) {

          if (graphAssignment[i] == j + 1) {

            if ((mouseY > sT + (uH * tHnow)) && (mouseY < sT + (uH * tHnow) + iH)) {

              // Down arrow
              if ((mouseX > iL + iW - (20 * uimult)) && (mouseX <= iL + iW)) {
                graphAssignment[i]++;
                if (graphAssignment[i] > graphMode + 1) graphAssignment[i] = graphMode + 1;
                redrawUI = true;
                redrawContent = true;
              }

              // Up arrow
              else if ((mouseX >= iL + iW - (40 * uimult)) && (mouseX <= iL + iW - (20 * uimult))) {
                graphAssignment[i]--;
                if (graphAssignment[i] < 1) graphAssignment[i] = 1;
                redrawUI = true;
                redrawContent = true;
              }

              // Change name of column
              else {
                final String colname = myShowInputDialog("Set the Data Signal Name", "Name:", dataColumns[i]);
                if (colname != null && colname != "") {
                  dataColumns[i] = colname;
                  redrawUI = true;
                }
              }
            }

            tHnow++;
          }
        }
      }
    }
  }


  /**
   	 * Check whether it is safe to exit the program
   	 *
   	 * @return True if the are no tasks active, false otherwise
   	 */
  boolean checkSafeExit() {
    if (recordData) return false;
    return true;
  }


  /**
   	 * End any active processes and safely exit the tab
   	 */
  void performExit() {
    if (recordData) stopRecording();
  }
}

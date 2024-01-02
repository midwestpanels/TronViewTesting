# TronView
Project for connecting efis/flight data to a 2nd screen or HUD.

## Features Include:
- Build custom efis or hud screens
- Record and Playback flight log data ( and fast forward through playback )
- All screens look and work the same for all supported data input.
- All display screen sizes and ratios supported.
- Text mode
- Touch screen support
- 30 + FPS on Raspberry Pi 4 or Pi 5
- Remote keypad / user input support.
- Display flight data in Knots, Standard, Metric, F or C
- Designed for Raspberry Pi but also runs on Mac OSx, Windows, and other linux systems.
- Show NAV needles for approaches. (If NAV data is available)
- Use multiple data input sources


## Includes F-18 style HUD screen:
Comes with a built F-18 HUD screen which features the following.

- F-18 style artificial horizon
- Flight Path marker
- Velocity Vector and Ghost Velocity Vector
- Waterline
- Glideslobe (If data is available)
- Bank angle
- Traffic Target Radar (Key 3 cycles with ranges)
- A-A Gun Cross Funnel for A-A Gunnery demonstration  Use Keypad Key Num 8 to cycle from Off to 25ft Tgt Wingspan/to 30ft wingspan to 35ft wingspan to Off, 
wingSpan ranges stert at 250ft at wide part of big U (& the Yellow + graphic), then extend to 500ft at first Yellow circle pipper, then to 750 ft, then 
to 1,000 ft at next pipper, the 1500ft, and last at 2,000ft.  for better description of this function check these links;  http://falcon4.wikidot.com/avionics:hud  & Video at https://www.youtube.com/watch?v=oOa9eWgFllE
- Shows RPM and next Way Point Distance read out to right of HUD below the Altitude.
- Amoung other commn things like Airspeed / altitude / VSI ...


## Use as backup display screen on dash
![cockpit1](docs/efis_cockpit1.jpeg?raw=true)
![cockpit2](docs/efis_cockpit2.jpeg?raw=true)


## Traditional EFIS looking screen
![screenshot1](docs/efis_screenshot.png?raw=true)


## F18 Style HUD
![hud_animation](docs/hud_animated_example.gif?raw=true)


## Text Mode
![hud_animation](docs/efis_screenshot_text.png?raw=true)


# About

This is a python3 application that will take in data from different input sources, process them into a common format, then output (draw) 
them to custom screens (HUD or efis style).  The system is created to have the inputs and screens seperate and non-dependent of each other.  
For example a user running a MGL iEFIS can run the same screen as a user with a Dynon D100.  Issues can come up with a input source does not 
have all the data available as other input sources do.  But if the screen is written well enough it will hide or show data if it's available.


## Currently supports:

MGL iEFIS (serial)

Garmin G3x (serial)

Dynon Skyview (serial)

Dynon D10/100 (serial)

Levil BOM (wifi)

Stratux or any stratux compatiable device (wifi)

Generic serial logger (Used for recording any serial data)

We are using the rapberry pi 4B and 5 for taking serial data from a EFIS (MGL,Dynon,G3x) and displaying a graphical Display out the hdmi output on the pi.  This is plugged into a Display or HUD device like the Hudly Classic.  Any sort of HDMI screen could be hooked to the Pi for displaying this flight data.

# Steps to get the software running on raspberry pi

1a) got https://www.raspberrypi.com/software/ to download the Raspberry Pi Imager.  This will create a bootable sd card you can insert into the pi. Download, install and run.  Following the directions.  You will need to get your pi onto the internet because you need to download the TronView source to your pi next.  tested with the lastest Raspberry Pi OS - Debian GNU/Linux 12 (bookworm)

1b) Now you need to plug the SD card into the pi and boot it up.  Plug a video screen, keyboard, and mouse in.  Once the pi boots up click on the Terminal window. Usally the icon to lanuch the Terminal is in the top left of the screen.

1c) For a quick start you can try running the following command in terminal.  This will automatically download the latest source and setup your pi to run TronView.  This will allow you to skip steps 2 to 6.

`wget http://nerd.cc/tronview.sh -O - | sh`

2) Install git command.   This will let you get the latest source from github directly onto the pi.  git may already be installed on your pi.  If so you will tell you 0 packages installed.  Type in the following in the terminal.

`sudo apt-get -y install git`

3) clone the source from github

`git clone https://github.com/flyonspeed/TronView.git`

when done this will create a TronView dir

4) run the setup.sh script to finish install.  This will setup serial port (if not allready setup), and install python libraries needed.

go into the TronView folder by typing

`cd TronView/util/rpi`

then to run the script type

`./setup.sh`

5) reboot the device.  this is only needed to make sure the serial port is setup.  type

`reboot`

6) Test the hud/efis screen with existing example data.  
run the following.  First make sure you are in the efis_hud dir.

`cd TronView`

Then run the python script and use the example data for dynon d100.

`sudo python main.py -i serial_d100 -s F18_HUD -e`

You should see a basic hud on the screen.  And it slowy moving around.  
If not then your video setup maybe be F'd up.  

To exit you hit "q" on the keyboard.


7a) Next if you want to run live serial data from your efis.
run the serial_read.py script to confirm data being sent is correct.
go into TronView dir then type

`cd TronView`

`sudo python util/serial_read.py`

you will have to pass in -m or -s for mgl or skyview data.

make sure the data is coming through and looks good.

to exit hit cntrl-c

7b) run it

Note:  You have to run using sudo in order to get access to serial port.

Example:

`sudo python3 main.py -i serial_mgl -e`

will show you command line arguments.

`-i {input data source 1}` 

load a input module for where to get the air data from. 

--in1 {input source 1} (same as -i)

--in2 {input source 2} 

-s (optional) {screen module name to load} (located in the lib/screens folder)

-t (optional) start up in text mode

-e (optional) demo mode. load default example demo data for input source selected.

-c FILENAME (optional) custom playback file. Enter filename of custom example demo file to use.

--playfile1 (optional) same as -c.. set playback file for input 1

--playfile2 (optional) set playback file for input 2

--listlogs (optional) show logs you saved. location id defined in config.cfg

--listusblogs (optional) show logs saved to usb drive (if available)

--listexamplelogs (optional) show example log files to playback


Run the command with no arguments and it will show you which input modules and screen modules are available to use.

`sudo python3 main.py`

## Example command line arguments

To set Input1 to garmin g3x and Input2 to stratux wifi and run default example data run the following.

`sudo python3 main.py --in1 serial_g3x --in2 stratux_wifi -e`


To run Input1 to MGL and Input2 as stratux.  then supply custom example log files...  Note that these custom log files are in the example data dir.  This will also check the DataRecorder path dir that you set in the config file. 

`sudo python3 main.py --in1 serial_mgl --playfile1 mgl_1.dat --in2 stratux_wifi --playfile2 startux_1.dat`


Lauch app in text mode playing dynon D100 example data. Press 'Q' to exit Text Mode.

`sudo python3 main.py --in1 serial_d100 -t -e`

## More help on raspberry pi.

Setup your serial input using the GPIO pins on pi zero.  This page will help you. https://www.instructables.com/id/Read-and-write-from-serial-port-with-Raspberry-Pi/


Here are more instructions on setting up for raspberry pi.

https://github.com/flyonspeed/TronView/blob/master/docs/rpi_setup.md



## DefaultScreen

The default screen has the following keyboard commands.

current commands are:

q - quit

t - switch to text mode  ( can not switch back to graphic mode after you enter text mode )

p - pause/unpause play back of log file (only if you are playing back a log file)

cntrl right arrow - fast forward play back file.

cntrl left arrow - rewind play back file (currently doesn't really work great)

cntrl d - show some debug info (same as using the number 7 key)

1 - create flight data log file

2 - close flight data log file

3 - cylce through traffic scope modes and ranges (if traffic input source available)

4 - drop target buoy at current location and altitude.  Will be visiable on traffic scope

5 - drop target buoy exactly 1 mile ahead of aircraft heading.

6 - clear all target buoys.

7 - cycle through debug modes (in graphic mode)

9 - show gun target in F-18 HUD screen.

PAGE UP - jump to next screen

PAGE DOWN - go to previous screen

To run with


## Creating your own custom screen file

Screen files are located in lib/screens folder.  For a quick start you can duplicate the DefaultScreen.py file and rename it to your own.  What ever name you give it you must name the Class in that file to the same name.  For exmaple if I create MyHud.py then in that file I must set the class to the following

`class MyHud(Screen):`


Functions for custom screens:

### initDisplay(self,pygamescreen,width,height):

this is called once on screen init and tells your class the screen width and height

### draw(self,aircraft):

draw() is called in the main draw loop.  And the current aircraft object is passed in with all current postion values.

### clearScreen(self):

each time the screen is drawn it first has to clear it.  this is called to do that.

### processEvent(self,event):

handles events like key presses.

## Creating your own Input source module

Input sources are located in lib/inputs folder.  Create your own!  You can use existing modules as examples for how to do this.


## Screen resolution for hudly


1.        Type “sudo raspi-config”
2.        Select "Advanced options"
3.        Select A5 – Resolution  (Screen)  -> Select “CEA Mode 3  720x480  60 Hz  16:9”
6.        Select “OK”
7.        Select:  "Finish"
8.        Select:  "Yes"
9.        Wait for the reboot



## config.cfg

config.cfg can be created and sit in the same dir as the main.py python app.  It's used for configuring the hud.  View config_example.cfg for a example of options.

by using the config file you don't need to pass in so many arguments to the command line.

Here is a example config.cfg file:

https://github.com/flyonspeed/TronView/blob/master/config_example.cfg

# Demo Sample EFIS Data

  Demo data is saved in lib/inputs/_example_data .  Demo data lets you run previously recorded data through the hud app for demo or testing purposes.  Each input source module uses it's own format for data.  So for example if you want to run dynon demo data then you must run it through the dynon input source module.
  
  Using the -c FILENAME command line option loads data from the _example_data folder.
  
  Example to run the MGL_2.dat demo data file would look like this:
  
  `sudo python3 main.py --in1 serial_mgl --playfile1 MGL_2.dat`

  Note this is setting up input1.


# MGL Data

  MGL EFIS Sample Data Link: https://drive.google.com/open?id=1mPOmQuIT-Q5IvIoVmyfRCtvBxCsLUuvz

  MGL EFIS Serial Protocol Link: https://drive.google.com/open?id=1OYj000ghHJqSfvacaHMO-jOcfd1raoVo

  MGL example data in this git repo include: (they can be ran by using the -c <filename> command line option.)

  MGL_V2.bin = G430_Bit_Test_24Mar19
  
  MGL_V3.bin = G430_Data_3Feb19_VertNdlFullUp_HzNdl_SltRt_toCtr
  
  MGL_V4.bin = G430_Data_3Feb19_HSI_Nedl_2degsRt_Vert_SlightLow
  
  MGL_V5.bin = G430_Data_3Feb19_HSI_Nedl_2degsLft_Vert_2Degs_Dwn
  
  MGL_V6.bin = G430_Data_3Feb19_HSI_Nedl_2degsRt_Vert_2Degs_Up
  
  MGL_V7.bin = G430_Data_3Feb19_Horz_Vert_Nedl_come to center
  
  MGL_V8.bin = G430_Data_13Ap19_VertNdlFullDwn_HzNdl_FullLft
  
  MGL_V9.bin = 13Ap_AltBug_0_500_1k_1.5k_to_10k_5.1K_5.2k_5.9kv10_v10
  
  MGL_V10.bin = 13Ap_XC_Nav 
  
  MGL_V11.bin = iEfis_NavDataCapture_5Feb19
  
  MGL_V12.bin = NavData_13Ap_HdgBug_360_10_20_30_to_360_001_002_003_to_010_v9

# Dynon Data

  Dynon Skyview EFIS Sample Data Link: https://drive.google.com/open?id=1jQ0q4wkq31C7BRn7qzMSMClqPw1Opwnp

  Dynon Skyview EFIS Serial Protocol Link: https://drive.google.com/open?id=1isurAOIiTzki4nh59lg6IDsy1XcsJqoG
  
  Dynon D100 Series EFIS Sample Data Link: https://drive.google.com/open?id=1_os-xv0Cv0AGFVypLfSeg6ucGv5lwDVj
  
  Dynon D100 Series EFIS Serial Protocol Link: https://drive.google.com/open?id=1vUBMJZC3W85fBu33ObuurYx81kj09rqE

# Garmin Data

  Garmin G3X EFIS Sample Data Link: https://drive.google.com/open?id=1gHPC3OipAs9K06wj5zMw_uXn3_iqZriS

  Garmin G3X EFIS Serial Protocol Link: https://drive.google.com/open?id=1uRRO-wdG7ya6_6-CfDVrZaKJsiYit-lm

# Stratux Data

  Recorded data for stratux is saved in the example data.  stratux_1.dat shows traffic near by.  Like 0.5 miles away.



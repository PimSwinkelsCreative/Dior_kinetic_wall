<h1>Dior kinetic wall Firmware</h1>

<h2> Introduction</h2>
This readme is for the final (non-interactive) installation only. This document is written after the interactive prototype has been released and further maintenance on this code is out of scope at the moment.

If everything works as planned, the boards should only have to be configured using the 8 bit dip switch on the bottom of the circuit board.

<h2>DIP Switch:</h2>
The Dip Switch has 8 switches, numbered from left to right. They are in their "On" position when they are pushed towards the ON marking on the top left of the switch housing.
The switched are in their "Off position when they are pushed towards the number below the switch.

<h3>Configuring for normal operation</h3>
For normal operation only the model of the installation has to be set. This has to be done by setting switches number 3 to 8 to zero and then configuring switch 1 and 2 as follows:
<table class="demo">
	<thead>
	<tr>
		<th>Installation Model</th>
		<th>Switch 1</th>
		<th>Switch 2</th>
	</tr>
	</thead>
	<tbody>
	<tr>
		<td>&nbsp;A</td>
		<td>&nbsp;Off</td>
		<td>&nbsp;Off</td>
	</tr>
	<tr>
		<td>&nbsp;B</td>
		<td>&nbsp;On</td>
		<td>&nbsp;Off</td>
	</tr>
	<tr>
		<td>&nbsp;C</td>
		<td>&nbsp;Off</td>
		<td>&nbsp;On</td>
	</tr>
	<tr>
		<td>&nbsp;D</td>
		<td>&nbsp;On</td>
		<td>&nbsp;On</td>
	</tr>
	</tbody>
</table>

<h3>Configuring for hardware test mode</h3>
For harware test mode the installation model type has to be set correctly (see previous paragraph). The only difference is that switch 8 needs to be set high.
This starts the interleaving waves animation. All rows should home correclty and then they should create a synchronous wave that alternates direction between the even and uneven rows. It is important that the waves meet in the middle, and that no two adjacent rows turn in the same direction.

<h2>Git Navigation</h2>
This Git repository conists of various folders. The table below will briefly elaborate on what they contain.

<table class="demo">
	<thead>
	<tr>
		<th>Folder name</th>
		<th>Contents</th>
	</tr>
	</thead>
	<tbody>
	<tr>
		<td>Final Installation V1.0 Firmware</td>
		<td>Firmware release V1.0 of the non-interactive installations. Folder contains two .zip archives; one for the firmware and one for the used libraries.</td>
	</tr>
	<tr>
		<td>Interactive prototype Installation V1.0 Firmware</td>
		<td>Firmware release for the interactive version of the installation. This code can be used as is, but will not be maintained or be further documented.</td>
	</tr>
	<tr>
		<td>electronic design</td>
		<td>&nbsp;A folder that contains information about the pcb that is relevant for the firmware. Currently only contains the pin mapping of the two designs.</td>
	</tr>
	<tr>
		<td>&nbsp;kinetic_wall</td>
		<td>&nbsp;Working folder for firmware development.</td>
	</tr>
	</tbody>
</table>

<h2>Uploading code to the installation PCBs</h2>
you will need the following materials to (re)program the installation pcbs:
<ul>
<li>A computer with a usb-A port (or with an external usb-c to usb-A dongle</li>
<li>a USB-A to micro-usb cable</li>
</ul>
The boards can be programmed by following these steps:
<ol>
<li>Make sure you have a computer with Arduino IDE 1.8.X installed (can be downloaded <a href="https://www.arduino.cc/en/software">here</a>).</li>
<li>Make sure the ESP32 Add-on ins installed in Arduino IDE. A tutorial on how to do this can be found <a href="https://randomnerdtutorials.com/installing-the-esp32-board-in-arduino-ide-windows-instructions/">here</a>. </li>
<li>Copy the unzipped contents of the "libraries" .zip folder to your libraries folder (on windows usually located at my documents/Arduino/Libraries).</li>
<li>Unzip the .zip file "kinetic_wall.zip and store  it in a location where you can find it easily.</li>
<li>Open the kinetic_wall.ino file in the folder you created in the previous step.</li>
<li>Remove the ESP32 board from the PCB.</li>
<li>Connect the ESP32 baord to the PC via the micro USB cable.</li>
<li>Select the right COM port in Arduino IDE. The ESP32 should show up as something containing "silicon labs".</li>
<li>Select the DOIT ESP32 DEVKIT V1 in the board manager.</li>
<li>Press the upload buttton in the top left. Uploading can take significantly longer than with a normal arduino. It should however not take more than a minute.</li>
</ol>

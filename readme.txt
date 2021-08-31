//Andre 2021 - 201903511@post.au.dk

Original/
	ihmt/ is the one used by the author for the figure in the paper
	
	Due-nmr-doc.pdf is the extra document that has some instructions for the operation of the device

	Due-nmr-firmware/ 
Contains the .ino file that is passed to the Arduino and the sine_table.h containing the tables for DAC and reference

my_changes/

/Matlab/
Contains the script I used to add scans and plot IQ data and a sample file that i placed there

/Pulse sequences/cpmg/ 

is the one used by me to play around and try to get some different parameters and understand the changes
		
*It appears to function perfectly in pulse width changes, number of echoes, frequency and relay control, as well as data collection. Issue appears to be the Vpp on the amplification channel*

Gcc version is the same as well as -Os to -O3 change from the compiler. Os goes for optimised space while the O3 goes for speed. Details in the link: 
https://gcc.gnu.org/onlinedocs/gcc/Optimize-Options.html. 

////// RUN SEQUENCE//////

To run a pulse sequence cd to the directory, Make the file and run the name followed by number of scans, and the width of the 90 pulse. (In imht case he also places there the 180º pulse)

Usage:

Cpmg: ./cpmg scans pw90 

(./cpmg 100 151 will create 100 files, from the ADC sampling and script from Matlab takes care of all of that, the 90º pulse used is 151us long, and in current settings, cpmg does 20 echoes 302us long, which are spaced 32ms)

Ihmt: ./ihmt scans pw90 pw180
(In this one pre-pulses are applied which transfer magnetization, which are the arrays that are declared first in the program) 

///////
If you want to change the amount of echoes need to be aware that the number of points also needs to change otherwise the program won't run. Right now is 20 echoes 20k points. If the numbers are changed and don't agree you will get segmentation faults





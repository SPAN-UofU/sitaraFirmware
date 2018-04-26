# sitaraFirmware

~~~
mkdir sitaraSoftware
cd sitaraSoftware
git clone git@github.com:SPAN-UofU/nRF_SDK_13_0_0.git
git clone git@github.com:SPAN-UofU/sitaraFirmware.git
~~~

Download and extract latest version of GNU toolchain for ARM from here :

https://launchpad.net/gcc-arm-embedded/+download

Extract it to /usr/local/ folder, and execute this line in terminal:

~~~
export PATH=$PATH:<INSTALL_PATH>/gcc-arm-none-eabi-5_4-2016q3/bin
~~~

Verify using 
~~~
arm-none-eabi-gcc --version
~~~

Modify makefile.posix file in NRF SDK folder: <SDK_PATH>/components/toolchain/gcc/Makefile.posix as follows:
Update value of GNU_INSTALL_ROOT variable to match the toolchain path. GNU_INSTALL_ROOT := /usr/local/gcc-arm-none-eabi-5_4-2016q3/ GNU_VERSION := 5.4.3
GNU_PREFIX := arm-none-eabi
Now try compiling the project.

Download nrfjprog from nordic website: https://www.nordicsemi.com/eng/nordic/Products/ . Itis bundled with nRF5x-Command-Line-Tools. Extract nrfjprog and add it to the path.
export PATH="/<your path>/ nRF5x-Command-Line-Tools_9_7_1_Linux-i386/ nrfjprog/nrfjprog:$PATH"
Check version with nrfjprog -v, to see if installed properly.

Download J-Link software from J-Link Software and Documentation
Pack:  https://www.segger.com/downloads/jlink . Download latest version and the one that matches your system, 64-bits in my case.
Execute following in terminal:
sudo dpkg -i ~/Downloads/<DOWNLOADED JLINK VERSION>
Check executing jlink command in terminal.

Building and programming project in terminal, go to makefile path of project and execute
“make flash”
In case of using Bluetooth softdevice first execute “make flash_softdevice” and then make flash.

For help and some of instruction go to following link:
https://gustavovelascoh.wordpress.com/2017/01/23/starting-development-with-nordic-nrf5x-and- gcc-on-linux/
  
Also you can search of instructions at :  https://devzone.nordicsemi.com/questions/


sudo chmod 666 /dev/ttyUSB0
sudo chmod 666 /dev/ttyUSB1
sudo chmod 666 /dev/ttyUSB2
sudo chmod 666 /dev/ttyUSB3
sudo chmod 666 /dev/ttyUSB4

#To install the application to 4 nodes
#make clean
#make telosb
#clear

make telosb reinstall,4 bsl,/dev/ttyUSB0
make telosb reinstall,2 bsl,/dev/ttyUSB1
make telosb reinstall,6 bsl,/dev/ttyUSB2
make telosb reinstall,8 bsl,/dev/ttyUSB3
make telosb reinstall,10 bsl,/dev/ttyUSB4

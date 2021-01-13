
sudo chmod 666 /dev/ttyUSB0
sudo chmod 666 /dev/ttyUSB1
sudo chmod 666 /dev/ttyUSB2
sudo chmod 666 /dev/ttyUSB3
#sudo chmod 666 /dev/ttyUSB4

#To install the application to 4 nodes
#make clean
#make telosb
#clear

make telosb reinstall,4 bsl,/dev/ttyUSB0
make telosb reinstall,1 bsl,/dev/ttyUSB1
make telosb reinstall,3 bsl,/dev/ttyUSB2
make telosb reinstall,5 bsl,/dev/ttyUSB3
#make telosb reinstall,10 bsl,/dev/ttyUSB4

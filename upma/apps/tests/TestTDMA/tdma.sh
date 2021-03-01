sudo chmod 666 /dev/ttyUSB0
sudo chmod 666 /dev/ttyUSB1
sudo chmod 666 /dev/ttyUSB2
sudo chmod 666 /dev/ttyUSB3
sudo chmod 666 /dev/ttyUSB4
sudo chmod 666 /dev/ttyUSB5
sudo chmod 666 /dev/ttyUSB6

make telosb

make telosb reinstall,0 bsl,/dev/ttyUSB0
make telosb reinstall,1 bsl,/dev/ttyUSB1
make telosb reinstall,2 bsl,/dev/ttyUSB2
make telosb reinstall,3 bsl,/dev/ttyUSB3
make telosb reinstall,4 bsl,/dev/ttyUSB4
make telosb reinstall,5 bsl,/dev/ttyUSB5
make telosb reinstall,6 bsl,/dev/ttyUSB6

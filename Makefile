# Makefile
# Sample for RH_RF95 (client and server) on Raspberry Pi
# Caution: requires bcm2835 library to be already installed
# http://www.airspayce.com/mikem/bcm2835/

CC            = g++
CFLAGS        = -DRASPBERRY_PI -DBCM2835_NO_DELAY_COMPATIBILITY -D__BASEFILE__=\"$*\" -std=c++11
LIBS          = -lbcm2835 -lmosquitto
RADIOHEADBASE = /home/pi/CoordinatorServer/include
INCLUDE       = -I$(RADIOHEADBASE)

all: coordinator

RasPi.o: $(RADIOHEADBASE)/RHutil/RasPi.cpp
				$(CC) $(CFLAGS) -c $(RADIOHEADBASE)/RHutil/RasPi.cpp $(INCLUDE)

coordinator.o: coordinator.cpp
				$(CC) $(CFLAGS) -c $(INCLUDE) $<

RH_RF95.o: $(RADIOHEADBASE)/RH_RF95.cpp
				$(CC) $(CFLAGS) -c $(INCLUDE) $<

RHDatagram.o: $(RADIOHEADBASE)/RHDatagram.cpp
				$(CC) $(CFLAGS) -c $(INCLUDE) $<

RHHardwareSPI.o: $(RADIOHEADBASE)/RHHardwareSPI.cpp
				$(CC) $(CFLAGS) -c $(INCLUDE) $<

RHSPIDriver.o: $(RADIOHEADBASE)/RHSPIDriver.cpp
				$(CC) $(CFLAGS) -c $(INCLUDE) $<

RHGenericDriver.o: $(RADIOHEADBASE)/RHGenericDriver.cpp
				$(CC) $(CFLAGS) -c $(INCLUDE) $<

RHGenericSPI.o: $(RADIOHEADBASE)/RHGenericSPI.cpp
				$(CC) $(CFLAGS) -c $(INCLUDE) $<

RHEncryptedDriver.o: $(RADIOHEADBASE)/RHEncryptedDriver.cpp
				$(CC) $(CFLAGS) -c $(INCLUDE) $<

coordinator: coordinator.o RH_RF95.o RasPi.o RHHardwareSPI.o RHGenericDriver.o RHGenericSPI.o RHSPIDriver.o
				$(CC) $^ $(LIBS) -o coordinatorLast

clean:
				rm -rf *.o coordinator

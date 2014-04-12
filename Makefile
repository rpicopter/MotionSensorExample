CXX=g++
CXXFLAGS= -Wall -g -O2
CXX_OPTS= -Wall -g -O2

INSTALL=install

%.o: %.c                                                                         
	$(CXX) $(CXXFLAGS) $(CXX_OPTS) $< -o $@ 


all: smtest.o 
	$(CXX) $(LDFLAGS) $(CXXFLAGS) -o smtest \
		main.c \
		MotionSensor/libMotionSensor.a \
		libs/libI2Cdev.a

smtest.o: MotionSensor/libMotionSensor.a libs/libI2Cdev.a

MotionSensor/libMotionSensor.a:
	$(MAKE) -C MotionSensor/ 

libs/libI2Cdev.a:
	$(MAKE) -C libs/I2Cdev

install1:
	$(INSTALL) -m 755 controller $(DESTDIR)/usr/local/bin/

clean:
	cd MotionSensor && $(MAKE) clean
	cd libs/I2Cdev && $(MAKE) clean
	rm -rf *.o *~ *.mod
	rm -rf smtest 

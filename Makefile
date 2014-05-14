GCC=gcc
GCC_FLAGS= -Wall -g -O2 -lm
GCC_OPTS= -Wall -g -O2

INSTALL=install

PROG=mstest

%.o: %.c
	$(GCC) $(GCC_FLAGS) $(GCC_OPTS) $< -o $@ 


all: $(PROG).o 
	$(GCC) $(LDFLAGS) $(GCC_FLAGS) -o $(PROG) \
		main.c \
		MotionSensor/libMotionSensor.a \
		libs/libI2Cdev.a

$(PROG).o: MotionSensor/libMotionSensor.a libs/libI2Cdev.a

MotionSensor/libMotionSensor.a:
	$(MAKE) -C MotionSensor/ 

libs/libI2Cdev.a:
	$(MAKE) -C libs/I2Cdev

install1:
	$(INSTALL) -m 755 $(PROG) $(DESTDIR)/usr/local/bin/

clean:
	cd MotionSensor && $(MAKE) clean
	cd libs/I2Cdev && $(MAKE) clean
	rm -rf *.o *~ *.mod
	rm -rf $(PROG)

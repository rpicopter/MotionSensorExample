CXX=g++
GCC=gcc
AR=ar
CXXFLAGS=-Wall -g -O2
CXX_OPTS=-c -DMPU6050 -DMPU_DEBUGOFF -I../libs/

GCC_FLAGS=-Wall -g -O2
GCC_OPTS=-c -DMPU6050 -DMPU_DEBUGOFF -I../libs/

SOURCES=sensor.c inv_mpu_lib/inv_mpu.c inv_mpu_lib/inv_mpu_dmp_motion_driver.c
OBJECTS=$(SOURCES:.c=.o)

LIB=libMotionSensor.a

%.o: %.c
	$(GCC) $(GCC_FLAGS) $(GCC_OPTS) $< -o $@ 

%.o: %.cpp
	$(CXX) $(CXXFLAGS) $(CXX_OPTS) $< -o $@ 

$(LIB): $(OBJECTS) 
	$(AR) rcs $(LIB) $(OBJECTS) 

all: $(LIB)

clean:
	rm -rf *.o
	rm -rf inv_mpu_lib/*.o
	rm -rf $(LIB)

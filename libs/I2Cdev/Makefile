GCC=gcc
AR=ar
GCC_CFLAGS=-Wall -g -O2
GCC_OPTS=-c
LDFLAGS= -lm -lrt

LIB=../libI2Cdev.a
OBJ=I2Cdev.o

%.o: %.c                                                                         
	$(GCC) $(GCC_FLAGS) $(GCC_OPTS) $< -o $@ 

$(LIB): $(OBJ)
	$(AR) rcs $(LIB) $(OBJ)

all: $(LIB)

clean:
	rm -rf *.o
	rm -rf $(LIB)

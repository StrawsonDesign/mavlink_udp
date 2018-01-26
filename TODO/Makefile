CC= gcc
CFLAGS= -g -pthread
LDFLAGS=
SRCDIR=src
SOURCES= $(SRCDIR)/*.c
DEPS= $(SRCDIR)/*.h
OBJECTS= mavlink_udp.o main.o
EXECUTABLE= mavlink_udp

all: $(EXECUTABLE)

$(EXECUTABLE): $(OBJECTS)
	$(CC) -o $@ $(OBJECTS)

$(OBJECTS):	
	$(CC) -c $(CFLAGS) $(SOURCES) $(DEPS)

clean:
	rm -f *.o core $(EXECUTABLE)
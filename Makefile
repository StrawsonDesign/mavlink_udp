CC= gcc
CFLAGS= -g
LDFLAGS=
SRCDIR=src
SOURCES= $(SRCDIR)/mavlink_udp.c
OBJECTS= mavlink_udp.o
EXECUTABLE= mavlink_udp

all: $(EXECUTABLE)

$(EXECUTABLE): $(OBJECTS)
	$(CC) -o $@ $<

$(OBJECTS):	
	$(CC) -c $(CFLAGS) $(SRCDIR)/mavlink_udp.c
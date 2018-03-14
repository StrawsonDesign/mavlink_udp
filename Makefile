

SRCDIR		:= src
BUILDDIR	:= build
INCLUDEDIR	:= include
BINDIR		:= bin


# This is a general use makefile for robotics cape projects written in C.
# Just change the target name to match your main source code filename.
TARGETS = $(BINDIR)/rc_test_mavlink


CC		:= gcc
LINKER		:= gcc
#CFLAGS		:= -Wall -Wextra -I $(INCLUDEDIR)
CFLAGS		:= -pthread -I $(INCLUDEDIR)
LFLAGS		:= -lm -lrt -pthread

SOURCES		:= $(shell find $(SRCDIR) -type f -name *.c)
OBJECTS		:= $(SOURCES:$(SRCDIR)/%.c=$(BUILDDIR)/%.o)
INCLUDES	:= $(shell find include -name '*.h')

prefix		:= /usr/local
RM		:= rm -rf
INSTALL		:= install -m 4755
INSTALLDIR	:= install -d -m 755




# linking rules
$(TARGETS): $(OBJECTS)
	@mkdir -p $(BINDIR)
	@$(LINKER) $(LFLAGS) -o $(TARGETS) $(OBJECTS)
	@echo "Done linking $(@)"


# rule for all other objects
$(BUILDDIR)/%.o : $(SRCDIR)/%.c $(INCLUDES)
	@mkdir -p $(dir $(@))
	@$(CC) -c $(CFLAGS) $(ARCFLAGS) $(OPT_FLAGS) $(FLOAT_FLAG) $(DEBUGFLAG) $< -o $(@)
	@echo "made: $(@)"


all:
	$(TARGETS)

debug:
	$(MAKE) $(MAKEFILE) DEBUGFLAG="-g -D DEBUG"
	@echo " "
	@echo "$(TARGET) Make Debug Complete"
	@echo " "



clean:
	@$(RM) $(BINDIR)
	@$(RM) $(BUILDDIR)
	@echo "Clean Complete"


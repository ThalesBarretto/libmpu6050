CC	=gcc
CPPFLAGS=
CFLAGS	=-fPIC -Wall -Wextra -Wpedantic
DBGFLAGS=-DMPU6050_DEBUG
LIBS	=-lm -li2c
MODULE	=mpu6050
MODV	=0
APIV	=0

# Sources, build and test directories
SRC	=src
BLD	=bld
OBJ	=$(BLD)/obj
TST	=$(BLD)/tst

# Headers and sources
HDRS	=$(wildcard $(SRC)/$(MODULE)*.h)
SRCS	=$(wildcard $(SRC)/$(MODULE)*.c)
OBJS	=$(patsubst $(SRC)/%.c, $(OBJ)/%.o, $(SRCS))

# Tests sources, object and binary files
TSTS	=$(wildcard $(SRC)/test_*.c)
TSTB	=$(patsubst $(SRC)/%.c, $(BLD)/%, $(TSTS))

# Install/uninstall instructions
INSTALL=install
INSTDIR=/opt
prefix=/usr
INCDIR=$(prefix)/include
BINDIR=$(prefix)/bin
LIBDIR=$(prefix)/lib

$(TST):
	mkdir -p $@

$(BLD):
	mkdir -p $@

$(OBJ):
	mkdir -p $@

$(OBJ)/%.o: $(SRC)/%.c | $(OBJ)
	$(CC) $(CPPFLAGS) $(CFLAGS) -c $< -o $@

$(TST)/test_%: $(SRC)/test_%.c $(SRC)/%.c | $(OBJS) $(TST)
	$(CC) $(CPPFLAGS) $(CFLAGS) $^ -o $@ $(LIBS)
	$@


partial: $(OBJS)
	ld -r $^ -o $(BLD)/$(MODULE).o

static: $(OBJS)
	ar rcs $(BLD)/lib$(MODULE).a $^

shared: $(OBJS)
	$(CC) $(CFLAGS) --shared $^ -o $(BLD)/lib$(MODULE).so $(LIBS)

test: $(TSTB)
	$<

module: partial static shared

all:  test module

clean:
	rm -rf $(BLD) $(BLD)

install:
	sudo $(INSTALL) -d --owner=root --group=root $(INSTDIR)/lib$(MODULE)
	sudo cp -r $(wildcard $(BLD)/*) $(INSTDIR)/lib$(MODULE)
	sudo $(INSTALL) -D --owner=root --group=root $(BLD)/lib$(MODULE).so $(LIBDIR)/lib$(MODULE).so.$(APIV).$(MODV)
	sudo ln -sf $(LIBDIR)/lib$(MODULE).so.$(APIV).$(MODV) $(LIBDIR)/lib$(MODULE).so.$(APIV)
	sudo ln -sf $(LIBDIR)/lib$(MODULE).so.$(APIV) $(LIBDIR)/lib$(MODULE).so
	sudo mkdir -p $(INSTDIR)/lib$(MODULE)/include
	sudo cp -r $(HDRS) $(INSTDIR)/lib$(MODULE)/include
	sudo mkdir -p $(INCDIR)/lib$(MODULE)
	sudo cp -r $(HDRS) $(INCDIR)/lib$(MODULE)

uninstall:
	sudo rm -rf $(INSTDIR)/lib$(MODULE)
	sudo rm $(LIBDIR)/lib$(MODULE).so.$(APIV).$(MODV)
	sudo rm $(LIBDIR)/lib$(MODULE).so.$(APIV)
	sudo rm $(LIBDIR)/lib$(MODULE).so
	sudo rm -rf $(INCDIR)/lib$(MODULE)

remove: uninstall

.PHONY: all clean install uninstall

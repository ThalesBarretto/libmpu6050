CC	=gcc
CPPFLAGS=
CFLAGS	=-g -O2 -fno-strict-aliasing -fPIC -Wall -Wextra -Wpedantic
DBGFLAGS=-DMPU6050_DEBUG
LIBS	=-lm -li2c
MODULE	=libmpu6050
MODV	=0
APIV	=0

# Sources, build and test directories
SRC	=src
BLD	=bld
OBJ	=$(BLD)/obj
TST	=$(BLD)/tst

# Headers and sources
HDRS	=$(wildcard $(SRC)/*.h)
SRCS	=$(wildcard $(SRC)/*.c)
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
PUBHDR=$(wildcard $(SRC)/*_public.h)

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

manpages:
	-cd man && make && cd ..

manpages_install:
	-cd man && make install && cd ..

manpages_uninstall:
	-cd man && make uninstall && cd ..

manpages_clean:
	-cd man && make clean && cd ..

partial: $(OBJS)
	ld -r $^ -o $(BLD)/$(MODULE).o

static: $(OBJS)
	ar rcs $(BLD)/$(MODULE).a $^

shared: $(OBJS)
	$(CC) $(CFLAGS) --shared $^ -o $(BLD)/$(MODULE).so $(LIBS)

test: $(TSTB)
	$<

module: partial static shared

all:  test module manpages

clean: manpages_clean
	rm -rf $(BLD) $(BLD)

install: manpages_install
	sudo $(INSTALL) -d --owner=root --group=root $(INSTDIR)/$(MODULE)
	sudo cp -r $(wildcard $(BLD)/*) $(INSTDIR)/$(MODULE)
	sudo $(INSTALL) -D --owner=root --group=root $(BLD)/$(MODULE).so $(LIBDIR)/$(MODULE).so.$(APIV).$(MODV)
	sudo ln -sf $(LIBDIR)/$(MODULE).so.$(APIV).$(MODV) $(LIBDIR)/$(MODULE).so.$(APIV)
	sudo ln -sf $(LIBDIR)/$(MODULE).so.$(APIV) $(LIBDIR)/$(MODULE).so
	sudo mkdir -p $(INSTDIR)/$(MODULE)/include
	sudo cp -r $(PUBHDR) $(INSTDIR)/$(MODULE)/include
	sudo mkdir -p $(INCDIR)/$(MODULE)
	sudo $(INSTALL) -D --owner=root --group=root --mode=0644 $(PUBHDR) $(INCDIR)/$(MODULE)/$(MODULE).h

uninstall: manpages_uninstall
	sudo rm -rf $(INSTDIR)/$(MODULE)
	sudo rm $(LIBDIR)/$(MODULE).so.$(APIV).$(MODV)
	sudo rm $(LIBDIR)/$(MODULE).so.$(APIV)
	sudo rm $(LIBDIR)/$(MODULE).so
	sudo rm -rf $(INCDIR)/$(MODULE)

remove: uninstall

.PHONY: all clean install uninstall

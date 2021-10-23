CC	=gcc
CPPFLAGS=
CFLAGS	=-fPIC -Wall -Wextra -Wpedantic
DBGFLAGS=
LIBS	=-lm -li2c
MODULE	=mpu6050

SRC	=src
BLD	=bld
OBJ	=$(BLD)/obj

# Headers and sources
HDRS	=$(wildcard $(SRC)/$(MODULE)*.h)
SRCS	=$(wildcard $(SRC)/$(MODULE)*.c)
OBJS	=$(patsubst $(SRC)/%.c, $(OBJ)/%.o, $(SRCS))

# Tests sources, object and binary files
TSTS	=$(wildcard $(SRC)/test_*.c)
TSTO	=$(patsubst $(SRC)/%.c, $(OBJ)/%.o, $(TSTS))
TSTB	=$(patsubst $(SRC)/%.c, $(BLD)/%, $(TSTS))

$(BLD):
	mkdir -p $@

$(OBJ):
	mkdir -p $@

$(OBJ)/%.o: $(SRC)/%.c | $(OBJ)
	$(CC) $(CPPFLAGS) $(CFLAGS) -c $< -o $@

$(BLD)/test_%: $(SRC)/test_%.c $(SRC)/%.c | $(OBJS)
	$(CC) $(CPPFLAGS) $(CFLAGS) -o $@ $^ $(LIBS)

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
	rm -rf $(BLD)

.PHONY: all clean

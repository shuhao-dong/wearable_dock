# ---- Toolchain ------------------------------
CC 		?= cc
CFLAGS 	:= -Wall -Wextra -std=c11 -O2
LDFLAGS := 
LIBS	:= -ludev -lmosquitto 

# ---- Sources --------------------------------
SRC		:= wearable_dock.c
OBJ		:= $(SRC:.c=.o)
BIN		:= wearable_dock_run 

$(BIN): $(OBJ)
	$(CC) $(OBJ) $(LDFLAGS) $(LIBS) -o $@

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

.PHONY: clean debug

clean:
	rm -f $(BIN) $(OBJ)

debug: CFLAGS := -Wall -Wextra -std=c11 -O0 -g
debug: clean $(BIN)

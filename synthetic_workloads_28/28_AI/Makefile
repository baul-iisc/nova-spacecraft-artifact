# Spacecraft AI - Makefile
CC      = gcc
CFLAGS  = -O2 -std=c11 -Wall -Wextra -Wpedantic \
          -fno-common -ffunction-sections -fdata-sections \
          -Wno-stringop-overflow -Wno-stringop-overread
LDFLAGS = -lm

# ARM bare-metal cross-compile (uncomment for target hardware)
# CC      = arm-none-eabi-gcc
# CFLAGS += -mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard
# LDFLAGS = --specs=nosys.specs -lm

TARGET  = spacecraft_ai

SRCS    = spacecraft_ai_main.c      \
          spacecraft_ai_workloads.c \
          spacecraft_ai_math.c      \
          spacecraft_ai_weights.c

OBJS    = $(SRCS:.c=.o)

.PHONY: all clean run size

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)
	@echo "Build complete -> $(TARGET)"

%.o: %.c spacecraft_ai.h
	$(CC) $(CFLAGS) -c -o $@ $<

run: $(TARGET)
	./$(TARGET)

size: $(TARGET)
	size $(TARGET)

clean:
	rm -f $(OBJS) $(TARGET)

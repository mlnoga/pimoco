TARGET=pimoco

SRCS=$(wildcard *.cpp)
OBJS=$(patsubst %.cpp,%.o,$(SRCS))
DEPS=$(patsubst %.cpp,%.d,$(SRCS))

CFLAGS=-Wall
LFLAGS=
GPP=g++

all: $(TARGET)

install:
	sudo cp $(TARGET) /usr/local/bin

clean:
	rm -f $(TARGET)

realclean: clean
	rm -f $(OBJS) $(DEPS)

$(TARGET): $(OBJS)
	$(GPP) -o $@ $(LFLAGS) $(OBJS)

# Compile .cpp source into .o object, and create .d dependency file via option -MMD
%.o: %.cpp
	$(GPP) -o $@ -MMD $(CFLAGS) -c $<

# Include dependency files if present, else ignore silently
-include $(DEPS)

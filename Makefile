TARGET_TEST=test_stepper
SRCS_TEST=main.cpp  pimoco_spi.cpp  pimoco_stepper.cpp  pimoco_tmc5160.cpp
OBJS_TEST=$(patsubst %.cpp,%.o,$(SRCS_TEST))
DEPS_TEST=$(patsubst %.cpp,%.d,$(SRCS_TEST))
LFLAGS_TEST=-lindidriver

TARGET_FOCUSER=indi_pimoco_focuser
SRCS_FOCUSER=pimoco_focuser.cpp  pimoco_spi.cpp  pimoco_stepper.cpp  pimoco_tmc5160.cpp
OBJS_FOCUSER=$(patsubst %.cpp,%.o,$(SRCS_FOCUSER))
DEPS_FOCUSER=$(patsubst %.cpp,%.d,$(SRCS_FOCUSER))
LFLAGS_FOCUSER=-lindidriver

TARGET_MOUNT=indi_pimoco_mount
SRCS_MOUNT=pimoco_mount.cpp  pimoco_spi.cpp  pimoco_stepper.cpp  pimoco_tmc5160.cpp
OBJS_MOUNT=$(patsubst %.cpp,%.o,$(SRCS_MOUNT))
DEPS_MOUNT=$(patsubst %.cpp,%.d,$(SRCS_MOUNT))
LFLAGS_MOUNT=-lindidriver


CFLAGS=-Wall
CXX=g++

all: $(TARGET_TEST) $(TARGET_FOCUSER)

install:
	sudo cp $(TARGET_FOCUSER) /usr/local/bin

clean:
	rm -f $(TARGET_TEST) $(TARGET_FOCUSER)

realclean: clean
	rm -f $(OBJS_TEST) $(DEPS_TEST) $(OBJS_FOCUSER) $(DEPS_FOCUSER)

count:
	wc -l *.cpp *.h

test: $(TARGET_TEST)
	./$(TARGET_TEST)

serve: $(TARGET_FOCUSER) $(TARGET_MOUNT)
	indiserver -v ./$(TARGET_FOCUSER) ./$(TARGET_MOUNT)

$(TARGET_TEST): $(OBJS_TEST)
	$(CXX) -o $@ $(LFLAGS_TEST) $(OBJS_TEST)

$(TARGET_FOCUSER): $(OBJS_FOCUSER)
	$(CXX) -o $@ $(LFLAGS_FOCUSER) $(OBJS_FOCUSER)

$(TARGET_MOUNT): $(OBJS_MOUNT)
	$(CXX) -o $@ $(LFLAGS_MOUNT) $(OBJS_MOUNT)

# Compile .cpp source into .o object, and create .d dependency file via option -MMD
%.o: %.cpp
	$(CXX) -o $@ -MMD $(CFLAGS) -c $<

# Include dependency files if present, else ignore silently
-include $(DEPS_TEST) $(DEPS_FOCUSER)

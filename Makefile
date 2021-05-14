TARGET_FOCUSER=indi_pimoco_focuser
SRCS_FOCUSER=pimoco_focuser.cpp  pimoco_spi.cpp  pimoco_stepper.cpp  pimoco_tmc5160.cpp
OBJS_FOCUSER=$(patsubst %.cpp,%.o,$(SRCS_FOCUSER))
DEPS_FOCUSER=$(patsubst %.cpp,%.d,$(SRCS_FOCUSER))
LFLAGS_FOCUSER=-lindidriver -lwiringPi

TARGET_MOUNT=indi_pimoco_mount
SRCS_MOUNT=pimoco_mount.cpp  pimoco_mount_ui.cpp pimoco_mount_timer.cpp \
           pimoco_mount_track.cpp  pimoco_mount_move.cpp  pimoco_mount_guide.cpp  pimoco_mount_goto.cpp  pimoco_mount_park.cpp  \
           pimoco_mount_limits.cpp  pimoco_spi.cpp  pimoco_stepper.cpp  pimoco_tmc5160.cpp
OBJS_MOUNT=$(patsubst %.cpp,%.o,$(SRCS_MOUNT))
DEPS_MOUNT=$(patsubst %.cpp,%.d,$(SRCS_MOUNT))
LFLAGS_MOUNT=-lindidriver -lnova -lwiringPi


CFLAGS=-Wall
CXX=g++

all: $(TARGET_FOCUSER) $(TARGET_MOUNT) test

# Indi requires drivers to be installed into /usr/bin, unfortunately the more suitable /usr/local/bin doesn't work
install: $(TARGET_FOCUSER) $(TARGET_MOUNT)
	sudo cp $(TARGET_FOCUSER) $(TARGET_MOUNT) /usr/bin/
	sudo cp indi_pimoco.xml /usr/share/indi/

clean:
	rm -f $(TARGET_TEST) $(TARGET_FOCUSER) $(TARGET_MOUNT) test

realclean: clean
	rm -f $(OBJS_TEST) $(DEPS_TEST) $(OBJS_FOCUSER) $(DEPS_FOCUSER) $(OBJS_MOUNT) $(DEPS_MOUNT)

count:
	wc -l *.cpp *.h

serve: $(TARGET_FOCUSER) $(TARGET_MOUNT)
	indiserver -v ./$(TARGET_FOCUSER) ./$(TARGET_MOUNT)

$(TARGET_FOCUSER): $(OBJS_FOCUSER)
	$(CXX) -o $@ $(LFLAGS_FOCUSER) $(OBJS_FOCUSER)

$(TARGET_MOUNT): $(OBJS_MOUNT)
	$(CXX) -o $@ $(LFLAGS_MOUNT) $(OBJS_MOUNT)

test: test.cpp
	$(CXX) -o $@ -Wall $<

# Compile .cpp source into .o object, and create .d dependency file via option -MMD
%.o: %.cpp
	$(CXX) -o $@ -MMD $(CFLAGS) -c $<

# Include dependency files if present, else ignore silently
-include $(DEPS_FOCUSER) $(DEPS_MOUNT) 

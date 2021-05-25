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

TARGETS=$(TARGET_FOCUSER) $(TARGET_MOUNT) spi0-3cs.dtbo spi0-4cs.dtbo spitest

CFLAGS=-Wall
CXX=g++

all: $(TARGETS)

# Indi requires drivers to be installed into /usr/bin, unfortunately the more suitable /usr/local/bin doesn't work
install: $(TARGET_FOCUSER) $(TARGET_MOUNT)
	sudo cp $(TARGET_FOCUSER) $(TARGET_MOUNT) /usr/bin/
	sudo cp indi_pimoco.xml /usr/share/indi/
	sudo cp spi0-3cs.dtbo spi0-4cs.dtbo /boot/overlays/

clean:
	rm -f $(TARGETS)

realclean: clean
	rm -f $(OBJS_TEST) $(DEPS_TEST) $(OBJS_FOCUSER) $(DEPS_FOCUSER) $(OBJS_MOUNT) $(DEPS_MOUNT)

count:
	wc -l *.cpp *.h

spitests: spitest
	./spitest -D /dev/spidev0.0 -s 4000000 -b 8 -d 0 -H -O
	./spitest -D /dev/spidev0.1 -s 4000000 -b 8 -d 0 -H -O
	./spitest -D /dev/spidev0.2 -s 4000000 -b 8 -d 0 -H -O
	./spitest -D /dev/spidev0.3 -s 4000000 -b 8 -d 0 -H -O

serve: $(TARGET_FOCUSER) $(TARGET_MOUNT)
	indiserver -v ./$(TARGET_FOCUSER) ./$(TARGET_MOUNT)

$(TARGET_FOCUSER): $(OBJS_FOCUSER)
	$(CXX) -o $@ $(LFLAGS_FOCUSER) $(OBJS_FOCUSER)

$(TARGET_MOUNT): $(OBJS_MOUNT)
	$(CXX) -o $@ $(LFLAGS_MOUNT) $(OBJS_MOUNT)

spitest: spitest.cpp
	$(CXX) -o $@ -Wall $<

# Compile .cpp source into .o object, and create .d dependency file via option -MMD
%.o: %.cpp
	$(CXX) -o $@ -MMD $(CFLAGS) -c $<

# Compile DTS device tree specification into DTBO device tree binary overlay
%.dtbo: %-overlay.dts
	dtc -@ -I dts -O dtb -o $@ $<

# Include dependency files if present, else ignore silently
-include $(DEPS_FOCUSER) $(DEPS_MOUNT) 

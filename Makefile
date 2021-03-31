TARGET_TEST=test
SRCS_TEST=main.cpp  pimoco_spi.cpp  pimoco_stepper.cpp  pimoco_tmc5160.cpp
OBJS_TEST=$(patsubst %.cpp,%.o,$(SRCS_TEST))
DEPS_TEST=$(patsubst %.cpp,%.d,$(SRCS_TEST))
LFLAGS_TEST=

TARGET_FOCUSER=indi_pimoco_focuser
SRCS_FOCUSER=pimoco_focuser.cpp  pimoco_spi.cpp  pimoco_stepper.cpp  pimoco_tmc5160.cpp
OBJS_FOCUSER=$(patsubst %.cpp,%.o,$(SRCS_FOCUSER))
DEPS_FOCUSER=$(patsubst %.cpp,%.d,$(SRCS_FOCUSER))
LFLAGS_FOCUSER=-lindidriver

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

$(TARGET_TEST): $(OBJS_TEST)
	$(CXX) -o $@ $(LFLAGS_TEST) $(OBJS_TEST)

$(TARGET_FOCUSER): $(OBJS_FOCUSER)
	$(CXX) -o $@ $(LFLAGS_FOCUSER) $(OBJS_FOCUSER)


# Compile .cpp source into .o object, and create .d dependency file via option -MMD
%.o: %.cpp
	$(CXX) -o $@ -MMD $(CFLAGS) -c $<

# Include dependency files if present, else ignore silently
-include $(DEPS_TEST) $(DEPS_FOCUSER)

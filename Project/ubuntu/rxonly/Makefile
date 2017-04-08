CC=g++
TARGET=rx
FEC=../../../modules/YAL/fec
HAL3288=../../../HAL/rk32885.1
ROOT=../../..
CFLAGS=-mssse3

SOURCE = rx.cpp \
		$(FEC)/append.cpp \
		$(FEC)/frame.cpp \
		$(FEC)/GFMath.cpp \
		$(FEC)/oRS.cpp \
		$(FEC)/cauchy_256.cpp \
		$(FEC)/MemXOR.cpp \
		$(FEC)/MemSwap.cpp \
		$(FEC)/sender.cpp \
		$(FEC)/reciever.cpp \
		$(HAL3288)/Apcap.cpp \
		$(HAL3288)/radiotap.cpp \

## End sources definition
INCLUDE = -I../../../ -I../../../modules
## end more includes

LIBS = $(shell pkg-config --libs libavcodec) -lpcap

VPATH=$(FEC) $(HAL3288)
OBJ=$(join $(addsuffix ../obj/, $(dir $(SOURCE))), $(notdir $(SOURCE:.cpp=.o))) 

## Fix dependency destination to be ../.dep relative to the src dir
DEPENDS=$(join $(addsuffix ../.dep/, $(dir $(SOURCE))), $(notdir $(SOURCE:.cpp=.d)))

## Default rule executed
all: $(TARGET)
		@true

## Clean Rule
clean:
		@-rm -f $(TARGET) $(OBJ) $(DEPENDS)

## Rule for making the actual target
$(TARGET): $(OBJ)
		@echo "============="
		@echo "Linking the target $@"
		@echo "============="
		@$(CC) $(CFLAGS) -o $@ $^ $(LIBS)
		@strip $@
		@echo -- Link finished --

## Generic compilation rule
%.o : %.cpp
		@mkdir -p $(dir $@)
		@echo "Compiling $<"
		$(CC) $(CFLAGS) -cccccc $< -o $@ $(INCLUDE)


## Rules for object files from cpp files
## Object file for each file is put in obj directory
## one level up from the actual source directory.
../obj/%.o : %.cpp
		@mkdir -p $(dir $@)
		@echo "Compiling $<"
		@$(CC) $(CFLAGS) -c $< -o $@ $(INCLUDE)

# Rule for "other directory"  You will need one per "other" dir
$(HAL3288)/../obj/%.o : %.cpp
		@mkdir -p $(dir $@)
		@echo "Compiling $<"
		@$(CC) $(CFLAGS) -c $< -o $@ $(INCLUDE)

# Rule for "other directory"  You will need one per "other" dir
$(FEC)/../obj/%.o : %.cpp
		@mkdir -p $(dir $@)
		@echo "Compiling $<"
		@$(CC) $(CFLAGS) -c $< -o $@ $(INCLUDE)

## Make dependancy rules
../.dep/%.d: %.cpp
		@mkdir -p $(dir $@)
		@echo "============="
		@echo Building dependencies file for $*.o
		@$(SHELL) -ec '$(CC) -M $(CFLAGS) $< | sed "s^$*.o^../obj/$*.o^" > $@'

## Dependency rule for "other" directory
$(FEC)/../.dep/%.d: %.cpp
		@mkdir -p $(dir $@)
		@echo "============="
		@echo Building dependencies file for $*.o
		@$(SHELL) -ec '$(CC) -M $(CFLAGS) $< | sed "s^$*.o^$(OTHERDIR)/../obj/$*.o^" > $@'

## Dependency rule for "other" directory
$(HAL3288)/../.dep/%.d: %.cpp
		@mkdir -p $(dir $@)
		@echo "============="
		@echo Building dependencies file for $*.o
		@$(SHELL) -ec '$(CC) -M $(CFLAGS) $< | sed "s^$*.o^$(OTHERDIR)/../obj/$*.o^" > $@'

## Include the dependency files
-include $(DEPENDS)
#########################################################################
#
# Generic Multi-Application Makefile (g++)
#
# Version 20261030
# Copyright (c) Robert Damerius
#
#########################################################################


# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# Toolchain
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
DEBUG_MODE := 0
CC         := g++
CPP        := g++
LD         := ld
MV         := mv
RM         := rm -f -r
MKDIR      := mkdir -p


# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# Project settings
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# Directories
DIRECTORY_EXAMPLES          := examples/
DIRECTORY_BUILD             := build/
DIRECTORY_INCLUDE           := include/
SUBDIRECTORY_EXAMPLE_SOURCE := source/

# Compiler flags
CC_FLAGS        = -Wall -Wextra -fopenmp -mtune=native
CPP_FLAGS       = -Wall -Wextra -fopenmp -mtune=native -std=c++23
LD_FLAGS        = -Wall -Wextra -mtune=native
LIBS_WINDOWS   := -lstdc++ -lpthread -lgomp
LIBS_LINUX     := -lstdc++ -lpthread -lgomp
DEP_FLAGS       = -MT $@ -MMD -MP -MF $(DIRECTORY_BUILD)$*.Td
POSTCOMPILE     = $(MV) -f $(DIRECTORY_BUILD)$*.Td $(DIRECTORY_BUILD)$*.d
CC_SYMBOLS      = 
ifeq ($(DEBUG_MODE), 1)
    CC_FLAGS   += -ggdb
    CPP_FLAGS  += -ggdb
    CC_SYMBOLS += -DDEBUG
else
    CC_FLAGS   += -O3
    CPP_FLAGS  += -O3
    LD_FLAGS   += -O3 -s
endif
ifeq ($(OS), Windows_NT)
    LD_LIBS    := -Wl,--as-needed -static-libgcc -static-libstdc++ -Wl,-Bstatic $(LIBS_WINDOWS) -Wl,-Bdynamic
else
    LD_LIBS    := -Wl,--as-needed $(LIBS_LINUX)
endif

# Recursive wildcard function
rwildcard       = $(wildcard $1$2) $(foreach d,$(wildcard $1*),$(call rwildcard,$d/,$2))


# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# Find all sources
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# Library content
DIRECTORY_ALL   = $(dir $(call rwildcard,$(DIRECTORY_INCLUDE),.))
SOURCES_BIN     = $(call rwildcard,$(DIRECTORY_INCLUDE),*.bin)
SOURCES_C       = $(call rwildcard,$(DIRECTORY_INCLUDE),*.c)
SOURCES_CPP     = $(call rwildcard,$(DIRECTORY_INCLUDE),*.cpp)

# Example content
ifneq ($(example), )
    DIRECTORY_EXAMPLE_SOURCE = $(DIRECTORY_EXAMPLES)$(example)/$(SUBDIRECTORY_EXAMPLE_SOURCE)
    DIRECTORY_ALL          += $(dir $(call rwildcard,$(DIRECTORY_EXAMPLE_SOURCE),.))
    SOURCES_BIN            += $(call rwildcard,$(DIRECTORY_EXAMPLE_SOURCE),*.bin)
    SOURCES_C              += $(call rwildcard,$(DIRECTORY_EXAMPLE_SOURCE),*.c)
    SOURCES_CPP            += $(call rwildcard,$(DIRECTORY_EXAMPLE_SOURCE),*.cpp)
endif

# Include and library paths
INCLUDE_PATHS   = -I/usr/include -I/usr/local/include $(addprefix -I,$(DIRECTORY_ALL))
LIBRARY_PATHS   = -L/usr/lib -L/usr/local/lib

# Object files
OBJECTS_BIN := $(SOURCES_BIN:.bin=.o)
OBJECTS_C   := $(SOURCES_C:.c=.o)
OBJECTS_CPP := $(SOURCES_CPP:.cpp=.o)
OBJECTS_ALL  = $(addprefix $(DIRECTORY_BUILD), $(OBJECTS_BIN) $(OBJECTS_C) $(OBJECTS_CPP))

# Final product
PRODUCT      = header-only
LINK_MESSAGE = header-only library is ready.\n
LINK_COMMAND = 
ifneq ($(example), )
    PRODUCT = $(DIRECTORY_EXAMPLES)$(example)/$(example)
    LINK_MESSAGE = [APP]  > $@\n
    LINK_COMMAND = $(CC) $(LD_FLAGS) $(LIBRARY_PATHS) -o $@ $^ $(LD_LIBS)
endif

# Create build folder
$(shell $(MKDIR) $(DIRECTORY_BUILD) $(addprefix $(DIRECTORY_BUILD), $(DIRECTORY_ALL)))


# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# Make targets
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
.PHONY: all examples clean

all: $(PRODUCT)

examples:
	@echo "Building benchmark"
	@make -j8 --no-print-directory example=benchmark
	@echo ""
	@echo "Building pathplanning"
	@make -j8 --no-print-directory example=pathplanning
	@echo ""
	@echo "Building motionplanning"
	@make -j8 --no-print-directory example=motionplanning
	@echo ""
	@echo "Building sequentialplanning"
	@make -j8 --no-print-directory example=sequentialplanning
	@echo ""
	@echo "Building onlineplanning"
	@make -j8 --no-print-directory example=onlineplanning
	@echo ""
	@echo "Building asynconlineplanning"
	@make -j8 --no-print-directory example=asynconlineplanning
	@echo ""
	@echo "Building asynconlineplanning2"
	@make -j8 --no-print-directory example=asynconlineplanning2

clean:
	@$(RM) $(DIRECTORY_BUILD)
	@echo "Clean: Done."

$(PRODUCT): $(OBJECTS_ALL)
	@printf "$(LINK_MESSAGE)"
	@$(LINK_COMMAND)

$(DIRECTORY_BUILD)%.o: %.c
$(DIRECTORY_BUILD)%.o: %.c $(DIRECTORY_BUILD)%.d
	@printf "[C]    > $<\n"
	@$(CC) $(INCLUDE_PATHS) $(CC_FLAGS) $(DEP_FLAGS) -o $@ -c $< $(CC_SYMBOLS)
	@$(POSTCOMPILE)

$(DIRECTORY_BUILD)%.o: %.cpp
$(DIRECTORY_BUILD)%.o: %.cpp $(DIRECTORY_BUILD)%.d
	@printf "[CPP]  > $<\n"
	@$(CPP) $(INCLUDE_PATHS) $(CPP_FLAGS) $(DEP_FLAGS) -o $@ -c $< $(CC_SYMBOLS)
	@$(POSTCOMPILE)

$(DIRECTORY_BUILD)%.o: %.bin
	@printf "[BIN]  > $<\n"
	@$(LD) -r -b binary -o $@ $<

$(DIRECTORY_BUILD)%.d: ;
.PRECIOUS: $(DIRECTORY_BUILD)%.d

-include $(patsubst %,$(DIRECTORY_BUILD)%.d,$(basename $(SOURCES_C) $(SOURCES_CPP)))

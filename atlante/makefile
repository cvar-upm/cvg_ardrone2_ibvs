#
# Atlante framework makefile
# Author: Ignacio Mellado
# Last revision: 11/11/2010
#
# IMPORTANT: If you are a makefile rookie, please just modify variables in the area 
# marked as "customizable parameters".
#
# Description: 
# It builds the framework library compiling every valid source file in the source 
# directory. Valid source files have any of the following extensions: c, cpp, cc, c++.
# Each source depends on all the valid header files. Valid header files are located 
# in the include path and have any of the extensions: h, hpp, h++.
#
# The followig targets are defined:
#   all - It compiles all source files and links them to generate the executable.
#   clean - It deletes the executable and all compiled objects.
#

# --------- Start of customizabe parameters ---------
# A name identifying the platform for which this makefile works. This label will name
# the subdirectories containing binary files for the platform (objects and library)
PLATFORM = gcc
# The path on which all paths are referenced
ROOTPATH = .
# The path where the source files reside
SOURCEPATH = $(ROOTPATH)/sources
# The path where the header files are
INCLUDEPATH = $(ROOTPATH)/include
# Where to save the compiled objects
OBJPATH = $(ROOTPATH)/obj/$(PLATFORM)
# Where to save the built library
LIBPATH = $(ROOTPATH)/lib/$(PLATFORM)
# Where 3rd party products are
3RDPARTYPATH = $(ROOTPATH)/3rdparty

# Compiler to be used
CC = g++
# Compilation options
CFLAGS = -c -O
# Include parameters
INCLUDES = -I/usr/include -I$(3RDPARTYPATH)/poshlib -I$(INCLUDEPATH)
# Linking parameters
LDFLAGS =
# ---------- End of customizable parameters ----------

# Allowed source and header extensions.
# The modification of this list is not enough to add/delete valid extensions. 
# Some lines below implement actions based on extensions and do not use this variables.
SOURCE_EXTENSIONS := cpp cc c++ c
HEADER_EXTENSIONS := h h++ hpp

#vpath $(foreach EXT,$(SOURCE_EXTENSIONS),%.$(EXT)) $(SOURCEPATH)
#vpath $(foreach EXT,$(HEADER_EXTENSIONS),%.$(EXT)) $(INCLUDEPATH)
#vpath $(foreach EXT,$(HEADER_EXTENSIONS),%%.$(EXT)o) $(OBJPATH)
vpath %.cpp $(SOURCEPATH)
vpath %.c $(SOURCEPATH)
vpath %.c++ $(SOURCEPATH)
vpath %.cc $(SOURCEPATH)
vpath %.h $(INCLUDEPATH)
vpath %.hpp $(INCLUDEPATH)
vpath %.h++ $(INCLUDEPATH)
vpath %.cppo $(OBJPATH)
vpath %.co $(OBJPATH)
vpath %.cco $(OBJPATH)
vpath %.c++o $(OBJPATH)

SOURCE_MASKS_SEARCH := $(addprefix *., $(SOURCE_EXTENSIONS))
HEADER_MASKS_SEARCH := $(addprefix *., $(HEADER_EXTENSIONS))
#TARGET_MASKS := $(foreach EXT,$(SOURCE_EXTENSIONS),.$(EXT).o)

_WINNAME := $(shell set OS)
GOTWINDOWS = $(patsubst OS=%,yes,$(_WINNAME))

SOURCES:=$(wildcard $(addprefix $(SOURCEPATH)/,$(SOURCE_MASKS_SEARCH))) $(3RDPARTYPATH)/poshlib/posh.c
HEADERS:=$(wildcard $(addprefix $(INCLUDEPATH)/,$(HEADER_MASKS_SEARCH)))

OBJECTS := $(strip $(foreach EXT,$(SOURCE_EXTENSIONS),$(patsubst %.$(EXT),%.$(EXT)o,$(filter %.$(EXT),$(notdir $(SOURCES))))))
OBJECTSWITHPATH := $(addprefix $(OBJPATH)/,$(OBJECTS))

LIBRARY = $(LIBPATH)/libatlante.a

.PHONY: all clean

all: $(LIBRARY) 

clean:
	@echo Cleaning objects...
ifeq ($(GOTWINDOWS), yes)
	@del /Q $(subst /,\,$(OBJECTSWITHPATH))
	@del /Q $(subst /,\,$(LIBRARY))
else
	@rm -f $(OBJECTSWITHPATH)
	@rm -f $(LIBRARY)
endif

$(LIBRARY): $(OBJECTS)
	@echo Creating library $@...	
	@$(AR) rcs $(LIBRARY) $(OBJECTSWITHPATH)

#$(TARGET_MASKS): $(HEADERS)
#	@echo Compiling [$<]...
#	@$(CC) $(CFLAGS) $< -o $(OBJPATH)/$(notdir $@) $(INCLUDES) $(LDFLAGS)

%.co: %.c
	@echo Compiling [$<]...
	@$(CC) $(CFLAGS) $< -o $(OBJPATH)/$(notdir $@) $(INCLUDES)

%.cppo: %.cpp
	@echo Compiling [$<]...
	@$(CC) $(CFLAGS) $< -o $(OBJPATH)/$(notdir $@) $(INCLUDES)

%.cco: %.cc
	@echo Compiling [$<]...
	@$(CC) $(CFLAGS) $< -o $(OBJPATH)/$(notdir $@) $(INCLUDES)

%.c++o: %.c++
	@echo Compiling [$<]...
	@$(CC) $(CFLAGS) $< -o $(OBJPATH)/$(notdir $@) $(INCLUDES)

posh.co: $(wildcard $(3RDPARTYPATH)/poshlib/*.c) $(wildcard $(3RDPARTYPATH)/poshlib/*.h)
	@echo Compiling [$<]...
	@$(CC) $(CFLAGS) $< -o $(OBJPATH)/$(notdir $@) $(INCLUDES) $(LDFLAGS)



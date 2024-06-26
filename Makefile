# Phil's multiplatform makefile template
# With auto-incrementing build number and automatic version.h generation
# Version 1.11, 2010-02-15
#
# The latest version of this Makefile can be found at http://www.philpem.me.uk/
#
#
# Copyright (c) 2010 Philip Pemberton <code@philpem.me.uk>
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
#
#
# Instructions for use:
#   Run 'make init' to create the required directories
#   Add your source files to the 'SOURCES' list, and change the TARGET filename
#   Set the desired build type and platform in the BUILD_TYPE and PLATFORM
#     variables respectively
#   Set your project type (C only, or C++) in the SRC_TYPE variable
#   Add any libraries you need to link against to the 'LIB' list
#   Run 'make'
#
# Object files are created in the 'obj' subdirectory, from source code in the
# 'src' directory. Dependency files are created in the 'dep' directory from
# the same source code the object files are created from.
#
# Supported targets are:
#   all                 Build everything.
#   update-revision     Increment the build number without building anything.
#   clean-versioninfo   Delete src/version.h (will be rebuilt on the next
#                       'make all').
#   init                Initialise the build system for a new project.
#                       WARNING: overwrites .buildnum and src/version.h.in!
#   cleandep            Delete all dependency files.
#   clean               Delete all dependency, intermediate and target files.
#   tidy                Delete all dependency and intermediate files, leaving
#                       the target file intact.
#
# If you want to reset the build number to zero, delete '.buildnum'. This
# should be done whenever the major or minor version changes. Excluding
# .buildnum from version control may also be a good idea, depending on how
# you want your build numbers to work.
#
# The BUILD_TYPE variable contains the current build type. There are two
# supported build types:
#   debug       Debug mode - object files are compiled with debug information
#               and the target is left unstripped.
#   release     Release mode - object files are not compiled with debug info,
#               and the target is fed through strip to remove redundant
#               data.
#
# The PLATFORM variable contains the current target platform. There are currently
# three supported platforms:
#   linux       GNU/Linux with GNU Compiler Collection
#   win32       Windows 32-bit with MinGW
#   osx         Mac OS X
#
# The EXTSRC variable is used to specify other files to build. It is typically
# used to specify platform or build-type specific source files, e.g.
#
# ifeq ($(BUILD_TYPE),debug-memwatch)
#   CFLAGS += -g -ggdb
#   CPPFLAGS += -DMEMWATCH
#   INCPATH += ./memwatch
#   EXTSRC += memwatch/memwatch.c
# endif
#
# (example taken from one of my projects that allowed the use of Memwatch to
#  track down memory allocation/deallocation bugs)
#
#
# Change history:
#   1.11- Add support for libraries which are registered with 'pkg-config'.
#         Removed ENABLE_GTK, ENABLE_CAIRO and ENABLE_SDL -- these can now be
#          specified as pkg-config managed libraries (gtk+-2.0, cairo and sdl,
#          respectively).
#   1.10- Add Mac OS X build support
#         Add automatic build platform identification
#         Bugfix -- if .buildnum doesn't exist, use '0' for the build number
#         Add ability to override wxWidgets build type from the command line
#           (OVERRIDE_WX_USE_RELEASE=1 forces use of wxWidgets release
#           libraries, even in debug mode).
#         Add GTK+ and Cairo library support
#   1.9 - Bugfix -- if CFLAGS contained a forward-slash, sed would fall over.
#         Also added SDL support and fixed the date/time formats. To use SDL,
#         set ENABLE_SDL to "yes".
#   1.8 - Now supports the use of the wxWidgets GUI framework. To turn
#         this on, set ENABLE_WX to "yes".
#   1.7 - Now creates a basic Hgignore file and directory keepers for the
#         dep and obj directories.
#   1.6 - Added CFLAGS and CXXFLAGS to the command-lines for the dependency
#         building commands. This was causing issues with C99 / C++0x mode.
#   1.5 - Added support for Mercurial revision (changeset ID) display
#         Fixed a few issues with Subversion support (svn: and version 0 would
#         be displayed for exported code)
#
#
# TODO list:
#   - Add a 'make help' option which lists valid make targets, platforms etc.
#   - Allow platform to be overridden (instead of using idplatform.sh)
#

####
# Build configuration
####

# version information -- major.minor.extra
# note that VER_EXTRA can be overridden on the command line, e.g.:
# make VER_EXTRA=12345 all
VER_MAJOR	= 0
VER_MINOR	= 0
VER_EXTRA	?= 

# build platform: win32, linux, osx
PLATFORM	?=	linux
# build type: release or debug
BUILD_TYPE	?=	debug

# target executable
TARGET		=	emutrak

# source files that produce object files
SRC			=	main.c uart.c datatrak_gen.c
SRC			+=	m68kcpu.c m68kdasm.c m68kops.c softfloat/softfloat.c

# source type - either "c" or "cpp" (C or C++)
SRC_TYPE	=	c

# additional object files that don't necessarily include source
EXT_OBJ		=

# List of libraries to link in -- these will be specified as "-l" parameters,
# the '-l' is prepended automatically
LIB			=	m

# List of libraries handled by pkg-config
LIBPKGC		=

# Library paths -- where to search for the above libraries
LIBPATH		=

# Include paths -- where to search for #include files (in addition to the
# standard paths
INCPATH		=

# Garbage files which should be deleted on a 'make clean' or 'make tidy'
GARBAGE		=	obj/m68kmake obj/m68kmake.exe obj/m68kmake.o

# extra dependencies - files that we don't necessarily know how to build, but
# that are required for building the application; e.g. object files or
# libraries in sub or parent directories
EXTDEP		=

# Extra libraries
# Set any of these variables to "yes" to enable, or anything else to disable
# --- wxWidgets ---
ENABLE_WX	=	no
# wxWidgets: list of wxWidgets libraries to enable
# Only valid if ENABLE_WX = yes
WX_LIBS		=	std


####
# Tool setup
####
MAKE	:=	make
CC		:=	gcc
CXX		:=	g++
CFLAGS	:=	-Wall -pedantic -std=gnu99 $(EXT_CFLAGS)
CXXFLAGS:=	-Wall -pedantic -std=gnu++0x $(EXT_CXXFLAGS)
LDFLAGS	:=	$(EXT_LDFLAGS)
RM		:=	rm
STRIP	:=	strip


####
# Win32 target-specific settings
####
ifeq ($(strip $(PLATFORM)),win32)
	# windows executables have a .exe suffix
	TARGET := $(addsuffix .exe,$(TARGET))
	# console mode application
	EXT_CFLAGS = -mconsole
endif


####
# OSX target-specific settings
####
ifeq ($(PLATFORM),osx)
CFLAGS	:=	-Wall -pedantic $(EXT_CFLAGS)
CXXFLAGS:=	-Wall -pedantic -Wno-long-long $(EXT_CXXFLAGS)
endif


###############################################################################
# You should not need to touch anything below here, unless you're adding a new
# platform or build type (or changing the version string format)
###############################################################################

####
# A quick sanity check on the platform type
####
PERMITTED_PLATFORMS=linux win32 osx
ifeq ($(filter $(PLATFORM),$(PERMITTED_PLATFORMS)),)
    $(error Platform '$(PLATFORM)' is not supported. Supported platforms are: $(PERMITTED_PLATFORMS))
endif

####
# Version info generation
####

#### --- begin Subversion revision grabber ---
# there are two ways to get the SVN revision - use svnversion, or use svn info
# then pipe through awk. which one you use is up to you.
VER_SVNREV		= $(shell LANG=C svn info 2>/dev/null || echo 'Revision: exported' | awk '/^Revision:/ { print$$2 }' )
#VER_SVNREV		= $(shell svnversion .)

# if the version string is "exported", then the CSD was not checked out of SVN
# note that if the CSD is not an SVN checkout, then @@svnrev@@ will be set to
# zero.
ifeq ($(VER_SVNREV),exported)
    VER_VCS		= none
    VER_VCSREV	= 0
else
    VER_VCS		= svn
    VER_VCSREV	= $(VER_SVNREV)
endif

#### --- begin Mercurial revision grabber ---
# If SVN didn't give us a revision, try Mercurial instead
ifeq ($(VER_VCS),none)
    # get the current Mercurial changeset number
	VER_HGREV=$(shell ((hg tip --template "{node|short}") || echo "000000000000") 2>/dev/null)
    ifneq ($(VER_HGREV),000000000000)
        # a non-empty repo
        VER_VCS		= hg
        VER_VCSREV	= $(VER_HGREV)
    else
        # either an empty Hg repo, or no repo at all
        VER_VCS		= none
        VER_VCSREV	= 0
    endif
endif

#### --- end version grabbers ---

# start creating the revision string
VER_FULLSTR		= $(VER_MAJOR).$(VER_MINOR)$(VER_EXTRA)

# if this is a VCS release, include the SVN revision in the version string
# also create a revision string that is either "svn:12345", "hg:12345" or
# blank
ifneq ($(VER_VCS),none)
    VER_FULLSTR	+= ($(VER_VCS) $(VER_VCSREV))
    VER_VCSSTR	= $(VER_VCS):$(VER_VCSREV)
else
    VER_VCSSTR	=
endif


####
# Build-type specific configuration
####
ifeq ($(BUILD_TYPE),debug)
	CFLAGS		+= -g -ggdb -DDEBUG
	CXXFLAGS	+= -g -ggdb -DDEBUG
else
 ifeq ($(BUILD_TYPE),release)
	CFLAGS		+= -O2
	CXXFLAGS	+= -O2
 else
 	$(error Unsupported build type: '$(BUILD_TYPE)')
 endif
endif

####
# wxWidgets support
####
ifeq ($(ENABLE_WX),yes)
	ifneq ($(OVERRIDE_WX_USE_RELEASE),)
		WX_BUILD_TYPE := release
	else
		WX_BUILD_TYPE := $(BUILD_TYPE)
	endif

	ifeq ($(WX_BUILD_TYPE),debug)
		LIBLNK		+=	`wx-config --debug --libs $(WX_LIBS)`
		CFLAGS		+=	`wx-config --debug --cflags $(WX_LIBS)`
		CXXFLAGS	+=	`wx-config --debug --cxxflags $(WX_LIBS)`
		CPPFLAGS	+=	`wx-config --debug --cppflags $(WX_LIBS)`
	else
		ifeq ($(WX_BUILD_TYPE),release)
			LIBLNK		+=	`wx-config --libs $(WX_LIBS)`
			CFLAGS		+=	`wx-config --cflags $(WX_LIBS)`
			CPPFLAGS	+=	`wx-config --cppflags $(WX_LIBS)`
			CXXFLAGS	+=	`wx-config --cxxflags $(WX_LIBS)`
		else
			$(error Unsupported build type: '$(BUILD_TYPE)')
		endif
	endif
endif


####
# rules
####

# object files
OBJ		=	$(addprefix obj/, $(addsuffix .o, $(basename $(SRC))) $(EXT_OBJ)) $(addsuffix .o, $(basename $(EXTSRC)))

# dependency files
DEPFILES =	$(addprefix dep/, $(addsuffix .d, $(basename $(SRC))) $(EXT_OBJ)) $(addsuffix .d, $(basename $(EXTSRC)))

# path commands
LIBLNK	+=	$(addprefix -l, $(LIB))
LIBPTH	+=	$(addprefix -L, $(LIBPATH))
INCPTH	+=	$(addprefix -I, $(INCPATH))

# Bring in libraries managed by pkg-config
ifneq ($(strip $(LIBPKGC)),)
    LIBLNK += $(shell pkg-config --libs $(LIBPKGC))
    CFLAGS += $(shell pkg-config --cflags $(LIBPKGC))
    CXXFLAGS += $(shell pkg-config --cflags $(LIBPKGC))
endif

CPPFLAGS +=	$(INCPTH)

####
# Make sure there is at least one object file to be linked in
####
ifeq ($(strip $(OBJ)),)
    $(error Unable to build: no object or source files specified in Makefile)
endif

####
# targets
####
.PHONY:	default all update-revision versionheader clean-versioninfo init cleandep clean tidy

all:
	@$(MAKE) versionheader
	$(MAKE) $(TARGET)

versionheader:
	@sed -e 's/@@datetime@@/$(shell LC_ALL=C date +"%a %d-%b-%Y %T %Z")/g'		\
		 -e 's/@@date@@/$(shell LC_ALL=C date +"%a %d-%b-%Y")/g'			\
		 -e 's/@@time@@/$(shell LC_ALL=C date +"%T %Z")/g'		\
		 -e 's/@@whoami@@/$(shell whoami)/g'				\
		 -e 's/@@hostname@@/$(shell hostname)/g'			\
		 -e 's|@@compiler@@|$(shell $(CC) $(CFLAGS) -v 2>&1 | tail -n 1 | sed -e "s;|;/;")|g'	\
		 -e 's/@@majorver@@/$(VER_MAJOR)/g'					\
		 -e 's/@@minorver@@/$(VER_MINOR)/g'					\
		 -e 's/@@extraver@@/$(subst \",,$(VER_EXTRA))/g'	\
		 -e 's/@@buildtype@@/$(BUILD_TYPE)/g'				\
		 -e 's/@@vcs@@/$(VER_VCS)/g'						\
		 -e 's/@@vcsrev@@/$(VER_VCSREV)/g'					\
		 -e 's/@@vcsstr@@/$(VER_VCSSTR)/g'					\
		 -e 's/@@fullverstr@@/$(VER_FULLSTR)/g'				\
		 -e 's#@@cflags@@#$(CFLAGS)#g'						\
		 < src/version.h.in > src/version.h

# initialise the build system for a new project
init:
	@mkdir -p src dep obj
	@echo "This file is a directory-keeper. Do not delete it." > dep/.keepme
	@echo "This file is a directory-keeper. Do not delete it." > obj/.keepme
	@echo 'syntax: glob' > .hgignore
	@echo 'obj/*.o' >> .hgignore
	@echo 'dep/*.d' >> .hgignore
	@echo '*~' >> .hgignore
	@echo '.*.sw?' >> .hgignore
	@echo '#define VER_COMPILE_DATETIME	"@@datetime@@"'			> src/version.h.in
	@echo '#define VER_COMPILE_DATE		"@@date@@"'				>> src/version.h.in
	@echo '#define VER_COMPILE_TIME		"@@time@@"'				>> src/version.h.in
	@echo '#define VER_COMPILE_BY			"@@whoami@@"'		>> src/version.h.in
	@echo '#define VER_COMPILE_HOST		"@@hostname@@"'			>> src/version.h.in
	@echo '#define VER_COMPILER			"@@compiler@@"'			>> src/version.h.in
	@echo '#define VER_BUILD_TYPE			"@@buildtype@@"'	>> src/version.h.in
	@echo '#define VER_CFLAGS				"@@cflags@@"'		>> src/version.h.in
	@echo ''													>> src/version.h.in
	@echo '#define VER_MAJOR				@@majorver@@'		>> src/version.h.in
	@echo '#define VER_MINOR				@@minorver@@'		>> src/version.h.in
	@echo '#define VER_EXTRA				"@@extraver@@"'		>> src/version.h.in
	@echo '#define VER_VCSREV				"@@vcsstr@@"'		>> src/version.h.in
	@echo ''													>> src/version.h.in
	@echo '#define VER_FULLSTR				"@@fullverstr@@"'	>> src/version.h.in
	@echo ''													>> src/version.h.in
	@echo Build system initialised

# remove the dependency files
cleandep:
	-rm -f $(DEPFILES)

# remove the dependency files and any target or intermediate build files
clean:	cleandep clean-versioninfo
	-rm -f $(OBJ) $(TARGET) $(GARBAGE)

# remove any dependency or intermediate build files, but not the final output
tidy:	cleandep clean-versioninfo
	-rm -f $(OBJ) $(GARBAGE)

#################################

$(TARGET):	$(OBJ) $(EXTDEP)
ifeq ($(SRC_TYPE),c)
	$(CC) $(CXXFLAGS) $(LDFLAGS) $(OBJ) $(LIBPTH) $(LIBLNK) -o $@
else
	$(CXX) $(CXXFLAGS) $(LDFLAGS) $(OBJ) $(LIBPTH) $(LIBLNK) -o $@
endif
ifeq ($(BUILD_TYPE),release)
	$(STRIP) $(TARGET)
endif

###
# extra rules
# example:
#src/parser.c:	src/parser.h


####
## musashi build rules
# 68k CPU builder
obj/m68kmake:	obj/m68kmake.o
	$(CC) $(CFLAGS) $(CPPFLAGS) obj/m68kmake.o -o $@
# 68k CPU sources
src/m68kops.h src/m68kops.c:	obj/m68kmake src/m68k_in.c
	./obj/m68kmake src src/m68k_in.c

####
# make object files from C source files
obj/%.o:	src/%.c
	$(CC) -c $(CFLAGS) $(CPPFLAGS) $< -o $@

##
# make object files from C++ source files
obj/%.o:	src/%.cc
	$(CXX) -c $(CXXFLAGS) $(CPPFLAGS) $< -o $@

obj/%.o:	src/%.cpp
	$(CXX) -c $(CXXFLAGS) $(CPPFLAGS) $< -o $@

###
# make C files from yacc/bison source
src/%.h src/%.c:	src/%.y
	$(YACC) $(YFLAGS) -d $<
	mv -f y.tab.c $*.c
	mv -f y.tab.h $*.h

###
# make C files from lex/flex source
src/%.c:	src/%.l
	$(LEX) $(LFLAGS) -o$@ $<

###
# make dependencies for our source files
dep/%.d:	src/%.c
	$(CC) -MM $(CFLAGS) $(CPPFLAGS) $< > $@.$$$$; \
		sed 's,\($*\)\.o[ :]*,obj/\1.o $@ : ,g' < $@.$$$$ > $@; \
		rm -f $@.$$$$

dep/%.d:	src/%.cpp
	$(CXX) -MM $(CXXFLAGS) $(CPPFLAGS) $< > $@.$$$$; \
		sed 's,\($*\)\.o[ :]*,obj/\1.o $@ : ,g' < $@.$$$$ > $@; \
		rm -f $@.$$$$

dep/%.d:	src/%.cc
	$(CXX) -MM $(CXXFLAGS) $(CPPFLAGS) $< > $@.$$$$; \
		sed 's,\($*\)\.o[ :]*,obj/\1.o $@ : ,g' < $@.$$$$ > $@; \
		rm -f $@.$$$$

####
# pull in the dependency files, but only for 'make $(TARGET)'
####

ifeq ($(MAKECMDGOALS),$(TARGET))
  -include $(DEPFILES)
endif

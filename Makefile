################################################################################
######################### User configurable parameters #########################
# filename extensions
CEXTS:=c
ASMEXTS:=s S
CXXEXTS:=cpp c++ cc

# probably shouldn't modify these, but you may need them below
ROOT=.
FWDIR:=$(ROOT)/firmware
BINDIR=$(ROOT)/bin
SRCDIR=$(ROOT)/src
INCDIR=$(ROOT)/include

WARNFLAGS+=
EXTRA_CFLAGS=
EXTRA_CXXFLAGS=

# Set to 1 to enable hot/cold linking
USE_PACKAGE:=1

# Add libraries you do not wish to include in the cold image here
# EXCLUDE_COLD_LIBRARIES:= $(FWDIR)/your_library.a
EXCLUDE_COLD_LIBRARIES:= 

# Embed PATH.JERRYIO and other static files without depending on LemLib's
# asset header. The generated symbols are consumed by aon/jerryio/asset.hpp.
STATIC_DIR:=$(ROOT)/static
STATIC_FILES:=$(wildcard $(STATIC_DIR)/*)
STATIC_OBJECTS:=$(patsubst $(STATIC_DIR)/%,$(BINDIR)/static/%.o,$(STATIC_FILES))
STATIC_LIBRARY:=$(BINDIR)/aon-static-assets.a
LIBRARIES+=$(STATIC_LIBRARY)

# Set this to 1 to add additional rules to compile your project as a PROS library template
IS_LIBRARY:=0
# TODO: CHANGE THIS! 
# Be sure that your header files are in the include directory inside of a folder with the
# same name as what you set LIBNAME to below.
LIBNAME:=libbest
VERSION:=1.0.0
# EXCLUDE_SRC_FROM_LIB= $(SRCDIR)/unpublishedfile.c
# this line excludes opcontrol.c and similar files
EXCLUDE_SRC_FROM_LIB+=$(foreach file, $(SRCDIR)/main,$(foreach cext,$(CEXTS),$(file).$(cext)) $(foreach cxxext,$(CXXEXTS),$(file).$(cxxext)))

# files that get distributed to every user (beyond your source archive) - add
# whatever files you want here. This line is configured to add all header files
# that are in the directory include/LIBNAME
TEMPLATE_FILES=$(INCDIR)/$(LIBNAME)/*.h $(INCDIR)/$(LIBNAME)/*.hpp

.DEFAULT_GOAL=quick

################################################################################
################################################################################
########## Nothing below this line should be edited by typical users ###########
-include ./common.mk

$(BINDIR)/static/%.o: $(STATIC_DIR)/%
	$(VV)mkdir -p $(dir $@)
	$(call test_output_2,Embedded $< ,cd "$(ROOT)" && $(ARCHTUPLE)ld -r -b binary "static/$*" -o "$@",$(OK_STRING))

$(STATIC_LIBRARY): $(STATIC_OBJECTS)
	$(VV)mkdir -p $(dir $@)
	-$(VV)rm -f $@
	$(call test_output_2,Archived static assets ,$(AR) rcs $@ $^,$(OK_STRING))

$(HOT_ELF) $(MONOLITH_ELF): $(STATIC_LIBRARY)

WX_RELEASE := $(shell wx-config --release)

all: installed

VERSION 	= wxpropgrid-1.4.15
FILENAME	= $(VERSION)-src.tar.gz
TARBALL		= build/$(VERSION)-src.tar.gz
TARBALL_URL	= http://prdownloads.sourceforge.net/wxpropgrid/$(FILENAME)?download
UNPACK_CMD	= tar xzf
SOURCE_DIR	= build/propgrid
#MD5SUM_FILE 	= $(FILENAME).md5sum
WXPROPGRID_ROOT = $(shell rospack find wxpropgrid)

include $(shell rospack find mk)/download_unpack_build.mk

UNAME := $(shell uname -s)
ifeq ($(UNAME),'Darwin')
ARCHOPTS = -arch i386 -arch x86_64
else
ARCHOPTS =
endif

ifneq ($(WX_RELEASE),2.9) # Do not build if wx-2.9, wxpropgrid is builtin in wx-2.9
installed: $(SOURCE_DIR)/unpacked
	cd $(SOURCE_DIR) && \
		export CFLAGS='-g -D__NOTWXPYTHON__ -g' $ARCHOPTS && \
		export CXXFLAGS='-g -D__NOTWXPYTHON__ -g' $ARCHOPTS && \
		./configure --prefix=$(WXPROPGRID_ROOT)/propgrid_install --disable-dependency-tracking && \
		make && \
		make install
	touch installed
else
installed:
	touch installed
endif

clean:
	-cd $(SOURCE_DIR) && \
		make clean && \
		rm -f Makefile
	-rm -rf propgrid_install
	-rm -f installed

wipe: clean
	-rm -rf build
	-rm -rf $(TARBALL)


.PHONY: clean

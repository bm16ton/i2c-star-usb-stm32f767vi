##
## This file is part of the i2c-star project.
##
## Copyright (C) 2009 Uwe Hermann <uwe@hermann-uwe.de>
##
## This library is free software: you can redistribute it and/or modify
## it under the terms of the GNU Lesser General Public License as published by
## the Free Software Foundation, either version 3 of the License, or
## (at your option) any later version.
##
## This library is distributed in the hope that it will be useful,
## but WITHOUT ANY WARRANTY; without even the implied warranty of
## MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
## GNU Lesser General Public License for more details.
##
## You should have received a copy of the GNU Lesser General Public License
## along with this library.  If not, see <http://www.gnu.org/licenses/>.
##

PREFIX		?= arm-none-eabi
#PREFIX		?= arm-elf

TARGETS := stm32/f7
#TARGETS		:= stm32/f0 stm32/f1 stm32/f2 stm32/f3 stm32/f4 stm32/l1
#TARGETS		+= lpc13xx lpc17xx #lpc43xx
#TARGETS		+= lm3s lm4f
#TARGETS		+= efm32/efm32tg efm32/efm32g efm32/efm32lg efm32/efm32gg

# Be silent per default, but 'make V=1' will show all compiler calls.
ifneq ($(V),1)
Q := @
# Do not print "Entering directory ...".
MAKEFLAGS += --no-print-directory
endif

OPENCM3_DIR := $(realpath libopencm3)
EXAMPLE_RULES = elf

all: build

bin: EXAMPLE_RULES += bin
hex: EXAMPLE_RULES += hex
srec: EXAMPLE_RULES += srec
list: EXAMPLE_RULES += list
images: EXAMPLE_RULES += images

bin: build
hex: build
srec: build
list: build
images: build

build: libopencm3 examples

libopencm3:
	$(Q)if [ ! "`ls -A libopencm3`" ] ; then \
		printf "######## ERROR ########\n"; \
		printf "\tlibopencm3 is not initialized.\n"; \
		printf "\tPlease run:\n"; \
		printf "\t$$ git submodule init\n"; \
		printf "\t$$ git submodule update\n"; \
		printf "\tbefore running make.\n"; \
		printf "######## ERROR ########\n"; \
		exit 1; \
		fi
	$(Q)$(MAKE) -C libopencm3 TARGETS="$(TARGETS)"

#EXAMPLE_DIRS:=$(sort $(dir $(wildcard $(addsuffix /*/*/Makefile,$(addprefix examples/,$(TARGETS))))))
EXAMPLE_DIRS := \
    src/i2c-stm32f7-usb

$(EXAMPLE_DIRS): libopencm3
	@printf "  BUILD   $@\n";
	$(Q)$(MAKE) --directory=$@ OPENCM3_DIR=$(OPENCM3_DIR) $(EXAMPLE_RULES)

examples: $(EXAMPLE_DIRS)
	$(Q)true

clean: $(EXAMPLE_DIRS:=.clean)
	$(Q)$(MAKE) -C libopencm3 clean

stylecheck: $(EXAMPLE_DIRS:=.stylecheck)
styleclean: $(EXAMPLE_DIRS:=.styleclean)


%.clean:
	$(Q)if [ -d $* ]; then \
		printf "  CLEAN   $*\n"; \
		$(MAKE) -C $* clean OPENCM3_DIR=$(OPENCM3_DIR) || exit $?; \
	fi;

%.styleclean:
	$(Q)$(MAKE) -C $* styleclean OPENCM3_DIR=$(OPENCM3_DIR)

%.stylecheck:
	$(Q)$(MAKE) -C $* stylecheck OPENCM3_DIR=$(OPENCM3_DIR)


.PHONY: build libopencm3 examples $(EXAMPLE_DIRS) install clean \
        stylecheck styleclean bin hex srec list images


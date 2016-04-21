# I am a comment, and I want to say that the variable CC will be
# the compiler to use.
CC=gcc

# Hey!, I am comment number 2. I want to say that CFLAGS will be the
# options I'll pass to the compiler.
#CFLAGS= -c -Wall

.PHONY : all target

#xenopath := /usr/xenomai/bin
#xenopath_x := /usr/xenomai-tn/bin

#prefix := $(shell $(xenopath)/xeno-config --prefix)
#prefix_x := $(shell $(xenopath_x)/xeno-config --prefix)

#ifeq ($(prefix),)
#$(error Please add <xeno-install-path>/bin to your PATH variable)
#endif
#$(OUTDIR)/ccmcore: $(OUTDIR)/main.o $(OUTDIR)/core.o $(OUTDIR)/watchdog.o $(OUTDIR)/tracelog.o $(OUTDIR)/state.o $(OUTDIR)/comp.o $(OUTDIR)/module.o $(OUTDIR)/sched.o $(OUTDIR)/hw.o $(OUTDIR)/rtc.o $(OUTDIR)/event.o $(OUTDIR)/api.o $(OUTDIR)/kwpsrv_e.o $(OUTDIR)/kwpsrv_i.o $(OUTDIR)/processsrv.o $(OUTDIR)/eventsrv.o $(OUTDIR)/event_hand.o $(OUTDIR)/serial_srv.o $(OUTDIR)/timersrv.o $(OUTDIR)/NMsrv.o $(OUTDIR)/kwp2000_services_CCMcore.o

OBJECTS :=  main fuelcalc

LIBRARYROOTPATH := 
LIBRARYLIST := 

LIBRARYLIST_LINUX_GCC := $(addsuffix /lib/LINUX-GCC/stable,$(addprefix $(LIBRARYROOTPATH),$(LIBRARYLIST)))

#LIBRARYLIST_LINUX_GCC_HEADERINCLUDES := $(addprefix -I,$(addsuffix /header,$(LIBRARYLIST_LINUX_GCC)))
#LIBRARYLIST_LINUX_GCC_LIBRARYINCLUDES := $(addprefix -L,$(LIBRARYLIST_LINUX_GCC))


CFLAGS := $(shell xeno-config --skin=native --cflags)
LDFLAGS :=  $(shell xeno-config --skin=native --ldflags)

#CFLAGS := $(shell /usr/xenomai/bin/xeno-config --skin=native --cflags)
#LDFLAGS :=  $(shell /usr/xenomai/bin/xeno-config --skin=native --ldflags)

CFLAGS_CUSTOM :=  -I../header -g
LDFLAGS_CUSTOM := -ldl -rdynamic 

OUTDIR := ../comp

#Librarypath
#LIBRARY_PATH =  -L$(SPIDER3LIB)/$(TIMER_LINUX) -L$(SPIDER3LIB)/$(SPIDERCOMMONLIB_LINUX) -L$(SPIDER3LIB)/$(PROCESSDATAMANAGER_LINUX) -L$(SPIDER3LIB)/$(NETWORKMANAGER_LINUX) -L$(SPIDER3LIB)/$(KWP2000_LINUX) -L$(SPIDER3LIB)/$(CAN_TP_LINUX) -L$(SPIDER3LIB)/$(EVENTHANDLER_LINUX)
LIBRARY_PATH = $(addprefix -L,$(LIBRARYLIST_LINUX_GCC))
LIBRARY =

#Headerlink
#HEADER_LINK = -I$(SPIDER3LIB)/$(EVENTHANDLER_LINUX_HEADER) -I$(SPIDER3LIB)/$(CAN_TP_LINUX_HEADER) -I$(SPIDER3LIB)/$(KWP2000_LINUX_HEADER) -I$(SPIDER3LIB)/$(NETWORKMANAGER_LINUX_HEADER) -I$(SPIDER3LIB)/$(PROCESSDATAMANAGER_LINUX_HEADER) -I$(SPIDER3LIB)/$(SPIDERCOMMONLIB_LINUX_HEADER) -I$(SPIDER3LIB)/$(TIMER_LINUX_HEADER)
HEADER_LINK = $(addprefix -I,$(addsuffix /header,$(LIBRARYLIST_LINUX_GCC)))

#flags for x86
CFLAGS +=  $(CFLAGS_CUSTOM)  $(HEADER_LINK)
LDFLAGS += $(LDFLAGS_CUSTOM) -lrtdm -Wl,-rpath -Wl,/usr/xenomai/lib -Wl,-rpath -Wl,'$$ORIGIN/../../../CCMFunctions/lib/LINUX-GCC' $(LIBRARY_PATH) $(LIBRARY)

all: $(OUTDIR)/EMS

$(OUTDIR)/EMS: $(addsuffix .o,$(addprefix $(OUTDIR)/,$(OBJECTS))) 
	$(CC) -o $@ $^ $(LDFLAGS) 


$(OUTDIR)/%.o: %.c
	@echo "Building with generic rule"
	$(CC) -o $@ -c $< $(CFLAGS) 
		
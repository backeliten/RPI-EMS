
WINDEFINE = WINDOWS

# Autodetect environment
SHELL   = sh
ifneq ($(or $(COMSPEC), $(ComSpec)),)
$(info COMSPEC detected $(COMSPEC) $(ComSpec))
ifeq ($(findstring cygdrive,$(shell set)),)
SHELL:=$(or $(COMSPEC),$(ComSpec))
SHELL_IS_WIN32=1
else
$(info cygwin detected)
#override user-setting since cygwin has rm
REMOVE_CMD:=rm
CP_CMD:=cp
WINDEFINE:= UNIX
endif
else
#most probaly a Unix/Linux/BSD system which should have rm
REMOVE_CMD:=rm
CP_CMD:=cp
WINDEFINE:= UNIX
endif

CSTANDARD = 
#CXSTANDARD = -std=gnu99
CXSTANDARD = 

COMPILED := comp


all: user kernel
	
process:
	make -C ./user
	
kernel:
	make -C ./src all


COPYFOLDER := RPi-EMS2/KEMS
include Makefile.copy

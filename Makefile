CONTIKI_PROJECT=CentralUnit Door Gate

all: $(CONTIKI_PROJECT)
CONTIKI=/home/user/contiki
PROJECT_SOURCEFILES+=nesproj.c
CONTIKI_WITH_RIME=1
include $(CONTIKI)/Makefile.include

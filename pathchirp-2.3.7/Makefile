# Generated automatically from Makefile.in by configure.
CC = gcc
CFLAGS = -g -O4 -Wall -I.

LIBS = -lm 
CPU = i686

SRC=	Src
BIN=	Bin/$(CPU)
SOBJS=   $(SRC)/pathchirp_snd.c $(SRC)/realtime.c $(SRC)/hash.c
ROBJS=   $(SRC)/realtime.c $(SRC)/pathchirp_rcv.c $(SRC)/alloc_rcv.c $(SRC)/chirps_rcv.c $(SRC)/compute_bw_contextsw_rcv.c $(SRC)/loss_reorder_rcv.c $(SRC)/open_files_rcv.c $(SRC)/parse_cmd_line_rcv.c $(SRC)/signal_alrm_rcv.c  $(SRC)/control_rcv.c $(SRC)/hash.c
OBJS=    $(SOBJS) $(ROBJS)

TARGETS=$(BIN)/pathchirp_snd $(BIN)/pathchirp_rcv

all: $(TARGETS)

$(BIN)/pathchirp_snd: $(SRC)/pathchirp_snd.c $(SRC)/pathchirp.h \
			$(SRC)/delay.h config.h $(SRC)/pathchirp_snd.h \
			$(SRC)/realtime.c $(SRC)/realtime.h $(SRC)/hash.c
	./mkinstalldirs $(BIN)
	$(CC) $(CFLAGS) $(SOBJS) -o $(BIN)/pathchirp_snd $(LIBS) 

$(BIN)/pathchirp_rcv: $(SRC)/pathchirp_rcv.c $(SRC)/pathchirp.h \
		      $(SRC)/realtime.c $(SRC)/realtime.h config.h \
		      $(SRC)/alloc_rcv.c $(SRC)/chirps_rcv.c \
                  $(SRC)/compute_bw_contextsw_rcv.c $(SRC)/loss_reorder_rcv.c \
                  $(SRC)/open_files_rcv.c $(SRC)/parse_cmd_line_rcv.c \
		  $(SRC)/signal_alrm_rcv.c  $(SRC)/control_rcv.c \
		  $(SRC)/pathchirp_rcv.h $(SRC)/hash.c \
	./mkinstalldirs $(BIN)
	$(CC) $(CFLAGS) $(ROBJS) -o $(BIN)/pathchirp_rcv $(LIBS) 

clean: 
	rm -f ${TARGETS}
	rm -f config.cache config.log config.status

distclean: clean
	rm -f Makefile configure config.h

configure: configure.in
	autoconf

Makefile: Makefile.in configure
	./configure

CFLAGS = -g -O2 

default: due-nmr-lib.so ihmt

ihmt: due-nmr-lib.h ihmt.c
	$(CC) $(CFLAGS) -o ihmt ihmt.c -L. -ldue-nmr-lib -lm

due-nmr-lib.so: due-nmr-lib.c due-nmr-lib.h
	$(CC) $(CFLAGS) -shared -fPIC -o libdue-nmr-lib.so due-nmr-lib.c

.PHONY : clean
clean :
	-rm -R due-nmr-lib.so ihmt libdue-nmr-lib.so libdue-nmr-lib.so.dSYM ihmt.dSYM
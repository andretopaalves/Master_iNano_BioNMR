CFLAGS = -g -O2 

default: due-nmr-lib.so cpmg

cpmg: due-nmr-lib.h mycpmg.c
	$(CC) $(CFLAGS) -o cpmg mycpmg.c -L. -ldue-nmr-lib -lm

due-nmr-lib.so: due-nmr-lib.c due-nmr-lib.h
	$(CC) $(CFLAGS) -shared -fPIC -o libdue-nmr-lib.so due-nmr-lib.c

.PHONY : clean
clean :
	-rm -R due-nmr-lib.so cpmg libdue-nmr-lib.so libdue-nmr-lib.so.dSYM cpmg.dSYM
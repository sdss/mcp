SHELL = /bin/sh

DIRS = ab ip mei-new util mcp-new etc

all :
	@for d in $(DIRS); do \
		echo $$d; \
		(cd $$d; $(MAKE) $(MFLAGS) all ); \
	done

install :
	@for d in $(DIRS); do \
		echo $$d; \
		(cd $$d; echo $(MAKE) $(MFLAGS) install); \
	done

tags :
	@rm -f TAGS
	@for d in $(DIRS); do \
		if [ $$d != "etc" ]; then \
			echo $$d; \
			etags --append -a -t -o TAGS $$d/*.[ch]; \
		fi; \
	done

clean :
	- rm *.o core *~
	@-for d in $(DIRS); do \
		echo $$d; \
		(cd $$d; $(MAKE) $(MFLAGS) clean ); \
	done

SHELL = /bin/sh

DIRS = ab bin etc ip mei-new mcp-new ups util

all :
	@for d in $(DIRS); do \
		echo $$d; \
		(cd $$d; $(MAKE) $(MFLAGS) all ); \
	done

install :
	@echo ""
	@echo "Make sure the current MCP directories under"
	@echo ""
	@echo "     `pwd`"
	@echo ""
	@echo "have the latest versions of the files.  These will be copied to"
	@echo "$(MCP_DIR) during the MCP's installation."
	@echo ""
	@if [ "$(MCP_DIR)" = "" ]; then \
		echo "The destination directory has not been specified." >&2; \
		echo "Set the environment variable MCP_DIR" >&2; \
		echo ""; \
		exit 1; \
	fi
	@if [ ! -d $(MCP_DIR) ]; then \
		echo $(MCP_DIR) "doesn't exist; making it"; \
		mkdir $(MCP_DIR); \
	fi
	@if [ `(cd $(MCP_DIR); pwd)` = `pwd` ]; then \
		echo "The destination directory is the same" \
			"as the current directory; aborting." >&2; \
		echo ""; \
		exit 1; \
	fi
	@echo "I'll give you 5 seconds to think about it (control-C to abort) ..."
	@for pos in          5 4 3 2 1; do \
	   echo "                              " | sed -e 's/ /'$$pos'/'$$pos; \
	   sleep 1; \
	done
	@echo "... and we're off... deleting"
	-@/bin/rm -rf $(MCP_DIR)
	@mkdir $(MCP_DIR)
	@cp Makefile branches $(MCP_DIR)

	@for d in $(DIRS); do \
		echo $$d; \
		(mkdir $(MCP_DIR)/$$d; cd $$d; \
			echo In $$d; $(MAKE) $(MFLAGS) install ); \
	done
	@chmod -R g+w $(MCP_DIR)

tags :
	@rm -f TAGS
	@for d in $(DIRS); do \
		if [ $$d != "bin" -a $$d != "etc" -a $$d != "ups" ]; then \
			echo $$d; \
			etags --append -a -t -o TAGS $$d/*.[ch]; \
		fi; \
	done

make :
	@for d in $(DIRS); do \
		if [ $$d = "mcp-new" ]; then \
			(cd $$d; $(MAKE) $(MFLAGS) make ); \
		fi; \
	done

clean :
	- rm -f *.o core *~
	@-for d in $(DIRS); do \
		echo $$d; \
		(cd $$d; $(MAKE) $(MFLAGS) clean ); \
	done

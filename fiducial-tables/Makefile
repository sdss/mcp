SHELL = /bin/sh

all :;

install : all
	@if [ X$(MCP_FIDUCIALS_DIR) != X"" ]; then \
	    if [ ! -d $(MCP_FIDUCIALS_DIR) ]; then \
		mkdir $(MCP_FIDUCIALS_DIR); \
	    fi; \
	    cp Makefile alt.dat az.dat rot.dat $(MCP_FIDUCIALS_DIR); \
	else \
	    cp Makefile alt.dat az.dat rot.dat $(MCP_DIR)/fiducial-tables; \
	fi

clean :
	- rm -f *.o core *~

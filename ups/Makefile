SHELL = /bin/sh

all :;

install : all
	cp Makefile $(MCP_DIR)/ups
	$(SDSSTOOLS_DIR)/bin/install_ups_table -w mcp.table $(MCP_DIR)/ups

clean :
	- rm -f *.o core *~

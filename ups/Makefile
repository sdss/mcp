SHELL = /bin/sh

all :;

install : all
	cp Makefile $(MCP_DIR)/ups
	$(SDSSTOOLS_DIR)/bin/install_ups_table -w mcp.table $(MCP_DIR)/ups
	# @: Don't freeze mcp_fiducials version
	@perl -i -p -e 's/mcp_fiducials[^"]*/mcp_fiducials/' \
						$(MCP_DIR)/ups/mcp.table

clean :
	- rm -f *.o core *~

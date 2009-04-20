#if !defined(MCPSPECTRO_H)
#define MCPSPECTRO_H

char *sp1_cmd(int uid, unsigned long cid, char *cmd);
char *sp2_cmd(int uid, unsigned long cid, char *cmd);

char *slitdoor_open_cmd(int uid, unsigned long cid, char *cmd);
char *slitdoor_close_cmd(int uid, unsigned long cid, char *cmd);
char *slithead_latch_open_cmd(int uid, unsigned long cid, char *cmd);
char *slithead_latch_close_cmd(int uid, unsigned long cid, char *cmd);

#endif

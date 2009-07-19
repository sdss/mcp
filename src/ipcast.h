int ipsdss_ini();
int ipsdss_send(char *sdss_msg, int sdss_size);

void init_broadcast_ipsdss();
void broadcast_ipsdss(int uid, unsigned long cid);

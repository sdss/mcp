#if !defined(SERIAL_H)
#define SERIAL_H

extern long tcc_taskId;
extern int tcc_may_release_semCmdPort;

int barcode_serial(int port);

#endif

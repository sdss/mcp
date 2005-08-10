#if !defined(SERIAL_H)
#define SERIAL_H

#define USE_BARCODE 0

extern long tcc_taskId;
extern int tcc_may_release_semCmdPort;


#if USE_BARCODE
   int barcode_serial(int port);
#endif

#endif

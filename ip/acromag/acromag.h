#if !defined(ACROMAG_H)
#define ACROMAG_H 1

int attach_ihandler(int trap, int vector, int zero, int (*handler)(void),
		    struct handler_data* hdata);
UWORD inpw(UWORD *addr);
UWORD outpw(UWORD *addr, int b);

#endif

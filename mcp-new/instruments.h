#if !defined(INSTRUMENT_LIFT_H)
#define INSTRUMENT_LIFT_H 1

int umbil(double el, double rot);
int umbilGet(int el, int rot);
/*
 * globals
 */
extern int check_stop_in(void);
extern int fsm(int inst,int action);
extern int il_ADC128F1;
void il_data_collection(void);
extern long *axis2pos;
extern long *axis4pos;

/*
 * 
 */
extern struct conf_blk sbrd;

#endif

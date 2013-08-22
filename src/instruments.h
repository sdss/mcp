#if !defined(INSTRUMENTS_H)
#define INSTRUMENTS_H 1

int get_inststatus(char *status_ans, int size);
/*
 * globals
 */
extern int check_stop_in(int update);
extern int fsm(int inst,int action);
extern int il_ADC128F1;
void il_data_collection(void);
void update_sdssdc_status_i6(void);
/*
 * 
 */
extern struct conf_blk sbrd;

#endif

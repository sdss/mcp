int tm_move_instchange (void);
void tm_bf(int axis, int vel, int accel, int pos1,int pos2, int times);
void tm_start_move(int mei_axis, int pos, int vel, int accel);
void tm_print_coeffs(int mei_axis);
void tm_set_filter_coeff(int mei_axis, int index, int val);
void tm_reset_integrator(int mei_axis);
void tm_get_position(int mei_axis, double *position);
void tm_get_velocity(int mei_axis, double *velocity);
void tm_set_sample_rate(unsigned short rate);
void tm_set_position(int mei_axis, int pos);
int tm_adjust_position(int axis, int offset);
void sem_controller_run(int mei_axis);
void tm_sem_controller_run(int mei_axis);
void sem_controller_idle(int mei_axis);
void tm_sem_controller_idle(int mei_axis);
int ADC128F1_initialize(unsigned char *addr, int occur);
void tm_data_collection(void);
void tm_jog_axis(void);
void tm_slit_clear(int door);
void tm_slit_open(int door);
void tm_slit_close(int door);
void tm_sp_slit_open(int door);
void tm_sp_slit_close(int door);
int tm_slit_status(void);
void tm_cart_latch(int door);
void tm_cart_unlatch(int door);
void tm_sp_cart_latch(int door);
void tm_sp_cart_unlatch(int door);
int ffs_enable(int val);		/* enable FF screen */
int ffs_open_status(void);
int ffs_close_status(void);

void tm_ffl_on(void);
void tm_ffl_off(void);
void tm_sp_ffl_on(void);
void tm_sp_ffl_off(void);
void tm_neon_on(void);
void tm_neon_off(void);
void tm_sp_neon_on(void);
void tm_sp_neon_off(void);
void tm_hgcd_on(void);
void tm_hgcd_off(void);
void tm_sp_hgcd_on(void);
void tm_sp_hgcd_off(void);
int az_amp_ok(void);
int alt_amp_ok(void);
int rot_amp_ok(void);
void tm_amp_mgt(void);
void tm_print_amp_status(void);
void tm_setup_wd (void);
void tm_amp_disengage(void);
void tm_amp_engage(void);
void tm_print_axis_status(int mei_axis);
int tm_axis_status(int mei_axis);
void tm_print_axis_state(int mei_axis);
int tm_axis_state(int mei_axis);
const char *axis_source_str(int mei_axis);
const char *axis_state_str(int mei_axis);
void tm_print_axis_source(int mei_axis);
void clear_sticky_bumps(int axis);
/*
 * Global variables
 */
extern int monitor_on[3];

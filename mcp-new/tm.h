void tm_move_time (int axis, int vel, int accel, int time);
void tm_move_offset (int axis, int off);
void tm_bf (int axis, int vel, int accel, int pos1,int pos2, int times);
void tm_start_move (int axis, int vel, int accel, int pos);
void tm_move_pos (int axis, int vel, int accel, int pos);
void tm_print_coeffs(int axis);
void tm_set_coeffs(int axis, int index, int val);
void tm_reset_integrator (int axis);
void tm_clear_pos (int axis);
void tm_get_pos (int axis,double *position);
void tm_get_vel (int axis,double *velocity);
void tm_set_sample_rate (unsigned short rate);
void tm_set_pos (int axis, int pos);
void tm_set_analog_encoder(int axis, int channel);
void tm_set_encoder(int axis);
void sem_controller_run (int axis);
void tm_controller_run (int axis);
void sem_controller_idle (int axis);
void tm_controller_idle (int axis);
void tm_dual_loop (int axis, int dual);
void tm_set_boot_filter (int axis);
void tm_display_axis(int axis);
void tm_nodisplay_axis(int axis);
void tm_display_continuous();
void tm_display_once();
void tm_display (int delay);
void tmDisplay (int delay);
void TM_Verbose();
void TM_help();
void TM_quiet();
int ADC128F1_initialize(unsigned char *addr, int occur);
void tm_data_collection();
void tm_read_all_adc(int cnt);
void tm_jog_axis();
int tm_az_brake(short val);
void tm_az_brake_on();
void tm_az_brake_off();
void tm_sp_az_brake_on();
void tm_sp_az_brake_off();
int tm_alt_brake(short val);
void tm_alt_brake_on();
void tm_alt_brake_off();
void tm_sp_alt_brake_on();
void tm_sp_alt_brake_off();
int tm_brake_status();
int tm_clamp(short val);
void tm_clamp_on();
void tm_clamp_off();
void tm_sp_clamp_on();
void tm_sp_clamp_off();
int tm_clamp_status();
void tm_slit_clear(int door);
void tm_slit_open(int door);
void tm_slit_close(int door);
void tm_sp_slit_open(int door);
void tm_sp_slit_close(int door);
int tm_slit_status();
void tm_cart_latch(int door);
void tm_cart_unlatch(int door);
void tm_sp_cart_latch(int door);
void tm_sp_cart_unlatch(int door);
void tm_ffs_open();
void tm_ffs_close();
void tm_sp_ffs_open();
void tm_sp_ffs_close();
void tm_ffl_on();
void tm_ffl_off();
void tm_sp_ffl_on();
void tm_sp_ffl_off();
void tm_neon_on();
void tm_neon_off();
void tm_sp_neon_on();
void tm_sp_neon_off();
void tm_hgcd_on();
void tm_hgcd_off();
void tm_sp_hgcd_on();
void tm_sp_hgcd_off();
int tm_ff_status();
int az_amp_ok();
int alt_amp_ok();
int rot_amp_ok();
void tm_amp_mgt();
void tm_print_amp_status();
void tm_setup_wd ();
void tm_amp_disengage();
void tm_amp_engage();
void tm_set_fiducial(int axis);
void tm_print_fiducial_all();
void tm_print_fiducial(int axis);
void tm_set_fiducials(int axis);
void tm_print_axis_status(int axis);
int tm_axis_status(int axis);
void tm_print_axis_state(int axis);
int tm_axis_state(int axis);
void tm_print_axis_source(int axis);

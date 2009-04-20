/*
	llio.c - low-level input/output drivers.
*/

/* Copyright(c) 1991-1996 by Motion Engineering, Inc.  All rights reserved.
 *
 * This software  contains proprietary and  confidential information  of
 * Motion Engineering Inc., and its suppliers.  Except as may be set forth
 * in the license agreement under which  this software is supplied, use,
 * disclosure, or  reproduction is prohibited without the prior express 
 * written consent of Motion Engineering, Inc.
 */

#	include "idsp.h"

#  ifdef __TURBOC__
#	include <dos.h>
#	endif


/*	------------------------
	I/O Ports
	------------------------
*/

static int16 pcdsp_port_map[] = {
    IO_P1 + PORT_A,
    IO_P1 + PORT_B,
    IO_P1 + PORT_C,
    IO_P3 + PORT_A,
    IO_P3 + PORT_B,
    IO_P3 + PORT_C,
    IO_P2 + PORT_A,
    IO_P2 + PORT_B,
    IO_P2 + PORT_C,
    IO_CFG_48 + PORT_A,
    IO_CFG_48 + PORT_B,
    IO_CFG_48 + PORT_C,
    IO_CFG_50 + PORT_A,
    IO_CFG_50 + PORT_B,
    IO_CFG_50 + PORT_C,
    };

static int16 pcdsp_port_config_map[] = {
    IO_P1 + PORT_CTRL,
    IO_P1 + PORT_CTRL,
    IO_P1 + PORT_CTRL,
    IO_P3 + PORT_CTRL,
    IO_P3 + PORT_CTRL,
    IO_P3 + PORT_CTRL,
    IO_P2 + PORT_CTRL,
    IO_P2 + PORT_CTRL,
    IO_P2 + PORT_CTRL,
    IO_CFG_48 + PORT_CTRL,
    IO_CFG_48 + PORT_CTRL,
    IO_CFG_48 + PORT_CTRL,
    IO_CFG_50 + PORT_CTRL,
    IO_CFG_50 + PORT_CTRL,
    IO_CFG_50 + PORT_CTRL,
    };

static int16 pcdsp_port_config_data[] =
		{ 0x10, 0x02, 0x09 };


int16 FNTYPE dsp_port_address(int16 port, P_INT address)
{	if ((port < 0) || (port >= (sizeof(pcdsp_port_map) / sizeof(int16))))
		return (dsp_error = DSP_ILLEGAL_IO);
	*address = pcdsp_port_map[port] ;
	return (dsp_error = DSP_OK) ;
}

/*	-----------------
	 User I/O stuff.
	-----------------
*/

int16 FNTYPE pcdsp_get_io(PDSP dsp, int16 port)
{
	int16 address ;
	if (! dsp_port_address(port, &address))
		return idsp_read_dm(dsp, address) ;
	return -1 ;
}


int16 FNTYPE pcdsp_set_io(PDSP dsp, int16 port, int16 value)
{
	DSP_DM curr = value;
	int16 addr ;

	if (! dsp_port_address(port, &addr))
		idsp_write_dm(dsp, addr, curr) ;
	return dsp_error ;
}


int16 FNTYPE pcdsp_init_io(PDSP dsp, int16 port, int16 config)
{
	DSP_DM config_data;
	int16 addr ;

	if (! dsp_port_address(port, &addr))
	{    config_data = idsp_read_dm(dsp, pcdsp_port_config_map[port]);
		
		if(config) /* output? */
			config_data = (config_data & ~pcdsp_port_config_data[port % 3]) | 0x80;
		else
			config_data = (config_data | pcdsp_port_config_data[port % 3]) | 0x80;

		idsp_write_dm(dsp, pcdsp_port_config_map[port], config_data);
	}
	return dsp_error ;
}



/*	------------------------
	Analog Input Ports
	------------------------
*/
int16 FNTYPE pcdsp_config_analog(PDSP dsp, int16 ch, int16 diff, int16 bipolar)
{
	if ((ch < 0) || (ch >= PCDSP_ANALOG_CHANNELS))
		return (dsp_error = DSP_ILLEGAL_ANALOG);

	dsp->analog_control[ch] =
			ch
		|	(diff?		ANALOG_DIFF : 0)
		|	(bipolar?	ANALOG_BIPOLAR : 0) ;

	return dsp_error ;
}


int16 FNTYPE pcdsp_start_analog(PDSP dsp, int16 channel)
{
	DSP_DM
		control ;

	if ((channel < 0) || (channel >= PCDSP_ANALOG_CHANNELS))
	{	dsp_error = DSP_ILLEGAL_ANALOG;
		return -1;
	}

	control = dsp->analog_control[channel] ;
	idsp_write_dm(dsp, SEL_A2D, control);
	return dsp_error ;
}

int16 FNTYPE pcdsp_read_analog(PDSP dsp)
{	return (idsp_read_dm(dsp, SEL_A2D) & ANALOG_MAX);
}



/*	-----------------------------
	 axis digital sensor inputs.
	-----------------------------
*/
static int16 pcdsp_switch_map[] = {
    DSP_SWITCHES,
    DSP_SWITCHES,
    DSP_SWITCHES+1,
    DSP_SWITCHES+1,
    DSP_SWITCHES+8,
    DSP_SWITCHES+8,
    DSP_SWITCHES+9,
    DSP_SWITCHES+9,
    };

int16	FNTYPE pcdsp_switches(PDSP dsp, int16 axis)
{
	int16
		shift = (axis & 0x01)? 4 : 0;

    return (idsp_read_dm(dsp, pcdsp_switch_map[axis]) >> shift) & 0x000F;
}


static void wait_for_sample(void)
{
	DSP_DM zero = 0 ;

	for (pcdsp_write_dm(dspPtr, DM_SIGNATURE, 1, &zero);
		 !pcdsp_read_dm(dspPtr, DM_SIGNATURE, 1, NULL);
		 )
		;
}


/*	Amp-enable output. */
int16 FNTYPE pcdsp_enable(PDSP dsp, int16 axis, int16 enable)
{
	P_DSP_DM	addr = (axis < 4)? (IO_P2 + 2) : (IO_P3 + 2);
	unsigned	mask = 1 << ((axis % 4) << 1);

	DSP_DM
		curr;

	curr = idsp_read_dm(dsp, addr);

	if (enable)
		curr |= mask ;
	else
		curr &= ~mask ;

	pcdsp_transfer_block(dsp, FALSE, FALSE, addr, 1, &curr);
	
	if (!dsp_error)
    {
		wait_for_sample() ;
		wait_for_sample() ;
    }

	return dsp_error ;
}





int16 FNTYPE pcdsp_enabled(PDSP dsp, int16 axis, P_INT enabled)
{
	P_DSP_DM	addr = (axis < 4)? (IO_P2 + 2) : (IO_P3 + 2);
	unsigned	mask = 1 << ((axis % 4) << 1), curr ;

	curr = idsp_read_dm(dsp, addr);

	*enabled = (curr & mask) != 0 ;
	return dsp_error ;
}


/*
	Configuration-related functions.
*/

int16 FNTYPE pcdsp_step_speed(PDSP dsp, int16 axis, P_INT spd)
{
	P_DSP_DM step_cfg = STEP_CFG ;
	int16
		curr =	(idsp_read_dm(dsp, step_cfg) & 0xFF) |
				((idsp_read_dm(dsp, (P_DSP_DM)(step_cfg + 1)) & 0xFF) << 8),
		mask =	3 << (axis * 2);

	*spd = curr & mask ;
	*spd >>= (axis << 1);
	*spd &= 3;

	return dsp_error ;
}


int16 FNTYPE pcdsp_set_step_speed(PDSP dsp, int16 axis, int16 spd)
{
	P_DSP_DM step_cfg = STEP_CFG ;
	int16
		curr =	(idsp_read_dm(dsp, step_cfg) & 0xFF) |
				((idsp_read_dm(dsp, (P_DSP_DM)(step_cfg + 1)) & 0xFF) << 8),
		mask =	3 << (axis << 1);

	DSP_DM
		u,
		v ;

	if (spd < 0)			spd = 0;
	if (spd > 3)			spd = 3;
	spd <<= (axis << 1);

	curr &= ~mask ;
	curr |= spd ;

	u = curr & 0xFF ;
	v = curr >> 8 ;

	idsp_write_dm(dsp, step_cfg, u);
	idsp_write_dm(dsp, (P_DSP_DM)(step_cfg + 1),  v);

	return dsp_error ;
}


int16 FNTYPE pcdsp_closed_loop(PDSP dsp, int16 axis)
{
	int16 curr ;
	P_DSP_DM step_cfg = STEP_CFG ;

	curr = idsp_read_dm(dsp, (P_DSP_DM)(step_cfg + PORT_C));
	return (curr & (1 << (axis / 2))) != 0;
}


int16 FNTYPE pcdsp_set_closed_loop(PDSP dsp, int16 axis, int16 closed)
{
	P_DSP_DM step_cfg = STEP_CFG ;
	DSP_DM
		curr = idsp_read_dm(dsp, (P_DSP_DM)(step_cfg + PORT_C)) & 0xFF;

	if (closed)
		curr |= (1 << (axis / 2));
	else
		curr &= ~(1 << (axis / 2));

	idsp_write_dm(dsp, (P_DSP_DM)(step_cfg + PORT_C), curr);
	return dsp_error ;
}

int16 FNTYPE pcdsp_home_index_config(PDSP dsp, int16 axis)
{
	P_DSP_DM step_ports = STEP_PORTS ;
   int16   shift, config, curr;

	if (pcdsp_sick(dsp, axis))
		return dsp_error;

	curr = idsp_read_dm(dsp, (P_DSP_DM)(step_ports + PORT_C)) & 0xFF;
	shift = axis/4;
	if(shift)
      shift++;
   config = ((~curr >> shift) & 3);

   return config;
}

int16 FNTYPE pcdsp_set_home_index_config(PDSP dsp, int16 axis, int16 config)
{
   int16   shift, mask;
	DSP_DM curr;
	P_DSP_DM step_ports = STEP_PORTS ;

	if (pcdsp_sick(dsp, axis))
		return dsp_error;

	curr = idsp_read_dm(dsp, (P_DSP_DM)(step_ports + PORT_C)) & 0xFF ;

	shift = axis/4;
	if(shift)
      shift++;
	mask = 3 << shift;
   config = config << shift;
   curr |= mask;

	curr &= ~config;

	idsp_write_dm(dsp, (P_DSP_DM)(step_ports + PORT_C), curr);
	return dsp_error;
}



int16 FNTYPE pcdsp_init_timer(PDSP dsp, int16 counter, int16 mode)
{
	DSP_DM
		control = (counter << 6) | (mode << 1) | 0x30 ;

	if ((counter < 0) || (counter >= PCDSP_8254_TIMERS))
		return (dsp_error = DSP_ILLEGAL_TIMER);

	idsp_write_dm(dsp, SEL_8254 + CTRL_8254, control) ;

	return dsp_error;
}


int16 FNTYPE pcdsp_set_timer(PDSP dsp, int16 channel, unsigned16 t)
{
	DSP_DM
		u = t & 0xFF, v = t >> 8;

	if ((channel < 0) || (channel >= PCDSP_8254_TIMERS))
		return (dsp_error = DSP_ILLEGAL_TIMER);

	idsp_write_dm(dsp, (P_DSP_DM)(SEL_8254 + channel), u);
	idsp_write_dm(dsp, (P_DSP_DM)(SEL_8254 + channel), v);
	return dsp_error ;
}


int16 FNTYPE pcdsp_get_timer(PDSP pdsp, int16 channel)
{
	int16 r = 0;

	if (!pcdsp_init_check(pdsp) && ((channel >= 0) || (channel < PCDSP_8254_TIMERS)))
	{	DSP_DM ch = (channel << 6) ;
		idsp_write_dm(pdsp, SEL_8254 + CTRL_8254, ch) ;/* latch */
		r = idsp_read_dm(pdsp, (P_DSP_DM)(SEL_8254 + channel)) & 0xFF ;
		r |= (idsp_read_dm(pdsp, (P_DSP_DM)(SEL_8254 + channel)) << 8) ;
	}
	else
		dsp_error = DSP_ILLEGAL_TIMER ;

	return (unsigned16)r ;
}

int16 FNTYPE pcdsp_io_mon(int16 port, int16 *status)
{
    int16 io_mon, port_addr;

    if(!pcdsp_init_check(dspPtr))
    {
       if( (port < 0) || (port >= MONITORED_PORTS))
           return (dsp_error = DSP_ILLEGAL_IO);

       port_addr = port < 3 ? port + 4 : port - 3; /* port map: 345X012 */
    
       io_mon = dsp_read_dm(DM_IO_BLOCK) + IO_MON + port_addr;
       *status = dsp_read_dm(io_mon);
    }
    return(dsp_error);
}

int16 FNTYPE pcdsp_get_io_mon_mask(int16 port, int16 *mask, int16 *value)
{
    int16 io_mask, io_val, port_addr;

    if(!pcdsp_init_check(dspPtr))
    {
      if( (port < 0) || (port >= MONITORED_PORTS))
         return (dsp_error = DSP_ILLEGAL_IO);

      port_addr = port < 3 ? port + 4 : port - 3; /* port map: 345X012 */
    
      io_mask = dsp_read_dm(DM_IO_BLOCK) + IO_MASK + port_addr;
      io_val = dsp_read_dm(DM_IO_BLOCK) + IO_EXPECTED + port_addr;
      *mask = dsp_read_dm(io_mask);
      *value = dsp_read_dm(io_val);
    }
    return(dsp_error);
}
int16 FNTYPE pcdsp_set_io_mon_mask(int16 port, int16 mask, int16 value)
{
    int16 io_mask, io_val, port_addr;

    if(!pcdsp_init_check(dspPtr))
    {
      if( (port < 0) || (port >= MONITORED_PORTS))
         return (dsp_error = DSP_ILLEGAL_IO);

      port_addr = port < 3 ? port + 4 : port - 3; /* port map: 345X012 */
    
      io_mask = dsp_read_dm(DM_IO_BLOCK) + IO_MASK + port_addr;
      io_val = dsp_read_dm(DM_IO_BLOCK) + IO_EXPECTED + port_addr;
      dsp_write_dm(io_mask,mask);
      dsp_write_dm(io_val,value);
    }
    return(dsp_error);
}

int16 FNTYPE pcdsp_clear_io_mon(void)
{
    int16 i, io_mon, clr[MONITORED_PORTS+2];

    if(!pcdsp_init_check(dspPtr))
    {
      io_mon = dsp_read_dm(DM_IO_BLOCK);
      dsp_write_dm((unsigned16)(io_mon + IO_CHANGE),0);

      for(i = 0; i < MONITORED_PORTS + 2; i++)
      {
         clr[i] = 0;
      }

	   pcdsp_transfer_block(dspPtr, FALSE, FALSE, (P_DSP_DM)(io_mon + IO_STAT),
         MONITORED_PORTS + 2, (PDSP_DM) clr);
    }
    return(dsp_error);
}

int16 FNTYPE pcdsp_io_changed(void)
{
    int16 io_mon;

    io_mon = dsp_read_dm(DM_IO_BLOCK) + IO_CHANGE;
    return dsp_read_dm(io_mon) != 0;
}

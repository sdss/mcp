shellPromptSet ("mcp-> ")
tyBackspaceSet(0x7F)
#
# Turn on all memory debugging options
#
memOptionsSet 0x1ff
#
# Shut the network interface diagnostics off
#
eiDebug = 0xfc
#
_tzname = "GMT"
#
hostAdd "sdss-hub.apo.nmsu.edu", "10.25.1.1"
hostAdd "sdss-host2.apo.nmsu.edu", "10.25.1.26"
hostAdd "tai-time.apo.nmsu.edu", "10.25.1.1"
hostAdd "utc-time.apo.nmsu.edu", "10.25.1.1"
#
nfsMount("sdss-hub.apo.nmsu.edu", "/home/vxworks", "/home/vxworks")
nfsMount("sdss-hub.apo.nmsu.edu", "/home/sdss4/products", "/home/sdss4/products")
nfsMount("sdss-hub.apo.nmsu.edu", "/linuxp/prd", "/linuxp/prd")
nfsMount("sdss-hub.apo.nmsu.edu", "/mcptpm", "/mcptpm")
#
# Add user vxworks (pid 602, gid 602)
#
au_p = malloc(4); *au_p = 602		/* group vxworks */
nfsAuthUnixSet "sdss-hub.apo.nmsu.edu", 602, 602, 1, au_p
#
# Go to the version root
#
cd "/home/vxworks/mcpbase"
#
# Remove MCP main RAM Slave Map that is defined in the vxWorks image.
#
#ld < vx_tools/objects/mvsup.mv162.o
#vmeSlaveMap1 0
#
# Load Ron's tracing tools from vx_tools
#
# traceClk = 0
ld < vx_tools/lib/vxt.mv162.o
#
# Load murmur
#
ld < vx_tools/objects/dvx_var_dvx.mv162.o
ld < murmur_client_vx/lib/muruser.m68040.o
#
# Load taskSwitchHook utils for the use of the tracer
#
ld < util/dscTrace.o
trace_limPerTimPeriod = 30	/* control throttling of murmur messages */
#
# Initialize MVME162 VMEchip2 tick timer 2 for use as a rolling micro-
# second timer for trace (and for the clk2Read and clk2Diff macros).
#
clk2Stop()			/* VMEchip2 tick timer 2 */
clk2Start ()			/* VMEchip2 tick timer 2 */
#
# Enable/disable tracing (via commenting).
#
# The main trace philosophy is that higher levels will print out more
# often as the system runs (if those levels are enabled).  The DA conventions
# for trace levels are:
#
#	   levels	meaning
#	   ------	-------------------------------------------------------
#	   0 -  3	Messages of interest during normal operation.
#	   4 -  7	Messages possibly of interest during normal operation.
#	   8 - 30	Messages used for debugging.
#	       31	Messages showing task switches.
#
# A further suggestion is to use an even trace level for subroutine entry
# and that level, plus one, for trace entries within the subroutine.
#
traceInit(80000, 40, 0xFFF4005c) /* (EntryCnt, MaxTask, TickTimer2) */
traceInitialOn(0, 30)
traceOn 0,  0,3; traceOn 0, 16,16	/* TRACE0 is for ISPs */
traceTtyOn 0, 4
traceMode 0x5			/* Entry -> queue -> user-defined fn */

taskSwitchHookAdd trc_tskSwHk	/* prepare to trace task switches */
excHookAdd trc_excHook		/* prepare to trace exceptions */

repeat 1, taskDelay, 0		/* need a task switch to create pSwHook */
traceOn 1, 31,31		/* trace task switches */

#
# Start MURMUR related items, including the server.
#
mur_set_proc_name "MCP"

taskSpawn "tMurServerAdd", 100, 0, 10000, mur_server_add, "sdss-host2.apo.nmsu.edu"
taskSpawn "tMurServerRetry", 100, 0, 10000, mur_server_retry, 30
taskSpawn "tMurRouter", 70, 0, 20000, mur_route_start, 200

#
# Add hooks for task creation/death
#
ld < util/tasks.o

taskCreateHookAdd taskCheckOnCreate
taskDeleteHookAdd taskCheckOnDelete

repeat 1, taskDelay, 0		/* create/delete a task */
traceOn 2, 31,31		/* trace task creation */
traceOn 3, 31,31		/* trace task deletion */

#
# Allow gdb to debug tasks called from the shell; 0x2 == VX_UNBREAKABLE
#
#taskOptionsSet tShell, 0x2, 0x0

#
# Spawn idle and timer tasks
#
taskSpawn "tIdleTask", 255, 0, 1000, idle
# RHL
#ld < util/timerTask.o
#timer_debug = 2
# n.b. timerStart has too small a stack
#timerStart
taskSpawn "tTimerTask", 5, 0, 7000, timerTask, 0,0
#
# IndustryPack serial drivers
#
#ld < ip/ipOctalSerial.o
ld < ip/mv162IndPackInit.o
ld < ip/systran/dio316.out
ld < ip/systran/dac128v.out
ld < ip/systran/adc128f1.out
ld < ip/systran/did48.out
#ld < ip/acromag/ip470.out
ld < ip/acromag/ip480.out
ld < util/utiltim.out
ld < util/timer.o
#
# Allen-Bradley SLC504
#
ld < ab/dhp.out
# change from default 57Kb to 115Kb
#DHP_baud=0x1
# change from default 57Kb to 230Kb
#DHP_baud=0x2
vmeinst (0x1000, 0x0e0000, 0x00, "ab/sddhp.bin")
#vmeinst (0x1000, 0x0e0000, 0x00, "sdhdhp.bin") 
#115Kb
#vmeinst (0x1000, 0x0e0000, 0x00, "sdudhp.bin") 
#230Kb
dhpd (10,0xe000,"chasb")
#
# Load slalib for the sake of the MJD.  We now use our copy as this
# version doesn't appear to be reentrant
#
# ld < /tpmbase/bin/mv162/slaLib
#
# Load NTP code
#
#ld < /astrobase/node/sdssid1/ntpvx/usrTime/usrLoad.mv167.o
ld < util/ntp.o
#
# Load the MCP itself
#
ld 1, 0, "src/mcpnew.out"

#ld < mei-src/llfirm.o
#ld < src/util.o
#ld < src/telescope_motion.o
#ld < src/counter_weight.o
#ld < src/serial.o
#ld < src/instrument_lift.o
#ld < src/data_collection.o
#ld < src/ipcast.o
#ld < src/ipsym.o
#ld < src/display.o
#ld < src/umbilical.o
#ld < src/telnetCmds.o
#ld < src/tagname.o
#ld < src/axis_cmds.o
#ld < src/cmd.o
#
# Initialise message queues and semaphores
#
cmdInit "The MCP has rebooted"
axisMotionInit
tMoveCWInit 0xFFFF4000, 0xC0
tLatchInit
spectroInit
tBarsInit
timeInit
tBrakesInit
as2Init
loadKeywordDictionary

#BCAST_Enable=0
#SM_COPY=0
#rebootHookAdd disable_trc_excHook
#rebootHookAdd ip_shutdown
DID48_initialize(0xFFF58000,0xB8)
lift_initialize(0xFFFF4000)
taskPrioritySet (taskIdFigure("tExcTask"),1)
#
# Get the current time from the NTP server
#
setSDSStimeFromNTP 1
#
# N.b. The `frequencies' supplied to the *_data_collection routines
# are interpreted relative to the rate that TimerStart calls serverDCStart,
#
# The other `frequencies' are really _inverse_ frequencies; they set the
# number of DC_freq ticks between calls to various data collection tasks
#
ADC128F1_initialize (0xfff58000,0)
taskSpawn "serverData",75,8,10000,serverData,1,DataCollectionTrigger
taskSpawn "MEI_DC",48,8,10000,mei_data_collection,1
taskSpawn "SLC_DC",69,8,10000,slc500_data_collection,20
check_encoder_freq = 20*60
TimerStart 20, 5, serverDCStart
ipsdss_ini
init_broadcast_ipsdss

set_rot_state -1
ffs_enable 1		/* enable the Flat Field Screen */

dbgInit
date
DIO316_initialize(0xFFF58000,0xB0)
tm_setup_wd()
taskSpawn "ampMgt",45,0,5000,tm_amp_mgt
VME162_IP_Memory_Enable (0xfff58000,3,0x72000000)
#taskSpawn "barcodcan",85,8,2500,cancel_read
#barcode_init(0xfff58000,0xf022,0xAA,2)
#taskDelay (60)
#azimuth barcode (2=altitude)
#barcode_open (3)
#barcode_serial 3
#
# Load fiducials tables
#
# AZIMUTH = 0
read_fiducials "/mcpbase/fiducial-tables/az.dat", 0
set_max_fiducial_correction 0, 600
set_min_encoder_mismatch_error 0, 1000
#
# ALTITUDE = 1
read_fiducials "/mcpbase/fiducial-tables/alt.dat", 1
set_max_fiducial_correction 1, 600
set_min_encoder_mismatch_error 1, 1000
#
# INSTRUMENT = 2 (== rotator)
read_fiducials "/mcpbase/fiducial-tables/rot.dat", 2
set_max_fiducial_correction 2, 600
set_min_encoder_mismatch_error 2, 1000
#
# Now that we're up, listen to the TCC and mcpMenu. They may have been trying
# to talk to us all of this time
#
oldServerPort = 31011
newServerPort = 31012
taskSpawn "tCPS",100,0,2000,cmdPortServer,oldServerPort
taskSpawn "tNewCPS",100,0,2000,cmdPortServer,newServerPort
taskSpawn "TCC",46,8,25000,tcc_serial,1
#
# Adjust tracing now that we're up
#
#traceOff barcodcan, 16,16
traceTtyOn 0, 4
#
# List all tasks in case we get a task ID later; can help with debugging
#
i

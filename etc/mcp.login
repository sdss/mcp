shellPromptSet ("mcp-> ")
tyBackspaceSet(0x7F)
#
# Turn on all memory debugging options
#
memOptionsSet 0x1ff
#
hostAdd "sdsshost.apo.nmsu.edu", "192.41.211.171"
hostAdd "tcc25m.apo.nmsu.edu",   "192.41.211.162"
hostAdd "utc-time.apo.nmsu.edu", "192.41.211.40"
#
routeAdd("0", "192.41.211.1")
#
nfsMount("sdsshost.apo.nmsu.edu", "/p", "/p")
nfsMount("sdsshost.apo.nmsu.edu", "/home", "/home")
nfsMount("sdsshost.apo.nmsu.edu", "/usrdevel", "/usrdevel")
nfsMount("sdsshost.apo.nmsu.edu", "/mcptpm", "/mcptpm")
#
# Add user vxboot (pid 5036, gid 5530) to group products
#
au_p = malloc(4); *au_p = 4525		/* group products */
nfsAuthUnixSet "sdsshost.apo.nmsu.edu", 5036, 5530, 1, au_p
#
# Go to the version root
#
cd "/p/mcpbase"

#
# Load Ron's tracing tools from vx_tools
#
ld < vx_tools/lib/vxt.mv167.o
#
# Load murmur
#
ld < vx_tools/objects/dvx_var_dvx.mv167.o
ld < murmur_client_vx/lib/muruser.m68040.o
#
# Load taskSwitchHook utils for the use of the tracer
#
ld < util/dscTrace.o
trace_limPerTimPeriod = 30	/* control throttling of murmur messages */
#
# Initialize MVME167 VMEchip2 tick timer 2 for use as a rolling micro-
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
traceInit(30000, 40, 0xFFF4005c) /* (EntryCnt, MaxTask, TickTimer2) */
traceInitialOn(0, 30)
traceOn 0,  0,3; traceOn 0, 16,16	/* TRACE0 is for ISPs */
traceTtyOn 0, 3
traceMode 5			/* Entry -> queue -> user-defined fn */

taskSwitchHookAdd trc_tskSwHk	/* prepare to trace task switches */
excHookAdd trc_excHook		/* prepare to trace exceptions */

repeat 1, taskDelay, 0		/* need a task switch to create pSwHook */
traceOn 1, 31,31		/* trace task switches */

#
# Start MURMUR related items, including the server.
#
mur_set_proc_name "MCP"

taskSpawn "tMurServerAdd", 100, 0, 10000, mur_server_add, "sdsshost.apo.nmsu.edu"
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
# Spawn idle and timer tasks
#
taskSpawn "tIdleTask", 255, 0, 500, idle
# RHL
#ld < util/timerTask.o
# n.b. timerStart has too small a stack
#timerStart
taskSpawn "tTimerTask", 5, 0, 7000, timerTask, 0,0
#
# IndustryPack serial drivers
#
ld < ip/ipOctalSerial.o
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
# Load slalib for the sake of the MJD
#
ld < /p/tpmbase/bin/mv162/slaLib
#
# Load NTP code
#
ld < /p/astrobase/node/sdssid1/ntpvx/usrTime/usrLoad.mv167.o
ld < util/ntp.o
#
# Load the MCP itself
#
ld < mcp-new/mcpnew.out
#ld < mei-new/llfirm.o
#ld < mcp-new/util.o
#ld < mcp-new/telescope_motion.o
#ld < mcp-new/counter_weight.o
#ld < mcp-new/serial.o
#ld < mcp-new/instrument_lift.o
#ld < mcp-new/data_collection.o
#ld < mcp-new/ipcast.o
#ld < mcp-new/ipsym.o
#ld < mcp-new/display.o
#ld < mcp-new/umbilical.o
#ld < mcp-new/telnetCmds.o
#ld < mcp-new/tagname.o
#ld < mcp-new/axis_cmds.o
#ld < mcp-new/cmd.o
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

#BCAST_Enable=0
#SM_COPY=0
rebootHookAdd (ip_shutdown)
DID48_initialize(0xFFF58000,0xB8)
lift_initialize(0xFFFF4000)
cmd_handler("init")
taskSpawn "TCC",46,8,10000,tcc_serial,1
taskSpawn "cmdPortServer",100,0,2000,cmdPortServer,31011
taskPrioritySet (taskIdFigure("tExcTask"),1)
#
# Get the current time from the NTP server
#
setTimeFromNTP "utc-time.apo.nmsu.edu", 0, 1, 0

ADC128F1_initialize (0xfff58000,0)
taskSpawn "serverData",75,8,10000,serverData,1,DataCollectionTrigger
taskSpawn "MEI_DC",48,8,10000,mei_data_collection,1
taskSpawn "SLC_DC",70,8,10000,slc500_data_collection,100
TimerStart (100,5,serverDCStart)
ipsdss_ini
serverSetSym
set_rot_state -1
ffs_enable 1		/* enable the Flat Field Screen */

dbgInit
date
DIO316_initialize(0xFFF58000,0xB0)
tm_setup_wd()
taskSpawn "ampMgt",45,0,2000,tm_amp_mgt
taskSpawn "taskTrg",100,8,10000,taskTrg
VME162_IP_Memory_Enable (0xfff58000,3,0x72000000)
taskSpawn "barcodcan",85,8,1500,cancel_read
barcode_init(0xfff58000,0xf022,0xAA,2)
taskDelay (60)
#azimuth barcode (2=altitude)
barcode_open (3)
#barcode_serial 3
#
# Adjust tracing now that we're up
#
traceOff barcodcan, 16,16
traceTtyOn 0, 4

shellPromptSet ("MCP->")
tyBackspaceSet(0x7F)
hostAdd ("oper","192.41.211.171")
#
# OK, this routing entry (use the router to get to the
# other half of the cockamamie APO merged subnet that we
# are physically on) is somehow getting added during boot.
# It keeps us from being able to broadcast to our own network.
# Delete it.
# routeDelete("192.41.211.160", "192.41.211.161")
#
routeAdd("0", "192.41.211.1")
#routeAdd("192.41.211.160", "192.41.211.166")
#
netDevCreate("galileo:","galileo",0)
#nfsMount ("galileo","/export/apotop/visitor1","/vxdsp")
# get to the version root
cd "/home/briegel/mcp/"
ld < mcp/ip/ipOctalSerial.o
ld < mcp/ip/mv162IndPackInit.o
ld < mcp/ip/systran/dio316.out
ld < mcp/ip/systran/dac128v.out
ld < mcp/ip/systran/adc128f1.out
ld < mcp/ip/systran/did48.out
#ld < mcp/ip/acromag/ip470.out
ld < mcp/ip/acromag/ip480.out
ld < mcp/util/utiltim.out
ld < mcp/util/timer.o
ld < mcp/util/vxt.mv167.o
ld < mcp/util/muruser.m68040.o
#mur_send_text("Klink! Who ees thees man?!")
#
# Allen-Bradley SLC504
ld < mcp/ab/dhp.out
# change from default 57Kb to 115Kb
#DHP_baud=0x1
# change from default 57Kb to 230Kb
#DHP_baud=0x2
vmeinst (0x1000, 0x0e0000, 0x00, "mcp/ab/sddhp.bin")
#vmeinst (0x1000, 0x0e0000, 0x00, "sdhdhp.bin") 
#115Kb
#vmeinst (0x1000, 0x0e0000, 0x00, "sdudhp.bin") 
#230Kb
dhpd (10,0xe000,"chasb")
#tests************************
#sbslc (4,7,150,10,10,0xe000,"cb")
#rdslc (4,7,0,110,10,0xe000,"cb")
#rdslc (4,7,110,50,10,0xe000,"cb")
#slc_read_blok (4,3,0x85,0,malloc(24*2),24)
#
ld < mcp/mcp-new/mcpnew.out
#BCAST_Enable=0
#SM_COPY=0
rebootHookAdd (ip_shutdown)
cmd_init()
DID48_initialize(0xFFF58000,0xB8)
balance_initialize(0xFFFF4000,0xC0)
lift_initialize(0xFFFF4000)
cmd_handler("init")
taskSpawn "TCC",46,8,10000,tcc_serial,1
taskPrioritySet (taskIdFigure("tExcTask"),1)
iptimeSet ("sdsshost",0)
sdss_init()
IL_Verbose()
taskDelay(20)
ADC128F1_initialize (0xfff58000,0)
taskSpawn "serverData",75,8,10000,serverData,1,DataCollectionTrigger
taskSpawn "MEI_DC",48,8,10000,mei_data_collection,1
taskSpawn "SLC_DC",70,8,10000,slc500_data_collection,100
TimerStart (100,5,serverDCStart)
ipsdss_ini()
serverSetSym
set_rot_state (-1)
tm_set_coeffs 0,0,160
tm_set_coeffs 0,1,6
tm_set_coeffs 0,2,1500
tm_set_coeffs 0,7,18000
tm_set_coeffs 0,8,-4
tm_set_coeffs 0,9,0
tm_set_coeffs 2,0,120
tm_set_coeffs 2,1,6
tm_set_coeffs 2,2,1200
tm_set_coeffs 2,7,10000
tm_set_coeffs 2,8,-4
tm_set_coeffs 4,0,120
tm_set_coeffs 4,1,12
tm_set_coeffs 4,2,600
tm_set_coeffs 4,4,0
tm_set_coeffs 4,7,12000
tm_set_coeffs 4,9,0
tm_set_coeffs 4,8,-5
tm_print_coeffs 0
tm_print_coeffs 2
tm_print_coeffs 4
dbgInit
date
DIO316_initialize(0xFFF58000,0xB0)
tm_setup_wd()
# switch for testing new mcpnew
###...taskPrioritySet(period (1,tm_amp_engage),47)
taskSpawn "ampMgt",45,0,1000,tm_amp_mgt
start_tm_TCC()
taskSpawn "taskTrg",100,8,10000,taskTrg
VME162_IP_Memory_Enable (0xfff58000,3,0x72000000)
taskSpawn "barcodcan",85,8,1000,cancel_read
barcode_init(0xfff58000,0xf022,0xAA,2)
taskDelay (60)
#azimuth barcode (2=altitude)
barcode_open (3)
#barcode_serial 3

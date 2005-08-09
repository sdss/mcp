#ifndef DSC_MSG_C_HEADER
#define DSC_MSG_C_HEADER
/******************************************************************
**
**                    MURMUR Message Utility
**
** File: ./dsc_msg_c.h
** Created: Mon Aug  8 10:58:50 2005
** Message Input File: ./dsc.msg
** By User: products
**
******************************************************************/
/* ****************************** Copyright Notice ******************************* */
/*                                                                               * */
/*   Copyright (c) 1992 Universities Research Association, Inc.                  * */
/*                 All Rights Reserved.                                          * */
/*                                                                               * */
/* ******************************************************************************* */
/*  @+Public+@ */
/*  PROJECT:	Drift Scan Camera */
/*  */
/*  ABSTRACT:	Message definitions.  Header/include files are generated with */
/* 		the following command: */
/*  */
/* 		setup murmur */
/* 		message -sl -sdb dsc.msg */
/*  */
/*  ENVIRONMENT:	MURMUR message file. */
/* 		dsc.msg */
/*  */
/*  AUTHOR:	Bryan MacKinnon, Creation date: */
/* 		Tom Nicinski */
/* 		Catherine Pluquet */
/* 		Gary Sergey */
/* 		Penelope Constanta-Fanourakis */
/*  @-Public-@ */
/* ******************************************************************************* */
/* HeaderEnd Necessary for nawk to know where the status header ends. */
/*  Note: an underscore will automatically appended to the prefix */
#define dsc_facnum 0x3
#define	   dsc_success    0x8000C009
#define	   dsc_frameNonPerm    0x8000C013 /*  Used as parameter value */
#define	   dsc_framePerm    0x8000C01B /*  Used as parameter value */
#define	   dsc_spacingProp    0x8000C023 /*  Used as parameter value */
#define	   dsc_spacingCell    0x8000C02B /*  Used as parameter value */
#define	   dsc_upperLeft    0x8000C033 /*  Used as configuration value */
#define	   dsc_upperRight    0x8000C03B /*  Used as configuration value */
#define	   dsc_lowerLeft    0x8000C043 /*  Used as configuration value */
#define	   dsc_lowerRight    0x8000C04B /*  Used as configuration value */
#define	   dsc_trcSucc    0x8000C051
#define	   dsc_trcInfo    0x8000C05B
#define	   dsc_trcWarn    0x8000C062
#define	   dsc_trcErr    0x8000C06C
#define	   dsc_accFailSCD    0x8000C074
#define	   dsc_alrLoggedIn    0x8000C07B
#define	   dsc_alrOpen    0x8000C082
#define	   dsc_alrReadCCD    0x8000C08A
#define	   dsc_archBadLabel    0x8000C094
#define	   dsc_archDErrC    0x8000C09B
#define	   dsc_archBOT    0x8000C0A4
#define	   dsc_archEOT    0x8000C0AB
#define	   dsc_archDirBad    0x8000C0B4
#define	   dsc_archFail    0x8000C0BC
#define	   dsc_archFailMsg    0x8000C0C3
#define	   dsc_archInited    0x8000C0CB
#define	   dsc_archLabel    0x8000C0D3
#define	   dsc_archLoadTape    0x8000C0DA
#define	   dsc_archMsgSFail    0x8000C0E4
#define	   dsc_archMsgRFail    0x8000C0EC
#define	   dsc_archNoAppend    0x8000C0F4
#define	   dsc_archNoControl    0x8000C0FC
#define	   dsc_archNoFtp    0x8000C102
#define	   dsc_archNoInit    0x8000C10C
#define	   dsc_archNoFMK    0x8000C114
#define	   dsc_archNoLabel    0x8000C11C
#define	   dsc_archNotInited    0x8000C124
#define	   dsc_archReadFail    0x8000C12B
#define	   dsc_archShutDown1    0x8000C134
#define	   dsc_archShutDown2    0x8000C13C
#define	   dsc_archStBadLabel    0x8000C143
#define	   dsc_archStDirBad    0x8000C14C
#define	   dsc_archStBusy    0x8000C153
#define	   dsc_archStDisabled    0x8000C15B
#define	   dsc_archStEOT    0x8000C163
#define	   dsc_archStError    0x8000C16B
#define	   dsc_archStInit    0x8000C173
#define	   dsc_archStInitPen    0x8000C17B
#define	   dsc_archStLoad    0x8000C183
#define	   dsc_archStModeSel    0x8000C18B
#define	   dsc_archStMount    0x8000C193
#define	   dsc_archStNoInit    0x8000C19B
#define	   dsc_archStNoLabel    0x8000C1A3
#define	   dsc_archStNoTape    0x8000C1AB
#define	   dsc_archStNotMount    0x8000C1B3
#define	   dsc_archStPause    0x8000C1BB
#define	   dsc_archStPosition    0x8000C1C3
#define	   dsc_archStRead    0x8000C1CB
#define	   dsc_archStReadLbl    0x8000C1D3
#define	   dsc_archStReady    0x8000C1DB
#define	   dsc_archStRewind    0x8000C1E3
#define	   dsc_archStsFail    0x8000C1EC
#define	   dsc_archStStop    0x8000C1F3
#define	   dsc_archStStopped    0x8000C1FC
#define	   dsc_archStUnload    0x8000C203
#define	   dsc_archStWait    0x8000C20B
#define	   dsc_archStWriteLbl    0x8000C213
#define	   dsc_archTapeStat    0x8000C21B
#define	   dsc_archWrFail    0x8000C224
#define	   dsc_archWrBOT    0x8000C22C
#define	   dsc_archRcvrRG    0x8000C233
#define	   dsc_archRcvrRB    0x8000C23B
#define	   dsc_archRcvrUB    0x8000C243
#define	   dsc_archRcvrMG    0x8000C24B
#define	   dsc_archRcvrDone    0x8000C253
#define	   dsc_archRcvrBad    0x8000C25B
#define	   dsc_asmFrameAcq    0x8000C263
#define	   dsc_asmFramePooled    0x8000C26B
#define	   dsc_asmFrameRegioned    0x8000C273
#define	   dsc_badICC    0x8000C27C
#define	   dsc_badRange    0x8000C284
#define	   dsc_calibrated    0x8000C28B
#define	   dsc_ccdFailure    0x8000C294
#define	   dsc_ccdSRError    0x8000C29A
#define	   dsc_ccdWarning    0x8000C2A2
#define	   dsc_chksumMismatch    0x8000C2AA
#define	   dsc_cliBadParam    0x8000C2B4
#define	   dsc_cliBadState    0x8000C2BC
#define	   dsc_cliCreateFail    0x8000C2C4
#define	   dsc_cliFail    0x8000C2CC
#define	   dsc_cliTooMany    0x8000C2D4
#define	   dsc_createFail    0x8000C2DC
#define	   dsc_dataDiscard    0x8000C2E2
#define	   dsc_dataEOF    0x8000C2E9
#define	   dsc_dataFlush    0x8000C2F2
#define	   dsc_dataTrunc    0x8000C2FA
#define	   dsc_defParam    0x8000C303
#define	   dsc_devPartFail    0x8000C30C
#define	   dsc_diskInitFail    0x8000C314
#define	   dsc_dqBufEmpty    0x8000C31C
#define	   dsc_dqBufNotEmpty    0x8000C324
#define	   dsc_errno    0x8000C32B
#define	   dsc_establishFail    0x8000C334
#define	   dsc_flushFail    0x8000C33C
#define	   dsc_frameArchived    0x8000C343
#define	   dsc_frameDeleted    0x8000C34B
#define	   dsc_frameExists    0x8000C354
#define	   dsc_frameFtped    0x8000C35B
#define	   dsc_frameGetFail    0x8000C363
#define	   dsc_frameHdrExists    0x8000C36C
#define	   dsc_frameIncomplete    0x8000C374
#define	   dsc_frameInfo    0x8000C37B
#define	   dsc_framePutFail    0x8000C383
#define	   dsc_frameTooBig    0x8000C38C
#define	   dsc_fsInitFail    0x8000C394
#define	   dsc_groupExists    0x8000C39B
#define	   dsc_ICIinProg    0x8000C3A1
#define	   dsc_ICIunavail    0x8000C3AC
#define	   dsc_iccReady    0x8000C3B3
#define	   dsc_ignDev    0x8000C3BA
#define	   dsc_ignDevToo    0x8000C3C2
#define	   dsc_ignDsk    0x8000C3CA
#define	   dsc_insfFrameArea    0x8000C3D2
#define	   dsc_insfMem    0x8000C3DC
#define	   dsc_insfPool    0x8000C3E4
#define	   dsc_insfSts    0x8000C3EC
#define	   dsc_ioctlFail    0x8000C3F4
#define	   dsc_ivArNdx    0x8000C3FC
#define	   dsc_ivCCDConfig    0x8000C404
#define	   dsc_ivClient    0x8000C40C
#define	   dsc_ivDirDev    0x8000C414
#define	   dsc_ivDriftAmps    0x8000C41C
#define	   dsc_ioFail    0x8000C424
#define	   dsc_ivFlag    0x8000C42C
#define	   dsc_ivFrameCtx    0x8000C434
#define	   dsc_ivFrameSpec    0x8000C43C
#define	   dsc_ivGroupSpec    0x8000C444
#define	   dsc_ivICCconfig    0x8000C44C
#define	   dsc_ivLineRate    0x8000C452
#define	   dsc_ivMCresponse    0x8000C45A
#define	   dsc_ivMethod    0x8000C464
#define	   dsc_ivMode    0x8000C46C
#define	   dsc_ivParam    0x8000C474
#define	   dsc_ivParamCnt    0x8000C47C
#define	   dsc_ivSemID    0x8000C484
#define	   dsc_ivStsArType    0x8000C48C
#define	   dsc_ivTapeDrive    0x8000C494
#define	   dsc_ivDriveInv    0x8000C49C
#define	   dsc_ivTCCpkt    0x8000C4A2
#define	   dsc_ivVolConfig    0x8000C4AC
#define	   dsc_ivVisPixRow    0x8000C4B4
#define	   dsc_lineTMO    0x8000C4BC
#define	   dsc_loadFail    0x8000C4C4
#define	   dsc_loadFsMismatch    0x8000C4CC
#define	   dsc_loadMismatch    0x8000C4D2
#define	   dsc_loadVerMismatch    0x8000C4DC
#define	   dsc_lockCreateFail    0x8000C4E4
#define	   dsc_lockDestroyFail    0x8000C4EC
#define	   dsc_lockFail    0x8000C4F4
#define	   dsc_lockNotHeld    0x8000C4FC
#define	   dsc_login    0x8000C503
#define	   dsc_logout    0x8000C50B
#define	   dsc_MCresp    0x8000C513
#define	   dsc_MCunavail    0x8000C51C
#define	   dsc_mismRunCompile    0x8000C524
#define	   dsc_missingLines    0x8000C52A
#define	   dsc_mmapFail    0x8000C534
#define	   dsc_mountFail    0x8000C53C
#define	   dsc_newFits    0x8000C543
#define	   dsc_noAnal    0x8000C54B
#define	   dsc_noCLI    0x8000C554
#define	   dsc_noClientServer    0x8000C55C
#define	   dsc_noData    0x8000C564
#define	   dsc_noDelete    0x8000C56A
#define	   dsc_noDeleteAll    0x8000C572
#define	   dsc_noDev    0x8000C57C
#define	   dsc_noDevSym    0x8000C583
#define	   dsc_noFrame    0x8000C58C
#define	   dsc_noFrameHdr    0x8000C594
#define	   dsc_noFrameAreaMap    0x8000C59C
#define	   dsc_noGroup    0x8000C5A4
#define	   dsc_noICCinit    0x8000C5AC
#define	   dsc_noIntHandler    0x8000C5B4
#define	   dsc_noLock    0x8000C5BC
#define	   dsc_noMatch    0x8000C5C4
#define	   dsc_noMCinit    0x8000C5CC
#define	   dsc_noPeer    0x8000C5D4
#define	   dsc_noPoolInit    0x8000C5DC
#define	   dsc_noReadAccess    0x8000C5E2
#define	   dsc_noRename    0x8000C5EA
#define	   dsc_noRequest    0x8000C5F4
#define	   dsc_noShareMap    0x8000C5FC
#define	   dsc_noSIGhand    0x8000C602
#define	   dsc_noSts    0x8000C60A
#define	   dsc_noTCCinit    0x8000C614
#define	   dsc_notLoggedIn    0x8000C61C
#define	   dsc_notOpen    0x8000C624
#define	   dsc_NTPqueryFail    0x8000C62C
#define	   dsc_openFail    0x8000C634
#define	   dsc_pastEOF    0x8000C63C
#define	   dsc_poolAvail    0x8000C643 /*  Pool state */
#define	   dsc_poolCreateGroup    0x8000C64B /*  Pool         substate */
#define	   dsc_poolDeleteFrame    0x8000C653 /*  Pool         substate */
#define	   dsc_poolDeleteGroup    0x8000C65B /*  Pool         substate */
#define	   dsc_poolDeleteHdr    0x8000C663 /*  Pool         substate */
#define	   dsc_poolDirWipe    0x8000C66B
#define	   dsc_poolEstablish    0x8000C673 /*  Pool         substate */
#define	   dsc_poolGetFrameDat    0x8000C67B /*  Pool         substate */
#define	   dsc_poolGetFrameHdr    0x8000C683 /*  Pool         substate */
#define	   dsc_poolIdle    0x8000C68B /*  Pool         substate */
#define	   dsc_poolInit    0x8000C693 /*  Pool state & substate */
#define	   dsc_poolInited    0x8000C699
#define	   dsc_poolLoadDir    0x8000C6A3 /*  Pool         substate */
#define	   dsc_poolPutFrameDat    0x8000C6AB /*  Pool         substate */
#define	   dsc_poolPutFrameHdr    0x8000C6B3 /*  Pool         substate */
#define	   dsc_poolRename    0x8000C6BB /*  Pool         substate */
#define	   dsc_poolRenameGroup    0x8000C6C3 /*  Pool         substate */
#define	   dsc_poolSaveDir    0x8000C6CB /*  Pool         substate */
#define	   dsc_poolSetFrameFlg    0x8000C6D3 /*  Pool         substate */
#define	   dsc_poolSetParam    0x8000C6DB /*  Pool         substate */
#define	   dsc_poolUnavail    0x8000C6E4 /*  Pool state (also) */
#define	   dsc_probeFail    0x8000C6EC
#define	   dsc_ptOpenFail    0x8000C6F4
#define	   dsc_ptError    0x8000C6FC
#define	   dsc_ptBadConfig    0x8000C704
#define	   dsc_readFail    0x8000C70C
#define	   dsc_readoutBad    0x8000C714
#define	   dsc_requestInProg    0x8000C71C
#define	   dsc_shareLost    0x8000C724
#define	   dsc_skippedLine    0x8000C72A
#define	   dsc_spawnFail    0x8000C734
#define	   dsc_stsExist    0x8000C73C
#define	   dsc_systemType    0x8000C743
#define	   dsc_tapeMounted    0x8000C74B
#define	   dsc_TCCidle    0x8000C753 /*  TCC  state */
#define	   dsc_TCCinit    0x8000C75B /*  TCC  state */
#define	   dsc_TCCunavail    0x8000C764 /*  TCC  state (also) */
#define	   dsc_timeGetFail    0x8000C76C
#define	   dsc_tmo    0x8000C774
#define	   dsc_trimDisabled    0x8000C77B
#define	   dsc_trimFail    0x8000C783
#define	   dsc_trimFrame    0x8000C78B
#define	   dsc_unexpData    0x8000C792
#define	   dsc_unknwnHost    0x8000C79A
#define	   dsc_verMismatch    0x8000C7A4
#define	   dsc_vxCallFail    0x8000C7AA
#define	   dsc_writeFail    0x8000C7B4
#define	   dsc_txtWarn    0x8000C7BA /*  3 * 30 = 90 chars total */
#define	   dsc_txtErr    0x8000C7C4 /*  3 * 30 = 90 chars total */
#define	   dsc_sockCreate    0x8000C7CC
#define	   dsc_sockOpt    0x8000C7D2
#define	   dsc_sockBind    0x8000C7DC
#define	   dsc_sockListen    0x8000C7E4
#define	   dsc_sockAccept    0x8000C7EC
#define	   dsc_sockSelect    0x8000C7F4
#define	   dsc_tcpLnkErr    0x8000C7FC
#define	   dsc_tcpLnkNoHost    0x8000C804
#define	   dsc_tcpLnkProto    0x8000C80C
#define	   dsc_tcpLnkDataErr    0x8000C814
#define	   dsc_iccNodeBadEnv    0x8000C81C
#define	   dsc_iccNodeDirOpen    0x8000C824
#define	   dsc_iccNodeFileOpen    0x8000C82A
#define	   dsc_iccNodeInvID    0x8000C832
#define	   dsc_iccNodeEmpty    0x8000C83A
#define	   dsc_iccNodeDupID    0x8000C842
#define	   dsc_iccNodeMapEntry    0x8000C84B
#define	   dsc_iciLinkEnvVar    0x8000C853
#define	   dsc_tcpLnkOpenFail    0x8000C85C
#define	   dsc_tcpLnkListenErr    0x8000C864
#define	   dsc_tcpLnkAcceptErr    0x8000C86C
#define	   dsc_tcpLnkClientErr    0x8000C874
#define	   dsc_tcpLnkWriteHdr    0x8000C87C
#define	   dsc_tcpLnkReadHdr    0x8000C884
#define	   dsc_tcpLnkWriteData    0x8000C88C
#define	   dsc_tcpLnkReadData    0x8000C894
#define	   dsc_tcpLnkRetry    0x8000C89A
#endif

/* -------------------------------------------------------------------------
// Copyright (c) 1990-1994 S-S Technologies Inc.
//
// Module Name: diag.c
// -------------------
//
// Demo module used to get station types, and display diagnostic counters
// for various types of stations on Data Highway Plus
//
// This module was compiled with Borland C++ 3.1
//
//
// Version 1.00 - jan 93
// -------------------------------------------------------------------------*/

#include "vxWorks.h"
#include "stdlib.h"
#include "string.h"
#include "stdio.h"
#include "abdh.h"
#include "diag.h"
#include "std_dhp.h"

#define _n_ 0xff        /* field Not applicable                             */
#define _DIAG           /* define so diag.h header file works correctly     */

/* global variables (filled in by get_sta_type ()) */
DIAG_STS_REPLY dg_sts_rply;
TYPE_INFO_STUC *dg_type;
uchr dg_dst;

/* external variables */

/* diagnostic counter data format array */
DIAG_CTR_INFO_STUC DIAG_CTR_INFO[]=
{
    {_BYTE    , 31,  2,  1},      /*000 ACK Timeout               Data Format 0  */
    {_BYTE    , 31,  3,  2},      /*001 Xmit Retries Exhausted                   */
    {_BYTE    , 31,  4,  3},      /*002 NAK: Bad Protocol Rcv'd                  */
    {_BYTE    , 31,  5,  4},      /*003 NAK: Bad LSAP Rcv'd                      */
    {_BYTE    , 31,  6,  5},      /*004 NAK: No Memory Rcv'd                     */
    {_BYTE    , 31,  7,  6},      /*005 Rcv'd ACK/NAK Too Short                  */
    {_BYTE    , 31,  8,  7},      /*006 Rcv'd ACK/NAK Too Long                   */
    {_BYTE    , 31,  9,  8},      /*007 Non-ACK/NAK Rcv'd                        */
    {_BYTE    , 31, 10,  9},      /*008 Token Pass Timeout                       */
    {_BYTE    , 31, 11, 10},      /*009 Token Pass Aborted                       */
    {_BYTE    , 31, 12, 11},      /*010 Token Claims Tried                       */
    {_BYTE    , 31, 13, 12},      /*011 Token Claimed                            */
    {_BYTE    , 31, 14, 13},      /*012 Bad CRC in Rcv'd Frame                   */
    {_BYTE    , 31, 15, 14},      /*013 NAK: Bad Protocol Sent                   */
    {_BYTE    , 66,  1, 15},      /*014 NAK: Bad LSAP Sent                       */
    {_BYTE    , 66,  2, 16},      /*015 NAK: No Memory Sent                      */
    {_BYTE    , 66,  3, 17},      /*016 Rcv'd Frame Too Small                    */
    {_BYTE    , 66,  4, 18},      /*017 Rcv'd Frame Too Long                     */
    {_BYTE    , 66,  5, 19},      /*018 Rcv'd a Frame Twice                      */
    {_BYTE    , 66,  6, 20},      /*019 Rcv'd Frame Aborted                      */
    {_WORD    , 64,  7, 21},      /*020 Message Sent OK                          */
    {_WORD    , 64,  8, 23},      /*021 Message Rcv'd OK                         */
    {_WORD    , 64,  9, 25},      /*022 Command Sent OK                          */
    {_WORD    , 64, 10, 27},      /*023 Reply Rcv'd OK                           */
    {_WORD    , 64, 11, 29},      /*024 Command Rcv'd OK                         */
    {_WORD    , 64, 12, 31},      /*025 Reply Sent OK                            */
    {_BYTE    , 66, 13, 33},      /*026 Reply Could Not Be Sent                  */
    {_BYTE    , 66, 14, 34},      /*027 Number of Active Nodes                   */
    {_BYTE    , 66, 15,  0},      /*028 Rcv'd ACK with Bad CRC                   */
    {_END     ,  0,  0,  0},      /*029                                          */
    {_WORD    , 19,  2,  4},      /*030 Sent                      Data Format 1  */
    {_WORD    , 19,  3,  2},      /*031 Received                                 */
    {_WORD    , 19,  5, 32},      /*032 Received                                 */
    {_WORD    , 19,  6, 40},      /*033 Received SAP off                         */
    {_WORD    , 19,  7, 52},      /*034 Transmit failed                          */
    {_WORD    , 19,  8, 46},      /*035 Transmit timeout                         */
    {_WORD    , 19, 10, 64},      /*036 Received                                 */
    {_WORD    , 19, 11, 50},      /*037 SDA/SDN retrans                          */
    {_WORD    , 19, 12, 28},      /*038 Duplicate node                           */
    {_WORD    , 19, 13, 14},      /*039 Claims Won                               */
    {_WORD    , 19, 14, 20},      /*040 Token retry                              */
    {_WORD    , 19, 15, 18},      /*041 New successor                            */
    {_WORD    , 47,  2,  8},      /*042 Sent with error                          */
    {_WORD    , 47,  3,  6},      /*043 Received with error                      */
    {_WORD    , 47,  5, 38},      /*044 Received but full                        */
    {_WORD    , 47,  6, 34},      /*045 Received with error                      */
    {_WORD    , 47,  7, 42},      /*046 Transmit confirm                         */
    {_WORD    , 47,  8, 54},      /*047 Transmit NAK full                        */
    {_WORD    , 47, 10, 60},      /*048 Transmit failed                          */
    {_WORD    , 47, 12, 16},      /*049 Claims lost                              */
    {_WORD    , 47, 13, 30},      /*050 Dropped token                            */
    {_WORD    , 47, 14, 62},      /*051 Solicit Rotations                        */
    {_WORD    , 47, 15, 22},      /*052 Token failed                             */
    {_WORD    , 74,  3, 10},      /*053 Unable to receive                        */
    {_WORD    , 74,  5, 36},      /*054 Received retrans                         */
    {_WORD    , 74,  6, 44},      /*055 Transmit NAK misc                        */
    {_WORD    , 74,  7, 48},      /*056 Transmit not ACKed                       */
    {_WORD    , 74,  8, 56},      /*057 Transmit NAKed SAP                       */
    {_WORD    , 74, 10, 58},      /*058 Transmit confirm                         */
    {_WORD    , 74, 12, 12},      /*059 Network dead                             */
    {_WORD    , 74, 13, 26},      /*060 Linear scan failed                       */
    {_WORD    , 74, 14, 24},      /*061 Started lin'r scan                       */
    {_END     ,  0,  0,  0},      /*062                                          */
    {_BYTE    , 30,  2,  1},      /*063 ACK Timeout               Data Format 2  */
    {_BYTE    , 30,  3,  2},      /*064 Xmit Retries Exhausted                   */
    {_BYTE    , 30,  4,  5},      /*065 NAK: No Memory Rcv'd                     */
    {_BYTE    , 30,  5,  6},      /*066 Rcv'd ACK/NAK Too Short                  */
    {_BYTE    , 30,  6,  7},      /*067 Rcv'd ACK/NAK Too Long                   */
    {_BYTE    , 30,  7,  8},      /*068 Non-ACK/NAK Rcv'd                        */
    {_BYTE    , 30,  8,  9},      /*069 Duplicate token found                    */
    {_BYTE    , 30,  9, 10},      /*070 Duplicate node found                     */
    {_BYTE    , 30, 10, 11},      /*071 Token Pass Timeout                       */
    {_BYTE    , 30, 11, 12},      /*072 Token Pass Aborted                       */
    {_BYTE    , 30, 12, 13},      /*073 Token Claims Tried                       */
    {_BYTE    , 30, 13, 14},      /*074 Token Claimed                            */
    {_BYTE    , 30, 14, 15},      /*075 Bad CRC in Rcv'd Frame                   */
    {_BYTE    , 30, 15, 18},      /*076 NAK: No Memory Sent                      */
    {_BYTE    , 68,  1, 21},      /*077 Rcv'd a Frame Twice                      */
    {_BYTE    , 68,  2, 22},      /*078 Rcv'd Frame Aborted                      */
    {_WORD    , 66,  3, 23},      /*079 Message Sent OK                          */
    {_WORD    , 66,  4, 25},      /*080 Message Rcv'd OK                         */
    {_WORD    , 66,  5, 27},      /*081 Command Sent OK                          */
    {_WORD    , 66,  6, 29},      /*082 Reply Rcv'd OK                           */
    {_WORD    , 66,  7, 31},      /*083 Command Rcv'd OK                         */
    {_WORD    , 66,  8, 33},      /*084 Reply Sent OK                            */
    {_BYTE    , 68,  9, 35},      /*085 Reply Could Not Be Sent                  */
    {_BYTE    , 68, 10, 38},      /*086 PLC-2 resyc's                            */
    {_BYTE    , 68, 11, 39},      /*087 T3 errors before FT cleared              */
    {_BYTE    , 68, 12, 40},      /*088 edit connections request                 */
    {_BYTE    , 68, 13, 41},      /*089 edit connects granted                    */
    {_BYTE    , 68, 14, 42},      /*090 edit connects timed out                  */
    {_BYTE    , 68, 15,  0},      /*091 Rcv'd ACK with Bad CRC                   */
    {_END     ,  0,  0,  0},      /*092                               */
    {_WORD    , 32,  1,  0},      /*093 Messages Transmitted      Data Format 3  */
    {_WORD    , 32,  2,  2},      /*094 Messages Received                        */
    {_BYTE    , 34,  3,  5},      /*095 Responses Timed out                      */
    {_BYTE    , 34,  4, 12},      /*096 Aborted Packets Rcv'd                    */
    {_BYTE    , 34,  5,  9},      /*097 NAKS: lsap Recv'd                        */
    {_BYTE    , 34,  6, 13},      /*098 CRC errors                               */
    {_BYTE    , 34,  7, 14},      /*099 Illegal Packet Size                      */
    {_BYTE    , 34,  8, 16},      /*100 Duplicate Node Seen                      */
    {_BYTE    , 68,  1, 17},      /*101 Link Deadtime                            */
    {_BYTE    , 68,  2,  8},      /*102 Nak No Mem Sent                          */
    {_BYTE    , 68,  3,  6},      /*103 NAKS Rcv'd                               */
    {_BYTE    , 68,  4, 10},      /*104 Duplicate Packets                        */
    {_BYTE    , 68,  5, 11},      /*105 Token Timeouts                           */
    {_BYTE    , 68,  6,  7},      /*106 Retries                                  */
    {_BYTE    , 68,  7, 15},      /*107 Duplicate Token                          */
    {_BYTE    , 68,  8,  4},      /*108 Undeliverable                            */
    {_END     ,  0,  0,  0},      /*109                                          */
    {_REVWORD , 32,  1,  0},      /*110 Messages Transmitted      Data Format 4  */
    {_REVWORD , 32,  2,  2},      /*111 Messages Received                        */
    {_BYTE    , 34,  3,  5},      /*112 Responses Timed out                      */
    {_BYTE    , 34,  4, 12},      /*113 Aborted Packets Rcv'd                    */
    {_BYTE    , 34,  5,  9},      /*114 NAKS: lsap Recv'd                        */
    {_BYTE    , 34,  6, 13},      /*115 CRC errors                               */
    {_BYTE    , 34,  7, 14},      /*116 Illegal Packet Size                      */
    {_BYTE    , 34,  8, 16},      /*117 Duplicate Node Seen                      */
    {_BYTE    , 68,  1, 17},      /*118 Link Deadtime                            */
    {_BYTE    , 68,  2,  8},      /*119 Nak No Mem Sent                          */
    {_BYTE    , 68,  3,  6},      /*120 NAKS Rcv'd                               */
    {_BYTE    , 68,  4, 10},      /*121 Duplicate Packets                        */
    {_BYTE    , 68,  5, 11},      /*122 Token Timeouts                           */
    {_BYTE    , 68,  6,  7},      /*123 Retries                                  */
    {_BYTE    , 68,  7, 15},      /*124 Duplicate Token                          */
    {_BYTE    , 68,  8,  4},      /*125 Undeliverable                            */
    {_END     ,  0,  0,  0},      /*126                                          */
    {_REVWORD , 26,  3,  8},      /*127 Messages Transmitted      Data Format 5  */
    {_REVWORD , 26,  4, 10},      /*128 Messages Received                        */
    {_REVWORD , 26,  5, 12},      /*129 Commands Generated                       */
    {_REVWORD , 26,  6, 14},      /*130 Requests Executed                        */
    {_REVWORD , 26,  7, 16},      /*131 Reply Sent                               */
    {_BYTE    , 72,  1,  0},      /*132 Transmit not ACKed                       */
    {_BYTE    , 72,  2,  1},      /*133 Transmit NAKed                           */
    {_BYTE    , 72,  3,  2},      /*134 Network Dead                             */
    {_BYTE    , 72,  4,  3},      /*135 Received but full                        */
    {_BYTE    , 72,  5,  4},      /*136 Received with error                      */
    {_BYTE    , 72,  6,  5},      /*137 Received retrans                         */
    {_BYTE    , 72,  7,  6},      /*138 Token failed                             */
    {_BYTE    , 72,  8,  7},      /*139 Transmit retries                         */
    {_BYTE    , 72,  9, 30},      /*140 Adapter Timeouts                         */
    {_BYTE    , 72, 10, 31},      /*141 Transmit timeout                         */
    {_END     ,  0,  0,  0},      /*142                                          */
    {_REVWORD , 26,  3,  8},      /*143 Messages Transmitted      Data Format 6  */
    {_REVWORD , 26,  4, 10},      /*144 Messages Received                        */
    {_REVWORD , 26,  5, 12},      /*145 Commands Generated                       */
    {_REVWORD , 26,  6, 14},      /*146 Requests Executed                        */
    {_REVWORD , 26,  7, 16},      /*147 Reply Sent                               */
    {_BYTE    , 72,  1,  0},      /*148 Transmit not ACKed                       */
    {_BYTE    , 72,  2,  1},      /*149 Transmit NAKed                           */
    {_BYTE    , 72,  3,  2},      /*150 Network Dead                             */
    {_BYTE    , 72,  4,  3},      /*151 Received but full                        */
    {_BYTE    , 72,  5,  4},      /*152 Received with error                      */
    {_BYTE    , 72,  6,  5},      /*153 Received retrans                         */
    {_BYTE    , 72,  7,  6},      /*154 Token failed                             */
    {_BYTE    , 72,  8,  7},      /*155 Transmit retries                         */
    {_BYTE    , 72,  9, 30},      /*156 Adapter Timeouts                         */
    {_BYTE    , 72, 10, 31},      /*157 Transmit timeout                         */
    {_BYTE    , 22, 13, 18},      /*158 Timeouts       Rack 1                    */
    {_BYTE    , 22, 14, 21},      /*159 CRC                                      */
    {_BYTE    , 22, 15, 24},      /*160 Block Transfers                          */
    {_BYTE    , 22, 16, 27},      /*161 Retries                                  */
    {_BYTE    , 26, 13, 19},      /*162 Timeouts       Rack 2                    */
    {_BYTE    , 26, 14, 22},      /*163 CRC                                      */
    {_BYTE    , 26, 15, 25},      /*164 Block Transfers                          */
    {_BYTE    , 26, 16, 28},      /*165 Retries                                  */
    {_BYTE    , 30, 13, 20},      /*166 Timeouts       Rack 3                    */
    {_BYTE    , 30, 14, 23},      /*167 CRC                                      */
    {_BYTE    , 30, 15, 26},      /*168 Block Transfers                          */
    {_BYTE    , 30, 16, 29},      /*169 Retries                                  */
    {_END     ,  0,  0,  0},      /*170                                          */
    {_REVWORD , 26,  3,  8},      /*171 Messages Transmitted DHP  Data Format 7  */
    {_REVWORD , 26,  4, 10},      /*172 Messages Received                        */
    {_REVWORD , 26,  5, 12},      /*173 Commands Generated                       */
    {_REVWORD , 26,  6, 14},      /*174 Requests Executed                        */
    {_REVWORD , 26,  7, 16},      /*175 Reply Sent                               */
    {_REVWORD , 35,  3, 18},      /*176 Messages Transmitted VME                 */
    {_REVWORD , 35,  4, 20},      /*177 Messages Received                        */
    {_REVWORD , 35,  5, 22},      /*178 Commands Generated                       */
    {_REVWORD , 35,  6, 24},      /*179 Requests Executed                        */
    {_REVWORD , 35,  7, 26},      /*180 Reply Sent                               */
    {_BYTE    , 72,  1,  0},      /*181 Transmit not ACKed                       */
    {_BYTE    , 72,  2,  1},      /*182 Transmit NAKed                           */
    {_BYTE    , 72,  3,  2},      /*183 Network Dead                             */
    {_BYTE    , 72,  4,  3},      /*184 Received but full                        */
    {_BYTE    , 72,  5,  4},      /*185 Received with error                      */
    {_BYTE    , 72,  6,  5},      /*186 Received retrans                         */
    {_BYTE    , 72,  7,  6},      /*187 Token failed                             */
    {_BYTE    , 72,  8,  7},      /*188 Transmit retries                         */
    {_BYTE    , 72,  9, 44},      /*189 Adapter Timeouts                         */
    {_BYTE    , 72, 10, 45},      /*190 Transmit timeout                         */
    {_BYTE    , 18, 13, 28},      /*191 Timeouts       Rack 0                    */
    {_BYTE    , 18, 14, 32},      /*192 CRC                                      */
    {_BYTE    , 18, 15, 36},      /*193 Block Transfers                          */
    {_BYTE    , 18, 16, 40},      /*194 Retries                                  */
    {_BYTE    , 22, 13, 29},      /*195 Timeouts       Rack 1                    */
    {_BYTE    , 22, 14, 33},      /*196 CRC                                      */
    {_BYTE    , 22, 15, 37},      /*197 Block Transfers                          */
    {_BYTE    , 22, 16, 41},      /*198 Retries                                  */
    {_BYTE    , 26, 13, 30},      /*199 Timeouts       Rack 2                    */
    {_BYTE    , 26, 14, 34},      /*200 CRC                                      */
    {_BYTE    , 26, 15, 38},      /*201 Block Transfers                          */
    {_BYTE    , 26, 16, 42},      /*202 Retries                                  */
    {_BYTE    , 30, 13, 31},      /*203 Timeouts       Rack 3                    */
    {_BYTE    , 30, 14, 35},      /*204 CRC                                      */
    {_BYTE    , 30, 15, 39},      /*205 Block Transfers                          */
    {_BYTE    , 30, 16, 43},      /*206 Retries                                  */
    {_END     ,  0,  0,  0},      /*207                                          */
    {_REVWORD , 26,  3,  8},      /*208 Messages Transmitted      Data Format 8  */
    {_REVWORD , 26,  4, 10},      /*209 Messages Received                        */
    {_REVWORD , 26,  5, 12},      /*210 Commands Generated                       */
    {_REVWORD , 26,  6, 14},      /*211 Requests Executed                        */
    {_REVWORD , 26,  7, 16},      /*212 Reply Sent                               */
    {_BYTE    , 72,  1,  0},      /*213 Transmit not ACKed                       */
    {_BYTE    , 72,  2,  1},      /*214 Transmit NAKed                           */
    {_BYTE    , 72,  3,  2},      /*215 Network Dead                             */
    {_BYTE    , 72,  4,  3},      /*216 Received but full                        */
    {_BYTE    , 72,  5,  4},      /*217 Received with error                      */
    {_BYTE    , 72,  6,  5},      /*218 Received retrans                         */
    {_BYTE    , 72,  7,  6},      /*219 Token failed                             */
    {_BYTE    , 72,  8,  7},      /*220 Transmit retries                         */
    {_BYTE    , 72,  9, 46},      /*221 Adapter Timeouts                         */
    {_BYTE    , 72, 10, 47},      /*222 Transmit timeout                         */
    {_BYTE    , 22, 13, 18},      /*223 Timeouts       Rack 1                    */
    {_BYTE    , 22, 14, 25},      /*224 CRC                                      */
    {_BYTE    , 22, 15, 32},      /*225 Block Transfers                          */
    {_BYTE    , 22, 16, 39},      /*226 Retries                                  */
    {_BYTE    , 26, 13, 19},      /*227 Timeouts       Rack 2                    */
    {_BYTE    , 26, 14, 26},      /*228 CRC                                      */
    {_BYTE    , 26, 15, 33},      /*229 Block Transfers                          */
    {_BYTE    , 26, 16, 40},      /*230 Retries                                  */
    {_BYTE    , 30, 13, 20},      /*231 Timeouts       Rack 3                    */
    {_BYTE    , 30, 14, 27},      /*232 CRC                                      */
    {_BYTE    , 30, 15, 34},      /*233 Block Transfers                          */
    {_BYTE    , 30, 16, 41},      /*234 Retries                                  */
    {_BYTE    , 34, 13, 21},      /*235 Timeouts       Rack 4                    */
    {_BYTE    , 34, 14, 28},      /*236 CRC                                      */
    {_BYTE    , 34, 15, 35},      /*237 Block Transfers                          */
    {_BYTE    , 34, 16, 42},      /*238 Retries                                  */
    {_BYTE    , 38, 13, 22},      /*239 Timeouts       Rack 5                    */
    {_BYTE    , 38, 14, 29},      /*240 CRC                                      */
    {_BYTE    , 38, 15, 36},      /*241 Block Transfers                          */
    {_BYTE    , 38, 16, 43},      /*242 Retries                                  */
    {_BYTE    , 42, 13, 23},      /*243 Timeouts       Rack 6                    */
    {_BYTE    , 42, 14, 30},      /*244 CRC                                      */
    {_BYTE    , 42, 15, 37},      /*245 Block Transfers                          */
    {_BYTE    , 42, 16, 44},      /*246 Retries                                  */
    {_BYTE    , 46, 13, 24},      /*247 Timeouts       Rack 7                    */
    {_BYTE    , 46, 14, 31},      /*248 CRC                                      */
    {_BYTE    , 46, 15, 38},      /*249 Block Transfers                          */
    {_BYTE    , 46, 16, 45},      /*250 Retries                                  */
    {_END     ,  0,  0,  0},      /*251                                          */
    {_BYTE    , 31,  2,  1},      /*252 ACK Timeout               Data Format 0  */
    {_BYTE    , 31,  3,  2},      /*253 Xmit Retries Exhausted                   */
    {_BYTE    , 31,  4,  3},      /*254 NAK: Bad Protocol Rcv'd                  */
    {_BYTE    , 31,  5,  4},      /*255 NAK: Bad LSAP Rcv'd                      */
    {_BYTE    , 31,  6,  5},      /*256 NAK: No Memory Rcv'd                     */
    {_BYTE    , 31,  7,  6},      /*257 Rcv'd ACK/NAK Too Short                  */
    {_BYTE    , 31,  8,  7},      /*258 Rcv'd ACK/NAK Too Long                   */
    {_BYTE    , 31,  9,  8},      /*259 Non-ACK/NAK Rcv'd                        */
    {_BYTE    , 31, 10, 11},      /*260 Token Pass Timeout                       */
    {_BYTE    , 31, 11, 12},      /*261 Token Pass Aborted                       */
    {_BYTE    , 31, 12, 13},      /*262 Token Claims Tried                       */
    {_BYTE    , 31, 13, 14},      /*263 Token Claimed                            */
    {_BYTE    , 31, 14, 15},      /*264 Bad CRC in Rcv'd Frame                   */
    {_BYTE    , 31, 15, 16},      /*265 NAK: Bad Protocol Sent                   */
    {_BYTE    , 66,  1, 17},      /*266 NAK: Bad LSAP Sent                       */
    {_BYTE    , 66,  2, 18},      /*267 NAK: No Memory Sent                      */
    {_BYTE    , 66,  3, 19},      /*268 Rcv'd Frame Too Small                    */
    {_BYTE    , 66,  4, 20},      /*269 Rcv'd Frame Too Long                     */
    {_BYTE    , 66,  5, 21},      /*270 Rcv'd a Frame Twice                      */
    {_BYTE    , 66,  6, 22},      /*271 Rcv'd Frame Aborted                      */
    {_WORD    , 64,  7, 23},      /*272 Message Sent OK                          */
    {_WORD    , 64,  8, 25},      /*273 Message Rcv'd OK                         */
    {_WORD    , 64,  9, 27},      /*274 Command Sent OK                          */
    {_WORD    , 64, 10, 29},      /*275 Reply Rcv'd OK                           */
    {_WORD    , 64, 11, 31},      /*276 Command Rcv'd OK                         */
    {_WORD    , 64, 12, 33},      /*277 Reply Sent OK                            */
    {_BYTE    , 66, 13, 35},      /*278 Reply Could Not Be Sent                  */
    {_BYTE    , 66, 14, 36},      /*279 Number of Active Nodes                   */
    {_BYTE    , 66, 15,  0},      /*280 Rcv'd ACK with Bad CRC                   */
    {_END     ,  0,  0,  0}      /*281                                          */
};


TYPE_INFO_STUC ID_INFO[]=
{
/*  name      ctrs txt dta name */
    {"PLCX  " ,  0 ,_n_,_n_,_n_},       /* 0x           */
    {"PLC2  " , 69 , 2 , 63,_n_},       /* 1x           */
    {"2/20  " , 69 , 2 , 63,_n_},       /* 2x           */
    {"MINI  " , 69 , 2 , 63,_n_},       /* 3x           */
    {"PLC3  " , 18 , 3 , 93, 2 },       /* 4x           */
    {"2/20  " , 69 , 2 , 63,_n_},       /* 5x           */
    {"2/15  " , 69 , 2 , 63,_n_},       /* 6x           */
    {"2/30  " , 69 , 2 , 63,_n_},       /* 7x           */
    {"PLC4  " ,  0 ,_n_,_n_,_n_},       /* 8x           */
    {""       ,  0 ,_n_,_n_,_n_},       /* 9x           */
    {"GATE  " ,  0 ,_n_,_n_,_n_},       /* Ax           */
    {"5/15  " ,  0 , 4 ,143, 0 },       /* Bx           */
    {"5VME  " ,  0 , 4 ,171, 0 },       /* Cx           */
    {"PI_RM " ,  0 , 3 ,110, 1 },       /* Dx           */
};

TYPE_INFO_STUC EXT_ID_INFO[]=
{
    {""       ,  0 ,_n_,_n_,_n_},       /* Ex 00        */
    {""       ,  0 ,_n_,_n_,_n_},       /* Ex 01        */
    {""       ,  0 ,_n_,_n_,_n_},       /* Ex 02        */
    {""       ,  0 ,_n_,_n_,_n_},       /* Ex 03        */
    {""       ,  0 ,_n_,_n_,_n_},       /* Ex 04        */
    {""       ,  0 ,_n_,_n_,_n_},       /* Ex 05        */
    {""       ,  0 ,_n_,_n_,_n_},       /* Ex 06        */
    {""       ,  0 ,_n_,_n_,_n_},       /* Ex 07        */
    {""       ,  0 ,_n_,_n_,_n_},       /* Ex 08        */
    {""       ,  0 ,_n_,_n_,_n_},       /* Ex 09        */
    {""       ,  0 ,_n_,_n_,_n_},       /* Ex 0A        */
    {""       ,  0 ,_n_,_n_,_n_},       /* Ex 0B        */
    {""       ,  0 ,_n_,_n_,_n_},       /* Ex 0C        */
    {""       ,  0 ,_n_,_n_,_n_},       /* Ex 0D        */
    {""       ,  0 ,_n_,_n_,_n_},       /* Ex 0E        */
    {""       ,  0 ,_n_,_n_,_n_},       /* Ex 0F        */
    {"85KA  " , 69 , 0 ,252,_n_},       /* Ex 10        */
    {"9NB1  " ,  0 ,_n_,_n_,_n_},       /* Ex 11        */
    {"9NB2  " ,  0 ,_n_,_n_,_n_},       /* Ex 12        */
    {"5/12  " ,  0 , 4 ,143, 0 },       /* Ex 13        */
    {"5/25  " ,  0 , 4 ,208, 0 },       /* Ex 14        */
    {"5/40  " ,  0 , 1 , 30, 0 },       /* Ex 15        */
    {"KP5   " , 69 , 0 , 0 ,_n_},       /* Ex 16        */
    {""       ,  0 ,_n_,_n_,_n_},       /* Ex 17        */
    {"L511  " ,  0 ,_n_,_n_,_n_},       /* Ex 18        */
    {"L12   " ,  0 ,_n_,_n_,_n_},       /* Ex 19        */
    {"L20   " ,  0 ,_n_,_n_,_n_},       /* Ex 1A        */
    {"PA1   " ,  0 ,_n_,_n_,_n_},       /* Ex 1B        */
    {"PTA1  " ,  0 ,_n_,_n_,_n_},       /* Ex 1C        */
    {"2/02  " , 69 , 2 , 63,_n_},       /* Ex 1D        */
    {"2/05  " , 69 , 2 , 63,_n_},       /* Ex 1E        */
    {"2/16  " , 69 , 2 , 63,_n_},       /* Ex 1F        */
    {"2/17  " , 69 , 2 , 63,_n_},       /* Ex 20        */
    {"DM6   " ,  0 ,_n_,_n_,_n_},       /* Ex 21        */
    {"5/10  " ,  0 , 4 ,127, 0 },       /* Ex 22        */
    {"5/60  " ,  0 , 1 , 30, 0 },       /* Ex 23        */
    {"PI_VAX" ,  0 ,_n_,_n_,_n_},       /* Ex 24        */
    {"5/02  " ,  0 ,_n_,_n_,_n_},       /* Ex 25        */
    {"DTAM  " ,  0 ,_n_,_n_,_n_},       /* Ex 26        */
    {""       ,  0 ,_n_,_n_,_n_},       /* Ex 27        */
    {"5/40L " ,  0 , 1 , 30, 0 },       /* Ex 28        */
    {"5/60L " ,  0 , 1 , 30, 0 },       /* Ex 29        */
    {""       ,  0 ,_n_,_n_,_n_},       /* Ex 2A        */
    {""       ,  0 ,_n_,_n_,_n_},       /* Ex 2B        */
    {""       ,  0 ,_n_,_n_,_n_},       /* Ex 2C        */
    {""       ,  0 ,_n_,_n_,_n_},       /* Ex 2D        */
    {""       ,  0 ,_n_,_n_,_n_},       /* Ex 2E        */
    {""       ,  0 ,_n_,_n_,_n_},       /* Ex 2F        */
    {"KA5   " , 69 , 0 , 0 ,_n_},       /* Ex 30        */
    {"5/11  " ,  0 , 1 , 30, 0 },       /* Ex 31        */
    {"5/20  " ,  0 , 1 , 30, 0 },       /* Ex 32        */
    {"5/30  " ,  0 , 1 , 30, 0 },       /* Ex 33        */
    {""       ,  0 ,_n_,_n_,_n_},       /* Ex 34        */
    {"5/V30 " ,  0 , 1 , 30, 0 },       /* Ex 35        */
    {""       ,  0 ,_n_,_n_,_n_},       /* Ex 36        */
    {"5/V40 " ,  0 , 1 , 30, 0 },       /* Ex 37        */
    {"5/V40L" ,  0 , 1 , 30, 0 },       /* Ex 38        */
    {"5/V60 " ,  0 , 1 , 30, 0 },       /* Ex 39        */
    {"5/V60L" ,  0 , 1 , 30, 0 },       /* Ex 3A        */
    {"TOME  " ,  0 ,_n_,_n_,_n_},       /* Ex 3B        */
    {""       ,  0 ,_n_,_n_,_n_},       /* Ex 3C        */
    {""       ,  0 ,_n_,_n_,_n_},       /* Ex 3D        */
    {""       ,  0 ,_n_,_n_,_n_},       /* Ex 3E        */
    {""       ,  0 ,_n_,_n_,_n_},       /* Ex 3F        */
    {""       ,  0 ,_n_,_n_,_n_},       /* Ex 40        */
    {""       ,  0 ,_n_,_n_,_n_},       /* Ex 41        */
    {"5/16  " ,  0 , 1 , 30, 0 },       /* Ex 42        */
    {"5/26  " ,  0 , 1 , 30, 0 },       /* Ex 43        */
    {"5/36  " ,  0 , 1 , 30, 0 },       /* Ex 44        */
    {"5/46  " ,  0 , 1 , 30, 0 },       /* Ex 45        */
    {"5/46L " ,  0 , 1 , 30, 0 },       /* Ex 46        */
    {"5/66  " ,  0 , 1 , 30, 0 },       /* Ex 47        */
    {"5/66L " ,  0 , 1 , 30, 0 },       /* Ex 48        */
    {""       ,  0 ,_n_,_n_,_n_},       /* Ex 49        */
    {"5/20E " ,  0 , 1 , 30, 0 },       /* Ex 4A        */
    {"5/40E " ,  0 , 1 , 30, 0 },       /* Ex 4B        */
    {"5/26E " ,  0 , 1 , 30, 0 },       /* Ex 4C        */
    {"5/46E " ,  0 , 1 , 30, 0 },       /* Ex 4D        */
    {""       ,  0 ,_n_,_n_,_n_},       /* Ex 4E        */
    {""       ,  0 ,_n_,_n_,_n_},       /* Ex 4F        */
    {""       ,  0 ,_n_,_n_,_n_},       /* Ex 50        */
    {""       ,  0 ,_n_,_n_,_n_},       /* Ex 51        */
    {""       ,  0 ,_n_,_n_,_n_},       /* Ex 52        */
    {""       ,  0 ,_n_,_n_,_n_},       /* Ex 53        */
    {""       ,  0 ,_n_,_n_,_n_},       /* Ex 54        */
    {"5/80  " ,  0 , 1 , 30, 0 },       /* Ex 55        */
    {"5/86  " ,  0 , 1 , 30, 0 },       /* Ex 56        */
};


TYPE_INFO_STUC MISC_ID_INFO[]=
{
    {"KF2   " , 69 , 0 , 0 ,_n_},       /* F3           */
    {"TERM  " , 69 , 0 , 0 , 4 },       /* FE 1B        */
    {"COMP  " , 69 , 0 , 0 ,_n_},       /* FF           */
    {"SLC504" , 0  , 0 , 0 , 3 },       /* EE 31 5b     */
};

TYPE_INFO_STUC UNKNOWN_ID =
{
    "??????" ,  0 ,_n_,_n_,_n_,
};

/* Text Format 0 */
char CTR_TEXT[4][1215]=
{
    {"                                      NAK: Bad LSAP Sent........[   ]\n"
    "ACK Timeout..................[   ]    NAK: No Memory Sent.......[   ]\n"
    "Xmit Retries Exhausted.......[   ]    Rcv'd Frame Too Small.....[   ]\n"
    "NAK: Bad Protocol Rcv'd......[   ]    Rcv'd Frame Too Long......[   ]\n"
    "NAK: Bad LSAP Rcv'd..........[   ]    Rcv'd a Frame Twice.......[   ]\n"
    "NAK: No Memory Rcv'd.........[   ]    Rcv'd Frame Aborted.......[   ]\n"
    "Rcv'd ACK/NAK Too Short......[   ]    Message Sent OK.........[     ]\n"
    "Rcv'd ACK/NAK Too Long.......[   ]    Message Rcv'd OK........[     ]\n"
    "Non-ACK/NAK Rcv'd............[   ]    Command Sent OK.........[     ]\n"
    "Token Pass Timeout...........[   ]    Reply Rcv'd OK..........[     ]\n"
    "Token Pass Aborted...........[   ]    Command Rcv'd OK........[     ]\n"
    "Token Claims Tried...........[   ]    Reply Sent OK...........[     ]\n"
    "Token Claimed................[   ]    Reply Could Not Be Sent...[   ]\n"
    "Bad CRC in Rcv'd Frame.......[   ]    Number of Active Nodes....[   ]\n"
    "NAK: Bad Protocol Sent.......[   ]    Rcv'd ACK with Bad CRC....[   ]"},

/* Text Format 1 */
    {"MESSAGES\n"
    "Sent.............[     ] Sent with error.....[     ]\n"
    "Received.........[     ] Received with error.[     ] Unable to receive..[     ]\n"
    "SEND DATA ACKNOWLEDGED\n"
    "Received.........[     ] Received but full...[     ] Received retrans...[     ]\n"
    "Received SAP off.[     ] Received with error.[     ] Transmit NAK misc..[     ]\n"
    "Transmit failed..[     ] Transmit confirm....[     ] Transmit not ACKed.[     ]\n"
    "Transmit timeout.[     ] Transmit NAK full...[     ] Transmit NAKed SAP.[     ]\n"
    "SEND DATA NO ACKNOWLEDGE\n"
    "Received.........[     ] Transmit failed.....[     ] Transmit confirm...[     ]\n"
    "SDA/SDN retrans..[     ]                                                       \n"
    "Duplicate node...[     ] Claims lost.........[     ] Network dead.......[     ]\n"
    "Claims Won.......[     ] Dropped token.......[     ] Linear scan failed.[     ]\n"
    "Token retry......[     ] Solicit Rotations...[     ] Started lin'r scan.[     ]\n"
    "New successor....[     ] Token failed........[     ]"},

/* Text Format 2 */
    {"                                    Rcv'd a Frame Twice...........[   ]\n"
    "ACK Timeout.................[   ]   Rcv'd Frame Aborted...........[   ]\n"
    "Xmit Retries Exhausted......[   ]   Message Sent OK.............[     ]\n"
    "NAK: No Memory Rcv'd........[   ]   Message Rcv'd OK............[     ]\n"
    "Rcv'd ACK/NAK Too Short.....[   ]   Command Sent OK.............[     ]\n"
    "Rcv'd ACK/NAK Too Long......[   ]   Reply Rcv'd OK..............[     ]\n"
    "Non-ACK/NAK Rcv'd...........[   ]   Command Rcv'd OK............[     ]\n"
    "Duplicate token found.......[   ]   Reply Sent OK...............[     ]\n"
    "Duplicate node found........[   ]   Reply Could Not Be Sent.......[   ]\n"
    "Token Pass Timeout..........[   ]   PLC-2 resyc's.................[   ]\n"
    "Token Pass Aborted..........[   ]   T3 errors before FT cleared...[   ]\n"
    "Token Claims Tried..........[   ]   edit connections request......[   ]\n"
    "Token Claimed...............[   ]   edit connects granted.........[   ]\n"
    "Bad CRC in Rcv'd Frame......[   ]   edit connects timed out.......[   ]\n"
    "NAK: No Memory Sent.........[   ]   Rcv'd ACK with Bad CRC........[   ]"},

/* Text Format 3 */
    {"Messages Transmitted..........[     ]          Link Deadtime......[   ]\n"
    "Messages Received.............[     ]          Nak No Mem Sent....[   ]\n"
    "Responses Timed out.............[   ]          NAKS Rcv'd.........[   ]\n"
    "Aborted Packets Rcv'd...........[   ]          Duplicate Packets..[   ]\n"
    "NAKS: lsap Recv'd...............[   ]          Token Timeouts.....[   ]\n"
    "CRC errors......................[   ]          Retries............[   ]\n"
    "Illegal Packet Size.............[   ]          Duplicate Token....[   ]\n"
    "Duplicate Node Seen.............[   ]          Undeliverable......[   ]"},

/* Text Format 4 */
    {"                                              Transmit not ACKed......[  0]\n"
    "                          DH+      VME        Transmit NAKed..........[  1]\n"
    "Messages Transmitted....[  8,9]..[     ]      Network Dead............[  2]\n"
    "Messages Received.......[10,11]..[     ]      Received but full.......[  3]\n"
    "Commands Generated......[12,13]..[     ]      Received with error.....[  4]\n"
    "Requests Executed.......[14,15]..[     ]      Received retrans........[  5]\n"
    "Reply Sent..............[16,17]..[     ]      Token failed............[  6]\n"
    "                                              Transmit retries........[  7]\n"
    "                                              Adapter Timeouts........[ 30]\n"
    "                                              Transmit timeout........[ 31]\n"
    "\n"
    "RACK ERRORS       0   1   2   3   4   5   6    7\n"
    "Timeouts........[   |   |   |   |   |   |   |   ]\n"
    "CRC.............[   |   |   |   |   |   |   |   ]\n"
    "Block Transfers.[   |   |   |   |   |   |   |   ]\n"
    "Retries.........[   |   |   |   |   |   |   |   ]"}
};


uchr GET_NODE_NAME_CMD[4][22]=
{
/* #0 (all PLC 5's except 5/250) */
    {0,0,19,0,0, /* dum,rem,len, dum_dst, dum_src            */
 /* cmd   sts   tns   ...   fnc   pckt_offset total_trans   */
    0x0f, 0x00, 0x00, 0x00, 0x68, 0x00, 0x00, 0x08, 0x00,
 /* flag  lev0  lev1  lev2  ...   ...   num_elemets         */
    0x07, 0x01, 0x00, 0xff, 0x00, 0x00, 0x08, 0x00},

/* #1 (PLC 5/250) */
    {0,0,15,0,0, /* dum,rem,len, dum_dst, dum_src            */
 /* cmd   sts   tns   ...   fnc   pckt_offset total_trans   */
    0x0f, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x04, 0x00,
 /* flag  lev1  lev5  num_elemets                           */
    0x22, 0x03, 0x06, 0x08,
    0,0,0,0},    /* filler */

/* #2 (PLC 3) */
    {0,0,12,0,0, /* dum,rem,len, dum_dst, dum_src                */
 /* cmd   sts   tns   ...   fnc         flag  lev1  lev2  lev3  */
    0x0f, 0x00, 0x00, 0x00, 0x2f, 0x24, 0x0e, 0x00, 0x04, 0x00,
    0,0,0,0,0,0,0},   /* filler */

/* #3 (SLC 5/04) */
    {0,0,11,0,0, /* dum,rem,len, dum_dst, dum_src                */
 /* cmd   sts   tns   ...   fnc   size  file  type  elem        */
    0x0f, 0x00, 0x00, 0x00, 0xa1, 0x08, 0x00, 0x00, 0x00,
    0,0,0,0,0,0,0,0}   /* filler */
};


/*
// builds a diag status message (cmd 6 funct 3), sends it out,
// and uses the mod_id and the ext_mod_id to create a pointer
// to a structure which gives the station type, diag ctr fmt etc.
// this funtion fills in the global variables dg_sts_rply,
// dg_type, and dg_dst which are used by subsequent diag.c
// function calls.
*/
uint get_sta_type (uchr dst, TYPE_INFO_STUC **t)
{
    uint err;
    MU   cmd;
extern uchr loc_sta;

    *t = NULL;
    dg_type = NULL;
    dg_dst  = dst;

    cmd.rem = 0;
    cmd.len = 7;
    cmd.dst = dst;
    cmd.src = loc_sta;
    cmd.cmd = 0x06;
    cmd.sts = 0;
    cmd.var[0] = 0x03;
    err = dh_msg(&cmd,&dg_sts_rply,NULL,2000);
    if (err)
    {
        return err;
    }

    if (dg_sts_rply.mod_id < 0xe0)
    {
        dg_type = &ID_INFO[dg_sts_rply.mod_id >> 4];
    }
    else if (dg_sts_rply.mod_id < 0xf0)
    {
        if ( (dg_sts_rply.ext_mod_id1 == 0x31) && 
			(dg_sts_rply.ext_mod_id2 == 0x5b) )
        {
            dg_type = &MISC_ID_INFO[3];
        }
        else if (dg_sts_rply.ext_mod_id1 < (sizeof(EXT_ID_INFO)/sizeof(TYPE_INFO_STUC)) )
        {
            dg_type = &EXT_ID_INFO[dg_sts_rply.ext_mod_id1];
        }
    }
    else if (dg_sts_rply.mod_id == 0xf3)
    {
        dg_type = &MISC_ID_INFO[0];
    }
    else if ((dg_sts_rply.mod_id == 0xfe) && (dg_sts_rply.ext_mod_id1 == 0x1b))
    {
        dg_type = &MISC_ID_INFO[1];
    }
    else if (dg_sts_rply.mod_id == 0xff)
    {
        dg_type = &MISC_ID_INFO[2];
    }

    if ( (dg_type == NULL) || (dg_type->name[0] == 0) )
    {
        dg_type = &MISC_ID_INFO[3];
        if ( ((dg_sts_rply.mod_id>>4) == 0xe) || ((dg_sts_rply.mod_id & 0xf) == 0xe) )
        {
            sprintf (UNKNOWN_ID.name,"?%02hx?%02hx",dg_sts_rply.mod_id,dg_sts_rply.ext_mod_id1);
        }
        else
        {
            sprintf (UNKNOWN_ID.name,"?%02hx   ",dg_sts_rply.mod_id);
        }
        dg_type = &UNKNOWN_ID;
    }
    *t = dg_type;
    return 0;
}


/*
// uses the station type pointer dg_type to look up the procedure to retrieve
// the station name from station dg_dst, builds the apropriate message, and
// get the station name out of the reply.
//
// NOTE: get_sta_type must be called before this funtion as get_sta_type fills
//       in dg_sts_rply, dg_type, and dg_dst which are used by this function.
*/
uint get_sta_name (char * name)
{
    uint err;
    uchr rply[80];
    uchr rply_len;
    char *pChar;
extern uchr loc_sta;

    switch (dg_type->nme_prc)
    {
    case _n_:
        strcpy (name,"        ");
        return 0;

    case 0:
        GET_NODE_NAME_CMD[dg_type->nme_prc][3] = dg_dst;
        GET_NODE_NAME_CMD[dg_type->nme_prc][4] = loc_sta;
        err = dh_msg(&GET_NODE_NAME_CMD[dg_type->nme_prc],
                     &rply,&rply_len,2000);
        if (err != 0)
        {
            strcpy (name," ****   ");
            return err;
        }
        strncpy (name,&rply[2],rply_len-2);
        name[rply_len-2]='\0';
        break;

    case 1:
        GET_NODE_NAME_CMD[dg_type->nme_prc][3] = dg_dst;
        GET_NODE_NAME_CMD[dg_type->nme_prc][4] = loc_sta;
        err = dh_msg(&GET_NODE_NAME_CMD[dg_type->nme_prc],
                     &rply,&rply_len,2000);
        if (err != 0)
        {
            strcpy (name," ****   ");
            return err;
        }
        swab(&rply[2],name,rply_len-2);
        name[rply_len-2]='\0';
        break;

    case 2:
        GET_NODE_NAME_CMD[dg_type->nme_prc][3] = dg_dst;
        GET_NODE_NAME_CMD[dg_type->nme_prc][4] = loc_sta;
        err = dh_msg(&GET_NODE_NAME_CMD[dg_type->nme_prc],
                     &rply,&rply_len,2000);
        if ( (err != 0) && (err != 0xf001))
        {
            strcpy (name," ****   ");
            return err;
        }
        strncpy (name,&rply[1],rply_len-1);
        name[rply_len-1]='\0';
        pChar = name;
        while (*pChar)
        {
            if ( (*pChar < ' ') || (*pChar > 'z') )
            {
                *pChar = ' ';
            }
            pChar++;
        }
        break;

    case 3:
        GET_NODE_NAME_CMD[dg_type->nme_prc][3] = dg_dst;
        GET_NODE_NAME_CMD[dg_type->nme_prc][4] = loc_sta;
        err = dh_msg(&GET_NODE_NAME_CMD[dg_type->nme_prc],
                     &rply,&rply_len,2000);
        if (err != 0)
        {
            strcpy (name," ****   ");
            return err;
        }
        strncpy (name,rply,rply_len);
        name[rply_len]='\0';
        break;

    case 4:
        strncpy (name,dg_sts_rply.name,8);
        break;

    default:
        strcpy (name," ????   ");
        return 1;
    }
    name[8] = '\0';
    while (strlen(name)<8)
    {
        strcat (name," ");
    }
    return 0;
}

/*
// uses the station type pointer dg_type to look up the info required
// to build an appropriate diagnostic read command (cmd 6, funct 1),
// sends the command, and processes the reply.
//
// NOTE: get_sta_type must be called before this funtion as get_sta_type fills
//       in dg_sts_rply, dg_type, and dg_dst which are used by this function.
*/
uint get_diag_counters (uchr *dg_ctrs)
{
    uint err;
    MU   cmd;
extern uchr loc_sta;

    cmd.rem = 0;
    cmd.dst = dg_dst;
    cmd.src = loc_sta;
    cmd.cmd = 0x06;
    cmd.sts = 0;
    cmd.var[0] = 0x01;

    if (dg_type->num_ctrs)
    {
        cmd.var[1] = dg_sts_rply.ctr_addr & 0xff;
        cmd.var[2] = dg_sts_rply.ctr_addr >> 8;
        cmd.var[3] = dg_type->num_ctrs;
        cmd.len = 10;
    }
    else
    {
        cmd.len = 7;
    }

    err = dh_msg(&cmd,dg_ctrs,NULL,2000);
    return err;
}

/*
// uses the station type pointer dg_type to look up the info required
// to build an appropriate diagnostic counter reset command (cmd 6, funct 7),
// and sends the command.
//
// NOTE: get_sta_type must be called before this funtion as get_sta_type fills
//       in dg_sts_rply, dg_type, and dg_dst which are used by this function.
*/
uint reset_diag_counters (void)
{
    uint err;
    MU   cmd;
    uchr rply[244];
extern uchr loc_sta;

    cmd.rem = 0;
    cmd.len = 7;
    cmd.dst = dg_dst;
    cmd.src = loc_sta;
    cmd.cmd = 0x06;
    cmd.sts = 0;
    cmd.var[0] = 0x07;

    err = dh_msg(&cmd,rply,NULL,2000);
    return err;
}

/*
// looks up a counter format in the DIAG_CTR_INFO array, and returns formatted
// string representing the data in the given diagnostic counter.
*/
void get_ctr_str (uint ctr, char *ctr_str, uchr * data)
{
    uint t;
    switch (DIAG_CTR_INFO[ctr].fmt)
    {
    case _BYTE:
        sprintf (ctr_str,"%3hu",data[DIAG_CTR_INFO[ctr].ofs]);
        return;

    case _WORD:
        t = *((uint*)&data[DIAG_CTR_INFO[ctr].ofs]);
        sprintf (ctr_str,"%5u",t);
        return;

    case _REVWORD:
        t = *((uint*)&data[DIAG_CTR_INFO[ctr].ofs]);
        t = (t >> 8) | (t << 8);
        sprintf (ctr_str,"%5u",t);
        return;

    default:
        *ctr_str='\0';
    }
}

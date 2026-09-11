// SPDX-License-Identifier: GPL-3.0-or-later
/*  This file is part of SeibuCup_MiSTer.

    SeibuCup_MiSTer is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    SeibuCup_MiSTer is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with SeibuCup_MiSTer.  If not, see <http://www.gnu.org/licenses/>.

    Author: Umberto Parisi (rmonic79)
    Version: 1.0
    Date: 2026

*/

// SeibuCup (Banpresto/Bandai 1991) - MiSTer core
// Porting base: Darius MiSTer core. MiSTer Template by Sorgelig.

module emu
(
	input         CLK_50M,
	input         RESET,
	inout  [45:0] HPS_BUS,
	output        CLK_VIDEO,
	output        CE_PIXEL,
	output [12:0] VIDEO_ARX,
	output [12:0] VIDEO_ARY,
	output  [7:0] VGA_R,
	output  [7:0] VGA_G,
	output  [7:0] VGA_B,
	output        VGA_HS,
	output        VGA_VS,
	output        VGA_DE,
	output        VGA_F1,
	output [1:0]  VGA_SL,
	output        VGA_SCALER,
	output        VGA_DISABLE,
	input  [11:0] HDMI_WIDTH,
	input  [11:0] HDMI_HEIGHT,
	output        HDMI_FREEZE,
	output        HDMI_BLACKOUT,
	output        HDMI_BOB_DEINT,

`ifdef MISTER_FB
	output        FB_EN,
	output  [4:0] FB_FORMAT,
	output [11:0] FB_WIDTH,
	output [11:0] FB_HEIGHT,
	output [31:0] FB_BASE,
	output [13:0] FB_STRIDE,
	input         FB_VBL,
	input         FB_LL,
	output        FB_FORCE_BLANK,
`ifdef MISTER_FB_PALETTE
	output        FB_PAL_CLK,
	output  [7:0] FB_PAL_ADDR,
	output [23:0] FB_PAL_DOUT,
	input  [23:0] FB_PAL_DIN,
	output        FB_PAL_WR,
`endif
`endif

	output        LED_USER,
	output  [1:0] LED_POWER,
	output  [1:0] LED_DISK,
	output  [1:0] BUTTONS,

	input         CLK_AUDIO,
	output [15:0] AUDIO_L,
	output [15:0] AUDIO_R,
	output        AUDIO_S,
	output  [1:0] AUDIO_MIX,

	inout   [3:0] ADC_BUS,

	output        SD_SCK,
	output        SD_MOSI,
	input         SD_MISO,
	output        SD_CS,
	input         SD_CD,

	output        DDRAM_CLK,
	input         DDRAM_BUSY,
	output  [7:0] DDRAM_BURSTCNT,
	output [28:0] DDRAM_ADDR,
	input  [63:0] DDRAM_DOUT,
	input         DDRAM_DOUT_READY,
	output        DDRAM_RD,
	output [63:0] DDRAM_DIN,
	output  [7:0] DDRAM_BE,
	output        DDRAM_WE,

	output        SDRAM_CLK,
	output        SDRAM_CKE,
	output [12:0] SDRAM_A,
	output  [1:0] SDRAM_BA,
	inout  [15:0] SDRAM_DQ,
	output        SDRAM_DQML,
	output        SDRAM_DQMH,
	output        SDRAM_nCS,
	output        SDRAM_nCAS,
	output        SDRAM_nRAS,
	output        SDRAM_nWE,

`ifdef MISTER_DUAL_SDRAM
	input         SDRAM2_EN,
	output        SDRAM2_CLK,
	output [12:0] SDRAM2_A,
	output  [1:0] SDRAM2_BA,
	inout  [15:0] SDRAM2_DQ,
	output        SDRAM2_nCS,
	output        SDRAM2_nCAS,
	output        SDRAM2_nRAS,
	output        SDRAM2_nWE,
`endif

	input         UART_CTS,
	output        UART_RTS,
	input         UART_RXD,
	output        UART_TXD,
	output        UART_DTR,
	input         UART_DSR,

	input   [6:0] USER_IN,
	output  [6:0] USER_OUT,

	input         OSD_STATUS,

	// CRT Adjust sys-side: valori dall'OSD + VBlank vero (vedi sys/emu_ports.vh)
	output              CRT_ON,
	output signed [4:0] CRT_HSIZE,
	output signed [8:0] CRT_HPOS,
	output signed [5:0] CRT_VSHIFT,
	output signed [5:0] CRT_VSIZE,
	output              CRT_VSMODE,
	output              CRT_VBL
);

///////// Unused ports /////////
assign ADC_BUS  = 'Z;
assign USER_OUT = '1;
assign {UART_RTS, UART_TXD, UART_DTR} = 0;
assign {SD_SCK, SD_MOSI, SD_CS} = 'Z;
// DDRAM ora pilotato da u_ddram_spr (sprite ROM in DDR3). CLK = clk_sys.
assign DDRAM_CLK = clk_sys;

assign VGA_SL = 0;
assign VGA_F1 = 0;
assign VGA_SCALER  = 0;
assign VGA_DISABLE = 0;
// Pause: toggle on rising edge of joy[12] (standard MiSTer pause bit)
reg pause_toggle;
reg joy_pause_prev;
always @(posedge clk_sys) begin
	if (reset) begin
		pause_toggle <= 1'b0;
		joy_pause_prev <= 1'b0;
	end else begin
		joy_pause_prev <= joy0[12] | joy1[12];
		if ((joy0[12] | joy1[12]) && !joy_pause_prev)
			pause_toggle <= ~pause_toggle;
	end
end
wire pause = pause_toggle | status[17];  // pad OR OSD
assign HDMI_FREEZE = 1'b0;  // overlay pause renderizzato real-time, no freeze scaler
assign HDMI_BLACKOUT = 0;
assign HDMI_BOB_DEINT = 0;

assign AUDIO_S = 1;  // signed audio
wire signed [15:0] game_audio_l, game_audio_r;
assign AUDIO_L = game_audio_l;
assign AUDIO_R = game_audio_r;
assign AUDIO_MIX = 0;

assign LED_DISK = 0;
assign LED_POWER = 0;
assign BUTTONS = 0;

//////////////////////////////////////////////////////////////////

wire [1:0] ar = status[127:126];

// Volumi audio OSD (Q4.4: 16 = 100%, 32 = 200%, 0 = mute)
wire [2:0] osd_fm_vol  = status[88:86];
wire [2:0] osd_oki_vol = status[91:89];

// Mixer per-canale (schema Raiden): 3 bit per selettore, 0=Default 1=Mute 2+=%.
// Bit ricavati dai trim OSD per-layer rimossi: FM 34-60, OKI 61-72.
wire [2:0] osd_fm_ch [0:8];
assign osd_fm_ch[0] = status[36:34];
assign osd_fm_ch[1] = status[39:37];
assign osd_fm_ch[2] = status[42:40];
assign osd_fm_ch[3] = status[45:43];
assign osd_fm_ch[4] = status[48:46];
assign osd_fm_ch[5] = status[51:49];
assign osd_fm_ch[6] = status[54:52];
assign osd_fm_ch[7] = status[57:55];
assign osd_fm_ch[8] = status[60:58];

wire [2:0] osd_oki_ch [0:3];
assign osd_oki_ch[0] = status[63:61];
assign osd_oki_ch[1] = status[66:64];
assign osd_oki_ch[2] = status[69:67];
assign osd_oki_ch[3] = status[72:70];   // batteria
// I volumi globali vanno direttamente al modulo audio come selettori 3 bit:
// la conversione la fa gain_resolve() partendo da DEF_GAIN_FM/DEF_GAIN_OKI
// (guadagni tarati, non unita'), esattamente come Raiden.

// Offset per-layer SeibuCup: SOLO trim OSD ±32 (base 0).
// La finestra +16 della famiglia cupsoc (redraw $C7B0/$C840) e' applicata
// allo SCROLL dei renderer (fetch, vedi bg/mg/fg_scroll_* e text sotto):
// il fetch resta coerente su tutte le colonne — niente banda al bordo.
// Qui restano solo i trim di rifinitura e la base famiglia sprite/text.
// I trim OSD per-layer sono stati RIMOSSI: i loro bit status[] servono ora al
// mixer audio per-canale. TRIM0 = 0 e' esattamente il valore con cui il core
// gira oggi (l'opzione OSD partiva da 0), quindi le posizioni sono INVARIATE.
// Le basi -1/-8/-1 qui sotto restano dov'erano, con le loro motivazioni.
localparam signed [9:0] TRIM0 = 10'sd0;

wire signed [9:0] osd_bg_xoff  = TRIM0;
wire signed [9:0] osd_bg_yoff  = TRIM0;
wire signed [9:0] osd_mg_xoff  = TRIM0;
wire signed [9:0] osd_mg_yoff  = TRIM0;
wire signed [9:0] osd_fg_xoff  = TRIM0;
wire signed [9:0] osd_fg_yoff  = TRIM0;
wire signed [9:0] osd_spr_xoff = TRIM0 - 10'sd1;  // -1 = base famiglia (match MAME su HW)
// -8 = base cupsoc. La finestra visibile e' set_visarea(0,319, 8,247): in MAME
// lo sprite a Y=8 finisce sulla riga 8 della bitmap, che la visarea mostra IN
// CIMA (m_xoffset/m_yoffset del SEI0211 sono 0, legionna.cpp non chiama
// set_offset). Nel core vpos_logic=0 E' GIA' la prima riga visibile, quindi
// senza correzione lo stesso sprite cade 8 righe piu' in basso.
// I tilemap erano gia' allineati perche' ricevono WIN_Y=8 sullo scroll: erano
// gli sprite a essere sfasati rispetto a loro. Si vedeva sulla mini-mappa in
// alto (alta pochi pixel: i puntini dei giocatori ne uscivano fuori), non sul
// campo. Stesso ruolo del -1 su osd_spr_xoff.
wire signed [9:0] osd_spr_yoff = TRIM0 - 10'sd8;
wire signed [9:0] osd_txt_xoff = TRIM0 - 10'sd1;  // -1 pipeline
wire signed [9:0] osd_txt_yoff = TRIM0;

`include "build_id.v"
// ─── MAPPA BIT OSD status[] — bit→significato (NO sovrapposizioni, verificato) ───
//  Per hardcodare un valore trovato nell'OSD: leggi il bit qui, poi sostituisci
//  il segnale RTL corrispondente (colonna "segnale RTL") con un literal.
//  Offset = signed 6-bit: 0..31 = +0..+31, valori "-32..-1" = bit5=1 → negativi.
//
//  bit(s)     opzione                  pag  valori                              segnale RTL
//  [0]        Reset / Reset+OSD        -    momentary (condiviso, OK)           reset_cause
//  [7:5]      Scale                    P1   0=Norm,1=Vint,2=Narrow,3=Wide,4=HV  .SCALE
//  [18]       Clean Pause              P1   Off=0/On=1                          .clean
//  [19]       Refresh Rate             P1   59.4Hz=0/60Hz=1                     .mode_60hz
//  [29]       Text layer enable        P4   On=0/Off=1                          ~status[29]
//  [30]       BG layer enable          P4   On=0/Off=1                          ~status[30]
//  [31]       MG layer enable          P4   On=0/Off=1                          ~status[31]
//  [32]       FG layer enable          P4   On=0/Off=1                          ~status[32]
//  [33]       Sprite layer enable      P4   On=0/Off=1                          ~status[33]
//  [88:86]    FM (YM3812) volume       P3   0=Default(0.69x),1=Mute,2..7=%      osd_fm_vol
//  [91:89]    OKI ADPCM volume         P3   0=Default(0.63x),1=Mute,2..7=%      osd_oki_vol
//  [36:34]..[60:58]  FM Ch1..Ch9 vol   P3   0=Default,1=Mute,2..7=% (3 bit/ch)  osd_fm_ch[0..8]
//  [63:61]..[72:70]  OKI Ch1..Ch4 vol  P3   idem; Ch4 = batteria                osd_oki_ch[0..3]
//  [101]      CRT Adjust On/Off        P1   Off/On                              crt_on
//  [100:96]   CRT H-Size               P1   0/+1..+15/-16..-1 (signed)          hsize_s
//  [85:79]    CRT H-Position           P1   0/+1..+48/-48..-1                    hpos_off
//  [78:74]    CRT V-Shift              P1   0/+1..+15/-16..-1 (signed)           vshift_off
//  [127:126]  Aspect ratio             P1   Original/Full/[ARC1]/[ARC2]         ar
//  NB: la pagina P5 "Layer Position" e' stata RIMOSSA. Gli offset sono ora
//      costanti (TRIM0=0 + basi -1/-8/-1), identici ai valori con cui il core
//      girava prima. I bit liberati sono andati al mixer audio per-canale.
//  NB: status[17] = PAUSE (riga ~162). NON usarlo per opzioni salvabili: un cfg
//      con bit17=1 mette il 68k in halt al boot -> il gioco non parte.
//  [22:20]    Start Stage (diag)       P4   0=Normal,1..5=Stage N                osd_start_stage
//  [74:78] [79:85] [96:100] [101] = CRT Adjust.
//  LIBERI: [4:1] [8:16] [23:28] [73] [92:95] [102:125].
localparam CONF_STR = {
	"SeibuCup;;",
	"-;",
	"O[18],Clean Pause,Off,On;",
	"-;",
	"P1,Video;",
	"P1O[127:126],Aspect ratio,Original,Full Screen,[ARC1],[ARC2];",
	"P1O[7:5],Scale,Normal,V-Integer,Narrower HV-Integer,Wider HV-Integer,HV-Integer;",
	"P1O[19],Refresh Rate,Original 61Hz,60Hz;",
	"P1O[101],CRT Adjust,Off,On;",
	"H1P1O[100:96],CRT H-Size,0,+1,+2,+3,+4,+5,+6,+7,+8,+9,+10,+11,+12,+13,+14,+15,-16,-15,-14,-13,-12,-11,-10,-9,-8,-7,-6,-5,-4,-3,-2,-1;",
	"H1P1O[85:79],CRT H-Position,0,+1,+2,+3,+4,+5,+6,+7,+8,+9,+10,+11,+12,+13,+14,+15,+16,+17,+18,+19,+20,+21,+22,+23,+24,+25,+26,+27,+28,+29,+30,+31,+32,+33,+34,+35,+36,+37,+38,+39,+40,+41,+42,+43,+44,+45,+46,+47,+48,-48,-47,-46,-45,-44,-43,-42,-41,-40,-39,-38,-37,-36,-35,-34,-33,-32,-31,-30,-29,-28,-27,-26,-25,-24,-23,-22,-21,-20,-19,-18,-17,-16,-15,-14,-13,-12,-11,-10,-9,-8,-7,-6,-5,-4,-3,-2,-1;",
	"H1P1O[78:74],CRT V-Shift,0,+1,+2,+3,+4,+5,+6,+7,+8,+9,+10,+11,+12,+13,+14,+15,-16,-15,-14,-13,-12,-11,-10,-9,-8,-7,-6,-5,-4,-3,-2,-1;",
	"H1P1O[107:104],CRT V-Size,0,+1,+2,+3,+4,+5,+6,+7,-7,-6,-5,-4,-3,-2,-1;",
	"H1P1O[108],CRT V-Size Mode,PVM,Cabinet;",
	"-;",
	"DIP;",
	"-;",
	"P3,Audio;",
	"P3O[88:86],FM (YM3812) volume,Default,Mute,25%,50%,75%,100%,150%,200%;",
	"P3O[91:89],OKI ADPCM volume,Default,Mute,25%,50%,75%,100%,150%,200%;",
	"P3-;",
	"P3O[36:34],FM Ch1 Volume,Default,Mute,25%,50%,75%,100%,150%,200%;",
	"P3O[39:37],FM Ch2 Volume,Default,Mute,25%,50%,75%,100%,150%,200%;",
	"P3O[42:40],FM Ch3 Volume,Default,Mute,25%,50%,75%,100%,150%,200%;",
	"P3O[45:43],FM Ch4 Volume,Default,Mute,25%,50%,75%,100%,150%,200%;",
	"P3O[48:46],FM Ch5 Volume,Default,Mute,25%,50%,75%,100%,150%,200%;",
	"P3O[51:49],FM Ch6 Volume,Default,Mute,25%,50%,75%,100%,150%,200%;",
	"P3O[54:52],FM Ch7 Volume,Default,Mute,25%,50%,75%,100%,150%,200%;",
	"P3O[57:55],FM Ch8 Volume,Default,Mute,25%,50%,75%,100%,150%,200%;",
	"P3O[60:58],FM Ch9 Volume,Default,Mute,25%,50%,75%,100%,150%,200%;",
	"P3-;",
	"P3O[63:61],OKI Ch1 Volume,Default,Mute,25%,50%,75%,100%,150%,200%;",
	"P3O[66:64],OKI Ch2 Volume,Default,Mute,25%,50%,75%,100%,150%,200%;",
	"P3O[69:67],OKI Ch3 Volume,Default,Mute,25%,50%,75%,100%,150%,200%;",
	"P3O[72:70],OKI Ch4 Volume (batteria),Default,Mute,25%,50%,75%,100%,150%,200%;",
	"-;",
	"P4,Debug;",
	"P4O[30],BG layer,On,Off;",
	"P4O[31],MG layer,On,Off;",
	"P4O[32],FG layer,On,Off;",
	"P4O[33],Sprite layer,On,Off;",
	"P4O[29],Text layer,On,Off;",
	"P4O[22:20],Start Stage,Normal,Phase 1,Phase 2,Stage 4,Stage 5,Stage 6,Ending;",
	"-;",
	"T[0],Reset;",
	"R[0],Reset and close OSD;",
	// J1: bit 4=Fire(A), 5=Roll(B), 6=Dynamite(X/C), 7,8,9=unused, 10=Start1, 11=Coin1, 12=Pause
	// 13=Start2, 14=Coin2 (MiSTer arcade convention fissa)
	"J1,Attack,Jump,Debug,-,-,-,Start,Coin,Pause,Start 2P,Coin 2P;",
	"jn,A,B,X,,,,Start,R,L,Select,;",
	"V,v",`BUILD_DATE
};

wire forced_scandoubler;
wire  [1:0] buttons;
wire [127:0] status;
wire [10:0] ps2_key;
wire [15:0] joy0, joy1, joy2, joy3;
wire        ioctl_download;
wire [15:0] ioctl_index;
wire        ioctl_wr;
wire [26:0] ioctl_addr;
wire [15:0] ioctl_dout;   // 16-bit: WIDE=1
wire        ioctl_wait;

hps_io #(.CONF_STR(CONF_STR), .WIDE(1)) hps_io
(
	.clk_sys(clk_sys),
	.HPS_BUS(HPS_BUS),
	.EXT_BUS(),
	.gamma_bus(),
	.forced_scandoubler(forced_scandoubler),
	.buttons(buttons),
	.status(status),
	.status_menumask({14'd0, ~status[101], 1'b0}),  // H1 group (CRT Adjust) shown only when On
	.ps2_key(ps2_key),
	.joystick_0(joy0),
	.joystick_1(joy1),
	.joystick_2(joy2),
	.joystick_3(joy3),
	.ioctl_download(ioctl_download),
	.ioctl_index(ioctl_index),
	.ioctl_wr(ioctl_wr),
	.ioctl_addr(ioctl_addr),
	.ioctl_dout(ioctl_dout),
	.ioctl_wait(ioctl_wait)
);

// --- Joystick to SD Gundam input mapping ---
// MAME P1_P2 port ($E0002): active low.
// Low byte P1 / high byte P2: bit0=U, bit1=D, bit2=L, bit3=R, bit4=Btn1, bit5=Btn2.
// MiSTer joy bits: joy[0]=R, joy[1]=L, joy[2]=D, joy[3]=U, joy[4]=A, joy[5]=B.
// MAME PLAYERS12 (legionna.cpp:439-444): bit[0]=UP, [1]=DOWN, [2]=LEFT, [3]=RIGHT,
// [4]=BTN1, [5]=BTN2, [6]=BTN3, [7]=unused (active low).
// Mappa direzioni: game bit0=UP=joy0[3], bit1=DOWN=joy0[2], bit2=LEFT=joy0[1],
// bit3=RIGHT=joy0[0]. (Prima erano in ordine joy0[3..0] = U,D,L,R sui bit 3..0
// del game -> rotazione 90gradi su->destra ecc. Corretto invertendo i 4 bit.)
wire [7:0] p1_input = {1'b1, ~joy0[6], ~joy0[5], ~joy0[4], ~joy0[0], ~joy0[1], ~joy0[2], ~joy0[3]};
wire [7:0] p2_input = {1'b1, ~joy1[6], ~joy1[5], ~joy1[4], ~joy1[0], ~joy1[1], ~joy1[2], ~joy1[3]};
wire [15:0] p1_p2_input = {p2_input, p1_input};

// PLAYERS34 ($100748), mappa del driver CUPSOC (non di heatbrl, che mette i
// gettoni in un'altra porta): INPUT_PORTS_START(cupsoc), porta "PLAYERS34"
//   bit0-3 U/D/L/R   bit4 Shoot   bit5 Pass   bit6 Debug   bit7 COIN3
//   bit8-11 idem P4  bit12        bit13       bit14        bit15 COIN4
// Tutti active low. Direzioni nell'ordine di p1_input/p2_input qui sopra
// (game bit0=UP=joy[3] ... bit3=RIGHT=joy[0]). Il bit Debug resta a 1.
wire [7:0] p3_input = {~joy2[11], 1'b1, ~joy2[5], ~joy2[4], ~joy2[0], ~joy2[1], ~joy2[2], ~joy2[3]};
wire [7:0] p4_input = {~joy3[11], 1'b1, ~joy3[5], ~joy3[4], ~joy3[0], ~joy3[1], ~joy3[2], ~joy3[3]};
wire [15:0] p3_p4_input = {p4_input, p3_input};

// SeibuCup SYSTEM port ($10074C) — MAME legionna.cpp:427-436
//   bit0 = START1 (active LOW)
//   bit1 = START2 (active LOW)
//   bit2..7 = UNKNOWN → tied 1
//   bit8..15 = UNKNOWN → tied 1
// Coin inputs NON sono qui: vanno via SEIBU_COIN_INPUTS → Z80 → soundlatch.
// Nessun service switch in SeibuCup SYSTEM port (era ipotesi residua SD Gundam).
// bit8 = START3, bit9 = START4 (MAME porta "SYSTEM" di cupsoc).
wire [15:0] system_input16 = {6'h3F, ~joy3[10], ~joy2[10],    // bit9=START4 bit8=START3
                              6'h3F, ~joy1[10], ~joy0[10]};   // bit1=START2 bit0=START1

// Seibu coin input (ACTIVE_HIGH per SEIBU_COIN_INPUTS macro): bit0=COIN1, bit1=COIN2.
// Letto dal Z80 a 0x4013 → coin_r → soundlatch sub2main → main 68k legge 0xA0004.
// In sim mode: auto-press coin1 every ~half second (sim accel)
`ifdef MISTER_SIM
reg [27:0] sim_coin_cnt = 0;
reg sim_coin_press = 1'b0;
always @(posedge clk_sys) begin
	sim_coin_cnt <= sim_coin_cnt + 1;
	if (sim_coin_cnt == 28'd50_000_000) sim_coin_press <= 1'b1;   // ~1s @96MHz
	if (sim_coin_cnt == 28'd60_000_000) sim_coin_press <= 1'b0;   // release
	if (sim_coin_cnt == 28'd70_000_000) sim_coin_press <= 1'b1;   // press start
end
wire [7:0] coin_input = {6'd0, sim_coin_press, sim_coin_press};
`else
wire [7:0] coin_input = {6'd0, joy1[11], joy0[11]};
`endif

// DIP switches — loaded from MRA via ioctl (index 254)
// Active-LOW: default "FF,FF,FF,FF" = all OFF = all 1s
// cupsoc: DSW1 ($100740, word 0) + DSW2 ($10075C, word 1)
reg [15:0] dip_sw  = 16'hFFFF;
reg [15:0] dip_sw2 = 16'hFFFF;
always @(posedge clk_sys)
	if (ioctl_wr && (ioctl_index == 16'd254)) begin
		if (ioctl_addr[26:1] == 26'd0) dip_sw  <= ioctl_dout;
		if (ioctl_addr[26:1] == 26'd1) dip_sw2 <= ioctl_dout;
	end

// ── Variante del set — byte MRA <rom index="1"> ─────────────────────────────
// L'HPS scarica index=1 PRIMA dei ROM (index=0), quindi il byte e' gia'
// latchato quando arriva il primo dato: si puo' usare per modificare il flusso
// mentre passa. I DIP (index=254) arrivano DOPO e non servirebbero.
// Non azzerato dal reset di gioco: solo ioctl_wr lo scrive.
//   00 = cupsoc / cupsoca      nessuna modifica
//   01 = olysoc92 / a / b      [$0FFFFE] = $3032  -> titolo Olympic Soccer '92
//   02 = cupsocb               [$0FFFFA] = $00FF  -> spegne il testo di debug
// Sono le stesse due scritture che MAME fa in init_olysoc92 / init_cupsocs.
reg [7:0] set_variant = 8'd0;
always @(posedge clk_sys)
	if (ioctl_wr && (ioctl_index == 16'd1)) set_variant <= ioctl_dout[7:0];

wire is_main_dl = ioctl_download && (ioctl_index == 16'd0);
wire hit_oly    = (set_variant == 8'd1) && is_main_dl && (ioctl_addr[26:1] == 27'h0FFFFE >> 1);
wire hit_dbg    = (set_variant == 8'd2) && is_main_dl && (ioctl_addr[26:1] == 27'h0FFFFA >> 1);
wire [15:0] ioctl_dout_set = hit_oly ? 16'h3032
                           : hit_dbg ? 16'h00FF
                                     : ioctl_dout;

///////////////////////   CLOCKS   ///////////////////////////////

wire clk_sys;
wire pll_locked /*verilator public_flat_rd*/;
pll pll
(
	.refclk(CLK_50M),
	.rst(0),
	.outclk_0(clk_sys),
	.locked(pll_locked)
);

// Game reset: 17-bit hold (~1.4ms @ 96MHz). Bilancio: troppo corto e
// SDRAM bridge non stabilizza; troppo lungo e vblank IRQ4 si arma prima
// che ROM init code finisca → crash subito dopo reset_release.
// Download stretch: dopo che ioctl_download cade, prolunga reset_cause per N cicli.
// Evita rilascio reset CPU prima che SDRAM bridge abbia finito di propagare gli ultimi
// write delle ROM (download multi-bank: 4 banchi per ogni word del main CPU ROM).
reg [23:0] dl_stretch_cnt = 24'd0;
reg        ioctl_download_prev = 1'b0;
always @(posedge clk_sys) begin
	ioctl_download_prev <= ioctl_download;
	if (ioctl_download) dl_stretch_cnt <= 24'hFFFFFF;
	else if (dl_stretch_cnt != 24'd0) dl_stretch_cnt <= dl_stretch_cnt - 24'd1;
end
wire dl_stretch_active = (dl_stretch_cnt != 24'd0);

wire reset_cause /*verilator public_flat_rd*/ = RESET | status[0] | buttons[1] | ~pll_locked | ioctl_download | dl_stretch_active;
reg [23:0] reset_hold_cnt /*verilator public_flat_rd*/ = 24'hFFFFFF;
always @(posedge clk_sys) begin
	if (reset_cause)                  reset_hold_cnt <= 24'hFFFFFF;
	else if (reset_hold_cnt != 24'd0) reset_hold_cnt <= reset_hold_cnt - 24'd1;
end
wire reset /*verilator public_flat_rd*/ = (reset_hold_cnt != 24'd0);
// Bridge reset: ONLY pll_locked — bridge must run during download, before RESET drops
wire bridge_reset = ~pll_locked;
// Video reset: ONLY pll_locked — CRT needs sync always, even during RESET and download
wire video_reset = ~pll_locked;

///////////////////////   SDRAM   ///////////////////////////////

// Genesis 4-port SDRAM controller (Sorgelig + donor bridge)
// Port 0: graphics ROM + download
// Port 1: main 68000 ROM
// Port 2: temporarily unused donor ROM path
// Port 3: audio/sample ROM path

wire [24:1] sd_addr0, sd_addr1, sd_addr2, sd_addr3;
wire [15:0] sd_din0, sd_din1, sd_din2, sd_din3;
wire        sd_wrl0, sd_wrh0, sd_wrl1, sd_wrh1, sd_wrl2, sd_wrh2, sd_wrl3, sd_wrh3;
wire        sd_req0, sd_req1, sd_req2, sd_req3;
wire        sd_ack0, sd_ack1, sd_ack2, sd_ack3;
wire [15:0] sd_dout0, sd_dout1, sd_dout2, sd_dout3;
wire        sdram_ready;

// OKI ADPCM ROM bridge ↔ jt6295 (via main_top). 19 bit: bit18 = bank godzilla.
wire [18:0] oki_rom_addr;
wire  [7:0] oki_rom_data;
wire        oki_rom_ok;

sdram sdram_ctrl
(
	.SDRAM_DQ(SDRAM_DQ),
	.SDRAM_A(SDRAM_A),
	.SDRAM_DQML(SDRAM_DQML),
	.SDRAM_DQMH(SDRAM_DQMH),
	.SDRAM_BA(SDRAM_BA),
	.SDRAM_nCS(SDRAM_nCS),
	.SDRAM_nWE(SDRAM_nWE),
	.SDRAM_nRAS(SDRAM_nRAS),
	.SDRAM_nCAS(SDRAM_nCAS),
	.SDRAM_CLK(SDRAM_CLK),
	.SDRAM_CKE(SDRAM_CKE),

	.init(~pll_locked),
	.clk(clk_sys),
	// VIDEO 75% (mode 3): porta 0 (tile) garantita >=2 slot su 3, il terzo slot
	// va a 68k/COP/OKI. Budget video: 252 word/riga x 8 clk x 1.5 = 3024 clk
	// << 6104 (riga intera) -> il prefetch NON puo' sforare nemmeno a cache
	// fredda e porte sature. Il vecchio video-first (2'd1) affamava i fetch ROM
	// del 68k nei burst di prefetch a inizio riga (~1k clk di starvation piena,
	// 224 volte a frame) -> main loop lento -> la coda (camera $B2A0) sconfinava
	// nell'IRQ4 -> redraw/commit spaiati -> pop 16px scroll FG/MG. Il round
	// robin EQUO (2'd0) resta vietato: video ~1/2 slot puo' sforare davvero.
	.prio_mode(2'd3),
	.ready(sdram_ready),

	.addr0(sd_addr0), .wrl0(sd_wrl0), .wrh0(sd_wrh0),
	.din0(sd_din0), .dout0(sd_dout0), .req0(sd_req0), .ack0(sd_ack0),

	.addr1(sd_addr1), .wrl1(sd_wrl1), .wrh1(sd_wrh1),
	.din1(sd_din1), .dout1(sd_dout1), .req1(sd_req1), .ack1(sd_ack1),

	.addr2(sd_addr2), .wrl2(sd_wrl2), .wrh2(sd_wrh2),
	.din2(sd_din2), .dout2(sd_dout2), .req2(sd_req2), .ack2(sd_ack2),

	.addr3(sd_addr3), .wrl3(sd_wrl3), .wrh3(sd_wrh3),
	.din3(sd_din3), .dout3(sd_dout3), .req3(sd_req3), .ack3(sd_ack3)
);

///////////////////////   BRIDGE   ///////////////////////////////

// Bridge between game logic (level protocol) and Genesis SDRAM (toggle protocol)
wire [23:0] game_tile_addr, game_main_addr /*verilator public_flat_rd*/, game_sub_addr;
wire        game_tile_req, game_main_req /*verilator public_flat_rd*/, game_sub_req;
wire  [2:0] game_tile_kind;     // 0=BG, 1=MG, 2=FG, 3=SPR, 4=TXT
wire [31:0] game_tile_data;
wire        game_tile_valid;
wire [15:0] game_main_data /*verilator public_flat_rd*/, game_sub_data;
// Audio Z80 ROM removed from SDRAM — will use BRAM when audio implemented
wire        game_main_ready /*verilator public_flat_rd*/, game_sub_ready;

// ROM instruction cache — between game and SDRAM bridge
wire [23:0] bridge_main_addr /*verilator public_flat_rd*/, bridge_sub_addr;
wire        bridge_main_req  /*verilator public_flat_rd*/, bridge_sub_req;
wire [15:0] bridge_main_data /*verilator public_flat_rd*/, bridge_sub_data;
wire        bridge_main_ready /*verilator public_flat_rd*/, bridge_sub_ready;

// CACHE_BITS 12 = 4K word (8KB): meno miss del 68k → meno traffico port 1 →
// meno contesa SDRAM residua col fetch tile (già declassato dal video-first).
rom_cache #(.CACHE_BITS(12)) u_main_cache (
	.clk(clk_sys), .reset(reset),
	.cpu_addr(game_main_addr), .cpu_req(game_main_req),
	.cpu_data(game_main_data), .cpu_ready(game_main_ready),
	.sdram_addr(bridge_main_addr), .sdram_req(bridge_main_req),
	.sdram_data(bridge_main_data), .sdram_ready(bridge_main_ready)
);

rom_cache #(.CACHE_BITS(9)) u_sub_cache (
	.clk(clk_sys), .reset(reset),
	.cpu_addr(game_sub_addr), .cpu_req(game_sub_req),
	.cpu_data(game_sub_data), .cpu_ready(game_sub_ready),
	.sdram_addr(bridge_sub_addr), .sdram_req(bridge_sub_req),
	.sdram_data(bridge_sub_data), .sdram_ready(bridge_sub_ready)
);

sdram_bridge bridge
(
	.clk(clk_sys),
	.reset(bridge_reset),
	.sdram_ready(sdram_ready),

	// HPS download
	.ioctl_download(ioctl_download),
	.ioctl_wr(ioctl_wr),
	.ioctl_addr(ioctl_addr),
	.ioctl_dout(ioctl_dout_set),   // flusso con la modifica di variante applicata
	.ioctl_index(ioctl_index),
	.ioctl_wait(ioctl_wait),

	// Game: Tile ROM (32-bit)
	.tile_byte_addr(game_tile_addr),
	.tile_req(game_tile_req),
	.gfx_kind(game_tile_kind),
	.tile_data(game_tile_data),
	.tile_valid(game_tile_valid),

	// Game: Main CPU ROM (16-bit)
	.main_byte_addr(bridge_main_addr),
	.main_req(bridge_main_req),
	.main_data(bridge_main_data),
	.main_ready(bridge_main_ready),

	// Game: temporarily unused donor ROM port
	.sub_byte_addr(bridge_sub_addr),
	.sub_req(bridge_sub_req),
	.sub_data(bridge_sub_data),
	.sub_ready(bridge_sub_ready),

	// OKI ADPCM ROM (port 3)
	.oki_byte_addr(oki_rom_addr),
	.oki_data(oki_rom_data),
	.oki_ok(oki_rom_ok),

	// SDRAM ports
	.sdram_addr0(sd_addr0), .sdram_din0(sd_din0),
	.sdram_wrl0(sd_wrl0), .sdram_wrh0(sd_wrh0),
	.sdram_req0(sd_req0), .sdram_ack0(sd_ack0), .sdram_dout0(sd_dout0),

	.sdram_addr1(sd_addr1), .sdram_din1(sd_din1),
	.sdram_wrl1(sd_wrl1), .sdram_wrh1(sd_wrh1),
	.sdram_req1(sd_req1), .sdram_ack1(sd_ack1), .sdram_dout1(sd_dout1),

	.sdram_addr2(sd_addr2), .sdram_din2(sd_din2),
	.sdram_wrl2(sd_wrl2), .sdram_wrh2(sd_wrh2),
	.sdram_req2(sd_req2), .sdram_ack2(sd_ack2), .sdram_dout2(sd_dout2),

	.sdram_addr3(sd_addr3), .sdram_din3(sd_din3),
	.sdram_wrl3(sd_wrl3), .sdram_wrh3(sd_wrh3),
	.sdram_req3(sd_req3), .sdram_ack3(sd_ack3), .sdram_dout3(sd_dout3)
);

///////////////////////   GAME   ///////////////////////////////

wire [9:0]  render_x;
wire [8:0]  render_y;
wire [15:0] map_xscroll_l0, map_xscroll_l1;
wire [15:0] map_ctrl_l0;
wire [15:0] map_yscroll_l0, map_yscroll_l1;
wire [15:0] map_xscroll_mg, map_yscroll_mg;
wire [15:0] map_text_base_y;
wire [15:0] map_base_bg_x, map_base_bg_y, map_base_mg_x, map_base_mg_y;
wire [15:0] map_base_fg_x, map_base_fg_y, map_base_txt_x;

darius_dual68k_top game
(
	.clk(clk_sys),
	.reset(reset),
	.pause(pause),
	.clk_sel(3'd2),              // Main CPU 10 MHz (target reale Gundam)
	.sub_clk_sel(3'd0),          // donor Darius path kept until the sub CPU is removed
	.z80_clk_sel(2'd0),          // Sound CPU default (3.58 MHz)
	.p1_input(p1_input),
	.p2_input(p2_input),
	.pin34_input(p3_p4_input),
	.system_input(system_input16),
	.dsw_input(dip_sw),
	.dsw2_input(dip_sw2),
	// Start Stage OSD: 0=Normal; 1-2 = giri missioni successivi (Phase 1/2,
	// mission select con difficolta'/mappe del giro N); 3-6 = fasi fisse
	// (Stage 4..7). Mapping diretto fase = valore.
	.osd_start_phase(status[22:20]),

	// SDRAM ROM (via bridge)
	.main_rom_rdata(game_main_data),
	.main_rom_ready(game_main_ready),
	.sub_rom_rdata(game_sub_data),
	.sub_rom_ready(game_sub_ready),
	.tilerom_data(game_tile_data),
	.tilerom_valid(game_tile_valid),

	.main_rom_addr(game_main_addr),
	.main_rom_req(game_main_req),
	.sub_rom_addr(game_sub_addr),
	.sub_rom_req(game_sub_req),
	// tilerom_* main_top tied off: arbiter Gundam pilota il bridge direttamente
	.tilerom_addr(),
	.tilerom_req(),
	.tilerom_kind(),

	// Audio ROM download (ioctl → BRAM)
	.ioctl_download(ioctl_download),
	.ioctl_wr(ioctl_wr),
	.ioctl_addr(ioctl_addr),
	.ioctl_dout(ioctl_dout),

	// Video
	.render_x(render_x),
	.render_y(render_y),
	.vblank_in(VBlank),
	// Scroll esposti dal CRTC Seibu
	.xscroll_l0(map_xscroll_l0),
	.xscroll_l1(map_xscroll_l1),
	.yscroll_l0(map_yscroll_l0),
	.yscroll_l1(map_yscroll_l1),
	.xscroll_mg(map_xscroll_mg),
	.yscroll_mg(map_yscroll_mg),
	.text_base_y(map_text_base_y),
	.base_bg_x(map_base_bg_x),   .base_bg_y(map_base_bg_y),
	.base_mg_x(map_base_mg_x),   .base_mg_y(map_base_mg_y),
	.base_fg_x(map_base_fg_x),   .base_fg_y(map_base_fg_y),
	.base_txt_x(map_base_txt_x),
	.ctrl_l0(map_ctrl_l0),
	// Palette read port (per video pipeline)
	.pal_b_addr(pal_b_addr),
	.pal_b_r(pal_b_r),
	.pal_b_g(pal_b_g),
	.pal_b_b(pal_b_b),
	// Text VRAM read (renderer text legge tile word)
	.text_vram_addr(text_vram_addr),
	.text_vram_data(text_vram_data),
	// BG VRAM read (renderer BG legge tile word)
	.bg_vram_addr(bg_vram_addr),
	.bg_vram_data(bg_vram_data),
	// MG VRAM read
	.mg_vram_addr(mg_vram_addr),
	.mg_vram_data(mg_vram_data),
	// FG VRAM read
	.fg_vram_addr(fg_vram_addr),
	.fg_vram_data(fg_vram_data),
	// Sprite VRAM read (per scanner sprite)
	.spr_vram_addr(spr_vram_addr),
	.spr_vram_data(spr_vram_data),
	// gfx_bank (per MG)
	.gfx_bank(gfx_bank),
	// Coin input (HW button → Z80 → soundlatch → main 0xA0004)
	.coin_input(coin_input),
	// OKI ADPCM ROM bridge (port 3)
	.oki_rom_addr(oki_rom_addr),
	.oki_rom_data(oki_rom_data),
	.oki_rom_ok(oki_rom_ok),
	// Volumi OSD
	.fm_vol_sel(osd_fm_vol),
	.oki_vol_sel(osd_oki_vol),
	.fm_ch_vol_sel0(osd_fm_ch[0]), .fm_ch_vol_sel1(osd_fm_ch[1]), .fm_ch_vol_sel2(osd_fm_ch[2]),
	.fm_ch_vol_sel3(osd_fm_ch[3]), .fm_ch_vol_sel4(osd_fm_ch[4]), .fm_ch_vol_sel5(osd_fm_ch[5]),
	.fm_ch_vol_sel6(osd_fm_ch[6]), .fm_ch_vol_sel7(osd_fm_ch[7]), .fm_ch_vol_sel8(osd_fm_ch[8]),
	.oki_ch_vol_sel0(osd_oki_ch[0]), .oki_ch_vol_sel1(osd_oki_ch[1]),
	.oki_ch_vol_sel2(osd_oki_ch[2]), .oki_ch_vol_sel3(osd_oki_ch[3]),
	// Audio
	.audio_l(game_audio_l),
	.audio_r(game_audio_r)
);

// Palette read-side: indirizzo deciso dal pixel pipeline (priorità tra layer)
wire [10:0] pal_b_addr;
wire [7:0]  pal_b_r, pal_b_g, pal_b_b;

// Text VRAM read wires
wire [10:0] text_vram_addr;
wire [15:0] text_vram_data;

///////////////////////   VIDEO   ///////////////////////////////

// SD Gundam timing single-screen 320x224 @ 59.4 Hz (original) / 60.1 Hz.
// Pixel clock 6 MHz = clk_sys/16. HTotal=384, VTotal=263 (59.4) o 260 (60Hz).
wire ce_pix;
wire HBlank, VBlank, HSync, VSync, video_de;
wire [9:0] timing_hpos;
wire [9:0] timing_vpos;

SeibuCup_video_timing u_video_timing (
	.clk        (clk_sys),
	.reset      (video_reset),
	.mode_60hz  (status[19]),
	.ce_pix     (ce_pix),
	.hpos       (timing_hpos),
	.vpos       (timing_vpos),
	.active_x   (render_x),
	.active_y   (render_y),
	.hblank     (HBlank),
	.vblank     (VBlank),
	.hsync      (HSync),
	.vsync      (VSync),
	.de         (video_de)
);

// ── Flip screen (CRTC reg 0x1A bit 0) ───────────────────────────────────────
// MAME: BIT(reg_1a, 0) → flip_screen. Arcade reale = monitor CRT capovolto;
// game scrive in VRAM convinto che lo schermo sia ruotato 180° → noi vediamo
// flippato finchè non invertiamo.
//
// Strategia:
//  - X flip: il read-side dei line_buffer (dentro tile_layer/text_renderer)
//    legge linebuf[hpos]. Sostituisco hpos con (319-hpos) → mostra mirror H.
//  - Y flip: la prefetch durante display vpos=N riempie il buffer per il
//    display vpos=N+1. In flip ON serve che mostri riga ROM (V_VISIBLE-1-(N+1))
//    = (V_VISIBLE-2-N). Sostituisco vpos del prefetch con (V_VISIBLE-2-vpos)
//    quando flip on, così target_y = V_VISIBLE-2-vpos + 1 = V_VISIBLE-1-vpos.
wire        flip_screen = map_ctrl_l0[5];
// Coordinate LOGICHE display (0..319, 0..223), shiftate dal timing CRTC raw.
// Il timing CRTC ora è SYNC→BP→VISIBLE→FP, quindi VISIBLE inizia a hpos=48,
// vpos=30. I renderer devono vedere coordinate "del gioco" 0..319/0..223.
localparam [9:0] H_VIS_START_TOP = 10'd88;   // = H_SYNC + H_BP del video_timing (32+56, HTotal 436)
localparam [8:0] V_VIS_START_TOP = 9'd13;    // = V_SYNC + V_BP del video_timing (3+10, VTotal 262)
wire [9:0]  hpos_logic = timing_hpos - H_VIS_START_TOP;
// (text_dy: vedi u_text sotto — segno = reg - 0x1EF, calibrazione famiglia)
wire [8:0]  vpos_logic = timing_vpos[8:0] - V_VIS_START_TOP;
// Tile_layer prefetch: con flip serve vpos_for_pf = 223 - vpos_logic (range 0..223,
// come il text a vpos_for_text). Con 222 (vecchio), a vpos_logic=223 -> vpos_for_pf =
// 222-223 = -1 = 511 (9-bit unsigned) >= V_VISIBLE -> vpos_visible=0 -> NESSUN toggle di
// active_buf su quella riga -> 223 toggle/frame (DISPARI) -> il double-buffer del BG si
// INVERTE ogni frame -> l'intero BG alterna buffer -> FLICKER (visibile nelle 2 righe
// basse, dove i due buffer differiscono, in stage 4 con flip attivo dalla schermata-mappa).
// Con 223: range 0..223, tutti < 224 -> 224 toggle (PARI) -> active_buf stabile -> no flicker.
// Lo slot di prefetch della PRIMA riga del frame (che nativamente cadrebbe
// all'ultima riga visibile, raster 248 = vpos_logic 223) e' SPOSTATO nel
// tardo vblank (raster 23): l'ISR IRQ4 del gioco gira alle righe ~249-253
// (DMA 0x14 contenuto + write scroll, disasm $37A/$444/$452) — prefetchando
// la riga 0 a raster 248 si accoppiava contenuto/scroll del vblank VECCHIO
// col frame nuovo → pop di 16px ai confini colonna. A raster 23 l'ISR e'
// finita da >25 righe: coppia scroll/contenuto dello stesso vblank per
// TUTTE le righe. Il conteggio toggle del double-buffer resta 224/frame
// (slot spostato, non aggiunto: l'originale a vpos_logic 223 e' inertizzato).
wire [8:0]  vpos_pf_src   = (timing_vpos == 10'd11)   ? 9'd239 :
                            (vpos_logic == 9'd239)    ? 9'd300 :   // slot originale OFF (>=240 = inerte)
                                                        vpos_logic;
wire [8:0]  vpos_for_pf   = flip_screen ? (9'd239 - vpos_pf_src) : vpos_pf_src;
// Text renderer legge tile direttamente da eff_y = vpos (no prefetch ahead).
wire [8:0]  vpos_for_text = flip_screen ? (9'd239 - vpos_logic) : vpos_logic;
// Read path (X): tile_layer linebuf[hpos]. Per text è eff_x = hpos+scroll.
wire [9:0]  hpos_for_read = flip_screen ? (10'd319 - hpos_logic) : hpos_logic;

// ── Text layer renderer (8x8, 4bpp, 64x32 grid) ─────────────────────────────
// SeibuCup MAME (ROM_START godzilla): "char" = REGIONE PROPRIA 128KB (11.620/10.615,
// ROM_LOAD16_BYTE), NON dentro user1 come Legionnaire. char NON descrambled (lineare).
// MRA carica char a ioctl 0x100000..0x11FFFF (128KB, = TXT region SDRAM). Il text lo
// legge tutto (128KB, tile index 16-bit). charrom BRAM 64Kw × 16-bit.
wire        text_opaque;
wire [10:0] text_pen;

wire        text_rom_dl_wr =
	ioctl_download && ioctl_wr && (ioctl_index == 16'd0) &&
	(ioctl_addr >= 27'h100000) && (ioctl_addr < 27'h120000);
// 17-bit address relativo alla region char (= ioctl_addr - 0x100000, 0..0x1FFFF).
wire [16:0] text_rom_dl_offset = ioctl_addr[16:0] - 17'h00000;   // gia' relativo: [16:0] di 0x1xxxxx
// SeibuCup text scroll-base (CRTC reg 0x3A, hook godzilla-only legionna.cpp:1326).
// SEGNO: source_row = vpos + (reg - 0x1EF). Calibrazione HW di famiglia (dump
// vregs in seibu_crtc.cpp): Legionnaire reg=0x01FF, visarea da riga 16 →
// 0x1FF-0x1EF=+16 = ESATTAMENTE il vecchio yoff text +16 del core Legionnaire
// funzionante; D-Con reg=0xFFEF (low9=0x1EF), visarea da 0 → dy=0.
// La formula MAME set_scrolldy(0x1EF-data) è INVERTITA rispetto all'HW
// (convenzione dy del tilemap.cpp) — mai fidarsi del verso MAME.
// Reg==0 (mai scritto dal gioco) → dy=0.
// (la scrolldy dinamica del text dal base $3A era godzilla-only — cupsoc
// usa la finestra fissa +16 applicata a scroll_x/scroll_y di u_text.)
SeibuCup_text_renderer u_text (
	.clk          (clk_sys),
	.reset        (reset),
	.ce_pix       (ce_pix),
	.hpos         (hpos_for_read),
	.vpos         (vpos_for_text),
	.de           (video_de),
	.layer_en     (map_ctrl_l0[3] & ~status[29]),
	.scroll_x     (WIN_X),    // stessa finestra dei tile layer (0, +8)
	.scroll_y     (WIN_Y),
	.xoff         (osd_txt_xoff),
	.yoff         (osd_txt_yoff),
	.vram_addr    (text_vram_addr),
	.vram_data    (text_vram_data),
	.rom_dl_wr    (text_rom_dl_wr),
	.rom_dl_addr  (text_rom_dl_offset),
	.rom_dl_data  (ioctl_dout),
	.opaque       (text_opaque),
	.pen_index    (text_pen)
);

// ── BG/MG/FG layer renderer (16x16, 4bpp, 32x32) — fetch SDRAM via arbiter ──
wire        bg_opaque, mg_opaque, fg_opaque;
wire [10:0] bg_pen, mg_pen, fg_pen;
wire [10:0] bg_vram_addr, mg_vram_addr, fg_vram_addr;
wire [15:0] bg_vram_data, mg_vram_data, fg_vram_data;
wire [15:0] gfx_bank;

// new_line pulse: hpos passa da H_TOTAL-1 a 0
reg [9:0] hpos_prev;
always @(posedge clk_sys) if (ce_pix) hpos_prev <= timing_hpos;
wire layer_new_line = ce_pix && (timing_hpos == 10'd0) && (hpos_prev != 10'd0);

// SeibuCup CRTC scroll passthrough (legionna_v.cpp:34-49 tile_scroll_w):
//   scroll_ram[0/1] → BG (m_background_layer)
//   scroll_ram[2/3] → MG (m_midground_layer)
//   scroll_ram[4/5] → FG (m_foreground_layer)
// Nel nostro CRTC: xscroll_l0 = ram[0/1], xscroll_mg = ram[2/3], xscroll_l1 = ram[4/5].
// Finestra visibile di CUPSOC: legionna.cpp fa
//     screen.set_visarea(0*8, 40*8-1, 1*8, 31*8-1)  =  [0..319] x [8..247]
// quindi origine X = 0 e origine Y = 8.
// ⚠ NON +16/+16: quello e' grainbow, da cui questo core e' stato forkato
// (il vecchio commento citava perfino il redraw $C7B0/$C840 di grainbow, che
// in cupsoc non esiste - verificato: nessun pattern addi/andi/lsr sulla
// camera in tutto il MB).
// L'offset va applicato allo SCROLL (fetch), mai al dst: spostare il dst
// lascia scoperta l'ultima colonna del linebuffer = banda di garbage al bordo.
// Vedi MiSTer_Discovery_Docs/14_visarea_window_offset_scroll_fetch_not_dst.md
localparam [15:0] WIN_X = 16'd0;
localparam [15:0] WIN_Y = 16'd8;
wire [15:0] bg_scroll_x = map_xscroll_l0 + WIN_X;
wire [15:0] bg_scroll_y = map_yscroll_l0 + WIN_Y;
wire [15:0] mg_scroll_x = map_xscroll_mg + WIN_X;
wire [15:0] mg_scroll_y = map_yscroll_mg + WIN_Y;
wire [15:0] fg_scroll_x = map_xscroll_l1 + WIN_X;
wire [15:0] fg_scroll_y = map_yscroll_l1 + WIN_Y;

// Arbiter wires
wire        arb_bg_req,  arb_mg_req,  arb_fg_req;
wire [23:0] arb_bg_addr, arb_mg_addr, arb_fg_addr;
wire [31:0] arb_bg_data, arb_mg_data, arb_fg_data;
wire        arb_bg_valid, arb_mg_valid, arb_fg_valid;

// SeibuCup BG: tilemap 32x32 (legionna_v.cpp:204). MAP_HEIGHT_4=0.
// MAME GFXDECODE "back" color base = 0 (gfx_legionna riga 1168), 32 colorset.
// HAS_TRANSP=1 perché m_background_layer->set_transparent_pen(15) in
// legionna_v.cpp:216 (pen 15 = trasparente, NON opaco come Blood Bros).
SeibuCup_tile_layer #(
	.COLOR_BASE   (11'h000),
	.HAS_TRANSP   (1),
	.HAS_GFX_BANK (1),               // godzilla: BG usa m_back_gfx_bank ($100470 bit14 -> +0x1000)
	.TILE_KIND    (3'd0),
	.MAP_HEIGHT_4 (0)
) u_bg (
	.clk(clk_sys), .reset(reset), .ce_pix(ce_pix),
	.hpos(hpos_for_read), .vpos(vpos_for_pf),
	.de(video_de), .layer_en(map_ctrl_l0[0] & ~status[30]),
	.new_line(layer_new_line),
	.scroll_x(bg_scroll_x), .scroll_y(bg_scroll_y),
	.xoff(osd_bg_xoff), .yoff(osd_bg_yoff),
	.gfx_bank(gfx_bank),
	.vram_addr(bg_vram_addr), .vram_data(bg_vram_data),
	.rom_req(arb_bg_req), .rom_addr(arb_bg_addr),
	.rom_data(arb_bg_data), .rom_valid(arb_bg_valid),
	.opaque(bg_opaque), .pen_index(bg_pen)
);

// SeibuCup MG: 16x16, 4bpp, 32x32 tilemap (legionna_v.cpp:204).
// get_mid_tile_info_share_bgrom (legionna_v.cpp:158): MG share BG ROM,
//   tile  = (vram & 0xfff) | 0x1000   (= seconda metà della BG ROM)
//   color = ((vram >> 12) & 0xf) | 0x10  (= palette set superiore: bit 4 = 1)
// In palette MAME "back" base = 0, 32 colorset → con color 4-bit (0..15) la
// metà alta dei 32 colorset (16..31) si raggiunge via "| 0x10".
// COLOR_BASE = 0 (back) + 0x10*16 = 0x100 (= colorset 16 × 16 pen).
// TILE_CODE_OFS = 0x1000 (seconda metà ROM back).
SeibuCup_tile_layer #(
	.COLOR_BASE    (11'h100),
	.HAS_TRANSP    (1),
	.HAS_GFX_BANK  (0),
	.TILE_KIND     (3'd1),
	.TILE_CODE_OFS (13'h1000)
) u_mg (
	.clk(clk_sys), .reset(reset), .ce_pix(ce_pix),
	.hpos(hpos_for_read), .vpos(vpos_for_pf),
	.de(video_de), .layer_en(map_ctrl_l0[1] & ~status[31]),
	.new_line(layer_new_line),
	.scroll_x(mg_scroll_x), .scroll_y(mg_scroll_y),
	.xoff(osd_mg_xoff), .yoff(osd_mg_yoff),
	.gfx_bank(16'd0),
	.vram_addr(mg_vram_addr), .vram_data(mg_vram_data),
	.rom_req(arb_mg_req), .rom_addr(arb_mg_addr),
	.rom_data(arb_mg_data), .rom_valid(arb_mg_valid),
	.opaque(mg_opaque), .pen_index(mg_pen)
);

// SeibuCup FG: tilemap 32x32 (legionna_v.cpp:213).
// get_fore_tile_info: tile = (vram & 0xfff), color = (vram >> 12). No +0x1000.
// MAME GFXDECODE "fore" color base = 32*16 = 0x200, 16 colorset.
// ROM "fore" è user1 1a metà (64KB), descrambled — fetch via sdram_bridge
// gfx_kind=3'd2 con address bitswap.
SeibuCup_tile_layer #(
	.COLOR_BASE    (11'h200),
	.HAS_TRANSP    (1),
	.HAS_GFX_BANK  (0),
	.TILE_KIND     (3'd2),
	.MAP_HEIGHT_4  (0),
	.TILE_CODE_OFS (13'h0000),
	// SEIBUCUP: fore = bg2.619 ROM_LOAD16_WORD_SWAP (legionna.cpp:1987) =
	// IDENTICO alla back bg1.618 → stesso decode DCBA di BG/MG (riscontro HW
	// utente: col BADC il FG era sbagliato). Il BADC era la calibrazione
	// Legionnaire, la cui fore veniva dalla user1 scrambled — non vale qui.
	.PEN_ORDER     (0)
) u_fg (
	.clk(clk_sys), .reset(reset), .ce_pix(ce_pix),
	.hpos(hpos_for_read), .vpos(vpos_for_pf),
	.de(video_de), .layer_en(map_ctrl_l0[2] & ~status[32]),  // SeibuCup: bit 2 = FG (legionna_v.cpp:344)
	.new_line(layer_new_line),
	.scroll_x(fg_scroll_x), .scroll_y(fg_scroll_y),
	.xoff(osd_fg_xoff), .yoff(osd_fg_yoff),
	.gfx_bank(16'd0),
	.vram_addr(fg_vram_addr), .vram_data(fg_vram_data),
	.rom_req(arb_fg_req), .rom_addr(arb_fg_addr),
	.rom_data(arb_fg_data), .rom_valid(arb_fg_valid),
	.opaque(fg_opaque), .pen_index(fg_pen)
);

// ── Sprite renderer (SEI252 / RISE, SeibuCup family) ─────────────────────
wire        spr_opaque;
wire [10:0] spr_pen;
wire  [1:0] spr_pri;
wire [10:0] spr_vram_addr_int;  // Sprite renderer: 512 entry × 4 word = 2048 word
wire [10:0] spr_vram_addr = spr_vram_addr_int;
wire [15:0] spr_vram_data;
wire        arb_spr_req;
wire [23:0] arb_spr_addr;
wire [31:0] arb_spr_data;
wire        arb_spr_valid;

// ── Sprite ROM su DDR3 (libera banda SDRAM ai layer tile) ───────────────────
// SeibuCup sprite = 6MB (obj1/2=2MB + obj3/4=1MB). MRA li carica a ioctl
// 0x320000..0x91FFFF. DDR3 base propria (separata dal layout SDRAM).
localparam [27:0] SPR_DDR3_BASE = 28'h0000000;  // base sprite in DDR3
wire [26:0] spr_dl_off = ioctl_addr - 27'h320000;
wire        spr_dl_sel = ioctl_download & (ioctl_index == 16'd0) &
                         (ioctl_addr >= 27'h320000) & (ioctl_addr < 27'h920000);

// DDR3 write (download): handshake we_req/we_ack toggle
reg  [27:0] spr_ddr_waddr;
reg  [15:0] spr_ddr_wdata;
reg         spr_ddr_we_req = 1'b0;
wire        spr_ddr_we_ack;
reg         spr_dl_wr_d = 1'b0;
always @(posedge clk_sys) begin
	spr_dl_wr_d <= ioctl_wr & spr_dl_sel;
	if (ioctl_wr & spr_dl_sel & ~spr_dl_wr_d) begin
		spr_ddr_waddr <= SPR_DDR3_BASE + {1'b0, spr_dl_off};
		spr_ddr_wdata <= ioctl_dout;
		spr_ddr_we_req <= ~spr_ddr_we_req;
	end
end

// DDR3 read (sprite fetch): bridge dal protocollo rom_req/rom_valid del renderer
reg  [27:0] spr_ddr_raddr;
reg         spr_ddr_rd_req = 1'b0;
wire        spr_ddr_rd_ack;
wire [31:0] spr_ddr_rdata;
reg  [1:0]  spr_rd_state = 2'd0;
reg         spr_rom_valid_r = 1'b0;
reg  [31:0] spr_rom_data_r;
always @(posedge clk_sys) begin
	spr_rom_valid_r <= 1'b0;
	case (spr_rd_state)
		2'd0: if (arb_spr_req) begin
			spr_ddr_raddr  <= SPR_DDR3_BASE + {4'd0, arb_spr_addr};
			spr_ddr_rd_req <= ~spr_ddr_rd_req;
			spr_rd_state   <= 2'd1;
		end
		2'd1: if (spr_ddr_rd_ack == spr_ddr_rd_req) begin
			spr_rom_data_r  <= spr_ddr_rdata;
			spr_rom_valid_r <= 1'b1;
			spr_rd_state    <= 2'd2;
		end
		2'd2: if (!arb_spr_req) spr_rd_state <= 2'd0;
	endcase
end
assign arb_spr_data  = spr_rom_data_r;
assign arb_spr_valid = spr_rom_valid_r;

ddram_sprite u_ddram_spr (
	.DDRAM_CLK(DDRAM_CLK), .DDRAM_BUSY(DDRAM_BUSY),
	.DDRAM_BURSTCNT(DDRAM_BURSTCNT), .DDRAM_ADDR(DDRAM_ADDR),
	.DDRAM_DOUT(DDRAM_DOUT), .DDRAM_DOUT_READY(DDRAM_DOUT_READY),
	.DDRAM_RD(DDRAM_RD), .DDRAM_DIN(DDRAM_DIN), .DDRAM_BE(DDRAM_BE), .DDRAM_WE(DDRAM_WE),
	.wraddr(spr_ddr_waddr), .din(spr_ddr_wdata), .we_req(spr_ddr_we_req), .we_ack(spr_ddr_we_ack),
	.rdaddr(spr_ddr_raddr), .dout(spr_ddr_rdata), .rd_req(spr_ddr_rd_req), .rd_ack(spr_ddr_rd_ack)
);

// Lookahead +1 pixel SOLO per il read-side sprite: campiona read_data/hpos/de
// @ce_pix (fix CRT spram latency-2, HW-ok: l'uscita cambia solo al tick), ma
// ogni stage gated @ce_pix costa 1 PIXEL reale (doc 07 MiSTer_Discovery) →
// sprite 1px dietro ai tile (lookup ungated) e colonna 0 MAI coperta (al tick
// del pixel 0 campionava hpos/de del blanking = "sprite tagliati a sinistra").
// Anticipando di 1 l'indirizzo e il DE, il dato campionato al tick e' quello
// del pixel CORRENTE: allineamento coi tile ripristinato (= calibrazione +1
// pixel-hunting MAME, fatta prima del fix CRT) e colonna 0 coperta. L'indirizzo
// resta stabile 14 clk prima del campionamento → zero transitori su CRT.
// In flip il pixel successivo dello schermo legge hpos-1 (mirror).
wire [9:0] hpos_for_spr = flip_screen ? (hpos_for_read - 10'd1)
                                      : (hpos_for_read + 10'd1);
wire       de_spr = (vpos_logic < 9'd240) && ((hpos_logic + 10'd1) < 10'd320);
SeibuCup_sprite_renderer u_spr (
	.clk(clk_sys), .reset(reset), .ce_pix(ce_pix),
	.hpos(hpos_for_spr), .vpos(vpos_logic),
	.de(de_spr), .layer_en(map_ctrl_l0[4] & ~status[33]),    // bit4 = sprite enable, OSD off
	.new_line(layer_new_line),
	.xoff(osd_spr_xoff), .yoff(osd_spr_yoff),
	.spr_addr(spr_vram_addr_int), .spr_data(spr_vram_data),
	.rom_req(arb_spr_req), .rom_addr(arb_spr_addr),
	.rom_data(arb_spr_data), .rom_valid(arb_spr_valid),
	.opaque(spr_opaque), .pen_index(spr_pen), .pri_code(spr_pri)
);

// ── Tile ROM arbiter (BG/MG/FG su SDRAM; sprite su DDR3 = r3 staccato) ──────
tile_rom_arbiter u_arb (
	.clk(clk_sys), .reset(reset), .hblank(HBlank),
	.r0_req(arb_bg_req),  .r0_addr(arb_bg_addr),  .r0_data(arb_bg_data),  .r0_valid(arb_bg_valid),
	.r1_req(arb_mg_req),  .r1_addr(arb_mg_addr),  .r1_data(arb_mg_data),  .r1_valid(arb_mg_valid),
	.r2_req(arb_fg_req),  .r2_addr(arb_fg_addr),  .r2_data(arb_fg_data),  .r2_valid(arb_fg_valid),
	.r3_req(1'b0), .r3_addr(24'd0), .r3_data(), .r3_valid(),   // sprite ora su DDR3
	.r4_req(1'b0), .r4_addr(24'd0), .r4_data(), .r4_valid(),
	.tile_req(game_tile_req), .tile_addr(game_tile_addr), .tile_kind(game_tile_kind),
	.tile_data(game_tile_data), .tile_valid(game_tile_valid)
);

// Pixel pipeline SeibuCup (4 layer: BG/MG/FG/Text + sprite, 4 priority).
//
// MAME riferimenti (VIDEO_START cupsoc, legionna_v.cpp:276-287):
//   m_sprite_pri_mask per cupsoc:
//     pri=0 → 0xFFF0  (copre pri-value 0,1,2 → bg,mg,fg → SOTTO text)
//     pri=1 → 0xFFFC  (copre 0,1 → bg,mg → sotto fg,text)
//     pri=2 → 0xFFFE  (copre 0 → bg → sotto mg,fg,text)
//     pri=3 → 0x0000  (copre TUTTO, text incluso → "Insert coin")
//   screen_update_cupsoc draw order (back→front):
//     BG (priority code 0) → MG (1) → FG (2) → Text (4)
//   pri level = cupsoc_pri_cb = {w1[15], w0[6]} (calcolato nel renderer).
//
// Ordine composite RTL (front→back), derivato dalle pri_mask sopra:
//   Sprite pri=3  (sopra tutto, text incluso)
//   Text          (pri-value 4)
//   Sprite pri=0  (sopra bg/mg/fg, sotto text)
//   FG            (pri-value 2)
//   Sprite pri=1  (sopra bg/mg)
//   MG            (pri-value 1)
//   Sprite pri=2  (sopra bg)
//   BG            (pri-value 0, disegnato per primo = in fondo)
//   Backdrop      (bitmap.fill(black_pen) → NERO, legionna_v.cpp:401)
wire spr_pri0 = spr_opaque & (spr_pri == 2'd0);
wire spr_pri1 = spr_opaque & (spr_pri == 2'd1);
wire spr_pri2 = spr_opaque & (spr_pri == 2'd2);
wire spr_pri3 = spr_opaque & (spr_pri == 2'd3);

wire no_layer = ~(spr_pri3 | text_opaque | spr_pri0 | fg_opaque |
                  spr_pri1 | mg_opaque | spr_pri2 | bg_opaque);
wire [10:0] pal_b_addr_c = spr_pri3     ? spr_pen  :
                           text_opaque  ? text_pen :
                           spr_pri0     ? spr_pen  :
                           fg_opaque    ? fg_pen   :
                           spr_pri1     ? spr_pen  :
                           mg_opaque    ? mg_pen   :
                           spr_pri2     ? spr_pen  :
                                          bg_pen;
// Register intermediate per ridurre path depth tra layer opaque/pen e palette
// port B addr (era 9 levels comb, ora 1 register stage). Aggiunge 1 ciclo di
// latency che è invisibile a 16 clk/ce_pix.
reg [10:0] pal_b_addr_r;
reg        no_layer_r, no_layer_rr;   // allineato alla latenza pal_b_addr_r + BRAM
always @(posedge clk_sys) begin
	pal_b_addr_r <= pal_b_addr_c;
	no_layer_r   <= no_layer;
	no_layer_rr  <= no_layer_r;
end
assign pal_b_addr = pal_b_addr_r;

reg video_de_r;
always @(posedge clk_sys) begin
	video_de_r <= video_de;
end
// Backdrop cupsoc = black_pen (screen_update_cupsoc bitmap.fill(black)):
// quando nessun layer e' opaco forziamo RGB=0 (no_layer_rr allineato a pal_b_*).
wire [7:0] video_r = (video_de_r & ~no_layer_rr) ? pal_b_r : 8'h00;
wire [7:0] video_g = (video_de_r & ~no_layer_rr) ? pal_b_g : 8'h00;
wire [7:0] video_b = (video_de_r & ~no_layer_rr) ? pal_b_b : 8'h00;

assign CLK_VIDEO = clk_sys;
assign CE_PIXEL  = ce_pix;

// Pause overlay: dim video + logo + SUPPORTERS + patron scroll.
// Modulo standalone 8-bit RGB. OSD "Clean Pause" (status[18]): ON=raw, OFF=overlay.
// Output su bus intermedi av_r/av_g/av_b (NON piu` diretto ai pin VGA_R/G/B):
// il CRT Adjust si inserisce fra questo overlay e i pin.
wire [7:0] av_r, av_g, av_b;
pause_overlay u_pause_ovl (
	.clk       (clk_sys),
	.pause     (pause),
	.clean     (status[18]),
	.vblank    (VBlank),
	.render_x  (render_x[8:0]),
	.render_y  (render_y),
	.rgb_r_in  (video_r),
	.rgb_g_in  (video_g),
	.rgb_b_in  (video_b),
	.rgb_r_out (av_r),
	.rgb_g_out (av_g),
	.rgb_b_out (av_b)
);

// ── CRT Adjust + V-Size: decode dell'OSD (variante SYS-SIDE) ────────────────
// I due moduli vivono in sys/ fra scanlines e vga_osd: qui restano solo il
// decode dell'OSD e il VBlank vero, esportati con le porte CRT_*. Cosi' l'HDMI
// resta bit-identico mentre si regola il CRT, ed e' il motivo per cui questa
// variante esiste. Geometria ANCORATA come Legionnaire (margini 88 sx / 28 dx,
// NON centrata) -> HPOS_MODE 0 (SYNCSHIFT), impostato in sys_top.

// ON/OFF (status[101]): OFF = bypass nativo, ON = modulo attivo.
reg crt_on;
always @(posedge clk_sys) if (ce_pix) crt_on <= status[101];

// H-Size bidirezionale (status[100:96], two's complement 5-bit).
reg signed [4:0] hsize_s;
always @(posedge clk_sys) if (ce_pix) hsize_s <= $signed(status[100:96]);

// H-Position (status[85:79], 7 bit): 0..48 = +0..+48 (destra), poi i negativi.
reg [6:0] hpos_d;
always @(posedge clk_sys) if (ce_pix) hpos_d <= status[85:79];
// Il menu salva l'INDICE nella lista: 97 voci (0, +1..+48, -48..-1). Il wrap va
// fatto sulla LUNGHEZZA DELLA LISTA: con -128 la voce "-1" valeva -32 px e tutto
// il lato negativo era spostato di 31 (bug del doc 17, gia' corretto su NS).
wire signed [8:0] hpos_off = (hpos_d <= 7'd48)
	? $signed({2'b0, hpos_d})
	: $signed({2'b0, hpos_d}) - 9'sd97;

// V-Shift (status[78:74], signed 5-bit -16..+15 righe).
reg signed [5:0] vshift_off;
always @(posedge clk_sys) if (ce_pix) vshift_off <= $signed(status[78:74]);

// ── CRT V-Size: decode dell'OSD, il modulo sta in sys/ a monte di crt_adjust_sys ─
// Ritempora il numero di righe per frame (il frame rate NON cambia): meno
// righe = riga piu' lunga = immagine PIU' ALTA. Nessuna riga duplicata o
// scartata. Il costo e' che l'HSync si sposta: un PVM segue, un chassis da
// cabinato con AFC stretta regge uno o due passi.
// Convenzione del modulo: vsize +N = immagine piu' BASSA. La negazione qui
// sotto fa si' che "+" nell'OSD significhi piu' ALTA, come per l'H-Size.
reg signed [5:0] crt_vsz;
reg              crt_vsmode;
// Anche qui il menu salva l'INDICE: 15 voci (0, +1..+7, -7..-1), wrap su 15.
// Con $signed sui 4 bit la voce "-7" valeva -8 e tutto il lato negativo era
// spostato di uno scatto, cioe' 3 righe.
wire signed [5:0] crt_vsz_step = (status[107:104] <= 4'd7)
	? $signed({2'b0, status[107:104]})
	: $signed({2'b0, status[107:104]}) - 6'sd15;
// Passo diverso nei due modi, come su Night Slashers. In PVM il tetto e' basso
// (misurato al banco: il quadro d'uscita arriva a 269 righe, cioe' +7 dal
// nativo) e con 3 righe per scatto se ne usavano DUE prima che la rampa
// rientrasse. A 1 riga per scatto la lista +-7 dell'OSD e' usabile per intero.
// In Cabinet il tetto di frequenza non esiste e il passo grosso ha senso.
wire signed [7:0] vsz_ext = {{2{crt_vsz_step[5]}}, crt_vsz_step};
wire signed [7:0] vsz_raw = status[108] ? -(vsz_ext + (vsz_ext <<< 1))  // Cabinet: 3 righe/scatto
                                        : -vsz_ext;                     // PVM:     1 riga/scatto

// ⛔ TETTO DELL'ENLARGE = front porch verticale di QUESTO gioco (V_FP = 9).
// Allargando, il modulo emette meno righe e il contenuto scende: oltre le
// righe di front porch le ultime dell'immagine non vengono piu' emesse.
// SeibuCup ha 240 righe attive dentro 262 totali, cioe' 22 di blanking:
// molto meno di quanto l'OSD offra (passi da 3 fino a 21).
localparam signed [7:0] ENL_MAX = 8'sd9;   // = V_FP del video_timing

// Tetto dello SHRINK: lo impone la profondita' del ring, |vsize| <= RING/2 - 2,
// cioe' 10 righe con RING_LINES 24 (sys_top). Vale per ENTRAMBI i modi, e deve
// restare allineato a quel parametro: un OSD che promette scatti oltre la
// capienza del ring e' proprio la trappola descritta nei doc.
// Non e' una perdita: misurato al banco, il quadro d'uscita in PVM sta fra 252
// e 269 righe (-10/+7 dal nativo), quindi oltre 10 non ci si arriva comunque;
// e l'enlarge e' gia' fermo a 9 per il front porch verticale.
wire signed [7:0] vsz_clamped = (vsz_raw >  8'sd10)  ?  8'sd10
                              : (vsz_raw < -ENL_MAX) ? -ENL_MAX : vsz_raw;
always @(posedge clk_sys) if (ce_pix) begin
	crt_vsz    <= vsz_clamped[5:0];
	crt_vsmode <= status[108];
end

// Valori esportati a sys_top: i moduli stanno in sys/, qui resta il decode.
assign CRT_ON     = crt_on;
assign CRT_HSIZE  = hsize_s;
assign CRT_HPOS   = hpos_off;
assign CRT_VSHIFT = vshift_off;
assign CRT_VSIZE  = crt_vsz;
assign CRT_VSMODE = crt_vsmode;
assign CRT_VBL    = VBlank;        // VBlank VERO nativo, mai il blank combinato

// Uscite native: la regolazione avviene in sys_top, a valle.
assign VGA_R  = av_r;
assign VGA_G  = av_g;
assign VGA_B  = av_b;
assign VGA_HS = HSync;
assign VGA_VS = VSync;

// Aspect ratio: Original = 4:3 arcade display, Full Screen = 0:0
wire [11:0] arx = (!ar) ? 12'd4 : (ar - 1'd1);
wire [11:0] ary = (!ar) ? 12'd3 : 12'd0;

// Integer scaling forzato: Narrower HV-Integer (default), V-Integer, HV-Integer.
// Normal scaling rimosso perché senza setup utente preciso dà sempre risultato sbagliato.
video_freak video_freak
(
	.CLK_VIDEO(clk_sys),
	.CE_PIXEL(ce_pix),
	.VGA_VS(VSync),
	.HDMI_WIDTH(HDMI_WIDTH),
	.HDMI_HEIGHT(HDMI_HEIGHT),
	.VGA_DE(VGA_DE),
	.VIDEO_ARX(VIDEO_ARX),
	.VIDEO_ARY(VIDEO_ARY),
	.VGA_DE_IN(~(HBlank | VBlank)),
	.ARX(arx),
	.ARY(ary),
	.CROP_SIZE(12'd0),
	.CROP_OFF(5'd0),
	.SCALE(status[7:5])    // 0=Normal,1=V-Int,2=Narrower,3=Wider,4=HV-Integer
);

// LED: blink during download
assign LED_USER = ioctl_download;

// ============================================================
// JTAG Debug Probes (readable via quartus_stp / System Console)
// ============================================================
// JTAG boot trace removed to save M10K for 64KB work RAM

endmodule

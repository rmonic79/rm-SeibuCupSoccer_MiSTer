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

/*  Main CPU memory map SeibuCup / SD Gundam (legionna.cpp cupsoc_map):
      $000000-$0FFFFF  ROM 68K (1MB)                → SDRAM via bridge
      $100000-$1003FF  RAM scratch (1KB)            → BRAM (Main RAM low)
      $100400-$1006FF  COP3 region                  → modulo cop3
      $100600-$10064F  Seibu CRTC                   → modulo CRTC
      $100680-$100681  IRQ ack                      → nopw
      $100700-$10071F  Seibu sound comm             → snd_cs (umask 0x00FF)
      $100740-$100741  DSW1                         → input
      $100744-$100745  P1+P2                        → input
      $100748-$100749  P3+P4                        → input
      $10074C-$10074D  System                       → input
      $10075C-$10075D  DSW2                         → input
      $100800-$100FFF  BG VRAM                      → BRAM
      $101000-$1017FF  FG VRAM                      → BRAM
      $101800-$101FFF  MG VRAM                      → BRAM
      $102000-$102FFF  Text VRAM                    → BRAM
      $103000-$103FFF  Palette LIVE (fade dst)      → BRAM (pallive)
      $104000-$104FFF  Palette STAGING (fade src)   → BRAM (pal)
      $105000-$106FFF  RAM filler                   → BRAM (Main RAM)
      $107000-$107FFF  Sprite RAM                   → BRAM
      $108000-$11FFFF  Main RAM (96KB)              → BRAM
*/

module SeibuCup_maincpu_map (
	input  wire        clk,
	input  wire        reset,
	// CPU bus
	input  wire [23:0] bus_addr,
	input  wire        bus_asn,
	input  wire        bus_rnw,
	input  wire  [1:0] bus_dsn,
	input  wire [15:0] bus_wdata,
	output reg  [15:0] bus_rdata,
	output reg         bus_cs,
	output reg         bus_busy,
	// Inputs
	input  wire  [7:0] p1_input,
	input  wire  [7:0] p2_input,
	input  wire [15:0] pin34_input,   // PLAYERS34: P3+P4, con COIN3/COIN4
	input  wire [15:0] system_input,
	input  wire [15:0] dsw_input,
	input  wire [15:0] dsw2_input,
	input  wire  [2:0] osd_start_phase,   // OSD: 0=Normal, 1..3 = parti da fase N (3=stage 4 finale)
	// Main ROM (SDRAM bridge)
	input  wire [15:0] main_rom_rdata,
	input  wire        main_rom_ready,
	output wire [23:0] main_rom_addr,
	output wire        main_rom_req,
	// Main RAM (128KB unified)
	input  wire [15:0] ram_rdata,
	output wire        ram_we,
	output wire [15:0] ram_addr,
	output wire [15:0] ram_wdata,
	output wire  [1:0] ram_be,
	// VRAM
	input  wire [15:0] bg_rdata,
	output wire        bg_we,
	input  wire [15:0] mg_rdata,
	output wire        mg_we,
	input  wire [15:0] fg_rdata,
	output wire        fg_we,
	input  wire [15:0] txt_rdata,
	output wire        txt_we,
	input  wire [15:0] pal_rdata,
	output wire        pal_we,
	input  wire [15:0] pallive_rdata,
	output wire        pallive_we,
	input  wire [15:0] spr_rdata,
	output wire        spr_we,
	output wire [10:0] vram_addr,
	output wire [15:0] vram_wdata,
	output wire  [1:0] vram_be,
	// CRTC
	output wire        crtc_cs,
	output wire        crtc_wr,
	output wire        crtc_rd,
	output wire  [6:1] crtc_addr,
	input  wire [15:0] crtc_rdata,
	// GFX bank
	output reg  [15:0] gfx_bank,
	// Sound comm
	output wire        snd_cs,
	output wire        snd_wr,
	output wire        snd_rd,
	output wire  [4:1] snd_addr,
	output wire [15:0] snd_wdata,
	input  wire [15:0] snd_rdata,
	// COP3
	output wire        cop_cs,
	output wire        cop_wr,
	output wire        cop_rd,
	output wire [10:1] cop_addr,
	output wire [15:0] cop_wdata,
	input  wire [15:0] cop_rdata,
	input  wire        cop_busy
);

	// ─── Region detect (MAME cupsoc_map, legionna.cpp:324) ─────────────────
	wire bus_active = ~bus_asn;
	wire is_rom     = bus_active && (bus_addr <  24'h100000);   // cupsoc: 1MB program
	wire is_ram_lo  = bus_active && (bus_addr >= 24'h100000) && (bus_addr < 24'h100400);
	wire is_crtc    = bus_active && (bus_addr >= 24'h100600) && (bus_addr < 24'h100650);
	wire is_irqack  = bus_active && (bus_addr >= 24'h100680) && (bus_addr < 24'h100682);
	wire is_cop     = bus_active && (bus_addr >= 24'h100400) && (bus_addr < 24'h100700)
	                  && !is_crtc && !is_irqack;
	wire is_snd     = bus_active && (bus_addr >= 24'h100700) && (bus_addr < 24'h100720);
	wire is_dip     = bus_active && (bus_addr >= 24'h100740) && (bus_addr < 24'h100744);
	wire is_pin     = bus_active && (bus_addr >= 24'h100744) && (bus_addr < 24'h100748);
	wire is_pin34   = bus_active && (bus_addr >= 24'h100748) && (bus_addr < 24'h10074C);
	wire is_sin     = bus_active && (bus_addr >= 24'h10074C) && (bus_addr < 24'h100750);
	wire is_dsw2    = bus_active && (bus_addr >= 24'h10075C) && (bus_addr < 24'h10075E);
	// VRAM cupsoc: layout shiftato -0x800 vs legionna (= layout heatbrl).
	wire is_bg      = bus_active && (bus_addr >= 24'h100800) && (bus_addr < 24'h101000);
	wire is_fg      = bus_active && (bus_addr >= 24'h101000) && (bus_addr < 24'h101800);
	wire is_mg      = bus_active && (bus_addr >= 24'h101800) && (bus_addr < 24'h102000);
	// Text VRAM cupsoc: $102000-$102FFF = 4KB = 2048 word, allineata.
	wire is_text    = bus_active && (bus_addr >= 24'h102000) && (bus_addr < 24'h103000);
	// Palette cupsoc a DOPPIO buffer (disasm $1C14/$224E):
	//   $103000-$103FFF = palette LIVE (dst del fade DMA, letta dal renderer)
	//   $104000-$104FFF = palette STAGING (CPU scrive qui, src del fade DMA)
	// La staging riusa la pal BRAM esistente (stesso indirizzo di godzilla);
	// la live e' una BRAM nuova (pallive_*).
	wire is_pallive = bus_active && (bus_addr >= 24'h103000) && (bus_addr < 24'h104000);
	wire is_pal     = bus_active && (bus_addr >= 24'h104000) && (bus_addr < 24'h105000);
	// CUPSOC: spriteram e' META' di grainbow — $107000-$1077FF (2KB = 256 entry)
	// e sopra c'e' la "Ani Dsp(?) Ram" $107800-$107FFF (RAM normale, MAME
	// cupsoc_map). Scansionare anche quella darebbe sprite fantasma.
	wire is_spr     = bus_active && (bus_addr >= 24'h107000) && (bus_addr < 24'h107800);
	wire is_rampad  = bus_active && ((bus_addr >= 24'h105000) && (bus_addr < 24'h107000)
	                             ||  (bus_addr >= 24'h107800) && (bus_addr < 24'h108000));
	wire is_ram_hi  = bus_active && (bus_addr >= 24'h108000) && (bus_addr < 24'h120000);

	wire is_ram     = is_ram_lo | is_ram_hi | is_rampad;
	wire is_bram    = is_ram | is_bg | is_fg | is_mg | is_text | is_pal | is_pallive | is_spr;
	wire is_io      = is_dip | is_pin | is_pin34 | is_sin | is_dsw2;

	// ─── ROM access (SDRAM bridge) ───────────────────────────────────────────
	assign main_rom_addr = {1'b0, bus_addr[22:1], 1'b0};
	assign main_rom_req  = is_rom & bus_rnw;

	// ─── BRAM common write/byte-enable ──────────────────────────────────────
	wire write = bus_active & ~bus_rnw & (|(~bus_dsn));
	// VRAM word-offset locale per regione. Le 6 BRAM sono separate ma tutte
	// indirizzate dallo stesso wire vram_addr (mux esterno via *_we). Quindi
	// l'indirizzo deve essere il word offset DENTRO la regione attiva, non
	// bus_addr[11:1] grezzo (che dava disallineamenti per FG/Text/Palette/Spr
	// rispetto al lato renderer che parte sempre da addr 0).
	//   BG  $101000-$1017FF (2KB)  → offset = bus_addr - 0x101000 → [10:1]
	//   FG  $101800-$101FFF (2KB)  → offset = bus_addr - 0x101800 → [10:1]
	//   MG  $102000-$1027FF (2KB)  → offset = bus_addr - 0x102000 → [10:1]
	//   TXT $102800-$1037FF (4KB)  → offset = bus_addr - 0x102800 → [11:1]
	//   PAL $104000-$104FFF (4KB)  → offset = bus_addr - 0x104000 → [11:1]
	//   SPR $105000-$105FFF (4KB)  → offset = bus_addr - 0x105000 → [11:1]
	// Ogni layer BRAM riceve l'OFFSET LOCALE word dentro la sua region.
	// Game scrive in CPU bus_addr range diverse, ma ogni BRAM è indicizzata 0-based.
	//   BG  $100800 base → offset = (bus_addr - $100800)/2 → bus_addr[10:1]
	//   FG  $101000 base → offset = (bus_addr - $101000)/2 → bus_addr[10:1]
	//   MG  $101800 base → offset = (bus_addr - $101800)/2 → bus_addr[10:1]
	//   TXT $102000 base → offset = (bus_addr - $102000)/2 → bus_addr[11:1] (allineata)
	//   PALLIVE $103000  → offset = (bus_addr - $103000)/2 → bus_addr[11:1]
	//   PAL $104000 base → offset = (bus_addr - $104000)/2 → bus_addr[11:1]
	//   SPR $107000 base → offset = (bus_addr - $107000)/2 → bus_addr[11:1]
	// bus_addr[10:1] strippa automaticamente i bit alti che differenziano la region.
	reg [10:0] vram_addr_r;
	always @(*) begin
		if      (is_bg)   vram_addr_r = {1'b0, bus_addr[10:1]};
		else if (is_fg)   vram_addr_r = {1'b0, bus_addr[10:1]};
		else if (is_mg)   vram_addr_r = {1'b0, bus_addr[10:1]};
		else if (is_text) vram_addr_r = bus_addr[11:1];
		else if (is_pal)  vram_addr_r = bus_addr[11:1];
		else if (is_spr)  vram_addr_r = {1'b0, bus_addr[10:1]};   // 2KB (cupsoc)
		else              vram_addr_r = bus_addr[11:1];   // pallive incluso
	end
	assign vram_addr  = vram_addr_r;
	assign vram_wdata = bus_wdata;
	assign vram_be    = ~bus_dsn;
	assign ram_addr   = bus_addr[16:1];   // 64Kw = 128KB Main RAM unified
	assign ram_wdata  = bus_wdata;
	assign ram_be     = ~bus_dsn;
	assign ram_we     = is_ram  & write;
	assign bg_we      = is_bg   & write;
	assign mg_we      = is_mg   & write;
	assign fg_we      = is_fg   & write;
	assign txt_we     = is_text & write;
	assign pal_we     = is_pal  & write;
	assign pallive_we = is_pallive & write;
	assign spr_we     = is_spr  & write;

	// ─── CRTC ────────────────────────────────────────────────────────────────
	assign crtc_cs    = is_crtc;
	assign crtc_wr    = is_crtc & write;
	assign crtc_rd    = is_crtc & bus_rnw;
	assign crtc_addr  = bus_addr[6:1];

	// ─── COP3 ────────────────────────────────────────────────────────────────
	assign cop_cs    = is_cop;
	assign cop_wr    = is_cop & write;
	assign cop_rd    = is_cop & bus_rnw;
	assign cop_addr  = bus_addr[10:1] - 10'h200;  // 0x100400 → 0
	assign cop_wdata = bus_wdata;

	// ─── Sound comm ──────────────────────────────────────────────────────────
	assign snd_cs    = is_snd;
	assign snd_wr    = is_snd & write;
	assign snd_rd    = is_snd & bus_rnw;
	// MAME legionna_map: main_r/main_w(offset>>1) con umask16(0x00ff). offset =
	// (byte-0x100700)/2 (word index), poi >>1 -> registro seibu = (byte-0x100700)/4
	// = bus_addr[4:2]. Il vecchio bus_addr[4:1] mancava il >>1 -> ogni reg sound
	// tranne 0 era misaddressato -> coin mai letta -> $3634 mai eseguito -> BG neri.
	// Vedi MAME seibusound.cpp main_r/main_w.
	assign snd_addr  = {1'b0, bus_addr[4:2]};
	assign snd_wdata = bus_wdata;

	// ─── GFX bank ────────────────────────────────────────────────────────────
	// cupsoc non ha il tile bank register ($100470 assente dalla mappa MAME):
	// gfx_bank resta 0. $100480-$48F (layer_config/abs limiter) cade nella
	// finestra COP e viene gestito/ignorato dal cop3.
	always @(posedge clk) begin
		if (reset) gfx_bank <= 16'd0;
	end

	// ─── FSM TXN per gestire latenza BRAM ───────────────────────────────────
	localparam S_IDLE = 2'd0;
	localparam S_WAIT = 2'd1;
	localparam S_DONE = 2'd2;

	reg [1:0] state;

	always @(posedge clk) begin
		if (reset) begin
			state <= S_IDLE;
		end else begin
			case (state)
				S_IDLE: if (is_bram) state <= S_WAIT;
				S_WAIT:              state <= S_DONE;
				S_DONE: if (~bus_active) state <= S_IDLE;
				default: state <= S_IDLE;
			endcase
		end
	end

	// ─── Read mux + bus_cs / bus_busy (pattern DCon, S_DONE gated) ──────────
	always @(*) begin
		bus_rdata = 16'hFFFF;
		bus_cs    = 1'b0;
		bus_busy  = 1'b0;

		if (is_rom) begin
			bus_rdata = main_rom_rdata;
			bus_cs    = 1'b1;
			// DEADLOCK su WRITE in ROM: main_rom_req parte solo in lettura
			// (assign sopra, is_rom & bus_rnw), quindi su una write in ROM
			// main_rom_ready non arriva MAI e con bus_busy = ~ready la CPU
			// resta bloccata per sempre = FREEZE. Sul PCB la write in ROM e'
			// semplicemente persa e il 68000 prosegue.
			// Trovato su Denjin Makai (sim Verilator, write a $000000 al
			// frame 72) e latente in tutta la famiglia legionna.
			bus_busy  = bus_rnw ? ~main_rom_ready : 1'b0;
			// ─── START PHASE da OSD (debug) ──────────────────────────────────
			// Init partita $2F3A: move.w $1080E8,d0 / move.w d0,$1086D2 (fase).
			// Fase 0-2 = giri missioni (le 3 missioni si scelgono dal mission
			// select in-game), fase 3 = stage 4 finale (indice tabelle $225B8,
			// check $21EE2). Patch in lettura: moveq #N,d0 + nop + nop.
			// ROM su disco intatta.
			if (osd_start_phase != 3'd0) begin
				if (bus_addr[22:1] == 22'h0179D) bus_rdata = {8'h70, 5'd0, osd_start_phase};  // moveq #N,d0 ($70 0N)
				if (bus_addr[22:1] == 22'h0179E) bus_rdata = 16'h4E71;  // nop
				if (bus_addr[22:1] == 22'h0179F) bus_rdata = 16'h4E71;  // nop
				// ROM check del POST ($2C68: somma long $0-$2AFFF == 0, cmpi a
				// $2C7C, bne->NG a $2C82): la patch sopra altera la somma, e
				// compensare l'immediato del cmpi e' AUTO-REFERENZIALE (anche
				// l'immediato sta nell'area sommata — non ha soluzione).
				// Si neutralizza il branch NG: bne.w $2CEA -> nop nop.
				if (bus_addr[22:1] == 22'h01641) bus_rdata = 16'h4E71;  // era $6600 (bne.w)
				if (bus_addr[22:1] == 22'h01642) bus_rdata = 16'h4E71;  // era $0066 (disp)
			end
		end
		else if (is_bram) begin
			if (state == S_DONE) begin
				bus_cs = 1'b1;
				if (is_ram)          bus_rdata = ram_rdata;
				else if (is_bg)      bus_rdata = bg_rdata;
				else if (is_fg)      bus_rdata = fg_rdata;
				else if (is_mg)      bus_rdata = mg_rdata;
				else if (is_text)    bus_rdata = txt_rdata;
				else if (is_pal)     bus_rdata = pal_rdata;
				else if (is_pallive) bus_rdata = pallive_rdata;
				else /* is_spr */    bus_rdata = spr_rdata;
			end else begin
				bus_busy = 1'b1;
			end
		end
		else if (is_snd) begin
			bus_rdata = snd_rdata;
			bus_cs    = 1'b1;
		end
		else if (is_crtc) begin
			bus_rdata = crtc_rdata;
			bus_cs    = 1'b1;
		end
		else if (is_dip) begin
			bus_rdata = dsw_input;
			bus_cs    = 1'b1;
		end
		else if (is_pin) begin
			bus_rdata = {p2_input, p1_input};
			bus_cs    = 1'b1;
		end
		else if (is_pin34) begin
			bus_rdata = pin34_input;
			bus_cs    = 1'b1;
		end
		else if (is_sin) begin
			bus_rdata = system_input;
			bus_cs    = 1'b1;
		end
		else if (is_dsw2) begin
			bus_rdata = dsw2_input;
			bus_cs    = 1'b1;
		end
		else if (is_cop) begin
			bus_rdata = cop_rdata;
			bus_cs    = 1'b1;
			// La READ COP stalla finche' il COP ha committato (cop_busy basso).
			// Replica la latenza-zero-osservabile di MAME (cop_cmd_w sincrono): il
			// gioco legge cop_hit_val_stat/cop_angle ($188/$1b4) nell'istruzione dopo
			// il trigger. Senza stall la CPU legge un valore RESIDUO del frame prima
			// -> bit7 in-range sbagliato ($497A) -> clr.l $48 ($7632) non scatta ->
			// la scivolata del bruto non si ferma e torna indietro.
			bus_busy  = cop_busy & bus_rnw;
		end
		else if (is_irqack) begin
			bus_rdata = 16'hFFFF;
			bus_cs    = 1'b1;
		end
		else if (bus_active) begin
			bus_rdata = 16'hFFFF;
			bus_cs    = 1'b1;
		end
	end

endmodule

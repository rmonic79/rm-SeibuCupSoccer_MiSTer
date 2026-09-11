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

// sdram_bridge — Bridge e arbitro SDRAM per SeibuCup.
// Multiplexa 3 client sul controller Genesis 4-port di Sorgelig:
//   Port 0: Tile/Sprite/Text ROM reads + ROM download writes
//   Port 1: Main CPU ROM
//   Port 2: Sub CPU ROM
//   Port 3: unused (audio Z80 ROM lives in BRAM, not SDRAM)
//
// Clients use ACTIVE-LOW level protocol:
//   - CPU raises req_n LOW (active) when it needs data
//   - Bridge translates to Genesis toggle protocol (req XOR → new request)
//   - Bridge holds dtack_n HIGH until SDRAM responds
//   - Bridge pulses dtack_n LOW for one cycle when data arrives
//
// The Genesis controller (sdram.sv by Sorgelig) uses toggle protocol:
//   - Toggle req to start a request
//   - ack matches req when data is ready
//   - dout is shared across all 3 ports (active port has valid data)
//
// SDRAM address map SeibuCup (word address dentro il banco [21:0]; ogni word
// del download e' quadruplicata nei 4 banchi, vedi nota finestra 8MB sotto):
//   0x000000-0x07FFFF: MAIN 68k  1MB   (byte 0x000000, 512KB reali + pad)
//   0x080000-0x08FFFF: CHAR/TXT  128KB (byte 0x100000, copia; il text usa BRAM)
//   0x090000-0x10FFFF: BG (+MG)  1MB   (byte 0x120000)
//   0x110000-0x18FFFF: FG        1MB   (byte 0x220000)
//   0x190000-0x1CFFFF: OKI       512KB (byte 0x320000, rimappata da ioctl 0x920000)
//   Sprite→DDR3, Z80→BRAM audio, COPX non usata (skip nel download).
//
// During ioctl_download, port 0 is used for ROM writes.
// After download completes, port 0 serves tile reads.

module sdram_bridge (
	input         clk,       // system clock (48+ MHz)
	input         reset,
	input         sdram_ready, // high when SDRAM controller init complete

	// ==================== Download from HPS ====================
	input         ioctl_download,
	input         ioctl_wr,
	input  [26:0] ioctl_addr,
	input  [15:0] ioctl_dout,    // 16-bit: WIDE=1
	input  [15:0] ioctl_index,
	output        ioctl_wait,

	// ==================== Video / Tile ROM (port 0) ====================
	// 32-bit tile data assembled from 2× 16-bit reads
	input  [23:0] tile_byte_addr,  // byte address relativo alla region selezionata
	input         tile_req,        // level: HIGH while data needed
	input   [2:0] gfx_kind,        // 0=BG, 1=MG, 2=FG, 3=SPR, 4=TXT
	output [31:0] tile_data,       // 32-bit assembled {hi_word, lo_word}
	output reg    tile_valid,      // pulse HIGH for 1 cycle when ready

	// ==================== Main CPU ROM (port 1) ====================
	input  [23:0] main_byte_addr,  // byte address from maincpu_map
	input         main_req,        // level: HIGH while data needed
	output [15:0] main_data,       // 16-bit read data
	output reg    main_ready,      // pulse HIGH for 1 cycle when ready

	// ==================== Sub CPU ROM (port 2) — UNUSED in Gundam ====================
	// Tied off: Gundam ha 1 sola CPU 68k. Mantengo i nomi per non rompere
	// l'istanziazione esistente in SeibuCup.sv ma li ignoro internamente.
	input  [23:0] sub_byte_addr,
	input         sub_req,
	output [15:0] sub_data,
	output reg    sub_ready,

	// ==================== OKI ADPCM ROM (port 3) ====================
	// jt6295 byte-level read polling: oki_byte_addr 19-bit (512KB godzilla,
	// bit18 = bank da IO port 0 Z80), oki_data 8-bit, oki_ok=1 quando data
	// corrisponde all'ultimo addr richiesto.
	input  [18:0] oki_byte_addr,
	output reg [7:0] oki_data,
	output reg    oki_ok,

	// ==================== Genesis SDRAM controller ports ====================
	// Port 0 (tile/download)
	output reg [24:1] sdram_addr0,
	output reg [15:0] sdram_din0,
	output reg        sdram_wrl0,
	output reg        sdram_wrh0,
	output reg        sdram_req0,
	input             sdram_ack0,
	input      [15:0] sdram_dout0,

	// Port 1 (main CPU)
	output reg [24:1] sdram_addr1,
	output     [15:0] sdram_din1,
	output             sdram_wrl1,
	output             sdram_wrh1,
	output reg        sdram_req1,
	input             sdram_ack1,
	input      [15:0] sdram_dout1,

	// Port 2 (sub CPU)
	output reg [24:1] sdram_addr2,
	output     [15:0] sdram_din2,
	output             sdram_wrl2,
	output             sdram_wrh2,
	output reg        sdram_req2,
	input             sdram_ack2,
	input      [15:0] sdram_dout2,

	// Port 3: OKI ADPCM ROM (jt6295)
	output reg  [24:1] sdram_addr3,
	output wire [15:0] sdram_din3,
	output wire        sdram_wrl3,
	output wire        sdram_wrh3,
	output reg         sdram_req3,
	input              sdram_ack3,
	input       [15:0] sdram_dout3
);

// CPU/Audio ROM ports are read-only
assign sdram_din1 = 16'd0;
assign sdram_wrl1 = 1'b0;
assign sdram_wrh1 = 1'b0;
assign sdram_din2 = 16'd0;
assign sdram_wrl2 = 1'b0;
assign sdram_wrh2 = 1'b0;
assign sdram_din3 = 16'd0;
assign sdram_wrl3 = 1'b0;
assign sdram_wrh3 = 1'b0;

// SDRAM base offsets (word addresses) — SD Gundam Psycho Salamander layout
//   gfx_kind 0=BG, 1=MG, 2=FG, 3=SPR, 4=TXT
// SeibuCup layout SDRAM (word address). Regioni godzilla (ROM_START godzilla):
//   MAIN 1MB, char(TXT) 128KB, back(BG) 1MB, fore(FG) 1MB SEPARATA, sprite 6MB, oki/audio/copx.
//   char e FG sono REGIONI PROPRIE (non user1 come Legionnaire). FG NON descrambled.
localparam [23:0] MAIN_BASE   = 24'h000000;  // 1MB main 68k       (byte 0x000000)
localparam [23:0] TXT_BASE    = 24'h080000;  // 128KB char/txt     (byte 0x100000)
localparam [23:0] BG_BASE     = 24'h090000;  // 1MB back (BG+MG)   (byte 0x120000)
localparam [23:0] FG_BASE     = 24'h110000;  // 1MB fore (FG)      (byte 0x220000)
localparam [23:0] MG_BASE     = 24'h090000;  // MG condivide back ROM (= BG_BASE)
localparam [23:0] SPR_BASE    = 24'h190000;  // MORTO: sprite su DDR3 (kind 3 mai richiesto); lo slot byte 0x320000 ora ospita OKI

// ================================================================
// PORT 0: Download writes + Tile 32-bit reads
// ================================================================

// --- Download byte-pair assembler ---
// MRA sends word-pair interleaved: HI byte (even addr), LO byte (odd addr)
// We assemble into 16-bit words and write to SDRAM
reg        dl_phase;       // 0=first byte, 1=second byte
reg  [7:0] dl_hi_byte;    // registered first byte
reg  [7:0] dl_lo_byte;    // registered second byte
reg [26:0] dl_addr_save;  // address of first byte (word addr = [26:1])
reg        dl_word_valid;  // assembled word ready to write
reg        dl_toggle;      // toggle for SDRAM request
reg        dl_wait_r;      // waiting for SDRAM to complete write
reg  [1:0] dl_bank_idx;   // current bank being written (0, 1, 2, 3) — quadruplication

wire dl_idle = (sdram_ack0 == dl_toggle);  // no pending request

// Registered download mode — prevents phantom toggle on ioctl_download falling edge.
// download_active stays high until last write completes (dl_idle).
reg download_active;
always @(posedge clk) begin
	if (reset)
		download_active <= 0;
	else if (ioctl_download)
		download_active <= 1;
	else if (dl_idle)
		download_active <= 0;
end

// Finestra INDIRIZZABILE dal controller = 8MB per banco, qualunque sia il
// modulo fisico (anche 128MB con banchi da 32MB): rtl/sdram.sv manda
// row = a[13:1] (13 bit) e col = a[22:14] (9 bit, A9 forzato 0), porte [24:1]
// → 22 bit word = 8MB/banco, e il download quadruplica ogni word nei 4 banchi
// → spazio utile UNICO = 8MB. dl_addr_save[22:1] scarta ioctl_addr[26:23].
// Lo stream MRA godzilla finisce a 0xA40000 (10.25MB): senza gate, tutto cio'
// che sta oltre ioctl 0x800000 wrappa e sovrascrive MAIN/BG/FG (schermo nero).
// Si scrive in SDRAM SOLO cio' che vi risiede: MAIN+CHAR+BG+FG (< 0x320000)
// e OKI (ioctl 0x920000, oltre la finestra) rimappata nel buco sprite a byte
// 0x320000. Sprite→DDR3, Z80→BRAM, COPX inutilizzata: skip.
wire dl_is_oki   = (ioctl_addr >= 27'h920000) && (ioctl_addr < 27'h9A0000);
wire dl_to_sdram = (ioctl_addr < 27'h320000) || dl_is_oki;

always @(posedge clk) begin
	if (reset) begin
		dl_toggle     <= 0;
		dl_wait_r     <= 0;
		dl_bank_idx   <= 2'd0;
	end else begin
		if (~ioctl_download) dl_wait_r <= 0;

		// WIDE=1: Genesis-style — latch word, then write to banks 0, 1, 2
		if (ioctl_download && ioctl_wr && ioctl_index == 16'd0 && dl_to_sdram) begin
			dl_hi_byte   <= ioctl_dout[15:8];
			dl_lo_byte   <= ioctl_dout[7:0];
			dl_addr_save <= dl_is_oki ? (ioctl_addr - 27'h600000) : ioctl_addr;
			dl_wait_r    <= 1;
			dl_bank_idx  <= 2'd0;   // start with bank 0
			dl_toggle    <= ~dl_toggle;
		end
		else if (dl_wait_r && dl_idle) begin
			if (dl_bank_idx < 2'd3) begin
				// Write same word to next bank
				dl_bank_idx <= dl_bank_idx + 2'd1;
				dl_toggle   <= ~dl_toggle;
			end else begin
				// All 4 banks written — done
				dl_wait_r <= 0;
			end
		end
	end
end

assign ioctl_wait = dl_wait_r | (ioctl_download & ~sdram_ready);

// --- Tile 32-bit prefetch FSM ---
// Two consecutive 16-bit reads assembled into 32-bit
reg [2:0]  tile_state;
reg [15:0] tile_hi_word;
reg        tile_req_prev;
reg        tile_toggle;

localparam [2:0]
	TS_IDLE    = 3'd0,
	TS_REQ_HI  = 3'd1,
	TS_WAIT_HI = 3'd2,
	TS_REQ_LO  = 3'd3,
	TS_WAIT_LO = 3'd4;

wire tile_idle = (sdram_ack0 == tile_toggle);
wire [23:1] tile_word_addr = tile_byte_addr[23:1];

always @(posedge clk) begin
	if (reset || download_active) begin
		tile_state    <= TS_IDLE;
		tile_valid    <= 0;
		// Inherit dl_toggle so no phantom toggle on transition
		tile_toggle   <= dl_toggle;
		tile_req_prev <= 0;
	end else begin
		tile_valid    <= 0;
		tile_req_prev <= tile_req;

		case (tile_state)
			TS_IDLE: begin
				// Rising edge of tile_req → start prefetch
				if (tile_req && !tile_req_prev) begin
					tile_state <= TS_REQ_HI;
				end
			end

			TS_REQ_HI: begin
				// Request high word (addr + 0) when port is idle
				if (tile_idle) begin
					tile_toggle <= ~tile_toggle;
					tile_state  <= TS_WAIT_HI;
				end
			end

			TS_WAIT_HI: begin
				if (tile_idle) begin
					// High word arrived
					tile_hi_word <= sdram_dout0;
					tile_state   <= TS_REQ_LO;
				end
			end

			TS_REQ_LO: begin
				// Request low word (addr + 1) when port is idle
				if (tile_idle) begin
					tile_toggle <= ~tile_toggle;
					tile_state  <= TS_WAIT_LO;
				end
			end

			TS_WAIT_LO: begin
				if (tile_idle) begin
					// Low word arrived → 32-bit data ready
					tile_valid <= 1;
					tile_state <= TS_IDLE;
				end
			end

			default: tile_state <= TS_IDLE;
		endcase
	end
end

// 32-bit output: Big Endian {hi_word, lo_word}
assign tile_data = {tile_hi_word, sdram_dout0};

// --- FG address descramble (legionna.cpp:1129 descramble_legionnaire_gfx) ---
// MAME applica un bitswap<24> agli indirizzi della ROM "fore" durante init:
//   src_descrambled[i] = src_raw[bitswap<24>(i, ... 6,5,15,14,13,12, 11,10,9,8, 7,4, 3..0)]
// Quando il GFX engine legge l'indirizzo `addr` dalla ROM (descrambled), prende
// in realtà il dato che era a `bitswap(addr)` nella ROM raw. Quindi se carichiamo
// user1 raw in SDRAM, per leggere "fore" all'indirizzo `addr` dobbiamo accedere
// alla SDRAM a `bitswap(addr)` (offset dentro region 64KB).
//
// La regione "fore" (BK3) sta nella 1a metà di user1, offset 0x00000..0x0FFFF
// byte (= word 0..0x7FFF). user1 è caricato a TXT_BASE word 0x040000.
function automatic [15:0] fg_bitswap_byte(input [15:0] i);
	begin
		fg_bitswap_byte[15] = i[6];
		fg_bitswap_byte[14] = i[5];
		fg_bitswap_byte[13] = i[15];
		fg_bitswap_byte[12] = i[14];
		fg_bitswap_byte[11] = i[13];
		fg_bitswap_byte[10] = i[12];
		fg_bitswap_byte[9]  = i[11];
		fg_bitswap_byte[8]  = i[10];
		fg_bitswap_byte[7]  = i[9];
		fg_bitswap_byte[6]  = i[8];
		fg_bitswap_byte[5]  = i[7];
		fg_bitswap_byte[4]  = i[4];
		fg_bitswap_byte[3]  = i[3];
		fg_bitswap_byte[2]  = i[2];
		fg_bitswap_byte[1]  = i[1];
		fg_bitswap_byte[0]  = i[0];
	end
endfunction

// --- Port 0 mux: download OR tile ---
// Uses registered download_active to avoid phantom toggle on ioctl_download edge.
always @(*) begin
	if (download_active) begin
		sdram_addr0 = {dl_bank_idx, dl_addr_save[22:1]};  // bank in [24:23]
		sdram_din0  = {dl_hi_byte, dl_lo_byte};
		sdram_wrl0  = 1'b1;
		sdram_wrh0  = 1'b1;
		sdram_req0  = dl_toggle;
	end else begin
		// GFX read: word address con region base secondo gfx_kind, forced bank 0
		begin
			reg [23:0] gfx_base;
			reg [23:0] gfx_addr;
			reg [23:0] fetch_word;        // word offset (HI o LO) dentro la region
			reg [15:0] fg_byte_in;        // byte offset originale FG (= fetch_word*2)
			reg [15:0] fg_byte_swapped;   // bitswap MAME del byte offset
			reg [14:0] fg_word_swapped;   // word offset descrambled (= fg_byte_swapped >> 1)
			case (gfx_kind)
				3'd0: gfx_base = BG_BASE;     // BG
				3'd1: gfx_base = BG_BASE;     // MG share BG ROM (TILE_CODE_OFS +0x1000)
				3'd2: gfx_base = FG_BASE;     // FG (fore): godzilla ROM SEPARATA (bg2.619), LINEARE (no descramble)
				3'd3: gfx_base = SPR_BASE;    // sprites
				3'd4: gfx_base = TXT_BASE;    // text (char): region propria (BRAM, non SDRAM)
				default: gfx_base = BG_BASE;
			endcase
			// Word da fetchare: HI = tile_word_addr, LO = tile_word_addr + 1
			case (tile_state)
				TS_WAIT_LO,
				TS_REQ_LO: fetch_word = {1'b0, tile_word_addr} + 24'd1;
				default:   fetch_word = {1'b0, tile_word_addr};
			endcase

			// SeibuCup: FG (gfx_kind==2) e' ROM propria bg2.619, LINEARE (no descramble).
			// Il descramble bitswap era specifico Legionnaire (FG dentro user1). Rimosso:
			// tutte le region leggono lineare gfx_base + fetch_word.
			gfx_addr = gfx_base + fetch_word;
			sdram_addr0 = {2'b00, gfx_addr[21:0]};  // force bank 0
		end
		sdram_din0  = 16'd0;
		sdram_wrl0  = 1'b0;
		sdram_wrh0  = 1'b0;
		sdram_req0  = tile_toggle;
	end
end

// ================================================================
// PORT 1: Main CPU ROM — level req → toggle bridge
// ================================================================
// Edge-detect on main_req rising, send one toggle, wait for ack,
// then pulse main_ready for 1 cycle. Hold DTACK by not pulsing
// ready until SDRAM responds.

reg        main_req_prev;
reg        main_pending;
reg [15:0] main_data_reg;

always @(posedge clk) begin
	if (reset) begin
		sdram_req1    <= 0;
		main_pending  <= 0;
		main_ready    <= 0;
		main_req_prev <= 0;
		main_data_reg <= 16'd0;
	end else begin
		main_ready    <= 0;
		main_req_prev <= main_req;

		// Level: main_req high AND nessun pending in corso AND CPU non sta consumando ready
		if (main_req && !main_pending && !main_ready) begin
			sdram_req1   <= ~sdram_req1;
			main_pending <= 1;
		end

		// SDRAM responded → latch data + pulse ready
		if (main_pending && (sdram_ack1 == sdram_req1)) begin
			main_data_reg <= sdram_dout1;
			main_ready    <= 1;
			main_pending  <= 0;
		end
	end
end

assign main_data = main_data_reg;

// Word address = byte_addr >> 1, plus MAIN_BASE offset, forced to bank 1
always @(*) begin
	reg [23:0] main_word;
	main_word = {1'b0, main_byte_addr[23:1]} + MAIN_BASE;
	sdram_addr1 = {2'b01, main_word[21:0]};  // force bank 1
end

// ================================================================
// PORT 2: Sub CPU ROM — same pattern as Main
// ================================================================

reg        sub_req_prev;
reg        sub_pending;
reg [15:0] sub_data_reg;

always @(posedge clk) begin
	if (reset) begin
		sdram_req2    <= 0;
		sub_pending   <= 0;
		sub_ready     <= 0;
		sub_req_prev  <= 0;
		sub_data_reg  <= 16'd0;
	end else begin
		sub_ready    <= 0;
		sub_req_prev <= sub_req;

		if (sub_req && !sub_req_prev && !sub_pending) begin
			sdram_req2  <= ~sdram_req2;
			sub_pending <= 1;
		end

		// Latch data at ack moment
		if (sub_pending && (sdram_ack2 == sdram_req2)) begin
			sub_data_reg <= sdram_dout2;
			sub_ready    <= 1;
			sub_pending  <= 0;
		end
	end
end

assign sub_data = sub_data_reg;

always @(*) begin
	reg [23:0] sub_word;
	// Sub-port riusato dal COP3 per leggere la MAIN ROM (descrittori hitbox in
	// ROM). Stesso mapping del main-port: bank 1 + MAIN_BASE.
	sub_word = {1'b0, sub_byte_addr[23:1]} + MAIN_BASE;
	sdram_addr2 = {2'b01, sub_word[21:0]};  // BANK 1 (main ROM)
end

// ================================================================
// PORT 3: OKI ADPCM ROM (256KB byte-stream)
// ================================================================
// MRA: <interleave output=16><part map="21"/></interleave> mette ogni byte
// del file in 1 byte SDRAM (NO duplicazione). 2 byte file consecutivi
// formano 1 word SDRAM (low=byte_pari, high=byte_dispari? — verifico
// guardando il pattern dei tile che funziona).
//
// Tile arbiter usa word_addr = byte_addr >> 1 senza preoccuparsi del byte
// LSB perché legge word interi (32-bit assembly). OKI ha bisogno del byte
// puntuale: word_offset = byte_addr[17:1], byte_lsb = byte_addr[0]
// seleziona LO o HI byte della word SDRAM.
//
// SDRAM base = byte 0x320000 / 2 = word 0x190000 (OKI rimappata dal download:
// ioctl 0x920000 - 0x600000; la vecchia base word 0x490000 non stava nei 22
// bit della porta e aliasava su BG_BASE).
localparam [23:0] OKI_BASE = 24'h190000;  // 512KB oki @ byte SDRAM 0x320000 (ioctl 0x920000 rimappato -0x600000, ex slot sprite)

reg [18:0] oki_addr_prev;
reg        oki_pending;
reg        oki_byte_lsb_d;   // bit0 dell'addr corrente (latched al request)

wire oki_addr_changed = (oki_byte_addr != oki_addr_prev);

always @(posedge clk) begin
	if (reset) begin
		sdram_req3     <= 1'b0;
		oki_addr_prev  <= 19'h7FFFF;
		oki_pending    <= 1'b0;
		oki_byte_lsb_d <= 1'b0;
		oki_data       <= 8'd0;
		oki_ok         <= 1'b0;
	end else begin
		if (oki_addr_changed && !oki_pending) begin
			sdram_req3     <= ~sdram_req3;
			oki_addr_prev  <= oki_byte_addr;
			oki_byte_lsb_d <= oki_byte_addr[0];
			oki_pending    <= 1'b1;
			oki_ok         <= 1'b0;
		end
		if (oki_pending && (sdram_ack3 == sdram_req3)) begin
			oki_data       <= oki_byte_lsb_d ? sdram_dout3[15:8] : sdram_dout3[7:0];
			oki_ok         <= 1'b1;
			oki_pending    <= 1'b0;
		end
	end
end

// Word offset = byte_addr[18:1] + OKI_BASE (19 bit = 512KB con bank godzilla)
always @(*) begin
	reg [23:0] oki_word;
	oki_word = OKI_BASE + {6'd0, oki_byte_addr[18:1]};
	sdram_addr3 = {2'b11, oki_word[21:0]};
end

endmodule

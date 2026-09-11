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

/*  Text layer renderer Seibu SeibuCup (8x8, 4bpp, 64x32).

    Specifiche da MAME (legionna.cpp:1117-1126, 1167):

      // charlayout
      8x8, RGN_FRAC(1,1), 4 bpp,
      planes  = { STEP4(0, 4) }                = { 0, 4, 8, 12 }       (bit offset)
      x_bits  = { STEP4(3,-1), STEP4(4*4+3,-1)} = { 3,2,1,0, 19,18,17,16 }
      y_bits  = { STEP8(0, 4*8) }              = { 0, 32, 64, ..., 224 }
      tile size = 8 row × 32 bit = 32 byte / tile (256 bit / 4 bpp)

      // GFXDECODE
      GFXDECODE_ENTRY( "char", 0, charlayout, 48*16, 16 )
      → color base = 0x300 + offset (48*16 = 0x300, palette index)
      → MA verificato in SeibuCup.sv pen_index: 0x700 + (color<<4) + pen
      → discrepanza: 48*16 = 768 = 0x300, NON 0x700. Verifica blocker.

      // get_text_tile_info
      tile  = textram[tile_index];
      color = (tile >> 12) & 0xf;
      tile  = tile & 0xfff;
      tileinfo.set(0, tile, color, 0);
      m_text_layer->set_transparent_pen(15);

    Char ROM in MAME: "char" region 64KB = ROM_COPY user1[0x10000..0x1FFFF].
    Nel nostro porting carichiamo user1 byte 0x080000..0x09FFFF via ioctl,
    e il text legge SOLO la 2a metà (ioctl 0x090000..0x09FFFF) in BRAM 64KB.

    Layout fisico byte per (row, col) di tile_idx:
      word index 16-bit = tile_idx*16 + row*2 + col_high
        dove col_high = (col >= 4) ? 1 : 0
      word 16-bit contiene plane 0/1/2/3 di 4 col:
        bit  3..0 : plane 0, col 3..0
        bit  7..4 : plane 1, col 3..0
        bit 11..8 : plane 2, col 3..0
        bit 15..12: plane 3, col 3..0
      Per col c (0..3 in word low, 0..3 in word high), pen 4-bit:
        sub = 3 - (col & 3)
        pen[0] = word[sub]
        pen[1] = word[sub + 4]
        pen[2] = word[sub + 8]
        pen[3] = word[sub + 12]

    Pen finale palette = 0x300 + (color << 4) + pen (color 0..15, 16 set).
    Trasparente se pen == 15.
*/

module SeibuCup_text_renderer (
	input  wire        clk,
	input  wire        reset,
	input  wire        ce_pix,

	// Video timing
	input  wire  [9:0] hpos,        // 0..383
	input  wire  [8:0] vpos,        // 0..262
	input  wire        de,
	input  wire        layer_en,

	// CRTC scroll (SD Gundam Text: 128 X, 0 Y; lasciamo input parametrico)
	input  wire [15:0] scroll_x,
	input  wire [15:0] scroll_y,

	// OSD offset di rendering (debug pixel-hunting)
	input  wire signed [9:0] xoff,
	input  wire signed [9:0] yoff,

	// Text VRAM read port (4KB = 2Kw, 64*32 grid)
	output reg  [10:0] vram_addr,   // 0..2047
	input  wire [15:0] vram_data,

	// Char ROM download via ioctl (WIDE=1 → 2 byte/word, addr[0]=0)
	// SeibuCup char = 128KB (11.620/10.615), region propria. rom_dl_addr 0..0x1FFFF.
	// rom_dl_data = {byte_high, byte_low} (BB convention WIDE=1).
	// Una sola BRAM 64Kw × 16-bit (charlayout planes={0,4,8,12} packed).
	input  wire        rom_dl_wr,
	input  wire [16:0] rom_dl_addr,
	input  wire [15:0] rom_dl_data,

	// Output pixel (combinatoriale: riduce latency totale di 1 ce_pix → match DCon)
	output wire        opaque,
	output wire [10:0] pen_index    // 0x700 + (color<<4) + pen
);

	// ── Char ROM cache: 128KB region = 64Kw × 16-bit (32 M10K) ──────────────
	// rom_dl_addr[16:1] = word index (0..65535).
	(* ramstyle = "M10K,no_rw_check" *) reg [15:0] charrom [0:65535];
	always @(posedge clk) begin
		if (rom_dl_wr) charrom[rom_dl_addr[16:1]] <= rom_dl_data;
	end

	// ── Pipeline ──────────────────────────────────────────────────────────────
	// Stage 0: input hpos/vpos → calc tile coords, emit vram_addr
	// Stage 1: vram_data registered, calc charrom addr
	// Stage 2: charrom byte_lo/byte_hi registered, decode pen
	// Stage 3: pen + color → pen_index latched

	// --- Stage 0: tile coords ---
	// Gate spaziale: tilemap text 32×32 = 256 px = larghezza schermo BB.
	// Il "wrap" mod 32 cadrebbe in mezzo allo schermo con xoff≠0 → "1UP" tagliato.
	// Soluzione: eff_x signed, marco fuori-range [0..255] → trasparente.
	// Wrap quindi cade SEMPRE a hpos=0 (off-screen) qualsiasi sia xoff.
	// Compensazione latency pipeline: 3 ce_pix tra hpos input e pen visualizzato
	// dal composite. Senza anticipo, screen X=0 mostra pen del tile letto a
	// timing_hpos=46 (HBLANK 0x3FE) → "1" appare a X=3 invece di X=0.
	// Con +3, screen X=0 mostra pen di hpos input=3 → ma de_s2=0 ancora per i
	// primi 3 pixel screen. Per allineare, devo anche bypassare il de_s2 latency:
	// uso direttamente il `de` corrente nell'output (sotto).
	wire signed [10:0] eff_x_s = $signed({1'b0, hpos}) + 11'sd3 + $signed(scroll_x[10:0])
	                            + $signed({xoff[9], xoff});
	wire signed [10:0] eff_y_s = $signed({1'b0, 1'b0, vpos}) + $signed(scroll_y[10:0])
	                            + $signed({yoff[9], yoff});
	wire        x_in_range_s0 = (eff_x_s >= 11'sd0) && (eff_x_s < 11'sd384);
	wire  [5:0] tile_x_s0 = eff_x_s[8:3];   // 6 bit: tilemap 64 colonne (512 px), schermo 320
	wire  [4:0] tile_y_s0 = eff_y_s[7:3];
	wire  [2:0] row_s0    = eff_y_s[2:0];
	wire  [2:0] col_s0    = eff_x_s[2:0];

	// MAME `legionna_v.cpp:214`: text tilemap = 64×32 (8x8 tiles, TILEMAP_SCAN_ROWS).
	// tile_index = tile_y*64 + tile_x. Anche se sullo schermo visibili solo 32 cols,
	// il game scrive textram con layout 64-wide → addr deve riflettere quello.
	always @(posedge clk) begin
		if (ce_pix) vram_addr <= {tile_y_s0, tile_x_s0};  // y*64 + x (tilemap 64-wide, tile_x 6 bit)
	end

	// --- Stage 1: vram_data, decode tile + color ---
	reg [2:0] row_s1, col_s1;
	reg       de_s1, layer_en_s1, x_in_range_s1;
	always @(posedge clk) begin
		if (ce_pix) begin
			row_s1        <= row_s0;
			col_s1        <= col_s0;
			de_s1         <= de;
			layer_en_s1   <= layer_en;
			x_in_range_s1 <= x_in_range_s0;
		end
	end

	wire [11:0] tile_idx_s1 = vram_data[11:0];
	wire  [3:0] tile_clr_s1 = vram_data[15:12];

	// charlayout SeibuCup: word index = tile_idx*16 + row*2 + col_high
	//   tile_idx 12-bit (0..4095), row 3-bit, col_high = col[2] (0 o 1)
	// = (tile_idx << 4) | (row << 1) | col[2]  → 16-bit (64K word, char 128KB)
	wire [15:0] crom_word_s1 = ({4'd0, tile_idx_s1} << 4)
	                         | ({13'd0, row_s1} << 1)
	                         | ({15'd0, col_s1[2]});

	// --- Stage 2: charrom word read ---
	reg [15:0] crom_word_s2 = 16'h0000;
	reg [1:0]  col_lo_s2;        // col[1:0] dentro la 4-col half-tile
	reg [3:0]  tile_clr_s2;
	reg        de_s2, layer_en_s2, x_in_range_s2;
	always @(posedge clk) begin
		if (ce_pix) begin
			crom_word_s2  <= charrom[crom_word_s1];
			col_lo_s2     <= col_s1[1:0];
			tile_clr_s2   <= tile_clr_s1;
			de_s2         <= de_s1;
			layer_en_s2   <= layer_en_s1;
			x_in_range_s2 <= x_in_range_s1;
		end
	end

	// --- Stage 3: pen decode (charlayout planes packed in nibbles) ---
	// Per col c (0..3 dentro la half-tile), bit position = 3 - c.
	// pen[0] = word[bit_pos + 0]   (plane 0)
	// pen[1] = word[bit_pos + 4]   (plane 1)
	// pen[2] = word[bit_pos + 8]   (plane 2)
	// pen[3] = word[bit_pos + 12]  (plane 3)
	// MAME charlayout planes={0,4,8,12}, x_bits={3,2,1,0,...}. MAME readbit usa
	// MSB-first: bit_offs N → byte[N/8] bit (7-N%8). In Verilog word concat
	// (lo=byte0[7:0], hi=byte1[7:0]) → MAME pos N = word[(7-N%8) + (N>=8?8:0)].
	// Per col c (sub=3-c), plane p: bit_offs = p*4 + sub.
	//   plane 0 col 0: bit_offs=3  → word[7-3]=4
	//   plane 1 col 0: bit_offs=7  → word[7-7]=0
	//   plane 2 col 0: bit_offs=11 → word[15-3]=12
	//   plane 3 col 0: bit_offs=15 → word[15-7]=8
	wire [1:0] sub = 2'd3 - col_lo_s2;
	// Plane order DCBA (verificato HW, come BG/MG).
	wire src_a = crom_word_s2[ 7 - {2'd0, sub}];   // A plane 0
	wire src_b = crom_word_s2[ 3 - {2'd0, sub}];   // B plane 1
	wire src_c = crom_word_s2[15 - {2'd0, sub}];   // C plane 2
	wire src_d = crom_word_s2[11 - {2'd0, sub}];   // D plane 3
	wire [3:0] pen = {src_a, src_b, src_c, src_d};  // DCBA

	// Output stage combinatoriale: latency compensata da +3 in eff_x_s, gate
	// `de` corrente (non `_s2`) per rispettare visarea screen reale.
	// MAME GFXDECODE char: color base = 48*16 = 0x300 (16 colorset × 16 pen).
	// Trasparente pen 15.
	wire pixel_active = de & layer_en & (pen != 4'd15);
	assign opaque    = pixel_active;
	assign pen_index = pixel_active ? (11'h300 + {3'd0, tile_clr_s2, pen}) : 11'd0;

endmodule

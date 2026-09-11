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

/*  Palette RAM Seibu SeibuCup (xBGR_555, 2048 entries).

    MAME: PALETTE(config, m_palette).set_format(palette_device::xBGR_555, 128*16);
          (legionna.cpp:1212, 128*16 = 2048 entry)
    Address space CPU: 0x104000..0x104FFF (4 KB = 2048 word).

    Layout word 16-bit (xBGR_555):
      bit 15     : x (unused)
      bit 14..10 : B (5-bit)
      bit  9.. 5 : G (5-bit)
      bit  4.. 0 : R (5-bit)

    Espansione 5→8 standard MAME palette_device: {val5, val5[4:2]}
    (bit replication che riempie i 3 LSB con i 3 MSB del valore 5-bit).

    Modulo simple dual-port:
      Port A (CPU)   : RW byte-enabled
      Port B (video) : RO 11-bit index → 24-bit RGB
*/

module SeibuCup_palette (
	input  wire         clk,
	// Port A: CPU
	input  wire         a_we,
	input  wire   [1:0] a_be,
	input  wire  [10:0] a_addr,
	input  wire  [15:0] a_din,
	output reg   [15:0] a_dout,
	// Port B: video read
	input  wire  [10:0] b_addr,
	output wire   [7:0] b_r,
	output wire   [7:0] b_g,
	output wire   [7:0] b_b
);

	(* ramstyle = "M10K,no_rw_check" *) reg [7:0] mem_hi [0:2047];
	(* ramstyle = "M10K,no_rw_check" *) reg [7:0] mem_lo [0:2047];

	// synthesis translate_off
	integer i;
	initial begin
		for (i = 0; i < 2048; i = i + 1) begin
			mem_hi[i] = 8'h00;
			mem_lo[i] = 8'h00;
		end
	end
	// synthesis translate_on

	reg [7:0] a_dout_hi = 8'h00, a_dout_lo = 8'h00;
	reg [7:0] b_dout_hi = 8'h00, b_dout_lo = 8'h00;

	always @(posedge clk) begin
		if (a_we & a_be[1]) mem_hi[a_addr] <= a_din[15:8];
		if (a_we & a_be[0]) mem_lo[a_addr] <= a_din[7:0];
		a_dout_hi <= mem_hi[a_addr];
		a_dout_lo <= mem_lo[a_addr];
		b_dout_hi <= mem_hi[b_addr];
		b_dout_lo <= mem_lo[b_addr];
	end

	always @(*) a_dout = {a_dout_hi, a_dout_lo};

	// SeibuCup: xBGR_555 → RGB888 (legionna.cpp:1212)
	// Layout: bit[4:0]=R, bit[9:5]=G, bit[14:10]=B. 5 bit per canale.
	// Espansione 5→8 standard MAME palette_device: {val5, val5[4:2]}
	// (replica i 3 MSB nei 3 LSB → range 0..255 con monotonia preservata).
	wire [15:0] b_word = {b_dout_hi, b_dout_lo};
	wire [4:0] vr5 = b_word[4:0];
	wire [4:0] vg5 = b_word[9:5];
	wire [4:0] vb5 = b_word[14:10];

	assign b_r = {vr5, vr5[4:2]};
	assign b_g = {vg5, vg5[4:2]};
	assign b_b = {vb5, vb5[4:2]};

endmodule

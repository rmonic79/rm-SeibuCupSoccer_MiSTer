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

/*  Video timing single-screen Seibu D-Con.

    SD Gundam Psycho Salamander gira su Seibu D-Con (PB91008), stessa famiglia
    di Blood Bros (1990). PCB measurement Blood Bros: VSync 59.4094 Hz,
    HSync 15.6246 kHz, master XTAL 20 MHz.

    Pixel clock dedotto: 20MHz / 4? non torna con HVisible=320 di SD Gundam
    (HTotal sarebbe < 320). Il pixel clock effettivo per matchare HSync
    15.625 kHz con HTotal=384 (320 attivi + 64 blank ≈ 80% duty) è:
        pix_clk = HTotal × HSync = 384 × 15625 = 6.0 MHz
    Frame rate: 6e6 / (384 × 263) = 59.40 Hz ✓ (PCB target 59.4094)

    Active area da MAME (sdgndmps_map): visarea 0..40*8-1, 2*8..30*8-1
    quindi 320×224, V offset = 16 (top blank).

    Modalità selezionabili da OSD:
      mode_60hz=0 : Original 59.40 Hz (VTotal=263)
      mode_60hz=1 : 60.10 Hz          (VTotal=260) — più liscio per LCD 60Hz
*/

module SeibuCup_video_timing
(
	input  wire        clk,            // 96 MHz core
	input  wire        reset,
	input  wire        mode_60hz,      // 0=59.4Hz, 1=60.1Hz
	output reg         ce_pix,         // pixel clock enable (clk/16 → 6 MHz)
	output reg [9:0]   hpos,           // 0..HTotal-1
	output reg [9:0]   vpos,           // 0..VTotal-1
	output wire [9:0]  active_x,       // 0..319 durante area attiva
	output wire [8:0]  active_y,       // 0..223 durante area attiva
	output wire        hblank,
	output wire        vblank,
	output wire        hsync,
	output wire        vsync,
	output wire        de              // active display enable
);

	// ── Pixel clock enable: 96 MHz / 14 = 6.857 MHz ─────────────────────────
	// SeibuCup reale (legionna.cpp:1318 set_raw): pclk 14.318180/2 =
	// 7.15909 MHz. 96/14 = 6.857 e' il divisore intero che, con H_TOTAL 436,
	// riproduce la riga reale: 6.857M/436 = 15.727 kHz (reale 15.734, -0.05%).
	reg [3:0] cediv;
	always @(posedge clk) begin
		if (reset) begin
			cediv  <= 4'd0;
			ce_pix <= 1'b0;
		end else if (cediv == 4'd13) begin
			cediv  <= 4'd0;
			ce_pix <= 1'b1;
		end else begin
			cediv  <= cediv + 4'd1;
			ce_pix <= 1'b0;
		end
	end

	// ── Constants timing ─────────────────────────────────────────────────────
	// SeibuCup REALE (legionna.cpp:1318): set_raw(14318180/2, 455, 0,
	// 320, 258, 0, 224) = 320x224, HTotal 455 @7.159MHz = 15.734 kHz,
	// VTotal 258 → 60.985 Hz.
	// Qui: pclk 96/14 = 6.857 MHz, H_TOTAL 436 → 15.727 kHz (-0.05%),
	// V_TOTAL 258 → 60.96 Hz. HBLANK = 116 px ≈ 16.9 µs (~ come i 135 px
	// reali: budget fetch/prefetch per riga ripristinato — coi vecchi 64 px
	// era dimezzato). Centratura analogica: OSD "Analog VGA H-Shift".
	localparam [9:0] H_TOTAL    = 10'd436;
	localparam [9:0] H_SYNC     = 10'd32;     // 0..31
	localparam [9:0] H_BP       = 10'd56;     // 32..87
	localparam [9:0] H_VISIBLE  = 10'd320;    // 88..407
	localparam [9:0] H_FP       = 10'd28;     // 408..435

	localparam [9:0] H_VIS_START = H_SYNC + H_BP;          // 88
	localparam [9:0] H_VIS_END   = H_VIS_START + H_VISIBLE; // 408

	localparam [9:0] V_SYNC     = 10'd3;      // 0..2
	// CUPSOC: 240 righe visibili (MAME visarea 8..247), 60 Hz.
	localparam [9:0] V_BP       = 10'd10;     // 3..12
	localparam [9:0] V_VISIBLE  = 10'd240;    // 13..252
	localparam [9:0] V_FP       = 10'd9;      // 253..261

	localparam [9:0] V_VIS_START = V_SYNC + V_BP;          // 25
	localparam [9:0] V_VIS_END_59 = V_VIS_START + V_VISIBLE; // 249
	// Raster unico 262: 6.857e6/(436*262) = 60.03 Hz = i 60 Hz di MAME.
	localparam [9:0] V_TOTAL_59  = 10'd262;
	localparam [9:0] V_TOTAL_60  = 10'd262;

	wire [9:0] V_TOTAL = mode_60hz ? V_TOTAL_60 : V_TOTAL_59;

	// ── HV counter ───────────────────────────────────────────────────────────
	always @(posedge clk) begin
		if (reset) begin
			hpos <= 10'd0;
			vpos <= 10'd0;
		end else if (ce_pix) begin
			if (hpos == H_TOTAL - 10'd1) begin
				hpos <= 10'd0;
				if (vpos == V_TOTAL - 10'd1) vpos <= 10'd0;
				else                         vpos <= vpos + 10'd1;
			end else begin
				hpos <= hpos + 10'd1;
			end
		end
	end

	// ── Sync, blanking, DE ───────────────────────────────────────────────────
	assign hsync  = (hpos < H_SYNC);
	assign vsync  = (vpos < V_SYNC);
	assign hblank = (hpos < H_VIS_START) || (hpos >= H_VIS_END);
	assign vblank = (vpos < V_VIS_START) || (vpos >= V_VIS_END_59);
	assign de     = ~hblank & ~vblank;

	// ── Active coordinates per renderer (0..319, 0..223) ─────────────────────
	assign active_x = de ? (hpos - H_VIS_START)        : 10'd0;
	assign active_y = de ? (vpos[8:0] - V_VIS_START[8:0]) : 9'd0;

endmodule

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

// pause_overlay.sv — overlay pausa (256x240, RGB 8-bit).
// Modulo standalone, layout identico a ChinaGate.
//
// Coord raster: render_x 9-bit (0..511), render_y 9-bit (0..511).
//               Visibile assunto come ChinaGate: 4..255 H × 8..246 V (256x240).
//
// Layout:
//   - Logo 48x48 sorgente, scale 2x = 96x96 sullo schermo, centrato
//   - SUPPORTERS header top, centrato (10 char × 1 row)
//   - PATRONS scroll vertical, centrato (30 char × 24 row visibili, MSG_ROWS=66)
//
// pause=1 ⇒ dim video + overlay. clean=1 (OSD bypass) lascia solo dim+raw.

module pause_overlay (
	input  wire        clk,
	input  wire        pause,
	input  wire        clean,    // OSD bypass overlay (no logo, no testo)
	input  wire        vblank,   // vblank pulse esterno per scroll tick

	input  wire [8:0]  render_x,
	input  wire [8:0]  render_y,

	input  wire [7:0]  rgb_r_in,
	input  wire [7:0]  rgb_g_in,
	input  wire [7:0]  rgb_b_in,

	output wire [7:0]  rgb_r_out,
	output wire [7:0]  rgb_g_out,
	output wire [7:0]  rgb_b_out
);

// Effective overlay: pause attiva ma clean disattivato.
wire overlay_on = pause & ~clean;

// VBlank pulse: rising edge del segnale vblank esterno (frame boundary).
reg vblank_d;
always @(posedge clk) vblank_d <= vblank;
wire vblank_pulse = vblank & ~vblank_d;

// =====================================================================
// Logo placement: 48x48 sorgente, SCALE 2x → 96x96.
// Schermo 256x240 (render_x 4..255, render_y 8..246).
// Centro X = (4+255)/2 = 129, top-left X = 129-48 = 81
// Centro Y = (8+246)/2 = 127, top-left Y = 127-48 = 79
// =====================================================================
// SeibuCup: render_x 0..319, render_y 0..239 (320x240 attivi). Logo 96x96
// centrato. ATTENZIONE: le costanti qui sotto erano rimaste quelle del
// porting da un gioco largo 256 -> tutto l'overlay usciva spostato a sinistra.
localparam [8:0] LOGO_X    = 9'd112;  // (320-96)/2 = 112
localparam [8:0] LOGO_Y    = 9'd72;   // (240-96)/2 = 72
localparam [8:0] LOGO_XEND = LOGO_X + 9'd96;
localparam [8:0] LOGO_YEND = LOGO_Y + 9'd96;

// Read-ahead 1 ck per BRAM
wire [8:0] x_ahead   = render_x + 9'd1;
wire [8:0] dx_screen = x_ahead - LOGO_X;
wire [8:0] dy_screen = render_y - LOGO_Y;

wire in_logo_ahead = overlay_on &&
	(x_ahead   >= LOGO_X) && (x_ahead   < LOGO_XEND) &&
	(render_y  >= LOGO_Y) && (render_y  < LOGO_YEND);

// SCALE 2x: dx/2, dy/2
wire [5:0] dx = dx_screen[6:1];
wire [5:0] dy = dy_screen[6:1];
// addr = dy*48 + dx = (dy<<5) + (dy<<4) + dx, max=2303
wire [11:0] logo_addr = {1'b0, dy, 5'd0} + {2'b0, dy, 4'd0} + {6'd0, dx};

// =====================================================================
// Logo BRAM 2304x2 init da logo/logo.mem
// =====================================================================
reg [1:0] logo_rom [0:2303] /* synthesis ramstyle = "M10K" */;
initial $readmemb("logo/logo.mem", logo_rom);
reg [1:0] logo_pix;
reg       in_logo_now;
always @(posedge clk) begin
	logo_pix    <= logo_rom[logo_addr];
	in_logo_now <= in_logo_ahead;
end

// Palette logo: pal0=nero (trasparente), pal1=magenta, pal2=cyan, pal3=bianco
reg [7:0] lr, lg, lb;
always @(*) case (logo_pix)
	2'd0: {lr, lg, lb} = 24'h000000;
	2'd1: {lr, lg, lb} = 24'hFF00FF;
	2'd2: {lr, lg, lb} = 24'h00E6E4;
	2'd3: {lr, lg, lb} = 24'hFFFFFF;
endcase

// Logo sempre opaque (incluso pal0=nero).
wire logo_opaque = 1'b1;

// =====================================================================
// Header "SUPPORTERS" — top, centrato.
// 10 char × 8 = 80 px, centrati su 320: (320-80)/2 = 120.
// =====================================================================
wire       header_on;
wire [1:0] header_tier;
pause_text #(
	.W_CHARS      (10),
	.H_CHARS      (1),
	.MSG_ROWS     (1),
	.ORIGIN_X     (10'd120),   // (320-80)/2 = 120
	.ORIGIN_Y     (9'd16),     // top + 16 px margine
	.SCROLL_EN    (0),
	.FONT_FILE    ("logo/font_darius.hex"),
	.MSG_FILE     ("logo/header.mem")
) u_header (
	.clk          (clk),
	.active       (overlay_on),
	.vblank_pulse (vblank_pulse),
	.render_x     ({1'b0, render_x}),
	.render_y     (render_y),
	.pixel_on     (header_on),
	.pixel_tier   (header_tier)
);

// =====================================================================
// Patron scroll — quadrante centrale.
// 30 char × 8 = 240 px, centrati su 320: (320-240)/2 = 40.
// 24 row × 8 = 192 px visibili. MSG_ROWS = righe totali di patrons.mem
// (loop scroll lungo): tenerlo allineato al file, o le ultime righe
// vengono troncate dal generatore senza dire niente.
// ORIGIN_Y = 32 (sotto SUPPORTERS).
// =====================================================================
wire       patron_on;
wire [1:0] patron_tier;
pause_text #(
	.W_CHARS       (30),
	.H_CHARS       (24),
	.MSG_ROWS      (73),
	.ORIGIN_X      (10'd40),    // (320-240)/2 = 40
	.ORIGIN_Y      (9'd32),     // sotto header
	.SCROLL_EN     (1),
	.SCROLL_PERIOD (3),
	.FONT_FILE     ("logo/font_darius.hex"),
	.MSG_FILE      ("logo/patrons.mem")
) u_patron (
	.clk          (clk),
	.active       (overlay_on),
	.vblank_pulse (vblank_pulse),
	.render_x     ({1'b0, render_x}),
	.render_y     (render_y),
	.pixel_on     (patron_on),
	.pixel_tier   (patron_tier)
);

// =====================================================================
// Palette tier:
//   tier 0 = bianco
//   tier 1 = cyan (bronze/base)
//   tier 2 = magenta (silver)
//   tier 3 = oro (gold)
// =====================================================================
function [23:0] tier_color;
	input [1:0] tier;
	begin
		case (tier)
			2'd0: tier_color = 24'hFFFFFF;
			2'd1: tier_color = 24'h00E6E4;
			2'd2: tier_color = 24'hFF00FF;
			2'd3: tier_color = 24'hFFD700;
		endcase
	end
endfunction

wire [23:0] header_rgb = 24'hFFD700;             // giallo/oro
wire [23:0] patron_rgb = tier_color(patron_tier);

// Priorità: header > patron > logo > dim > raw
wire        text_on  = header_on | patron_on;
wire [23:0] text_rgb = header_on ? header_rgb : patron_rgb;

// =====================================================================
// Output mux
// =====================================================================
wire [7:0] dim_r = {1'b0, rgb_r_in[7:1]};
wire [7:0] dim_g = {1'b0, rgb_g_in[7:1]};
wire [7:0] dim_b = {1'b0, rgb_b_in[7:1]};

assign rgb_r_out = !overlay_on              ? rgb_r_in :
                   text_on                  ? text_rgb[23:16] :
                   in_logo_now & logo_opaque ? lr        :
                                              dim_r;
assign rgb_g_out = !overlay_on              ? rgb_g_in :
                   text_on                  ? text_rgb[15:8]  :
                   in_logo_now & logo_opaque ? lg        :
                                              dim_g;
assign rgb_b_out = !overlay_on              ? rgb_b_in :
                   text_on                  ? text_rgb[7:0]   :
                   in_logo_now & logo_opaque ? lb        :
                                              dim_b;

endmodule

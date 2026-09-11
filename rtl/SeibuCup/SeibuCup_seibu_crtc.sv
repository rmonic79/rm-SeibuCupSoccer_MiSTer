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

/*  Seibu CRTC fedele a MAME (seibu_crtc.cpp).

    Mappa registri 0x00..0x4F (40 word, indirizzato dal main 68k a $0C0000).
    Pure register file: scrittura → RAM interna, niente side effect su read.
    Nessun IRQ generato dal chip stesso (vblank/raster vengono dal video timing).

    Callback rilevanti per SD Gundam / Blood Bros / D-Con:
      0x1A  reg_1a       (bit0=flip screen, bit1=dynamic size)
      0x1C  layer_en     (b0=BG, b1=MG, b2=FG, b3=Text, b4=Sprites)
      0x20  bg  scrollX
      0x22  bg  scrollY
      0x24  mg  scrollX
      0x26  mg  scrollY
      0x28  fg  scrollX
      0x2A  fg  scrollY
      0x2C..0x3A  layer base scroll (rowscroll bases)
      0x40..0x4E  override sprite chip (SEI251/252/RISE) per giochi avanzati;
                  SD Gundam li scrive da routine dedicata, ignoriamo.

    Reset: solo reg 0x1A=0; il resto non viene azzerato dal chip
    (la ROM è responsabile di inizializzarli).
*/

module SeibuCup_seibu_crtc (
	input  wire        clk,
	input  wire        reset,

	// Bus 16-bit dal main 68k (mapped 0xC0000..0xC004F → offset[6:1])
	input  wire        cs,
	input  wire        wr,        // ~rnw & ~asn
	input  wire        rd,
	input  wire  [6:1] addr,      // word address 0x00..0x4E
	input  wire  [1:0] dsn,       // byte enable (DS_n, active low)
	input  wire [15:0] wdata,
	output reg  [15:0] rdata,

	// Layer enable (bit-stable per SD Gundam)
	output wire        layer_en_bg,
	output wire        layer_en_mg,
	output wire        layer_en_fg,
	output wire        layer_en_text,
	output wire        layer_en_spr,

	// Scroll X/Y per layer (15-bit ciascuno; SD Gundam aggiunge +128 nel dispatcher)
	output wire [15:0] scroll_bg_x,
	output wire [15:0] scroll_bg_y,
	output wire [15:0] scroll_mg_x,
	output wire [15:0] scroll_mg_y,
	output wire [15:0] scroll_fg_x,
	output wire [15:0] scroll_fg_y,
	// Scroll-base per-layer 0x2C-0x3A (origini layer, programmate dal gioco:
	// godzilla BG/MG/FG/TXT X=$1D8/$1DA/$1D9/$1D8 Y=$1EF; cupsoc set0
	// X=$1C8/$1CA/$1C9/$1C8 Y=$1FF = shift -16, set1 X=$177 Y=$100)
	output wire [15:0] scroll_base_bg_x,     // 0x2C
	output wire [15:0] scroll_base_bg_y,     // 0x2E
	output wire [15:0] scroll_base_mg_x,     // 0x30
	output wire [15:0] scroll_base_mg_y,     // 0x32
	output wire [15:0] scroll_base_fg_x,     // 0x34
	output wire [15:0] scroll_base_fg_y,     // 0x36
	output wire [15:0] scroll_base_text_x,   // 0x38
	output wire [15:0] scroll_base_text_y,   // reg 0x3A raw (godzilla: text dy = 0x1EF - reg)

	// Mode register
	output wire        flip_screen,
	output wire        dyn_layer_size
);

	// 40 word file (0x00..0x4E), indirizzato come addr[5:1] (32 word effettive)
	// Spec dice 7-bit address ma usiamo 0x00..0x4E (uso 6:1).
	reg [15:0] regs [0:39];

	// SeibuCup: CRTC standard read/write (no bitswap come BB/DCon/Gundam).
	// addr è [6:1] (6 bit) — usiamo tutti i 6 bit.
	wire [5:0] widx = addr[6:1];
	wire [1:0] be   = ~dsn;

	// ── Write ────────────────────────────────────────────────────────────────
	integer i;
	always @(posedge clk) begin
		if (reset) begin
			// Solo 0x1A è azzerato dal device reale (m_reg_1a = 0).
			// Altri registri lasciati ai valori scritti dalla ROM,
			// ma per evitare X iniziali in simulazione li azzeriamo tutti.
			for (i = 0; i < 40; i = i + 1) regs[i] <= 16'h0000;
		end else if (cs && wr) begin
			if (widx < 6'd40) begin
				if (be[1]) regs[widx][15:8] <= wdata[15:8];
				if (be[0]) regs[widx][7:0]  <= wdata[7:0];
			end
		end
	end

	// ── Read (pure register file, no side effects) ───────────────────────────
	always @(*) begin
		rdata = (cs && rd && widx < 6'd40) ? regs[widx] : 16'hFFFF;
	end

	// ── Output decode ────────────────────────────────────────────────────────
	// reg index = byte_offset / 2
	// 0x1A → idx 0x0D ; 0x1C → idx 0x0E
	// 0x20..0x2A → idx 0x10..0x15
	wire [15:0] reg_1a   = regs[6'h0D];
	wire [15:0] reg_1c   = regs[6'h0E];

	assign flip_screen     = reg_1a[0];
	assign dyn_layer_size  = reg_1a[1];

	// MAME mappa: bit0=screen0(BG), bit1=screen2(MG), bit2=screen1(FG), bit3=screen3(Text)
	// (vedi seibu_crtc.cpp commento "screen 0=BG, screen 1=FG, screen 2=MG, screen 3=Text")
	// In SD Gundam dcon.cpp layer_en_w gira la maschera così:
	//   ~layer_en bit 0 → BG draw
	//   ~layer_en bit 1 → MG draw   (corretto: il bit Seibu è invertito? VEDI sotto)
	//   ~layer_en bit 2 → FG draw
	//   ~layer_en bit 3 → Text draw
	//   ~layer_en bit 4 → Sprites draw
	// Quindi il chip espone direttamente 5 bit; "abilitato"=bit ATTIVO (non invertito).
	// Il dispatcher MAME usa "if (BIT(~m_layer_en, n))" per saltare = layer disabilitato
	// quando il bit è 1. Quindi la convenzione del bit nel registro è:
	//   bit=1 → layer DISABILITATO (mask bit)
	//   bit=0 → layer abilitato
	// Esponiamo qui i flag NEGATI per chiarezza nel renderer.
	// SeibuCup layer ordering (legionna_v.cpp:384-390 screen_update_SEIBUCUP):
	//   bit 0 = BG (background)
	//   bit 1 = MG (midground)
	//   bit 2 = FG (foreground)
	//   bit 3 = Text
	//   bit 4 = Sprites
	// ATTENZIONE: e' l'OPPOSTO di legionna (screen_update_legionna :342-343 ha
	// bit0=MG, bit1=BG). Il vecchio decode citava legionna → enable BG/MG scambiati.
	assign layer_en_bg   = ~reg_1c[0];
	assign layer_en_mg   = ~reg_1c[1];
	assign layer_en_fg   = ~reg_1c[2];
	assign layer_en_text = ~reg_1c[3];
	assign layer_en_spr  = ~reg_1c[4];

	assign scroll_bg_x = regs[6'h10];   // 0x20 / 2
	assign scroll_bg_y = regs[6'h11];   // 0x22
	assign scroll_mg_x = regs[6'h12];   // 0x24
	assign scroll_mg_y = regs[6'h13];   // 0x26
	assign scroll_fg_x = regs[6'h14];   // 0x28
	assign scroll_fg_y = regs[6'h15];   // 0x2A

	// Scroll-base 0x2C-0x3A (seibu_crtc.cpp:243 layer_scroll_base_w;
	// screen0=BG, screen2=MG, screen1=FG, screen3=Text). Raw esce qui; le
	// formule (delta vs riferimento godzilla, Y invertita come il
	// set_scrolldy(0x1ef-data) di MAME) stanno nel top.
	assign scroll_base_bg_x   = regs[6'h16];   // 0x2C
	assign scroll_base_bg_y   = regs[6'h17];   // 0x2E
	assign scroll_base_mg_x   = regs[6'h18];   // 0x30
	assign scroll_base_mg_y   = regs[6'h19];   // 0x32
	assign scroll_base_fg_x   = regs[6'h1A];   // 0x34
	assign scroll_base_fg_y   = regs[6'h1B];   // 0x36
	assign scroll_base_text_x = regs[6'h1C];   // 0x38
	assign scroll_base_text_y = regs[6'h1D];   // 0x3A

endmodule

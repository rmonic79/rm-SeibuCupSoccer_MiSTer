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

// SeibuCup_main_top — Top SeibuCup (1× M68000 @ 10 MHz, Seibu COP3 + custom).
// Base: DCon_main_top. Aggiunto modulo COP3 + Main RAM 128KB.

module darius_dual68k_top
#(
	parameter [1:0] MAIN_CORE_IMPL    = 2'd1,
	parameter [1:0] SUB_CORE_IMPL     = 2'd1,
	parameter       HOLD_SUB_IN_RESET = 1'b0,
	parameter       ENABLE_C00050_NOP = 1'b1,
	parameter       ENABLE_WATCHDOG   = 1'b1,
	parameter       ENABLE_PC080_CTRL = 1'b1,
	parameter       ENABLE_DC0000     = 1'b1,
	parameter       ENABLE_C00060     = 1'b1,
	parameter       ENABLE_C00020     = 1'b1,
	parameter       ENABLE_C00022     = 1'b1,
	parameter       ENABLE_C00024     = 1'b1,
	parameter       ENABLE_C00030     = 1'b1,
	parameter       ENABLE_C00032     = 1'b1,
	parameter       ENABLE_C00034     = 1'b1,
	parameter       ENABLE_D40000     = 1'b1,
	parameter       ENABLE_D40002     = 1'b1,
	parameter       ENABLE_D20000     = 1'b1,
	parameter       ENABLE_D20002     = 1'b1,
	parameter       ENABLE_C0000C     = 1'b1,
	parameter       ENABLE_C00010     = 1'b1,
	parameter       ENABLE_MAIN_PC060HA_PORT = 1'b1,
	parameter       ENABLE_MAIN_PC060HA_COMM = 1'b1,
	parameter       ENABLE_MAIN_D00000 = 1'b1,
	parameter       ENABLE_MAIN_PALETTE = 1'b1,
	parameter       ENABLE_FG_RAM      = 1'b1,
	parameter       ENABLE_MAIN_CTRL  = 1'b1,
	parameter       ENABLE_MAIN_SHARED = 1'b1,
	parameter       ENABLE_MAIN_SPRITE = 1'b1,
	parameter       ENABLE_MAIN_IO    = 1'b1,
	parameter       ENABLE_MAIN_VIDEO = 1'b1,
	parameter       ENABLE_MAIN_PLAYER_IO = 1'b1,
	parameter       ENABLE_SUB_SHARED = 1'b1,
	parameter       ENABLE_SUB_SPRITE = 1'b1,
	parameter       ENABLE_SUB_PALETTE = 1'b1,
	parameter       ENABLE_SUB_IO     = 1'b1,
	parameter       ENABLE_VBLANK_IRQ = 1'b1
)
(
	input  wire        clk,
	input  wire        reset,
	input  wire        pause,
	input  wire  [2:0] clk_sel,
	input  wire  [2:0] sub_clk_sel,
	input  wire  [1:0] z80_clk_sel,
	input  wire  [7:0] p1_input,
	input  wire  [7:0] p2_input,
	input  wire [15:0] pin34_input,   // PLAYERS34: P3+P4 (+COIN3/COIN4)
	input  wire [15:0] system_input,
	input  wire [15:0] dsw_input,
	input  wire [15:0] dsw2_input,
	input  wire  [2:0] osd_start_phase,
	input  wire [15:0] main_rom_rdata,
	input  wire        main_rom_ready,
	input  wire [15:0] sub_rom_rdata,
	input  wire        sub_rom_ready,
	input  wire [9:0]  render_x,
	input  wire [8:0]  render_y,
	input  wire        vblank_in,
	input  wire [31:0] tilerom_data,
	input  wire        tilerom_valid,
	output wire [23:0] main_rom_addr,
	output wire        main_rom_req,
	output wire [23:0] sub_rom_addr,
	output wire        sub_rom_req,
	output wire [23:0] tilerom_addr,
	output wire        tilerom_req,
	output wire  [2:0] tilerom_kind,
	input  wire        ioctl_download,
	input  wire        ioctl_wr,
	input  wire [26:0] ioctl_addr,
	input  wire [15:0] ioctl_dout,
	output wire [15:0] xscroll_l0,
	output wire [15:0] xscroll_l1,
	output wire [15:0] yscroll_l0,
	output wire [15:0] yscroll_l1,
	output wire [15:0] xscroll_mg,
	output wire [15:0] yscroll_mg,
	output wire [15:0] text_base_y,     // CRTC reg 0x3A raw (godzilla text scrolldy)
	// CRTC scroll-base per-layer raw (origini layer programmate dal gioco)
	output wire [15:0] base_bg_x,  output wire [15:0] base_bg_y,
	output wire [15:0] base_mg_x,  output wire [15:0] base_mg_y,
	output wire [15:0] base_fg_x,  output wire [15:0] base_fg_y,
	output wire [15:0] base_txt_x,
	output wire [15:0] ctrl_l0,
	input  wire [10:0] pal_b_addr,
	output wire  [7:0] pal_b_r,
	output wire  [7:0] pal_b_g,
	output wire  [7:0] pal_b_b,
	input  wire [10:0] text_vram_addr,
	output wire [15:0] text_vram_data,
	input  wire [10:0] bg_vram_addr,
	output wire [15:0] bg_vram_data,
	input  wire [10:0] mg_vram_addr,
	output wire [15:0] mg_vram_data,
	input  wire [10:0] fg_vram_addr,
	output wire [15:0] fg_vram_data,
	input  wire [10:0] spr_vram_addr,    // SeibuCup: 2048 word sprite RAM
	output wire [15:0] spr_vram_data,
	output wire [15:0] gfx_bank,
	input  wire  [7:0] coin_input,
	output wire [18:0] oki_rom_addr,   // bit18 = bank OKI godzilla (512KB)
	input  wire  [7:0] oki_rom_data,
	input  wire        oki_rom_ok,
	input  wire  [2:0] fm_vol_sel,
	input  wire  [2:0] oki_vol_sel,
	input  wire  [2:0] fm_ch_vol_sel0,
	input  wire  [2:0] fm_ch_vol_sel1,
	input  wire  [2:0] fm_ch_vol_sel2,
	input  wire  [2:0] fm_ch_vol_sel3,
	input  wire  [2:0] fm_ch_vol_sel4,
	input  wire  [2:0] fm_ch_vol_sel5,
	input  wire  [2:0] fm_ch_vol_sel6,
	input  wire  [2:0] fm_ch_vol_sel7,
	input  wire  [2:0] fm_ch_vol_sel8,
	input  wire  [2:0] oki_ch_vol_sel0,
	input  wire  [2:0] oki_ch_vol_sel1,
	input  wire  [2:0] oki_ch_vol_sel2,
	input  wire  [2:0] oki_ch_vol_sel3,
	output wire signed [15:0] audio_l,
	output wire signed [15:0] audio_r
);

// ─── Sub-ROM port riusato dal COP3 per leggere la MAIN ROM (hitbox b100/b900) ──
wire [23:0] cop_sub_rom_addr;
wire        cop_sub_rom_req;
assign sub_rom_addr      = cop_sub_rom_addr;
assign sub_rom_req       = cop_sub_rom_req;
assign tilerom_addr = 24'd0;
assign tilerom_req  = 1'b0;
assign tilerom_kind = 3'd0;

assign xscroll_l0     = crtc_scroll_bg_x;
assign yscroll_l0     = crtc_scroll_bg_y;
assign xscroll_l1     = crtc_scroll_fg_x;
assign yscroll_l1     = crtc_scroll_fg_y;
assign xscroll_mg     = crtc_scroll_mg_x;
assign yscroll_mg     = crtc_scroll_mg_y;
assign text_base_y    = crtc_scroll_base_text_y;
assign base_bg_x      = crtc_base_bg_x;
assign base_bg_y      = crtc_base_bg_y;
assign base_mg_x      = crtc_base_mg_x;
assign base_mg_y      = crtc_base_mg_y;
assign base_fg_x      = crtc_base_fg_x;
assign base_fg_y      = crtc_base_fg_y;
assign base_txt_x     = crtc_base_txt_x;
assign ctrl_l0        = {9'd0, crtc_dyn_size, crtc_flip_screen,
                          crtc_layer_en_spr, crtc_layer_en_text,
                          crtc_layer_en_fg, crtc_layer_en_mg, crtc_layer_en_bg};

// VBlank-synced pause (DCon pattern)
reg vblank_in_prev;
reg paused_safe_r;
always @(posedge clk) begin
	if (reset) begin
		vblank_in_prev <= 1'b0;
		paused_safe_r  <= 1'b0;
	end else begin
		vblank_in_prev <= vblank_in;
		if (vblank_in && !vblank_in_prev) paused_safe_r <= pause;
	end
end
wire paused_safe = paused_safe_r;

// ─── Audio Seibu reale ──────────────────────────────────────────────────────
SeibuCup_audio_z80 u_audio (
	.clk(clk), .reset(reset),
	.pause(paused_safe),
	.clk_sel(z80_clk_sel),
	.ioctl_download(ioctl_download), .ioctl_wr(ioctl_wr),
	.ioctl_addr(ioctl_addr), .ioctl_dout(ioctl_dout),
	.snd_cs(snd_cs),
	.snd_addr(snd_addr),
	.snd_wr(snd_wr),
	.snd_rd(snd_rd),
	.snd_wdata(snd_wdata),
	.snd_rdata(snd_rdata),
	.snd_nmi_n(1'b1),
	.snd_reset_in(1'b0),
	.coin_input(coin_input),
	.oki_rom_addr(oki_rom_addr),
	.oki_rom_data(oki_rom_data),
	.oki_rom_ok(oki_rom_ok),
	.fm_vol_sel(fm_vol_sel),
	.oki_vol_sel(oki_vol_sel),
	.fm_ch_vol_sel0(fm_ch_vol_sel0), .fm_ch_vol_sel1(fm_ch_vol_sel1), .fm_ch_vol_sel2(fm_ch_vol_sel2),
	.fm_ch_vol_sel3(fm_ch_vol_sel3), .fm_ch_vol_sel4(fm_ch_vol_sel4), .fm_ch_vol_sel5(fm_ch_vol_sel5),
	.fm_ch_vol_sel6(fm_ch_vol_sel6), .fm_ch_vol_sel7(fm_ch_vol_sel7), .fm_ch_vol_sel8(fm_ch_vol_sel8),
	.oki_ch_vol_sel0(oki_ch_vol_sel0), .oki_ch_vol_sel1(oki_ch_vol_sel1),
	.oki_ch_vol_sel2(oki_ch_vol_sel2), .oki_ch_vol_sel3(oki_ch_vol_sel3),
	.audio_l(audio_l), .audio_r(audio_r)
);

// ─── 68000 main ──────────────────────────────────────────────────────────────
reg [6:0] clk_num;
reg [7:0] clk_den;
always @(*) case (clk_sel)
	3'd0: begin clk_num = 7'd5;  clk_den = 8'd60; end // 8 MHz
	3'd1: begin clk_num = 7'd5;  clk_den = 8'd40; end // 12 MHz
	3'd2: begin clk_num = 7'd5;  clk_den = 8'd48; end // 10 MHz
	3'd3: begin clk_num = 7'd5;  clk_den = 8'd30; end // 16 MHz
	3'd4: begin clk_num = 7'd5;  clk_den = 8'd20; end // 24 MHz
	3'd5: begin clk_num = 7'd5;  clk_den = 8'd15; end // 32 MHz
	default: begin clk_num = 7'd5;  clk_den = 8'd48; end // 10 MHz default
endcase

wire [23:0] cpu_addr;
wire        cpu_asn;
wire        cpu_rnw;
wire [1:0]  cpu_dsn;
wire [15:0] cpu_dout;
wire [15:0] cpu_din;
wire        cpu_cs;
wire        map_busy;
wire        cop_dma_busy;   // FSM-based: drives port MUXes (487-606)
wire        cop_cpu_stall;  // dma_busy + trigger early: stall CPU SOLO (chiude buco DTACK)
wire        cpu_busy = map_busy | cop_cpu_stall;
wire        cpu_iack;

// VBLANK IRQ4 — MAME legionna.cpp: m_maincpu->set_vblank_int(irq4_line_hold)
// MAME HOLD_LINE: IRQ asserted on edge, released ONLY at IACK (= when CPU
// fetches vector). Identical to DCon donor (which works).
// No timeout-based release (Trio-style fix not applicable here: FX68K cpu_iack
// signal works correctly for HOLD_LINE behavior).
// VBLANK IRQ4 — MAME legionna.cpp: m_maincpu->set_vblank_int(irq4_line_hold)
// MAME HOLD_LINE: IRQ asserted on edge, released ONLY at IACK (= when CPU
// fetches vector). Identical to DCon donor (which works).
// No timeout-based release (Trio-style fix not applicable here: FX68K cpu_iack
// signal works correctly for HOLD_LINE behavior).
reg vblank_prev;
reg main_irq4_pending;
reg iack_prev;
always @(posedge clk) begin
	if (reset) begin
		vblank_prev       <= 1'b0;
		main_irq4_pending <= 1'b0;
		iack_prev         <= 1'b0;
	end else begin
		vblank_prev <= vblank_in;
		iack_prev   <= cpu_iack;
		// PRIORITA' AL SET: se il fronte vblank e il fronte IACK cadono nello
		// stesso ciclo clk (IACK ritardato dai DTACK-hold COP/SDRAM fin quasi al
		// vblank successivo), il vecchio ordine (clear DOPO il set nel sorgente)
		// faceva vincere il clear -> il NUOVO vblank era PERSO = tick di gioco
		// perso in silenzio ($10810E non incrementa). Con else-if il set vince:
		// nessun vblank puo' collassare.
		if (vblank_in && !vblank_prev)       main_irq4_pending <= 1'b1;
		// One-shot clear sul FRONTE di cpu_iack (non sul livello): cpu_iack resta
		// alto per piu' cicli durante l'IACK bus cycle.
		else if (cpu_iack && !iack_prev)     main_irq4_pending <= 1'b0;
	end
end
wire [2:0] ipl_n = main_irq4_pending ? 3'b011 : 3'b111;

darius_cpu_node #(.CPU_ID(1'b0), .CORE_IMPL(MAIN_CORE_IMPL)) u_cpu (
	.clk(clk),
	.reset(reset),
	.soft_reset(1'b0),
	.halt_n(~paused_safe),
	.clk_num(clk_num),
	.clk_den(clk_den),
	.ipl_n(ipl_n),
	.bus_din(cpu_din),
	.bus_cs(cpu_cs),
	.bus_busy(cpu_busy),
	.dev_br(1'b0),
	.bus_addr(cpu_addr),
	.bus_asn(cpu_asn),
	.bus_rnw(cpu_rnw),
	.bus_dsn(cpu_dsn),
	.bus_dout(cpu_dout),
	.dbg_pc(),
	.dbg_fc(),
	.dbg_dtackn(),
	.dbg_fave(),
	.dbg_fworst(),
	.iack(cpu_iack)
);

// ─── Memory map ─────────────────────────────────────────────────────────────
wire [15:0] ram_rdata, bg_rdata, fg_rdata, mg_rdata, txt_rdata, pal_rdata, pallive_rdata, spr_rdata;
wire        ram_we, bg_we, fg_we, mg_we, txt_we, pal_we, map_pallive_we, spr_we;
wire [10:0] map_vram_addr;
wire [15:0] map_vram_wdata;
wire  [1:0] map_vram_be;
wire [15:0] map_ram_addr;        // SeibuCup: 16-bit = 128KB Main RAM
wire [15:0] map_ram_wdata;
wire  [1:0] map_ram_be;

// CRTC
wire        crtc_cs, crtc_wr, crtc_rd;
wire  [6:1] crtc_addr;
wire [15:0] crtc_rdata;
wire        crtc_layer_en_bg, crtc_layer_en_mg, crtc_layer_en_fg, crtc_layer_en_text, crtc_layer_en_spr;
wire [15:0] crtc_scroll_bg_x, crtc_scroll_bg_y;
wire [15:0] crtc_scroll_mg_x, crtc_scroll_mg_y;
wire [15:0] crtc_scroll_fg_x, crtc_scroll_fg_y;
wire [15:0] crtc_scroll_base_text_y;
wire [15:0] crtc_base_bg_x, crtc_base_bg_y, crtc_base_mg_x, crtc_base_mg_y;
wire [15:0] crtc_base_fg_x, crtc_base_fg_y, crtc_base_txt_x;
wire        crtc_flip_screen, crtc_dyn_size;

// Sound comm
wire        snd_cs, snd_wr, snd_rd;
wire  [4:1] snd_addr;
wire [15:0] snd_wdata;
wire [15:0] snd_rdata;

// COP3
wire        cop_cs, cop_wr, cop_rd;
wire [10:1] cop_addr;
wire [15:0] cop_wdata;
wire [15:0] cop_rdata;

// Memory map module
SeibuCup_maincpu_map u_map (
	.clk            (clk),
	.reset          (reset),
	.bus_addr       (cpu_addr),
	.bus_asn        (cpu_asn),
	.bus_rnw        (cpu_rnw),
	.bus_dsn        (cpu_dsn),
	.bus_wdata      (cpu_dout),
	.bus_rdata      (cpu_din),
	.bus_cs         (cpu_cs),
	.bus_busy       (map_busy),
	.p1_input       (p1_input),
	.p2_input       (p2_input),
	.pin34_input    (pin34_input),
	.system_input   (system_input),
	.dsw_input      (dsw_input),
	.dsw2_input     (dsw2_input),
	.osd_start_phase(osd_start_phase),
	.main_rom_rdata (main_rom_rdata),
	.main_rom_ready (main_rom_ready),
	.main_rom_addr  (main_rom_addr),
	.main_rom_req   (main_rom_req),
	.ram_rdata      (ram_rdata),
	.ram_we         (ram_we),
	.ram_addr       (map_ram_addr),
	.ram_wdata      (map_ram_wdata),
	.ram_be         (map_ram_be),
	.bg_rdata       (bg_rdata),
	.bg_we          (bg_we),
	.mg_rdata       (mg_rdata),
	.mg_we          (mg_we),
	.fg_rdata       (fg_rdata),
	.fg_we          (fg_we),
	.txt_rdata      (txt_rdata),
	.txt_we         (txt_we),
	.pal_rdata      (pal_rdata),
	.pal_we         (pal_we),
	.pallive_rdata  (pallive_rdata),
	.pallive_we     (map_pallive_we),
	.spr_rdata      (spr_rdata),
	.spr_we         (spr_we),
	.vram_addr      (map_vram_addr),
	.vram_wdata     (map_vram_wdata),
	.vram_be        (map_vram_be),
	.crtc_cs        (crtc_cs),
	.crtc_wr        (crtc_wr),
	.crtc_rd        (crtc_rd),
	.crtc_addr      (crtc_addr),
	.crtc_rdata     (crtc_rdata),
	.gfx_bank       (gfx_bank),
	.snd_cs         (snd_cs),
	.snd_wr         (snd_wr),
	.snd_rd         (snd_rd),
	.snd_addr       (snd_addr),
	.snd_wdata      (snd_wdata),
	.snd_rdata      (snd_rdata),
	.cop_cs         (cop_cs),
	.cop_wr         (cop_wr),
	.cop_rd         (cop_rd),
	.cop_addr       (cop_addr),
	.cop_wdata      (cop_wdata),
	.cop_rdata      (cop_rdata),
	.cop_busy       (cop_cpu_stall)
);

// ─── COP3 (SEI300) — DMA cablate per init game ──────────────────────────────
// Porte DMA cablate al main_top:
//   - Main RAM: priorità mux su porta A (CPU stallata via dma_busy)
//   - Palette renderer: cmd 0x15 → u_pal porta A
//   - Sprite RAM CPU-side: per sprite DMA / sort
//   - Tilemap renderer: cmd 0x14 → u_bg/fg/mg/txt_REND porta A (split
//     staging/render, vedi blocco BRAM sotto)
//   - dma_src_byte: read via porta B delle BRAM staging / Main RAM
wire [15:0] cop_dma_ram_addr;
wire [15:0] cop_dma_ram_wdata;
wire        cop_dma_ram_we;
wire  [1:0] cop_dma_ram_be;
wire [23:0] cop_dma_src_byte;
wire [15:0] cop_dma_src_rdata;
wire [10:0] cop_dma_spr_addr;
wire [15:0] cop_dma_spr_wdata;
wire        cop_dma_spr_we;
wire [12:0] cop_dma_vram_addr;
wire [15:0] cop_dma_vram_wdata;
wire        cop_dma_vram_we;
wire [12:0] cop_dma_vram_addr_now;
wire        cop_dma_vram_we_now;
wire [10:0] cop_dma_pal_addr;
wire [15:0] cop_dma_pal_wdata;
wire        cop_dma_pal_we;
wire        cop_dma_pal_stage_we;
wire        cop_dma_fill_we;
wire [22:0] cop_dma_fill_addr;
wire [15:0] cop_dma_fill_wdata;

SeibuCup_cop3 u_cop3 (
	.clk(clk), .reset(reset),
	.cs(cop_cs), .wr(cop_wr), .rd(cop_rd),
	.addr(cop_addr), .dsn(cpu_dsn), .wdata(cop_wdata),
	.rdata(cop_rdata),
	.dma_ram_addr  (cop_dma_ram_addr),
	.dma_ram_wdata (cop_dma_ram_wdata),
	.dma_ram_be    (cop_dma_ram_be),
	.dma_ram_rdata (ram_rdata),
	.dma_ram_we    (cop_dma_ram_we),
	.dma_src_byte  (cop_dma_src_byte),
	.dma_src_rdata (cop_dma_src_rdata),
	.cop_rom_addr  (cop_sub_rom_addr),    // -> sub_rom port (libero) -> SDRAM main ROM
	.cop_rom_req   (cop_sub_rom_req),
	.cop_rom_rdata (sub_rom_rdata),
	.cop_rom_ready (sub_rom_ready),
	.dma_spr_addr  (cop_dma_spr_addr),
	.dma_spr_wdata (cop_dma_spr_wdata),
	.dma_spr_we    (cop_dma_spr_we),
	.dma_vram_addr (cop_dma_vram_addr),
	.dma_vram_wdata(cop_dma_vram_wdata),
	.dma_vram_we   (cop_dma_vram_we),
	.dma_vram_addr_now(cop_dma_vram_addr_now),
	.dma_vram_we_now  (cop_dma_vram_we_now),
	.dma_pal_addr  (cop_dma_pal_addr),
	.dma_pal_wdata (cop_dma_pal_wdata),
	.dma_pal_we    (cop_dma_pal_we),
	.dma_pal_stage_we (cop_dma_pal_stage_we),
	.dma_fill_we   (cop_dma_fill_we),
	.dma_fill_addr (cop_dma_fill_addr),
	.dma_fill_wdata(cop_dma_fill_wdata),
	.dma_busy      (cop_dma_busy),
	.cpu_stall     (cop_cpu_stall)
);

// ── DMA SRC read ────────────────────────────────────────────────────────────
// COP3 source byte address copre tutto lo spazio 68k. Il mux source indirizza
// la BRAM corretta in base al range memory map cupsoc:
//   $100800-$100FFF (2KB) BG        $101000-$1017FF (2KB) FG
//   $101800-$101FFF (2KB) MG        $102000-$102FFF (4KB) Text
//   $103000-$103FFF (4KB) Pal LIVE  $104000-$104FFF (4KB) Pal STAGING
//   $107000-$107FFF (4KB) Sprite RAM
//   altrove → Main RAM (incluso $105000-$106FFF = blend target del fade)
wire [12:0] src_top13 = cop_dma_src_byte[23:11];   // 2KB granularity
reg [15:0] cop_dma_src_rdata_mux;
always @(*) begin
    case (src_top13)
        13'h201: cop_dma_src_rdata_mux = bg_stage_rdata;   // $100800-$100FFF (STAGING porta B)
        13'h202: cop_dma_src_rdata_mux = fg_stage_rdata;   // $101000-$1017FF
        13'h203: cop_dma_src_rdata_mux = mg_stage_rdata;   // $101800-$101FFF
        13'h204,13'h205: cop_dma_src_rdata_mux = txt_stage_rdata;  // $102000-$102FFF
        13'h206,13'h207: cop_dma_src_rdata_mux = pallive_src_rdata; // $103000-$103FFF (LIVE porta B, src del 0x15)
        13'h208,13'h209: cop_dma_src_rdata_mux = pal_stage_rdata;  // $104000-$104FFF (STAGING porta B, src del fade)
        13'h20E: cop_dma_src_rdata_mux = spr_rdata;  // $107000-$1077FF (spriteram 2KB, porta A hijack)
        // $107800-$107FFF = "Ani Dsp Ram": nella mappa CPU e' Main RAM (is_rampad),
        // ed e' dove stanno le liste che la sort-DMA ordina ($107A28, $107A54...).
        // Deve cadere nel default (ram_rdata), NON nella BRAM sprite: altrimenti
        // il sort legge la meta' alta della BRAM, che la CPU non scrive mai (zeri).
        default:         cop_dma_src_rdata_mux = ram_rdata;  // resto = Main RAM
    endcase
end
assign cop_dma_src_rdata = cop_dma_src_rdata_mux;

// ─── BRAM istanze ───────────────────────────────────────────────────────────
// Main RAM: porta unica con mux COP3 priority (CPU stallata via dma_busy)
wire [15:0] mr_addr  = cop_dma_busy
                       ? (cop_dma_ram_we ? cop_dma_ram_addr : cop_dma_src_byte[16:1])
                       : map_ram_addr;
wire [15:0] mr_wdata = cop_dma_busy ? cop_dma_ram_wdata : map_ram_wdata;
wire        mr_we    = cop_dma_busy ? cop_dma_ram_we    : ram_we;
wire  [1:0] mr_be    = cop_dma_busy ? cop_dma_ram_be     : map_ram_be;
SeibuCup_ram_128k u_main_ram (
	.clk(clk), .addr(mr_addr), .din(mr_wdata), .dout(ram_rdata),
	.we(mr_we), .be(mr_be)
);

// COP3 cmd 0x14 dma_tilemap_buffer (legionna_v.cpp:86 videowrite_cb_w):
//   offset 0x000..0x3FF  → BG  (back_data)
//   offset 0x400..0x7FF  → FG  (fore_data)
//   offset 0x800..0xBFF  → MG  (mid_data)
//   offset 0xC00..0x13FF → Text (textram, 0x800 word)
// SRC = porta B delle BRAM STAGING (read), DST = porta A delle BRAM REND
// (write). Use CURRENT-cycle write intent (combinational, valid in D_WRITE(i)):
// write data = cop_dma_src_rdata (mem[src_i], valido durante D_WRITE(i)).
wire [12:0] cop_vram_off = cop_dma_vram_addr_now;
wire cop_to_bg   = cop_dma_busy & cop_dma_vram_we_now & (cop_vram_off < 13'h0400);
wire cop_to_fg   = cop_dma_busy & cop_dma_vram_we_now & (cop_vram_off >= 13'h0400) & (cop_vram_off < 13'h0800);
wire cop_to_mg   = cop_dma_busy & cop_dma_vram_we_now & (cop_vram_off >= 13'h0800) & (cop_vram_off < 13'h0C00);
wire cop_to_text = cop_dma_busy & cop_dma_vram_we_now & (cop_vram_off >= 13'h0C00) & (cop_vram_off < 13'h1400);

// Source range selectors (BRAM-specifico): 1 quando cop_dma_src_byte cade in
// quel range. La porta A della BRAM emette quell'addr per leggere il dato.
wire src_in_bg   = (src_top13 == 13'h201);
wire src_in_fg   = (src_top13 == 13'h202);
wire src_in_mg   = (src_top13 == 13'h203);
wire src_in_text = (src_top13 == 13'h204) | (src_top13 == 13'h205);
wire src_in_pal  = (src_top13 == 13'h208) | (src_top13 == 13'h209);
wire src_in_spr  = (src_top13 == 13'h20E) | (src_top13 == 13'h20F);

// Fill COP 0x116/0x118 + write assolute COP (fade dst, c480) → scratch per
// regione (cop_dma_fill_addr = word addr 68k assoluto). Il fill scrive le
// BRAM STAGING (= la host RAM di MAME): il per-frame DMA 0x14/0x15 le
// propaga al renderer.
// SeibuCup: BG $100800→w$80400, FG $101000→w$80800, MG $101800→w$80C00,
// TXT $102000→w$81000, PAL LIVE $103000→w$81800, PAL STG $104000→w$82000,
// SPR $107000→w$83800 (clear fill 0x118 + entry c480).
wire bg_fill      = cop_dma_fill_we & (cop_dma_fill_addr >= 23'h80400) & (cop_dma_fill_addr < 23'h80800);
wire fg_fill      = cop_dma_fill_we & (cop_dma_fill_addr >= 23'h80800) & (cop_dma_fill_addr < 23'h80C00);
wire mg_fill      = cop_dma_fill_we & (cop_dma_fill_addr >= 23'h80C00) & (cop_dma_fill_addr < 23'h81000);
wire txt_fill     = cop_dma_fill_we & (cop_dma_fill_addr >= 23'h81000) & (cop_dma_fill_addr < 23'h81800);
wire pallive_fill = cop_dma_fill_we & (cop_dma_fill_addr >= 23'h81800) & (cop_dma_fill_addr < 23'h82000);
wire pal_fill     = cop_dma_fill_we & (cop_dma_fill_addr >= 23'h82000) & (cop_dma_fill_addr < 23'h82800);
wire spr_fill     = cop_dma_fill_we & (cop_dma_fill_addr >= 23'h83800) & (cop_dma_fill_addr < 23'h84000);

// ── SPLIT STAGING vs RENDERER (semantica MAME 2-stadi, come la palette) ─────
// MAME godzilla: $101000-$103FFF e' .ram() PURO (handler background_w/…
// COMMENTATI nel map, legionna.cpp) — le scritture CPU NON toccano i tilemap.
// I tilemap leggono m_back/fore/mid_data/m_textram = buffer SEPARATI
// (make_unique_clear, legionna_v.cpp) scritti SOLO da videowrite_cb_w =
// DMA 0x14. Con BRAM unica il redraw colonne del gioco (nell'ISR, DOPO la
// write scroll) trapelava SUBITO al renderer -> contenuto nuovo con scroll
// vecchio per un frame -> pop 16px su MID (ogni 2 frame) e FORE (ogni frame).
//   u_XX_ram   = STAGING: porta A = CPU (+fill 0x116/0x118, come MAME che
//                filla la host RAM), porta B = sorgente del DMA 0x14.
//   u_XX_rend  = RENDERER: porta A scritta SOLO dal DMA 0x14 (cop_to_*),
//                porta B letta dal video. Coppia contenuto/scroll = atomica
//                a livello ISR, come MAME.
wire        bg_we_mux   = bg_fill ? 1'b1 : bg_we;
wire [10:0] bg_addr_mux = bg_fill ? {1'b0, cop_dma_fill_addr[9:0]} : map_vram_addr;
wire  [1:0] bg_be_mux   = bg_fill ? 2'b11 : map_vram_be;
wire [15:0] bg_stage_rdata;
SeibuCup_vram_dp #(.AW(11)) u_bg_ram (
	.clk(clk), .a_we(bg_we_mux), .a_be(bg_be_mux),
	.a_addr(bg_addr_mux),
	.a_din(bg_fill ? cop_dma_fill_wdata : map_vram_wdata),
	.a_dout(bg_rdata),
	.b_addr(cop_dma_src_byte[11:1] - 11'h400), .b_dout(bg_stage_rdata)  // base $100800
);
SeibuCup_vram_dp #(.AW(11)) u_bg_rend (
	.clk(clk), .a_we(cop_to_bg), .a_be(2'b11),
	.a_addr(cop_vram_off[10:0]),
	.a_din(cop_dma_src_rdata), .a_dout(),
	.b_addr(bg_vram_addr), .b_dout(bg_vram_data)
);

wire        fg_we_mux   = fg_fill ? 1'b1 : fg_we;
wire [10:0] fg_addr_mux = fg_fill ? {1'b0, cop_dma_fill_addr[9:0]} : map_vram_addr;
wire  [1:0] fg_be_mux   = fg_fill ? 2'b11 : map_vram_be;
wire [15:0] fg_stage_rdata;
SeibuCup_vram_dp #(.AW(11)) u_fg_ram (
	.clk(clk), .a_we(fg_we_mux), .a_be(fg_be_mux),
	.a_addr(fg_addr_mux),
	.a_din(fg_fill ? cop_dma_fill_wdata : map_vram_wdata),
	.a_dout(fg_rdata),
	.b_addr(cop_dma_src_byte[11:1]), .b_dout(fg_stage_rdata)  // base $101000 allineata
);
SeibuCup_vram_dp #(.AW(11)) u_fg_rend (
	.clk(clk), .a_we(cop_to_fg), .a_be(2'b11),
	.a_addr(cop_vram_off[10:0] - 11'h400),
	.a_din(cop_dma_src_rdata), .a_dout(),
	.b_addr(fg_vram_addr), .b_dout(fg_vram_data)
);

// MG: dma_vram_addr 0x800..0xBFF → local 0..0x3FF (bit[10:0] = 0..0x3FF naturally)
wire        mg_we_mux = mg_fill ? 1'b1 : mg_we;
wire  [1:0] mg_be_mux = mg_fill ? 2'b11 : map_vram_be;
wire [15:0] mg_stage_rdata;
SeibuCup_vram_dp #(.AW(11)) u_mg_ram (
	.clk(clk), .a_we(mg_we_mux), .a_be(mg_be_mux),
	.a_addr(mg_fill ? {1'b0, cop_dma_fill_addr[9:0]} : map_vram_addr),
	.a_din(mg_fill ? cop_dma_fill_wdata : map_vram_wdata),
	.a_dout(mg_rdata),
	.b_addr(cop_dma_src_byte[11:1] - 11'h400), .b_dout(mg_stage_rdata)  // base $101800
);
SeibuCup_vram_dp #(.AW(11)) u_mg_rend (
	.clk(clk), .a_we(cop_to_mg), .a_be(2'b11),
	.a_addr(cop_vram_off[10:0]),
	.a_din(cop_dma_src_rdata), .a_dout(),
	.b_addr(mg_vram_addr), .b_dout(mg_vram_data)
);

wire        txt_we_mux   = txt_fill ? 1'b1 : txt_we;
wire  [1:0] txt_be_mux   = txt_fill ? 2'b11 : map_vram_be;
wire [15:0] txt_stage_rdata;
SeibuCup_vram_dp #(.AW(11)) u_txt_ram (
	.clk(clk), .a_we(txt_we_mux), .a_be(txt_be_mux),
	.a_addr(txt_fill ? cop_dma_fill_addr[10:0] : map_vram_addr),
	.a_din(txt_fill ? cop_dma_fill_wdata : map_vram_wdata),
	.a_dout(txt_rdata),
	.b_addr(cop_dma_src_byte[11:1]), .b_dout(txt_stage_rdata)  // base $102000 allineata
);
SeibuCup_vram_dp #(.AW(11)) u_txt_rend (
	.clk(clk), .a_we(cop_to_text), .a_be(2'b11),
	.a_addr(cop_vram_off[10:0] - 11'h400),
	.a_din(cop_dma_src_rdata), .a_dout(),
	.b_addr(text_vram_addr), .b_dout(text_vram_data)
);
// ── Palette: split STAGING ($104000) vs RENDERER (semantica MAME 2-stadi) ───
// MAME: la CPU e il fade (cmd 0x80) scrivono la STAGING RAM a $104000. Poi cmd
// 0x15 (dma_palette_buffer, emesso ogni IRQ4) copia staging → palette renderer
// (BRAM separato letto dal video). Lo split rende gli sprite visibili ogni frame.
//
// u_pal_stage = STAGING $104000:
//   porta A write = CPU (is_pal, pal_we) quando non-busy, oppure fade
//                   (cop_dma_pal_stage_we) quando busy.
//   porta B read  = sorgente per cmd 0x15 (src $104000 → cop_dma_src_byte).
wire        pal_stage_we_mux   = pal_fill ? 1'b1 : (cop_dma_busy ? cop_dma_pal_stage_we : pal_we);
wire [10:0] pal_stage_addr_mux = pal_fill ? cop_dma_fill_addr[10:0]
                                          : (cop_dma_busy ? cop_dma_pal_addr : map_vram_addr);
wire [15:0] pal_stage_din_mux  = pal_fill ? cop_dma_fill_wdata
                                          : (cop_dma_busy ? cop_dma_pal_wdata : map_vram_wdata);
wire  [1:0] pal_stage_be_mux   = (pal_fill | cop_dma_busy) ? 2'b11 : map_vram_be;
wire [15:0] pal_stage_rdata;       // porta B: dato letto da cmd 0x15
SeibuCup_vram_dp #(.AW(11)) u_pal_stage (
	.clk(clk),
	.a_we(pal_stage_we_mux), .a_be(pal_stage_be_mux),
	.a_addr(pal_stage_addr_mux), .a_din(pal_stage_din_mux), .a_dout(pal_rdata),
	.b_addr(cop_dma_src_byte[11:1]), .b_dout(pal_stage_rdata)
);

// u_pallive = PALETTE LIVE $103000 (solo cupsoc, disasm $1C14):
//   host RAM visibile alla CPU, DESTINAZIONE del fade DMA 0x8x
//   (write assolute via canale fill, finestra pallive_fill) e SORGENTE
//   del cmd 0x15 che la copia nel renderer (porta B via src mux).
wire        pallive_we_mux   = pallive_fill ? 1'b1 : map_pallive_we;
wire [10:0] pallive_addr_mux = pallive_fill ? cop_dma_fill_addr[10:0] : map_vram_addr;
wire [15:0] pallive_din_mux  = pallive_fill ? cop_dma_fill_wdata : map_vram_wdata;
wire  [1:0] pallive_be_mux   = pallive_fill ? 2'b11 : map_vram_be;
wire [15:0] pallive_src_rdata;
SeibuCup_vram_dp #(.AW(11)) u_pallive (
	.clk(clk),
	.a_we(pallive_we_mux), .a_be(pallive_be_mux),
	.a_addr(pallive_addr_mux), .a_din(pallive_din_mux), .a_dout(pallive_rdata),
	.b_addr(cop_dma_src_byte[11:1]), .b_dout(pallive_src_rdata)
);

// u_pal = RENDERER: letto dal video, scritto SOLO da cmd 0x15 (dma_pal_we).
// MAME godzilla: $104000 e' RAM pura (legionna.cpp:262, handler palette
// commentato) — la palette VISIBILE si aggiorna SOLO via paletteramout_cb =
// dma_palette_buffer 0x15 (seibucop_dma.ipp:27); il fade DMA scrive lo
// STAGING (host RAM, dma.ipp:125). NIENTE mirror CPU->renderer: rendeva
// visibile la palette full-brightness appena scritta dal gioco -> FLASH
// dell'immagine PRIMA del fade-in. (Il mirror era il workaround Legionnaire
// per il suo fade software $263E — non vale per godzilla.)
wire        pal_we_mux   = cop_dma_busy & cop_dma_pal_we;
wire  [1:0] pal_be_mux   = cop_dma_busy ? 2'b11            : map_vram_be;
wire [10:0] pal_addr_mux = cop_dma_busy ? cop_dma_pal_addr : map_vram_addr;
wire [15:0] pal_din_mux  = cop_dma_busy ? cop_dma_pal_wdata: map_vram_wdata;
SeibuCup_palette u_pal (
	.clk(clk),
	.a_we(pal_we_mux), .a_be(pal_be_mux),
	.a_addr(pal_addr_mux),
	.a_din(pal_din_mux), .a_dout(),
	.b_addr(pal_b_addr),
	.b_r(pal_b_r), .b_g(pal_b_g), .b_b(pal_b_b)
);

// Sprite RAM CPU-side: COP3 sprite DMA scrive qui via dma_spr_we.
// spr_fill: il gioco AZZERA la sprite RAM col fill COP (precedente heatbrl:
// senza, gli sprite vecchi persistono → duplicati/ghost).
wire        spr_we_mux   = spr_fill ? 1'b1 : (cop_dma_busy ? cop_dma_spr_we : spr_we);
wire [10:0] spr_addr_mux = spr_fill ? cop_dma_fill_addr[10:0]
                          : cop_dma_busy ? (cop_dma_spr_we ? cop_dma_spr_addr
                                                          : cop_dma_src_byte[11:1])
                                        : map_vram_addr;
wire [15:0] spr_din_mux  = spr_fill ? cop_dma_fill_wdata
                                    : (cop_dma_busy ? cop_dma_spr_wdata : map_vram_wdata);
wire  [1:0] spr_be_mux   = (spr_fill | cop_dma_busy) ? 2'b11 : map_vram_be;
SeibuCup_vram_dp #(.AW(11)) u_spr_ram_cpu (
	.clk(clk), .a_we(spr_we_mux), .a_be(spr_be_mux),
	.a_addr(spr_addr_mux), .a_din(spr_din_mux), .a_dout(spr_rdata),
	.b_addr(spr_copy_src_addr), .b_dout(spr_copy_src_data)
);

// Sprite RAM SHADOW — copy CPU→shadow al rising edge di vblank
reg [11:0] spr_copy_cnt = 12'd2100;   // idle: > 2048
reg        vblank_d = 1'b0;
wire       spr_copy_active = (spr_copy_cnt < 12'd2049);

wire [10:0] spr_copy_src_addr = spr_copy_cnt[10:0];
wire [15:0] spr_copy_src_data;

wire [10:0] spr_copy_dst_addr = spr_copy_cnt[10:0] - 11'd1;
wire        spr_copy_we = spr_copy_active && (spr_copy_cnt >= 12'd1) && (spr_copy_cnt <= 12'd2048);

always @(posedge clk) begin
	vblank_d <= vblank_in;
	if (reset) begin
		spr_copy_cnt <= 12'd2100;
	end else begin
		if (vblank_in && !vblank_d) begin
			spr_copy_cnt <= 12'd0;
		end else if (spr_copy_active) begin
			spr_copy_cnt <= spr_copy_cnt + 12'd1;
		end
	end
end

(* ramstyle = "M10K,no_rw_check" *) reg [15:0] spr_ram_shadow [0:2047];
reg [15:0] spr_shadow_b_q = 16'd0;
// synthesis translate_off
integer shadow_init_i;
initial begin
	for (shadow_init_i = 0; shadow_init_i < 2048; shadow_init_i = shadow_init_i + 1)
		spr_ram_shadow[shadow_init_i] = 16'h0000;
end
// synthesis translate_on
always @(posedge clk) begin
	if (spr_copy_we) spr_ram_shadow[spr_copy_dst_addr] <= spr_copy_src_data;
	spr_shadow_b_q <= spr_ram_shadow[spr_vram_addr];
end
assign spr_vram_data = spr_shadow_b_q;

// ─── Seibu CRTC ─────────────────────────────────────────────────────────────
SeibuCup_seibu_crtc u_crtc (
	.clk(clk), .reset(reset),
	.cs(crtc_cs), .wr(crtc_wr), .rd(crtc_rd),
	.addr(crtc_addr), .dsn(cpu_dsn), .wdata(cpu_dout),
	.rdata(crtc_rdata),
	.layer_en_bg(crtc_layer_en_bg), .layer_en_mg(crtc_layer_en_mg),
	.layer_en_fg(crtc_layer_en_fg), .layer_en_text(crtc_layer_en_text),
	.layer_en_spr(crtc_layer_en_spr),
	.scroll_bg_x(crtc_scroll_bg_x), .scroll_bg_y(crtc_scroll_bg_y),
	.scroll_mg_x(crtc_scroll_mg_x), .scroll_mg_y(crtc_scroll_mg_y),
	.scroll_fg_x(crtc_scroll_fg_x), .scroll_fg_y(crtc_scroll_fg_y),
	.scroll_base_bg_x(crtc_base_bg_x),   .scroll_base_bg_y(crtc_base_bg_y),
	.scroll_base_mg_x(crtc_base_mg_x),   .scroll_base_mg_y(crtc_base_mg_y),
	.scroll_base_fg_x(crtc_base_fg_x),   .scroll_base_fg_y(crtc_base_fg_y),
	.scroll_base_text_x(crtc_base_txt_x),
	.scroll_base_text_y(crtc_scroll_base_text_y),
	.flip_screen(crtc_flip_screen), .dyn_layer_size(crtc_dyn_size)
);

endmodule


// ─── BRAM helper: 128KB main RAM (64Kw × 16, byte-enable) ───────────────────
module SeibuCup_ram_128k (
	input  wire        clk,
	input  wire [15:0] addr,
	input  wire [15:0] din,
	output reg  [15:0] dout = 16'd0,   // init output reg: M10K no_rw_check NON azzera
	input  wire        we,             // -> X al powerup -> fetch garbage -> $C00002
	input  wire  [1:0] be
);
	(* ramstyle = "M10K,no_rw_check" *) reg [7:0] mem_hi [0:65535];
	(* ramstyle = "M10K,no_rw_check" *) reg [7:0] mem_lo [0:65535];
	// synthesis translate_off
	integer i;
	initial begin
		for (i = 0; i < 65536; i = i + 1) begin
			mem_hi[i] = 8'h00;
			mem_lo[i] = 8'h00;
		end
	end
	// synthesis translate_on
	always @(posedge clk) begin
		if (we & be[1]) mem_hi[addr] <= din[15:8];
		if (we & be[0]) mem_lo[addr] <= din[7:0];
		dout <= {mem_hi[addr], mem_lo[addr]};
	end
endmodule


// ─── BRAM helper: video word RAM true dual-port (CPU + renderer) ────────────
module SeibuCup_vram_dp #(parameter AW = 11) (
	input  wire           clk,
	input  wire           a_we,
	input  wire     [1:0] a_be,
	input  wire [AW-1:0]  a_addr,
	input  wire    [15:0] a_din,
	output reg     [15:0] a_dout = 16'd0,   // init: vedi SeibuCup_ram_128k
	input  wire [AW-1:0]  b_addr,
	output reg     [15:0] b_dout = 16'd0
);
	localparam DEPTH = 1 << AW;
	(* ramstyle = "M10K,no_rw_check" *) reg [7:0] mem_hi [0:DEPTH-1];
	(* ramstyle = "M10K,no_rw_check" *) reg [7:0] mem_lo [0:DEPTH-1];
	// synthesis translate_off
	integer i;
	initial begin
		for (i = 0; i < DEPTH; i = i + 1) begin
			mem_hi[i] = 8'h00;
			mem_lo[i] = 8'h00;
		end
	end
	// synthesis translate_on
	always @(posedge clk) begin
		if (a_we & a_be[1]) mem_hi[a_addr] <= a_din[15:8];
		if (a_we & a_be[0]) mem_lo[a_addr] <= a_din[7:0];
		a_dout <= {mem_hi[a_addr], mem_lo[a_addr]};
		b_dout <= {mem_hi[b_addr], mem_lo[b_addr]};
	end
endmodule

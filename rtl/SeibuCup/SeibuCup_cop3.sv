// SPDX-License-Identifier: GPL-3.0-or-later
/*  This file is part of SeibuCupSoccer_MiSTer.

    SeibuCupSoccer_MiSTer is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    SeibuCupSoccer_MiSTer is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with SeibuCupSoccer_MiSTer.  If not, see <http://www.gnu.org/licenses/>.

    Author: Umberto Parisi (rmonic79)
    Version: 1.0
    Date: 2026

*/

// SeibuCup_cop3.sv — Seibu SEI300 ("COP3") coprocessor.
//
// Seibu Cup Soccer (Seibu Kaihatsu, 1992) leans on this chip for everything
// that matters: player movement, where the ball lands, contacts, headers,
// volleys, the radar dots and the depth ordering of the players. MAME marks
// the game MACHINE_UNEMULATED_PROTECTION, so most of what is implemented here
// was derived from the program ROM itself rather than copied.
//
// Which variant of a command the chip runs is decided by the MICROCODE TABLE
// the ROM uploads at boot ($00B720), not by the game name — that rule settled
// the 6200 variant, the atan family and the length rule for who writes the
// resulting angle. The trigonometric tables were later confirmed against the
// bootleg board, which carries the chip's lookup ROMs.
//
// References: MAME seibucop.cpp / seibucop_cmd.ipp / seibucop_dma.ipp /
// legionna.cpp — used as a starting point, and corrected where the program ROM
// disagreed with them. See docs/COP_cupsoc.md.
//
// Mapped on the 68K bus at $100400-$1006FF (legionna_cop_map). The `addr`
// input is the WORD offset relative to $100400 (addr=10'h000 -> byte $100400,
// addr=10'h17E -> byte $1006FC).

module SeibuCup_cop3 (
	input  wire        clk,
	input  wire        reset,

	// CPU bus (region decoded externally)
	input  wire        cs,
	input  wire        wr,
	input  wire        rd,
	input  wire [10:1] addr,        // word offset relative to 0x100400
	input  wire  [1:0] dsn,
	input  wire [15:0] wdata,
	output reg  [15:0] rdata,

	// DMA master — Main RAM (port B for source-read, port A for write)
	output reg  [15:0] dma_ram_addr,
	output reg  [15:0] dma_ram_wdata,
	input  wire [15:0] dma_ram_rdata,
	output reg         dma_ram_we,
	// byte-enable della write: default 2'b11 (word). Per i cop_write_byte (es.
	// l'angolo 138e a 0x37=byte basso) usare 2'b01 per NON azzerare il byte
	// adiacente (0x36 = divisore/ampiezza letto da 42c2 e sin/cos).
	output reg   [1:0] dma_ram_be,

	// DMA SOURCE read: byte address on 68K bus, decoded by main_top mux
	output reg  [23:0] dma_src_byte /*verilator public_flat_rd*/,
	input  wire [15:0] dma_src_rdata,

	// ROM read (descrittori hitbox b100/b900 sono in MAIN ROM): handshake
	// req/ready via sub-rom port SDRAM (latency variabile). Vedi M_B1_* states.
	output reg  [23:0] cop_rom_addr /*verilator public_flat_rd*/,
	output reg         cop_rom_req /*verilator public_flat_rd*/,
	input  wire [15:0] cop_rom_rdata,
	input  wire        cop_rom_ready,

	// DMA master — Spriteram (write only)
	output reg  [10:0] dma_spr_addr,
	output reg  [15:0] dma_spr_wdata,
	output reg         dma_spr_we,

	// DMA master — VRAM tilemap (cmd 0x14)
	output reg  [12:0] dma_vram_addr,
	output reg  [15:0] dma_vram_wdata,
	output reg         dma_vram_we,

	// VRAM write-intent for the CURRENT cycle (combinational). The registered
	// dma_vram_we/_addr above are 1 cycle LATE: asserted during the NEXT D_READ,
	// hijacking the shared M10K port-A address (read src vs write dst collide on
	// real HW). main_top drives the bg/fg/mg/txt port-A mux from these instead so
	// the write commits in D_WRITE(i) using src_rdata (mem[src_i], valid now),
	// freeing the next D_READ for the src fetch. Fixes tilemap shift dst[i]=src[i-1].
	output wire [12:0] dma_vram_addr_now,
	output wire        dma_vram_we_now,

	// DMA master — Palette renderer (cmd 0x15 only)
	output reg  [10:0] dma_pal_addr,
	output reg  [15:0] dma_pal_wdata,
	output reg         dma_pal_we,

	// DMA master — Palette STAGING (cmd 0x80-0x87 pre-fade scratch copy)
	// Shares addr+wdata bus with dma_pal_*; only the we strobe differs.
	output reg         dma_pal_stage_we,

	// FILL 0x116/0x118 verso VRAM/PAL/SPR staging ($101000-$105FFF): il fill
	// scrive le BRAM scratch via main_top (pattern HeatedBarrel CONFERMATO HW:
	// senza, gli sprite/tilemap vecchi NON si cancellano → duplicati/residui).
	// dma_fill_addr = word address 68k assoluto (byte>>1).
	output wire        dma_fill_we,
	output wire [22:0] dma_fill_addr,
	output wire [15:0] dma_fill_wdata,

	// Busy: high during cmd or DMA execution (FSM-based ONLY).
	// Drives the port-A MUXes in main_top (RAM/VRAM/sprite/palette hijack):
	// must reflect the cycles where cop_dma_* actually drive valid values.
	output wire        dma_busy,

	// CPU stall: dma_busy OR the combinational trigger pulse. Asserted ONE cycle
	// EARLIER than dma_busy (already in the trigger write cycle) to close the
	// 1-cycle DTACK hole between back-to-back COP commands (a180/a980/b100/b900
	// confine check). Used ONLY for the CPU bus_busy stall in main_top — NEVER
	// for the port MUXes (which would hijack ports while cop_dma_* are stale).
	output wire        cpu_stall
);

	// ────────────────────────────────────────────────────────────────────────
	// Register file
	// ────────────────────────────────────────────────────────────────────────

	// cop_regs[0..6] — 7 pointers, 32-bit each (high/low banked at 0x4A0/0x4C0)
	reg [15:0] cop_reg_hi [0:6] /*verilator public_flat_rd*/;
	reg [15:0] cop_reg_lo [0:6] /*verilator public_flat_rd*/;

	// Per-mode DMA params (mode index = cop_dma_mode[7:0], range 0..255)
	reg [15:0] cop_dma_src   [0:255];
	reg [15:0] cop_dma_size  [0:255];
	reg [15:0] cop_dma_dst   [0:255];

	// Misc registers
	reg [8:0]  cop_dma_mode;        // selects bank for src/size/dst
	// Voci selezionate dal mode, REGISTRATE: indicizzare array da 256
	// dentro il ciclo del trigger significa mettere tre mux 256:1 in serie
	// col decode del dispatcher, ed era il percorso critico del core.
	// Il mode e le voci arrivano da scritture CPU, distanti molti cicli
	// dal trigger: quando serve, il valore registrato e' gia' quello giusto.
	reg [15:0] dma_src_sel, dma_size_sel, dma_dst_sel;
	reg [15:0] cop_dma_adr_rel;
	reg [15:0] cop_dma_v1;
	reg [15:0] cop_dma_v2;
	reg [15:0] cop_scale;           // only bits 1..0 effective
	reg [15:0] cop_angle_target;
	reg [15:0] cop_angle_step;
	// $100446/$100448 = OPERANDO B a 32 bit (16.16) della famiglia moltiplicatore
	// 5105/f105 — NON e' un indirizzo ROM (il gioco ci mette 0.75, 5/6, 2/g...).
	// $10044A = precmd: il gioco lo mette a $F prima di questi comandi e lo
	// riazzera dopo; solo latch, non gattare l'esecuzione su di esso.
	reg [15:0] cop_opnd_hi;
	reg [15:0] cop_opnd_lo;
	reg [15:0] cop_precmd;
	// Z-sorting DMA: registri $100450-$100458 (MAME cop_sort_*).
	reg [15:0] cop_sort_ram_hi, cop_sort_ram_lo;   // $100450 / $100452
	reg [15:0] cop_sort_lu_hi,  cop_sort_lu_lo;    // $100454 / $100456
	reg [15:0] cop_sort_param;                     // $100458 (1=cresc, 2=decresc)
	reg [15:0] cop_pal_brightness_val;
	reg [15:0] cop_pal_brightness_mode;
	reg [15:0] cop_unk_param_a;
	reg [15:0] cop_unk_param_b;
	reg [15:0] cop_hit_baseadr /*verilator public_flat_rd*/;
	reg [15:0] cop_prng_maxvalue;
	reg [15:0] cop_itoa_low;
	reg [15:0] cop_itoa_high;
	reg [15:0] cop_itoa_mode;
	// MAME cop_itoa_digits[10]: BCD digits of cop_itoa (low<<0 | high<<16).
	// 9 digits (0..8) + terminator (9)=0. Read via cop_itoa_digits_r at 0x100590.
	reg [7:0]  cop_itoa_digits [0:9];
	// BCD update FSM: triggered by write to cop_itoa_low/high.
	// Double-dabble a TEMPO COSTANTE (32 shift + 1 commit = 33 clk). Il vecchio
	// loop a sottrazioni ripetute era O(valore) (~valore/9 clk): con i long
	// 32-bit di godzilla (score, writer $156E/$1F93E) il 68k restava congelato
	// (dma_busy include bcd_pending) fino a MILLISECONDI per chiamata — su
	// MAME/HW reale i digit sono pronti subito (il gioco li legge 3 istruzioni
	// dopo la write). Main loop gonfiato -> la coda (camera $B2A0) sconfinava
	// nell'IRQ4 -> redraw/commit spaiati -> pop 16px scroll FG/MG.
	reg        bcd_pending;
	reg [5:0]  bcd_step;       // 0..31 = shift double-dabble, 32 = commit digit
	reg [31:0] bcd_val;        // valore binario residuo (shift left)
	reg [35:0] bcd_acc;        // 9 nibble BCD (double-dabble)

	// ────────────────────────────────────────────────────────────────────────
	// MAME cop_program command table (seibucop.cpp lines 318-457)
	// At boot, game code uploads 32 macro slots × 8 micro-op words via
	//   pgm_addr  (0x100434) → slot*8 + sub-index
	//   pgm_data  (0x100432) → micro-op word
	//   pgm_value (0x100438) → cop_func_value[slot]
	//   pgm_mask  (0x10043A) → cop_func_mask[slot]
	//   pgm_trigger (0x10043C) → cop_func_trigger[slot]
	// Then game writes a trigger value to 0x100500 (cmd_w). MAME calls
	// find_trigger_match(triggerval, 0xff00) to find the slot whose
	// cop_func_trigger matches. The slot's micro-ops decide what runs.
	//
	// Our RTL keeps the hardcoded math dispatch (sin/cos/atan2/dist/etc.)
	// but uses the table to translate the raw triggerval written by the
	// game into the "canonical" triggerval understood by the dispatch.
	// This unblocks game code that writes triggerval with different LSBs
	// (e.g. 0x8123 instead of 0x8100) — the table maps both to slot N
	// whose canonical trigger is 0x8100.
	reg [15:0] cop_program       [0:255];  // 32 slots × 8 micro-op words
	reg [15:0] cop_func_trigger  [0:31];
	reg [15:0] cop_func_value    [0:31];
	reg [15:0] cop_func_mask     [0:31];
	reg [15:0] cop_latch_trigger;
	reg [15:0] cop_latch_value;
	reg [15:0] cop_latch_mask;
	reg [7:0]  cop_latch_addr;

	// Sprite DMA latches (stubs — SeibuCup ROM doesn't trigger sprite DMA via COP)
	reg [15:0] cop_spr_dma_param_lo, cop_spr_dma_param_hi;
	reg [15:0] cop_spr_dma_size;
	reg [15:0] cop_spr_dma_src_hi,   cop_spr_dma_src_lo;
	reg [15:0] cop_spr_dma_abs_x,    cop_spr_dma_abs_y;

	// Read-back status
	reg [15:0] cop_status /*verilator public_flat_rd*/;
	reg [15:0] cop_angle;
	reg [15:0] cop_dist;

	// Collision state
	// pos[slot][axis], allow_swap[slot], flags_swap[slot]
	reg [15:0] coll_pos    [0:1][0:2] /*verilator public_flat_rd*/;      // pos word for slot 0,1 — axes Y,X,Z
	reg        coll_allow_swap [0:1] /*verilator public_flat_rd*/;
	reg [15:0] coll_flags_swap [0:1] /*verilator public_flat_rd*/;
	reg [15:0] coll_min    [0:1][0:2] /*verilator public_flat_rd*/;
	reg [15:0] coll_max    [0:1][0:2] /*verilator public_flat_rd*/;
	reg [15:0] cop_hit_status /*verilator public_flat_rd*/;
	reg [15:0] cop_hit_val_stat /*verilator public_flat_rd*/;
	reg [15:0] cop_hit_val [0:2] /*verilator public_flat_rd*/;

	// PRNG: LFSR 16-bit free-running (maximal, taps 16/15/13/4).
	// NON contatore semplice: il gioco legge $1005A4 a cadenza FISSA
	// (script per-frame, 191 call sites in cupsoc) → un counter & maxvalue
	// va in lock-step (pochi residui ciclici) e uno spawn che aspetta un
	// valore random specifico non arriva MAI (boss stage 4 cupsoc).
	// MAME usa total_cycles%256 (surrogato dell'RNG hw ignoto): l'LFSR e'
	// un surrogato equivalente ma robusto alle letture regolari.
	reg [15:0] prng_lfsr;
	always @(posedge clk) begin
		if (reset) prng_lfsr <= 16'hACE1;
		else       prng_lfsr <= {prng_lfsr[14:0],
		                         prng_lfsr[15] ^ prng_lfsr[14] ^ prng_lfsr[12] ^ prng_lfsr[3]};
	end

	// RIDUZIONE AL RANGE 0..maxvalue — NON una maschera.
	// Il gioco programma cop_prng_maxvalue = 99 una sola volta, al boot
	// (ROM $000682 `move.w #$0063,$2C(a1)` con a1 = $00100400; e' l'unica
	// scrittura a $10042C in tutto il MB). 99 = 7'b1100011 NON e' 2^n-1,
	// quindi `lfsr & 99` lascia i bit 2,3,4 costantemente a ZERO: escono
	// solo 16 valori su 100.
	// Che il gioco pretenda il range PIENO lo dicono i suoi stessi confronti:
	//   - dispatcher a quintili $059A80/$0631FA/$0633F2 (cmpi #20/#40/#60),
	//     dove con la maschera la fascia [40,59] e' IRRAGGIUNGIBILE;
	//   - soglie cmpi.b #$4B (75) a $01E5DA/$01E600;
	//   - sei test su singolo bit che con la maschera non sono mai veri:
	//     andi.w #4 ($015C56,$015CAE), #$1C ($018AB8/AE0/B08/B30),
	//     #8 ($018DCE,$018EB8).
	// La scala moltiplicativa copre 0..maxvalue in modo uniforme (LFSR
	// massimale, x in 1..65535) ed e' esatta anche per maxvalue = 2^n-1,
	// che e' il default di reset. Il bit 32 e' sempre 0 perche' il prodotto
	// vale al massimo 65535*65536 < 2^32.
	wire [32:0] prng_scaled = prng_lfsr * ({1'b0, cop_prng_maxvalue} + 17'd1);

	// ────────────────────────────────────────────────────────────────────────
	// CPU bus decode
	// ────────────────────────────────────────────────────────────────────────
	wire [9:0] widx = addr;          // word offset
	// Edge-detect cs+wr combinato (rising = nuovo write bus cycle). 68k bus
	// pattern: AS_n giù → DSn giù (1/2 ciclo dopo). Edge solo su cs perde
	// il primo ciclo se wr non ancora attivo. Senza edge, dma_pending si
	// re-arma in loop quando CPU stalla → DMA infinito.
	wire       cs_wr_now = cs & wr;
	reg        cs_wr_prev;
	always @(posedge clk) begin
		if (reset) cs_wr_prev <= 1'b0;
		else       cs_wr_prev <= cs_wr_now;
	end
	wire       cpu_wr_pulse = cs_wr_now & ~cs_wr_prev;

	// ── Double-dabble helpers (BCD itoa) ─────────────────────────────────────
	// dd_adjust: per ogni nibble >= 5 somma 3 (pre-shift, algoritmo standard).
	// 9 adder a 4 bit in parallelo: path banale, nessun loop dipendente.
	function [35:0] dd_adjust(input [35:0] a);
		integer k;
		reg [3:0] n;
		begin
			for (k = 0; k < 9; k = k + 1) begin
				n = a[k*4 +: 4];
				if (n >= 4'd5) n = n + 4'd3;
				dd_adjust[k*4 +: 4] = n;
			end
		end
	endfunction
	// dd_leadzero[k]=1 <=> nibble k..8 tutti zero (= valore < 10^k): il digit k
	// sopra il MSD e' filler. Identico al test del vecchio loop
	// (bcd_val==0 && bcd_quotient==0 && step!=0). Digit 0 mai filler.
	function [8:0] dd_leadzero(input [35:0] a);
		integer k;
		reg z;
		begin
			z = 1'b1;
			for (k = 8; k >= 1; k = k - 1) begin
				z = z & (a[k*4 +: 4] == 4'd0);
				dd_leadzero[k] = z;
			end
			dd_leadzero[0] = 1'b0;
		end
	endfunction
	wire [35:0] bcd_dd_adj    = dd_adjust(bcd_acc);
	wire [8:0]  bcd_lead_zero = dd_leadzero(bcd_acc);

	// Triggers (write-only addresses)
	// MAME legionna.cpp:172 maps cop_cmd_w to $100500-$100505 (3 words).
	// Game ROM writes sequential macros to all 3 offsets (verified by static
	// decode: $100500 14 writes, $100502 4 writes, $100504 2 writes).
	wire trig_macro    = cpu_wr_pulse && (widx == 10'h080 ||
	                                       widx == 10'h081 ||
	                                       widx == 10'h082);   // 0x100500/02/04
	wire trig_dma      = cpu_wr_pulse && (widx == 10'h17E);    // 0x1006FC
	wire trig_sort_dma = cpu_wr_pulse && (widx == 10'h17F);    // 0x1006FE z-sort DMA
	wire trig_spr_inc  = cpu_wr_pulse && (widx == 10'h008);    // 0x100410

	// ────────────────────────────────────────────────────────────────────────
	// CPU writes
	// ────────────────────────────────────────────────────────────────────────
	integer i_r, dd_k;
	always @(posedge clk) begin
		if (reset) begin
			// Mass reset
			for (i_r = 0; i_r < 7;   i_r = i_r + 1) begin
				cop_reg_hi[i_r] <= 16'd0;
				cop_reg_lo[i_r] <= 16'd0;
			end
			for (i_r = 0; i_r < 256; i_r = i_r + 1) begin
				cop_dma_src [i_r] <= 16'd0;
				cop_dma_size[i_r] <= 16'd0;
				cop_dma_dst [i_r] <= 16'd0;
				cop_program [i_r] <= 16'd0;
			end
			for (i_r = 0; i_r < 32; i_r = i_r + 1) begin
				cop_func_trigger[i_r] <= 16'd0;
				cop_func_value  [i_r] <= 16'd0;
				cop_func_mask   [i_r] <= 16'd0;
			end
			cop_latch_addr    <= 8'd0;
			cop_latch_trigger <= 16'd0;
			cop_latch_value   <= 16'd0;
			cop_latch_mask    <= 16'd0;
			cop_dma_mode             <= 9'd0;
			cop_dma_adr_rel          <= 0;
			cop_dma_v1               <= 0;
			cop_dma_v2               <= 0;
			cop_scale                <= 0;
			cop_angle_target         <= 0;
			cop_angle_step           <= 0;
			cop_opnd_hi              <= 0;
			cop_opnd_lo              <= 0;
			cop_precmd               <= 0;
			cop_sort_ram_hi          <= 0;
			cop_sort_ram_lo          <= 0;
			cop_sort_lu_hi           <= 0;
			cop_sort_lu_lo           <= 0;
			cop_sort_param           <= 0;
			cop_pal_brightness_val   <= 0;
			cop_pal_brightness_mode  <= 0;
			cop_unk_param_a          <= 0;
			cop_unk_param_b          <= 0;
			cop_hit_baseadr          <= 0;
			cop_prng_maxvalue        <= 16'h00FF;
			cop_itoa_low             <= 0;
			cop_itoa_high            <= 0;
			cop_itoa_mode            <= 0;
			cop_itoa_digits[0]<=8'h30; cop_itoa_digits[1]<=8'h20;
			cop_itoa_digits[2]<=8'h20; cop_itoa_digits[3]<=8'h20;
			cop_itoa_digits[4]<=8'h20; cop_itoa_digits[5]<=8'h20;
			cop_itoa_digits[6]<=8'h20; cop_itoa_digits[7]<=8'h20;
			cop_itoa_digits[8]<=8'h20; cop_itoa_digits[9]<=8'h00;
			bcd_pending <= 0; bcd_step <= 0; bcd_val <= 0;
			bcd_acc <= 0;
			cop_spr_dma_param_lo     <= 0;
			cop_spr_dma_param_hi     <= 0;
			cop_spr_dma_size         <= 0;
			cop_spr_dma_src_hi       <= 0;
			cop_spr_dma_src_lo       <= 0;
			cop_spr_dma_abs_x        <= 0;
			cop_spr_dma_abs_y        <= 0;
		end else if (cpu_wr_pulse) begin
			case (widx)
				// Sprite DMA latches (treated as plain RW latches)
				10'h000: cop_spr_dma_param_lo <= wdata;  // 0x100400
				10'h001: cop_spr_dma_param_hi <= wdata;  // 0x100402
				10'h006: cop_spr_dma_size     <= wdata;  // 0x10040C
				10'h008: begin
					// 0x100410 sprite_dma_inc (MAME cop_sprite_dma_inc_w):
					//   if x_clip in [-160, 320): cop_regs[4] += 8. MA in MAME
					//   sprite_x e' uint16_t assegnato a int -> MAI negativo ->
					//   il -160 e' codice morto: check EFFETTIVO = unsigned < 320
					//   (X "negative" 0xFFxx NON avanzano il puntatore entry).
					//   cop_sprite_dma_src += 6; cop_sprite_dma_size--;
					//   cop_status bit 1 = (size > 0) ? 0 : 1
					// (unsigned <320 provato su HW = REGRESSIONE; il signed [-160,320)
					// dell'intento MAME e' il comportamento giusto sul core)
					if (x_clip_r >= -16'sd160 && x_clip_r < 16'sd320)
						cop_reg_lo[4]  <= cop_reg_lo[4] + 16'd8;
					{cop_spr_dma_src_hi, cop_spr_dma_src_lo} <=
						{cop_spr_dma_src_hi, cop_spr_dma_src_lo} + 32'd6;
					// size>0: size-- e status[1]=(size era 1). A size==0 NON wrappare
					// (il size-- uint di MAME wrapperebbe a $FFFF: il loop del
					// builder su status[1] andrebbe in livelock — provato peggio).
					if (cop_spr_dma_size != 16'd0) begin
						cop_spr_dma_size <= cop_spr_dma_size - 16'd1;
						if (cop_spr_dma_size == 16'd1) cop_status[1] <= 1'b1;
						else                            cop_status[1] <= 1'b0;
					end else begin
						cop_status[1] <= 1'b1;
					end
				end
				10'h009: cop_spr_dma_src_hi   <= wdata;  // 0x100412
				10'h00A: cop_spr_dma_src_lo   <= wdata;  // 0x100414
				10'h00E: cop_angle_target     <= wdata;  // 0x10041C
				10'h00F: cop_angle_step       <= wdata;  // 0x10041E
				10'h028: cop_sort_ram_hi      <= wdata;  // 0x100450 sort_ram_addr hi
				10'h029: cop_sort_ram_lo      <= wdata;  // 0x100452 sort_ram_addr lo
				10'h02A: cop_sort_lu_hi       <= wdata;  // 0x100454 lookup hi
				10'h02B: cop_sort_lu_lo       <= wdata;  // 0x100456 lookup lo
				10'h02C: cop_sort_param       <= wdata;  // 0x100458 param (1=cresc)
				10'h023: cop_opnd_hi          <= wdata;  // 0x100446 operando B hi
				10'h024: cop_opnd_lo          <= wdata;  // 0x100448 operando B lo
				10'h025: cop_precmd           <= wdata;  // 0x10044A
				10'h010: begin
					cop_itoa_low <= wdata;                            // 0x100420
					bcd_pending  <= 1'b1;
					bcd_step     <= 6'd0;
					bcd_val      <= {cop_itoa_high, wdata};
					bcd_acc      <= 36'd0;
				end
				10'h011: begin
					cop_itoa_high <= wdata;                           // 0x100422
					bcd_pending   <= 1'b1;
					bcd_step      <= 6'd0;
					bcd_val       <= {wdata, cop_itoa_low};
					bcd_acc       <= 36'd0;
				end
				10'h012: cop_itoa_mode        <= wdata;  // 0x100424
				10'h014: cop_dma_v1           <= wdata;  // 0x100428
				10'h015: cop_dma_v2           <= wdata;  // 0x10042A
				10'h016: cop_prng_maxvalue    <= wdata;  // 0x10042C
				10'h01B: cop_hit_baseadr      <= wdata;  // 0x100436

				// Macro program upload — MAME seibucop.cpp lines 318-457
				// pgm_data writes one micro-op word to cop_program[latch_addr]
				//   AND latches the slot's trigger/value/mask from current latch regs.
				10'h019: begin                            // 0x100432 pgm_data
					cop_program[cop_latch_addr] <= wdata;
					cop_func_trigger[cop_latch_addr[7:3]] <= cop_latch_trigger;
					cop_func_value  [cop_latch_addr[7:3]] <= cop_latch_value;
					cop_func_mask   [cop_latch_addr[7:3]] <= cop_latch_mask;
				end
				10'h01A: cop_latch_addr    <= wdata[7:0]; // 0x100434 pgm_addr
				10'h01C: cop_latch_value   <= wdata;      // 0x100438 pgm_value
				10'h01D: cop_latch_mask    <= wdata;      // 0x10043A pgm_mask
				10'h01E: cop_latch_trigger <= wdata;      // 0x10043C pgm_trigger

				// Misc
				10'h020: cop_unk_param_a            <= wdata;  // 0x100440
				10'h021: cop_unk_param_b            <= wdata;  // 0x100442
				10'h022: cop_scale                  <= wdata & 16'h0003;  // 0x100444
				10'h02D: cop_pal_brightness_val     <= wdata;  // 0x10045A
				10'h02E: cop_pal_brightness_mode    <= wdata;  // 0x10045C
				10'h03B: cop_dma_adr_rel            <= wdata;  // 0x100476
				10'h03C: cop_dma_src [cop_dma_mode[7:0]] <= wdata;  // 0x100478
				10'h03D: cop_dma_size[cop_dma_mode[7:0]] <= wdata;  // 0x10047A
				10'h03E: cop_dma_dst [cop_dma_mode[7:0]] <= wdata;  // 0x10047C
				10'h03F: cop_dma_mode               <= wdata[8:0];  // 0x10047E
				10'h046: cop_spr_dma_abs_y          <= wdata;  // 0x10048C
				10'h047: cop_spr_dma_abs_x          <= wdata;  // 0x10048E

				// cop_regs hi (0x1004A0..0x1004AD = widx 0x50..0x56)
				10'h050: cop_reg_hi[0] <= wdata;
				10'h051: cop_reg_hi[1] <= wdata;
				10'h052: cop_reg_hi[2] <= wdata;
				10'h053: cop_reg_hi[3] <= wdata;
				10'h054: cop_reg_hi[4] <= wdata;
				10'h055: cop_reg_hi[5] <= wdata;
				10'h056: cop_reg_hi[6] <= wdata;

				// cop_regs lo (0x1004C0..0x1004CD = widx 0x60..0x66)
				10'h060: cop_reg_lo[0] <= wdata;
				10'h061: cop_reg_lo[1] <= wdata;
				10'h062: cop_reg_lo[2] <= wdata;
				10'h063: cop_reg_lo[3] <= wdata;
				10'h064: cop_reg_lo[4] <= wdata;
				10'h065: cop_reg_lo[5] <= wdata;
				10'h066: cop_reg_lo[6] <= wdata;

				default: ;
			endcase
		end

		// BCD update step (separato dal cpu_wr_pulse block).
		// Race protection: skip se cpu_wr_pulse appena scrive itoa low/high (i.e.
		// widx 0x010/0x011), perché il write CPU ha già azzerato bcd_step e
		// bcd_val viene sovrascritto. Il blocco sotto NON deve toccare bcd_val
		// nello stesso ciclo del write CPU.
		if (bcd_pending && !(cpu_wr_pulse && (widx == 10'h010 || widx == 10'h011))) begin
			if (bcd_step != 6'd32) begin
				// double-dabble: adjust nibble (>=5 -> +3) poi shift-in del MSB
				bcd_acc  <= {bcd_dd_adj[34:0], bcd_val[31]};
				bcd_val  <= {bcd_val[30:0], 1'b0};
				bcd_step <= bcd_step + 6'd1;
			end else begin
				// commit (1 ciclo): digit k = '0'+nibble; sopra il MSD filler ' '
				// (mode 3: '0'). Semantica IDENTICA al vecchio loop: filler <=>
				// valore < 10^k (provato: N_k=floor(V/10^k)==0), digit 0 sempre reale.
				for (dd_k = 0; dd_k < 9; dd_k = dd_k + 1) begin
					if (bcd_lead_zero[dd_k])
						cop_itoa_digits[dd_k] <= (cop_itoa_mode == 16'd3) ? 8'h30 : 8'h20;
					else
						cop_itoa_digits[dd_k] <= {4'h3, bcd_acc[dd_k*4 +: 4]};
				end
				bcd_pending <= 1'b0;
			end
		end
	end

	// ────────────────────────────────────────────────────────────────────────
	// CPU reads — combinational on `addr`, registered output `rdata`
	// ────────────────────────────────────────────────────────────────────────
	always @(posedge clk) begin
		if (reset) begin
			rdata <= 16'hFFFF;
		end else if (cs && rd) begin
			case (widx)
				10'h016: rdata <= cop_prng_maxvalue;
				10'h03F: rdata <= {7'd0, cop_dma_mode};
				// cop_regs read-back
				10'h050: rdata <= cop_reg_hi[0];
				10'h051: rdata <= cop_reg_hi[1];
				10'h052: rdata <= cop_reg_hi[2];
				10'h053: rdata <= cop_reg_hi[3];
				10'h054: rdata <= cop_reg_hi[4];
				10'h055: rdata <= cop_reg_hi[5];
				10'h056: rdata <= cop_reg_hi[6];
				10'h060: rdata <= cop_reg_lo[0];
				10'h061: rdata <= cop_reg_lo[1];
				10'h062: rdata <= cop_reg_lo[2];
				10'h063: rdata <= cop_reg_lo[3];
				10'h064: rdata <= cop_reg_lo[4];
				10'h065: rdata <= cop_reg_lo[5];
				10'h066: rdata <= cop_reg_lo[6];
				// itoa_digits (MAME 0x100590-99, 5 word). digits[offset*2] | (digits[offset*2+1]<<8)
				10'h0C8: rdata <= {cop_itoa_digits[1], cop_itoa_digits[0]};  // 0x100590
				10'h0C9: rdata <= {cop_itoa_digits[3], cop_itoa_digits[2]};  // 0x100592
				10'h0CA: rdata <= {cop_itoa_digits[5], cop_itoa_digits[4]};  // 0x100594
				10'h0CB: rdata <= {cop_itoa_digits[7], cop_itoa_digits[6]};  // 0x100596
				10'h0CC: rdata <= {cop_itoa_digits[9], cop_itoa_digits[8]};  // 0x100598
				// Collision status
				10'h0C0: rdata <= cop_hit_status;    // 0x100580
				10'h0C1: rdata <= cop_hit_val[0];    // 0x100582 dY
				10'h0C2: rdata <= cop_hit_val[1];    // 0x100584 dX
				10'h0C3: rdata <= cop_hit_val[2];    // 0x100586 dZ
				10'h0C4: rdata <= cop_hit_val_stat;  // 0x100588
				// Status / dist / angle (seibucop.cpp:916 cop_status_r → cop_status)
				10'h0D8: rdata <= cop_status;        // 0x1005B0
				10'h0D9: rdata <= cop_dist;          // 0x1005B2
				10'h0DA: rdata <= cop_angle;         // 0x1005B4
				// PRNG ridotto al range 0..maxvalue (vedi prng_scaled).
				10'h0D0,10'h0D1,10'h0D2,10'h0D3: begin
					rdata <= prng_scaled[31:16];
				end
				default: rdata <= 16'hFFFF;  // MAME open bus
			endcase
		end
	end

	// ────────────────────────────────────────────────────────────────────────
	// FSM — serializes macro commands and DMA so port B / port A are
	// never used concurrently.
	// ────────────────────────────────────────────────────────────────────────
	localparam [6:0]
		S_IDLE       = 6'd0,
		// DMA paths
		D_PREP       = 6'd1,
		D_READ       = 6'd2,
		D_WRITE      = 6'd3,
		D_FILL_W     = 6'd4,
		// Macro: 0x0205 movement
		M_0205_RD_PPOS_HI = 6'd10,
		M_0205_RD_PPOS_LO = 6'd11,
		M_0205_RD_VEL_HI  = 6'd12,
		M_0205_RD_VEL_LO  = 6'd13,
		M_0205_WR_NPOS_HI = 6'd14,
		M_0205_WR_NPOS_LO = 6'd15,
		M_0205_RD_SCRN    = 6'd16,
		M_0205_WR_SCRN    = 6'd17,
		// Macro: 0x0905 jump (add velocity to dy)
		M_0905_RD_HI      = 6'd18,
		M_0905_RD_LO      = 6'd19,
		M_0905_RD_GRAV_HI = 6'd20,
		M_0905_RD_GRAV_LO = 6'd21,
		M_0905_WR_HI      = 6'd22,
		M_0905_WR_LO      = 6'd23,
		// Macro: 0x138e atan
		M_138E_RD0_Y_HI = 6'd24,
		M_138E_RD0_Y_LO = 6'd25,
		M_138E_RD1_Y_HI = 6'd26,
		M_138E_RD1_Y_LO = 6'd27,
		M_138E_RD0_X_HI = 6'd28,
		M_138E_RD0_X_LO = 6'd29,
		M_138E_RD1_X_HI = 6'd30,
		M_138E_RD1_X_LO = 6'd31,
		M_138E_CALC     = 6'd32,
		// Macro: 0x3bb0 dist
		M_3BB0_LOAD     = 6'd33,
		M_3BB0_CALC     = 6'd34,
		// Macro: 0x42c2 divide
		M_42C2_RD_DIV   = 6'd35,
		M_42C2_CALC     = 6'd36,
		// Macro: 0x8100/0x8900 sin/cos
		M_SC_RD_ANG     = 6'd37,
		M_SC_RD_AMP     = 6'd38,
		M_SC_CALC       = 6'd39,
		M_SC_WR_HI      = 6'd40,
		M_SC_WR_LO      = 6'd41,
		// Macro: 0xa180/0xa980 collision read pos
		M_A1_RD_FLAGS   = 6'd42,
		M_A1_RD_POS_Y   = 6'd43,
		M_A1_RD_POS_X   = 6'd44,
		M_A1_RD_POS_Z   = 6'd45,
		M_A1_RD_POS_ZW  = 6'd54,    // wait extra ciclo per latchare pos[2]
		// Macro: 0xb100/0xb900 collision update hitbox (stati REQ/WAIT ROM piu sotto)
		M_B1_CALC       = 6'd50,
		// DMA cmd 0x80 mode 5 palette fade: 2 read (src + target) + write blend
		D_FADE_RD_TGT   = 6'd51,    // emit target addr, latch paldata
		D_FADE_WAIT_TGT = 6'd52,    // wait target rdata
		D_FADE_CALC     = 7'd64,    // pipeline: calcola 6 fade_table (registra)
		D_FADE_WRITE    = 6'd53,    // pipeline: somme, registra fade_out_val
		D_FADE_COMMIT   = 7'd68,    // commit write al dst ASSOLUTO (fill/mr), 1:1 MAME write_word(dst)
		// (6'd54 = M_A1_RD_POS_ZW, già definito sopra)
		// atan 138e/338e: riusa gli stati M_138E_RD0_Y_HI..M_138E_CALC (24-32)
		//   per le 4 letture dword + il loop CORDIC dentro M_138E_CALC.
		// dist 3bb0: riusa M_3BB0_LOAD..M_3BB0_CALC (33-34) + stati extra qui sotto.
		// divide 42c2/4aa0: riusa M_42C2_RD_DIV..M_42C2_CALC (35-36) + extra.
		M_3BB0_RD_X1LO  = 6'd55,    // letture dword dx/dy per dist
		M_3BB0_RD_X0HI  = 6'd56,
		M_3BB0_RD_X0LO  = 6'd57,
		M_3BB0_RD_Y1HI  = 6'd58,
		M_3BB0_RD_Y1LO  = 6'd59,
		M_3BB0_RD_Y0HI  = 6'd60,
		M_3BB0_RD_Y0LO  = 6'd61,
		M_3BB0_SQRT     = 6'd62,    // restoring sqrt loop (16 iter via contatore)
		M_42C2_DIV      = 6'd63,    // divide loop (16 iter via contatore)
		M_3BB0_WR       = 6'd5,     // dist: write word cop_dist + set status
		M_42C2_WR       = 6'd6,     // divide: write result word (42c2/4aa0)
		M_3BB0_CALC2    = 6'd7,     // dist: stage 1 (differenze)
		M_3BB0_CALCM    = 6'd9,     // dist: stage 2 (moltiplicazioni)
		M_3BB0_CALC3    = 6'd8,     // dist: stage 3 (somma + init sqrt)
		M_138E_CORDIC   = 7'd65,    // atan: loop CORDIC vectoring 1 iter/ciclo
		M_138E_WR       = 7'd66,    // atan: write cop_angle + byte r0+0x34
		M_0205_WAIT_SCRN = 7'd67,   // 0205: wait latency BRAM su read 0x1C (scroll)
		M_B1_CALC2      = 7'd69,    // hitbox: stage 2 (comparatori overlap, path corto)
		// Hitbox b100/b900: TUTTE le letture (puntatore + 3 assi descrittore) sono
		// in MAIN ROM -> handshake req/ready via cop_rom_* (latency variabile).
		// Sequenza: REQ (1 ciclo req) -> WAIT (stalla finche ready, latcha) -> next.
		M_B1_REQ_PTR    = 7'd70,    // emit addr puntatore, req
		M_B1_WAIT_PTR   = 7'd71,    // wait ready, latcha puntatore -> hb_adr2
		M_B1_REQ_H0     = 7'd72,
		M_B1_WAIT_H0    = 7'd73,
		M_B1_REQ_H1     = 7'd74,
		M_B1_WAIT_H1    = 7'd75,
		M_B1_REQ_H2     = 7'd76,
		M_B1_WAIT_H2    = 7'd77,
		M_0905_RD0      = 7'd78,    // 0905: primo read vel_hi con macro_offset (coerente)
		// LEGACY sprite-DMA (seibucop_cmd.ipp): 6880 (carica rel_xy),
		// c480 (single-step: compone l'entry sprite a cop_regs[4]+offs con
		// X/Y = rel + (obj - abs)). Usati da godzilla per TUTTI gli oggetti
		// mobili (pietre del title incluse). Mask match: 6880&FFF3, C4xx&FF00.
		M_68_WAIT       = 7'd80,
		M_68_LATCH      = 7'd81,
		M_C4_A          = 7'd82,
		M_C4_B          = 7'd83,
		M_C4_WRI        = 7'd84,    // write info @ entry+0 (abs)
		M_C4_C          = 7'd85,
		M_C4_D          = 7'd86,
		M_C4_WRX        = 7'd87,    // write X @ entry+4 (abs)
		M_C4_E          = 7'd88,
		M_C4_WRY        = 7'd89,    // write Y @ entry+6 (abs)
		M_C4_F          = 7'd90,    // (param bit17) read entry+2
		M_C4_G          = 7'd91,    // latch, |0x8000
		M_C4_WRP        = 7'd92,    // write pri @ entry+2 (abs)
		// 6200 (gruppo $6000-$67FF, microcodice cupsoc-variant caricato da
		// godzilla): ruota l'angolo obj+0x34 verso cop_angle_target di
		// cop_angle_step per chiamata; flags(regs[0] host+2): bit2 = raggiunto.
		M_62_A          = 7'd93,
		M_62_B          = 7'd94,
		M_62_C          = 7'd95,
		M_62_D          = 7'd96,    // flags &~4 |reached, write flags
		M_62_E          = 7'd97,    // write angolo (word {0,angle} come MAME host_endian)
		// 5105/f105 — moltiplicatore 16.16 (ricavato da ROM+disasm, non da MAME:
		// li' gli handler sono vuoti). dst = (src * {$100446,$100448}) >>> 16.
		M_MUL_A         = 7'd98,
		M_MUL_B         = 7'd99,
		M_MUL_C         = 7'd100,
		M_MUL_LOOP      = 7'd101,
		M_MUL_WR_HI     = 7'd102,
		M_MUL_WR_LO     = 7'd103,
		// d104: prima di moltiplicare sottrae l'origine del campo, letta da reg3
		M_MUL_ORG_A     = 7'd104,
		M_MUL_ORG_B     = 7'd105,
		M_MUL_ORG_C     = 7'd106,
		// dde5 — i PALLINI DEL RADAR (MAME LEGACY_execute_dde5).
		M_DDE5_A        = 7'd114,
		M_DDE5_B        = 7'd115,
		M_DDE5_C        = 7'd116,
		M_DDE5_D        = 7'd117,
		M_DDE5_E        = 7'd118,
		M_DDE5_DIV      = 7'd119,
		M_DDE5_WR       = 7'd120,
		// 1 ciclo di attesa dopo una write in MAIN RAM dentro la copia DMA:
		// la Main RAM ha UNA SOLA porta e in main_top l'indirizzo di write
		// vince su quello di lettura, quindi senza attesa la lettura del
		// dato successivo legge dall'indirizzo di DESTINAZIONE.
		D_COPY_WAIT     = 7'd121,
		// Z-sorting DMA ($1006FE): carica (val, key), bubble sort stabile, riscrive.
		M_SORT_LD_A     = 7'd107,
		M_SORT_LD_B     = 7'd108,
		M_SORT_LD_C     = 7'd109,
		M_SORT_LD_D     = 7'd110,
		M_SORT_LD_E     = 7'd111,
		M_SORT_CMP      = 7'd112,
		M_138E_CORDIC_B = 7'd122,   // atan: seconda meta' dell'iterazione CORDIC
		M_0905_WR_HI_B  = 7'd123,   // 0905: seconda meta' (somma alta col carry)
		M_138E_CALC_B   = 7'd124,   // atan: valori assoluti e carico del CORDIC
		M_SORT_SWAP     = 7'd125,   // sort: confronto e scambio su valori registrati
		D_FADE_TAB      = 7'd126,   // fade: tabella (moltiplicatore) su dati registrati
		M_SORT_ST       = 7'd113;

	reg [6:0]  fsm /*verilator public_flat_rd*/;
	reg [5:0]  return_state;     // tail-call target after a read latch
	reg        cmd_pending;
	reg [15:0] cmd_value;
	reg        dma_pending;
	wire [23:0] sort_ram_addr    = {cop_sort_ram_hi[7:0], cop_sort_ram_lo};
	wire [23:0] sort_lookup_addr = {cop_sort_lu_hi[7:0],  cop_sort_lu_lo};
	// Z-sorting: storage (N_MAX 24; cupsoc usa 22 e 11 voci)
	// 60 voci: e' la capacita' delle prime tre liste della tabella $002DB6
	// ($107800/$107878/$1078F0), e il gioco triggera con capacita'-1 = 59
	// ($00364C move.w $4(a0),d1 / subq.w #1,d1). Con 24 il sort dei piani
	// ordinerebbe solo i primi 24 slot su 60 e la profondita' resterebbe
	// sbagliata per meta' dei giocatori.
	reg [15:0]        sort_val [0:59];
	// Coppia letta dall'array, registrata: separa il mux 60:1 dal confronto.
	reg signed [15:0] sort_ka, sort_kb;
	reg        [15:0] sort_va, sort_vb;
	reg signed [15:0] sort_key [0:59];
	reg [5:0]  sort_i, sort_j, sort_pass;
	reg        sort_swapped;
	reg        sort_pending;
	reg [5:0]  sort_n;
	// Extend dma_busy 1 ciclo dopo S_IDLE return per coprire l'ultimo write
	// pulse (dma_pal_we / dma_vram_we / dma_ram_we sono registered: scrittura
	// fisica avviene al posedge DOPO che fsm è tornato S_IDLE).
	reg        dma_busy_tail;
	reg        dma_busy_tail2;   // 2o ciclo di coda: margine per la read CPU dopo commit LSW
	// Trigger latches — MAME seibucop.cpp cop_cmd_w line 1069.
	// MAME dispatches on raw data after clearing cop_status bit 15.
	// PIPELINE: la ricerca canonical su 32 slot era combinational profonda
	// (43 livelli logici, -33ns slack a 96MHz). Spezzata in 3 stage registered:
	//   S1: trig_macro registra wdata in cmd_search_data + bit match[k]
	//   S2: priority encode su 32 match bit → cmd_match_slot[4:0]
	//   S3: lookup cop_func_trigger[slot] → cmd_value
	// (Dichiarati PRIMA dell'uso in dma_busy: ModelSim richiede decl-before-use.)
	reg [15:0] cmd_search_data;
	reg [1:0]  cmd_offset;        // 0..2 (word offset of cmd write: 0x100500/02/04)
	reg [1:0]  macro_offset;      // cmd_offset latchato all'avvio macro (immune a trigger successivi)
	reg [31:0] cmd_match_bits;
	reg [1:0]  cmd_search_state;  // 0=idle, 1=do_match, 2=do_encode, 3=lookup
	reg [4:0]  cmd_match_slot;
	reg        cmd_match_found;
	integer    pe_k;

	// dma_busy_tail a 2 CICLI: l'ultima write COP in Main RAM (es. 0905 LSW $4e,
	// parte bassa di $4c=velZ) si committa nel ciclo S_IDLE DOPO che fsm e' tornato
	// IDLE. Con tail a 1 ciclo dma_busy cadeva NELLO STESSO ciclo del commit LSW ->
	// margine ZERO -> dipendente dal fasamento cpu_cen: nel caso peggiore la CPU
	// rilegge $4c ($7640 move.l $4c,d0 subito dopo il 0905) STANTIO di 1 frame ->
	// la traiettoria Z del salto del bruto diverge -> il rimbalzo converge in 5 colpi
	// invece di 3 -> $48 (rampa decel) attraversa lo zero -> il bruto torna indietro.
	// 2 cicli danno il margine: la read CPU vede SEMPRE la write committata. +1 ciclo
	// di stall per macro (trascurabile). Solo timing della porta, NON logica.
	always @(posedge clk) begin
		if (reset) begin dma_busy_tail <= 1'b0; dma_busy_tail2 <= 1'b0; end
		else begin
			dma_busy_tail  <= (fsm != S_IDLE);
			dma_busy_tail2 <= dma_busy_tail;
		end
	end
	// dma_busy: FSM-based ONLY (drives port MUXes in main_top). True solo nei
	// cicli in cui cop_dma_* pilotano valori VALIDI. NON include i trigger
	// combinatori (che salirebbero quando cop_dma_* sono ancora stale -> i mux
	// dirotterebbero BG/FG/MG/TXT/RAM/sprite verso garbage -> moonwalk/strabordo).
	assign     dma_busy = (fsm != S_IDLE) || cmd_pending || dma_pending || sort_pending || dma_busy_tail ||
	                       dma_busy_tail2 || (cmd_search_state != 2'd0) || bcd_pending;

	// cpu_stall: dma_busy + i trigger combinatori (cpu_wr_pulse). I trigger alzano
	// lo stall GIA' nel ciclo del write del trigger, chiudendo il buco di 1 ciclo
	// tra comandi COP consecutivi (es. b100->b900 nel check-confine a 4 trigger) in
	// cui tutti i termini FSM sono 0 (fsm=S_IDLE, cmd_pending consumato, tail
	// scaduto, cmd_search_state non ancora salito). Senza, DTACKn cadeva 1 ciclo ->
	// la CPU leggeva cop_hit_val_stat RESIDUO prima del commit di b900 -> nemici non
	// bloccati ai confini. Influenza SOLO bus_busy, NON i mux di porta.
	//
	assign     cpu_stall = dma_busy || trig_macro || trig_dma || trig_sort_dma;

	// Per-FSM working registers usati dagli assign dma_vram_*_now sotto: dichiarati
	// QUI (prima dell'uso) per compatibilità con tool Verilog strict (ModelSim).
	// Quartus/Verilator accettano l'uso-prima-della-decl, ModelSim no.
	reg [15:0] dma_dst;             // local dest word index (BG/Pal i)
	reg [8:0]  dma_mode_lat;
	// DMA mode 0x80-0x87: MAME dma_palette_brightness() fa FADE solo con
	// pal_brightness_mode 4/5; con qualsiasi altro valore fa una COPIA SEMPLICE
	// src->dst (ramo else, seibucop_dma.ipp:119-123) verso QUALUNQUE dst — incl.
	// sprite RAM. SeibuCup intro: le PIETRE-sprite si copiano a $105800 via 0x80
	// con mode!=4/5. dma_copy80 instrada quel caso al path FILL (che decodifica
	// gia' bg/fg/mg/txt/pal/SPR per word-addr assoluto) invece del fade-palette.
	reg        dma_copy80;          // 1 = mode 0x8x copia (non fade)
	reg [23:0] dma_dst_abs;         // byte-addr assoluto dest della copia (avanza)
	// DMA generici 0x09/0x0E: MAME write_word(dst) va nello spazio host = arriva
	// a QUALSIASI RAM mappata, incluse le BRAM video ($101000-$105FFF: staging
	// tilemap, palette-stage $104000, sprite $105000). Il core scriveva SEMPRE
	// la Main RAM (mr port): le copie con dst nelle BRAM (es. $B582 dst=$104000
	// nelle transizioni $E902/$EA12) finivano nell'OMBRA della main RAM che la
	// CPU non vede -> copia PERSA. dma_copy_gen instrada quel caso al path fill
	// (decode per word-addr assoluto in main_top), come dma_copy80.
	reg        dma_copy_gen;        // 1 = mode 0x09/0x0E con dst nelle BRAM video
	// Output del fade/blend 0x8x: MAME dma_palette_brightness fa
	// write_word(dst) nello SPAZIO HOST = raggiunge QUALUNQUE RAM mappata
	// (pal-stage $104000, sprite $105800, Main RAM...). Il core forzava
	// l'output SEMPRE su pal_stage. Ora: pal_val registrato in D_FADE_WRITE e
	// committato in D_FADE_COMMIT via canale fill (decode per dst assoluto in
	// main_top: bg/fg/mg/txt/pal/spr) o mr (dst in Main RAM) — 1:1 MAME.
	// (Il vecchio dma80_to_spr era INVERTITO: il blit pietre e' src=$105800
	// dst=$104000, non il contrario — MAME map: $100478=SRC, $10047C=DST.)
	reg [15:0] fade_out_val;        // pal_val calcolato, scritto in D_FADE_COMMIT
	// Termini precalcolati per togliere aritmetica dal ciclo che consuma il dato
	// letto dalla BRAM: dopo la M10K deve restarci UNA somma sola, non due.
	reg [15:0] scrn_delta;          // (tmp32_a[31:16] - tmp_hi) del 0205
	reg signed [31:0] calc_dx, calc_dy;  // differenze del 138e, registrate
	reg [15:0] p905_low;            // somma/differenza bassa del 0905
	reg        p905_cb;             // carry (o borrow) uscente da quella somma
	// LEGACY sprite-DMA state (MAME m_sprite_dma_rel_x/y, m_sprite_dma_x_clip)
	reg        spr_src_is_rom;      // src template in ROM 68k (< $100000): porta cop_rom
	reg [15:0] spr_rom_lat;         // word letta dalla ROM (info c480)
	reg  [7:0] spr_dma_rel_x, spr_dma_rel_y;
	reg [15:0] spr_info_l;          // sprite_info (+param[5:0]) del c480 corrente
	reg [15:0] abs_x_r, abs_y_r;    // obj - abs (camera) latchati
	reg signed [15:0] x_clip_r;     // clip interno per l'inc $100410
	reg  [7:0] ang62_l;             // 6200: angolo letto (dominio stored, host+0x37)

	// Current-cycle VRAM write intent (cmd 0x14): true DURING D_WRITE(i), when
	// dma_dst already holds dst(i) and dma_src_rdata holds mem[src_i]. Replaces the
	// registered dma_vram_we/_addr in main_top's port-A mux (those lag 1 cycle).
	assign dma_vram_we_now   = (fsm == D_WRITE) && (dma_mode_lat[3:0] == 4'h4);
	assign dma_vram_addr_now = dma_dst[12:0];

	// FILL 0x116/0x118 → staging: cupsoc_map $100800-$104FFF (BG/FG/MG/TXT/
	// PAL-live/PAL-staging) + $107000-$107FFF (sprite RAM: clear via 0x118,
	// disasm $20C6 src=$41C0<<6). dma_mode_lat[8:4]==5'b1_0001 copre 0x11x.
	// Il fill DEVE colpire le BRAM scratch vere (il gioco ci azzera sprite RAM
	// e tilemap): pattern HeatedBarrel.
	wire fill_is_vram = (dma_mode_lat[8:4] == 5'b1_0001)
	                    && (((dma_src >= 24'h100800) && (dma_src < 24'h105000))
	                     || ((dma_src >= 24'h107000) && (dma_src < 24'h108000)));
	// Fill 0x116/0x118 (valore costante) OPPURE copia 0x8x (dato da src): entrambi
	// usano il path fill del main_top (decode per regione). Per la copia l'addr e'
	// la destinazione assoluta (dma_dst_abs) e il dato e' quello letto (dma_src_rdata).
	// dst assoluto nelle BRAM video? (per copy80/copy_gen/fade-commit/spr-DMA)
	// cupsoc: include la spriteram $107000 (entry c480) e la pal live $103000
	// (dst del fade); $105000-$106FFF e' Main RAM (fuori finestra).
	wire dst_abs_is_vram = ((dma_dst_abs >= 24'h100800) && (dma_dst_abs < 24'h105000))
	                    || ((dma_dst_abs >= 24'h107000) && (dma_dst_abs < 24'h108000));
	// Stati che scrivono fade_out_val al dst assoluto (fade-commit + c480)
	wire abs_wr_state = (fsm == D_FADE_COMMIT) || (fsm == M_C4_WRI)
	                 || (fsm == M_C4_WRX)      || (fsm == M_C4_WRY)
	                 || (fsm == M_C4_WRP);
	wire        fill_we_c    = ((fsm == D_FILL_W) && fill_is_vram)
	                        || ((fsm == D_WRITE) && (dma_copy80 || dma_copy_gen))
	                        || (abs_wr_state && dst_abs_is_vram);
	wire [22:0] fill_addr_c  = (dma_copy80 || dma_copy_gen || abs_wr_state)
	                        ? dma_dst_abs[23:1] : dma_src[23:1];
	wire [15:0] fill_wdata_c = abs_wr_state ? fade_out_val
	                      : (dma_copy80 || dma_copy_gen) ? dma_src_rdata
	                                   : (dma_src[1] ? cop_dma_v1 : cop_dma_v2);

	// STADIO DI USCITA REGISTRATO. Il dato nasceva da un mux combinatorio
	// sull'uscita della M10K e attraversava mezzo chip fino alla RAM sprite:
	// era il percorso critico. I tre segnali sono registrati INSIEME, quindi la
	// scrittura e' identica, solo un ciclo dopo — nessun cambio di contenuto.
	reg        fill_we_r;
	reg [22:0] fill_addr_r;
	reg [15:0] fill_wdata_r;
	always @(posedge clk) begin
		fill_we_r    <= fill_we_c;
		fill_addr_r  <= fill_addr_c;
		fill_wdata_r <= fill_wdata_c;
	end
	assign dma_fill_we    = fill_we_r;
	assign dma_fill_addr  = fill_addr_r;
	assign dma_fill_wdata = fill_wdata_r;

	always @(posedge clk) begin : trig_blk
		if (reset) begin
			cmd_pending      <= 1'b0;
			cmd_value        <= 0;
			cmd_offset       <= 2'd0;
			dma_pending      <= 1'b0;
			sort_pending     <= 1'b0;
			sort_n           <= 6'd0;
			cmd_search_data  <= 0;
			cmd_match_bits   <= 32'd0;
			cmd_search_state <= 2'd0;
			cmd_match_slot   <= 5'd0;
			cmd_match_found  <= 1'b0;
		end else begin
			case (cmd_search_state)
				2'd0: begin // idle
					if (trig_macro) begin
						cmd_search_data  <= wdata;
						cmd_offset       <= widx[1:0];   // 0=0x100500, 1=0x100502, 2=0x100504
						cmd_search_state <= 2'd1;
					end
				end
				2'd1: begin // S1: compute match[k] for all 32 slots (parallel, 1 ciclo)
					for (pe_k = 0; pe_k < 32; pe_k = pe_k + 1) begin
						cmd_match_bits[pe_k] <= (cop_func_trigger[pe_k] != 16'd0) &&
						   ((cmd_search_data & 16'hf800) == (cop_func_trigger[pe_k] & 16'hf800));
					end
					cmd_search_state <= 2'd2;
				end
				2'd2: begin // S2: priority encode (first hit wins) — tree-based log2(32)=5 levels
					// Casewise priority encoder con cascade comb 5 livelli.
					casez (cmd_match_bits)
						32'b???????????????????????????????1: begin cmd_match_slot <= 5'd0;  cmd_match_found <= 1'b1; end
						32'b??????????????????????????????10: begin cmd_match_slot <= 5'd1;  cmd_match_found <= 1'b1; end
						32'b?????????????????????????????100: begin cmd_match_slot <= 5'd2;  cmd_match_found <= 1'b1; end
						32'b????????????????????????????1000: begin cmd_match_slot <= 5'd3;  cmd_match_found <= 1'b1; end
						32'b???????????????????????????10000: begin cmd_match_slot <= 5'd4;  cmd_match_found <= 1'b1; end
						32'b??????????????????????????100000: begin cmd_match_slot <= 5'd5;  cmd_match_found <= 1'b1; end
						32'b?????????????????????????1000000: begin cmd_match_slot <= 5'd6;  cmd_match_found <= 1'b1; end
						32'b????????????????????????10000000: begin cmd_match_slot <= 5'd7;  cmd_match_found <= 1'b1; end
						32'b???????????????????????100000000: begin cmd_match_slot <= 5'd8;  cmd_match_found <= 1'b1; end
						32'b??????????????????????1000000000: begin cmd_match_slot <= 5'd9;  cmd_match_found <= 1'b1; end
						32'b?????????????????????10000000000: begin cmd_match_slot <= 5'd10; cmd_match_found <= 1'b1; end
						32'b????????????????????100000000000: begin cmd_match_slot <= 5'd11; cmd_match_found <= 1'b1; end
						32'b???????????????????1000000000000: begin cmd_match_slot <= 5'd12; cmd_match_found <= 1'b1; end
						32'b??????????????????10000000000000: begin cmd_match_slot <= 5'd13; cmd_match_found <= 1'b1; end
						32'b?????????????????100000000000000: begin cmd_match_slot <= 5'd14; cmd_match_found <= 1'b1; end
						32'b????????????????1000000000000000: begin cmd_match_slot <= 5'd15; cmd_match_found <= 1'b1; end
						32'b???????????????10000000000000000: begin cmd_match_slot <= 5'd16; cmd_match_found <= 1'b1; end
						32'b??????????????100000000000000000: begin cmd_match_slot <= 5'd17; cmd_match_found <= 1'b1; end
						32'b?????????????1000000000000000000: begin cmd_match_slot <= 5'd18; cmd_match_found <= 1'b1; end
						32'b????????????10000000000000000000: begin cmd_match_slot <= 5'd19; cmd_match_found <= 1'b1; end
						32'b???????????100000000000000000000: begin cmd_match_slot <= 5'd20; cmd_match_found <= 1'b1; end
						32'b??????????1000000000000000000000: begin cmd_match_slot <= 5'd21; cmd_match_found <= 1'b1; end
						32'b?????????10000000000000000000000: begin cmd_match_slot <= 5'd22; cmd_match_found <= 1'b1; end
						32'b????????100000000000000000000000: begin cmd_match_slot <= 5'd23; cmd_match_found <= 1'b1; end
						32'b???????1000000000000000000000000: begin cmd_match_slot <= 5'd24; cmd_match_found <= 1'b1; end
						32'b??????10000000000000000000000000: begin cmd_match_slot <= 5'd25; cmd_match_found <= 1'b1; end
						32'b?????100000000000000000000000000: begin cmd_match_slot <= 5'd26; cmd_match_found <= 1'b1; end
						32'b????1000000000000000000000000000: begin cmd_match_slot <= 5'd27; cmd_match_found <= 1'b1; end
						32'b???10000000000000000000000000000: begin cmd_match_slot <= 5'd28; cmd_match_found <= 1'b1; end
						32'b??100000000000000000000000000000: begin cmd_match_slot <= 5'd29; cmd_match_found <= 1'b1; end
						32'b?1000000000000000000000000000000: begin cmd_match_slot <= 5'd30; cmd_match_found <= 1'b1; end
						32'b10000000000000000000000000000000: begin cmd_match_slot <= 5'd31; cmd_match_found <= 1'b1; end
						default:                              begin cmd_match_slot <= 5'd0;  cmd_match_found <= 1'b0; end
					endcase
					cmd_search_state <= 2'd3;
				end
				2'd3: begin // S3: signal cmd_pending. MAME cop_cmd_w (seibucop.cpp:1075):
					// switch(data) raw, NON canonical. find_trigger_match è solo per log.
					// Quindi cmd_value = wdata originale del game.
					cmd_value        <= cmd_search_data;
					cmd_pending      <= 1'b1;
					cmd_search_state <= 2'd0;
				end
			endcase

			if (trig_dma) begin
				dma_pending <= 1'b1;
			end
			// MAME cop_sort_dma_trig_w -> dma_zsorting(data): n = data + 1
			if (trig_sort_dma) begin
				sort_pending <= 1'b1;
				sort_n       <= (wdata >= 16'd59) ? 6'd60 : (wdata[5:0] + 6'd1);
			end
			// FSM consumes pending in S_IDLE → clears them itself
			if (fsm == S_IDLE && (cmd_pending || dma_pending || sort_pending)) begin
				cmd_pending  <= 1'b0;
				dma_pending  <= 1'b0;
				sort_pending <= 1'b0;
			end
		end
	end

	// Sprite DMA inc status — merged into CPU writes block to avoid multi-driver.
	// See trig_spr_inc branch inside the main always block above (case 10'h008).

	// ────────────────────────────────────────────────────────────────────────
	// cop_regs_byte_addr: helper per ottenere il byte-address 24-bit completo
	// di cop_regs[N] = {hi, lo}. cop_reg_hi è già scritto dalla CPU al boot
	// (vedi routine $3290 in maincpu: scrive hi a $1004A0 + lo a $1004C0).
	function [23:0] cop_regs_byte_addr(input integer n, input [15:0] offs);
		begin
			cop_regs_byte_addr = {cop_reg_hi[n][7:0], cop_reg_lo[n]} + {8'd0, offs};
		end
	endfunction

	// ────────────────────────────────────────────────────────────────────────
	// find_trigger_match (seibucop.cpp:464)
	// Cerca slot N tale che (trigger & 0xf800) == (cop_func_trigger[N] & 0xf800)
	// && cop_func_trigger[N] != 0. Restituisce canonical trigger value del slot
	// (NB: MAME LEGACY_cop_cmd_w usa il match slot per dispatchare; in pratica
	// equivale a restituire cop_func_trigger[slot]).
	// Se nessun match: ritorna l'input originale (= no-op nel dispatcher).
	function [15:0] cop_trigger_canonical(input [15:0] trig);
		reg [15:0] r;
		integer k;
		begin
			r = trig;
			for (k = 0; k < 32; k = k + 1) begin
				if (cop_func_trigger[k] != 16'd0 &&
				    ((trig & 16'hf800) == (cop_func_trigger[k] & 16'hf800)) &&
				    r == trig) begin
					r = cop_func_trigger[k];
				end
			end
			cop_trigger_canonical = r;
		end
	endfunction

	// ────────────────────────────────────────────────────────────────────────
	// fade_table — MAME seibucop.cpp:733 (reverse engineered from Seibu Cup Soccer bootleg)
	//   v = pal_channel(5-bit) | (brightness_val_xor_X)  [10-bit input]
	//   low  = v & 0x1F
	//   high = v & 0x3E0
	//   return (low * (high | (high>>5)) + 0x210) >> 10  [8-bit out, range 0..31]
	function [7:0] fade_table_fn(input [9:0] v);
		reg [4:0] low;
		reg [9:0] high;
		reg [9:0] hi_dup;
		reg [19:0] prod;
		reg [19:0] sum;
		begin
			low    = v[4:0];
			high   = {v[9:5], 5'b00000};                     // bit 9..5, low cleared
			hi_dup = high | {5'b00000, v[9:5]};              // high | (high>>5)
			prod   = low * hi_dup;
			sum    = prod + 20'h00210;
			fade_table_fn = sum[17:10];                       // >> 10 → bit [17:10]
		end
	endfunction

	// ────────────────────────────────────────────────────────────────────────
	// Per-FSM working registers
	// ────────────────────────────────────────────────────────────────────────
	reg [15:0] dma_cnt;
	reg [23:0] dma_src;             // byte address (full 68K bus)
	// dma_dst / dma_mode_lat dichiarati più in alto (prima degli assign *_now).

	// Macro scratch
	reg [15:0] tmp_hi, tmp_lo;
	// Fade blend pipeline: risultati fade_table registrati (spezza 6 mul in 2 stadi)
	reg  [7:0] fade_fb_t, fade_fb_c, fade_fg_t, fade_fg_c, fade_fr_t, fade_fr_c;
	reg [15:0] fade_paldata;
	reg        fade_nofade;
	reg [15:0] fade_tgtdata;   // mode 4: target puro a val 0x10/0xFFFF
	reg        fade_pure;
	reg [31:0] tmp32_a, tmp32_b;
	reg [15:0] m_target_addr;        // used by M_*_WR_*

	// ── Hitbox b100/b900 scratch (MAME cop_collision_update_hitbox) ──────────────
	// dx[i] = int8 (signed) byte basso, size[i] = uint8 (unsigned) byte alto del
	// word descrittore @ hitadr2+2*i. 3 assi (Y,X,Z) per SeibuCup.
	reg signed [15:0] hb_dx   [0:2] /*verilator public_flat_rd*/;
	reg        [15:0] hb_size [0:2] /*verilator public_flat_rd*/;
	reg        [23:0] hb_adr2 /*verilator public_flat_rd*/;        // base descrittore hitbox (in ROM)
	reg        [23:0] hb_ptr_addr;    // addr del puntatore (cop_regs[2/3], in ROM)
	// num_axis: MAME seibucop.cpp:1010 "if (data & 0x100) num_axis = 3" else 2.
	// b100/b900 (bit8=1) = 3 assi come oggi; b000/b800 (patch init_godzilla,
	// bit8=0) = 2 assi (Z disabilitata). res init 7 vs 3 (seibucop.cpp:1038-39).
	reg               hb_axis3;

	// ── CORDIC / math scratch (atan 138e, dist 3bb0, divide 42c2) ──────────────
	// dx/dy 32-bit signed (differenze posizione r1-r0), risultati cop_angle/cop_dist.
	reg signed [31:0] math_dx, math_dy;
	// Delta LATCHATI dall'ultimo comando d'angolo: il 3bb0 NON li ricalcola,
	// li riusa (in MAME sono m_LEGACY_r0/r1, e il TODO sopra execute_3b30 dice
	// esattamente questo). Con reg1 al posto dei latch, dopo un e30e la
	// distanza non e' quella dalla palla e il contatto non scatta mai.
	reg signed [31:0] latch_dx, latch_dy;
	reg signed [31:0] cordic_x, cordic_y;     // CORDIC vectoring accumulators
	reg signed [23:0] cordic_z;               // angolo accumulato ×65536 (16-bit frazione)
	reg        [4:0]  cordic_i;               // iterazione 0..23
	// Meta' pipeline del CORDIC: l'iterazione stava tutta in un ciclo, e la
	// catena shift-variabile + somma a 32 bit era il percorso critico del core
	// (~11.8 ns contro 10.416 disponibili a 96 MHz). Ora il ciclo A registra
	// shift, segno e voce di tabella; il ciclo B fa solo le somme.
	reg signed [31:0] cordic_xs, cordic_ys;   // cordic_x/y >>> i, registrati
	reg signed [23:0] cordic_lut;             // voce atan, registrata
	reg               cordic_neg;             // segno di cordic_y al ciclo A
	reg signed [31:0] sqrt_acc, sqrt_rem;     // Newton/restoring sqrt
	// dist 3bb0 restoring sqrt scratch (34-bit resto + radicando shiftato)
	reg        [33:0] sqrt_rem34;
	reg        [31:0] sqrt_radsh;
	reg        [4:0]  sqrt_i;
	reg        [31:0] div_num;                // dividendo per 42c2/4aa0
	reg        [15:0] div_den;
	reg        [5:0]  div_i;                  // 6-bit per contare 32 iter (review D1)
	reg        [31:0] div_q, div_r;
	// dde5: divisore CON SEGNO (in MAME e' una divisione C int, quindi
	// troncamento verso zero). Il numeratore puo' essere negativo.
	reg        [15:0] dde5_div;       // divisore, da cop_regs[4]+offs
	reg signed [15:0] dde5_dir;       // offset,   da cop_regs[4]+offs+8
	reg               dde5_neg;       // segno del numeratore
	reg        [15:0] dde5_offs;      // (offset&3)*4
	// moltiplicatore 16.16 condiviso 5105/f105: shift-add signed 32x32->64,
	// 1 bit/ciclo (niente DSP, niente prodotto combinatorio sul path critico).
	reg signed [63:0] mul_acc;                // prodotto in accumulo
	reg signed [63:0] mul_md;                 // multiplicando esteso in segno, shiftato
	reg        [31:0] mul_mr;                 // moltiplicatore, shiftato a destra
	reg        [5:0]  mul_i;                  // 32 iterazioni
	reg        [15:0] mul_off4;               // offset*4 del comando
	// sorgente/destinazione parametriche: 5105 r0+00->r0+04, f105 r0+10->r0+10,
	// 5905 r2+10->r1+04 (r2 = oggetto, programmato una volta a $011AA2).
	reg        [1:0]  mul_src_reg, mul_dst_reg;
	reg        [15:0] mul_src_off, mul_dst_off;
	// atan2 (slot 02 = 130e/138e/118e -> reg1 ; slot 1C = e30e/e38e/e18e -> reg2 =
	// la palla ; slot 06 = 330e/338e -> reg1). Il secondo punto cambia col comando.
	reg               atan_src_r2;
	reg               mul_sub_org;   // d104: sottrai l'origine (reg3) dalla sorgente
	reg        [31:0] mul_org;

	// atan LUT: arctan(2^-i) in unità angolo Seibu SCALATE ×256 (1/256 di unità).
	//   valore = round( arctan(2^-i) * 128/π * 256 ).  i=0..15.
	// Il loop CORDIC accumula in cordic_z (×256); il finale fa (z+128)>>8 per
	// tornare a unità intere con rounding. Valori esatti (calcolati, no approx):
	// atan LUT a 16-BIT DI FRAZIONE (angolo_byte ×65536), 24 entry. La precisione
	// ×256/16-iter sovrastimava ~+0.006 al bordo intero -> cop_angle cadeva sul lato
	// sbagliato del bordo della tabella-decisione $48DA del nemico-martello (0x07/08,
	// 0x77/78, 0x87/88, 0xf7/f8) -> non azzerava velX -> il bruto SLITTAVA durante
	// l'attacco. Con 16-bit frac + 24 iter + z>>16: 0 divergenze di decisione vs MAME
	// int(atan(dx/dy)*128/pi) su ~640k casi (dimostrato numericamente).
	function signed [23:0] cordic_atan_lut(input [4:0] i);
		case (i)
			5'd0:  cordic_atan_lut = 24'sd2097152; // 45.000° ×65536
			5'd1:  cordic_atan_lut = 24'sd1238021; // 26.565°
			5'd2:  cordic_atan_lut = 24'sd654136;  // 14.036°
			5'd3:  cordic_atan_lut = 24'sd332050;  // 7.125°
			5'd4:  cordic_atan_lut = 24'sd166669;  // 3.576°
			5'd5:  cordic_atan_lut = 24'sd83416;   // 1.790°
			5'd6:  cordic_atan_lut = 24'sd41718;   // 0.895°
			5'd7:  cordic_atan_lut = 24'sd20860;   // 0.448°
			5'd8:  cordic_atan_lut = 24'sd10430;   // 0.224°
			5'd9:  cordic_atan_lut = 24'sd5215;    // 0.112°
			5'd10: cordic_atan_lut = 24'sd2608;    // 0.056°
			5'd11: cordic_atan_lut = 24'sd1304;
			5'd12: cordic_atan_lut = 24'sd652;
			5'd13: cordic_atan_lut = 24'sd326;
			5'd14: cordic_atan_lut = 24'sd163;
			5'd15: cordic_atan_lut = 24'sd81;
			5'd16: cordic_atan_lut = 24'sd41;
			5'd17: cordic_atan_lut = 24'sd20;
			5'd18: cordic_atan_lut = 24'sd10;
			5'd19: cordic_atan_lut = 24'sd5;
			5'd20: cordic_atan_lut = 24'sd3;
			5'd21: cordic_atan_lut = 24'sd1;
			default: cordic_atan_lut = 24'sd0;   // i>=22
		endcase
	endfunction

	// ────────────────────────────────────────────────────────────────────────
	// Sin/Cos table — 256 entries of (sin(i * π/128) * 32768), signed 16-bit
	// Precomputed; covers the full 8-bit angle space.
	// ────────────────────────────────────────────────────────────────────────
	function [15:0] sin_table_lookup(input [7:0] a);
		case (a)
			8'h00:sin_table_lookup=16'sd0;     8'h01:sin_table_lookup=16'sd804;
			8'h02:sin_table_lookup=16'sd1608;  8'h03:sin_table_lookup=16'sd2410;
			8'h04:sin_table_lookup=16'sd3212;  8'h05:sin_table_lookup=16'sd4011;
			8'h06:sin_table_lookup=16'sd4808;  8'h07:sin_table_lookup=16'sd5602;
			8'h08:sin_table_lookup=16'sd6393;  8'h09:sin_table_lookup=16'sd7179;
			8'h0a:sin_table_lookup=16'sd7962;  8'h0b:sin_table_lookup=16'sd8739;
			8'h0c:sin_table_lookup=16'sd9512;  8'h0d:sin_table_lookup=16'sd10278;
			8'h0e:sin_table_lookup=16'sd11039; 8'h0f:sin_table_lookup=16'sd11793;
			8'h10:sin_table_lookup=16'sd12539; 8'h11:sin_table_lookup=16'sd13279;
			8'h12:sin_table_lookup=16'sd14010; 8'h13:sin_table_lookup=16'sd14732;
			8'h14:sin_table_lookup=16'sd15446; 8'h15:sin_table_lookup=16'sd16151;
			8'h16:sin_table_lookup=16'sd16846; 8'h17:sin_table_lookup=16'sd17530;
			8'h18:sin_table_lookup=16'sd18204; 8'h19:sin_table_lookup=16'sd18868;
			8'h1a:sin_table_lookup=16'sd19519; 8'h1b:sin_table_lookup=16'sd20159;
			8'h1c:sin_table_lookup=16'sd20787; 8'h1d:sin_table_lookup=16'sd21403;
			8'h1e:sin_table_lookup=16'sd22005; 8'h1f:sin_table_lookup=16'sd22594;
			8'h20:sin_table_lookup=16'sd23170; 8'h21:sin_table_lookup=16'sd23731;
			8'h22:sin_table_lookup=16'sd24279; 8'h23:sin_table_lookup=16'sd24811;
			8'h24:sin_table_lookup=16'sd25329; 8'h25:sin_table_lookup=16'sd25832;
			8'h26:sin_table_lookup=16'sd26319; 8'h27:sin_table_lookup=16'sd26790;
			8'h28:sin_table_lookup=16'sd27245; 8'h29:sin_table_lookup=16'sd27683;
			8'h2a:sin_table_lookup=16'sd28105; 8'h2b:sin_table_lookup=16'sd28510;
			8'h2c:sin_table_lookup=16'sd28898; 8'h2d:sin_table_lookup=16'sd29268;
			8'h2e:sin_table_lookup=16'sd29621; 8'h2f:sin_table_lookup=16'sd29956;
			8'h30:sin_table_lookup=16'sd30273; 8'h31:sin_table_lookup=16'sd30571;
			8'h32:sin_table_lookup=16'sd30852; 8'h33:sin_table_lookup=16'sd31113;
			8'h34:sin_table_lookup=16'sd31356; 8'h35:sin_table_lookup=16'sd31580;
			8'h36:sin_table_lookup=16'sd31785; 8'h37:sin_table_lookup=16'sd31971;
			8'h38:sin_table_lookup=16'sd32137; 8'h39:sin_table_lookup=16'sd32285;
			8'h3a:sin_table_lookup=16'sd32412; 8'h3b:sin_table_lookup=16'sd32521;
			8'h3c:sin_table_lookup=16'sd32609; 8'h3d:sin_table_lookup=16'sd32678;
			8'h3e:sin_table_lookup=16'sd32728; 8'h3f:sin_table_lookup=16'sd32757;
			8'h40:sin_table_lookup=16'sd32767; 8'h41:sin_table_lookup=16'sd32757;
			8'h42:sin_table_lookup=16'sd32728; 8'h43:sin_table_lookup=16'sd32678;
			8'h44:sin_table_lookup=16'sd32609; 8'h45:sin_table_lookup=16'sd32521;
			8'h46:sin_table_lookup=16'sd32412; 8'h47:sin_table_lookup=16'sd32285;
			8'h48:sin_table_lookup=16'sd32137; 8'h49:sin_table_lookup=16'sd31971;
			8'h4a:sin_table_lookup=16'sd31785; 8'h4b:sin_table_lookup=16'sd31580;
			8'h4c:sin_table_lookup=16'sd31356; 8'h4d:sin_table_lookup=16'sd31113;
			8'h4e:sin_table_lookup=16'sd30852; 8'h4f:sin_table_lookup=16'sd30571;
			8'h50:sin_table_lookup=16'sd30273; 8'h51:sin_table_lookup=16'sd29956;
			8'h52:sin_table_lookup=16'sd29621; 8'h53:sin_table_lookup=16'sd29268;
			8'h54:sin_table_lookup=16'sd28898; 8'h55:sin_table_lookup=16'sd28510;
			8'h56:sin_table_lookup=16'sd28105; 8'h57:sin_table_lookup=16'sd27683;
			8'h58:sin_table_lookup=16'sd27245; 8'h59:sin_table_lookup=16'sd26790;
			8'h5a:sin_table_lookup=16'sd26319; 8'h5b:sin_table_lookup=16'sd25832;
			8'h5c:sin_table_lookup=16'sd25329; 8'h5d:sin_table_lookup=16'sd24811;
			8'h5e:sin_table_lookup=16'sd24279; 8'h5f:sin_table_lookup=16'sd23731;
			8'h60:sin_table_lookup=16'sd23170; 8'h61:sin_table_lookup=16'sd22594;
			8'h62:sin_table_lookup=16'sd22005; 8'h63:sin_table_lookup=16'sd21403;
			8'h64:sin_table_lookup=16'sd20787; 8'h65:sin_table_lookup=16'sd20159;
			8'h66:sin_table_lookup=16'sd19519; 8'h67:sin_table_lookup=16'sd18868;
			8'h68:sin_table_lookup=16'sd18204; 8'h69:sin_table_lookup=16'sd17530;
			8'h6a:sin_table_lookup=16'sd16846; 8'h6b:sin_table_lookup=16'sd16151;
			8'h6c:sin_table_lookup=16'sd15446; 8'h6d:sin_table_lookup=16'sd14732;
			8'h6e:sin_table_lookup=16'sd14010; 8'h6f:sin_table_lookup=16'sd13279;
			8'h70:sin_table_lookup=16'sd12539; 8'h71:sin_table_lookup=16'sd11793;
			8'h72:sin_table_lookup=16'sd11039; 8'h73:sin_table_lookup=16'sd10278;
			8'h74:sin_table_lookup=16'sd9512;  8'h75:sin_table_lookup=16'sd8739;
			8'h76:sin_table_lookup=16'sd7962;  8'h77:sin_table_lookup=16'sd7179;
			8'h78:sin_table_lookup=16'sd6393;  8'h79:sin_table_lookup=16'sd5602;
			8'h7a:sin_table_lookup=16'sd4808;  8'h7b:sin_table_lookup=16'sd4011;
			8'h7c:sin_table_lookup=16'sd3212;  8'h7d:sin_table_lookup=16'sd2410;
			8'h7e:sin_table_lookup=16'sd1608;  8'h7f:sin_table_lookup=16'sd804;
			default: begin
				// Negative half: sin(x+π) = -sin(x). Lookup mirror.
				sin_table_lookup = -sin_table_lookup_lo(a - 8'h80);
			end
		endcase
	endfunction

	// Helper for negative-half lookup
	function [15:0] sin_table_lookup_lo(input [7:0] a);
		// Identical to first half (a in 0..7F) — duplicated to avoid recursion.
		case (a)
			8'h00:sin_table_lookup_lo=16'sd0;     8'h01:sin_table_lookup_lo=16'sd804;
			8'h02:sin_table_lookup_lo=16'sd1608;  8'h03:sin_table_lookup_lo=16'sd2410;
			8'h04:sin_table_lookup_lo=16'sd3212;  8'h05:sin_table_lookup_lo=16'sd4011;
			8'h06:sin_table_lookup_lo=16'sd4808;  8'h07:sin_table_lookup_lo=16'sd5602;
			8'h08:sin_table_lookup_lo=16'sd6393;  8'h09:sin_table_lookup_lo=16'sd7179;
			8'h0a:sin_table_lookup_lo=16'sd7962;  8'h0b:sin_table_lookup_lo=16'sd8739;
			8'h0c:sin_table_lookup_lo=16'sd9512;  8'h0d:sin_table_lookup_lo=16'sd10278;
			8'h0e:sin_table_lookup_lo=16'sd11039; 8'h0f:sin_table_lookup_lo=16'sd11793;
			8'h10:sin_table_lookup_lo=16'sd12539; 8'h11:sin_table_lookup_lo=16'sd13279;
			8'h12:sin_table_lookup_lo=16'sd14010; 8'h13:sin_table_lookup_lo=16'sd14732;
			8'h14:sin_table_lookup_lo=16'sd15446; 8'h15:sin_table_lookup_lo=16'sd16151;
			8'h16:sin_table_lookup_lo=16'sd16846; 8'h17:sin_table_lookup_lo=16'sd17530;
			8'h18:sin_table_lookup_lo=16'sd18204; 8'h19:sin_table_lookup_lo=16'sd18868;
			8'h1a:sin_table_lookup_lo=16'sd19519; 8'h1b:sin_table_lookup_lo=16'sd20159;
			8'h1c:sin_table_lookup_lo=16'sd20787; 8'h1d:sin_table_lookup_lo=16'sd21403;
			8'h1e:sin_table_lookup_lo=16'sd22005; 8'h1f:sin_table_lookup_lo=16'sd22594;
			8'h20:sin_table_lookup_lo=16'sd23170; 8'h21:sin_table_lookup_lo=16'sd23731;
			8'h22:sin_table_lookup_lo=16'sd24279; 8'h23:sin_table_lookup_lo=16'sd24811;
			8'h24:sin_table_lookup_lo=16'sd25329; 8'h25:sin_table_lookup_lo=16'sd25832;
			8'h26:sin_table_lookup_lo=16'sd26319; 8'h27:sin_table_lookup_lo=16'sd26790;
			8'h28:sin_table_lookup_lo=16'sd27245; 8'h29:sin_table_lookup_lo=16'sd27683;
			8'h2a:sin_table_lookup_lo=16'sd28105; 8'h2b:sin_table_lookup_lo=16'sd28510;
			8'h2c:sin_table_lookup_lo=16'sd28898; 8'h2d:sin_table_lookup_lo=16'sd29268;
			8'h2e:sin_table_lookup_lo=16'sd29621; 8'h2f:sin_table_lookup_lo=16'sd29956;
			8'h30:sin_table_lookup_lo=16'sd30273; 8'h31:sin_table_lookup_lo=16'sd30571;
			8'h32:sin_table_lookup_lo=16'sd30852; 8'h33:sin_table_lookup_lo=16'sd31113;
			8'h34:sin_table_lookup_lo=16'sd31356; 8'h35:sin_table_lookup_lo=16'sd31580;
			8'h36:sin_table_lookup_lo=16'sd31785; 8'h37:sin_table_lookup_lo=16'sd31971;
			8'h38:sin_table_lookup_lo=16'sd32137; 8'h39:sin_table_lookup_lo=16'sd32285;
			8'h3a:sin_table_lookup_lo=16'sd32412; 8'h3b:sin_table_lookup_lo=16'sd32521;
			8'h3c:sin_table_lookup_lo=16'sd32609; 8'h3d:sin_table_lookup_lo=16'sd32678;
			8'h3e:sin_table_lookup_lo=16'sd32728; 8'h3f:sin_table_lookup_lo=16'sd32757;
			8'h40:sin_table_lookup_lo=16'sd32767; 8'h41:sin_table_lookup_lo=16'sd32757;
			8'h42:sin_table_lookup_lo=16'sd32728; 8'h43:sin_table_lookup_lo=16'sd32678;
			8'h44:sin_table_lookup_lo=16'sd32609; 8'h45:sin_table_lookup_lo=16'sd32521;
			8'h46:sin_table_lookup_lo=16'sd32412; 8'h47:sin_table_lookup_lo=16'sd32285;
			8'h48:sin_table_lookup_lo=16'sd32137; 8'h49:sin_table_lookup_lo=16'sd31971;
			8'h4a:sin_table_lookup_lo=16'sd31785; 8'h4b:sin_table_lookup_lo=16'sd31580;
			8'h4c:sin_table_lookup_lo=16'sd31356; 8'h4d:sin_table_lookup_lo=16'sd31113;
			8'h4e:sin_table_lookup_lo=16'sd30852; 8'h4f:sin_table_lookup_lo=16'sd30571;
			8'h50:sin_table_lookup_lo=16'sd30273; 8'h51:sin_table_lookup_lo=16'sd29956;
			8'h52:sin_table_lookup_lo=16'sd29621; 8'h53:sin_table_lookup_lo=16'sd29268;
			8'h54:sin_table_lookup_lo=16'sd28898; 8'h55:sin_table_lookup_lo=16'sd28510;
			8'h56:sin_table_lookup_lo=16'sd28105; 8'h57:sin_table_lookup_lo=16'sd27683;
			8'h58:sin_table_lookup_lo=16'sd27245; 8'h59:sin_table_lookup_lo=16'sd26790;
			8'h5a:sin_table_lookup_lo=16'sd26319; 8'h5b:sin_table_lookup_lo=16'sd25832;
			8'h5c:sin_table_lookup_lo=16'sd25329; 8'h5d:sin_table_lookup_lo=16'sd24811;
			8'h5e:sin_table_lookup_lo=16'sd24279; 8'h5f:sin_table_lookup_lo=16'sd23731;
			8'h60:sin_table_lookup_lo=16'sd23170; 8'h61:sin_table_lookup_lo=16'sd22594;
			8'h62:sin_table_lookup_lo=16'sd22005; 8'h63:sin_table_lookup_lo=16'sd21403;
			8'h64:sin_table_lookup_lo=16'sd20787; 8'h65:sin_table_lookup_lo=16'sd20159;
			8'h66:sin_table_lookup_lo=16'sd19519; 8'h67:sin_table_lookup_lo=16'sd18868;
			8'h68:sin_table_lookup_lo=16'sd18204; 8'h69:sin_table_lookup_lo=16'sd17530;
			8'h6a:sin_table_lookup_lo=16'sd16846; 8'h6b:sin_table_lookup_lo=16'sd16151;
			8'h6c:sin_table_lookup_lo=16'sd15446; 8'h6d:sin_table_lookup_lo=16'sd14732;
			8'h6e:sin_table_lookup_lo=16'sd14010; 8'h6f:sin_table_lookup_lo=16'sd13279;
			8'h70:sin_table_lookup_lo=16'sd12539; 8'h71:sin_table_lookup_lo=16'sd11793;
			8'h72:sin_table_lookup_lo=16'sd11039; 8'h73:sin_table_lookup_lo=16'sd10278;
			8'h74:sin_table_lookup_lo=16'sd9512;  8'h75:sin_table_lookup_lo=16'sd8739;
			8'h76:sin_table_lookup_lo=16'sd7962;  8'h77:sin_table_lookup_lo=16'sd7179;
			8'h78:sin_table_lookup_lo=16'sd6393;  8'h79:sin_table_lookup_lo=16'sd5602;
			8'h7a:sin_table_lookup_lo=16'sd4808;  8'h7b:sin_table_lookup_lo=16'sd4011;
			8'h7c:sin_table_lookup_lo=16'sd3212;  8'h7d:sin_table_lookup_lo=16'sd2410;
			8'h7e:sin_table_lookup_lo=16'sd1608;  8'h7f:sin_table_lookup_lo=16'sd804;
			default: sin_table_lookup_lo = 16'sd0;
		endcase
	endfunction

	// ────────────────────────────────────────────────────────────────────────
	// FSM body
	// ────────────────────────────────────────────────────────────────────────
	always @(posedge clk) begin
		// selezione per-mode sempre pronta un ciclo prima del trigger
		dma_src_sel  <= cop_dma_src [cop_dma_mode[7:0]];
		dma_size_sel <= cop_dma_size[cop_dma_mode[7:0]];
		dma_dst_sel  <= cop_dma_dst [cop_dma_mode[7:0]];
		if (reset) begin
			fsm           <= S_IDLE;
			return_state  <= S_IDLE;
			dma_cnt       <= 0;
			dma_src       <= 24'd0;
			dma_dst       <= 0;
			dma_src_byte  <= 24'd0;
			dde5_div      <= 16'd1;
			dde5_dir      <= 16'd0;
			dde5_neg      <= 1'b0;
			dde5_offs     <= 16'd0;
			dma_mode_lat  <= 0;
			dma_copy80    <= 1'b0;
			dma_dst_abs   <= 24'd0;
			fade_out_val  <= 16'd0;
			dma_copy_gen  <= 1'b0;
			spr_dma_rel_x <= 8'd0;
			spr_dma_rel_y <= 8'd0;
			ang62_l       <= 8'd0;
			spr_info_l    <= 16'd0;
			abs_x_r       <= 16'd0;
			abs_y_r       <= 16'd0;
			x_clip_r      <= 16'sd0;
			spr_src_is_rom <= 1'b0;
			spr_rom_lat   <= 16'd0;
			dma_ram_addr  <= 0;
			dma_ram_wdata <= 0;
			dma_ram_we    <= 0;
			dma_ram_be    <= 2'b11;
			dma_spr_addr  <= 0;
			dma_spr_wdata <= 0;
			dma_spr_we    <= 0;
			cop_rom_addr  <= 0;
			cop_rom_req   <= 0;
			dma_vram_addr <= 0;
			dma_vram_wdata<= 0;
			dma_vram_we   <= 0;
			dma_pal_addr  <= 0;
			dma_pal_wdata <= 0;
			dma_pal_we    <= 0;
			dma_pal_stage_we <= 0;
			// cop_status[1] è gestito da cpu_wr_pulse block (sprite_dma_inc). Qui
			// resetto solo gli altri bit per evitare multi-driver di [1].
			cop_status[15:2] <= 0;
			cop_status[0]    <= 0;
			cop_angle     <= 0;
			cop_dist      <= 0;
			cop_hit_status   <= 0;
			cop_hit_val_stat <= 0;
			cop_hit_val[0]   <= 0; cop_hit_val[1] <= 0; cop_hit_val[2] <= 0;
			tmp_hi <= 0; tmp_lo <= 0; tmp32_a <= 0; tmp32_b <= 0;
			m_target_addr <= 0;
		end else begin
			// Default each cycle: deassert write strobes
			dma_ram_we    <= 1'b0;
			dma_ram_be    <= 2'b11;   // default word-write (i byte-write lo override)
			dma_spr_we    <= 1'b0;
			dma_vram_we   <= 1'b0;
			dma_pal_we    <= 1'b0;
			dma_pal_stage_we <= 1'b0;

			case (fsm)
			// ════════════════════════════════════════════════════════════════
			S_IDLE: begin
				// Priority: DMA > CMD (so VBlank DMA doesn't get starved)
				if (dma_pending) begin
					// MAME cop_dma_trigger_w: NON modifica cop_status (vedi seibucop.cpp:741).
					// Lo facevamo per errore — l'unica cosa che modifica status durante
					// DMA è cop_sprite_dma_inc_w che gestisce bit 1 (in/out of size).
					dma_mode_lat <= cop_dma_mode;
					dma_copy80   <= 1'b0;   // default: azzerato ogni DMA (settato solo dal case 0x8x-copy)
					dma_copy_gen <= 1'b0;   // default: azzerato ogni DMA (settato solo da 0x09/0x0E dst-BRAM)
					case (cop_dma_mode)
						9'h014: begin
							// MAME dma_tilemap_buffer: copy staging RAM → renderer BRAM
							// src = cop_dma_src[mode] << 6 (byte addr 0x101000+ = staging)
							// dst = i (loop counter 0..0x13FF = renderer index)
							// count = 0x2800/2 = 5120 words
							dma_src <= {2'd0, dma_src_sel, 6'd0};
							dma_dst <= 16'd0;
							dma_cnt <= 16'd5120;
							fsm     <= D_PREP;
						end
						9'h015: begin
							// MAME dma_palette_buffer: copy staging palette → renderer
							// src = cop_dma_src[mode] << 6 (0x104000 = palette staging)
							// dst = i (loop counter 0..0x7FF = renderer index)
							// count = 0x1000/2 = 2048 words
							dma_src <= {2'd0, dma_src_sel, 6'd0};
							dma_dst <= 16'd0;
							dma_cnt <= 16'd2048;
							fsm     <= D_PREP;
						end
						9'h009, 9'h00E: begin
							// generic RAM-to-RAM. MAME write_word(dst) su spazio host:
							// con dst nelle BRAM video ($101000-$105FFF: staging/pal/spr,
							// es. $B582 dst=$104000 nelle transizioni) la write DEVE
							// raggiungere la BRAM, non l'ombra main-RAM: dma_copy_gen
							// instrada al path fill per dst assoluto (come copy80).
							dma_src <= {2'd0, dma_src_sel, 6'd0};
							dma_dst <= dma_dst_sel << 5; // word offset
							dma_dst_abs  <= {2'd0, dma_dst_sel, 6'd0};
							dma_copy_gen <= (({2'd0, dma_dst_sel, 6'd0} >= 24'h100800)
							              && ({2'd0, dma_dst_sel, 6'd0} <  24'h105000))
							             || (({2'd0, dma_dst_sel, 6'd0} >= 24'h107000)
							              && ({2'd0, dma_dst_sel, 6'd0} <  24'h108000));
							// size = ((size<<5) - (dst<<6) + 0x20) / 2
							dma_cnt <= ((dma_size_sel << 4)
							           - (dma_dst_sel << 5)
							           + 16'd16);  // simplified, MAME size>>1
							fsm     <= D_PREP;
						end
						9'h080, 9'h081, 9'h082, 9'h083,
						9'h084, 9'h085, 9'h086, 9'h087: begin
							// MAME seibucop_dma.ipp dma_palette_brightness() mode 5/4.
							// Init SeibuCup ($35B2): src=$4180 ($106000), dst=$4100 ($104000),
							// size=$827F, adr_rel=4.
							// MAME formulas:
							//   src   = cop_dma_src[mode] << 6
							//   dst   = cop_dma_dst[mode] << 6
							//   size  = ((cop_dma_size << 5) - (cop_dma_dst << 6) + $20)/2
							// Init: src=$106000, dst=$104000, size=($827F<<5 - $4100<<6 + $20)/2
							//       = ($104FE0 - $104000 + $20)/2 = $1000/2 = $800 words.
							// Mode 5: blend tra paldata(src) e targetdata(src+adr_rel*$400)
							// usando pal_brightness_val (0..0x1F) come ratio fade.
							dma_src <= {2'd0, dma_src_sel, 6'd0};
							// dst offset (entry in palette renderer): (dst<<6 - $104000)/2 = (dst<<5) - $2000
							dma_dst <= (dma_dst_sel << 5) - 16'h2000;
							// MAME size = (((size<<5) - (dst<<6) + 0x20)/2)
							dma_cnt <= ((dma_size_sel << 4)
							           - (dma_dst_sel << 5)
							           + 16'd16);
							// dma_mode_lat = $80 → branch dedicato D_FADE_*
							dma_mode_lat <= cop_dma_mode;
							// MAME: mode 4/5 = fade palette; QUALSIASI ALTRO = copia semplice
							// src->dst (seibucop_dma.ipp:119-123). SeibuCup intro pietre-sprite:
							// copia a sprite RAM $105800 con mode!=4/5. copy80 -> path fill
							// (decode regione, incl. SPR), dest assoluta = dst<<6 (byte addr).
							dma_copy80  <= (cop_pal_brightness_mode != 16'd4)
							            && (cop_pal_brightness_mode != 16'd5);
							dma_dst_abs <= {2'd0, dma_dst_sel, 6'd0};
							fsm <= D_PREP;
						end
						9'h116: begin
							// MAME seibucop.cpp cop_dma_trigger_w case 0x116 — SEIBUCUP:
							// fill come 0x118 ma SENZA guard dst!=0 e length bytes =
							// (size+1)<<4 (metà). legionna.cpp:56-61: "The COP-MCU appears
							// to write to the work ram area, otherwise it RESETS in
							// mid-animation of the title screen" → senza questo fill
							// il gioco torna al logo da solo.
							dma_src <= {2'd0, dma_src_sel, 6'd0};
							// length bytes = (size+1)<<4 → word = (size+1)<<3
							dma_cnt <= ((dma_size_sel + 16'd1) << 3);
							fsm     <= D_FILL_W;
						end
						9'h118, 9'h119, 9'h11A, 9'h11B,
						9'h11C, 9'h11D, 9'h11E, 9'h11F: begin
							// Skip if dst != 0 (MAME guard, dma.ipp:135)
							if (dma_dst_sel != 16'd0) begin
								fsm <= S_IDLE;
							end else begin
								dma_src <= {2'd0, dma_src_sel, 6'd0};
								// length bytes = (size+1) << 5; we step in words → cnt = (size+1)<<4
								dma_cnt <= ((dma_size_sel + 16'd1) << 4);
								fsm     <= D_FILL_W;
							end
						end
						default: fsm <= S_IDLE;
					endcase
				end else if (sort_pending) begin
					sort_i       <= 6'd0;
					sort_j       <= 6'd0;
					sort_pass    <= 6'd0;
					sort_swapped <= 1'b0;
					dma_src_byte <= sort_lookup_addr;
					fsm          <= M_SORT_LD_B;
				end else if (cmd_pending) begin
					// MAME cop_cmd_w (seibucop.cpp:1073): cop_status &= 0x7fff (clear bit 15).
					// NO set bit 2:0 = 111 (era assunzione errata).
					// MAME: cop_status &= 0x7fff (clear bit 15). cop_status[1] gestito altrove.
				cop_status[15]   <= 1'b0;
				cop_status[14:2] <= cop_status[14:2];
				cop_status[0]    <= cop_status[0];
					// Latch dell'offset del comando: i 3 trigger 0905 ($100/$102/$104)
					// condividono cmd_offset. Senza questo latch, il trigger successivo
					// sovrascrive cmd_offset mentre la macro multi-ciclo lo usa ->
					// 0905 offset 1 legge accel da $64 (neg) invece di $60 (+) ->
					// bruto-martello accelera all'indietro. macro_offset lo congela.
					macro_offset <= cmd_offset;
					// LEGACY sprite-DMA (match a maschera, MAME find_trigger_match):
					//   c480 mask $FF00 -> qualunque c4xx; 6880 mask $FFF3.
					// c480 = single-step: compone l'entry sprite a cop_regs[4]+offs
					// (info+param, X = rel + objX - abs_x, Y idem) — e' il COP che
					// muove TUTTI gli oggetti-sprite di godzilla (pietre incluse).
					// MAME LEGACY_cop_cmd_w: find_trigger_match(data, 0xF800) →
					// slot per GRUPPO f800, poi handler dal microcodice caricato.
					// SeibuCup (tabella standard): gruppo $C000-$C7FF → c480,
					// gruppo $6800-$6FFF → 6980. Match per gruppo, NON literal.
					// SEIBUCUP: i template sprite-DMA stanno in ROM 68k ($30880+,
					// disasm $2150/$1e08: src = puntatore template ROM). Le read
					// info/rel con src < $100000 vanno sulla porta cop_rom (SDRAM),
					// NON sul mux BRAM/Main-RAM (che per la ROM da' garbage →
					// entry con bit15=0 → nessuno sprite).
					if ((cmd_value & 16'hf800) == 16'hc000) begin : c4_disp
						reg [23:0] c4a;
						c4a = {cop_spr_dma_src_hi[7:0], cop_spr_dma_src_lo}
						      + {20'd0, cmd_offset, 2'd0};
						spr_src_is_rom <= (c4a < 24'h100000);
						if (c4a < 24'h100000) begin
							cop_rom_addr <= c4a;
							cop_rom_req  <= 1'b1;
						end else begin
							dma_src_byte <= c4a;
						end
						fsm <= M_C4_A;
					end else if ((cmd_value & 16'hf800) == 16'h6800) begin : c68_disp
						reg [23:0] a68;
						a68 = {cop_spr_dma_src_hi[7:0], cop_spr_dma_src_lo}
						      + 24'd4 + {20'd0, cmd_offset, 2'd0};
						spr_src_is_rom <= (a68 < 24'h100000);
						if (a68 < 24'h100000) begin
							cop_rom_addr <= a68;
							cop_rom_req  <= 1'b1;
						end else begin
							dma_src_byte <= a68;
						end
						fsm <= M_68_WAIT;
					end else if ((cmd_value & 16'hf800) == 16'h6000) begin
						// 6200 rotate-towards: leggi word angolo (regs1 host+0x0E)
						dma_src_byte <= cop_regs_byte_addr(1, 16'h000E);
						fsm <= M_62_A;
					end else
					case (cmd_value)
						// ════════════════════════════════════════════════════════════
						// 0x0205 — linear movement (offset = 0)
						//   ppos = *(cop_regs[0]+4)
						//   npos = ppos + *(cop_regs[0]+0x10)
						//   *(cop_regs[0]+4) = npos
						//   delta = (npos>>16) - (ppos>>16)
						//   *(cop_regs[0]+0x1E) += delta
						// 0204 = 0205 con l'addendo NEGATO: il gioco lo usa per
						// ANNULLARE il passo appena applicato quando l'oggetto
						// finisce fuori dal campo ($01F6D0/$01F6D6 avanzano,
						// $006EDA/$006EE0/$006EE6 e $01F6E6/$01F6EC riportano
						// indietro). MAME lo esegue come una somma, quindi
						// raddoppia il passo invece di annullarlo.
						16'h0204, 16'h0205: begin
							// MAME execute_0205: read dword at cop_regs[0]+4+offset*4 (ppos)
							dma_src_byte <= cop_regs_byte_addr(0, 16'h0004 + {12'd0, cmd_offset, 2'd0});
							fsm <= M_0205_RD_PPOS_HI;
						end

						// 0x0905 — jump: *(cop_regs[0]+0x10) += *(cop_regs[0]+0x28)
						// Il PRIMO read (vel_hi) NON viene emesso qui col cmd_offset: il
						// dispatch usa cmd_offset, gli stati interni macro_offset. Se i 3
						// trigger 0905 ($100/$102/$104) sono back-to-back e un trigger
						// successivo cambia cmd_offset tra il dispatch e il primo stato
						// interno, vel_hi (da cmd_offset) e vel_lo/grav (da macro_offset)
						// finiscono su OFFSET DIVERSI -> vel/grav MISTI tra assi -> per
						// l'offset 1 ($48) il 0905 fa $48 += valore sbagliato (es. $64
						// gravita' invece di $60 decel) -> $48 ACCELERA invece di decelerare
						// -> la martellata-scivolata non si ferma e va fuori schermo.
						// FIX: il primo read si emette in M_0905_RD0 usando macro_offset
						// (ormai registrato e stabile), coerente con tutti gli altri read.
						16'h0905, 16'h0904: begin
							fsm <= M_0905_RD0;
						end

						// 0x138e / 0x338e — atan(dx/dy) MAME execute_338e.
						// Legge gli STESSI 8 word del 3BB0 (dword r1+4,r0+4,r1+8,r0+8).
						// Parte da r1+4 hi (offset 0x0004), come 3BB0/MAME.
						// Famiglia atan2. Tre slot, stesso microcodice a meno del
						// registro del secondo punto e delle ultime word:
						//   slot 02: 118e/130e/138e -> reg1 (bersaglio)
						//   slot 06: 330e/338e      -> reg1 (contrasto)
						//   slot 1C: e18e/e30e/e38e -> reg2 = LA PALLA ($011AA2)
						// Chi SCRIVE l'angolo in RAM lo decide il campo LUNGHEZZA
						// (len = ((cmd>>7)&7)+1), non il bit 7: vedi M_138E_WR.
						16'h118e, 16'h130e, 16'h138e, 16'h330e, 16'h338e,
						16'he18e, 16'he30e, 16'he38e: begin
							// Lo slot 06 ($330e/$338e) prende il secondo punto da reg1,
							// NON da reg2. Lo impone il microcodice che il gioco carica
							// nel chip (ROM $00B828): le word $0AA4/$0AA2 hanno
							// reg = (v >> 5) & 3 = 1, mentre lo slot 1C ($00B960) usa
							// $0AC4/$0AC2 = reg 2. Il chip esegue il microcodice.
							// Il fallback CPU a $006C3A (cmp.l $4(a0),$4(a2), a2 = palla)
							// prova solo che il programmatore VOLEVA reg2: la voce di
							// tabella e' un copia-incolla dello slot 02 col solo offset
							// cambiato ($38 al posto di $34), e intento e hardware
							// divergono. E' lo stesso tipo di caso del 6200, dove la
							// tabella del ROM ha smentito il fork ed e' risultata giusta.
							// Al trigger reg1 non e' spazzatura: vale oggetto+$40, il
							// campo su cui il d104 scrive il bersaglio di zona
							// ($013436 addi.l #$40,d1); misurato in gioco r1 = r2 + $40.
							atan_src_r2  <= (cmd_value[15:11] == 5'h1c);
							dma_src_byte <= cop_regs_byte_addr(
							                  (cmd_value[15:11] == 5'h1c) ? 32'd2 : 32'd1,
							                  16'h0004);
							fsm <= M_138E_RD0_Y_HI;
						end

						// 0x3bb0 — dist (Pythagoras)
						// 3bb0 — distanza. NON ricalcola dx/dy dai registri: usa
						// quelli LATCHATI dall'ultimo comando d'angolo eseguito
						// (130e/138e/118e -> reg1, e30e/e18e/e38e -> reg2 = la
						// palla). Ricalcolandoli su reg1, dopo un e30e la distanza
						// e' quella dal bersaglio di zona e il gate del contatto
						// ($006CDE, dist < $3C) fallisce quasi sempre: i giocatori
						// attraversano la palla senza prenderla.
						16'h3bb0: begin
							fsm <= M_3BB0_CALC2;
						end

						// 0x42c2 / 0x4aa0 — divide. Endian asimmetrico (review D1):
						// 42c2 div=cop_read_word(r0+0x36)→host 0x34; 4aa0 div=read RAW 0x38.
						16'h42c2: begin
							dma_src_byte <= cop_regs_byte_addr(0, 16'h0034);  // 0x36^2
							fsm <= M_42C2_RD_DIV;
						end
						16'h4aa0: begin
							dma_src_byte <= cop_regs_byte_addr(0, 16'h0038);  // RAW
							fsm <= M_42C2_RD_DIV;
						end

						// 0x5105 / 0xf105 / 0x5905 — moltiplicatore 16.16 (M_MUL_*).
						// Emette qui la word ALTA della sorgente; M_MUL_A la bassa.
						//   5105  r0+00 -> r0+04   (t = (2/g)*Vz, e catene generiche)
						//   f105  r0+10 -> r0+10   (in place, coeff. $C000 = 0.75)
						//   5905  r2+10 -> r1+04   (V*t = spostamento; r2 = oggetto
						//                           palla, r1 = obj+$40 -> obj+$44/$48)
						// d104 — BERSAGLIO POSIZIONALE di zona:
						//   dst[r1+$04+4n] = (palla[r2+$04+4n] - origine[r3+4n]) * K >> 16
						// r2 = la palla ($011AA2), r1 = obj+$40 ($013040),
						// r3 = $0010A2A0 precaricato da ROM $00F3B0 con
						// {$04200000, $01800000} = {1056.0, 384.0} in 16.16.
						// Subito dopo il 68k somma il minimo della zona
						// ($01307E/$013098): bersaglio = zonaMin + (palla-org)*lato.
						// Senza, obj+$44/$48 non viene mai aggiornato e i giocatori
						// inseguono un punto sempre piu' lontano: escono dal campo.
						16'hd104: begin
							mul_off4     <= {12'd0, cmd_offset, 2'd0};
							mul_sub_org  <= 1'b1;
							mul_src_reg  <= 2'd2;
							mul_src_off  <= 16'h0004;
							mul_dst_reg  <= 2'd1;
							mul_dst_off  <= 16'h0004;
							dma_src_byte <= cop_regs_byte_addr(32'd2,
							                  16'h0004 + {12'd0, cmd_offset, 2'd0});
							fsm <= M_MUL_A;
						end

						// 0xdde5 — I PALLINI DEL RADAR.
						// MAME LEGACY_execute_dde5 (seibucop_cmd.ipp):
						//   offs       = (offset & 3) * 4
						//   div        = read_word(cop_regs[4] + offs)
						//   dir_offset = read_word(cop_regs[4] + offs + 8)
						//   if (div == 0) div = 1
						//   write_word(cop_regs[6] + offs + 4,
						//              (read_word(cop_regs[5] + offs + 4) + dir_offset) / div)
						// Letture RAW (read_word, non cop_read_word): nessun xor.
						//
						// A cosa serve: e' la scala del RADAR. cop_regs[4] = $10A280,
						// caricato al boot da ROM $00F3A0 con 000F 0014 FC69 07F4:
						//   porta 0 -> Y_radar = (Y - 919) / 15   in regs6+$04
						//   porta 1 -> X_radar = (X + 2036) / 20  in regs6+$08
						// La routine $004554 triggera dde5 sulle due porte e SUBITO
						// dopo copia $C(a0),$8(a0),$4(a0) in spriteram (8 byte = una
						// entry sprite): sono proprio i due campi appena calcolati.
						// Senza dde5 restano invariati e tutti i pallini finiscono
						// alle stesse coordinate, in alto a sinistra fuori schermo.
						16'hdde5: begin
							dde5_offs    <= {12'd0, cmd_offset, 2'd0};
							dma_src_byte <= cop_regs_byte_addr(32'd4,
							                  {12'd0, cmd_offset, 2'd0});
							fsm <= M_DDE5_A;
						end

						16'h5105, 16'hf105, 16'h5905: begin
							mul_off4     <= {12'd0, cmd_offset, 2'd0};
							mul_sub_org  <= 1'b0;
							mul_src_reg  <= (cmd_value == 16'h5905) ? 2'd2 : 2'd0;
							mul_src_off  <= (cmd_value == 16'h5105) ? 16'h0000 : 16'h0010;
							mul_dst_reg  <= (cmd_value == 16'h5905) ? 2'd1 : 2'd0;
							mul_dst_off  <= (cmd_value == 16'hf105) ? 16'h0010 : 16'h0004;
							dma_src_byte <= cop_regs_byte_addr(
							                  (cmd_value == 16'h5905) ? 32'd2 : 32'd0,
							                  ((cmd_value == 16'h5105) ? 16'h0000 : 16'h0010)
							                  + {12'd0, cmd_offset, 2'd0});
							fsm <= M_MUL_A;
						end

						// 0x8100 — sin. MAME: raw_angle = cop_read_word(r0+0x34) =
						// host_read_word(0x34^2 = 0x36); amp = cop_read_word(r0+0x36) =
						// host_read_word(0x36^2 = 0x34). Host endianness word^2.
						// → angle a host+0x36 (letto in tmp_lo), amp a host+0x34 (letto in CALC).
						16'h8100: begin
							dma_src_byte <= cop_regs_byte_addr(0, 16'h0036);
							fsm <= M_SC_RD_ANG;
							tmp_hi <= 16'h0001; // tag = sin (write +0x10 dword)
						end

						// 0x8900 — cos
						16'h8900: begin
							dma_src_byte <= cop_regs_byte_addr(0, 16'h0036);
							fsm <= M_SC_RD_ANG;
							tmp_hi <= 16'h0002; // tag = cos (write +0x14 dword)
						end

						// 0xa180 — collision read pos for slot 0
						// MAME cop_collision_read_pos: cop_read_word(spradr+N) =
						// host_read_word((spradr+N)^2) per host_endian=true.
						// flags @ cop+2 → host+0; pos[i] @ cop+6+4i → host+4+4i.
						16'ha180: begin
							coll_allow_swap[0] <= 1'b1; // bit 7 of data
							dma_src_byte <= cop_regs_byte_addr(0, 16'h0000);
							tmp_hi <= 16'h0000; // tag = slot 0
							fsm <= M_A1_RD_FLAGS;
						end

						// 0xa980 — collision read pos for slot 1
						16'ha980: begin
							coll_allow_swap[1] <= 1'b1;
							dma_src_byte <= cop_regs_byte_addr(1, 16'h0000);
							tmp_hi <= 16'h0001; // tag = slot 1
							fsm <= M_A1_RD_FLAGS;
						end

						// 0xb100/0xb000 — update hitbox slot 0. Il puntatore cop_regs[2]
						// e in ROM -> tutte le letture via cop_rom_* (handshake).
						// b000 NON e' un no-op: il dispatcher LEGACY di godzilla
						// (seibucop.cpp:1365) matcha con mask 0xf800, quindi la patch
						// init_godzilla (b100->b000) ESEGUE comunque l'update ma con
						// num_axis=2 (bit8=0 = Z disattivata). Trattarlo da no-op
						// lasciava hit_status=0 = "collide su tutto" -> morti immediate.
						16'hb100, 16'hb000: begin
							hb_ptr_addr <= cop_regs_byte_addr(2, 16'h0000);
							tmp_hi <= 16'h0000;
							hb_axis3 <= cmd_value[8];
							fsm <= M_B1_REQ_PTR;
						end

						// 0xb900/0xb800 — update hitbox slot 1 (idem: bit8 = num assi)
						16'hb900, 16'hb800: begin
							hb_ptr_addr <= cop_regs_byte_addr(3, 16'h0000);
							tmp_hi <= 16'h0001;
							hb_axis3 <= cmd_value[8];
							fsm <= M_B1_REQ_PTR;
						end

						default: begin
							// unknown macro — return idle
							fsm <= S_IDLE;
						end
					endcase
				end
			end

			// ════════════════════════════════════════════════════════════════
			// DMA path: src is byte addr, advances by 2 per word.
			// dst is local word index for VRAM/Pal target.
			// Mode $80-$87 (fade): 2 read (src=paldata, src+adr_rel*$400=target)
			//                       + 1 write con blend mode 5.
			// ════════════════════════════════════════════════════════════════
			D_PREP: begin
				dma_src_byte <= dma_src;
				fsm <= D_READ;
			end
			D_READ: begin
				// BRAM staging/main-RAM port B: output REGISTRATO, latency 1. L'addr
				// (word i) e' emesso a fine ciclo precedente (D_PREP / loop), quindi
				// in D_READ dma_src_rdata = mem[i] NON e' ancora valido: contiene
				// mem[i-1]. Per il DMA non-fade scriviamo dma_*_wdata DIRETTAMENTE da
				// dma_src_rdata in D_WRITE (1 ciclo dopo, mem[i] valido). dma_*_wdata
				// sono gia' registri (1 livello logico) -> nessun path lungo da
				// spezzare qui, quindi NIENTE latch anticipato (causava renderer[i]=
				// staging[i-1] -> palette sprite shiftata -> silhouette).
				if (dma_mode_lat[7:4] == 4'h8 && !dma_copy80) begin
					// FADE (mode 0x8x, pal_brightness_mode 4/5): src rdata valido al
					// prossimo ciclo → D_FADE_RD_TGT. Se copy80 (mode!=4/5) → D_WRITE
					// (copia src->dst via path fill, come mode 0x0e ma verso la regione
					// decodificata dal dst assoluto, incl. sprite RAM).
					fsm <= D_FADE_RD_TGT;
				end else begin
					fsm <= D_WRITE;
				end
			end
			D_WRITE: begin
				if (dma_copy80 || dma_copy_gen)
					dma_dst_abs <= dma_dst_abs + 24'd2;  // copia: avanza dest assoluta
				// (Il ramo copia 0x8x verso Main RAM non serve su cupsoc:
				// pal_brightness_mode vale 5 dal boot alla fine — unica write
				// $0006D8 move.w #$0005,$5C(a1) con a1=$100400 — quindi
				// dma_copy80 non puo' mai valere 1 e le DMA $8x prendono
				// sempre il ramo FADE. vedi la documentazione del progetto
				case (dma_mode_lat[3:0])
					4'h4: begin // 0x14 tilemap
						dma_vram_we    <= 1'b1;
						dma_vram_addr  <= dma_dst[12:0];
						dma_vram_wdata <= dma_src_rdata;   // mem[i] valido (latency 1, D_WRITE = emit+1)
					end
					4'h5: begin // 0x15 palette renderer
						dma_pal_we     <= 1'b1;
						dma_pal_addr   <= dma_dst[10:0];
						dma_pal_wdata  <= dma_src_rdata;
					end
					4'h9, 4'hE: begin // RAM-to-RAM copy
						// dst nelle BRAM video (dma_copy_gen): write via path fill
						// (dst assoluto, decode regione in main_top) — la write mr
						// finirebbe nell'ombra main-RAM mai letta dalla CPU.
						if (!dma_copy_gen) begin
							dma_ram_we     <= 1'b1;
							dma_ram_addr   <= dma_dst;
							dma_ram_wdata  <= dma_src_rdata;
						end
					end
					default: ;
				endcase
				dma_src <= dma_src + 24'd2;
				dma_dst <= dma_dst + 16'd1;
				if (dma_cnt == 16'd1) begin
					fsm <= S_IDLE;
				end else begin
					dma_cnt      <= dma_cnt - 16'd1;
					dma_src_byte <= dma_src + 24'd2;
					// Se questo ciclo ha alzato dma_ram_we (copia 0x09/0x0E con
					// dst in Main RAM), il we e' un impulso che cade nel ciclo
					// SUCCESSIVO: in D_READ ruberebbe mr_addr alla lettura e la
					// word i+1 verrebbe letta dall'indirizzo di destinazione i
					// (main_top.sv: mr_addr = we ? ram_addr : src_byte).
					// Un ciclo di attesa lascia consumare l'impulso, come fanno
					// gia' D_FADE_WAIT_TGT per il fade e M_0205_WAIT_SCRN per il
					// macro 0205. Colpiva la copia $106000 -> $105000 (ROM
					// $005D00), che al CONTINUE prepara il TARGET del fade: da
					// li' la palette nera.
					if ((dma_mode_lat[3:0] == 4'h9 || dma_mode_lat[3:0] == 4'hE)
					    && !dma_copy_gen)
						fsm <= D_COPY_WAIT;
					else
						fsm <= D_READ;
				end
			end
			D_COPY_WAIT: begin
				// il we e' gia' rientrato: ora mr_addr segue dma_src_byte
				fsm <= D_READ;
			end

			// ── MODE 5 FADE: 1° rdata = paldata. Emit target addr. ─────────
			D_FADE_RD_TGT: begin
				tmp_lo <= dma_src_rdata;                                // paldata latched
				dma_src_byte <= dma_src + ({8'd0, cop_dma_adr_rel} * 24'h400);
				fsm <= D_FADE_WAIT_TGT;
			end
			D_FADE_WAIT_TGT: begin
				// 1 ciclo wait per BRAM read latency
				fsm <= D_FADE_CALC;
			end
			D_FADE_CALC: begin
				// Pipeline stage 1: calcola i 6 fade_table_fn (6 mul) e REGISTRA.
				// Spezza il path lungo (6 mul + somme in 1 ciclo era -3ns).
				// Mode 5 (Legionnaire): indice fade = val[4:0], bypass bit15 "fade
				// me not". Mode 4 (SEIBUCUP/Denjin, seibucop_dma.ipp:87-117):
				// indice = (val*2)[4:0], val 0x0010/0xFFFF = TARGET PURO,
				// NESSUN bypass bit15.
				begin : fade_calc_blk
					reg [15:0] paldata, tgtdata;
					reg  [4:0] bv, bv_xor_inv;
					reg        mode4;
					paldata    = tmp_lo;
					tgtdata    = dma_src_rdata;
					mode4      = (cop_pal_brightness_mode == 16'd4);
					bv         = mode4 ? {cop_pal_brightness_val[3:0], 1'b0}
					                   : cop_pal_brightness_val[4:0];
					bv_xor_inv = bv ^ 5'h1F;
					fade_paldata <= paldata;
					fade_tgtdata <= tgtdata;
					fade_nofade  <= mode4 ? 1'b0 : paldata[15];
					fade_pure    <= mode4 && ((cop_pal_brightness_val == 16'h0010) ||
					                          (cop_pal_brightness_val == 16'hFFFF));
					// La tabella (che contiene un moltiplicatore) e' passata a D_FADE_TAB:
					// qui c'era in serie alla lettura dalla palette RAM ed era il collo.
				end
				fsm <= D_FADE_TAB;
			end
			// Stadio intermedio: le sei voci di tabella, calcolate su dati GIA'
			// registrati. Il moltiplicatore non e' piu' in serie con la BRAM.
			D_FADE_TAB: begin : fade_tab_blk
				reg [4:0] bv2, bv2_inv;
				bv2     = (cop_pal_brightness_mode == 16'd4)
				          ? {cop_pal_brightness_val[3:0], 1'b0}
				          : cop_pal_brightness_val[4:0];
				bv2_inv = bv2 ^ 5'h1F;
				fade_fb_t <= fade_table_fn({fade_tgtdata[14:10], bv2});
				fade_fb_c <= fade_table_fn({fade_paldata[14:10], bv2_inv});
				fade_fg_t <= fade_table_fn({fade_tgtdata[9:5],  bv2});
				fade_fg_c <= fade_table_fn({fade_paldata[9:5],  bv2_inv});
				fade_fr_t <= fade_table_fn({fade_tgtdata[4:0],  bv2});
				fade_fr_c <= fade_table_fn({fade_paldata[4:0],  bv2_inv});
				fsm <= D_FADE_WRITE;
			end
			D_FADE_WRITE: begin
				// Pipeline stage 2: somme (input registrati = path corto) + write.
				begin : fade_wr_blk
					reg [15:0] pal_val;
					reg  [4:0] out_b, out_g, out_r;
					if (fade_pure) begin
						// Mode 4, val 0x10/0xFFFF: colore TARGET puro (bit15 scartato)
						pal_val = {1'b0, fade_tgtdata[14:0]};
					end else if (fade_nofade) begin
						pal_val = fade_paldata;
					end else begin
						// Tronca OGNI canale a 5 bit PRIMA del concat: senza i temp
						// reg[4:0], (8bit+8bit)&8'h1F resta 8-bit self-determined ->
						// concat 25-bit -> troncato a 16 -> G shiftato, B perso ->
						// palette corrotta -> sprite "scompaiono". (regressione pipeline)
						out_b = (fade_fb_t + fade_fb_c) & 8'h1F;
						out_g = (fade_fg_t + fade_fg_c) & 8'h1F;
						out_r = (fade_fr_t + fade_fr_c) & 8'h1F;
						pal_val = {1'b0, out_b, out_g, out_r};
					end
					// MAME: m_host_space->write_word(dst, pal_val) — dst ASSOLUTO.
					// Registro il valore e committo in D_FADE_COMMIT via canale
					// fill (BRAM per regione) o mr (Main RAM): niente piu' output
					// forzato su pal_stage, e nessuna collisione col src-read
					// (il commit ha il suo ciclo dedicato).
					fade_out_val <= pal_val;
				end
				fsm <= D_FADE_COMMIT;
			end
			D_FADE_COMMIT: begin
				// dma_fill_we/addr/wdata (combinatori, vedi assign) scrivono la
				// BRAM giusta quando dst_abs e' in $101000-$105FFF; Main RAM qui.
				if (!dst_abs_is_vram) begin
					dma_ram_we    <= 1'b1;
					dma_ram_addr  <= dma_dst_abs[16:1];
					dma_ram_wdata <= fade_out_val;
				end
				dma_src <= dma_src + 24'd2;
				dma_dst <= dma_dst + 16'd1;
				dma_dst_abs <= dma_dst_abs + 24'd2;
				if (dma_cnt == 16'd1) begin
					fsm <= S_IDLE;
				end else begin
					dma_cnt      <= dma_cnt - 16'd1;
					dma_src_byte <= dma_src + 24'd2;
					fsm          <= D_READ;
				end
			end
			D_FILL_W: begin
				// Fill verso VRAM/PAL/SPR staging → via dma_fill_* (combinatorio,
				// vedi assign sotto la FSM). Fill verso Main RAM → qui. Mai
				// entrambi (evita doppia scrittura). Pattern HeatedBarrel.
				dma_ram_we    <= ~fill_is_vram;
				dma_ram_addr  <= dma_src[16:1];
				// MAME (0x118 e 0x116): write_dword(i, v1 | v2<<16) su spazio BE 68k
				// → word a i = v2, word a i+2 = v1 (alternati). Identico a prima
				// quando v1==v2 (clear a zero).
				dma_ram_wdata <= dma_src[1] ? cop_dma_v1 : cop_dma_v2;
				dma_src <= dma_src + 24'd2;
				if (dma_cnt == 16'd1) begin
					fsm <= S_IDLE;
				end else begin
					dma_cnt <= dma_cnt - 16'd1;
				end
			end

			// ════════════════════════════════════════════════════════════════
			// Macro 0x0205 — linear movement (MAME execute_0205)
			//   ppos = read_dword(cop_regs[0]+0x04)
			//   vel  = read_dword(cop_regs[0]+0x10)
			//   npos = ppos + vel
			//   write_dword(cop_regs[0]+0x04, npos)
			//   delta = (npos>>16) - (ppos>>16)
			//   write_word(cop_regs[0]+0x1E, read_word(cop_regs[0]+0x1E) + delta)
			// dma_ram_addr è WORD addr → byte_addr[16:1]
			// ════════════════════════════════════════════════════════════════
			M_0205_RD_PPOS_HI: begin
				dma_src_byte <= cop_regs_byte_addr(0, 16'h0006 + {12'd0, macro_offset, 2'd0});
				fsm <= M_0205_RD_PPOS_LO;
			end
			M_0205_RD_PPOS_LO: begin
				tmp_hi <= dma_src_rdata;                          // ppos hi latched
				dma_src_byte <= cop_regs_byte_addr(0, 16'h0010 + {12'd0, macro_offset, 2'd0});
				fsm <= M_0205_RD_VEL_HI;
			end
			M_0205_RD_VEL_HI: begin
				tmp_lo <= dma_src_rdata;                          // ppos lo latched
				dma_src_byte <= cop_regs_byte_addr(0, 16'h0012 + {12'd0, macro_offset, 2'd0});
				fsm <= M_0205_RD_VEL_LO;
			end
			M_0205_RD_VEL_LO: begin
				tmp32_a <= {tmp_hi, tmp_lo};                      // ppos
				tmp32_b <= {dma_src_rdata, 16'h0};                // partial vel (hi)
				fsm <= M_0205_WR_NPOS_HI;
			end
			M_0205_WR_NPOS_HI: begin : npos_blk
				reg [31:0] vel32, npos32;
				vel32  = {tmp32_b[31:16], dma_src_rdata};
				// bit0 del comando: 1 = somma (0205), 0 = sottrae (0204)
				npos32 = cmd_value[0] ? (tmp32_a + vel32) : (tmp32_a - vel32);
				tmp32_b[15:0] <= dma_src_rdata;                   // vel lo
				tmp32_a       <= npos32;
				dma_ram_we    <= 1'b1;
				dma_ram_addr  <= cop_regs_byte_addr(0, 16'h0004 + {12'd0, macro_offset, 2'd0}) >> 1;
				// La word ALTA deve venire dalla STESSA npos della word bassa
				// (M_0205_WR_NPOS_LO scrive tmp32_a[15:0] = npos32[15:0]) e della
				// coordinata schermo: prima era sempre la SOMMA -> per 0204 la
				// dword in obj+$04 era ibrida e divergeva dallo sprite.
				dma_ram_wdata <= npos32[31:16];
				fsm <= M_0205_WR_NPOS_LO;
			end
			M_0205_WR_NPOS_LO: begin
				dma_ram_we    <= 1'b1;
				dma_ram_addr  <= cop_regs_byte_addr(0, 16'h0006 + {12'd0, macro_offset, 2'd0}) >> 1;
				dma_ram_wdata <= tmp32_a[15:0];
				fsm <= M_0205_RD_SCRN;
			end
			M_0205_RD_SCRN: begin
				// MAME cop_read/write_word(cop_regs[0]+0x1e) = host word @ (0x1e^2)=0x1c
				// (host endianness word^2; conferma doc 06 micro-op 'addmem16 0x1C(r0)').
				dma_src_byte <= cop_regs_byte_addr(0, 16'h001C + {12'd0, macro_offset, 2'd0});
				fsm <= M_0205_WAIT_SCRN;
			end
			M_0205_WAIT_SCRN: begin
				// Wait 1 stato: read 0x1C valida 2 stati dopo emit (latency BRAM=2).
				// Senza, WR_SCRN leggeva addr vecchio (vel_lo) -> scroll corrotto.
				scrn_delta <= tmp32_a[31:16] - tmp_hi;   // non dipende dalla lettura
				fsm <= M_0205_WR_SCRN;
			end
			M_0205_WR_SCRN: begin
				dma_ram_we    <= 1'b1;
				dma_ram_addr  <= cop_regs_byte_addr(0, 16'h001C + {12'd0, macro_offset, 2'd0}) >> 1;
				dma_ram_wdata <= dma_src_rdata + scrn_delta;
				fsm <= S_IDLE;
			end

			// ════════════════════════════════════════════════════════════════
			// Macro 0x0905/0x0904 — jump (MAME execute_0904)
			//   write_dword(cop_regs[0]+0x10, read_dword(cop_regs[0]+0x10)
			//                                  ± read_dword(cop_regs[0]+0x28))
			//   0x0905 (bit 0=1) = +, 0x0904 (bit 0=0) = -
			// ════════════════════════════════════════════════════════════════
			M_0905_RD0: begin
				// Primo read vel_hi @ 0x10+macro_offset*4 (macro_offset ora STABILE,
				// registrato al dispatch). Tutti i read del 0905 usano lo stesso offset.
				dma_src_byte <= cop_regs_byte_addr(0, 16'h0010 + {12'd0, macro_offset, 2'd0});
				fsm <= M_0905_RD_HI;
			end
			M_0905_RD_HI: begin
				dma_src_byte <= cop_regs_byte_addr(0, 16'h0012 + {12'd0, macro_offset, 2'd0});
				fsm <= M_0905_RD_LO;
			end
			M_0905_RD_LO: begin
				tmp_hi <= dma_src_rdata;
				dma_src_byte <= cop_regs_byte_addr(0, 16'h0028 + {12'd0, macro_offset, 2'd0});
				fsm <= M_0905_RD_GRAV_HI;
			end
			M_0905_RD_GRAV_HI: begin
				tmp_lo <= dma_src_rdata;
				dma_src_byte <= cop_regs_byte_addr(0, 16'h002A + {12'd0, macro_offset, 2'd0});
				fsm <= M_0905_RD_GRAV_LO;
			end
			M_0905_RD_GRAV_LO: begin
				// MAME execute_0904: bit 0 di data = 1 → add, =0 → sub
				// tmp32_a = vel ± grav. vel = {tmp_hi, tmp_lo}. grav_hi = dma_src_rdata.
				if (cmd_value[0])
					tmp32_a <= {tmp_hi, tmp_lo} + {dma_src_rdata, 16'h0};
				else
					tmp32_a <= {tmp_hi, tmp_lo} - {dma_src_rdata, 16'h0};
				fsm <= M_0905_WR_HI;
			end
			// Ciclo A: SOLO la somma bassa a 17 bit; il bit 16 e' il carry (o il
			// borrow). Prima qui c'erano una somma, un confronto e una seconda somma
			// in fila dopo la lettura BRAM: era il percorso critico del core.
			M_0905_WR_HI: begin
				if (cmd_value[0]) {p905_cb, p905_low} <= {1'b0, tmp32_a[15:0]} + {1'b0, dma_src_rdata};
				else              {p905_cb, p905_low} <= {1'b0, tmp32_a[15:0]} - {1'b0, dma_src_rdata};
				dma_ram_we <= 1'b0;   // la scrittura avviene nel ciclo B
				fsm <= M_0905_WR_HI_B;
			end
			// Ciclo B: solo la somma alta, sul carry gia' registrato.
			M_0905_WR_HI_B: begin
				tmp32_a[15:0] <= p905_low;
				dma_ram_wdata <= cmd_value[0] ? (tmp32_a[31:16] + {15'd0, p905_cb})
				                              : (tmp32_a[31:16] - {15'd0, p905_cb});
				dma_ram_we    <= 1'b1;
				dma_ram_addr  <= cop_regs_byte_addr(0, 16'h0010 + {12'd0, macro_offset, 2'd0}) >> 1;
				fsm <= M_0905_WR_LO;
			end
			M_0905_WR_LO: begin
				dma_ram_we    <= 1'b1;
				dma_ram_addr  <= cop_regs_byte_addr(0, 16'h0012 + {12'd0, macro_offset, 2'd0}) >> 1;
				dma_ram_wdata <= tmp32_a[15:0];
				fsm <= S_IDLE;
			end

			// ════════════════════════════════════════════════════════════════
			// 0x138E/0x338E atan(dx/dy) — MAME execute_338e (cmd.ipp:159)
			//   dx = read_dword(r1+4) - read_dword(r0+4)
			//   dy = read_dword(r1+8) - read_dword(r0+8)
			//   if(!dy){status|=0x8000; angle=0} else angle=atan(dx/dy)*128/pi; dy<0→+0x80
			//   if(data&0x80) write_byte(r0+0x34, angle)
			// dword 68k BE: hi@off+0, lo@off+2. Latenza BRAM 2 cicli. Dispatch emette r1+4(hi).
			// ════════════════════════════════════════════════════════════════
				// Read chain IDENTICA al 3BB0 (latenza BRAM 2 cicli). Mappatura:
				// tmp32_a=r1x, tmp32_b=r0x, math_dx=r1y, math_dy=r0y. Dispatch r1+4 hi.
				M_138E_RD0_Y_HI: begin
					dma_src_byte <= cop_regs_byte_addr(atan_src_r2 ? 32'd2 : 32'd1, 16'h0006);
					fsm <= M_138E_RD0_Y_LO;
				end
				M_138E_RD0_Y_LO: begin
					tmp32_a[31:16] <= dma_src_rdata;
					dma_src_byte <= cop_regs_byte_addr(0, 16'h0004);
					fsm <= M_138E_RD1_Y_HI;
				end
				M_138E_RD1_Y_HI: begin
					tmp32_a[15:0] <= dma_src_rdata;
					dma_src_byte <= cop_regs_byte_addr(0, 16'h0006);
					fsm <= M_138E_RD1_Y_LO;
				end
				M_138E_RD1_Y_LO: begin
					tmp32_b[31:16] <= dma_src_rdata;
					dma_src_byte <= cop_regs_byte_addr(atan_src_r2 ? 32'd2 : 32'd1, 16'h0008);
					fsm <= M_138E_RD0_X_HI;
				end
				M_138E_RD0_X_HI: begin
					tmp32_b[15:0] <= dma_src_rdata;
					dma_src_byte <= cop_regs_byte_addr(atan_src_r2 ? 32'd2 : 32'd1, 16'h000A);
					fsm <= M_138E_RD0_X_LO;
				end
				M_138E_RD0_X_LO: begin
					math_dx[31:16] <= dma_src_rdata;
					dma_src_byte <= cop_regs_byte_addr(0, 16'h0008);
					fsm <= M_138E_RD1_X_HI;
				end
				M_138E_RD1_X_HI: begin
					math_dx[15:0] <= dma_src_rdata;
					dma_src_byte <= cop_regs_byte_addr(0, 16'h000A);
					fsm <= M_138E_RD1_X_LO;
				end
				M_138E_RD1_X_LO: begin
					math_dy[31:16] <= dma_src_rdata;
					fsm <= M_138E_CALC;
				end
				M_138E_CALC: begin
					// MAME: angle = atan(dx/dy)*128/pi (+0x80 se dy<0). Il vectoring
					// accumula atan(y/x) con x>0. Per ottenere atan(dx/dy) col segno
					// giusto di dy: x=|dy|, y = (dy<0)? -dx : dx. Cosi' z = atan(dx/dy)
					// e il +0x80 (in WR) copre solo il quadrante, come MAME.
					// NIENTE prescale: i dword diff sono FIXED-POINT coord<<16 (come il
					// 3BB0 che usa dx32[31:16] e MAME dx>>16), NON interi piccoli. Un
					// <<8 strariperebbe signed[31:0] per |delta|>=128px -> overflow ->
					// angolo sbagliato. atan e' scale-invariant: i dword pieni bastano
					// (max |axdy|~2^25, *gain 1.65 ~2^26, dentro signed[31:0]).
					// Ciclo A: SOLO le due differenze a 32 bit. Dopo la lettura BRAM ci deve
					// stare una sottrazione sola: il valore assoluto (che e' un'altra somma a
					// 32 bit) e il carico del CORDIC passano al ciclo B.
					calc_dx <= $signed(tmp32_a) - $signed(tmp32_b);
					calc_dy <= $signed(math_dx) - $signed({math_dy[31:16], dma_src_rdata});
					fsm <= M_138E_CALC_B;
				end
				// Ciclo B: valori assoluti e carico del CORDIC, su valori gia' registrati.
				M_138E_CALC_B: begin
					cordic_x <= (calc_dy < 0) ? -calc_dy : calc_dy;   // |dy|
					cordic_y <= (calc_dy < 0) ? -calc_dx : calc_dx;   // dx col segno di dy
					cordic_z <= 24'sd0;
					math_dy  <= calc_dy;
					latch_dx <= calc_dx;   // per il 3bb0 che segue
					latch_dy <= calc_dy;
					cordic_i <= 5'd0;
					fsm <= M_138E_CORDIC;
				end
				// Ciclo A: solo shift, segno e tabella. Nessuna somma.
				M_138E_CORDIC: begin
					cordic_xs  <= cordic_x >>> cordic_i;
					cordic_ys  <= cordic_y >>> cordic_i;
					cordic_neg <= (cordic_y < 0);
					cordic_lut <= cordic_atan_lut(cordic_i);
					fsm <= M_138E_CORDIC_B;
				end
				// Ciclo B: solo somme, su operandi gia' registrati.
				M_138E_CORDIC_B: begin
					if (!cordic_neg) begin
						cordic_x <= cordic_x + cordic_ys;
						cordic_y <= cordic_y - cordic_xs;
						cordic_z <= cordic_z + cordic_lut;
					end else begin
						cordic_x <= cordic_x - cordic_ys;
						cordic_y <= cordic_y + cordic_xs;
						cordic_z <= cordic_z - cordic_lut;
					end
					if (cordic_i == 5'd23) fsm <= M_138E_WR;
					else begin
						cordic_i <= cordic_i + 5'd1;
						fsm      <= M_138E_CORDIC;
					end
				end
				M_138E_WR: begin
					begin : atan_wr_blk
						reg signed [15:0] ang_unit;
						reg        [7:0]  ang_byte;
						// MAME (int)(...) tronca VERSO ZERO. cordic_z e' ×65536 (16-bit
						// frazione, 24 iter) -> ang_unit = z>>16 toward-zero.
						// BUG TROVATO (validato vs MAME su 29241 casi): il CORDIC SOTTO-CONVERGE
						// di 1 LSB sulle DIAGONALI (es. dx==dy -> z=2097151 invece di 2097152=
						// 32.0×65536). z>>16 toward-zero da' 31 invece di 32 -> cop_angle 0x9f
						// invece di 0xa0 -> settore $48DA sbagliato (0x9f>>3=0x13 vs 0xa0>>3=0x14)
						// -> d1 bit0 errato -> clr.l $48 NON scatta quando il bruto e' in DIAGONALE
						// col player -> il martello scivola nella posa. Compenso l'epsilon di
						// sotto-convergenza con +1 toward-zero PRIMA del troncamento: bias=1
						// azzera tutte le 133 divergenze (bias=0) -> 0/29241 vs MAME.
						begin : ang_round
							reg signed [23:0] z_comp;
							z_comp  = (cordic_z >= 0) ? (cordic_z + 24'sd1) : (cordic_z - 24'sd1);
							ang_unit = (z_comp >= 0) ? (z_comp >>> 16)
							                         : -((-z_comp) >>> 16);
						end
						// MAME execute_338e (seibucop_cmd.ipp:175-188): la write dell'angolo
						// in RAM (obj+0x34 host, gated da data&0x80) e' FUORI dall'if/else di
						// dy -> avviene SEMPRE, anche a dy==0 (dove cop_angle=0 -> scrive 0^0x80
						// = 0x80). Il mio scriveva SOLO nel ramo dy!=0 -> a dy==0 (bruto
						// allineato vert col player) l'angolo in RAM NON si aggiornava -> il
						// sincos rileggeva l'angolo VECCHIO -> velocita' sbagliata -> la mossa
						// scivolata tornava indietro. Ora calcolo ang_byte in entrambi i rami
						// e scrivo SEMPRE, come MAME.
						// is_yflip (SeibuCup TAD): XOR 0x80 SOLO sulla write RAM (obj+0x34,
						// che il SINCOS 8100 rilegge), NON sul registro cop_angle readback.
						// STATO BUONO (EDIT#238, martello NON scivola, confermato HW
						// 2026-06-08): con dy==0 il 138e mette cop_angle=0 e status[15]=1
						// ma NON scrive l'angolo in RAM (la write resta DENTRO il ramo
						// dy!=0). Scrivere l'angolo anche a dy==0 ("1:1 MAME") REGREDISCE:
						// il martello scivola/moonwalk. Ramo dy==0 INTOCCABILE.
						if (math_dy == 32'sd0) begin
							ang_byte       = 8'h00;          // MAME: dy==0 -> cop_angle=0
							cop_angle      <= 16'h0000;
							cop_status[15] <= 1'b1;
						end else begin
							ang_byte       = ang_unit[7:0] + ((math_dy < 0) ? 8'h80 : 8'h00);
							cop_angle      <= {8'h00, ang_byte};          // readback: SENZA xor
							// status[15] guarda la PARTE INTERA, non i 32 bit.
							// Il chip lavora in PIXEL: le tabelle del bootleg mettono
							// distanza e angolo nella STESSA word, indicizzata da
							// (A<<9)|B con A,B a 9 bit (la documentazione del progetto),
							// e il nostro stesso 3bb0 tronca gia' i delta con
							// dx32[31:16] (come MAME m_LEGACY_r0>>16). Solo il 138e
							// li guardava in 16.16: con |dy| sotto il pixel ma non
							// zero, status[15] non si alzava MAI.
							// EFFETTO: il gioco non eseguiva il proprio fallback
							// ($006344/$00634C, che forza l'angolo cardinale $40/$C0
							// dove cos vale ESATTAMENTE 0) e la telecamera, inseguendo
							// la palla in rimessa laterale, oscillava di 1 px per frame
							// senza punto fisso. Stesso difetto in MAME (if (!dx) sui
							// 32 bit), da cui il sintomo identico nei due emulatori.
							// La SOPPRESSIONE della write in RAM resta gated dal ramo
							// dy==0 ESATTO qui sopra, intoccabile (EDIT#238, HW
							// 2026-06-08: allargarla fa scivolare il martello).
							cop_status[15] <= (math_dy[31:16] == 16'sd0);
							// Chi scrive l'angolo NON e' il bit 7 ma la LUNGHEZZA:
							// len = ((cmd>>7)&7)+1 = word di microcodice eseguite.
							//   118e/e18e len 4 -> caricano solo dx/dy: NON scrivono
							//   130e/e30e len 7 -> calcolano l'angolo, non lo scrivono
							//   138e/e38e len 8 -> scrivono
							// Con la regola del bit 7 (quella di MAME) e18e sovrascrive
							// obj+$37 su tutti gli oggetti ogni frame, fra il calcolo del
							// movimento e quello dell'orientamento: i giocatori camminano
							// giusto ma restano girati male.
							if ((cmd_value[9:7] + 3'd1) == 3'd0) begin   // len 8 (wrap a 0)
								dma_ram_we    <= 1'b1;
								dma_ram_be    <= 2'b01;
								dma_ram_addr  <= cop_regs_byte_addr(0, 16'h0036) >> 1;
								// SEIBUCUP: write RAW — il ROM carica il microcode 138e
								// variante cupsoc/SDGundam (tabella $498: ...b9a b9a A9A)
								// = MAME LEGACY_execute_130e_cupsoc, SENZA ^0x80 (il ^0x80
								// e' della variante famiglia $B9A / is_yflip: legionna).
								// PROVA sim 8 casi: col ^0x80 vettore sincos INVERTITO
								// 180° vs MAME → boss 138e-driven marcia via (stage 4).
								// ⛔ NON toccare il sincos (lookup diretto = sin MAME):
								// toccarlo inverte i game-written = regressione build 13.
								dma_ram_wdata <= {8'h00, ang_byte};  // RAW (cupsoc variant)
							end
						end
					end
					cop_status[2]    <= 1'b1;
					cop_status[14:3] <= 12'd0;
					cop_status[0]    <= 1'b1;
					fsm <= S_IDLE;
				end


			// ════════════════════════════════════════════════════════════════
			// 0x3BB0 dist — MAME execute_3b30 (cmd.ipp:197)
			//   dx = (read_dword(r1+4) - read_dword(r0+4)) >> 16  (aritmetico)
			//   dy = (read_dword(r1+8) - read_dword(r0+8)) >> 16
			//   cop_dist = sqrt(dx*dx + dy*dy)
			//   if(data&0x80) write_word(r0 + (data&0x200?0x3a:0x38), dist)
			//   3bb0: bit 0x200 set → cop 0x3a → host word (0x3a^2)=0x38.
			// Stesse 8 letture word di atan. Dispatch emette r1+4(hi).
			// ════════════════════════════════════════════════════════════════
			// 0x3BB0 dist — MAME execute_3b30 (cmd.ipp:197): 1:1
			//   dx = (read_dword(r1+4) - read_dword(r0+4)) >> 16  (signed)
			//   dy = (read_dword(r1+8) - read_dword(r0+8)) >> 16
			//   cop_dist = floor(sqrt(dx*dx + dy*dy))
			//   if(data&0x80) write_word(r0+(data&0x200?0x3a:0x38), dist)
			// Accumulatori: tmp32_a=r1x, tmp32_b=r0x, math_dx=r1y, math_dy=r0y.
			// Dispatch ha emesso r1+4(hi). Latenza BRAM: rdata valido 2 stati dopo emit.
			M_3BB0_LOAD: begin
				dma_src_byte <= cop_regs_byte_addr(1, 16'h0006);  // r1+4 lo
				fsm <= M_3BB0_RD_X1LO;
			end
			M_3BB0_RD_X1LO: begin
				tmp32_a[31:16] <= dma_src_rdata;                  // r1x hi
				dma_src_byte <= cop_regs_byte_addr(0, 16'h0004);  // r0+4 hi
				fsm <= M_3BB0_RD_X0HI;
			end
			M_3BB0_RD_X0HI: begin
				tmp32_a[15:0] <= dma_src_rdata;                   // r1x lo
				dma_src_byte <= cop_regs_byte_addr(0, 16'h0006);  // r0+4 lo
				fsm <= M_3BB0_RD_X0LO;
			end
			M_3BB0_RD_X0LO: begin
				tmp32_b[31:16] <= dma_src_rdata;                  // r0x hi
				dma_src_byte <= cop_regs_byte_addr(1, 16'h0008);  // r1+8 hi
				fsm <= M_3BB0_RD_Y1HI;
			end
			M_3BB0_RD_Y1HI: begin
				tmp32_b[15:0] <= dma_src_rdata;                   // r0x lo
				dma_src_byte <= cop_regs_byte_addr(1, 16'h000A);  // r1+8 lo
				fsm <= M_3BB0_RD_Y1LO;
			end
			M_3BB0_RD_Y1LO: begin
				math_dx[31:16] <= dma_src_rdata;                  // r1y hi
				dma_src_byte <= cop_regs_byte_addr(0, 16'h0008);  // r0+8 hi
				fsm <= M_3BB0_RD_Y0HI;
			end
			M_3BB0_RD_Y0HI: begin
				math_dx[15:0] <= dma_src_rdata;                   // r1y lo
				dma_src_byte <= cop_regs_byte_addr(0, 16'h000A);  // r0+8 lo
				fsm <= M_3BB0_RD_Y0LO;
			end
			M_3BB0_RD_Y0LO: begin
				math_dy[31:16] <= dma_src_rdata;                  // r0y hi
				fsm <= M_3BB0_CALC;
			end
			M_3BB0_CALC: begin
				// Pipeline stage 0 (timing): registra SOLO la differenza r0y completata
				// con dma_src_rdata (dato dalla Main RAM = path critico -5.3ns).
				// Spezza il path RAM→sottrazione→mul. Il resto in CALC2/CALC3.
				math_dy <= {math_dy[31:16], dma_src_rdata};  // completa r0y lo
				fsm <= M_3BB0_CALC2;
			end
			M_3BB0_CALC2: begin
				// Stage 1: SOLO differenze 32-bit (registra dxh/dyh in tmp32_a/b[15:0]).
				// Spezza sottrazione dalla moltiplicazione (era catena sub->mul 12ns).
				begin : dist_diff_blk
					reg signed [31:0] dx32, dy32;
					// delta LATCHATI dall'ultimo comando d'angolo, non rlietti:
					// dopo un e30e sono riferiti alla PALLA (reg2), dopo un 130e
					// al bersaglio (reg1). E' il TODO dichiarato in MAME.
					dx32 = latch_dx;
					dy32 = latch_dy;
					tmp32_a <= {16'd0, dx32[31:16]};             // dxh registrato
					tmp32_b <= {16'd0, dy32[31:16]};             // dyh registrato
				end
				fsm <= M_3BB0_CALCM;
			end
			M_3BB0_CALCM: begin
				// Stage 2: moltiplicazioni (input registrati = path corto).
				math_dx <= $signed(tmp32_a[15:0]) * $signed(tmp32_a[15:0]);  // dx*dx
				math_dy <= $signed(tmp32_b[15:0]) * $signed(tmp32_b[15:0]);  // dy*dy
				fsm <= M_3BB0_CALC3;
			end
			M_3BB0_CALC3: begin
				// Stage 3: somma i prodotti registrati + init sqrt.
				sqrt_radsh <= math_dx + math_dy;
				sqrt_rem34 <= 34'd0;
				sqrt_acc   <= 32'd0;
				sqrt_i     <= 5'd16;
				fsm <= M_3BB0_SQRT;
			end
			M_3BB0_SQRT: begin
				// restoring integer sqrt, 1 bit/ciclo, 16 iter.
				begin : dist_sqrt_blk
					reg [33:0] rem_next;
					reg [17:0] cand;
					rem_next = {sqrt_rem34[31:0], sqrt_radsh[31:30]};
					cand     = {sqrt_acc[15:0], 2'b01};          // (root<<2)|1
					if (rem_next >= {16'd0, cand}) begin
						sqrt_rem34 <= rem_next - {16'd0, cand};
						sqrt_acc   <= {sqrt_acc[14:0], 1'b1};    // (root<<1)|1
					end else begin
						sqrt_rem34 <= rem_next;
						sqrt_acc   <= {sqrt_acc[14:0], 1'b0};    // root<<1
					end
					sqrt_radsh <= sqrt_radsh << 2;
				end
				if (sqrt_i == 5'd1) fsm <= M_3BB0_WR;
				else                sqrt_i <= sqrt_i - 5'd1;
			end
			M_3BB0_WR: begin
				cop_dist <= sqrt_acc[15:0];                      // floor(sqrt(rad))
				if (cmd_value[7]) begin                          // data&0x80
					dma_ram_we    <= 1'b1;
					// Offset di scrittura distanza. (Il fix MAME-aligned 0x3a:0x38
					// faceva fuggire i nemici -> MAME invertito vs HW. Ripristinato.)
					dma_ram_addr  <= cmd_value[9]                // data&0x200 ? 0x38 : 0x3a
					                 ? (cop_regs_byte_addr(0, 16'h0038) >> 1)
					                 : (cop_regs_byte_addr(0, 16'h003a) >> 1);
					dma_ram_wdata <= sqrt_acc[15:0];
				end
				// MAME: macro completata → cop_status = 7 (gate-poll del gioco).
				// bit[1] gestito dal cpu_wr_pulse block → NON toccare.
				cop_status[2]    <= 1'b1;
				cop_status[14:3] <= 12'd0;
				cop_status[0]    <= 1'b1;
				cop_status[15]   <= 1'b0;
				fsm <= S_IDLE;
			end

			// 0x42C2/0x4AA0 divide — MAME execute_42c2/4aa0 (cmd.ipp:219/247): 1:1
			//   DIVIDEND = cop_dist << (5 - cop_scale)   (cop_scale 0..3 → shift 5..2)
			//   42c2: div=cop_read_word(r0+0x36)→host 0x34; if(!div){status|=0x8000;
			//         write 0 a host 0x3a} else write DIVIDEND/div a host 0x3a.
			//   4aa0: div=read RAW 0x38; if(!div)div=1; write DIVIDEND/div a host 0x36.
			//   Divide = restoring unsigned 32-iter (1 bit/ciclo, no operatore /).
			//   Dispatch ha emesso src corretto. Latenza BRAM: rdata=div in M_42C2_CALC.
			M_42C2_RD_DIV: begin
				fsm <= M_42C2_CALC;   // wait latenza BRAM
			end
			M_42C2_CALC: begin
				// dma_src_rdata = div. cop_status=7 (gate-poll); bit1 non toccare.
				cop_status[2]    <= 1'b1;
				cop_status[14:3] <= 12'd0;
				cop_status[0]    <= 1'b1;
				cop_status[15]   <= 1'b0;
				if (dma_src_rdata == 16'd0) begin
					if (cmd_value == 16'h42c2) begin
						// 42c2 div0: status|=0x8000, write 0 a host 0x3a, return
						cop_status[15] <= 1'b1;
						dma_ram_we    <= 1'b1;
						dma_ram_addr  <= cop_regs_byte_addr(0, 16'h003a) >> 1;
						dma_ram_wdata <= 16'd0;
						fsm <= S_IDLE;
					end else begin
						// 4aa0 div0: div=1 → result=DIVIDEND. Procedi.
						div_den <= 16'd1;
						case (cop_scale[1:0])
							2'd0: div_num <= {11'd0, cop_dist} << 5;
							2'd1: div_num <= {11'd0, cop_dist} << 4;
							2'd2: div_num <= {11'd0, cop_dist} << 3;
							2'd3: div_num <= {11'd0, cop_dist} << 2;
						endcase
						div_q <= 32'd0; div_r <= 32'd0; div_i <= 6'd32;
						fsm   <= M_42C2_DIV;
					end
				end else begin
					div_den <= dma_src_rdata;
					case (cop_scale[1:0])
						2'd0: div_num <= {11'd0, cop_dist} << 5;
						2'd1: div_num <= {11'd0, cop_dist} << 4;
						2'd2: div_num <= {11'd0, cop_dist} << 3;
						2'd3: div_num <= {11'd0, cop_dist} << 2;
					endcase
					div_q <= 32'd0; div_r <= 32'd0; div_i <= 6'd32;
					fsm   <= M_42C2_DIV;
				end
			end
			// ════════════════════════════════════════════════════════════════
			// dde5 — pallini del radar. Vedi il dispatch per la derivazione.
			// ════════════════════════════════════════════════════════════════
			M_DDE5_A: begin
				// indirizzo di div gia' emesso dal dispatch; ora quello di dir_offset
				dma_src_byte <= cop_regs_byte_addr(32'd4, dde5_offs + 16'd8);
				fsm <= M_DDE5_B;
			end
			M_DDE5_B: begin
				// div valido. MAME: if (div == 0) div = 1
				dde5_div     <= (dma_src_rdata == 16'd0) ? 16'd1 : dma_src_rdata;
				dma_src_byte <= cop_regs_byte_addr(32'd5, dde5_offs + 16'd4);
				fsm <= M_DDE5_C;
			end
			M_DDE5_C: begin
				dde5_dir <= $signed(dma_src_rdata);   // dir_offset valido
				fsm <= M_DDE5_D;
			end
			M_DDE5_D: begin : dde5_blk
				// valore valido: num = read_word(regs5+offs+4) + dir_offset.
				// In MAME read_word e' unsigned e dir_offset int16_t: la somma e'
				// int, la divisione int/int -> troncamento VERSO ZERO.
				reg signed [31:0] num;
				num = $signed({16'd0, dma_src_rdata}) + $signed({{16{dde5_dir[15]}}, dde5_dir});
				dde5_neg <= (num < 0);
				div_num  <= num[31] ? (32'd0 - num) : num;   // divido il valore assoluto
				div_den  <= dde5_div;
				div_q    <= 32'd0;
				div_r    <= 32'd0;
				div_i    <= 6'd32;
				fsm      <= M_DDE5_DIV;
			end
			M_DDE5_DIV: begin : dde5_div_blk
				// stessa divisione restoring del 42c2, ma sul valore assoluto
				reg [32:0] rem_sh;
				rem_sh = {div_r[31:0], div_num[31]};
				if (rem_sh >= {17'd0, div_den}) begin
					div_r <= rem_sh - {17'd0, div_den};
					div_q <= (div_q << 1) | 32'd1;
				end else begin
					div_r <= rem_sh[31:0];
					div_q <= div_q << 1;
				end
				div_num <= div_num << 1;
				if (div_i == 6'd1) fsm <= M_DDE5_WR;
				else               div_i <= div_i - 6'd1;
			end
			M_DDE5_WR: begin
				// write_word(cop_regs[6] + offs + 4, risultato) — RAW, niente xor
				dma_ram_we    <= 1'b1;
				dma_ram_addr  <= cop_regs_byte_addr(32'd6, dde5_offs + 16'd4) >> 1;
				dma_ram_wdata <= dde5_neg ? (16'd0 - div_q[15:0]) : div_q[15:0];
				fsm <= S_IDLE;
			end

			M_42C2_DIV: begin
				// Restoring division unsigned, 1 bit/ciclo, MSB-first, 32 iter.
				begin : div_blk
					reg [32:0] rem_sh;
					rem_sh = {div_r[31:0], div_num[31]};   // rem<<1 | num MSB
					if (rem_sh >= {17'd0, div_den}) begin
						div_r <= rem_sh - {17'd0, div_den};
						div_q <= (div_q << 1) | 32'd1;
					end else begin
						div_r <= rem_sh[31:0];
						div_q <= div_q << 1;
					end
					div_num <= div_num << 1;
				end
				if (div_i == 6'd1) fsm <= M_42C2_WR;
				else               div_i <= div_i - 6'd1;
			end
			M_42C2_WR: begin
				// div_q = DIVIDEND/div. Write 16-bit risultato.
				//   42c2 → host 0x3a (0x38^2) ; 4aa0 → host 0x36 (RAW)
				dma_ram_we    <= 1'b1;
				dma_ram_addr  <= (cmd_value == 16'h42c2)
				                 ? (cop_regs_byte_addr(0, 16'h003a) >> 1)
				                 : (cop_regs_byte_addr(0, 16'h0036) >> 1);
				dma_ram_wdata <= div_q[15:0];
				fsm <= S_IDLE;
			end

			// ════════════════════════════════════════════════════════════════
			// 0x8100/0x8900 sin/cos (MAME execute_8100/8900)
			//   raw_angle = cop_read_word(cop_regs[0]+0x34) & 0xff
			//   amp       = (65536>>5) * (cop_read_word(cop_regs[0]+0x36) & 0xff)
			//                = 2048 * amp_byte
			//   if (raw_angle == 0xC0 sin / 0x80 cos): amp *= 2
			//   res = int(amp * sin/cos(raw_angle * pi/128)) << cop_scale
			//   write_dword(cop_regs[0] + 0x10 sin / 0x14 cos, res)
			//
			// sin_table_lookup ritorna signed16 = sin(i*pi/128) * 32768
			// amp_full * sin_norm = (2048 * amp_byte) * (sin_lut/32768)
			//                     = (amp_byte * sin_lut) / 16
			// → in fixed: signed_product[31:0] = amp_byte * sin_lut → shift>>4
			// Per cos: usa angle+0x40
			// ════════════════════════════════════════════════════════════════
			M_SC_RD_ANG: begin
				// pre-emit amp byte addr (host+0x34). Dispatch ha emesso host+0x36
				// (angle); per latenza BRAM 2-cicli, tmp_lo catturerà read(0x36)=angle
				// e CALC vedrà dma_src_rdata=read(0x34)=amp.
				dma_src_byte <= cop_regs_byte_addr(0, 16'h0034);
				fsm <= M_SC_RD_AMP;
			end
			M_SC_RD_AMP: begin
				tmp_lo <= dma_src_rdata;  // angle word (host+0x36, byte basso = angle)
				fsm <= M_SC_CALC;
			end
			M_SC_CALC: begin
				// angle_byte = tmp_lo[7:0]; amp_byte = dma_src_rdata[7:0]
				// tmp_hi: 1=sin (write +0x10), 2=cos (write +0x14, angle+=0x40)
				begin : sc_blk
					reg  [7:0] raw_angle;
					reg  [7:0] angle_byte;
					reg  [8:0] amp_ext;             // 9-bit per doubling caso speciale
					reg signed [15:0] sin_v;
					reg signed [16:0] sin_full;
					reg signed [31:0] prod;
					reg signed [31:0] res;
					raw_angle  = tmp_lo[7:0];
					angle_byte = (tmp_hi == 16'd2) ? (raw_angle + 8'h40) : raw_angle;
					amp_ext    = {1'b0, dma_src_rdata[7:0]};
					// MAME special-case: sin && raw==0xC0 OR cos && raw==0x80 → amp *= 2
					if (tmp_hi == 16'd1 && raw_angle == 8'hC0) amp_ext = {dma_src_rdata[7:0], 1'b0};
					if (tmp_hi == 16'd2 && raw_angle == 8'h80) amp_ext = {dma_src_rdata[7:0], 1'b0};
					sin_v = sin_table_lookup(angle_byte);
					// PICCO al pieno SOLO nel calcolo velocita': la sin_table satura a 32767
					// (valore HW corretto per i nemici, NON toccare la tabella - project_sintable_fix).
					// Ma nel PRODOTTO l'HW reale usa 32768 pieno: al picco cos(0)/cos(180)
					// (sin_v=±32767) la vel $48=amp*32767>>4=131068 NON e' mult.16; la decel della
					// martellata-scivolata ($60=-$48/16, stop $48==0 esatto, state[11] $6E8C) scavalca
					// lo zero -> $48 runaway -> scivola fuori schermo. Con 32768 -> 131072 (mult.16)
					// -> ferma. Tocca SOLO il picco (1 angolo, 0x40), tabella INVARIATA per i nemici.
					if      (sin_v ==  16'sd32767) sin_full =  17'sd32768;
					else if (sin_v == -16'sd32767) sin_full = -17'sd32768;
					else                           sin_full = {sin_v[15], sin_v};
					prod  = $signed({1'b0, amp_ext}) * sin_full;
					// res = prod >> 4 (= /16 per matchare formula MAME 2048/32768 = 1/16)
					res = prod >>> 4;
					case (cop_scale[1:0])
						2'd0: tmp32_a <= res;
						2'd1: tmp32_a <= res <<< 1;
						2'd2: tmp32_a <= res <<< 2;
						2'd3: tmp32_a <= res <<< 3;
					endcase
				end
				dma_ram_we <= 1'b0;
				fsm <= M_SC_WR_HI;
			end
			M_SC_WR_HI: begin
				// write hi-word a +0x10 (sin) o +0x14 (cos)
				dma_ram_we    <= 1'b1;
				dma_ram_addr  <= (tmp_hi == 16'd1)
				                 ? (cop_regs_byte_addr(0, 16'h0010) >> 1)
				                 : (cop_regs_byte_addr(0, 16'h0014) >> 1);
				dma_ram_wdata <= tmp32_a[31:16];
				fsm <= M_SC_WR_LO;
			end
			M_SC_WR_LO: begin
				dma_ram_we    <= 1'b1;
				dma_ram_addr  <= (tmp_hi == 16'd1)
				                 ? (cop_regs_byte_addr(0, 16'h0012) >> 1)
				                 : (cop_regs_byte_addr(0, 16'h0016) >> 1);
				dma_ram_wdata <= tmp32_a[15:0];
				fsm <= S_IDLE;
			end

			// ════════════════════════════════════════════════════════════════
			// 0xa180/0xa980 — read pos (MAME cop_collision_read_pos)
			//   flags_swap = cop_read_word(spradr+2)  → host_read_word(spradr+0)
			//   pos[0] = cop_read_word(spradr+6)      → host_read_word(spradr+4)
			//   pos[1] = cop_read_word(spradr+10)     → host_read_word(spradr+8)
			//   pos[2] = cop_read_word(spradr+14)     → host_read_word(spradr+12)
			// spradr = cop_regs[0/1] (passato +2 da macro setup → leggo +2 da setup)
			// In macro setup ho già impostato dma_src_byte = cop_regs_byte_addr(N, +2).
			// Però MAME cop_read_word(+2) = host_read_word(+2^2) = host_read_word(+0).
			// Quindi flags effettivi sono a host_byte_addr = spradr+0.
			// ════════════════════════════════════════════════════════════════
			M_A1_RD_FLAGS: begin
				// Pre-emit addr pos[0] @ host+4. Wait state per BRAM latency.
				dma_src_byte <= (tmp_hi == 16'd0)
				                ? cop_regs_byte_addr(0, 16'h0004)
				                : cop_regs_byte_addr(1, 16'h0004);
				fsm <= M_A1_RD_POS_Y;
			end
			M_A1_RD_POS_Y: begin
				// rdata = word @ host+0 = flags (MAME cop_read_word(spradr+2) ^ 2).
				coll_flags_swap[tmp_hi[0]] <= dma_src_rdata;
				// Pre-emit pos[1] @ host+8.
				dma_src_byte <= (tmp_hi == 16'd0)
				                ? cop_regs_byte_addr(0, 16'h0008)
				                : cop_regs_byte_addr(1, 16'h0008);
				fsm <= M_A1_RD_POS_X;
			end
			M_A1_RD_POS_X: begin
				// rdata = word @ host+4 = pos[0] (Y axis per MAME)
				coll_pos[tmp_hi[0]][0] <= dma_src_rdata;
				// Pre-emit pos[2] @ host+12.
				dma_src_byte <= (tmp_hi == 16'd0)
				                ? cop_regs_byte_addr(0, 16'h000C)
				                : cop_regs_byte_addr(1, 16'h000C);
				fsm <= M_A1_RD_POS_Z;
			end
			M_A1_RD_POS_Z: begin
				// rdata = word @ host+8 = pos[1] (X axis)
				coll_pos[tmp_hi[0]][1] <= dma_src_rdata;
				fsm <= M_A1_RD_POS_ZW;
			end
			M_A1_RD_POS_ZW: begin
				// rdata = word @ host+12 = pos[2] (Z axis)
				coll_pos[tmp_hi[0]][2] <= dma_src_rdata;
				fsm <= S_IDLE;
			end

			// ════════════════════════════════════════════════════════════════
			// 0xb100/0xb900 — update hitbox & compute intersection (1:1 MAME
			// cop_collision_update_hitbox, seibucop.cpp:1003-1065). slot j=tmp_hi[0].
			// TUTTE le letture sono in MAIN ROM (puntatore cop_regs[2/3]=$04xxxx +
			// descrittore 3 word @ hb_adr2+2*i). Lettura via cop_rom_* handshake:
			// REQ alza cop_rom_req 1 ciclo, WAIT stalla finche cop_rom_ready, latcha.
			// dx[i]=int8 [7:0] (signed), size[i]=uint8 [15:8] (unsigned).
			// hb_adr2 = read_word(regs[slot]) | (cop_hit_baseadr[7:0]<<16).
			// ════════════════════════════════════════════════════════════════
			M_B1_REQ_PTR: begin
				cop_rom_addr <= hb_ptr_addr;     // addr del puntatore (in ROM)
				cop_rom_req  <= 1'b1;
				fsm <= M_B1_WAIT_PTR;
			end
			M_B1_WAIT_PTR: begin
				cop_rom_req <= 1'b0;
				if (cop_rom_ready) begin
					// cop_rom_rdata = read_word(cop_regs[slot]) = base descrittore.
					hb_adr2 <= {cop_hit_baseadr[7:0], cop_rom_rdata};
					fsm <= M_B1_REQ_H0;
				end
			end
			M_B1_REQ_H0: begin
				cop_rom_addr <= hb_adr2;          // asse0 @ +0
				cop_rom_req  <= 1'b1;
				fsm <= M_B1_WAIT_H0;
			end
			M_B1_WAIT_H0: begin
				cop_rom_req <= 1'b0;
				if (cop_rom_ready) begin
					hb_dx[0]   <= {{8{cop_rom_rdata[7]}}, cop_rom_rdata[7:0]};  // int8
					hb_size[0] <= {8'd0, cop_rom_rdata[15:8]};                 // uint8
					fsm <= M_B1_REQ_H1;
				end
			end
			M_B1_REQ_H1: begin
				cop_rom_addr <= hb_adr2 + 24'd2;  // asse1 @ +2
				cop_rom_req  <= 1'b1;
				fsm <= M_B1_WAIT_H1;
			end
			M_B1_WAIT_H1: begin
				cop_rom_req <= 1'b0;
				if (cop_rom_ready) begin
					hb_dx[1]   <= {{8{cop_rom_rdata[7]}}, cop_rom_rdata[7:0]};
					hb_size[1] <= {8'd0, cop_rom_rdata[15:8]};
					if (hb_axis3) begin
						fsm <= M_B1_REQ_H2;
					end else begin
						// 2 assi (b000/b800): terza word NON letta, dx/size azzerati
						// (seibucop.cpp:1014-1022)
						hb_dx[2]   <= 16'sd0;
						hb_size[2] <= 16'd0;
						fsm <= M_B1_CALC;
					end
				end
			end
			M_B1_REQ_H2: begin
				cop_rom_addr <= hb_adr2 + 24'd4;  // asse2 @ +4
				cop_rom_req  <= 1'b1;
				fsm <= M_B1_WAIT_H2;
			end
			M_B1_WAIT_H2: begin
				cop_rom_req <= 1'b0;
				if (cop_rom_ready) begin
					hb_dx[2]   <= {{8{cop_rom_rdata[7]}}, cop_rom_rdata[7:0]};
					hb_size[2] <= {8'd0, cop_rom_rdata[15:8]};
					fsm <= M_B1_CALC;
				end
			end
			// STAGE 1: solo box min/max (1 add/sub per canale) -> REGISTRA in
			// coll_min/max[j]. Path corto (1 livello aritmetico). I comparatori
			// overlap stanno nello stage 2 con input registrati.
			M_B1_CALC: begin : b1_calc_blk
				reg signed [15:0] nmin, nmax;
				reg signed [15:0] sz;
				integer i;
				reg j;
				j = tmp_hi[0];
				for (i = 0; i < 3; i = i + 1) begin
					sz = $signed({1'b0, hb_size[i][14:0]});
					if (coll_allow_swap[j] && coll_flags_swap[j][i]) begin
						nmax = $signed(coll_pos[j][i]) - hb_dx[i];
						nmin = nmax - sz;
					end else begin
						nmin = $signed(coll_pos[j][i]) + hb_dx[i];
						nmax = nmin + sz;
					end
					// 2 assi: l'asse 2 (Z) NON viene aggiornato (loop MAME i<num_axis)
					if (i < 2 || hb_axis3) begin
						coll_min[j][i] <= nmin;
						coll_max[j][i] <= nmax;
					end
				end
				// dY/dX/dZ: 1 sub per canale, indipendente (path corto separato)
				cop_hit_val[0] <= coll_pos[0][0] - coll_pos[1][0];
				cop_hit_val[1] <= coll_pos[0][1] - coll_pos[1][1];
				if (hb_axis3)
					cop_hit_val[2] <= coll_pos[0][2] - coll_pos[1][2];
				fsm <= M_B1_CALC2;
			end
			// STAGE 2: comparatori overlap su coll_min/max REGISTRATI (entrambi gli
			// slot ora validi). res init 7, clear bit i su overlap signed. Path
			// corto (solo comparatori, nessuna catena aritmetica a monte).
			M_B1_CALC2: begin : b1_calc2_blk
				reg [2:0] res;
				integer i;
				// res init: 3 assi = 111, 2 assi = 011 (seibucop.cpp:1038-39:
				// bit Z gia' 0 = "collide banale" -> hit deciso solo da X/Y)
				res = hb_axis3 ? 3'b111 : 3'b011;
				for (i = 0; i < 3; i = i + 1) begin
					if ((i < 2 || hb_axis3) &&
					    (($signed(coll_max[0][i]) > $signed(coll_min[1][i]) &&
					      $signed(coll_min[0][i]) < $signed(coll_max[1][i])) ||
					     ($signed(coll_max[1][i]) > $signed(coll_min[0][i]) &&
					      $signed(coll_min[1][i]) < $signed(coll_max[0][i]))))
						res[i] = 1'b0;   // overlap su asse i -> collide
				end
				cop_hit_status   <= {13'd0, res};
				cop_hit_val_stat <= {13'd0, res};
				// Segnala "macro completata" come fanno 138e/3bb0/8100 (il gioco
				// polla cop_status[2] prima di leggere cop_hit_status). Senza
				// questo il gioco leggeva il risultato a timing casuale (la lettura
				// ROM hitbox ha latency variabile) -> confine nemici INTERMITTENTE.
				cop_status[2]    <= 1'b1;
				cop_status[14:3] <= 12'd0;
				cop_status[0]    <= 1'b1;
				fsm <= S_IDLE;
			end

			// ════════════════════════════════════════════════════════════════
			// LEGACY 6880 — carica rel_xy per il prossimo c480
			// (MAME LEGACY_execute_6980: rel_xy = read_word(spr_src+4+offs))
			// ════════════════════════════════════════════════════════════════
			M_68_WAIT: begin
				cop_rom_req <= 1'b0;
				if (!spr_src_is_rom)
					fsm <= M_68_LATCH;   // wait latenza BRAM
				else if (cop_rom_ready) begin
					// src in ROM: dato dalla porta cop_rom (cupsoc template)
					spr_dma_rel_x <= cop_rom_rdata[7:0];
					spr_dma_rel_y <= cop_rom_rdata[15:8];
					fsm <= S_IDLE;
				end
			end
			M_68_LATCH: begin
				spr_dma_rel_x <= dma_src_rdata[7:0];
				spr_dma_rel_y <= dma_src_rdata[15:8];
				fsm <= S_IDLE;
			end

			// ════════════════════════════════════════════════════════════════
			// LEGACY c480 — single-step sprite DMA (MAME LEGACY_execute_c480):
			//   info = read(spr_src+offs) + (param & 0x3f) -> entry+0
			//   abs  = read(regs[0]+8/+4) - spr_dma_abs_x/y
			//   X(fx/dx/rel_x) -> entry+4 (+ x_clip per l'inc $410)
			//   Y(fy/rel_y)    -> entry+6
			//   param bit17 -> entry+2 |= 0x8000
			// Write assolute via canale fill (dst_abs), read con latenza 2.
			// ════════════════════════════════════════════════════════════════
			M_C4_A: begin
				cop_rom_req <= 1'b0;
				dma_dst_abs <= cop_regs_byte_addr(4, {10'd0, macro_offset, 2'd0});
				if (!spr_src_is_rom)
					fsm <= M_C4_B;
				else if (cop_rom_ready) begin
					spr_rom_lat <= cop_rom_rdata;   // info word dal template ROM
					fsm <= M_C4_B;
				end
			end
			M_C4_B: begin
				begin : c4_info_blk
					reg [15:0] info;
					info = (spr_src_is_rom ? spr_rom_lat : dma_src_rdata)
					       + {10'd0, cop_spr_dma_param_lo[5:0]};
					spr_info_l   <= info;
					fade_out_val <= info;
				end
				dma_src_byte <= cop_regs_byte_addr(0, 16'h0008);   // objX
				fsm <= M_C4_WRI;
			end
			M_C4_WRI: begin
				// write info @ entry+0 (canale fill se BRAM, mr altrimenti)
				if (!dst_abs_is_vram) begin
					dma_ram_we    <= 1'b1;
					dma_ram_addr  <= dma_dst_abs[16:1];
					dma_ram_wdata <= fade_out_val;
				end
				fsm <= M_C4_C;
			end
			M_C4_C: begin
				abs_x_r <= dma_src_rdata - cop_spr_dma_abs_x;
				dma_src_byte <= cop_regs_byte_addr(0, 16'h0004);   // objY
				fsm <= M_C4_D;
			end
			M_C4_D: begin
				begin : c4_x_blk
					reg [15:0] sx;
					if (spr_info_l[14])
						sx = (16'h0100 - ({9'd0, spr_info_l[12:10], 4'd0} + 16'd16))
						     + abs_x_r - {9'd0, spr_dma_rel_x[6:3], 3'd0}
						     - (spr_dma_rel_x[7] ? 16'h0080 : 16'h0100);
					else
						sx = {9'd0, spr_dma_rel_x[6:3], 3'd0} + abs_x_r
						     - (spr_dma_rel_x[7] ? 16'h0080 : 16'h0000);
					fade_out_val <= sx;
					x_clip_r     <= sx;
				end
				dma_dst_abs <= cop_regs_byte_addr(4, {10'd0, macro_offset, 2'd0} + 16'd4);
				fsm <= M_C4_WRX;
			end
			M_C4_WRX: begin
				if (!dst_abs_is_vram) begin
					dma_ram_we    <= 1'b1;
					dma_ram_addr  <= dma_dst_abs[16:1];
					dma_ram_wdata <= fade_out_val;
				end
				abs_y_r <= dma_src_rdata - cop_spr_dma_abs_y;      // objY valido ora
				fsm <= M_C4_E;
			end
			M_C4_E: begin
				begin : c4_y_blk
					reg [15:0] sy;
					if (spr_info_l[13])
						sy = abs_y_r + (spr_dma_rel_y[7] ? 16'h0080 : 16'h0000)
						     - {9'd0, spr_dma_rel_y[6:3], 3'd0};
					else
						sy = {9'd0, spr_dma_rel_y[6:3], 3'd0} + abs_y_r
						     - (spr_dma_rel_y[7] ? 16'h0080 : 16'h0000);
					fade_out_val <= sy;
				end
				dma_dst_abs <= cop_regs_byte_addr(4, {10'd0, macro_offset, 2'd0} + 16'd6);
				fsm <= M_C4_WRY;
			end
			M_C4_WRY: begin
				if (!dst_abs_is_vram) begin
					dma_ram_we    <= 1'b1;
					dma_ram_addr  <= dma_dst_abs[16:1];
					dma_ram_wdata <= fade_out_val;
				end
				if (cop_spr_dma_param_hi[1]) begin   // param bit17: priority flag
					dma_src_byte <= cop_regs_byte_addr(4, {10'd0, macro_offset, 2'd0} + 16'd2);
					fsm <= M_C4_F;
				end else begin
					fsm <= S_IDLE;
				end
			end
			M_C4_F: begin
				fsm <= M_C4_G;       // wait latenza BRAM
			end
			M_C4_G: begin
				fade_out_val <= dma_src_rdata | 16'h8000;
				dma_dst_abs  <= cop_regs_byte_addr(4, {10'd0, macro_offset, 2'd0} + 16'd2);
				fsm <= M_C4_WRP;
			end
			M_C4_WRP: begin
				if (!dst_abs_is_vram) begin
					dma_ram_we    <= 1'b1;
					dma_ram_addr  <= dma_dst_abs[16:1];
					dma_ram_wdata <= fade_out_val;
				end
				fsm <= S_IDLE;
			end

			// ════════════════════════════════════════════════════════════════
			// 6200 — rotate-towards. cupsoc carica il microcodice
			// 3a0 3a6 380 aa0 2a6 => MAME LEGACY_execute_6200 ("this is for
			// cupsoc, different sequence, works on different registers"):
			// regs[1]+0x0c, NON regs[0]+0x34 della variante famiglia.
			// Matematica identica alle due varianti, cambia solo l'indirizzo:
			//   angolo  = cop_read_byte (regs1+0x0c) -> host regs1+0x0F
			//   flags   = cop_read_word (regs1)      -> host regs1+0x02, bit2=raggiunto
			// Target RAW (il 138e di cupsoc scrive l'angolo raw, non ^0x80).
			// Write angolo = WORD {0,angle} (MAME host_endian write-word).
			// ════════════════════════════════════════════════════════════════
			M_62_A: begin
				fsm <= M_62_B;       // wait latenza BRAM (word angolo)
			end
			M_62_B: begin
				ang62_l <= dma_src_rdata[7:0];                    // byte host+0x0F
				dma_src_byte <= cop_regs_byte_addr(1, 16'h0002);  // flags (cop_read_word(regs1) = host+2)
				fsm <= M_62_C;
			end
			M_62_C: begin
				fsm <= M_62_D;       // wait latenza BRAM (flags)
			end
			M_62_D: begin
				begin : rot62_blk
					reg  [7:0] tgt8, step8, na;
					reg signed [8:0] sd;
					reg reached;
					tgt8  = cop_angle_target[7:0];                // RAW (RAM raw, variante cupsoc)
					step8 = cop_angle_step[7:0];
					sd = $signed({1'b0, ang62_l}) - $signed({1'b0, tgt8});
					// wrap a signed 8-bit (MAME: +=/-=256 su int)
					sd = $signed({sd[7], sd[7:0]});
					reached = 1'b0;
					na = ang62_l;
					if (sd < 0) begin
						if (sd >= -$signed({1'b0, step8})) begin
							na = tgt8; reached = 1'b1;
						end else
							na = ang62_l + step8;
					end else begin
						if (sd <= $signed({1'b0, step8})) begin
							na = tgt8; reached = 1'b1;
						end else
							na = ang62_l - step8;
					end
					ang62_l       <= na;
					// flags: &~4, |4 se raggiunto — write a regs1 host+2
					dma_ram_we    <= 1'b1;
					dma_ram_addr  <= cop_regs_byte_addr(1, 16'h0002) >> 1;
					dma_ram_wdata <= (dma_src_rdata & ~16'h0004) | (reached ? 16'h0004 : 16'h0000);
				end
				fsm <= M_62_E;
			end
			M_62_E: begin
				dma_ram_we    <= 1'b1;
				// MAME cop_write_byte(regs1+$0F): SOLO il byte basso (indirizzo
				// dispari -> be[0], main_top.sv). Con 2'b11 si azzererebbe anche
				// host+$0E = angolo del corpo, che $0134CE copia nello sprite
				// (move.b $f(a2),$f(a3)) -> giocatori orientati a caso.
				dma_ram_be    <= 2'b01;
				dma_ram_addr  <= cop_regs_byte_addr(1, 16'h000E) >> 1;
				dma_ram_wdata <= {8'h00, ang62_l};
				fsm <= S_IDLE;
			end

			// ════════════════════════════════════════════════════════════════
			// 5105 / f105 — MOLTIPLICATORE 16.16 (nessun handler in MAME:
			// ricavato da ROM + disasm di cupsoc).
			//   dst = (src * {$100446,$100448}) >>> 16    dword, signed
			//   5105: src = regs0+0x00+off*4  dst = regs0+0x04+off*4
			//   f105: src = dst = regs0+0x10+off*4        (in place)
			// PROVA: $01C95C.. mette in regs0 un buffer, scrive l'operando in
			// $46(a5), triggera 5105 e RILEGGE $4(a1), che poi SOMMA alla
			// posizione -> il risultato e' gia' in 16.16 (shift 16, non 32).
			// Costanti usate dal gioco: $C000=0.75 (f105, sempre), $D554=5/6,
			// $000C3EC1 = 2/|g| con g=$FFFFD630 -> t = (2/g)*Vz = tempo di volo:
			// chiude solo se l'operazione e' una MOLTIPLICAZIONE.
			// Il troncamento e' floor (>>> aritmetico): 2/|g| = 802497.6 e in
			// ROM c'e' 802497.
			// ════════════════════════════════════════════════════════════════
			M_MUL_A: begin
				// dispatch ha emesso l'indirizzo della word ALTA; ora la BASSA.
				dma_src_byte <= cop_regs_byte_addr({30'd0, mul_src_reg},
				                  mul_src_off + 16'd2 + mul_off4);
				fsm <= M_MUL_B;
			end
			M_MUL_B: begin
				mul_md[63:32] <= {32{dma_src_rdata[15]}};  // hi word, estesa in segno
				mul_md[31:16] <= dma_src_rdata;
				fsm <= M_MUL_C;
			end
			M_MUL_C: begin
				mul_md[15:0] <= dma_src_rdata;             // lo word
				mul_mr       <= {cop_opnd_hi, cop_opnd_lo};
				mul_acc      <= 64'd0;
				mul_i        <= 6'd32;
				if (mul_sub_org) begin
					// d104: serve (sorgente - origine). Leggo l'origine da reg3.
					dma_src_byte <= cop_regs_byte_addr(32'd3, mul_off4);
					fsm <= M_MUL_ORG_A;
				end else
					fsm <= M_MUL_LOOP;
			end
			M_MUL_ORG_A: begin
				dma_src_byte <= cop_regs_byte_addr(32'd3, mul_off4 + 16'd2);
				fsm <= M_MUL_ORG_B;
			end
			M_MUL_ORG_B: begin
				mul_org[31:16] <= dma_src_rdata;
				fsm <= M_MUL_ORG_C;
			end
			M_MUL_ORG_C: begin
				begin : org_sub
					reg signed [31:0] src32, org32;
					src32 = mul_md[31:0];
					org32 = {mul_org[31:16], dma_src_rdata};
					mul_md <= {{32{src32[31]}}, src32} - {{32{org32[31]}}, org32};
				end
				fsm <= M_MUL_LOOP;
			end
			M_MUL_LOOP: begin
				// shift-add signed: l'ultimo bit del moltiplicatore ha peso
				// NEGATIVO (complemento a due) -> si sottrae invece di sommare.
				if (mul_i == 6'd1) begin
					if (mul_mr[0]) mul_acc <= mul_acc - mul_md;
					fsm <= M_MUL_WR_HI;
				end else begin
					if (mul_mr[0]) mul_acc <= mul_acc + mul_md;
					mul_i <= mul_i - 6'd1;
				end
				mul_md <= mul_md <<< 1;
				mul_mr <= mul_mr >> 1;
			end
			M_MUL_WR_HI: begin
				dma_ram_we    <= 1'b1;
				dma_ram_addr  <= cop_regs_byte_addr({30'd0, mul_dst_reg},
				                   mul_dst_off + mul_off4) >> 1;
				dma_ram_wdata <= mul_acc[47:32];
				fsm <= M_MUL_WR_LO;
			end
			M_MUL_WR_LO: begin
				dma_ram_we    <= 1'b1;
				dma_ram_addr  <= cop_regs_byte_addr({30'd0, mul_dst_reg},
				                   mul_dst_off + 16'd2 + mul_off4) >> 1;
				dma_ram_wdata <= mul_acc[31:16];
				fsm <= S_IDLE;
			end

			// ════════════════════════════════════════════════════════════════
			// Z-sorting DMA ($1006FE) — MAME dma_zsorting (seibucop_dma.ipp:164)
			//   n   = data+1
			//   val = read_word(lookup + 2*i)                 (RAW, senza xor)
			//   key = (int16)cop_read_word(sort_ram + val)    (host (addr)^2, CON SEGNO)
			//   param 1 = crescente, 2 = decrescente, altro = nessun riordino
			//   write_word(lookup + 2*i, val)                 (RAW)
			// key = obj+$38 = la distanza scritta dal 3bb0 ($006A80) e poi decrementata
			// di 3 ($006A86): va letta CON SEGNO, altrimenti chi e' addosso alla palla
			// finisce in fondo alla lista e non contrasta mai.
			// ════════════════════════════════════════════════════════════════
			M_SORT_LD_B: fsm <= M_SORT_LD_C;      // latenza BRAM
			M_SORT_LD_C: begin
				sort_val[sort_i] <= dma_src_rdata;
				// La chiave e' la word CON SEGNO all'indirizzo ALLINEATO AL LONG:
				//     key = read_word( (sort_ram + val) & ~3 )
				// E' l'unica funzione che soddisfa i due testimoni del ROM:
				//   piani    $00363E: $110004 + val (val ≡0 mod 4) = $1157EC, e la
				//            chiave e' la Y che $003630 scrive in +$04  -> f(D)=D
				//   contatto $006A98: $11003A + val -> ≡2 mod 4, e la chiave e' la
				//            distanza che $006A86 tiene in +$38        -> f(D)=D-2
				// L'XOR ^2 (quello che c'era, e che MAME ha tuttora) sbaglia il primo:
				// legge +$06, campo che nessuna istruzione del ROM scrive mai, quindi
				// tutte le chiavi valgono 0 e il sort dei piani e' INERTE -> i corpi
				// restano in ordine di allocazione (squadra 1 tutta davanti alla 2).
				// Verificato in gioco su MAME il 2026-07-29.
				dma_src_byte <= (sort_ram_addr + {8'd0, dma_src_rdata}) & 24'hFFFFFC;
				fsm <= M_SORT_LD_D;
			end
			M_SORT_LD_D: fsm <= M_SORT_LD_E;      // latenza BRAM
			M_SORT_LD_E: begin
				sort_key[sort_i] <= dma_src_rdata;
				if ((sort_i + 6'd1) >= sort_n) begin
					sort_i       <= 6'd0;
					sort_j       <= 6'd0;
					sort_pass    <= 6'd0;
					sort_swapped <= 1'b0;
					fsm <= M_SORT_CMP;
				end else begin
					sort_i       <= sort_i + 6'd1;
					dma_src_byte <= sort_lookup_addr + {17'd0, (sort_i + 6'd1), 1'b0};
					fsm <= M_SORT_LD_B;
				end
			end
			// Bubble sort: swap SOLO se strettamente fuori ordine = stabile come
			// lo std::stable_sort di MAME. 1 confronto/ciclo, <= ~270 cicli per n=22.
			// Ciclo A: SOLO le letture indicizzate. Prima, in un ciclo solo, c'erano
			// due mux 60:1, il comparatore e il write-enable di tutti e 60 i registri
			// in fila: era il percorso critico del core. Qui resta il mux e basta.
			M_SORT_CMP: begin
				sort_ka <= sort_key[sort_j];
				sort_kb <= sort_key[sort_j + 6'd1];
				sort_va <= sort_val[sort_j];
				sort_vb <= sort_val[sort_j + 6'd1];
				if (({1'b0, sort_j} + {1'b0, sort_pass} + 7'd1) >= {1'b0, sort_n}) begin
					if (!sort_swapped || (({1'b0, sort_pass} + 7'd2) >= {1'b0, sort_n})) begin
						sort_i <= 6'd0;
						fsm    <= M_SORT_ST;
					end else begin
						sort_pass    <= sort_pass + 6'd1;
						sort_j       <= 6'd0;
						sort_swapped <= 1'b0;
					end
				end else begin
					fsm <= M_SORT_SWAP;
				end
			end
			// Ciclo B: confronto e scambio, su valori gia' registrati.
			M_SORT_SWAP: begin : sort_swap_blk
				reg do_swap;
				do_swap = (cop_sort_param == 16'd1) ? (sort_ka > sort_kb)
				        : (cop_sort_param == 16'd2) ? (sort_ka < sort_kb) : 1'b0;
				if (do_swap) begin
					sort_key[sort_j]        <= sort_kb;
					sort_key[sort_j + 6'd1] <= sort_ka;
					sort_val[sort_j]        <= sort_vb;
					sort_val[sort_j + 6'd1] <= sort_va;
					sort_swapped <= 1'b1;
				end
				sort_j <= sort_j + 6'd1;
				fsm <= M_SORT_CMP;
			end
			M_SORT_ST: begin
				dma_ram_we    <= 1'b1;                          // be = 2'b11 (word)
				dma_ram_addr  <= (sort_lookup_addr + {17'd0, sort_i, 1'b0}) >> 1;
				dma_ram_wdata <= sort_val[sort_i];
				if ((sort_i + 6'd1) >= sort_n) fsm <= S_IDLE;
				else                           sort_i <= sort_i + 6'd1;
			end

			default: fsm <= S_IDLE;
			endcase
		end
	end

endmodule

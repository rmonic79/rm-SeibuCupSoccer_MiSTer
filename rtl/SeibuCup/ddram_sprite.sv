// SPDX-License-Identifier: GPL-3.0-or-later
//
// Adapted for SeibuCupSoccer_MiSTer by Umberto Parisi (rmonic79), 2026.
// Derived from ddram_4port.sv (Sorgelig / BoogieWings) — original copyright
// and licence retained by its authors.
//
// ddram_sprite.sv — DDR3 reader/writer per sprite ROM SeibuCup.
// Derivato da ddram_4port.sv (Sorgelig / BoogieWings). 1 write port (download)
// + 1 read port 32-bit (sprite renderer). RAM DDR3 @ 0x30000000.
//
module ddram_sprite
(
	input         DDRAM_CLK,

	input         DDRAM_BUSY,
	output  [7:0] DDRAM_BURSTCNT,
	output [28:0] DDRAM_ADDR,
	input  [63:0] DDRAM_DOUT,
	input         DDRAM_DOUT_READY,
	output        DDRAM_RD,
	output [63:0] DDRAM_DIN,
	output  [7:0] DDRAM_BE,
	output        DDRAM_WE,

	// Write port (download sprite ROM)
	input  [27:0] wraddr,
	input  [15:0] din,
	input         we_req,
	output reg    we_ack = 0,

	// Read port 32-bit (sprite ROM fetch)
	input     [27:0] rdaddr,
	output reg [31:0] dout = 0,
	input            rd_req,
	output reg       rd_ack = 0
);

reg  [7:0] ram_burst;
reg [63:0] ram_q, next_q;
reg [63:0] ram_data;
reg [27:0] ram_address;
reg [27:0] cache_addr = '1;
reg        ram_read = 0;
reg        ram_write = 0;
reg  [7:0] ram_wr_be;

reg [1:0]  state = 0;

// Estrae 32 bit dalla qword 64 (sel = rdaddr[2]) e SWAPPA le due word16, per
// dare al renderer lo stesso ordine {hi_word, lo_word} big-endian che la SDRAM
// produceva (sdram_bridge: tile_data={tile_hi_word, sdram_dout0}). Senza lo
// swap gli sprite si mescolano (doc 06_ddr3_to_sdram §8/§12: byte order).
// Estrae 32 bit dalla qword 64 (sel=rdaddr[2]) e swappa le due word16 (= stato
// commit 303baf7 con sprite visibili). Doc 06 §8: ordine {hi_word, lo_word}.
function [31:0] extract32_swap(input [63:0] q, input sel);
	reg [31:0] half;
	begin
		half = sel ? q[63:32] : q[31:0];
		extract32_swap = {half[15:0], half[31:16]};
	end
endfunction

assign DDRAM_BURSTCNT = ram_burst;
assign DDRAM_BE       = ram_wr_be | {8{ram_read}};
assign DDRAM_ADDR     = {4'b0011, ram_address[27:3]}; // RAM at 0x30000000
assign DDRAM_RD       = ram_read;
assign DDRAM_DIN      = ram_data;
assign DDRAM_WE       = ram_write;

always @(posedge DDRAM_CLK) begin
	if(!DDRAM_BUSY) begin
		ram_write <= 0;
		ram_read  <= 0;

		case(state)
			0: if(we_ack != we_req) begin
					ram_data    <= {4{din}};
					ram_address <= wraddr;
					ram_write   <= 1;
					ram_burst   <= 1;
					ram_wr_be   <= (8'd3 << {wraddr[2:1],1'b0});
					state       <= 1;
				end
				else if(rd_req != rd_ack) begin
					if(cache_addr[27:3] == rdaddr[27:3]) begin
						rd_ack <= rd_req;
						dout   <= extract32_swap(ram_q, rdaddr[2]);
					end
					else if((cache_addr[27:3]+1'd1) == rdaddr[27:3]) begin
						rd_ack      <= rd_req;
						ram_q       <= next_q;
						dout        <= extract32_swap(next_q, rdaddr[2]);
						cache_addr  <= {rdaddr[27:3],3'b000};
						ram_address <= {rdaddr[27:3]+1'd1,3'b000};
						ram_read    <= 1;
						ram_burst   <= 1;
						state       <= 3;
					end
					else begin
						ram_address <= {rdaddr[27:3],3'b000};
						cache_addr  <= {rdaddr[27:3],3'b000};
						ram_read    <= 1;
						ram_burst   <= 2;
						state       <= 2;
					end
				end

			1: begin
					cache_addr      <= '1;
					cache_addr[3:0] <= 0;
					we_ack <= we_req;
					state  <= 0;
				end

			2: if(DDRAM_DOUT_READY) begin
					ram_q  <= DDRAM_DOUT;
					dout   <= extract32_swap(DDRAM_DOUT, rdaddr[2]);
					rd_ack <= rd_req;
					state  <= 3;
				end

			3: if(DDRAM_DOUT_READY) begin
					next_q <= DDRAM_DOUT;
					state  <= 0;
				end
		endcase
	end
end

endmodule

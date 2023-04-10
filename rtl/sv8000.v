
module SuperVision8000
(
	input         clk_sys,
	input         clk_3m58,
	input         reset,
	
	///////////// CPU RAM Interface /////////////
	output [15:0] cpu_ram_a_o,
	output reg    cpu_ram_ce_n_o,
	output reg    cpu_ram_we_n_o,
	input   [7:0] cpu_ram_d_i,
	output  [7:0] cpu_ram_d_o,

	//////////// Controller Interface /////////////
	input [10:0]  ps2_key,
	input [31:0]  joy0,
	input [31:0]  joy1,

	////////////// AUDIO Interface //////////////
	output [9:0] audio,

	////////////// VIDEO Interface //////////////
	output reg    HBlank,
	output reg    HSync,
	output reg    VBlank,
	output reg    VSync,
	output reg    ce_pix,
	output [12:0] vram_a,
	input   [7:0] vram_di,

	output [8:0] v_count,
	output [8:0] h_count,

	output [23:0] video
);


reg nMEM;
reg nRD;
reg nWR;
reg nIRQ;
reg nINT;
reg nNMI;
reg nWAIT;

reg [15:0] cpu_addr;
reg [7:0] data_to_cpu;
reg [7:0] data_from_cpu;

cpu_z80 Z80CPU(
	.CLK_4M(clk_snd),
	.nRESET(~reset),
	.SDA(cpu_addr),
	.SDD_IN(data_to_cpu),
	.SDD_OUT(data_from_cpu),
	.nIORQ(nIRQ),
	.nMREQ(nMEM),
	.nRD(nRD),
	.nWR(nWR),
	.nINT(nINT),
	.nNMI(nNMI),
	.nWAIT(nWAIT)
);

assign cpu_ram_we_n_o = (cpu_ram_a_o > 16'h7FFF && (~nMEM && ~nWR)) ? 1'b0 : 1'b1;
assign cpu_ram_ce_n_o = (~nMEM && !nRD) ? 1'b0 : 1'b1;
assign nWAIT = 1'b1;
assign nNMI = 1'b1;
assign nINT = ~VBlank;
assign cpu_ram_a_o = cpu_addr;
assign cpu_ram_d_o = data_from_cpu;
assign data_to_cpu = (cpu_addr[7:2] == 6'h20 && (~nRD && ~nIRQ)) ? ppi_dout : cpu_ram_d_i;

///////////////////////////SOUND///////////////////////////


reg       clk_snd;

always @(posedge clk_3m58) begin
	if(reset) clk_snd <= 1'b1;
	else clk_snd <= ~clk_snd;
end

ym2149 sound
(
	//Data BUS
	.I_DA(data_from_cpu),
	.O_DA(),
	.O_DA_OE_L(),
	//control
	.I_A9_L((cpu_ram_a_o[7:1] == 7'h60 && ~nWR && ~nIRQ) ? 1'b0 : 1'b1),
	.I_A8(1'b1),
	.I_BDIR((cpu_ram_a_o[7:1] == 7'h60 && ~nWR && ~nIRQ) ? 1'b1 : 1'b0),
	.I_BC2((cpu_ram_a_o[7:0] == 8'hC0 && ~nWR && ~nIRQ) ? 1'b1 : 1'b0),
	.I_BC1(1'b0),
	.I_SEL_L(1'b1),
	.O_AUDIO(audio),
	//port a
	.I_IOA(8'hFF),
	.O_IOA(port_a_o),
	.O_IOA_OE_L(),
	//port b
	.I_IOB(8'hFF),
	.O_IOB(),
	.O_IOB_OE_L(),

	.ENA(1'b1),
	.RESET_L(~reset),
	.CLK(clk_snd)
);

reg [7:0] port_a_o;

/////////////////////////// IO ///////////////////////////

reg [7:0] io_data;
reg [7:0] io_regs[8];
reg [7:0] FD_data;
reg       FD_buffer_flag;
reg [1:0] rd_sampler,wr_sampler;
reg [7:0] port_status;

always @(posedge clk_3m58) begin
	rd_sampler = {rd_sampler[0],nRD};
	wr_sampler = {wr_sampler[0],nWR};
end

//Keyboard
wire       pressed = ps2_key[9];
wire [8:0] code    = ps2_key[8:0];
//
//123     123
//QWE     456
//ASD     789
//ZXC     *0-

always @(posedge clk_3m58) begin
	reg old_state;
	old_state <= ps2_key[10];
	
	if(old_state != ps2_key[10]) begin
		casex(code[7:0])
//Left Controller
			'h16: btn_L1     <= ~pressed; // 1
			'h1E: btn_L2     <= ~pressed; // 2
			'h26: btn_L3     <= ~pressed; // 3
			'h15: btn_L4     <= ~pressed; // q
			'h1D: btn_L5     <= ~pressed; // w
			'h24: btn_L6     <= ~pressed; // e
			'h1C: btn_L7     <= ~pressed; // a
			'h1B: btn_L8     <= ~pressed; // s
			'h23: btn_L9     <= ~pressed; // d
			'h1A: btn_L_A    <= ~pressed; // z
			'h22: btn_L0     <= ~pressed; // x
			'h21: btn_L_H    <= ~pressed; // c
			'h25: btn_L4     <= ~pressed; // 4
			'h2E: btn_L5     <= ~pressed; // 5
			'h36: btn_L6     <= ~pressed; // 6
			'h3D: btn_L7     <= ~pressed; // 7
			'h3E: btn_L8     <= ~pressed; // 8
			'h46: btn_L9     <= ~pressed; // 9
			'h45: btn_L0     <= ~pressed; // 0
			'h4E: btn_L_A    <= ~pressed; // -
			'h55: btn_L_H    <= ~pressed; // =

//Right Controller - Num Pad
			'h69: btn_R1     <= ~pressed; // 1
			'h72: btn_R2     <= ~pressed; // 2
			'h7A: btn_R3     <= ~pressed; // 3
			'h6B: btn_R4     <= ~pressed; // 4
			'h73: btn_R5     <= ~pressed; // 5
			'h74: btn_R6     <= ~pressed; // 6
			'h6C: btn_R7     <= ~pressed; // 7
			'h75: btn_R8     <= ~pressed; // 8
			'h7D: btn_R9     <= ~pressed; // 9
			'h7C: btn_R_A    <= ~pressed; // *
			'h71: btn_R_A    <= ~pressed; // .
			'h70: btn_R0     <= ~pressed; // 0
			'h7B: btn_R_H    <= ~pressed; // -
			'h5A: btn_R_H    <= ~pressed; // Enter
		endcase
	end
end

wire [7:0] key_row[3];

assign key_row[0] = { btn_L1 & ~joy1[6] , btn_L4 & ~joy1[9]  , btn_L7 & ~joy1[12] , btn_L_A & ~joy1[4] & ~joy1[16] , btn_R1 & ~joy0[6] , btn_R4 & ~joy0[9]  , btn_R7 & ~joy0[12] , btn_R_A & ~joy0[4] & ~joy0[16] };
assign key_row[1] = { btn_L2 & ~joy1[7] , btn_L5 & ~joy1[10] , btn_L8 & ~joy1[13] , btn_L0 & ~joy1[15] ,             btn_R2 & ~joy0[7] , btn_R5 & ~joy0[10] , btn_R8 & ~joy0[13] , btn_R0 & ~joy0[15] };
assign key_row[2] = { btn_L3 & ~joy1[8] , btn_L6 & ~joy1[11] , btn_L9 & ~joy1[14] , btn_L_H & ~joy1[5] & ~joy1[17] , btn_R3 & ~joy0[8] , btn_R6 & ~joy0[11] , btn_R9 & ~joy0[14] , btn_R_H & ~joy0[5] & ~joy0[17] };


// Left Keypad
reg btn_L1 = 1;
reg btn_L2 = 1;
reg btn_L3 = 1;
reg btn_L4 = 1;
reg btn_L5 = 1;
reg btn_L6 = 1;
reg btn_L7 = 1;
reg btn_L8 = 1;
reg btn_L9 = 1;
reg btn_L0 = 1;
reg btn_L_A = 1; //*
reg btn_L_H = 1; //#


// Left Keypad
reg btn_R1 = 1;
reg btn_R2 = 1;
reg btn_R3 = 1;
reg btn_R4 = 1;
reg btn_R5 = 1;
reg btn_R6 = 1;
reg btn_R7 = 1;
reg btn_R8 = 1;
reg btn_R9 = 1;
reg btn_R0 = 1;
reg btn_R_A = 1; //*
reg btn_R_H = 1; //#


////////////////////////////PIA////////////////////////////

reg [7:0] ppi_dout, key_column;
reg [7:0] keydata;
reg [7:0] joydata;

assign keydata = 8'hFF & (key_column[0] ? 8'hFF : key_row[0]) & (key_column[1] ? 8'hFF : key_row[1]) & (key_column[2] ? 8'hFF : key_row[2]);
assign joydata = ~{ joy1[0] , joy1[1] , joy1[2] , joy1[3] , joy0[0] , joy0[1] , joy0[2] , joy0[3] };
i8255 PPI
(
	.reset(reset),
	.clk_sys(clk_snd),

	.addr(cpu_addr[1:0]),
	.idata(data_from_cpu),
	.odata(ppi_dout),
	.cs(cpu_addr[7:2] == 6'h20 ? 1'b1 : 1'b0),
	.we((~nWR && ~nIRQ)),
	.oe((~nRD && ~nIRQ)),

	.ipa(joydata), 
	.opa(),
	.ipb(keydata),
	.opb(),
	.ipc(8'hFF), 
	.opc(key_column)
);
///////////////////////////VIDEO///////////////////////////
reg [7:0] R,G,B;
reg [2:0] vdg_gm;
reg       vdg_hs_n, vdg_fs_n, vdg_an_g, vdg_an_s, vdg_int_ext, vdg_css, vdg_inv, vdg_blk_bgd;
reg [12:0] v_addr;

assign vdg_gm = {port_a_o[6], port_a_o[3], port_a_o[3]};
assign vdg_an_g = port_a_o[4];
assign vdg_css = port_a_o[4];
assign vdg_blk_bgd = ~port_a_o[1];
assign vdg_inv = vram_di[7];
assign vram_a = vdg_an_g ? (vdg_gm[2] ? ((v_addr & 13'h1FC0) >> 1) | (v_addr & 13'h001f) : v_addr) : v_addr;

mc6847 vdg
(
	.clk               (clk_sys),
	.clk_ena           (clk_vdg_en),
	.reset             (reset),
	.da0               (),
	.videoaddr         (v_addr),
	.dd                ((vdg_an_g == 1'b0 && vram_di == 8'h00) ? 8'h20 : vram_di),
	.hs_n              (vdg_hs_n),
	.fs_n              (vdg_fs_n),
	.an_g              (vdg_an_g),
	.an_s              (vdg_an_s),
	.intn_ext          (vdg_int_ext),
	.gm                (vdg_gm),
	.css               (vdg_css),
	.inv               (vdg_inv),
	.red               (R),
	.green             (G),
	.blue              (B),
	.hsync             (HSync),
	.vsync             (VSync),
	.artifact_en       (1'b0),
	.hblank            (HBlank),
	.vblank            (VBlank),
	.cvbs              (),
	.black_backgnd     (vdg_blk_bgd),
	.char_a            (chrrom_addr),
	.char_d_o          (chrrom_dout),
	.o_v_count         (v_count),
	.o_h_count         (h_count),

	.pixel_clock       (ce_pix)
);

reg clk_vdg_en ;
reg [1:0] div;


always @(posedge clk_sys) begin
	if (reset) div <= 0;
	else	begin
		clk_vdg_en <= 0;
		if (div == 2'd2) begin
		  clk_vdg_en <= 1;
        div <= 0;
		end
		else div <= div + 1'b1;
	end
end

assign video = {R,G,B};


reg   [7:0] chrrom[1536];
initial begin
	$readmemh("../CharRom.hex", chrrom);
end

wire   [7:0] chrrom_dout;
wire [10:0] chrrom_addr;
assign chrrom_dout = chrrom[chrrom_addr];

endmodule

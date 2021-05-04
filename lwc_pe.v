`include "mor1kx-defines.v"
`define OPTION_OPERAND_WIDTH 32

module lwc_pe(
	input clk,
	input rst,
	input padv_decode_i,
	input padv_execute_i,
	input padv_ctrl_i,
	input pipeline_flush_i,
	input [`OR1K_ALU_OPC_WIDTH-1:0] opc_alu_i,
	input [`OR1K_ALU_OPC_WIDTH-1:0] opc_alu_secondary_i,
	input [`OR1K_IMM_WIDTH-1:0] imm16_i,
	input [`OPTION_OPERAND_WIDTH-1:0] immediate_i,
	input immediate_sel_i,
	input [`OPTION_OPERAND_WIDTH-1:0] decode_immediate_i,
	input decode_immediate_sel_i,
	input decode_valid_i,
	input decode_op_mul_i,
	input op_alu_i,
	input op_add_i,
	input op_mul_i,
	input op_mul_signed_i,
	input op_mul_unsigned_i,
	input op_div_i,
	input op_div_signed_i,
	input op_div_unsigned_i,
	input op_shift_i,
	input op_ffl1_i,
	input op_setflag_i,
	input op_mtspr_i,
	input op_mfspr_i,
	input op_movhi_i,
	input op_ext_i,
	input [`OR1K_FPUOP_WIDTH-1:0] op_fpu_i,
	input [`OR1K_FPCSR_RM_SIZE-1:0] fpu_round_mode_i,
	input op_jbr_i,
	input op_jr_i,
	input [9:0] immjbr_upper_i,
	input [`OPTION_OPERAND_WIDTH-1:0] pc_execute_i,
	input adder_do_sub_i,
	input adder_do_carry_i,
	input [`OPTION_OPERAND_WIDTH-1:0] decode_rfa_i,
	input [`OPTION_OPERAND_WIDTH-1:0] decode_rfb_i,
	input [`OPTION_OPERAND_WIDTH-1:0] rfa_i,
	input [`OPTION_OPERAND_WIDTH-1:0] rfb_i,
	input flag_i,
	output flag_set_o,
	output flag_clear_o,
	input carry_i,
	output carry_set_o,
	output carry_clear_o,
	output overflow_set_o,
	output overflow_clear_o,
	output [`OR1K_FPCSR_WIDTH-1:0] fpcsr_o,
	output fpcsr_set_o,
	output [`OPTION_OPERAND_WIDTH-1:0] alu_result_o,
	output alu_valid_o,
	output [`OPTION_OPERAND_WIDTH-1:0] mul_result_o,
	output [`OPTION_OPERAND_WIDTH-1:0] adder_result_o,
	input  [6:0] shifta,
	input  [6:0] shiftb,
	input  rot,
	input  bypass,
	input  xor_out,
	input  store
);

reg [`OPTION_OPERAND_WIDTH-1:0] rfa_int, rfb_int;
wire [`OPTION_OPERAND_WIDTH-1:0] rfa_int_w, rfb_int_w;
wire [`OPTION_OPERAND_WIDTH-1:0] alu_result_int;
reg [`OPTION_OPERAND_WIDTH-1:0] aluReg;

//TODO piptline this whole thing
assign rfa_int_w = (shifta == 7'h1)?{rfa_i[30:0],rfa_i[31]}:((shifta == 7'h2)?{rfa_i[29:0],rfa_i[31:30]}:(shifta == 7'h3)?{rfa_i[28:0],rfa_i[31:29]}:(shifta == 7'h4)?{rfa_i[27:0],rfa_i[31:28]}:(shifta == 7'h5)?{rfa_i[26:0],rfa_i[31:27]}:(shifta == 7'h6)?{rfa_i[25:0],rfa_i[31:26]}:(shifta == 7'h7)?{rfa_i[24:0],rfa_i[31:25]}:(shifta == 7'h8)?{rfa_i[23:0],rfa_i[31:24]}:rfa_i);

always @(posedge clk or posedge rst) begin
  if (rst)      rfa_int <= 0;
  else          rfa_int <= rfa_int_w;
end

assign rfb_int_w = (shiftb == 7'h1)?{rfb_i[30:0],rfb_i[31]}:((shiftb == 7'h2)?{rfb_i[29:0],rfb_i[31:30]}:(shiftb == 7'h3)?{rfb_i[28:0],rfb_i[31:29]}:(shiftb == 7'h4)?{rfb_i[27:0],rfb_i[31:28]}:(shiftb == 7'h5)?{rfb_i[26:0],rfb_i[31:27]}:(shiftb == 7'h6)?{rfb_i[25:0],rfb_i[31:26]}:(shiftb == 7'h7)?{rfb_i[24:0],rfb_i[31:25]}:(shiftb == 7'h8)?{rfb_i[23:0],rfb_i[31:24]}:rfb_i);

always @(posedge clk or posedge rst) begin
  if (rst)      rfb_int <= 0;
  else          rfb_int <= rfb_int_w;
end
mor1kx_execute_alu #() mor1kx_execute_alu(
	clk,
	rst,
	padv_decode_i,
	padv_execute_i,
	padv_ctrl_i,
	pipeline_flush_i,
	opc_alu_i,
	opc_alu_secondary_i,
	imm16_i,
	immediate_i,
	immediate_sel_i,
	decode_immediate_i,
	decode_immediate_sel_i,
	decode_valid_i,
	decode_op_mul_i,
	op_alu_i,
	op_add_i,
	op_mul_i,
	op_mul_signed_i,
	op_mul_unsigned_i,
	op_div_i,
	op_div_signed_i,
	op_div_unsigned_i,
	op_shift_i,
	op_ffl1_i,
	op_setflag_i,
	op_mtspr_i,
	op_mfspr_i,
	op_movhi_i,
	op_ext_i,
	op_fpu_i,
	fpu_round_mode_i,
	op_jbr_i,
	op_jr_i,
	immjbr_upper_i,
	pc_execute_i,
	adder_do_sub_i,
	adder_do_carry_i,
	decode_rfa_i,
	decode_rfb_i,
	rfa_int,
	rfb_int,
	flag_i,
	flag_set_o,
	flag_clear_o,
	carry_i,
	carry_set_o,
	carry_clear_o,
	overflow_set_o,
	overflow_clear_o,
	fpcsr_o,
	fpcsr_set_o,
	alu_result_int,
	alu_valid_o,
	mul_result_o,
	adder_result_o
);

always @(posedge clk or posedge rst) begin
  if (rst)        aluReg <= 0;
  else if (store) aluReg <= alu_result_int;
end
assign alu_result_o = bypass? alu_result_int:alu_result_int ^ aluReg;
endmodule

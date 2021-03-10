`timescale 1ns/10ps
parameter delay = 0.05;

module PipelinedCPU(clk,reset);
	input logic clk,reset;
//	
	wire [63:0] Addr;
	wire [31:0] Instruction;
//
	
	wire [3:0] flagALU;
	wire [3:0] flagReg;
	wire [2:0] ALUOP;

	wire [8:0] DAddr9;
	wire [25:0] BrAddr26;
	wire [18:0] CondAddr19;
	wire [5:0] SHAMT;
	
	wire [3:0] xfer_size;
	wire [11:0] ALU_Imm12;
	wire [15:0] Imm16;

	wire [1:0] movShamt;
	wire [4:0] Rd,Rn,Rm;
	
	wire UncondBr,BrTaken,MemToReg,ALUSrc,Reg2Loc,SetFlags,RegWrite,ALU_Interm,MemWrite,Byte,movk,movz,CBZ,SetFlags_pipe2;
		
	instructmem Instruction_Memory(Addr,Instruction,clk);
	
	controlDataPath controlLogic(Instruction,UncondBr,BrTaken,MemToReg,ALUSrc,RegWrite,Reg2Loc,SetFlags,ALUOP,xfer_size,ALU_Interm,MemWrite,Rd,Rn,Rm,flagReg,flagALU,Byte,movk,movz,CBZ,SetFlags_pipe2,BLT);
	
	
	assign movShamt = Instruction[22:21];//2bit
	assign ALU_Imm12 = Instruction[21:10];//12bit
	assign DAddr9 = Instruction[20:12];//9 bit
	assign BrAddr26 = Instruction[25:0];//26 bit
	assign CondAddr19 = Instruction[23:5];//19 bit
	assign SHAMT = Instruction[15:10];//6 bit
	assign Imm16 = Instruction[20:5];//16 bit
	
	
	wire [4:0] Rd_pipe1,Rn_pipe1,Rm_pipe1;
	wire [8:0] DAddr9_pipe1;
	wire [25:0] BrAddr26_pipe1;
	wire [18:0] CondAddr19_pipe1;
	wire [2:0] ALUOP_pipe1;
	wire [3:0] xfer_size_pipe1;
	wire [11:0] ALU_Imm12_pipe1;
	wire [15:0] Imm16_pipe1;
	
	
	wire [1:0] movShamt_pipe1;
	
			

	wire UncondBr_pipe1,BrTaken_pipe1,MemToReg_pipe1,ALUSrc_pipe1,Reg2Loc_pipe1,SetFlags_pipe1,RegWrite_pipe1,ALU_Interm_pipe1,MemWrite_pipe1,Byte_pipe1,movk_pipe1,movz_pipe1,CBZ_pipe1,BLT_pipe1;
	
	
	wire [31:0] Instruction_pipe1,Instruction_pipe2,Instruction_pipe3;
	Register32b Instruction_pipe1_reg(Instruction,Instruction_pipe1,1'b1,reset,clk);
	Register32b Instruction_pipe2_reg(Instruction_pipe1,Instruction_pipe2,1'b1,reset,clk);
	Register32b Instruction_pipe3_reg(Instruction_pipe2,Instruction_pipe3,1'b1,reset,clk);
	
	
	
	
	Register2b movShamt_pipe1_reg(movShamt,movShamt_pipe1,1'b1,reset,clk);
	
	Register4b xfer_size_pipe1_reg(xfer_size,xfer_size_pipe1,1'b1,reset,clk);
	
	Register3b ALUOP_pipe1_reg(ALUOP,ALUOP_pipe1,1'b1,reset,clk);
	
	Register12b ALU_Imm12_pipe1_reg(ALU_Imm12,ALU_Imm12_pipe1,1'b1,reset,clk);
	
	Register9b DAddr9_pipe1_reg(DAddr9,DAddr9_pipe1,1'b1,reset,clk);
	
	Register26b BrAddr26_pipe1_reg(BrAddr26,BrAddr26_pipe1,1'b1,reset,clk);
	
	Register19b CondAddr19_pipe1_reg(CondAddr19,CondAddr19_pipe1,1'b1,reset,clk);
	
	Register16b Imm16_pipe1_reg(Imm16,Imm16_pipe1,1'b1,reset,clk);
	
	Register5b RD_pipe1_reg(Rd,Rd_pipe1,1'b1,reset,clk);
	
	Register5b RM_pipe1_reg(Rm,Rm_pipe1,1'b1,reset,clk);
	
	Register5b RN_pipe1_reg(Rn,Rn_pipe1,1'b1,reset,clk);
	
	
	
	
	register BLT_pipe1_reg(BLT, BLT_pipe1,1'b1,reset,clk);
	
	register CBZ_pipe1_reg(CBZ, CBZ_pipe1,1'b1,reset,clk);
	
	register UncondBr_pipe1_reg(UncondBr, UncondBr_pipe1,1'b1,reset,clk);
	
	register BrTaken_pipe1_reg(BrTaken, BrTaken_pipe1,1'b1,reset,clk);
	
	register MemToReg_pipe1_reg(MemToReg, MemToReg_pipe1,1'b1,reset,clk);
	
	register ALUSrc_pipe1_reg(ALUSrc, ALUSrc_pipe1,1'b1,reset,clk);
	
	register Reg2Loc_pipe1_reg(Reg2Loc, Reg2Loc_pipe1,1'b1,reset,clk);
	
	register SetFlags_pipe1_reg(SetFlags, SetFlags_pipe1,1'b1,reset,clk);
	
	register RegWrite_pipe1_reg(RegWrite, RegWrite_pipe1,1'b1,reset,clk);
	
	register ALU_Interm_pipe1_reg(ALU_Interm, ALU_Interm_pipe1,1'b1,reset,clk);
	
	register MemWrite_pipe1_reg(MemWrite, MemWrite_pipe1,1'b1,reset,clk);
	
	register Byte_pipe1_reg(Byte, Byte_pipe1,1'b1,reset,clk);
	
	register movk_pipe1_reg(movk, movk_pipe1,1'b1,reset,clk);
	
	register movz_pipe1_reg(movz, movz_pipe1,1'b1,reset,clk);

	//PIPELINE STAGE 1


//
//	// flag 0 -> Zero Flag
//	// flag 1 -> Neg
//	// flag 2 -> Overflow
//	// flag 3 -> Carry


		wire [63:0] ReadData1,ReadData2,ALUout,ReadData2_pipe2,ReadData2_pipe3,MemToRegMuxOut;
		wire [4:0] readRegB;


		
		
		
		//Creating Stage 5 Wires	
		wire RegWrite_pipe4;
		
		wire [4:0] Rd_pipe4;

		wire [63:0] MemToRegister_pipe4;
		
		//wire Rm_forward,Rn_forward,Rm_mem_forward,Rn_mem_forward;
		wire [4:0] Rd_pipe2;
		
		
		wire [4:0] Rd_pipe3;
		wire [63:0] ALUout_pipe3,MemToRegister_pipe3,data2_forward;
		
		wire held_reset1,held_reset2,held_reset3;
		D_FF_special resethold1(held_reset1, reset, 1'b1, clk);
		D_FF_special resethold2(held_reset2, held_reset1, 1'b1, clk);
		D_FF_special resethold3(held_reset3, held_reset2, 1'b1, clk);
		
		
		
//		wire CBZ_isZero,branch,CBZPick,fwd_pipe2,fwd_pipe3;
//		isZero CBZ_zero_Check(data2_forward,CBZ_isZero);
//		and#delay CBZsel(CBZPick,CBZ_isZero,CBZ_pipe1);
//		wire conBRanch_mux_out,LT,BLT_Branch;
//		
//		
//		xor#delay lessthan(LT,flagReg[1],flagReg[2]);
//		and#delay BLT_check(BLT_Branch,LT,BrTaken_pipe1);
//		
//		mux2_1 CondBranchTaken_Mux(conBRanch_mux_out,1'b0,1'b1,BLT_Branch);
//		
//		mux2_1 CBZ_taken(branch,conBRanch_mux_out, CBZ_pipe1, CBZPick);
		

		//wire [63:0] data2_sel;
		
		
		
		wire CBZ_isZero,branch,CBZPick,fwd_pipe2,fwd_pipe3;
		
		isZero CBZ_zero_Check(data2_forward,CBZ_isZero);
		and#delay CBZsel(CBZPick,CBZ_isZero,CBZ_pipe1);
		wire conBRanch_mux_out,LT,BLT_Branch;
		
		
		xor#delay lessthan(LT,flagReg[1],flagReg[2]);
		and#delay BLT_decide(BLT_Branch,LT,BLT_pipe1);
		
		mux2_1 CondBranchTaken_Mux(conBRanch_mux_out,BLT_Branch,CBZPick,CBZ_pipe1);
		
		wire conBranch,Btaken_CBZ_LT;
		or# delay(conBranch,BLT_pipe1,CBZ_pipe1);
		
		//mux2_1 Braken_mux(Btaken_CBZ_LT,BrTaken_pipe1,conBRanch_mux_out,conBranch);
		
		
		mux2_1 CBZ_taken(branch,BrTaken_pipe1, conBRanch_mux_out, conBranch);
		

		
		
		
		ProgramCounterDataPath CPUProgramCounter(UncondBr_pipe1,branch,Addr,CondAddr19_pipe1,BrAddr26_pipe1,reset,clk);
		
		wire MemWrite_pipe2,MemWrite_pipe3,mem_forward_pipe2,mem_forward_pipe3,forwardData;
		
		logic NOTclk;
		not#delay clkinvert(NOTclk,clk);

		mux2_5 Reg2LocMux(readRegB,Rd_pipe1, Rm_pipe1, Reg2Loc_pipe1);
		regfile registers(ReadData1,ReadData2,MemToRegister_pipe4, Rn_pipe1, readRegB, Rd_pipe4, RegWrite_pipe4, NOTclk,reset);

		//forwarding_control forwardingLogic(Rm_pipe1,Rn_pipe1,Rd_pipe2,Rd_pipe3,Rn_forward,Rm_forward,Rn_mem_forward,Rm_mem_forward,MemWrite_pipe2,MemWrite_pipe3,Instruction_pipe2,Instruction_pipe3);	
		
		//mem_forwarding_control forwardMEM(Rd_pipe1,Rd_pipe2,Rd_pipe3,Reg2Loc_pipe1,mem_forward_pipe2,mem_forward_pipe3,forwardData,held_reset3,Instruction_pipe2,Instruction_pipe3);
		
		//movzk_forwarding_control Movforward(Rd_pipe1,Rd_pipe2,Rd_pipe3,movz_pipe1,movk_pipe1,Instruction_pipe2,Instruction_pipe3,fwd_pipe2,fwd_pipe3);
		
		wire [63:0] Reg_forward,data1_forward,data2_forward_mem,mem_forward_data,Rn_mux,Rm_mux,Rd_mux,RD_Pipe2_forwar,MemToRegister;
		
		
		wire Rd_pipe2_to_Rd_Pipe1,Rd_pipe3_to_Rd_Pipe1,Rd_pipe2_to_Rm_Pipe1,Rd_pipe3_to_Rm_Pipe1,Rd_pipe2_to_Rn_Pipe1,Rd_pipe3_to_Rn_Pipe1,Rd_forward,store;
		
		global_forward forwardingLogic(Rd_pipe1,Rd_pipe2,Rd_pipe3,Rn_pipe1,Rm_pipe1,held_reset2,Instruction_pipe1,Instruction_pipe2,Instruction_pipe3,Rd_pipe2_to_Rd_Pipe1,Rd_pipe3_to_Rd_Pipe1,Rd_pipe2_to_Rm_Pipe1,Rd_pipe3_to_Rm_Pipe1,Rd_pipe2_to_Rn_Pipe1,Rd_pipe3_to_Rn_Pipe1,Rd_forward,store);
		
		//RN //data1_forward
		
		
		wire Rd_forward_enable,Rn_forward_enable,Rm_forward_enable;
		
		or#delay Rd_toggle_forward(Rd_forward_enable,Rd_pipe2_to_Rd_Pipe1, Rd_pipe3_to_Rd_Pipe1);
		or#delay Rn_toggle_forward(Rn_forward_enable,Rd_pipe2_to_Rn_Pipe1,Rd_pipe3_to_Rn_Pipe1);
		or#delay Rm_toggle_forward(Rm_forward_enable,Rd_pipe2_to_Rm_Pipe1,Rd_pipe3_to_Rm_Pipe1);
		
		
		mux2_64 Rn_Sel_RD_mux(Rn_mux,MemToRegister,MemToRegMuxOut,Rd_pipe3_to_Rn_Pipe1);
		mux2_64 RN_forward_mux(data1_forward,ReadData1,Rn_mux,Rn_forward_enable);
		
		
		
		//RM data2_forward
		
		wire [63:0] Rm_forward_data,Rd_forward_data,Rd_muxed_forward,store_mux;
		
		
		
		mux2_64 Rm_Sel_RD_mux(Rm_mux,MemToRegister,MemToRegMuxOut,Rd_pipe3_to_Rm_Pipe1);
		mux2_64 Rm_forward_mux(Rm_forward_data,ReadData2,Rm_mux,Rm_forward_enable);
		
		
		
		//RD data2_forward
		mux2_64 Rd_Sel_RD_mux(Rd_mux,MemToRegister,MemToRegMuxOut,Rd_pipe3_to_Rd_Pipe1);
		mux2_64 Rd_store_case(Rd_muxed_forward,ReadData2,Rd_mux,Rd_forward_enable);
		
		//mux2_64 storeMux(store_mux,ReadData2_pipe2,ReadData2_pipe3,store);
		
		mux2_64 Rd_forward_mux(Rd_forward_data,Rd_muxed_forward,MemToRegMuxOut,store);
		
		//data2_forward Sel
		
		
		
		mux2_64 Data2_forward_mux(data2_forward,Rm_forward_data,Rd_forward_data,Rd_forward);
		
		
//		mux2_64 RN_pick_pipe2(Rn_mux,ALUout,MemToRegMuxOut,Rn_mem_forward);
//		mux2_64 RM_pick_pipe2(Rm_mux,ALUout,MemToRegMuxOut,Rm_mem_forward);
//		
//		
//		
//		mux2_64 RN_forwardmux_pipe2(data1_forward,ReadData1,Rn_mux,Rn_forward);//replaces ALUout with Rn/Rm_forward
//		mux2_64 RM_forwardmux_pipe2(Reg_forward,ReadData2,Rm_mux,Rm_forward);
//		
//		
//		
//		
//		mux2_64 MEM_forward_sel(data2_forward_mem,ALUout,MemToRegMuxOut,mem_forward_pipe3);
//		
//		mux2_64 RD_Pipe2_Forward(data2_forward,Reg_forward,data2_forward_mem,forwardData);
//		
//		mux2_64 RD_FORWARD(data2_forward,Reg_forward,data2_forward_mem,forwardData);



//		mux2_64 RN_pick_pipe2(Rn_mux,MemToRegister,MemToRegMuxOut,Rn_mem_forward);
//		mux2_64 RM_pick_pipe2(Rm_mux,MemToRegister,MemToRegMuxOut,Rm_mem_forward);
//		
//		
//		
//		mux2_64 RN_forwardmux_pipe2(data1_forward,ReadData1,Rn_mux,Rn_forward);//replaces ALUout with Rn/Rm_forward
//		mux2_64 RM_forwardmux_pipe2(Reg_forward,ReadData2,Rm_mux,Rm_forward);
//		
//		
//		
//		
//		mux2_64 MEM_forward_sel(data2_forward_mem,MemToRegister,MemToRegMuxOut,mem_forward_pipe3);
//		
//		mux2_64 RD_Pipe2_Forward(data2_forward,Reg_forward,data2_forward_mem,forwardData);
//		
//		
//		wire [63:0] forwarded_mov;
//		mux2_64 mov_forward_mux(forwarded_mov,MemToRegister,ALUout_pipe3,fwd_pipe3);
//		
//		wire mov_sel;
//		or#delay mov_op(mov_sel,fwd_pipe3,fwd_pipe2);
//		
//		mux2_64 sel_mux(data2_sel,data2_forward,forwarded_mov,mov_sel);



		
		
		
		
		
		
		
		
		wire UncondBr_pipe2,BrTaken_pipe2,MemToReg_pipe2,ALUSrc_pipe2,RegWrite_pipe2,ALU_Interm_pipe2,Byte_pipe2,movk_pipe2,movz_pipe2;
	
		wire [8:0] DAddr9_pipe2;
		wire [25:0] BrAddr26_pipe2;
		wire [18:0] CondAddr19_pipe2;
		wire [2:0] ALUOP_pipe2;
		wire [3:0] xfer_size_pipe2;
		wire [11:0] ALU_Imm12_pipe2;
		wire [15:0] Imm16_pipe2;
		
		
		
		wire [1:0] movShamt_pipe2;
		wire [63:0] ReadData1_pipe2;
		
		
		Register5b RD_pipe2_reg(Rd_pipe1,Rd_pipe2,1'b1,reset,clk);
		
		Register4b xfer_size_pipe2_reg(xfer_size_pipe1,xfer_size_pipe2,1'b1,reset,clk);
		
		Register3b ALUOP_pipe2_reg(ALUOP_pipe1,ALUOP_pipe2,1'b1,reset,clk);
		
		Register2b movShamt_pipe2_reg(movShamt_pipe1,movShamt_pipe2,1'b1,reset,clk);
		
		Register12b ALU_Imm12_pipe2_reg(ALU_Imm12_pipe1,ALU_Imm12_pipe2,1'b1,reset,clk);
		
		Register9b DAddr9_pipe2_reg(DAddr9_pipe1,DAddr9_pipe2,1'b1,reset,clk);
		
		Register26b BrAddr26_pipe2_reg(BrAddr26_pipe1,BrAddr26_pipe2,1'b1,reset,clk);
		
		Register19b CondAddr19_pipe2_reg(CondAddr19_pipe1,CondAddr19_pipe2,1'b1,reset,clk);
		
		Register16b Imm16_pipe2_reg(Imm16_pipe1,Imm16_pipe2,1'b1,reset,clk);
		
		Register64b ReadData1_pipe2_reg(data1_forward,ReadData1_pipe2,1'b1,reset,clk);
		
		//Register64b ReadData2_pipe2_reg(data2_sel,ReadData2_pipe2,1'b1,reset,clk);
		Register64b ReadData2_pipe2_reg(data2_forward,ReadData2_pipe2,1'b1,reset,clk);
		
		
		register UncondBr_pipe2_reg(UncondBr_pipe1, UncondBr_pipe2,1'b1,reset,clk);
		
		register BrTaken_pipe2_reg(BrTaken_pipe1, BrTaken_pipe2,1'b1,reset,clk);
		
		register MemToReg_pipe2_reg(MemToReg_pipe1, MemToReg_pipe2,1'b1,reset,clk);
		
		register ALUSrc_pipe2_reg(ALUSrc_pipe1, ALUSrc_pipe2,1'b1,reset,clk);
		
		register SetFlags_pipe2_reg(SetFlags_pipe1, SetFlags_pipe2,1'b1,reset,clk);
		
		register RegWrite_pipe2_reg(RegWrite_pipe1, RegWrite_pipe2,1'b1,reset,clk);
		
		register ALU_Interm_pipe2_reg(ALU_Interm_pipe1, ALU_Interm_pipe2,1'b1,reset,clk);
		
		register MemWrite_pipe2_reg(MemWrite_pipe1, MemWrite_pipe2,1'b1,reset,clk);
		
		register Byte_pipe2_reg(Byte_pipe1, Byte_pipe2,1'b1,reset,clk);
		
		register movk_pipe2_reg(movk_pipe1, movk_pipe2,1'b1,reset,clk);
		
		register movz_pipe2_reg(movz_pipe1, movz_pipe2,1'b1,reset,clk);
		

		
		
		//PIPELINE STAGE 2
		
		wire [63:0] Mem,movkOut,movZout,movKmuxOut,inputB,MemoryOut,extendedDAddr9,extendedALU_Imm12,ALUConstant;
		
		SignExtendImm9 extendDaddr9(DAddr9_pipe2,extendedDAddr9);
		
		ZeroExtend12 zeroExtender(ALU_Imm12_pipe2, extendedALU_Imm12);
		
		mux2_64 ADDI(ALUConstant,extendedDAddr9,extendedALU_Imm12,ALU_Interm_pipe2);
		
		mux2_64 ALUSrcMux(inputB,ReadData2_pipe2,ALUConstant,ALUSrc_pipe2);
		
		alu_64b ALU(ReadData1_pipe2,inputB,ALUOP_pipe2,ALUout,flagALU[1], flagALU[0], flagALU[2], flagALU[3]);
		
		//flag registers
		register ZeroFlagReg(flagALU[0], flagReg[0],SetFlags_pipe2,reset,NOTclk);
		register NegFlagReg(flagALU[1], flagReg[1],SetFlags_pipe2,reset,NOTclk);
		register OvrFlowFlagReg(flagALU[2], flagReg[2],SetFlags_pipe2,reset,NOTclk);
		register CarryFlagReg(flagALU[3], flagReg[3],SetFlags_pipe2,reset,NOTclk);
		
		
//		movK movKControl(ReadData2_pipe2,Imm16_pipe2,movShamt_pipe2,movkOut);
//		movZ movZcontrol(Imm16_pipe2,movShamt_pipe2,movZout);
//	
//
//		mux2_64 movkMux(movKmuxOut,MemToRegMuxOut,movkOut,movk_pipe2);
//		mux2_64 movzMux(MemToRegister,movKmuxOut,movZout,movz_pipe2);//Broken Might need to move stages
//		
		
		

		movK movKControl(ReadData2_pipe2,Imm16_pipe2,movShamt_pipe2,movkOut);
		movZ movZcontrol(Imm16_pipe2,movShamt_pipe2,movZout);
	

		mux2_64 movkMux(movKmuxOut,ALUout,movkOut,movk_pipe2);
		mux2_64 movzMux(MemToRegister,movKmuxOut,movZout,movz_pipe2);//Broken Might need to move stages
		
		
		wire MemToReg_pipe3,RegWrite_pipe3,Byte_pipe3;
	

		
		
		wire [3:0] xfer_size_pipe3;
		wire [63:0] ReadData1_pipe3;
		

		
		//Register64b ReadData1_pipe3_reg(ReadData1_pipe2,ReadData1_pipe3,1'b1,reset,clk);
		
		
		Register64b ReadData1_pipe3_reg(ReadData1_pipe2,ReadData1_pipe3,1'b1,reset,clk);
		Register64b ReadData2_pipe3_reg(ReadData2_pipe2,ReadData2_pipe3,1'b1,reset,clk);
		
		Register64b ALUout_pipe3_reg(MemToRegister,ALUout_pipe3,1'b1,reset,clk);
		
		//Register64b MemToRegister_pipe3_reg(MemToRegister,MemToRegister_pipe3,1'b1,reset,clk);
		
		Register5b RD_pipe3_reg(Rd_pipe2,Rd_pipe3,1'b1,reset,clk);
		
		Register4b xfer_size_pipe3_reg(xfer_size_pipe2,xfer_size_pipe3,1'b1,reset,clk);
		
		register MemToReg_pipe3_reg(MemToReg_pipe2, MemToReg_pipe3,1'b1,reset,clk);
		
		register RegWrite_pipe3_reg(RegWrite_pipe2, RegWrite_pipe3,1'b1,reset,clk);
		
		register MemWrite_pipe3_reg(MemWrite_pipe2, MemWrite_pipe3,1'b1,reset,clk);
		
		register Byte_pipe3_reg(Byte_pipe2, Byte_pipe3,1'b1,reset,clk);

		//need Logic to forward Rd for stores MAYBE
		//Not sure
		
		
		//Pipeline Stage3

		
		datamem MainMemory(ALUout_pipe3,MemWrite_pipe3,MemToReg_pipe3,ReadData2_pipe3,clk,xfer_size_pipe3,Mem);
		
		
		ZeroExtend8 extendByte(Mem,Byte_pipe3,MemoryOut);
		mux2_64 MemToRegMux(MemToRegMuxOut,ALUout_pipe3,MemoryOut,MemToReg_pipe3);

		

		//Register64b MemToRegister_pipe4_reg(ALUout_pipe3,MemToRegister_pipe4,1'b1,reset,clk);
		Register64b MemToRegister_pipe4_reg(MemToRegMuxOut,MemToRegister_pipe4,1'b1,reset,clk);
		
		Register5b RD_pipe4_reg(Rd_pipe3,Rd_pipe4,1'b1,reset,clk);
		
		register RegWrite_pipe4_reg(RegWrite_pipe3, RegWrite_pipe4,1'b1,reset,clk);
		

		//Pipeline Stage4
		
		
		
		//Register storing Data 
		//Rd
		//
		//Pipeline Stage5
		
		

//		
//	//ALU DATA PATH END
	
endmodule
module PipelinedCPU_Testbench();
	logic reset,clk;
	PipelinedCPU dut(clk,reset);

	parameter clk_PERIOD = 1000;	
	initial begin	
		clk <= 0;	
		forever #(clk_PERIOD/2) clk <= ~clk;	
	end
	
	initial begin
		reset <= 1;@(posedge clk);
		@(posedge clk);
		@(posedge clk);
		@(posedge clk);
		reset <= 0;@(posedge clk);
		
		for(int i = 0 ; i < 1550; i++)begin
			@(posedge clk);
		end
		
		$stop;
	end

endmodule


//Ignore
//module movzk_forwarding_control(Rd_pipe1,Rd_pipe2,Rd_pipe3,movz,movk,Instruction_pipe2,Instruction_pipe3,fwd_pipe2,fwd_pipe3);
//		input logic movz,movk;
//		input logic [4:0] Rd_pipe1,Rd_pipe2,Rd_pipe3;
//		input logic [31:0] Instruction_pipe2,Instruction_pipe3;
//		output logic fwd_pipe2,fwd_pipe3;
//		
//		wire [8:0] MovType_pipe3;
//		wire [8:0] MovType_pipe2;
//		
//		
//		assign MovType_pipe2 = Instruction_pipe2[31:23];
//		assign MovType_pipe3 = Instruction_pipe3[31:23];
//		
//		always_comb begin
//		
//			fwd_pipe2 = 1'b0;
//			fwd_pipe3 = 1'b0;
//			if(movz | movk)begin
//				if((Rd_pipe1 == Rd_pipe2) & (Rd_pipe1 == Rd_pipe3))begin
//				
//					fwd_pipe2 = 1'b1;
//					fwd_pipe3 = 1'b0;
//				
//				end
//				
//				if((Rd_pipe1 != Rd_pipe2) & (Rd_pipe1 == Rd_pipe3))begin
//					fwd_pipe2 = 1'b0;
//					fwd_pipe3 = 1'b1;
//				
//				
//				end
//				
//				if((Rd_pipe1 == Rd_pipe2) & (Rd_pipe1 != Rd_pipe3))begin
//					fwd_pipe2 = 1'b1;
//					fwd_pipe3 = 1'b0;
//				
//				
//				
//				end
//				
//			
//			
//			
//			end
//		
//		
//		end
//		
//		
//
//
//endmodule


module global_forward(Rd_pipe1,Rd_pipe2,Rd_pipe3,Rn_pipe1,Rm_pipe1,held_reset3,Instruction_pipe1,Instruction_pipe2,Instruction_pipe3,Rd_pipe2_to_Rd_Pipe1,Rd_pipe3_to_Rd_Pipe1,Rd_pipe2_to_Rm_Pipe1,Rd_pipe3_to_Rm_Pipe1,Rd_pipe2_to_Rn_Pipe1,Rd_pipe3_to_Rn_Pipe1,Rd_forward,store);
	input logic held_reset3;
	input logic [4:0] Rd_pipe1,Rd_pipe2,Rd_pipe3,Rn_pipe1,Rm_pipe1;
	input logic [31:0] Instruction_pipe1,Instruction_pipe2,Instruction_pipe3;
	output logic Rd_pipe2_to_Rd_Pipe1,Rd_pipe3_to_Rd_Pipe1,Rd_pipe2_to_Rm_Pipe1,Rd_pipe3_to_Rm_Pipe1,Rd_pipe2_to_Rn_Pipe1,Rd_pipe3_to_Rn_Pipe1,Rd_forward,store;

	wire [5:0] BType_pipe1,BType_pipe2,BType_pipe3;
	wire [7:0] CBType_pipe1,CBType_pipe2,CBType_pipe3;
	wire [10:0] RType_pipe1,RType_pipe2,RType_pipe3;
	wire [9:0] IType_pipe1,IType_pipe2,IType_pipe3;
	wire [8:0] MovType_pipe1,MovType_pipe2,MovType_pipe3;
	
	
	

	
	assign BType_pipe1 = Instruction_pipe1[31:26];
	assign CBType_pipe1 = Instruction_pipe1[31:24];
	assign RType_pipe1 = Instruction_pipe1[31:21];
	assign IType_pipe1 = Instruction_pipe1[31:22];
	assign MovType_pipe1 = Instruction_pipe1[31:23];
	
	assign CBType_pipe2 = Instruction_pipe2[31:24];
	assign RType_pipe2 = Instruction_pipe2[31:21];
	
	assign CBType_pipe3 = Instruction_pipe3[31:24];
	assign RType_pipe3 = Instruction_pipe3[31:21];
	
	
	
	always_comb begin
		if(Instruction_pipe1 == 32'bx)begin
			
			Rd_pipe2_to_Rd_Pipe1 = 1'b0;
			Rd_pipe3_to_Rd_Pipe1 = 1'b0;
			
			Rd_pipe2_to_Rm_Pipe1 = 1'b0;
			Rd_pipe3_to_Rm_Pipe1 = 1'b0;
			
			Rd_pipe2_to_Rn_Pipe1 = 1'b0;
			Rd_pipe3_to_Rn_Pipe1 = 1'b0;
			store = 1'b0;
			
			
			Rd_forward = 1'b0;
		
		end
		
		
		if(Instruction_pipe1 == 32'b0)begin
			Rd_pipe2_to_Rd_Pipe1 = 1'b0;
			Rd_pipe3_to_Rd_Pipe1 = 1'b0;
			
			Rd_pipe2_to_Rm_Pipe1 = 1'b0;
			Rd_pipe3_to_Rm_Pipe1 = 1'b0;
			
			Rd_pipe2_to_Rn_Pipe1 = 1'b0;
			Rd_pipe3_to_Rn_Pipe1 = 1'b0;
			store = 1'b0;
			
			Rd_forward = 1'b0;
		
		end
		
		
		if((RType_pipe1 == 11'b11101011000) | (RType_pipe1 == 11'b10101011000) | (IType_pipe1 == 10'b1001000100)) begin //SUBS,Add, Addi
			Rd_pipe2_to_Rd_Pipe1 = 1'b0;
			Rd_pipe3_to_Rd_Pipe1 = 1'b0;
			Rd_pipe2_to_Rm_Pipe1 = 1'b0;
			Rd_pipe3_to_Rm_Pipe1 = 1'b0;
			
			Rd_pipe2_to_Rn_Pipe1 = 1'b0;
			Rd_pipe3_to_Rn_Pipe1 = 1'b0;
			store = 1'b0;
			
			Rd_forward = 1'b0;
			
			//RM
			if((Rm_pipe1 == Rd_pipe2) & (Rm_pipe1 == Rd_pipe3))begin
				Rd_pipe2_to_Rm_Pipe1 = 1'b1;
				Rd_pipe3_to_Rm_Pipe1 = 1'b0;
				
			end
			
			if((Rm_pipe1 != Rd_pipe2) & (Rm_pipe1 == Rd_pipe3))begin
				Rd_pipe2_to_Rm_Pipe1 = 1'b0;
				Rd_pipe3_to_Rm_Pipe1 = 1'b1;
				
			end
			
			if((Rm_pipe1 == Rd_pipe2) & (Rm_pipe1 != Rd_pipe3))begin
				Rd_pipe2_to_Rm_Pipe1 = 1'b1;
				Rd_pipe3_to_Rm_Pipe1 = 1'b0;
				
			end
			
			if((Rm_pipe1 != Rd_pipe2) & (Rm_pipe1 != Rd_pipe3))begin
				Rd_pipe2_to_Rm_Pipe1 = 1'b0;
				Rd_pipe3_to_Rm_Pipe1 = 1'b0;
				
			end
			
			//RN
			if((Rn_pipe1 == Rd_pipe2) & (Rn_pipe1 == Rd_pipe3))begin
				Rd_pipe2_to_Rn_Pipe1 = 1'b1;
				Rd_pipe3_to_Rn_Pipe1 = 1'b0;
				
			end
			
			if((Rn_pipe1 != Rd_pipe2) & (Rn_pipe1 == Rd_pipe3))begin
				Rd_pipe2_to_Rn_Pipe1 = 1'b0;
				Rd_pipe3_to_Rn_Pipe1 = 1'b1;
				
			end
			
			if((Rn_pipe1 == Rd_pipe2) & (Rn_pipe1 != Rd_pipe3))begin
				Rd_pipe2_to_Rn_Pipe1 = 1'b1;
				Rd_pipe3_to_Rn_Pipe1 = 1'b0;
				
			end
			
			if((Rn_pipe1 != Rd_pipe2) & (Rn_pipe1 != Rd_pipe3))begin
				Rd_pipe2_to_Rn_Pipe1 = 1'b0;
				Rd_pipe3_to_Rn_Pipe1 = 1'b0;
				
			end
			
			
		
		end


		
		if((RType_pipe1 == 11'b11111000010) | (RType_pipe1 == 11'b00111000010)) begin //LDUR LDURB
			Rd_pipe2_to_Rd_Pipe1 = 1'b0;
			Rd_pipe3_to_Rd_Pipe1 = 1'b0;
			Rd_pipe2_to_Rm_Pipe1 = 1'b0;
			Rd_pipe3_to_Rm_Pipe1 = 1'b0;
			
			Rd_pipe2_to_Rn_Pipe1 = 1'b0;
			Rd_pipe3_to_Rn_Pipe1 = 1'b0;
			store = 1'b0;
			Rd_forward = 1'b0;
			
			//RN
			if((Rn_pipe1 == Rd_pipe2) & (Rn_pipe1 == Rd_pipe3))begin
				Rd_pipe2_to_Rn_Pipe1 = 1'b1;
				Rd_pipe3_to_Rn_Pipe1 = 1'b0;
				
			end
			
			if((Rn_pipe1 != Rd_pipe2) & (Rn_pipe1 == Rd_pipe3))begin
				Rd_pipe2_to_Rn_Pipe1 = 1'b0;
				Rd_pipe3_to_Rn_Pipe1 = 1'b1;
				
			end
			
			if((Rn_pipe1 == Rd_pipe2) & (Rn_pipe1 != Rd_pipe3))begin
				Rd_pipe2_to_Rn_Pipe1 = 1'b1;
				Rd_pipe3_to_Rn_Pipe1 = 1'b0;
				
			end
			
			if((Rn_pipe1 != Rd_pipe2) & (Rn_pipe1 != Rd_pipe3))begin
				Rd_pipe2_to_Rn_Pipe1 = 1'b0;
				Rd_pipe3_to_Rn_Pipe1 = 1'b0;
			end
			
			
			
		end
		
		
		if((RType_pipe1 == 11'b11111000000) | (RType_pipe1 == 11'b00111000000))begin //STUR STURB
			Rd_pipe2_to_Rd_Pipe1 = 1'b0;
			Rd_pipe3_to_Rd_Pipe1 = 1'b0;
			Rd_pipe2_to_Rm_Pipe1 = 1'b0;
			Rd_pipe3_to_Rm_Pipe1 = 1'b0;
			
			Rd_pipe2_to_Rn_Pipe1 = 1'b0;
			Rd_pipe3_to_Rn_Pipe1 = 1'b0;
			store = 1'b0;
			
			Rd_forward = 1'b1;
			
			
			//RD
			if((Rd_pipe1 == Rd_pipe2) & (Rd_pipe1 == Rd_pipe3))begin
				Rd_pipe2_to_Rd_Pipe1 = 1'b1;
				Rd_pipe3_to_Rd_Pipe1 = 1'b0;
				
			end
			
			if((Rd_pipe1 != Rd_pipe2) & (Rd_pipe1 == Rd_pipe3))begin
				Rd_pipe2_to_Rd_Pipe1 = 1'b0;
				Rd_pipe3_to_Rd_Pipe1 = 1'b1;
				
			end
			
			if((Rd_pipe1 == Rd_pipe2) & (Rd_pipe1 != Rd_pipe3))begin
				Rd_pipe2_to_Rd_Pipe1 = 1'b1;
				Rd_pipe3_to_Rd_Pipe1 = 1'b0;
				
			end
			
			if((Rd_pipe1 != Rd_pipe2) & (Rd_pipe1 != Rd_pipe3))begin
				Rd_pipe2_to_Rd_Pipe1 = 1'b0;
				Rd_pipe3_to_Rd_Pipe1 = 1'b0;
			end
			
			
			
			
			
			//RN
			if((Rn_pipe1 == Rd_pipe2) & (Rn_pipe1 == Rd_pipe3))begin
				Rd_pipe2_to_Rn_Pipe1 = 1'b1;
				Rd_pipe3_to_Rn_Pipe1 = 1'b0;
				
			end
			
			if((Rn_pipe1 != Rd_pipe2) & (Rn_pipe1 == Rd_pipe3))begin
				Rd_pipe2_to_Rn_Pipe1 = 1'b0;
				Rd_pipe3_to_Rn_Pipe1 = 1'b1;
				
			end
			
			if((Rn_pipe1 == Rd_pipe2) & (Rn_pipe1 != Rd_pipe3))begin
				Rd_pipe2_to_Rn_Pipe1 = 1'b1;
				Rd_pipe3_to_Rn_Pipe1 = 1'b0;
				
			end
			
			if((Rn_pipe1 != Rd_pipe2) & (Rn_pipe1 != Rd_pipe3))begin
				Rd_pipe2_to_Rn_Pipe1 = 1'b0;
				Rd_pipe3_to_Rn_Pipe1 = 1'b0;
			end
			
			
			//Store Edge Case
			if((RType_pipe3 == 11'b11111000000) | (RType_pipe3 == 11'b00111000000))begin //STUR STURB
				//Rd_pipe3_to_Rd_Pipe1 = 1'b0;
				if(Rd_pipe3 == Rd_pipe1)begin
					//store = 1'b1;
				end
			end
		
		
			if((RType_pipe2 == 11'b11111000000) | (RType_pipe2 == 11'b00111000000))begin //STUR STURB
				//Rd_pipe2_to_Rd_Pipe1 = 1'b0;
				if(Rd_pipe2 == Rd_pipe1)begin
					store = 1'b1;
				end
			end
		
			
			
		end


		
		if((MovType_pipe1 == 9'b111100101) | (MovType_pipe1 == 9'b110100101))begin //MOVK MOVZ
			Rd_pipe2_to_Rd_Pipe1 = 1'b0;
			Rd_pipe3_to_Rd_Pipe1 = 1'b0;
			Rd_pipe2_to_Rm_Pipe1 = 1'b0;
			Rd_pipe3_to_Rm_Pipe1 = 1'b0;
			
			Rd_pipe2_to_Rn_Pipe1 = 1'b0;
			Rd_pipe3_to_Rn_Pipe1 = 1'b0;
			Rd_forward = 1'b1;
			store = 1'b0;
			
			
			//RD
			if((Rd_pipe1 == Rd_pipe2) & (Rd_pipe1 == Rd_pipe3))begin
				Rd_pipe2_to_Rd_Pipe1 = 1'b1;
				Rd_pipe3_to_Rd_Pipe1 = 1'b0;
				
			end
			
			if((Rd_pipe1 != Rd_pipe2) & (Rd_pipe1 == Rd_pipe3))begin
				Rd_pipe2_to_Rd_Pipe1 = 1'b0;
				Rd_pipe3_to_Rd_Pipe1 = 1'b1;
				
			end
			
			if((Rd_pipe1 == Rd_pipe2) & (Rd_pipe1 != Rd_pipe3))begin
				Rd_pipe2_to_Rd_Pipe1 = 1'b1;
				Rd_pipe3_to_Rd_Pipe1 = 1'b0;
				
			end
			
			if((Rd_pipe1 != Rd_pipe2) & (Rd_pipe1 != Rd_pipe3))begin
				Rd_pipe2_to_Rd_Pipe1 = 1'b0;
				Rd_pipe3_to_Rd_Pipe1 = 1'b0;
			end
			 
		end
		
		
		
		if((CBType_pipe1 == 8'b01010100) & (Instruction_pipe1[4:0] == 5'b01011) | (BType_pipe1 == 6'b000101))begin //B.LT B
			
			//No forwarding
			Rd_pipe2_to_Rd_Pipe1 = 1'b0;
			Rd_pipe3_to_Rd_Pipe1 = 1'b0;
			
			Rd_pipe2_to_Rm_Pipe1 = 1'b0;
			Rd_pipe3_to_Rm_Pipe1 = 1'b0;
			
			Rd_pipe2_to_Rn_Pipe1 = 1'b0;
			Rd_pipe3_to_Rn_Pipe1 = 1'b0;
			
			Rd_forward = 1'b0;
			store = 1'b0;
	
	
		end
		
		
		
		
		if((CBType_pipe1 == 8'b10110100))begin //CBZ
			Rd_pipe2_to_Rd_Pipe1 = 1'b0;
			Rd_pipe3_to_Rd_Pipe1 = 1'b0;
			
			Rd_pipe2_to_Rm_Pipe1 = 1'b0;
			Rd_pipe3_to_Rm_Pipe1 = 1'b0;
			
			Rd_pipe2_to_Rn_Pipe1 = 1'b0;
			Rd_pipe3_to_Rn_Pipe1 = 1'b0;
			
			Rd_forward = 1'b1;
			store = 1'b0;
			
			
			if((Rd_pipe1 == Rd_pipe2) & (Rd_pipe1 == Rd_pipe2))begin
				Rd_pipe2_to_Rd_Pipe1 = 1'b1;
				Rd_pipe3_to_Rd_Pipe1 = 1'b0;
				
				
			end
			if((Rd_pipe1 != Rd_pipe2) & (Rd_pipe1 == Rd_pipe2))begin
				Rd_pipe2_to_Rd_Pipe1 = 1'b0;
				Rd_pipe3_to_Rd_Pipe1 = 1'b1;
			end
			if((Rd_pipe1 == Rd_pipe2) & (Rd_pipe1 != Rd_pipe2))begin
				Rd_pipe2_to_Rd_Pipe1 = 1'b1;
				Rd_pipe3_to_Rd_Pipe1 = 1'b0;
			end
			if((Rd_pipe1 != Rd_pipe2) & (Rd_pipe1 != Rd_pipe2))begin
				Rd_pipe2_to_Rd_Pipe1 = 1'b0;
				Rd_pipe3_to_Rd_Pipe1 = 1'b0;
			end
		
		
			if((RType_pipe3 == 11'b11111000010) | (RType_pipe3 == 11'b00111000010))begin
				store = 1'b1;
			end
		
		end
		
		
		
		if(held_reset3)begin
			//Rd_pipe2_to_Rd_Pipe1 = 1'b0;
			//Rd_pipe3_to_Rd_Pipe1 = 1'b0;
			
			//Rd_pipe2_to_Rm_Pipe1 = 1'b0;
			//Rd_pipe3_to_Rm_Pipe1 = 1'b0;
			
			//Rd_pipe2_to_Rn_Pipe1 = 1'b0;
			//Rd_pipe3_to_Rn_Pipe1 = 1'b0;
			
			//Rd_forward = 1'b0;
		
		end
		
		
		
		
		
		//Needed Anti Garabge forwarding
		
		if(Rm_pipe1 == 5'b11111)begin
			Rd_pipe2_to_Rm_Pipe1 = 1'b0;
			Rd_pipe3_to_Rm_Pipe1 = 1'b0;
		end
		if(Rn_pipe1 == 5'b11111)begin
			Rd_pipe2_to_Rn_Pipe1 = 1'b0;
			Rd_pipe3_to_Rn_Pipe1 = 1'b0;
		end
		
		
		if((IType_pipe2 == 10'b1001000100))begin
			Rd_pipe2_to_Rm_Pipe1 = 1'b0;
		end
		
		if((IType_pipe3 == 10'b1001000100))begin
			Rd_pipe3_to_Rm_Pipe1 = 1'b0;
		end
		
		
		if((CBType_pipe2 == 8'b01010100) & (Instruction_pipe3[4:0] == 5'b01011) | (BType_pipe2 == 6'b000101))begin //B.LT B
			
			
			Rd_pipe2_to_Rd_Pipe1 = 1'b0;
			Rd_pipe2_to_Rm_Pipe1 = 1'b0;
			Rd_pipe2_to_Rn_Pipe1 = 1'b0;
	
	
		end
		
		if((CBType_pipe3 == 8'b01010100) & (Instruction_pipe3[4:0] == 5'b01011) | (BType_pipe3 == 6'b000101))begin //B.LT B
			
			
			Rd_pipe3_to_Rd_Pipe1 = 1'b0;
			Rd_pipe3_to_Rm_Pipe1 = 1'b0;
			Rd_pipe3_to_Rn_Pipe1 = 1'b0;
	
	
		end
		
		if((RType_pipe3 == 11'b11111000000) | (RType_pipe3 == 11'b00111000000))begin //STUR STURB
			if((RType_pipe1 != 11'b11111000000) & (RType_pipe1 != 11'b00111000000))begin
				Rd_pipe3_to_Rd_Pipe1 = 1'b0;
			end
			
			
			Rd_pipe3_to_Rm_Pipe1 = 1'b0;
			Rd_pipe3_to_Rn_Pipe1 = 1'b0;
		end
		
		
		if((RType_pipe2 == 11'b11111000000) | (RType_pipe2 == 11'b00111000000))begin //STUR STURB
		
			if((RType_pipe1 != 11'b11111000000) & (RType_pipe1 != 11'b00111000000))begin
				Rd_pipe2_to_Rd_Pipe1 = 1'b0;
			end
			
			
			Rd_pipe2_to_Rm_Pipe1 = 1'b0;
			Rd_pipe2_to_Rn_Pipe1 = 1'b0;
		end
		
		
		
		
		
		
		
		//Block From Forwarding Garbage might not need
//		if((RType_pipe3 == 11'b11111000010) | (RType_pipe3 == 11'b00111000010))begin //LDUR LDURB
//			
//			
//		end
//	
//		if((RType_pipe2 == 11'b11111000010) | (RType_pipe2 == 11'b00111000010))begin //LDUR LDURB
//			
//			
//		end
//		
//		

		
		
		
		
		
		
	
	end
	




endmodule


module mem_forwarding_control(Rd_pipe1,Rd_pipe2,Rd_pipe3,Reg2Loc,mem_forward_pipe2,mem_forward_pipe3,forwardData,heldreset,Instruction_pipe2,Instruction_pipe3);
	input logic [4:0] Rd_pipe1,Rd_pipe2,Rd_pipe3;
	input logic [31:0] Instruction_pipe2,Instruction_pipe3;
	input logic Reg2Loc,heldreset;
	output logic mem_forward_pipe2,mem_forward_pipe3,forwardData;
	
		wire [10:0] RType_pipe3;
		wire [8:0] MovType_pipe3;
		wire [9:0] IType_pipe3;
		
		wire [5:0] BType_pipe2,BType_pipe3;
		wire [7:0]CBType_pipe2,CBType_pipe3;
		//assign BType_pipe2 = Instruction_pipe2[31:26];
		//assign BType_pipe3 = Instruction_pipe3[31:26];
	
	
		//assign CBType_pipe2 = Instruction_pipe2[31:24];
		//assign CBType_pipe3 = Instruction_pipe3[31:24];
		
		
		assign RType_pipe2 = Instruction_pipe2[31:21];
		assign RType_pipe3 = Instruction_pipe3[31:21];
		
		assign MovType_pipe2 = Instruction_pipe2[31:23];
		assign MovType_pipe3 = Instruction_pipe3[31:23];
		
		//IType_pipe2 = Instruction_pipe2[31:22];
		assign IType_pipe3 = Instruction_pipe3[31:22];
	
	always_comb begin
		mem_forward_pipe2 = 1'b0;
		mem_forward_pipe3 = 1'b0;
		forwardData  = 1'b0;
		
	
		
		

		
		if(~Reg2Loc & ~heldreset)begin
			
			if((Rd_pipe1 == Rd_pipe2) & (Rd_pipe1 == Rd_pipe3))begin
					
					
					
					
				
					if((RType_pipe3 == 11'b11101011000) |(RType_pipe3 == 11'b10101011000) |(IType_pipe3 == 10'b1001000100) |(MovType_pipe3 == 9'b111100101) | (MovType_pipe3 == 9'b110100101))begin
						mem_forward_pipe2 = 1'b0;
						mem_forward_pipe3 = 1'b1;
					
					end else begin
						mem_forward_pipe2 = 1'b1;
						mem_forward_pipe3 = 1'b0;
					end
				
				
				forwardData  = 1'b1;
				
			end
			
			if((Rd_pipe1 == Rd_pipe2) & (Rd_pipe1 != Rd_pipe3))begin
				mem_forward_pipe2 = 1'b1;
				mem_forward_pipe3 = 1'b0;
				forwardData  = 1'b1;
				
			end
			
			if((Rd_pipe1 != Rd_pipe2) & (Rd_pipe1 == Rd_pipe3))begin
				
				mem_forward_pipe2 = 1'b0;
				mem_forward_pipe3 = 1'b1;
				forwardData  = 1'b1;
			end
			
			
			
			
		end
		
//	if(MemWrite_pipe2|MemWrite_pipe3)begin
//			Rm_forward =1'b0;
//	end
//		
//		Rn_forward = (Rn_forward | Rn_Mem_forward);
//		Rm_forward = (Rm_forward | Rm_Mem_forward);
//		if((Rm_pipe1 == 5'b11111))begin
//			Rm_forward = 1'b0;
//		end
//		if((Rn_pipe1 == 5'b11111))begin
//			Rn_forward = 1'b0;
//	end
	
	
//	if((RType_pipe3 == 11'b11111000000)|(RType_pipe3 == 11'b00111000000))begin
//				mem_forward_pipe3 = 1'b1;
//		end
//		//(RType_pipe2 == 11'b11111000010)|(RType_pipe2 == 11'b00111000010)|
//		if((RType_pipe2 == 11'b11111000000)|(RType_pipe2 == 11'b00111000000))begin
//				mem_forward_pipe2 = 1'b0;
//				
//		end
//		
//		
//		if((BType_pipe2 == 6'b000101) | (CBType_pipe2 == 8'b01010100) |(CBType_pipe2 == 8'b10110100))begin
//			mem_forward_pipe2 =1'b0;
//		
//		end
//		
//		if((BType_pipe3 == 6'b000101) | (CBType_pipe3 == 8'b01010100) |(CBType_pipe3 == 8'b10110100))begin
//				mem_forward_pipe3 = 1'b1;
//		
//		
//		end
//	
//
//	
	end
	
	
	
endmodule


//module forwarding_control(Rm_pipe1,Rn_pipe1,Rd_pipe2,Rd_pipe3,Rn_forward,Rm_forward,Rn_Mem_forward,Rm_Mem_forward,MemWrite_pipe2,MemWrite_pipe3,Instruction_pipe2,Instruction_pipe3);
//	//input logic [63:0] ReadData1,ReadData2;
//	input logic [4:0] Rm_pipe1,Rn_pipe1,Rd_pipe2,Rd_pipe3;
//	input logic [31:0] Instruction_pipe2,Instruction_pipe3;
//	input logic MemWrite_pipe2,MemWrite_pipe3;
//	output logic Rm_forward,Rn_forward,Rn_Mem_forward,Rm_Mem_forward;
//	//output logic [63:0] ReadData1_forward,ReadData2_forward;
//	
//	
//	
//		wire [10:0] RType_pipe2,RType_pipe3;
//		wire [8:0] MovType_pipe3;
//		wire [9:0] IType_pipe3;
//		
//		//assign RType_pipe3 = Instruction_pipe3[31:21];
//		//assign RType_pipe2 = Instruction_pipe2[31:21];
//		//assign MovType_pipe3 = Instruction_pipe3[31:23];
//		//assign IType_pipe3 = Instruction_pipe3[31:22];
//	
//		wire [5:0] BType_pipe2,BType_pipe3;
//		wire [7:0]CBType_pipe2,CBType_pipe3;
//	assign BType_pipe2 = Instruction_pipe2[31:26];
//	assign BType_pipe3 = Instruction_pipe3[31:26];
//	
//	
//	assign CBType_pipe2 = Instruction_pipe2[31:24];
//	assign CBType_pipe3 = Instruction_pipe3[31:24];
//	
//	always_comb begin
//	
//		
//	
//	
//	
//		Rn_forward = 1'b0;
//		Rm_forward = 1'b0;
//		Rn_Mem_forward = 1'b0;
//		Rm_Mem_forward = 1'b0;
//		
//		if(Rd_pipe2 != Rd_pipe3)begin
//			
//			if((Rm_pipe1 == Rd_pipe2) & (Rn_pipe1 == Rd_pipe2))begin
//				Rn_forward = 1'b1;
//				Rm_forward = 1'b1;
//				Rn_Mem_forward = 1'b0;
//				Rm_Mem_forward = 1'b0;
//				
//				
//			end
//			
//				
//			if((Rm_pipe1 != Rd_pipe2) & (Rn_pipe1 != Rd_pipe2))begin
//				Rn_forward = 1'b0;
//				Rm_forward = 1'b0;
//				
//				
//				
//				if(Rn_pipe1 == Rd_pipe3)begin
//					Rn_Mem_forward = 1'b1;
//				end else begin
//					Rn_Mem_forward = 1'b0;
//				end
//				
//				
//				
//				if(Rm_pipe1 == Rd_pipe3)begin
//					Rm_Mem_forward = 1'b1;
//				end else begin
//					Rm_Mem_forward = 1'b0;
//				end
//				
//				
//			end
//				
//			//THIS IF	BROKKKKEN
//			if((Rm_pipe1 == Rd_pipe2) & (Rn_pipe1 != Rd_pipe2))begin
//				Rn_forward = 1'b0;
//				Rm_forward = 1'b1;
//				Rm_Mem_forward = 1'b0;
//				
//				if(Rn_pipe1 == Rd_pipe3)begin
//					Rn_Mem_forward = 1'b1;
//				end else begin
//					Rn_Mem_forward = 1'b0;
//				end
//			end
//			
//			if((Rm_pipe1 != Rd_pipe2) & (Rn_pipe1 == Rd_pipe2))begin
//				Rn_forward = 1'b1;
//				Rm_forward = 1'b0;
//				Rn_Mem_forward = 1'b0;
//				if(Rm_pipe1 == Rd_pipe3)begin
//					Rm_Mem_forward = 1'b1;
//				end else begin
//					Rm_Mem_forward = 1'b0;
//				end
//
//			end
//			
//
//		end else begin
//		
//			if((Rm_pipe1 == Rd_pipe2) & (Rn_pipe1 == Rd_pipe2))begin
//				Rn_forward = 1'b1;
//				Rm_forward = 1'b1;
//				Rn_Mem_forward = 1'b0;
//				Rm_Mem_forward = 1'b0;
//				
//				
//			end
//				
//			if((Rm_pipe1 != Rd_pipe2) & (Rn_pipe1 != Rd_pipe2))begin
//				Rn_forward = 1'b0;
//				Rm_forward = 1'b0;
//				Rn_Mem_forward = 1'b0;
//				Rm_Mem_forward = 1'b0;
//				
//				
//				
//			end
//				
//				
//			if((Rm_pipe1 == Rd_pipe2) & (Rn_pipe1 != Rd_pipe2))begin
//				Rn_forward = 1'b0;
//				Rm_forward = 1'b1;
//				Rn_Mem_forward = 1'b0;
//				Rm_Mem_forward = 1'b0;
//				
//				
//			end
//			
//			if((Rm_pipe1 != Rd_pipe2) & (Rn_pipe1 == Rd_pipe2))begin
//				Rn_forward = 1'b1;
//				Rm_forward = 1'b0;
//				Rn_Mem_forward = 1'b0;
//				Rm_Mem_forward = 1'b0;
//
//			end
//			
//		end
//		//(RType_pipe3 == 11'b11111000010)|(RType_pipe3 == 11'b00111000010)|
//		
////		if((RType_pipe3 == 11'b11111000000)|(RType_pipe3 == 11'b00111000000))begin
////				Rn_Mem_forward = 1'b0;
////				Rm_Mem_forward = 1'b0;
////		end
////		//(RType_pipe2 == 11'b11111000010)|(RType_pipe2 == 11'b00111000010)|
////		if((RType_pipe2 == 11'b11111000000)|(RType_pipe2 == 11'b00111000000))begin
////				Rn_forward = 1'b0;
////				Rm_forward = 1'b0;
////		end
////		
////		
////		if((BType_pipe2 == 6'b000101) | (CBType_pipe2 == 8'b01010100) |(CBType_pipe2 == 8'b10110100))begin
////			Rm_forward =1'b0;
////			Rn_forward =1'b0;
////		
////		
////		end
////		
////		if((BType_pipe3 == 6'b000101) | (CBType_pipe3 == 8'b01010100) |(CBType_pipe3 == 8'b10110100))begin
////			Rm_Mem_forward =1'b0;
////			Rn_Mem_forward =1'b0;
////		
////		
////		end
//
//		
//		
//		if(MemWrite_pipe2|MemWrite_pipe3)begin
//			Rm_forward =1'b0;
//		end
//		
//		Rn_forward = (Rn_forward | Rn_Mem_forward);
//		Rm_forward = (Rm_forward | Rm_Mem_forward);
//		if((Rm_pipe1 == 5'b11111))begin
//			Rm_forward = 1'b0;
//		end
//		if((Rn_pipe1 == 5'b11111))begin
//			Rn_forward = 1'b0;
//		end
//		
//		
//	
//	end
//endmodule 


module movK(in,Imm16,movShamt,out);
	input logic [63:0] in;
	input logic [1:0] movShamt;
	input logic [15:0] Imm16;
	logic [63:0] shiftout,mask;
	output logic [63:0] out;
	//new code
	wire [63:0] mask0,mask1,mask2,mask3;
	assign mask0 = 64'hFFFFFFFFFFFF0000;
	assign mask1 = 64'hFFFFFFFF0000FFFF;
	assign mask2 = 64'hFFFF0000FFFFFFFF;
	assign mask3 = 64'h0000FFFFFFFFFFFF;
	mux4_64 maskSel(mask,mask0,mask1,mask2,mask3,movShamt);
	
	wire [63:0] shift0,shift1,shift2,shift3;
	assign shift0 = {{48'b0},Imm16};
	assign shift1 = {{32'b0},Imm16,{16'b0}};
	assign shift2 = {{16'b0},Imm16,{32'b0}};
	assign shift3 = {Imm16,{48'b0}};
	mux4_64 shiftSel(shiftout,shift0,shift1,shift2,shift3,movShamt);
	
	wire [63:0] inANDmask;
	
	AND64 filter(mask,in,inANDmask);
	OR64 	ADD(inANDmask,shiftout,out);
	
	//assign out = ((in & mask)|shiftout);//replace with logic gates

endmodule
module AND64(A,B, out);
	input logic [63:0] A,B;
	output logic [63:0] out;
	
	genvar i;
	
	generate	
		for(i = 0; i < 64; i++) begin: AND64
			and#delay gategen(out[i],A[i],B[i]);
		end
	endgenerate


endmodule

module OR64(A,B, out);
	input logic [63:0] A,B;
	output logic [63:0] out;
	
	genvar i;
	
	generate	
		for(i = 0; i < 64; i++) begin: OR64
			or#delay gategen(out[i],A[i],B[i]);
		end
	endgenerate

endmodule

module movZ(Imm16,movShamt,out);
	input logic [1:0] movShamt;
	input logic [15:0] Imm16;
	output logic [63:0] out;
	
	wire [63:0] shift0,shift1,shift2,shift3;
	assign shift0 = {{48'b0},Imm16};
	assign shift1 = {{32'b0},Imm16,{16'b0}};
	assign shift2 = {{16'b0},Imm16,{32'b0}};
	assign shift3 = {Imm16,{48'b0}};
	mux4_64 shiftSel(out,shift0,shift1,shift2,shift3,movShamt);

endmodule


module control_Testbench();
	logic [31:0] Instruction;
	logic [3:0] flagReg,flagALU;
	logic UncondBr,BrTaken,MemToReg,ALUSrc,Reg2Loc,SetFlags,RegWrite,ALUConstant,MemWrite,Byte,movk,movz;
	logic [4:0] Rd,Rn,Rm;
	logic [2:0] ALUOP;
	logic [3:0] xfer_size;
	
	controlDataPath dut(Instruction,UncondBr,BrTaken,MemToReg,ALUSrc,RegWrite,Reg2Loc,SetFlags,ALUOP,xfer_size,ALUConstant,MemWrite,Rd,Rn,Rm,flagReg,flagALU,Byte,movk,movz);
	
	initial begin
		Instruction = 32'b1001000100_110011001100_11111_00010; flagReg = 4'hF;flagALU = 4'hF;#1000;
		Instruction = 32'b110100101_00_1101111010101101_00000; flagReg = 4'hF;flagALU = 4'hF;#1000;

		Instruction = 32'b110100101_01_1011111011101111_00000; flagReg = 4'hF;flagALU = 4'hF;#1000;
	
	end
	


endmodule 

module controlDataPath(Instruction,UncondBr,BrTaken,MemToReg,ALUSrc,RegWrite,Reg2Loc,SetFlags,ALUOP,xfer_size,ALUConstant,MemWrite,Rd,Rn,Rm,flagReg,flagALU,Byte,movk,movz,CBZ,SetFlags_pipe2,BLT);
	input logic [31:0] Instruction;
	input logic [3:0] flagReg,flagALU;
	input logic SetFlags_pipe2;
	output logic UncondBr,BrTaken,MemToReg,ALUSrc,RegWrite,Reg2Loc,SetFlags,ALUConstant,MemWrite,Byte,movk,movz,CBZ,BLT;//,Rn_allow_forward,Rm_allow_forward;
	output logic [4:0] Rd,Rn,Rm;
	output logic [2:0] ALUOP;
	output logic [3:0] xfer_size;
	
	


	
	wire [5:0] BType;
	wire [7:0]CBType;
	wire [10:0]RType;
	wire [9:0]IType;
	wire [8:0]MovType;
	
	
	

	
	assign BType = Instruction[31:26];
	assign CBType = Instruction[31:24];
	assign RType = Instruction[31:21];
	assign IType = Instruction[31:22];
	assign MovType = Instruction[31:23];
	
	
	always_comb begin
		CBZ = 1'b0;
		if(Instruction == 32'bx)begin
			UncondBr = 1'b0;
			BrTaken =  1'b0;
			MemToReg = 1'b0;
			ALUSrc = 1'b0;
			RegWrite = 1'b0;
			MemWrite = 1'b0;
			Reg2Loc = 1'b1;
			SetFlags = 1'b0;
			ALUOP  = 3'b000;
			xfer_size  = 4'b0;
			ALUConstant = 1'b0;
			Byte = 1'b0;
			CBZ = 1'b0;
			BLT = 1'b0;
			
			
						
			movk = 1'b0;
			movz = 1'b0;
			
			Rd = 5'b11111;
			Rn = 5'b11111;
			Rm = 5'b11111;
			
			
			
			//Rn_allow_forward = 1'b0;
			//Rm_allow_forward = 1'b0;
			
		
		end
		
		
		if(Instruction == 32'b0)begin
			UncondBr = 1'b0;
			BrTaken =  1'b0;
			MemToReg = 1'b0;
			ALUSrc = 1'b0;
			RegWrite = 1'b0;
			MemWrite = 1'b0;
			Reg2Loc = 1'b1;
			SetFlags = 1'b0;
			ALUOP  = 3'b000;
			xfer_size  = 4'b0;
			ALUConstant = 1'b0;
			Byte = 1'b0;
			CBZ = 1'b0;
			BLT = 1'b0;
			
			movk = 1'b0;
			movz = 1'b0;
			
			Rd = 5'b11111;
			Rn = 5'b11111;
			Rm = 5'b11111;
		
			//Rn_allow_forward = 1'b0;
			//Rm_allow_forward = 1'b0;
		end
		
		
		
		
		if(MovType == 9'b111100101)begin //MOVK
			UncondBr = 1'b0;
			BrTaken =  1'b0;
			MemToReg = 1'b0;
			ALUSrc = 1'b0;
			RegWrite = 1'b1;
			MemWrite = 1'b0;
			Reg2Loc = 1'b0;
			SetFlags = 1'b0;
			ALUOP  = 3'b000;
			xfer_size  = 4'b0;
			ALUConstant = 1'b0;
			Byte = 1'b0;
			CBZ = 1'b0;
			BLT = 1'b0;
			
			movk = 1'b1;
			movz = 1'b0;
			
			Rd = Instruction[4:0];
			Rn = Instruction[9:5];
			Rm = Instruction[20:16];
			
			//Rn_allow_forward = 1'b0;
			//Rm_allow_forward = 1'b0;
			 
			 
		end
		
		if(MovType == 9'b110100101)begin //MOVZ
			UncondBr =1'b0;
			BrTaken =  1'b0;
			MemToReg = 1'b0;
			ALUSrc = 1'b0;
			RegWrite = 1'b1;
			MemWrite = 1'b0;
			Reg2Loc = 1'b0;
			SetFlags = 1'b0;
			ALUOP  = 3'b000;
			xfer_size  = 4'b0;
			ALUConstant = 1'b1;
			Byte = 1'b0;
			CBZ = 1'b0;
			BLT = 1'b0;
			
			movk = 1'b0;
			movz = 1'b1;
			
			Rd = Instruction[4:0];
			Rn = Instruction[9:5];
			Rm = Instruction[20:16];
			
			//Rn_allow_forward = 1'b0;
			//Rm_allow_forward = 1'b0;
			 
			 
		end
	
		
		if(BType == 6'b000101)begin //B
			UncondBr =1'b1;
			BrTaken =  1'b1;
			MemToReg = 1'b0;
			ALUSrc = 1'b0;
			RegWrite = 1'b0;
			MemWrite = 1'b0;
			Reg2Loc = 1'b0;
			SetFlags = 1'b0;
			ALUOP =  3'b0;
			xfer_size  = 4'b0;
			ALUConstant = 1'b0;
			Byte = 1'b0;
			CBZ = 1'b0;
			BLT = 1'b0;
			
			movk = 1'b0;
			movz = 1'b0;
			
			Rd = Instruction[4:0];
			Rn = Instruction[9:5];
			Rm = Instruction[20:16];
			
			//Rn_allow_forward = 1'b0;
			//Rm_allow_forward = 1'b0;
		end
		
		if(RType == 11'b11101011000)begin //SUBS
			UncondBr =1'b0;
			BrTaken =  1'b0;
			MemToReg = 1'b0;
			ALUSrc = 1'b0;
			RegWrite = 1'b1;
			MemWrite = 1'b0;
			Reg2Loc = 1'b1;
			SetFlags = 1'b1;
			ALUOP  = 3'b011;
			xfer_size  = 4'b0;
			ALUConstant = 1'b0;
			Byte = 1'b0;
			CBZ = 1'b0;
			BLT = 1'b0;
			
			movk = 1'b0;
			movz = 1'b0;
		
			Rd = Instruction[4:0];
			Rn = Instruction[9:5];
			Rm = Instruction[20:16];
			
			//Rn_allow_forward = 1'b1;
			//Rm_allow_forward = 1'b1;
			
		
		end
		
		if(RType == 11'b10101011000)begin //ADDS
			UncondBr =1'b0;
			BrTaken =  1'b0;
			MemToReg = 1'b0;
			ALUSrc = 1'b0;
			RegWrite = 1'b1;
			MemWrite = 1'b0;
			Reg2Loc = 1'b1;
			SetFlags = 1'b1;
			ALUOP  = 3'b010;
			xfer_size  = 4'b0;
			ALUConstant = 1'b0;
			Byte = 1'b0;
			CBZ = 1'b0;
			BLT = 1'b0;
			
			movk = 1'b0;
			movz = 1'b0;
			
			Rd = Instruction[4:0];
			Rn = Instruction[9:5];
			Rm = Instruction[20:16];
			
			//Rn_allow_forward = 1'b1;
			//Rm_allow_forward = 1'b1;
			
		end
		
		if(IType == 10'b1001000100)begin //ADDI
			UncondBr =1'b0;
			BrTaken =  1'b0;
			MemToReg = 1'b0;
			ALUSrc = 1'b1;
			RegWrite = 1'b1;
			MemWrite = 1'b0;
			Reg2Loc = 1'b1;
			SetFlags = 1'b0;
			ALUOP  = 3'b010;
			xfer_size  = 4'b0;
			ALUConstant = 1'b1;
			Byte = 1'b0;
			CBZ = 1'b0;
			BLT = 1'b0;
						
			movk = 1'b0;
			movz = 1'b0;
			
			Rd = Instruction[4:0];
			Rn = Instruction[9:5];
			Rm = Instruction[20:16];
			
			
			//Rn_allow_forward = 1'b0;
			//Rm_allow_forward = 1'b0;
			 
		end
		
		if(RType == 11'b11111000010)begin //LDUR
			UncondBr =1'b0;
			BrTaken =  1'b0;
			MemToReg = 1'b1;
			ALUSrc = 1'b1;
			RegWrite = 1'b1;
			MemWrite = 1'b0;
			Reg2Loc = 1'b0;
			SetFlags = 1'b0;
			ALUOP  = 3'b010;
			xfer_size  = 4'd8;
			ALUConstant = 1'b0;
			Byte = 1'b0;
			CBZ = 1'b0;
			BLT = 1'b0;
						
			movk = 1'b0;
			movz = 1'b0;
			
			Rd = Instruction[4:0];
			Rn = Instruction[9:5];
			Rm = Instruction[20:16];
		end
		
		if(RType == 11'b00111000010)begin //LDURB
			UncondBr =1'b0;
			BrTaken =  1'b0;
			MemToReg = 1'b1;
			ALUSrc = 1'b1;
			RegWrite = 1'b1;
			MemWrite = 1'b0;
			Reg2Loc = 1'b0;
			SetFlags = 1'b0;
			ALUOP  = 3'b010;
			xfer_size  = 4'd1;
			ALUConstant = 1'b0;
			Byte = 1'b1;
			CBZ = 1'b0;
			BLT = 1'b0;
						
			movk = 1'b0;
			movz = 1'b0;
			
			Rd = Instruction[4:0];
			Rn = Instruction[9:5];
			Rm = Instruction[20:16];
		end
		
		
		if(RType == 11'b11111000000)begin //STUR
			UncondBr =1'b0;
			BrTaken =  1'b0;
			MemToReg = 1'b0;
			ALUSrc = 1'b1;
			RegWrite = 1'b0;
			MemWrite = 1'b1;
			Reg2Loc = 1'b0;
			SetFlags = 1'b0;
			ALUOP  = 3'b010;
			xfer_size  = 4'd8;
			ALUConstant = 1'b0;
			Byte = 1'b0;
			CBZ = 1'b0;
			BLT = 1'b0;
						
			movk = 1'b0;
			movz = 1'b0;
			 
			Rd = Instruction[4:0];
			Rn = Instruction[9:5];
			Rm = Instruction[20:16];
		end

		
		if(RType == 11'b00111000000)begin //STURB
			UncondBr =1'b0;
			BrTaken =  1'b0;
			MemToReg = 1'b0;
			ALUSrc = 1'b1;
			RegWrite = 1'b0;
			MemWrite = 1'b1;
			Reg2Loc = 1'b0;
			SetFlags = 1'b0;
			ALUOP  = 3'b010;
			xfer_size  = 4'd1;
			ALUConstant = 1'b0;
			Byte = 1'b1;
			CBZ = 1'b0;
			BLT = 1'b0;
						
			movk = 1'b0;
			movz = 1'b0;
			
			Rd = Instruction[4:0];
			Rn = Instruction[9:5];
			Rm = Instruction[20:16];
		end
		
		
		
		if((CBType == 8'b01010100) & (Instruction[4:0] == 5'b01011))begin //B.LT
			
			
			
			MemToReg = 1'b0;
			ALUSrc = 1'b0;
			RegWrite = 1'b0;
			MemWrite = 1'b0;
			Reg2Loc = 1'b0;
			SetFlags = 1'b0;
			ALUOP  = 3'b0;
			xfer_size  = 4'b0;
			ALUConstant = 1'b0;
			Byte = 1'b0;
			CBZ = 1'b0;
			BLT = 1'b1;
						
			movk = 1'b0;
			movz = 1'b0;
			
			Rd = Instruction[4:0];
			Rn = Instruction[9:5];
			Rm = Instruction[20:16];
			
			
//			if(flagReg[1] != flagReg[2])begin //if neg != overflow
//				UncondBr =1'b0;
//				BrTaken =  1'b1;
//			end
//			else begin
//				UncondBr =1'b0;
//				BrTaken =  1'b0;
//			
//			end

			UncondBr = 1'b0;
			BrTaken =  1'b0;
//			
		
		
		end
		
		
		
		
		if((CBType == 8'b10110100))begin //CBZ
			
			Rd = Instruction[4:0];
			Rn = 5'b11111;
			Rm = 5'b11111;
			
			
			
			MemToReg = 1'b0;
			ALUSrc = 1'b0;
			RegWrite = 1'b0;
			MemWrite = 1'b0;
			Reg2Loc = 1'b0;
			SetFlags = 1'b0;
			ALUOP  = 3'b000;
			xfer_size  = 4'b0;
			ALUConstant = 1'b0;
			Byte = 1'b0;
			CBZ = 1'b1;
			BLT = 1'b0;
						
			movk = 1'b0;
			movz = 1'b0;

			UncondBr = 1'b0;
			BrTaken =  1'b0;
			
			
			
//			if(flagALU[0])begin //if zero flag == 0
//				UncondBr =1'b0;
//				BrTaken =  1'b1;
//			end
//			else begin
//				UncondBr =1'b0;
//				BrTaken =  1'b0;
//			
//			end
		
		
		end
		
		
	
	end

endmodule 



module ProgramCounter(nextAddr,Addr,reset,clk);
	input logic[63:0]nextAddr;
	input logic clk,reset;
	output logic[63:0]Addr;
	
	Register64b PCReg(nextAddr,Addr,1'b1,reset,clk);
	
endmodule

//module ProgramCounterDataPath(UncondBr,BrTaken,Addr_CBZ,CondAddr19,BrAddr26,reset,clk,CBZ);
//	input logic UncondBr,BrTaken,reset,clk,CBZ;
//	input logic [18:0] CondAddr19;
//	input logic [25:0] BrAddr26;
//	output logic [63:0] Addr_CBZ;
//	wire [63:0] CondAddr_ex,BrAddr26_ex,BranchAddr,Addr;
//	SignExtendImm19 extend19(CondAddr19,CondAddr_ex);
//	SignExtendImm26 extend26(BrAddr26,BrAddr26_ex);
//
//
//	
//	wire [63:0] nextAddr,PCinc4,PCincBr,nextAddr_CBZ,CBZ_adderOUT;
//	wire [63:0] PC_Uncond;
//	mux2_64 UncondBrMux(PC_Uncond, CondAddr_ex, BrAddr26_ex, UncondBr);
//	//shifter UncondBrShifter(PC_Uncond,1'b0,6'd2,BranchAddr);
//	assign BranchAddr = {{PC_Uncond[61:0]},2'b0};
//	
//	wire c1,c2,c3;
//	adder_64b PCINC(Addr,64'd4,PCinc4,c1);
//	
//	
//	adder_64b UncondBrINC(Addr,BranchAddr,PCincBr,c2);
//	
//	
//	mux2_64 BrTakenMux(nextAddr,PCinc4, PCincBr, BrTaken);
//	
//	adder_64b CBZadder(64'd4,Addr_CBZ,CBZ_adderOUT,c3);
//	
//	
//	mux2_64 CBZ_nextAddr(nextAddr_CBZ,nextAddr,CBZ_adderOUT, CBZ);
//	
//	
//	ProgramCounter PC(nextAddr_CBZ,Addr,reset,clk);
//	
//	mux2_64 CBZ_Addr(Addr_CBZ,Addr, nextAddr, CBZ);
//	
//
//endmodule

module ProgramCounterDataPath(UncondBr,BrTaken,Addr,CondAddr19,BrAddr26,reset,clk);
	input logic UncondBr,BrTaken,reset,clk;
	input logic [18:0] CondAddr19;
	input logic [25:0] BrAddr26;
	output logic [63:0] Addr;
	wire [63:0] CondAddr_ex,BrAddr26_ex,BranchAddr;
	SignExtendImm19 extend19(CondAddr19,CondAddr_ex);
	SignExtendImm26 extend26(BrAddr26,BrAddr26_ex);


	
	wire [63:0] PCinc,nextAddr,PCinc4,PCincBr,nextAddr_adj;
	wire [63:0] PC_Uncond;
	mux2_64 UncondBrMux(PC_Uncond, CondAddr_ex, BrAddr26_ex, UncondBr);
	//shifter UncondBrShifter(PC_Uncond,1'b0,6'd2,BranchAddr);
	assign BranchAddr = {{PC_Uncond[61:0]},2'b0};
	
	wire c1,c2,c3;
	wire takeBranch;
	adder_64b PCINC(Addr,64'd4,PCinc4,c1);
	
	
	adder_64b UncondBrINC(Addr,BranchAddr,PCincBr,c2);
	
	
	mux2_64 BrTakenMux(PCinc,PCinc4, PCincBr, takeBranch);
	
	
	
	
	adder_64b DelaySlotAddressFixer(PCinc,64'hFFFFFFFFFFFFFFFC,nextAddr_adj,c3);
	
	
	
	or#delay branch_orGate(takeBranch,UncondBr,BrTaken);
	
	mux2_64 NextAddr(nextAddr,PCinc, nextAddr_adj, takeBranch);
	
	
	ProgramCounter PC(nextAddr,Addr,reset,clk);


endmodule


module PC_Testbench();
	logic UncondBr,BrTaken,clk,reset;
	logic [18:0] CondAddr19;
	logic [25:0] BrAddr26;
	logic [63:0] Addr;
	ProgramCounterDataPath dut(UncondBr,BrTaken,Addr,CondAddr19,BrAddr26,reset,clk);
	parameter clk_PERIOD = 100000;	
	initial begin	
		clk <= 0;	
		forever #(clk_PERIOD/2) clk <= ~clk;	
	end
	initial begin	
		UncondBr <= 1'b0;BrTaken<=1'b0;CondAddr19 <= 19'd0;reset <= 1;BrAddr26 <= 26'd0;@(posedge clk);
		UncondBr <= 1'b0;BrTaken<=1'b0;CondAddr19 <= 19'd0;reset <= 1;BrAddr26 <= 26'd0;@(posedge clk);
		UncondBr <= 1'b0;BrTaken<=1'b0;CondAddr19 <= 19'd0;reset <= 0;BrAddr26 <= 26'd0;@(posedge clk);
		UncondBr <= 1'b0;BrTaken<=1'b0;CondAddr19 <= 19'd0;reset <= 0;BrAddr26 <= 26'd0;@(posedge clk);
		UncondBr <= 1'b0;BrTaken<=1'b0;CondAddr19 <= 19'd0;reset <= 0;BrAddr26 <= 26'd0;@(posedge clk);


		$stop;
	end
	
	


endmodule




module SignExtendImm26(in,out);
	input logic [25:0] in;
	output logic [63:0] out;
	
	assign out = {{38{in[25]}},in[25:0]};

endmodule

module ZeroExtend8(in,ByteLOAD,out);
	input logic [63:0] in;
	input logic ByteLOAD;
	output logic [63:0] out;
	
	mux2_64 Byte(out,in,{{56'b0,in[7:0]}},ByteLOAD);



endmodule

module ZeroExtend12(in, out);
	input logic [11:0] in;
	output logic [63:0] out;
	
	assign out = {{52'b0,in[11:0]}};

endmodule

module SignExtendImm19(in,out);
	input logic [18:0] in;
	output logic [63:0] out;
	
	assign out = {{45{in[18]}},in[18:0]};


endmodule

module SignExtendImm9(in,out);
	input logic [8:0] in;
	output logic [63:0] out;
	
	assign out = {{55{in[8]}},in[8:0]};

endmodule






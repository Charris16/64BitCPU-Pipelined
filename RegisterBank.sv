module D_FF (q, d, reset, clk);//basic dff taken straight from lab1 doc
												
		output reg q;
		input d, reset, clk;
		always_ff @(posedge clk)
			if (reset)
				q <= 0; // On reset, set to 0
			else
				q <= d; // Otherwise out = d
endmodule 

module D_FF_special (q, d, reset, clk);//basic dff taken straight from lab1 doc
												
		output reg q;
		input d, reset, clk;
		always_ff @(posedge clk)
				q <= d; // Otherwise out = d
endmodule 



module register(in, out,update,reset,clk);
//dff with register to control its value
//only updates when update value = 1
//otherwise holds previous input														
	input logic in,update,reset,clk;
	output logic out;
	
	wire inputFF;
	mux2_1 dataSel(inputFF,out,in,update);
	//uses a 2 by 1 mux to control data flow
	D_FF flipflop(out,inputFF,reset,clk);


endmodule


module Register64b(in,out,update,reset,clk);
	input logic [63:0] in;
	input logic update;
	input logic reset,clk;
	
	output logic [63:0] out;
	
	
	genvar i;
	
	generate	
	//uses a generate statment to create 64 1 bit registers to hold a 64 bit value
	//all 1bit registers share update,reset and clk
	//outputs to a 64bit output
		for(i = 0; i < 64; i++) begin: REG64b
			register EACHREG(in[i],out[i],update,reset,clk);
		end
	endgenerate
	
endmodule

module Register32b(in,out,update,reset,clk);
	input logic [31:0] in;
	input logic update;
	input logic reset,clk;
	
	output logic [31:0] out;
	
	
	genvar i;
	
	generate	
	//uses a generate statment to create 32 1 bit registers to hold a 32 bit value
	//all 1bit registers share update,reset and clk
	//outputs to a 32bit output
		for(i = 0; i < 32; i++) begin: REG64b
			register EACHREG(in[i],out[i],update,reset,clk);
		end
	endgenerate
	
endmodule



module Register2b(in,out,update,reset,clk);
	input logic [1:0] in;
	input logic update;
	input logic reset,clk;
	
	output logic [1:0] out;
	
	genvar i;
	
	generate	
	//uses a generate statment to create 2 1 bit registers to hold a 2 bit value
	//all 1bit registers share update,reset and clk
	//outputs to a 2bit output
		for(i = 0; i < 2; i++) begin: REG2b
			register EACHREG(in[i],out[i],update,reset,clk);
		end
	endgenerate
	
endmodule

module Register3b(in,out,update,reset,clk);
	input logic [2:0] in;
	input logic update;
	input logic reset,clk;
	
	output logic [2:0] out;
	
	genvar i;
	
	generate	
	//uses a generate statment to create 3 1 bit registers to hold a 3 bit value
	//all 1bit registers share update,reset and clk
	//outputs to a 3bit output
		for(i = 0; i < 3; i++) begin: REG2b
			register EACHREG(in[i],out[i],update,reset,clk);
		end
	endgenerate
	
endmodule

module Register4b(in,out,update,reset,clk);
	input logic [3:0] in;
	input logic update;
	input logic reset,clk;
	
	output logic [3:0] out;
	
	genvar i;
	
	generate	
	//uses a generate statment to create 4 1 bit registers to hold a 4 bit value
	//all 1bit registers share update,reset and clk
	//outputs to a 4bit output
		for(i = 0; i < 4; i++) begin: REG2b
			register EACHREG(in[i],out[i],update,reset,clk);
		end
	endgenerate
	
endmodule


module Register12b(in,out,update,reset,clk);
	input logic [11:0] in;
	input logic update;
	input logic reset,clk;
	
	output logic [11:0] out;
	
	genvar i;
	
	generate	
	//uses a generate statment to create 12 1 bit registers to hold a 12 bit value
	//all 1bit registers share update,reset and clk
	//outputs to a 12bit output
		for(i = 0; i < 12; i++) begin: REG12b
			register EACHREG(in[i],out[i],update,reset,clk);
		end
	endgenerate
	
endmodule


module Register9b(in,out,update,reset,clk);
	input logic [8:0] in;
	input logic update;
	input logic reset,clk;
	
	output logic [8:0] out;
	
	genvar i;
	
	generate	
	//uses a generate statment to create 9 1 bit registers to hold a 9 bit value
	//all 1bit registers share update,reset and clk
	//outputs to a 9bit output
		for(i = 0; i < 9; i++) begin: REG9b
			register EACHREG(in[i],out[i],update,reset,clk);
		end
	endgenerate
	
endmodule


module Register26b(in,out,update,reset,clk);
	input logic [25:0] in;
	input logic update;
	input logic reset,clk;
	
	output logic [25:0] out;
	
	genvar i;
	
	generate	
	//uses a generate statment to create 26 1 bit registers to hold a 26 bit value
	//all 1bit registers share update,reset and clk
	//outputs to a 26bit output
		for(i = 0; i < 26; i++) begin: REG26b
			register EACHREG(in[i],out[i],update,reset,clk);
		end
	endgenerate
	
endmodule


module Register19b(in,out,update,reset,clk);
	input logic [18:0] in;
	input logic update;
	input logic reset,clk;
	
	output logic [18:0] out;
	
	genvar i;
	
	generate	
	//uses a generate statment to create 19 1 bit registers to hold a 19 bit value
	//all 1bit registers share update,reset and clk
	//outputs to a 19bit output
		for(i = 0; i < 19; i++) begin: REG19b
			register EACHREG(in[i],out[i],update,reset,clk);
		end
	endgenerate
	
endmodule


module Register6b(in,out,update,reset,clk);
	input logic [5:0] in;
	input logic update;
	input logic reset,clk;
	
	output logic [5:0] out;
	
	genvar i;
	
	generate	
	//uses a generate statment to create 6 1 bit registers to hold a 6 bit value
	//all 1bit registers share update,reset and clk
	//outputs to a 6bit output
		for(i = 0; i < 6; i++) begin: REG6b
			register EACHREG(in[i],out[i],update,reset,clk);
		end
	endgenerate
	
endmodule


module Register16b(in,out,update,reset,clk);
	input logic [15:0] in;
	input logic update;
	input logic reset,clk;
	
	output logic [15:0] out;
	
	genvar i;
	
	generate	
	//uses a generate statment to create 16 1 bit registers to hold a 16 bit value
	//all 1bit registers share update,reset and clk
	//outputs to a 16bit output
		for(i = 0; i < 16; i++) begin: REG16b
			register EACHREG(in[i],out[i],update,reset,clk);
		end
	endgenerate
	
endmodule


module Register5b(in,out,update,reset,clk);
	input logic [4:0] in;
	input logic update;
	input logic reset,clk;
	
	output logic [4:0] out;
	
	genvar i;
	
	generate	
	//uses a generate statment to create 4 1 bit registers to hold a 4 bit value
	//all 1bit registers share update,reset and clk
	//outputs to a 4bit output
		for(i = 0; i < 5; i++) begin: REG5b
			register EACHREG(in[i],out[i],update,reset,clk);
		end
	endgenerate
	
endmodule



module RegisterArray(in,out,update,reset,clk);//array of 32 64 bit registers
	input logic [63:0] in;
	input logic [31:0]update;
	input logic reset,clk;
	
	output logic [31:0][63:0] out;
		
		genvar i;
	
	generate	//generates 31 64 bit registers and stores output in a multidimential array
		for(i = 0; i < 31; i++) begin: EachReg
			Register64b REG(in,out[i],update[i],reset,clk);
			//each registers's update is linked from decoder to control when register updates
		end
	endgenerate
	Register64b zero(64'b0,out[31],1'b1,reset,clk);
	//creates zero register edge case where it is always a 64 bit zero
	
endmodule


module register_testbench();
	parameter CLOCK_PERIOD = 1000000;	
	logic in,update,reset,clk,out;
	
	register dut(in, out,update,reset,clk);
	initial begin	
		clk <= 0;	
		forever #(CLOCK_PERIOD/2) clk <= ~clk;	
	end

	initial begin
		reset<=1;@(posedge clk);
		@(posedge clk);
		@(posedge clk);
		in<=0;update<=0;reset<=1;@(posedge clk);
		@(posedge clk);
		@(posedge clk);
		in<=0;update<=0;reset<=0;@(posedge clk);
		@(posedge clk);
		in<=1;update<=0;@(posedge clk);
		in<=1;update<=0;@(posedge clk);
		in<=1;update<=0;@(posedge clk);
		in<=1;update<=1;@(posedge clk);
		@(posedge clk);
		@(posedge clk);
		update<=0;@(posedge clk);
		@(posedge clk);
		in<=0;update<=0;reset<=0;@(posedge clk);
		@(posedge clk);
		@(posedge clk);
		reset<=1;@(posedge clk);
		@(posedge clk);
		
	$stop;
	end

endmodule


module register64_testbench();

	logic [63:0] in;
	logic update;
	logic reset,clk;
	
	logic [63:0] out;

	parameter CLOCK_PERIOD = 1000000;	
	
	Register64b dut(in,out,update,reset,clk);
	
	initial begin	
		clk <= 0;	
		forever #(CLOCK_PERIOD/2) clk <= ~clk;	
	end
	initial begin
		reset <= 1;in <= 0;@(posedge clk);
		reset <= 0;in <= 0;@(posedge clk);
		in <= 9473;update <=1; @(posedge clk);
		update <=0; @(posedge clk);
		in <= 42742;@(posedge clk);
		@(posedge clk);
		@(posedge clk);
		@(posedge clk);
		@(posedge clk);
		@(posedge clk);
		@(posedge clk);
		update <=1; @(posedge clk);
		@(posedge clk);
		@(posedge clk);
		$stop;
		
	end

endmodule


module registerarray_testbench();
	logic [63:0] in;
	logic [31:0]update;
	logic reset,clk;
	
	logic [31:0][63:0] out;

	
	parameter CLOCK_PERIOD = 1000000;	
	
	RegisterArray dut(in,out,update,reset,clk);
	
	initial begin	
		clk <= 0;	
		forever #(CLOCK_PERIOD/2) clk <= ~clk;	
	end
	
	
	initial begin
		reset <= 1;in <= 0;@(posedge clk);
		reset <= 0;in <= 0;@(posedge clk);
		in <= 9473;update <=1; @(posedge clk);
		update <=0; @(posedge clk);
		in <= 42742;@(posedge clk);
		@(posedge clk);
		@(posedge clk);
		@(posedge clk);
		@(posedge clk);
		@(posedge clk);
		@(posedge clk);
		update <=32768;in<= 69; @(posedge clk);

		update <=512;in<= 239; @(posedge clk);
		update <=8192;in<= 1239; @(posedge clk);
		@(posedge clk);
		@(posedge clk);
		reset <= 1;in <= 0;@(posedge clk);
		@(posedge clk);
		@(posedge clk);
		$stop;
		
	end
	
	

endmodule


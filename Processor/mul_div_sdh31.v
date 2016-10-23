module mul_div_sdh31(data_operandA, data_operandB, ctrl_MULT, ctrl_DIV, clock, data_result, data_exception, data_inputRDY, data_resultRDY, multOperation, divOperation);

	input [31:0] data_operandA, data_operandB;
	input clock, ctrl_MULT, ctrl_DIV;
	wire [63:0] product;
	wire [31:0] data_result_temp;
	output data_resultRDY;
	output data_inputRDY;
	output data_exception;
	
	assign data_inputRDY = data_resultRDY;
	
	assign data_result_temp[31:0] = product[31:0];
	assign data_exception = divOperation & isBZero;
	
	wire isBZero;
	zeroEquivalent(.in(aluMultiplicandInput), .out(isBZero));
	
	
	// necessary to store upper bit of both data_operandA and data_operand B to know the sign of the integer
	
	wire signOfA;
	wire signOfAOutput;
	wire signOfB;
	wire signOfBOutput;
	
	DFFE (.clk(clock), .d(signOfA), .ena(1), .q(signOfAOutput));
	TriState1Bit(.in(data_operandA[31]), .enable(~clock & (ctrl_MULT | ctrl_DIV)), .out(signOfA));
	TriState1Bit(.in(signOfAOutput), .enable(clock | ~(ctrl_MULT | ctrl_DIV)), .out(signOfA));
	
	DFFE (.clk(clock), .d(signOfB), .ena(1), .q(signOfBOutput));
	TriState1Bit(.in(data_operandB[31]), .enable(~clock & (ctrl_MULT | ctrl_DIV)), .out(signOfB));
	TriState1Bit(.in(signOfBOutput), .enable(clock | ~(ctrl_MULT | ctrl_DIV)), .out(signOfB));
	
	// necessary to store ctrl_MULT and ctrl_DIV
	output multOperation;
	wire multOperationOutput;
	output divOperation;
	wire divOperationOutput;
		
		
	TriState1Bit mulTri(.in(1), .out(multOperation), .enable(ctrl_MULT));	
	TriState1Bit mulTri2(.in(multOperationOutput), .out(multOperation), .enable(~ctrl_MULT & ~data_resultRDY));
	TriState1Bit mulTri3(.in(0), .out(multOperation), .enable(data_resultRDY));
	DFFE mulDFFE(.d(multOperation), .q(multOperationOutput), .clk(clock), .ena(1), .clrn(low_clrn_mult), .prn(1));
		
	
	TriState1Bit divTri(.in(1), .out(divOperation), .enable(ctrl_DIV));	
	TriState1Bit divTri2(.in(divOperationOutput), .out(divOperation), .enable(~ctrl_DIV & ~data_resultRDY));
	TriState1Bit divTri3(.in(0), .out(divOperation), .enable(data_resultRDY));
	DFFE divDFFE(.d(divOperation), .q(divOperationOutput), .clk(clock), .ena(1), .clrn(low_clrn_div), .prn(1));
	
	wire low_clrn_div, low_clrn_mult;
	assign low_clrn_div = ~(ctrl_MULT & ~ctrl_DIV);
	assign low_clrn_mult = ~(ctrl_DIV & ~ctrl_MULT);
	
	//latch multLatch(.ena((ctrl_MULT | ctrl_DIV) & ~clock ), .d(ctrl_MULT), .q(multOperation));
	
	
	// ************************************************************************************************
	// aluOp specifies the op-code for the ALU. Addition needs 00000 while subtraction needs 00001.   *
	// ************************************************************************************************
	
	wire [4:0] aluOp;
	assign aluOp[0] = (multOperation & (prodCtrl[1] & ~prodCtrl[0])) | (divOperation & ~diffSign);
	assign aluOp[4:1] = 4'b0;
	
	wire diffSign;
	xor(diffSign, productToAlu[31], aluMultiplicandInput[31]);
	
	// ************************************************************************************************
	// Design to control the ALU properly. First, a DFFE (1 bit) is necessary as we must store
	// the bit that we shifted off the end of the product. The DFFE is enabled on a low clock edge.
	// This requires the output of the right shift to also return as an output the bit that is shifted
	// off.                                             
	// ************************************************************************************************
	wire [1:0] prodCtrl;
	
	TriState1Bit(.in(0), .enable(ctrl_MULT), .out(lowEnabledBitShiftedOffProduct));
	
	DFFE a_dffe1(.clk(clock), .d(lowEnabledBitShiftedOffProduct), .ena(1), .q(prodCtrl[0]));
	
	DFFE a_dffe2(.clk(clock), .d(product[0]), .ena(1), .q(prodCtrl[1]));
	
	// ************************************************************************************************
	// When the user specifies mult_enable, there multiplicand bits are specified. It is necessary to *
	// route them through a DFF in order to maintain the values of the multiplicand bits              *
	// ************************************************************************************************
	
	wire [31:0] multiplicandBufferOutput;
	DFF32Bit multiplicandBuffer(.clock(clock), .write_enable(1), .dataInput(aluMultiplicandInput), 
							 .dataOutput(multiplicandBufferOutput));
							 
	// tri states are used to initialize multiplicand into aluMultiplicandInput, then to continuously route
	// DFF back into aluMultiplicandInput						 
	TriState32Bit multiplicandBufferTRI(.in(multiplicandBufferOutput), .enable(clock | ~(ctrl_MULT | ctrl_DIV)), .out(aluMultiplicandInput));
	TriState32Bit initializeMultiplicandTRI(.in(data_operandB), .enable((ctrl_MULT | ctrl_DIV) & ~clock), .out(aluMultiplicandInput));
	
	// ************************************************************************************************
	// This section specifies the control of the Arithmetic Logic Unit.                               *
	// We must enable the multiplicand, otherwise it is set to all zeros (so no add/sub occurs).      *
	// Enabling occurs when our multiplier bits are 01 or 10                                          *
	// ************************************************************************************************

	wire multiplicandEnable;
	xor(multiplicandEnable, prodCtrl[0], prodCtrl[1]);
	
	wire [31:0] aluMultiplicandInput;
	wire [31:0] aluOutput; // output of alu
	
	
	alu_sdh31 ALU(.data_operandA(productToAlu), .data_operandB(aluMultiplicandInput & ({32{multiplicandEnable}} |{32{divOperation}})),
				  .ctrl_ALUopcode(aluOp), .data_result(aluOutput));
	
	// ************************************************************************************************
	// After addition or subtraction, the output of the ALU will have 16 lower order bits (17 on      *
	// overflow). These bits will be placed into the upper 16 bits of the new product through a       *
	// tristate (enabled on low clock and ~mult_enable). The lower order product bits will remain the *
	// same.                                                                                          *
	// ************************************************************************************************
	wire [63:0] aluToProductInput;
	
	wire divisionXOR;
	wire divisionXORHelper;
	wire zeroEquivalentOutput;
	
	xor(divisionXORHelper, aluOutput[31], productToAlu[31]);
	
	assign divisionXOR = divisionXORHelper & ~zeroEquivalentOutput;
	zeroEquivalent(.in(aluOutput), .out(zeroEquivalentOutput));
	
	TriState64Bit aluToProduct(.in(aluToProductInput), .enable(clock), .out(product));
	assign aluToProductInput[63:32]  = (aluOutput[31:0] & {32{~divisionXOR}} & {32{divOperation}}) |
	                                   (productToAlu[31:0] & {32{divisionXOR}} & {32{divOperation}}) | 
	                                   (aluOutput & {32{~divOperation}}); 
	assign aluToProductInput[31:1] = highEnabledProductOutput[31:1];
	assign aluToProductInput[0] = (multOperation & highEnabledProductOutput[0]) 
								| (divOperation & ~divisionXOR);
	
    // ************************************************************************************************
	// Initializes the product buffer to have the multiplier as the bits 16:1 bits, and zeros for     *
	// all other bits. The tristate is enabled on mult_enable and a low clock                         *
	// ************************************************************************************************
	
    wire [63:0] initializeProductInput;
	assign initializeProductInput[63:33] = ({31{~divOperation}} & 31'b0) | ({31{divOperation}} & {31{data_operandA[31]}});
	assign initializeProductInput[31:1] = ({31{divOperation}} & data_operandA[30:0]) | ({31{multOperation}} & data_operandA[31:1]);
	assign initializeProductInput[0] = (divOperation & 1'b0) | (multOperation & data_operandA[0]);
	assign initializeProductInput[32] = (divOperation & data_operandA[31]) | (multOperation & 1'b0);
	
	TriState64Bit initializeProductTristate(.in(initializeProductInput), .enable((ctrl_MULT | ctrl_DIV) & ~clock), .out(product));
	
    // ************************************************************************************************
	// Pipes the shifted product into a pos-edge triggered DFFE. This is then                         *
	// connected to the proudct through a tristate, which is enabled with a low clock and no mult_ena *                                   
	// ************************************************************************************************
	wire [63:0] dffShiftOutput;
	wire [63:0] productShiftedOutput;
	wire [63:0] productShiftedRightOutput;
	wire [63:0] productShiftedLeftOutput;  
	
	wire bitShiftedOffProduct;
	OneBitRightShifterArithmetic(.in(product), .out(productShiftedRightOutput), .enable(1), .shiftedOff(bitShiftedOffProduct));
	OneBitLeftShifterWithInput(.in(product), .out(productShiftedLeftOutput), .enable(1), .oneBitIn(0));
	
	TriState64Bit(.in(productShiftedRightOutput), .enable(multOperation), .out(productShiftedOutput));
	TriState64Bit(.in(productShiftedLeftOutput), .enable(~multOperation), .out(productShiftedOutput));
	
	DFF64Bit(.clock(~clock), .write_enable(1), .dataInput(productShiftedOutput), .dataOutput(dffShiftOutput));
	
	wire lowEnabledBitShiftedOffProduct;
	wire triStateInputToLowEnabledBitShiftedOffProduct;
	DFFE (.clk(~clock), .d(bitShiftedOffProduct), .ena(1), .q(triStateInputToLowEnabledBitShiftedOffProduct));
	TriState1Bit(.in(triStateInputToLowEnabledBitShiftedOffProduct), .enable(~ctrl_MULT), .out(lowEnabledBitShiftedOffProduct));
	TriState64Bit shiftDFFToOutput(.in(dffShiftOutput), .enable(~clock & ~(ctrl_MULT | ctrl_DIV)), .out(product));
		
	// ************************************************************************************************
	// Pipes the shifted product into a high-edge triggered DFFE. On a high edge, we can take the     *
	// product that was just shifted and pipe it into the ALU                                         *                                 
	// ************************************************************************************************
	wire [63:0] highEnabledProductOutput;	
	wire [63:0] productToAlu; // correct bits of product that will be routed into alu

	DFF64Bit(.clock(clock), .write_enable(1), .dataInput(product), .dataOutput(highEnabledProductOutput));
								
	// assigns the upper 16 bits of the product output to the lower 16 bits of dataA in the ALU,
	// as the algorithm (as specified by the 'Booth Hardware' circuit, only wants the upper 16 bits				
	assign productToAlu[31:0] = highEnabledProductOutput[63:32];
	

	
	alu_sdh31 twosCompALU(.data_operandA(32'b0), .data_operandB(data_result_temp),
				  .ctrl_ALUopcode(5'b00001), .data_result(twosCompALUOut));
	wire [31:0] twosCompALUOut;
				  
	TriState32Bit(.in(data_result_temp), .out(data_result), .enable(~divOperation | ~diffSignInput));
	TriState32Bit(.in(twosCompALUOut), .out(data_result), .enable(divOperation & diffSignInput));	  
	
	output [31:0] data_result;
	wire diffSignInput;
	xor(diffSignInput, signOfA, signOfB);
	
	// LOGIC FOR ASSERTING WHEN THE RESULT IS READY
	
	wire [63:0] assertBuffer;
	wire [63:0] assertInputToShifter;
	wire [63:0] assertOutputOfShifter;
	wire [63:0] assertOutputOfDFF;
	assign assertBuffer[30:0] = 31'b0;
	assign assertBuffer[32] = divOperation;
	assign assertBuffer[31] = multOperation;
	assign assertBuffer[63:33] = 31'b0;
	
	TriState64Bit(.in(assertBuffer), .out(assertInputToShifter), .enable(ctrl_MULT | ctrl_DIV));
	TriState64Bit(.in(assertOutputOfDFF), .out(assertInputToShifter), .enable(~(ctrl_MULT | ctrl_DIV)));
	
	OneBitLeftShifterWithInput leftShiftAssert(.in(assertInputToShifter), .out(assertOutputOfShifter), .enable(1), .oneBitIn(0));
	DFF64Bit(.clock(~clock), .write_enable(1), .dataInput(assertOutputOfShifter), .dataOutput(assertOutputOfDFF));
	
	assign data_resultRDY = assertInputToShifter[63]; // & ~clock; got rid of this, i think it will still work
			
endmodule


module DFF64Bit(clock, write_enable, dataInput, reset, dataOutput); // enabled on pos-edge clock?
	input clock, reset, write_enable;
	input [63:0] dataInput;
	output [63:0] dataOutput;
	
	genvar i;
	generate
	for (i=0; i<64; i=i+1) begin: loop1
		DFFE a_dffe(.clk(clock), .d(dataInput[i]), .clrn(~reset), .ena(write_enable), .q(dataOutput[i]));
	end
	endgenerate
endmodule

module OneBitRightShifterArithmetic(in, out, enable, shiftedOff);
	input [63:0] in;
	input enable;
	output shiftedOff;
	output [63:0] out;
	
	assign out[63] = in[63];
	
	genvar i;
	generate
		for (i = 62; i >= 0; i = i - 1) begin: loop1
			assign out[i] = (in[i+1] & enable) | (in[i] & ~enable);
		end
	endgenerate
	assign shiftedOff = in[0];
	
endmodule

module TriState64Bit(in, enable, out);
	input [63:0] in;
	input enable;
	output [63:0] out;
	
	assign out = enable ? in : 64'bz;
endmodule

module TriState1Bit(in, enable, out);
	input in;
	input enable;
	output out;
	
	assign out = enable ? in : 1'bz;
endmodule

module OneBitLeftShifterWithInput(in, out, enable, oneBitIn);
	input [63:0] in;
	input enable, oneBitIn;
	output [63:0] out;
	
	
	assign out[0] = (in[0] & ~enable) | (oneBitIn & enable);
	
	genvar i;
	generate
		for (i = 1; i <= 63; i = i + 1) begin: loop1
			assign out[i] = (in[i-1] & enable) | (in[i] & ~enable);
		end
	endgenerate
	
endmodule

module zeroEquivalent(in, out);
	input [31:0] in;
	output out;
	assign out = ~in[0] & ~in[1] &  ~in[2] &  ~in[3] &  ~in[4] &  ~in[5] & ~in[6] & ~in[7] &
				 ~in[8] & ~in[9] &  ~in[10] &  ~in[11] &  ~in[12] &  ~in[13] & ~in[14] & ~in[15] &
		         ~in[16] & ~in[17] &  ~in[18] &  ~in[19] &  ~in[20] &  ~in[21] & ~in[22] & ~in[23] &
		         ~in[24] & ~in[25] &  ~in[26] &  ~in[27] &  ~in[28] &  ~in[29] & ~in[30] & ~in[31];
endmodule
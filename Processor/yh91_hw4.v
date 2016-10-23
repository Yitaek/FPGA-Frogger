module yh91_hw4(data_operandA, data_operandB, ctrl_MULT, ctrl_DIV, clock,
				data_result, data_exception, data_inputRDY, data_resultRDY,
				multOperation, divOperation);
   input [31:0] data_operandA;
   input [31:0] data_operandB;
   input ctrl_MULT, ctrl_DIV, clock;             
   output [31:0] data_result; 
   output data_exception, data_inputRDY, data_resultRDY;
   
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
	
   // Connect the mult-operator 
   mult op_mult(.data_operandA(data_operandA), .data_operandB(data_operandB), .clock(clock), .ctrl_MULT(ctrl_MULT), .data_result(multResult), .data_resultRDY(mult_data_resultRDY), .data_inputRDY(mult_data_inputRDY));
   wire [31:0] multResult; 
   wire mult_data_resultRDY, mult_data_inputRDY; 
   
   //tristate32 multresultdisplay(.in(multResult), .oe(mult_data_resultRDY), .outVal(data_result));
   
   assign data_result = (multResult & {32{mult_data_resultRDY}}) | (divResult & {32{div_data_resultRDY}});
   
   //tristate1 multdataresultrdydisplay(.in(mult_data_resultRDY), .oe(mult_data_resultRDY), .outVal(data_resultRDY));
   tristate1 multdatainputrdydisplay(.in(mult_data_inputRDY), .oe(mult_data_inputRDY), .outVal(data_inputRDY));
   
   // Connect the div-operator
   div op_div(.data_operandA(data_operandA), .data_operandB(data_operandB), .ctrl_DIV(ctrl_DIV), .clock(clock), .data_quotient(divResult), .data_resultRDY(div_data_resultRDY), .data_exception(data_exception), .data_inputRDY(div_data_inputRDY));
   wire [31:0] divResult;
   wire div_data_resultRDY, div_data_inputRDY;
   
   //tristate32 divresultdisplay(.in(divResult), .oe(div_data_resultRDY), .outVal(data_result));
   
   assign data_resultRDY = div_data_resultRDY | mult_data_resultRDY; 
   //tristate1 divdataresultrdydisplay(.in(div_data_resultRDY), .oe(div_data_resultRDY), .outVal(data_resultRDY));
   tristate1 divdatainputrdydisplay(.in(div_data_inputRDY), .oe(div_data_inputRDY), .outVal(data_inputRDY));
   
endmodule

module div(data_operandA, data_operandB, ctrl_DIV, clock, data_quotient, data_remainder, data_resultRDY, data_exception, data_inputRDY);
   input [31:0] data_operandB, data_operandA;
   input ctrl_DIV, clock;
   output data_resultRDY, data_exception, data_inputRDY;         
   output [31:0] data_quotient; 
   output [31:0] data_remainder; 
   
     // Throw an exception when dividing by zero
     wire exception, dffexception;
   assign exception = ~(data_operandB[15] | data_operandB[14] | data_operandB[13] | data_operandB[12] | data_operandB[11] |
	data_operandB[10] | data_operandB[9] | data_operandB[8] | data_operandB[7] | data_operandB[6] | data_operandB[5] | data_operandB[4] |
	data_operandB[3] | data_operandB[2] | data_operandB[1] | data_operandB[0]);
	
	DFFE DFF_exception(.d(exception), .clk(clock), .q(dffexception), .ena(ctrl_DIV)); 
	tristate1 assertException(.in(dffexception), .oe(data_resultRDY), .outVal(data_exception));
   
   // 32 bit counter
   wire [31:0] initialCount;
   genvar n;
   generate
	for (n=1; n<32; n=n+1) begin: loop3
		assign initialCount[n] = 0; 
	end
   endgenerate
   
   assign initialCount[0] = 1;
   
   // Count 32 cycles using a 32bit register
   wire[31:0] counterMuxResult, countOutput, shiftedCount;
   Register32bit DFF_counter(.data_in(counterMuxResult), .clock(clock), .data_out(countOutput));
   oneSLL shiftcounter(.dataIn(countOutput), .dataOut(shiftedCount));
   thirtyTwoBitMux chooseCounter(.a(initialCount), .b(shiftedCount), .select(ctrl_DIV), .out(counterMuxResult));
   assign data_resultRDY = countOutput[31] & ~clock; 
   
   wire resultRDYHigh = countOutput[31] & clock; 
   
   // Next input is ready one clock cycle after data_resultRDY is high
   DFF inputready(.d(resultRDYHigh), .clk(clock), .q(data_inputRDY));

   // Find whether or not to take two's complement at the end
   wire negateAnswer, takeComplement; 
   assign negateAnswer = data_operandA[31] ^ data_operandB[15]; // The signs of dividend and divisor are different
   DFFE DFF_negate(.d(negateAnswer), .clk(clock), .q(takeComplement), .ena(ctrl_DIV)); // keeep the answer
   
   // Choose output the dividend when the result is ready and choose between negative or positive answer
   wire [31:0] dividendBeforeSign;
   tristate32 dividendSignChange(.in(dividendOut), .oe(data_resultRDY), .outVal(dividendBeforeSign));
   wire [31:0] finalAnswer, dividendBeforeSignNot;
   
   TwosComplement forFinalAnswer(.in(dividendBeforeSign), .out(dividendBeforeSignNot));
   thirtyTwoBitMux chooseSign(.a(dividendBeforeSignNot), .b(dividendBeforeSign), .select(takeComplement), .out(finalAnswer));
   
   // Set dividend and divisor
   wire [31:0] dividend, dividendBeforeLogic, dividendNot, dividendMuxResult;
   wire [31:0] divisor;
   wire [15:0] divisorBeforeLogic, divisorNot, divisorMuxResult;
   
   assign dividendBeforeLogic = data_operandA;
   assign divisorBeforeLogic = data_operandB[15:0];
   TwosComplement forDividend(.in(dividendBeforeLogic), .out(dividendNot));
   TwosComplement forDivisor(.in(divisorBeforeLogic), .out(divisorNot));
   
   thirtyTwoBitMux setDividend(.a(dividendNot), .b(dividendBeforeLogic), .select(data_operandA[31]), .out(dividendMuxResult));
   sixteenBitMux setDivisor(.a(divisorNot), .b(divisorBeforeLogic),. select(data_operandB[15]), .out(divisorMuxResult));
   
   assign dividend = dividendMuxResult;
   
   // Zeropad the divisor to make 32 bit number
   assign divisor[15:0] = divisorMuxResult;
   genvar i;
   generate
	for (i=16; i<32; i=i+1) begin: loop1
		assign divisor[i] = 0; 
	end
   endgenerate
   
   // Divisor is connected to the DFF
   wire [31:0] dffDivisor;
   Register32bitKeep DFF_divisor(.data_in(divisor), .clock(clock), .data_out(dffDivisor), .enable(ctrl_DIV));
   
   // Initialize the division block
   wire [63:0] initialDivisionValue;
   assign initialDivisionValue[31:0] = dividend;
   genvar x;
   generate
	for (x=32; x<64; x=x+1) begin: loop2
		assign initialDivisionValue[x] = 0; 
	end
   endgenerate
   
   // Initial Division Value is added on low clock and when div control is on
   wire [63:0] triDivisionInput;
   tristate64 initialInputDivision(.in(initialDivisionValue), .oe(~clock & ctrl_DIV), .outVal(triDivisionInput));
   
   wire [31:0] remainderOut;
   wire [31:0] dividendOut; 
   wire [63:0] divisionOutput, shiftedDivision, shiftedDInput;
   divisionBlock division_A(.divisionIn(triDivisionInput), .MSBDivision(remainderOut), .LSBDivision(dividendOut), .divisionOut(divisionOutput));
   
   assign data_quotient = finalAnswer;
   assign data_remainder = remainderOut;
   
   // Shifted Division Side
   oneSLL shiftDivision(.dataIn(divisionOutput), .dataOut(shiftedDivision));
   Register64bit DFF_shiftd(.data_in(shiftedDivision), .clock(clock), .data_out(shiftedDInput));
   tristate64 shiftedInputDivision(.in(shiftedDInput), .oe(clock), .outVal(triDivisionInput));
   
   // MSB Side
   wire [31:0] dffALURemainder, sumValD, diffValD;
   wire [31:0] dffDividendOut; 
   Register32bit DFF_DALUInput(.data_in(remainderOut), .clock(~clock), .data_out(dffALURemainder));
   Register32bit DFF_Dividend(.data_in(dividendOut), .clock(~clock), .data_out(dffDividendOut));
   addsub_tek ALUBlock_B(.a(dffALURemainder), .b(dffDivisor), .sum(sumValD), .diff(diffValD));
   
   // Check if remainder is negative or not and feed the result to the division block
   wire [63:0] remainderPositive, remainderNegative;
   assign remainderPositive[63:32] = diffValD;
   assign remainderPositive[31:1] = dffDividendOut[31:1];
   assign remainderPositive[0] = 1; 
   
   assign remainderNegative[63:32] = dffALURemainder;
   assign remainderNegative[31:1] = dffDividendOut[31:1]; 
   assign remainderNegative[0] = 0; 
   
   wire[63:0] nextDivisionInput; 
   sixtyFourBitMux chooseRemainder(.a(remainderNegative), .b(remainderPositive), .select(diffValD[31]), .out(nextDivisionInput));
   
   tristate64 ALUInputDivision(.in(nextDivisionInput), .oe(~clock & ~ctrl_DIV), .outVal(triDivisionInput));

endmodule

module mult(data_operandA, data_operandB, ctrl_MULT, clock, data_result, data_resultRDY, checkMtCand, checkMtplier, data_inputRDY);
   input [31:0] data_operandA;
   input [31:0] data_operandB;
   input ctrl_MULT, clock;         
   output [31:0] data_result; 
   output data_resultRDY, data_inputRDY;
   
   // Debug Outputs
   output [15:0] checkMtCand, checkMtplier;
   assign checkMtCand = multiplicand;
   assign checkMtplier = multiplier;
   
   // 16 bit counter
   wire [15:0] initialCount;
   genvar n;
   generate
	for (n=1; n<16; n=n+1) begin: loop3
		assign initialCount[n] = 0; 
	end
   endgenerate
   
   assign initialCount[0] = 1;
   
   // Count 16 cycles using a 16 bit register
   wire[15:0] counterMuxResult, countOutput, shiftedCount;
   Register16bit DFF_counter(.data_in(counterMuxResult), .clock(clock), .data_out(countOutput));
   oneSLLsixteenBit shiftcounter(.dataIn(countOutput), .dataOut(shiftedCount));
   sixteenBitMux chooseCounter(.a(initialCount), .b(shiftedCount), .select(ctrl_MULT), .out(counterMuxResult));
   assign data_resultRDY = countOutput[15] & ~clock; 
   
   wire resultRDYHigh = countOutput[15] & clock; 
   
   assign data_result = dataOutput;
  // tristate32 dividendSignChange(.in(dataOutput), .oe(data_resultRDY), .outVal(data_result));
   
   // Next input is ready one clock cycle after data_resultRDY is high
   DFF inputready(.d(resultRDYHigh), .clk(clock), .q(data_inputRDY));
   
   // Take the 16 LSB of the inputs
   wire [15:0] multiplicand, multiplier;
   assign multiplicand = data_operandA[15:0];
   assign multiplier = data_operandB[15:0];
   
   // Multiplicand is connected to the DFF
   wire [15:0] dffMtCand;
   Register16bitKeep DFF_mtCand(.data_in(multiplicand), .clock(clock), .data_out(dffMtCand), .enable(ctrl_MULT));
   
   // INITIALIZE THE PRODUCT BLOCK
   wire [31:0] initialProductValue;
	assign initialProductValue[15:0] = multiplier;
	genvar x;
	generate
	for (x=16; x<32; x=x+1) begin: loop1
		assign initialProductValue[x] = 0; 
	end
   endgenerate
   
   // Initial Product Value is added on low clock and when multplication control is on
   wire [31:0] triProductInput, dataOutput, shiftedProduct, shiftedInput; 
   wire [15:0] multiplierInputValues;
   tristate32 initialInputProduct(.in(initialProductValue), .oe(~clock & ctrl_MULT), .outVal(triProductInput));
   
   productBlock product_A(.productIn(triProductInput), .MSBProduct(ALUInputValues), .ALUDecBit(ALUBit), .productOut(dataOutput), .multiplierBits(multiplierInputValues));

   // Shifted Product Side
   oneSRA shiftProduct(.dataIn(dataOutput), .dataOut(shiftedProduct));
   Register32bit DFF_shift(.data_in(shiftedProduct), .clock(~clock), .data_out(shiftedInput));
   tristate32 shiftedInputProduct(.in(shiftedInput), .oe(~clock & ~ctrl_MULT), .outVal(triProductInput));
   
   // MSB Side
   wire [15:0] ALUInputValues, dffALUInput, multVals, sumVal, diffVal;
   Register16bit DFF_ALUInput(.data_in(ALUInputValues), .clock(clock), .data_out(dffALUInput));
   Register16bit DFF_multiplier(.data_in(multiplierInputValues), .clock(clock), .data_out(multVals));
   addsub16 ALUBlock_A (.a(dffALUInput), .b(zerosOrMtCand), .sum(sumVal), .diff(diffVal)); 
   
   // Control - Make ALU Decision 
   wire ALUBit, ALUOpcode, previousBit, DFFSaveBitInput, keepBit;
   DFF ALUSelect(.d(ALUBit), .clk(clock), .q(ALUOpcode));
   DFF saveLastBit(.d(ALUOpcode), .clk(clock), .q(previousBit));
   
   oneBitMux initialZero(.a(0), .b(previousBit), .select(~clock & ctrl_MULT), .out(keepBit));
   
   // choose between multiplicand or zero
   wire [15:0] zerosOrMtCand; 
   wire keepOrAdd;
   assign keepOrAdd = ALUOpcode ~^ keepBit; 
   
   sixteenBitMux decideKeep(.a(16'b0), .b(dffMtCand), .select(keepOrAdd), .out(zerosOrMtCand));
   
   // choose between sum and diff
   
   wire [15:0] MSBProductInput;
   sixteenBitMux addOrSub(.a(diffVal), .b(sumVal), .select(ALUOpcode), .out(MSBProductInput));
   
   // feed the added or subtracted MSB into the product block
   
   wire [31:0] ALUResult;
   assign ALUResult[31:16] = MSBProductInput;
   assign ALUResult[15:0] = multVals;
   tristate32 ALUInputProduct(.in(ALUResult), .oe(clock), .outVal(triProductInput));
   
endmodule

module Register16bitKeep (data_in, reset, clock, data_out, enable);

	input [15:0] data_in;
	input reset, clock, enable;
	output [15:0] data_out;
	
	genvar c;
	generate
	for (c=0; c<16; c=c+1) begin: loop1
		DFFE a_dff(.d(data_in[c]), .clk(clock), .q(data_out[c]), .clrn(!reset), .ena(enable));
		end
	endgenerate
	
endmodule

module Register16bit(data_in, reset, clock, data_out);

	input [15:0] data_in;
	input reset, clock;
	output [15:0] data_out;
	
	genvar c;
	generate
	for (c=0; c<16; c=c+1) begin: loop1
		DFF b_dff(.d(data_in[c]), .clk(clock), .q(data_out[c]), .clrn(!reset));
		end
	endgenerate
	
endmodule

module Register32bit(data_in, reset, clock, data_out, enable);

	input [31:0] data_in;
	input reset, clock, enable;
	output [31:0] data_out;
	
	genvar c;
	generate
	for (c=0; c<32; c=c+1) begin: loop1
		DFF c_dff(.d(data_in[c]), .clk(clock), .q(data_out[c]));
		end
	endgenerate
	
endmodule

module productBlock(productIn, MSBProduct, ALUDecBit, productOut, multiplierBits); 
	input[31:0] productIn;
	output[15:0] MSBProduct, multiplierBits;
	output[31:0] productOut;
	output ALUDecBit;
	
	// Pass on multiplier bits to ALU decision tree
	assign ALUDecBit = productIn[0];
	
	// Pass on the MSB value and multiplierBits
	assign MSBProduct = productIn[31:16];
	assign multiplierBits = productIn[15:0];
	assign productOut = productIn; 
	
endmodule

module oneSRA(dataIn, dataOut);

	input [31:0] dataIn;
	output [31:0] dataOut;

	assign dataOut[31] = dataIn[31];
	
	genvar c;
	generate
		for (c=30; c>=0; c = c-1) begin: loop1
			assign dataOut[c] = dataIn[c+1];
		end
	endgenerate
endmodule
   
module sixteenBitMux(a, b, select, out);
	input select;
	input [15:0] a, b;
	output [15:0] out;
	
	assign out = ({16{select}} & a) | ({16{!select}} & b);
endmodule

module oneBitMux (a, b, select, out);
	input select;
	input a, b;
	output out;
	
	assign out = (select & a) | (!select & b);
endmodule

module addsub16(a, b, sum, diff);
	input [15:0] a, b;
	output [15:0] sum, diff;
	
	CLA16 CLA_sum(.a(a), .b(b), .c0(0), .sum(sum));
	CLA16 CLA_diff(.a(a), .b(~b), .c0(1), .sum(diff));
	
endmodule

module CLA16(a, b, c0, sum);
	input [15:0] a, b;
	input c0;
	output [15:0] sum;
	wire [7:0] P0, G0, P1, G1;
	wire c8, c16;
	
	// connect the second level CLA blocks together
	CLA8bit_tek block0(.a(a[7:0]), .b(b[7:0]), .c(c0), .P(P0), .G(G0), .s(sum[7:0]));
	assign c8 = G0 | P0&c0;
	CLA8bit_tek block1(.a(a[15:8]), .b(b[15:8]), .c(c8), .P(P1), .G(G1), .s(sum[15:8]));
	assign c16 = G1 | P1&G0 | P1&P0&c0;
	
endmodule

module CLA8bit_tek(a, b, c, P, G, s);

	input [7:0] a, b;
	input c;
	output [7:0] P, G, s;
	
	// Generate the propagate bits
	wire [7:0] p;
	assign p = a | b;
	
	assign P = p[7]&p[6]&p[5]&p[4]&p[3]&p[2]&p[1]&p[0];
	
	// Generate the generate bits
	wire [7:0] g;
	assign g = a & b;
	
	assign G = g[7] | p[7]&g[6] | p[7]&p[6]&g[5] | p[7]&p[6]&p[5]&g[4] | p[7]&p[6]&p[5]&p[4]&g[3] | 
		p[7]&p[6]&p[5]&p[4]&p[3]&g[2] | p[7]&p[6]&p[5]&p[4]&p[3]&p[2]&g[1] |  p[7]&p[6]&p[5]&p[4]&p[3]&p[2]&p[1]&g[0];
	
	// Generate the sum bits
	wire[7:0] carryBit;
	assign carryBit[0] = c;
	
	genvar i;
	generate
		for (i=0; i<=6; i = i + 1) begin: loop1
			assign carryBit[i+1] = g[i] | (p[i]&carryBit[i]);
		end
	endgenerate
	
	assign s = p^g^carryBit;
	
endmodule

module tristate32(in, oe, outVal);
	input [31:0] in;
	input oe;
	output [31:0] outVal;

	assign outVal = oe ? in : 32'bz;
endmodule

module oneSLLsixteenBit(dataIn, dataOut);

	input [15:0] dataIn;
	output [15:0] dataOut;
	
	assign dataOut[0] = 0;
	
	genvar c;
	generate
		for (c=1; c<16; c = c+1) begin: loop1
			assign dataOut[c] = dataIn[c-1];
		end
	endgenerate
endmodule

module tristate1(in, oe, outVal);
	input in;
	input oe;
	output outVal;

	assign outVal = oe ? in : 1'bz;
endmodule

module tristate64(in, oe, outVal);
	input [63:0] in;
	input oe;
	output [63:0] outVal;

	assign outVal = oe ? in : 64'bz;
endmodule

module addsub_tek(a, b, sum, diff);
	input [31:0] a, b;
	output [31:0] sum, diff;
	
	CLA_tek CLA_sum(.a(a), .b(b), .c0(1'b0), .sum(sum));
	CLA_tek CLA_diff(.a(a), .b(~b), .c0(1'b1), .sum(diff));
	
endmodule

module CLA_tek(a, b, c0, sum);
	input [31:0] a, b;
	input c0;
	output [31:0] sum;
	wire [7:0] P0, G0, P1, G1, P2, G2, P3, G3;
	wire c8, c16, c24, c32;
	
	// connect the second level CLA blocks together
	CLA8bit_tek block0(.a(a[7:0]), .b(b[7:0]), .c(c0), .P(P0), .G(G0), .s(sum[7:0]));
	assign c8 = G0 | P0&c0;
	CLA8bit_tek block1(.a(a[15:8]), .b(b[15:8]), .c(c8), .P(P1), .G(G1), .s(sum[15:8]));
	assign c16 = G1 | P1&G0 | P1&P0&c0;
	CLA8bit_tek block2(.a(a[23:16]), .b(b[23:16]), .c(c16), .P(P2), .G(G2), .s(sum[23:16]));
	assign c24 = G2 | P2&G1 | P2&P1&G0 | P2&P1&P0&c0;
	CLA8bit_tek block3(.a(a[31:24]), .b(b[31:24]), .c(c24), .P(P3), .G(G3), .s(sum[31:24]));
	assign c32 = G3 | P3&G2 | P3&P2&G1 | P3&P2&P1&G0 | P3&P2&P1&P0&c0;
	
endmodule

module sixtyFourBitMux(a, b, select, out);
	input select;
	input [63:0] a, b;
	output [63:0] out;
	
	assign out = ({64{select}} & a) | ({64{!select}} & b);
endmodule

module thirtyTwoBitMux(a, b, select, out);
	input select;
	input [31:0] a, b;
	output [31:0] out;
	
	assign out = ({32{select}} & a) | ({32{!select}} & b);
endmodule

module divisionBlock(divisionIn, MSBDivision, LSBDivision, divisionOut); 
	input[63:0] divisionIn;
	output[31:0] MSBDivision;
	output[31:0] LSBDivision;
	output[63:0] divisionOut;

	// Pass on the MSB value and multiplierBits
	assign MSBDivision = divisionIn[63:32];
	assign LSBDivision = divisionIn[31:0];
	assign divisionOut = divisionIn;
	
endmodule

module oneSLL(dataIn, dataOut);

	input [63:0] dataIn;
	output [63:0] dataOut;
	
	assign dataOut[0] = 0;
	
	genvar c;
	generate
		for (c=1; c<64; c = c+1) begin: loop1
			assign dataOut[c] = dataIn[c-1];
		end
	endgenerate
endmodule

module Register64bit(data_in, reset, clock, data_out, enable);

	input [63:0] data_in;
	input reset, clock, enable;
	output [63:0] data_out;
	
	genvar c;
	generate
	for (c=0; c<64; c=c+1) begin: loop1
		DFF c_dff(.d(data_in[c]), .clk(clock), .q(data_out[c]));
		end
	endgenerate
	
endmodule

module TwosComplement (in, out);
	input [31:0] in;
	output [31:0] out;
	
	wire [31:0] thirtytwobitOne;
	
    genvar n;
    generate
	 for (n=1; n<32; n=n+1) begin: loop4
	 	assign thirtytwobitOne[n] = 0; 
	 end
    endgenerate
   
    assign thirtytwobitOne[0] = 1;
	
	addsub_tek addOne(.a(~in), .b(thirtytwobitOne), .sum(out));
	
endmodule
	

module Register32bitKeep (data_in, reset, clock, data_out, enable);

	input [31:0] data_in;
	input reset, clock, enable;
	output [31:0] data_out;
	
	genvar c;
	generate
	for (c=0; c<32; c=c+1) begin: loop1
		DFFE a_dff(.d(data_in[c]), .clk(clock), .q(data_out[c]), .clrn(!reset), .ena(enable));
		end
	endgenerate
	
endmodule
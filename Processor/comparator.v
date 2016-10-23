module comparator(data_operandA, data_operandB, isEqual, isLessThan, isGreaterThan);
   input [31:0] data_operandA, data_operandB;
   output isEqual, isLessThan, isGreaterThan;
      
   // Do all the ALU operations
   addsub a_addsub(.a(data_operandA), .b(data_operandB), .diff(data_diff));
   wire [31:0] data_diff;
   
   assign isEqual = ~(data_diff[31] | data_diff[30] | data_diff[29] | data_diff[28] | data_diff[27] | data_diff[26] |
	data_diff[25] | data_diff[24] | data_diff[23] | data_diff[22] | data_diff[21] | data_diff[20] | | data_diff[19] |
	data_diff[18] | data_diff[17] | data_diff[16] | data_diff[15] | data_diff[14] | data_diff[13] | | data_diff[12] |
	data_diff[11] | data_diff[10] | data_diff[9] | data_diff[8] | data_diff[7] | data_diff[6] | | data_diff[5] |
	data_diff[4] | data_diff[3] | data_diff[2] | data_diff[1] | | data_diff[0]);
  
   assign isLessThan = data_diff[31] & 1; 
   
   assign isGreaterThan = ~isEqual & ~isLessThan;
 
   
endmodule

module addsub(a, b, sum, diff);
	input [31:0] a, b;
	output [31:0] sum, diff;
	
	CLA CLA_diff(.a(a), .b(~b), .c0(1), .sum(diff));
	
endmodule

module CLA(a, b, c0, sum);
	input [31:0] a, b;
	input c0;
	output [31:0] sum;
	wire [7:0] P0, G0, P1, G1, G2, G3, P2, P3;
	wire c8, c16, c24, c32;
	
	// connect the second level CLA blocks together
	CLA8bit block0(.a(a[7:0]), .b(b[7:0]), .c(c0), .P(P0), .G(G0), .s(sum[7:0]));
	assign c8 = G0 | P0&c0;
	CLA8bit block1(.a(a[15:8]), .b(b[15:8]), .c(c8), .P(P1), .G(G1), .s(sum[15:8]));
	assign c16 = G1 | P1&G0 | P1&P0&c0;
	CLA8bit block2(.a(a[23:16]), .b(b[23:16]), .c(c16), .P(P2), .G(G2), .s(sum[23:16]));
	assign c24 = G2 | P2&G1 | P2&P1&G0 | P2&P1&P0&c0;
	CLA8bit block3(.a(a[31:24]), .b(b[31:24]), .c(c24), .P(P3), .G(G3), .s(sum[31:24]));
	assign c32 = G3 | P3&G2 | P3&P2&G1 | P3&P2&P1&G0 | P3&P2&P1&P0&c0;
	
endmodule

module CLA8bit(a, b, c, P, G, s);
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

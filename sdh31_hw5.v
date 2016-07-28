module sdh31_hw5(	inclock, resetn, ps2_clock, ps2_data, debug_word, debug_addr, leds, 
					lcd_data, lcd_rw, lcd_en, lcd_rs, lcd_on, lcd_blon, 	
					seg1, seg2, seg3, seg4, seg5, seg6, seg7, seg8,
					hsync, vsync, R, G, B, vga_clock, blank, sync);

	input 			inclock, resetn;
	inout 			ps2_data, ps2_clock;
	
	output 			lcd_rw, lcd_en, lcd_rs, lcd_on, lcd_blon;
	output 	[7:0] 	leds, lcd_data;
	output 	[6:0] 	seg1, seg2, seg3, seg4, seg5, seg6, seg7, seg8;
	output 	[31:0] 	debug_word;
	output  [11:0]  debug_addr;
	
	wire			clock;
	wire			lcd_write_en;
	wire 	[31:0]	lcd_write_data;
	wire	[7:0]	ps2_key_data;
	wire			ps2_key_pressed;
	wire	[7:0]	ps2_out;
	
	wire [31:0] registerOneOutput, registerTwoOutput, registerThreeOutput,
					registerFourOutput, registerFiveOutput, registerSixOutput, registerSevenOutput,
				   registerTenOutput, register11Output;
	wire [31:0] lives;
	
	// clock divider (by 5, i.e., 10 MHz)
	//wire clock_25;
	//pll div(inclock,vga_clock);
	
	// UNCOMMENT FOLLOWING LINE AND COMMENT ABOVE LINE TO RUN AT 50 MHz
	assign clock = inclock;
	
		// your processor

			
	processor myprocessor(.clock(vga_clock), .reset(~resetn), .ps2_key_pressed(ps2_key_pressed), .ps2_out(ps2_out),
						  .lcd_write(lcd_write_en), .lcd_data(lcd_write_data),
						  .registerOneOutput(registerOneOutput), .registerTwoOutput(registerTwoOutput),
						  .registerThreeOutput(registerThreeOutput), .registerFourOutput(registerFourOutput),
						  .registerFiveOutput(registerFiveOutput), .registerSixOutput(registerSixOutput),
						  .registerSevenOutput(registerSevenOutput),
						  .registerTenOutput(registerTenOutput), .register11Output(register11Output),
						  .collision(collision),
						  .endGameCollision(endGameCollision),
						  .frogInsideLog(frogInsideLog), 
						  .lives(lives));
			
	// keyboard controller
	PS2_Interface myps2(vga_clock, resetn, ps2_clock, ps2_data, ps2_key_data, ps2_key_pressed, ps2_out);
	
	// lcd controller
	lcd mylcd(clock, ~resetn, lcd_write_en, lcd_write_data[7:0], lcd_data, lcd_rw, lcd_en, lcd_rs, lcd_on, lcd_blon);
	
	// example for sending ps2 data to the first two seven segment displays
	//Hexadecimal_To_Seven_Segment hex1(winSegment4, seg1);
	//Hexadecimal_To_Seven_Segment hex2(winSegment3, seg2);
	
	// the other seven segment displays are currently set to 0
	//Hexadecimal_To_Seven_Segment hex3(winSegment2, seg3);
	//Hexadecimal_To_Seven_Segment hex4(winSegment1, seg4);
	Hexadecimal_To_Seven_Segment hex5(4'b0, seg5);
	Hexadecimal_To_Seven_Segment hex6(4'b0, seg6);
	Hexadecimal_To_Seven_Segment hex7(lives[3:0], seg7);
	Hexadecimal_To_Seven_Segment hex8(lives[7:4], seg8);
	
	wire loseEnable = registerTenOutput[1];
	wire winEnable = registerTenOutput[0];
	
	wire [6:0] winSegment1 = (7'b1000001 & {7{winEnable}}) | (7'b1111001 & {7{loseEnable}});
	wire [6:0] winSegment2 = (7'b1000001 & {7{winEnable}}) | (7'b1000000 & {7{loseEnable}});
	wire [6:0] winSegment3 = (7'b1111001 & {7{winEnable}}) | (7'b0010010 & {7{loseEnable}});
	wire [6:0] winSegment4 = (7'b1001000 & {7{winEnable}}) | (7'b0000110 & {7{loseEnable}});
	
	assign seg1 = winSegment4;
	assign seg2 = winSegment3;
	assign seg3 = winSegment2;
	assign seg4 = winSegment1;
	
	
	output hsync, vsync, vga_clock, blank, sync;
	output [7:0] R, G, B;

	display_sdh31(.FPGA_clock(clock), .vga_clock(vga_clock), .R(R), .G(G), .B(B), .vga_hs_sync(hsync),
				.vga_vs_sync(vsync), .vga_blank(blank), .vga_sync(sync),
			   .regOne(registerOneOutput),
				.regTwo(registerTwoOutput),
				.regThree(registerThreeOutput), 
				.regFour(registerFourOutput),
				.regFive(registerFiveOutput),
				.regSix(registerSixOutput),
				.regSeven(registerSevenOutput),
				.reg11(register11Output),
				.collision(collision),
				.frogInsideLog(frogInsideLog),
				.endGameCollision(endGameCollision));

	wire collision, endGameCollision;
	
	assign leds = registerOneOutput[7:0];
	
endmodule

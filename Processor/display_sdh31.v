module display_sdh31(FPGA_clock, vga_clock, R, G, B, vga_hs_sync,
				vga_vs_sync, vga_blank, vga_sync,
				regOne, regTwo, regThree, regFour, regFive, regSix, regSeven, reg11, collision,
				endGameCollision, frogInsideLog);//, LED_vga_hs_ON, LED_vga_vs_on);
	input FPGA_clock;
	output collision, endGameCollision;
	
	// registers coming directly from regfile of processor
	input [31:0] regOne, regTwo, regThree, regFour, regFive, regSix, regSeven, reg11;
	
	output vga_clock;
	output vga_hs_sync, vga_vs_sync, vga_blank, vga_sync;
	output[7:0] R, G, B;

	
	reg[31:0] counterX; 
	reg[31:0] counterY; 
	wire counterXmaxed, counterYMaxed;
	
	reg vga_hs, vga_vs;
	
	// clock
	clksrc get_VGAclk(FPGA_clock, vga_clock);
	
		
	wire endGame = (counterY > end_top_pos) && (counterY < end_bottom_pos) && (counterX > end_left_pos) && (counterX < end_right_pos);
	
	wire frog = (counterY > frog_top_pos) && (counterY < frog_bottom_pos) && (counterX > frog_left_pos) && (counterX < frog_right_pos);
	
	wire enemy1 = (counterY > enemy1_top_pos) & (counterY < enemy1_bottom_pos) & 
	              ((counterX > enemy1_left_pos & counterX < enemy1_right_pos) |
					  (counterX > enemy1_left_pos_wave1 & counterX < enemy1_right_pos_wave1) |
					  (counterX > enemy1_left_pos_wave2 & counterX < enemy1_right_pos_wave2));
				  
	wire enemy2 = (counterY > enemy2_top_pos) && (counterY < enemy2_bottom_pos) && (counterX > enemy2_left_pos) && (counterX < enemy2_right_pos);
	
	wire enemy3 = (counterY > enemy3_top_pos) & (counterY < enemy3_bottom_pos) & 
	              ((counterX > enemy3_left_pos & counterX < enemy3_right_pos) |
					  (counterX > enemy3_left_pos_wave1 & counterX < enemy3_right_pos_wave1) |
					  (counterX > enemy3_left_pos_wave2 & counterX < enemy3_right_pos_wave2));
					  
	wire log = (counterY > log_top_pos) && (counterY < log_bottom_pos) && (counterX > log_left_pos) && (counterX < log_right_pos);
	
	wire lake1 = (counterY > lake1_top_pos) && (counterY < lake1_bottom_pos) && (counterX > lake1_left_pos) && (counterX < lake1_right_pos);

	
	// enemy 1 wave
	wire [31:0] enemy1_left_pos_wave1, enemy1_right_pos_wave1, enemy1_left_pos_wave2, enemy1_right_pos_wave2;
	wire [31:0] enemy1_wave_offset = 32'b00000000000000000000000010111000;
	CarryLookaheadAdder2(.x(enemy1_left_pos), .y(enemy1_wave_offset), .sum(enemy1_left_pos_wave1), .subtractEnable(1'b1));
	CarryLookaheadAdder2(.x(enemy1_right_pos), .y(enemy1_wave_offset), .sum(enemy1_right_pos_wave1), .subtractEnable(1'b1));
	CarryLookaheadAdder2(.x(enemy1_left_pos_wave1), .y(enemy1_wave_offset), .sum(enemy1_left_pos_wave2), .subtractEnable(1'b1));
	CarryLookaheadAdder2(.x(enemy1_right_pos_wave1), .y(enemy1_wave_offset), .sum(enemy1_right_pos_wave2), .subtractEnable(1'b1));
	
	
	// enemy 3 wave
	wire [31:0] enemy3_left_pos_wave1, enemy3_right_pos_wave1, enemy3_left_pos_wave2, enemy3_right_pos_wave2;
	wire [31:0] enemy3_wave_offset = 32'b00000000000000000000000000111000;
	CarryLookaheadAdder2(.x(enemy3_left_pos), .y(enemy3_wave_offset), .sum(enemy3_left_pos_wave1), .subtractEnable(1'b1));
	CarryLookaheadAdder2(.x(enemy3_right_pos), .y(enemy3_wave_offset), .sum(enemy3_right_pos_wave1), .subtractEnable(1'b1));
	CarryLookaheadAdder2(.x(enemy3_left_pos_wave1), .y(enemy3_wave_offset), .sum(enemy3_left_pos_wave2), .subtractEnable(1'b1));
	CarryLookaheadAdder2(.x(enemy3_right_pos_wave1), .y(enemy3_wave_offset), .sum(enemy3_right_pos_wave2), .subtractEnable(1'b1));
	
	
	// collisions
	// frog enemy2
	isCollision(.x_left(frog_left_pos), .x_right(frog_right_pos), .x_top(frog_top_pos), .x_bottom(frog_bottom_pos),
					.y_left(enemy2_left_pos), .y_right(enemy2_right_pos), .y_top(enemy2_top_pos), .y_bottom(enemy2_bottom_pos),
					.out(collision_frog_enemy2));
	
	// frog enemy1 wave
	isCollision(.x_left(frog_left_pos), .x_right(frog_right_pos), .x_top(frog_top_pos), .x_bottom(frog_bottom_pos),
					.y_left(enemy1_left_pos), .y_right(enemy1_right_pos), .y_top(enemy1_top_pos), .y_bottom(enemy1_bottom_pos),
					.out(collision_frog_enemy1));
					
	isCollision(.x_left(frog_left_pos), .x_right(frog_right_pos), .x_top(frog_top_pos), .x_bottom(frog_bottom_pos),
					.y_left(enemy1_left_pos_wave1), .y_right(enemy1_right_pos_wave1), .y_top(enemy1_top_pos), .y_bottom(enemy1_bottom_pos),
					.out(collision_frog_enemy1_wave1));
					
	isCollision(.x_left(frog_left_pos), .x_right(frog_right_pos), .x_top(frog_top_pos), .x_bottom(frog_bottom_pos),
					.y_left(enemy1_left_pos_wave2), .y_right(enemy1_right_pos_wave2), .y_top(enemy1_top_pos), .y_bottom(enemy1_bottom_pos),
					.out(collision_frog_enemy1_wave2));
					
	// enemy3			
	isCollision(.x_left(frog_left_pos), .x_right(frog_right_pos), .x_top(frog_top_pos), .x_bottom(frog_bottom_pos),
					.y_left(enemy3_left_pos), .y_right(enemy3_right_pos), .y_top(enemy3_top_pos), .y_bottom(enemy3_bottom_pos),
					.out(collision_frog_enemy3));
					
	isCollision(.x_left(frog_left_pos), .x_right(frog_right_pos), .x_top(frog_top_pos), .x_bottom(frog_bottom_pos),
					.y_left(enemy3_left_pos_wave1), .y_right(enemy3_right_pos_wave1), .y_top(enemy3_top_pos), .y_bottom(enemy3_bottom_pos),
					.out(collision_frog_enemy3_wave1));
					
	isCollision(.x_left(frog_left_pos), .x_right(frog_right_pos), .x_top(frog_top_pos), .x_bottom(frog_bottom_pos),
					.y_left(enemy3_left_pos_wave2), .y_right(enemy3_right_pos_wave2), .y_top(enemy3_top_pos), .y_bottom(enemy3_bottom_pos),
					.out(collision_frog_enemy3_wave2));
					
	isWithin(.x_left(frog_left_pos), .x_right(frog_right_pos), .x_top(frog_top_pos), .x_bottom(frog_bottom_pos),
					.y_left(end_left_pos), .y_right(end_right_pos), .y_top(end_top_pos), .y_bottom(end_bottom_pos),
					.out(endGameCollision));		
	
	
	wire lake1Collision;
	isCollision(.x_left(frog_left_pos), .x_right(frog_right_pos), .x_top(frog_top_pos), .x_bottom(frog_bottom_pos),
					.y_left(lake1_left_pos), .y_right(lake1_right_pos), .y_top(lake1_top_pos), .y_bottom(lake1_bottom_pos),
					.out(lake1Collision));
	

	output frogInsideLog;
	isWithin(.x_left(frog_left_pos), .x_right(frog_right_pos), .x_top(frog_top_pos), .x_bottom(frog_bottom_pos),
					.y_left(log_left_pos), .y_right(log_right_pos), .y_top(log_top_pos), .y_bottom(log_bottom_pos),
					.out(frogInsideLog));

	
	wire collision_frog_enemy2, collision_frog_enemy1, collision_frog_enemy1_wave1, collision_frog_enemy1_wave2,
		  collision_frog_enemy3, collision_frog_enemy3_wave1, collision_frog_enemy3_wave2;
	
	assign collision = collision_frog_enemy2 | collision_frog_enemy1 | collision_frog_enemy1_wave1 | 
	                   collision_frog_enemy1_wave2 | collision_frog_enemy3 | collision_frog_enemy3_wave1 |
							 collision_frog_enemy3_wave2 | (lake1Collision & ~frogInsideLog);
	
	assign R = {8{enemy1 | endGame}} | (8'b01100110 & {8{log}});
	assign G = {8{frog | endGame | enemy3}} | (8'b00110011 & {8{log}});
	assign B = {8{enemy2 | enemy3 | lake1}};

	assign counterXmaxed = (counterX == 767);
	assign counterYmaxed = (counterY == 9'b111111111);
	
	always @(posedge vga_clock) begin
		if(counterXmaxed) 
			begin
				counterX = 0;
				if (counterYmaxed)
				begin
					counterY = 0;
				end
				else
					counterY = counterY + 32'b00000000000000000000000000000001; 
			end
		else
			counterX = counterX + 32'b00000000000000000000000000000001; 
	end

	always @(posedge vga_clock) begin
		vga_hs = (counterX[9:4] == 0); 
		vga_vs = (counterY==0); 
	end

	assign vga_hs_sync = ~vga_hs;
	assign vga_vs_sync = ~vga_vs;
	
	assign vga_blank = vga_hs_sync & vga_vs_sync;
	assign vga_sync = vga_blank;
	

	// frogger logic
	wire [31:0] frog_left_pos, frog_right_pos, frog_top_pos, frog_bottom_pos;
	getFroggerPosition(.register_x(regOne), .register_y(regTwo), .left_pos(frog_left_pos),
							 .right_pos(frog_right_pos), .top_pos(frog_top_pos), .bottom_pos(frog_bottom_pos));
							 
	// enemy1 logic						 
	wire [31:0] enemy1_left_pos, enemy1_right_pos, enemy1_top_pos, enemy1_bottom_pos;
	getEnemyOnePosition(.register_x(regThree), .left_pos(enemy1_left_pos),
							  .right_pos(enemy1_right_pos), .top_pos(enemy1_top_pos),
							  .bottom_pos(enemy1_bottom_pos));
							  
	wire [31:0] enemy2_left_pos, enemy2_right_pos, enemy2_top_pos, enemy2_bottom_pos;
	getEnemyTwoPosition(.register_x(regFour), .left_pos(enemy2_left_pos),
							  .right_pos(enemy2_right_pos), .top_pos(enemy2_top_pos),
							  .bottom_pos(enemy2_bottom_pos));
							  
							  
	wire [31:0] enemy3_left_pos, enemy3_right_pos, enemy3_top_pos, enemy3_bottom_pos;
	getEnemyThreePosition(.register_x(regFive), .left_pos(enemy3_left_pos),
							  .right_pos(enemy3_right_pos), .top_pos(enemy3_top_pos),
							  .bottom_pos(enemy3_bottom_pos));
	
	wire [31:0] log_left_pos, log_right_pos, log_top_pos, log_bottom_pos;
	getLogPosition(.register_x(regSix), .left_pos(log_left_pos), .right_pos(log_right_pos), .top_pos(log_top_pos), .bottom_pos(log_bottom_pos));
	
	wire [31:0] lake1_left_pos, lake1_right_pos, lake1_top_pos, lake1_bottom_pos;
	getLake1(.register_x(regSeven), .left_pos(lake1_left_pos), .right_pos(lake1_right_pos), .top_pos(lake1_top_pos), .bottom_pos(lake1_bottom_pos));
	
	wire [31:0] end_left_pos, end_right_pos, end_top_pos, end_bottom_pos;						  
	getEndGame(.register_x(reg11), .left_pos(end_left_pos), .right_pos(end_right_pos), .top_pos(end_top_pos), .bottom_pos(end_bottom_pos));
	
	
endmodule


module isCollision(x_left, x_right, x_top, x_bottom, y_left, y_right, y_top, y_bottom, out);
	input [31:0] x_left, x_right, x_top, x_bottom, y_left, y_right, y_top, y_bottom;
	output out;
	
	wire x_left_gt_y_right;
	comparator(.data_operandA(x_left), .data_operandB(y_right), .isEqual(), .isLessThan(), .isGreaterThan(x_left_gt_y_right));
	wire x_right_lt_y_left;
	comparator(.data_operandA(x_right), .data_operandB(y_left), .isEqual(), .isLessThan(x_right_lt_y_left), .isGreaterThan());
	wire x_bottom_lt_y_top;
	comparator(.data_operandA(x_bottom), .data_operandB(y_top), .isEqual(), .isLessThan(x_bottom_lt_y_top), .isGreaterThan());
	wire x_top_gt_y_bottom;
	comparator(.data_operandA(x_top), .data_operandB(y_bottom), .isEqual(), .isLessThan(), .isGreaterThan(x_top_gt_y_bottom));
	
	assign out = ~(
	x_left_gt_y_right |  // x leftmost edge further right than y rightmost edge
	x_right_lt_y_left | // x rightmost edge further left than y leftmost edge
	x_bottom_lt_y_top |// x bottom edge higher right than y top edge
	x_top_gt_y_bottom); // x top edge lower than y bottom edge
	
endmodule

// x within y
module isWithin(x_left, x_right, x_top, x_bottom, y_left, y_right, y_top, y_bottom, out);
	input [31:0] x_left, x_right, x_top, x_bottom, y_left, y_right, y_top, y_bottom;
	output out;
	
	wire x_left_gt_y_left;
	comparator(.data_operandA(x_left), .data_operandB(y_left), .isEqual(), .isLessThan(), .isGreaterThan(x_left_gt_y_left));
	wire x_right_lt_y_right;
	comparator(.data_operandA(x_right), .data_operandB(y_right), .isEqual(), .isLessThan(x_right_lt_y_right), .isGreaterThan());
	wire x_bottom_lt_y_bottom;
	comparator(.data_operandA(x_bottom), .data_operandB(y_bottom), .isEqual(), .isLessThan(x_bottom_lt_y_bottom), .isGreaterThan());
	wire x_top_gt_y_top;
	comparator(.data_operandA(x_top), .data_operandB(y_top), .isEqual(), .isLessThan(), .isGreaterThan(x_top_gt_y_top));
	
	assign out = x_left_gt_y_left & x_right_lt_y_right & x_bottom_lt_y_bottom & x_top_gt_y_top;
	//assign out = (x_left > y_left) & (x_right < y_right) & (x_top > y_top) & (x_bottom < y_bottom);
	
endmodule


module getFroggerPosition(register_x, register_y, left_pos, right_pos, top_pos, bottom_pos);
	input [31:0] register_x, register_y; // x and y positions stored in some register
	output [31:0] left_pos, right_pos, top_pos, bottom_pos;
	
	
	wire [31:0] frogger_width =  32'b00000000000000000000000000001101;
	wire [31:0] frogger_height = 32'b00000000000000000000000000001101;
	
	wire [31:0] x_offset = 32'b00000000000000000000000110111100;
	wire [31:0] y_offset = 32'b00000000000000000000000111100000;
	
	
	wire [31:0] x_pos_with_offset;
	CarryLookaheadAdder2(.x(register_x), .y(x_offset), .sum(x_pos_with_offset), .subtractEnable(1'b0));
	CarryLookaheadAdder2(.x(x_pos_with_offset), .y(frogger_width), .sum(left_pos), .subtractEnable(1'b1));
	CarryLookaheadAdder2(.x(x_pos_with_offset), .y(frogger_width), .sum(right_pos), .subtractEnable(1'b0));
	
	wire [31:0] y_pos_with_offset;
	CarryLookaheadAdder2(.x(register_y), .y(y_offset), .sum(y_pos_with_offset), .subtractEnable(1'b0));
	CarryLookaheadAdder2(.x(y_pos_with_offset), .y(frogger_height), .sum(top_pos), .subtractEnable(1'b1));
	CarryLookaheadAdder2(.x(y_pos_with_offset), .y(frogger_height), .sum(bottom_pos), .subtractEnable(1'b0));
	
endmodule

module getEnemyOnePosition(register_x, left_pos, right_pos, top_pos, bottom_pos);
	input [31:0] register_x; // x and y positions stored in some register
	output [31:0] left_pos, right_pos, top_pos, bottom_pos;
	
	
	wire [31:0] enemy_width =  32'b00000000000000000000000000100000;
	wire [31:0] enemy_height = 32'b00000000000000000000000000001000;
	
	wire [31:0] y_offset = 32'b00000000000000000000000001100100;	
	
	wire [31:0] x_pos_with_offset;
	CarryLookaheadAdder2(.x(register_x), .y(enemy_width), .sum(left_pos), .subtractEnable(1'b1));
	CarryLookaheadAdder2(.x(register_x), .y(enemy_width), .sum(right_pos), .subtractEnable(1'b0));
	
	CarryLookaheadAdder2(.x(y_offset), .y(enemy_height), .sum(top_pos), .subtractEnable(1'b1));
	CarryLookaheadAdder2(.x(y_offset), .y(enemy_height), .sum(bottom_pos), .subtractEnable(1'b0));
	
endmodule


module getEnemyTwoPosition(register_x, left_pos, right_pos, top_pos, bottom_pos);
	input [31:0] register_x; // x and y positions stored in some register
	output [31:0] left_pos, right_pos, top_pos, bottom_pos;
	
	
	wire [31:0] enemy_width =  32'b00000000000000000000000000011000;
	wire [31:0] enemy_height = 32'b00000000000000000000000000100010;
	
	wire [31:0] y_offset = 32'b00000000000000000000000011000111;	
	
	wire [31:0] x_pos_with_offset;
	CarryLookaheadAdder2(.x(register_x), .y(enemy_width), .sum(left_pos), .subtractEnable(1'b1));
	CarryLookaheadAdder2(.x(register_x), .y(enemy_width), .sum(right_pos), .subtractEnable(1'b0));
	
	CarryLookaheadAdder2(.x(y_offset), .y(enemy_height), .sum(top_pos), .subtractEnable(1'b1));
	CarryLookaheadAdder2(.x(y_offset), .y(enemy_height), .sum(bottom_pos), .subtractEnable(1'b0));
	
endmodule

module getEnemyThreePosition(register_x, left_pos, right_pos, top_pos, bottom_pos);
	input [31:0] register_x; // x and y positions stored in some register
	output [31:0] left_pos, right_pos, top_pos, bottom_pos;
	
	wire [31:0] enemy_width =  32'b00000000000000000000000000000100;
	wire [31:0] enemy_height = 32'b00000000000000000000000000000100;
	
	wire [31:0] y_offset = 32'b00000000000000000000000101000000;	
	
	wire [31:0] x_pos_with_offset;
	CarryLookaheadAdder2(.x(register_x), .y(enemy_width), .sum(left_pos), .subtractEnable(1'b1));
	CarryLookaheadAdder2(.x(register_x), .y(enemy_width), .sum(right_pos), .subtractEnable(1'b0));
	
	CarryLookaheadAdder2(.x(y_offset), .y(enemy_height), .sum(top_pos), .subtractEnable(1'b1));
	CarryLookaheadAdder2(.x(y_offset), .y(enemy_height), .sum(bottom_pos), .subtractEnable(1'b0));
	
endmodule

module getLogPosition(register_x, left_pos, right_pos, top_pos, bottom_pos);
	input [31:0] register_x; // x and y positions stored in some register
	output [31:0] left_pos, right_pos, top_pos, bottom_pos;
	
	
	wire [31:0] enemy_width =  32'b00000000000000000000000001110000;
	wire [31:0] enemy_height = 32'b00000000000000000000000000111110;
	
	wire [31:0] y_offset = 32'b00000000000000000000000011111111;	
	
	wire [31:0] x_pos_with_offset;
	CarryLookaheadAdder2(.x(register_x), .y(enemy_width), .sum(left_pos), .subtractEnable(1'b1));
	CarryLookaheadAdder2(.x(register_x), .y(32'b0), .sum(right_pos), .subtractEnable(1'b0));
	
	CarryLookaheadAdder2(.x(y_offset), .y(enemy_height), .sum(top_pos), .subtractEnable(1'b1));
	CarryLookaheadAdder2(.x(y_offset), .y(enemy_height), .sum(bottom_pos), .subtractEnable(1'b0));
	
endmodule

module getLake1(register_x, left_pos, right_pos, top_pos, bottom_pos);
	input [31:0] register_x; // x and y positions stored in some register
	output [31:0] left_pos, right_pos, top_pos, bottom_pos;
	
	
	wire [31:0] enemy_width =  32'b00000000000000000000000100111000;
	wire [31:0] enemy_height = 32'b00000000000000000000000000011110;
	
	wire [31:0] y_offset = 32'b00000000000000000000000011111111;	
	
	wire [31:0] x_pos_with_offset;
	CarryLookaheadAdder2(.x(register_x), .y(enemy_width), .sum(left_pos), .subtractEnable(1'b1));
	CarryLookaheadAdder2(.x(register_x), .y(enemy_width), .sum(right_pos), .subtractEnable(1'b0));
	
	CarryLookaheadAdder2(.x(y_offset), .y(enemy_height), .sum(top_pos), .subtractEnable(1'b1));
	CarryLookaheadAdder2(.x(y_offset), .y(enemy_height), .sum(bottom_pos), .subtractEnable(1'b0));
	
endmodule

module getEndGame(register_x, left_pos, right_pos, top_pos, bottom_pos);
	input [31:0] register_x; // x and y positions stored in some register
	output [31:0] left_pos, right_pos, top_pos, bottom_pos;
	
	
	wire [31:0] enemy_width =  32'b00000000000000000000000100111111;
	wire [31:0] enemy_height = 32'b00000000000000000000000000011010;
	
	wire [31:0] y_offset = 32'b00000000000000000000000000100111;	
	
	wire [31:0] x_pos_with_offset;
	CarryLookaheadAdder2(.x(register_x), .y(enemy_width), .sum(left_pos), .subtractEnable(1'b1));
	CarryLookaheadAdder2(.x(register_x), .y(enemy_width), .sum(right_pos), .subtractEnable(1'b0));
	
	CarryLookaheadAdder2(.x(y_offset), .y(enemy_height), .sum(top_pos), .subtractEnable(1'b1));
	CarryLookaheadAdder2(.x(y_offset), .y(enemy_height), .sum(bottom_pos), .subtractEnable(1'b0));
	
endmodule


module processor(clock, reset, ps2_key_pressed, ps2_out, lcd_write, lcd_data, debug_data, debug_addr, 
                 registerOneOutput, registerTwoOutput, registerThreeOutput, registerFourOutput,
					  registerFiveOutput, registerSixOutput, registerSevenOutput, registerTenOutput, register11Output,
					  collision, endGameCollision, lives, frogInsideLog, mul_in_execute, div_in_execute, mult_alu_op,
					  div_alu_op, multOperation,  divOperation);

	input 			clock, reset, ps2_key_pressed, collision, endGameCollision, frogInsideLog;
	input 	[7:0]	ps2_out;
	
	output 			lcd_write;
	output 	[31:0] 	lcd_data;
	
	// GRADER OUTPUTS - YOU MUST CONNECT TO YOUR DMEM
	output 	[31:0] 	debug_data;
	output	[11:0]	debug_addr;
	
	
	output [31:0] registerOneOutput, registerTwoOutput, registerThreeOutput, registerFourOutput, registerFiveOutput,
					  registerSixOutput, registerSevenOutput, registerTenOutput, register11Output;


	output [31:0] lives;
	
	// your processor here
	//
	
	//////////////////////////////////////
	////// THIS IS REQUIRED FOR GRADING
	// CHANGE THIS TO ASSIGN YOUR DMEM WRITE ADDRESS ALSO TO debug_addr
	assign debug_addr = dmemAddressInput;
	// CHANGE THIS TO ASSIGN YOUR DMEM DATA INPUT (TO BE WRITTEN) ALSO TO debug_data
	assign debug_data = dmemDataInput;
	////////////////////////////////////////////////////////////
	
		
	// You'll need to change where the dmem and imem read and write...
	
	wire rType_f_d, iType_f_d; 
	assign rType_f_d = ~f_d_insn_output[31] & ~f_d_insn_output[30] & ~f_d_insn_output[29] & ~f_d_insn_output[28] & ~f_d_insn_output[27];
	is_iTypeInsn(.insn(f_d_insn_output[31:27]), .eq(iType_f_d));
	
	wire rType_d_e, iType_d_e; 
	assign rType_d_e = ~d_e_insn_output[31] & ~d_e_insn_output[30] & ~d_e_insn_output[29] & ~d_e_insn_output[28] & ~d_e_insn_output[27];
	is_iTypeInsn(.insn(d_e_insn_output[31:27]), .eq(iType_d_e));
		
	wire rType_e_m, iType_e_m; 
	assign rType_e_m = ~e_m_insn_output[31] & ~e_m_insn_output[30] & ~e_m_insn_output[29] & ~e_m_insn_output[28] & ~e_m_insn_output[27];
	is_iTypeInsn(.insn(e_m_insn_output[31:27]), .eq(iType_e_m));
	
	wire [31:0] imemOutput;
	imem myimem(	.address 	(pc_reg_output),
					.clken		(1'b1),
					.clock		(clock),
					.q 			(imemOutput) // change where output q goes...
	); 
	
	wire [31:0] pc_reg_output;
	
	wire [31:0] pcInput;
	
	// logic for j command
	wire [31:0] pcInputWithJump;
	wire jumpInExecute;
	assign jumpInExecute = j_in_execute | jal_in_execute;
	
	wire j_in_execute;
	wire jal_in_execute;
	wire jr_in_execute;
	
	assign pcInputWithJump[31:27] = pc_reg_output_plus_one[31:27];
	assign pcInputWithJump[26:0] = d_e_insn_output[26:0];

	// determines whether there is a 'j' or 'jal' instruction in the exectue stage
	registerEquivalent(.inputA(d_e_insn_output[31:27]), .inputB(5'b00001), .eq(j_in_execute));
	registerEquivalent(.inputA(d_e_insn_output[31:27]), .inputB(5'b00011), .eq(jal_in_execute));
	registerEquivalent(.inputA(d_e_insn_output[31:27]), .inputB(5'b00100), .eq(jr_in_execute));
	
	CarryLookaheadAdder2 addOne(.x(d_e_pc_output), .y(valueOfOne), .sum(pc_reg_output_plus_one), .subtractEnable(1'b0));
	
	wire [31:0] valueOfOne, pc_reg_output_plus_one;
	assign valueOfOne[31:1] = 31'b0;
	assign valueOfOne[0] = 1;
	
	
	// end of logic for j command
	
	wire [31:0] actual_pcInput;
	assign actual_pcInput = ({32{jumpInExecute & ~jr_in_execute}} & pcInputWithJump) |
							({32{~jumpInExecute & ~jr_in_execute}} & pcInput) | 
							({32{jr_in_execute}} & aluAInput); // was decode_execute_A_output
	
	pc_reg(.clock(clock), .dataInputPC(actual_pcInput), .dataOutputPC(pc_reg_output), .reset(reset));
	
	// does not increment pc on a stall
	wire [31:0] addValue;
	assign addValue[31:1] = 31'b0;
	assign addValue[0] = ~stall & ~jump_due_to_bne & ~jump_due_to_blt & ~jump_due_to_bex;
	
	wire [31:0] addFourAInput;
	assign addFourAInput = ({32{~jump_due_to_bne & ~jump_due_to_blt & ~jump_due_to_bex}} & pc_reg_output)
	                     | ({32{jump_due_to_bne | jump_due_to_blt | jump_due_to_bex}} & d_e_pc_output);
	
	CarryLookaheadAdder2 addFour(.x(addFourAInput), .y(addValue), .sum(pcIntermediate), .subtractEnable(1'b0));

	
	wire [31:0] pcIntermediate;
	
	// choose 0 or N
	wire [31:0] addValueIntermediate;
	assign addValueIntermediate = ({32{jump_due_to_bne | jump_due_to_blt}} & signExtendedImmediate)
								   | ({32{jump_due_to_bex}} & signExtendedImmediate26);
	
	
	CarryLookaheadAdder2 addOffset(.x(pcIntermediate), .y(addValueIntermediate), .sum(pcInput), .subtractEnable(1'b0));
		
	// on a stall, pipe current instruction back into f/d register
	wire [31:0] f_d_insn_output;
	wire [31:0] pc_input_to_f_d;
	assign pc_input_to_f_d = ({32{stall & ~jump_due_to_bne & ~jump_due_to_blt & ~jump_due_to_bex & ~jumpInExecute & ~jr_in_execute}} & f_d_insn_output) |
                             ({32{~stall & ~jump_due_to_bne & ~jump_due_to_blt & ~jump_due_to_bex & ~jumpInExecute & ~jr_in_execute}} & imemOutput)     | 
                             ({32{jump_due_to_bne | jump_due_to_blt | jump_due_to_bex | jumpInExecute | jr_in_execute | jr_in_execute}} & nop);
	
	
	wire [31:0] f_d_pc_output;
	fetch_decode_reg(.clock(~clock), .dataInputPC(pc_reg_output), .instructionInput(pc_input_to_f_d), .dataOutputPC(f_d_pc_output), .instructionOutput(f_d_insn_output), .reset(reset));
	
	// attempt to introduce stall logic into the program
	
	wire loadWordInExecuteStage;
	registerEquivalent(.inputA(d_e_insn_output[31:27]), .inputB(5'b01000), .eq(loadWordInExecuteStage));
	
	// f/d.ir.rs1 = d/x.ir.rd , f/d.ir.rs2 = d/x.ir.rd
	wire decode_rs1_execute_rd_equivalent;
	registerEquivalent(.inputA(f_d_insn_output[21:17]), .inputB(d_e_insn_output[26:22]), .eq(decode_rs1_execute_rd_equivalent));
	wire decode_rs2_execute_rd_equivalent;
	registerEquivalent(.inputA(f_d_insn_output[16:12]), .inputB(d_e_insn_output[26:22]), .eq(decode_rs2_execute_rd_equivalent));
	 
	// store word in decode stage
	wire storeWordInDecodeStage;
	registerEquivalent(.inputA(f_d_insn_output[31:27]), .inputB(5'b00111), .eq(storeWordInDecodeStage));
	
	wire stall;
	assign stall = ((loadWordInExecuteStage) & ((decode_rs1_execute_rd_equivalent) |
					((decode_rs2_execute_rd_equivalent) & ~storeWordInDecodeStage))) | multOperation | divOperation | mul_div_ready;
	
	
	wire bne_in_fd, blt_in_fd;
	registerEquivalent(.inputA(f_d_insn_output[31:27]), .inputB(5'b00010), .eq(bne_in_fd));
	registerEquivalent(.inputA(f_d_insn_output[31:27]), .inputB(5'b00110), .eq(blt_in_fd));
	wire bne_or_blt_in_fd;
	assign bne_or_blt_in_fd = bne_in_fd | blt_in_fd;
	
	wire [4:0] regA_ctrlInput, regB_ctrlInput;
	wire [4:0] writeReg_ctrlInput;
	assign regA_ctrlInput = ({5{~bne_or_blt_in_fd & ~jr_in_decode}} & f_d_insn_output[21:17]) | 
	                        ({5{bne_or_blt_in_fd | jr_in_decode}} & f_d_insn_output[26:22]); 
	assign regB_ctrlInput = ({5{rType_f_d & ~bne_or_blt_in_fd}} & f_d_insn_output[16:12]) |
	                        ({5{iType_f_d & ~bne_or_blt_in_fd}} & f_d_insn_output[26:22]) |
	                        ({5{bne_or_blt_in_fd}} & f_d_insn_output[21:17]); // if i-Type, we choose rD value instead of rT
	
	assign writeReg_ctrlInput = ({5{~jal_inWriteback}} & m_w_insn_output[26:22]) |
	                            ({5{jal_inWriteback}} & 5'b11111);
	
	// is there jump return in fetch decode stage
	wire jr_in_decode;
	registerEquivalent(.inputA(f_d_insn_output[31:27]), .inputB(5'b00100), .eq(jr_in_decode));
	
	regFile_sdh31(.clock(clock), .ctrl_writeEnable(needToWritebackActual), .ctrl_reset(reset), .ctrl_writeReg(writeReg_ctrlInput),
				  .ctrl_readRegA(regA_ctrlInput), .ctrl_readRegB(regB_ctrlInput), .data_writeReg(writeback_to_reg),
				  .data_readRegA(readRegAOutput), .data_readRegB(readRegBOutput),
				  .registerOneOutput(registerOneOutput), .registerTwoOutput(registerTwoOutput),
              .ctrl_write_reg_one(ctrl_write_reg_one), .ctrl_write_reg_two(ctrl_write_reg_two),
              .data_write_reg_one(writeDataRegOne), .data_write_reg_two(writeDataRegTwo),
				  .registerThreeOutput(registerThreeOutput),
				  .registerFourOutput(registerFourOutput), 
				  .registerFiveOutput(registerFiveOutput),
				  .registerSixOutput(registerSixOutput), .registerSevenOutput(registerSevenOutput),
				  .registerTenOutput(registerTenOutput),
				  .register11Output(register11Output), 
				  .collision(collision),
				  .endGameCollision(endGameCollision),
				  .frogInsideLog(frogInsideLog),
				  .lives(lives));
				  
	wire [31:0] readRegAOutput, readRegBOutput;
				  
	
	// ******************************** 
	// Control for Reg1 & Reg2
	
	wire [31:0] writeDataRegOne, writeDataRegTwo;
	wire ctrl_write_reg_one, ctrl_write_reg_two;
	
	
	wire livesLeft;
	NotEqualZero(.x(lives), .out(livesLeft));
	
	
	
	wire [31:0] xSpeed = 32'b00000000000000000000000000001111 & {32{livesLeft & ~registerTenOutput[0]}};
	wire [31:0] ySpeed = 32'b00000000000000000000000000001111  & {32{livesLeft & ~registerTenOutput[0]}};
		
	// must determine if we can allow movement.... must ensure x is within bounds of screen

	CarryLookaheadAdder2(.x(registerOneOutput), .y(xSpeed), .sum(writeDataRegOne), .subtractEnable(add_or_sub_reg_one));

	
	wire add_or_sub_reg_one;
	assign add_or_sub_reg_one = leftArrowPressed; // 1 means subtract        
	          
	wire rightArrowPressed, leftArrowPressed;
	
	wire signed [31:0] neg_300;
	wire signed [31:0] pos_300;
	wire signed [31:0] regOneSigned;
	assign regOneSigned = registerOneOutput;
	assign neg_300 = 32'b11111111111111111111111011010100;
	assign pos_300 = 32'b00000000000000000000000100101100;
	
	assign ctrl_write_reg_one = ((leftArrowPressed & (regOneSigned > neg_300))
											| ((regOneSigned < pos_300) & rightArrowPressed)) & ps2_key_pressed; 
	
	keyboardEquivalent(.inputA(ps2_out), .inputB(8'b01110100), .eq(rightArrowPressed));
	keyboardEquivalent(.inputA(ps2_out), .inputB(8'b01101011), .eq(leftArrowPressed));
	
	CarryLookaheadAdder2(.x(registerTwoOutput), .y(ySpeed), .sum(writeDataRegTwo),
								.subtractEnable(add_or_sub_reg_two));

	wire add_or_sub_reg_two;
	assign add_or_sub_reg_two = upArrowPressed; // 1 means subtract        
	          
	wire upArrowPressed, downArrowPressed;
	
	wire signed [31:0] regTwoSigned;
	assign regTwoSigned = registerTwoOutput;
	
	assign ctrl_write_reg_two = ((downArrowPressed & regTwoSigned < 0) | upArrowPressed) & ps2_key_pressed; 
	
	keyboardEquivalent(.inputA(ps2_out), .inputB(8'b01110101), .eq(upArrowPressed));
	keyboardEquivalent(.inputA(ps2_out), .inputB(8'b01110010), .eq(downArrowPressed));
	
	// End of Control for Reg1 and Reg2
	// *********************************
			  	
	// on a stall, insert NOP into d/e register		
	
	wire [31:0] nop = 32'b11111111111111111111111111111111;
	
	wire [31:0] insn_input_to_d_e;
	
	
	// this is an important case to consider
	// 1) there is some sort of jump in the execute stage... this includes bne, blt, bex, j, jal, jr, so we pipe in no-ops into the execute stage
	//    here, and into the decode stage as well.
	// 2) there is a stall, due to loading a word into a register or a mult/div operation
	//
	// if none of the above two cases occur, we take the normal fetch/decode instrution output
	assign insn_input_to_d_e = ({32{  stall | jump_due_to_bne | jump_due_to_blt | jump_due_to_bex | jumpInExecute | jr_in_execute | multOperation | divOperation | mul_div_ready}} & nop) |
	                           ({32{~(stall | jump_due_to_bne | jump_due_to_blt | jumpInExecute | jr_in_execute | jump_due_to_bex | multOperation | divOperation | mul_div_ready)}} & f_d_insn_output);
	        
	
	
	
	
	wire [31:0] d_e_pc_output;
	
	decode_execute_reg(.clock(~clock), .dataInputA(readRegAOutput), .dataInputB(readRegBOutput), .dataInputPC(f_d_pc_output),
					                  .dataOutputA(decode_execute_AOutput), .dataOutputB(decode_execute_BOutput), 
					                  .dataOutputPC(d_e_pc_output),
					                  .instructionInput(insn_input_to_d_e), .instructionOutput(d_e_insn_output),
					                  .reset(reset));
					                  
	wire [31:0] decode_execute_AOutput;
	wire [31:0] decode_execute_BOutput;
	wire [31:0] d_e_insn_output;	
	wire [4:0] ctrl_ALUopcode = ({5{rType_d_e}} & d_e_insn_output[6:2]) | ({5{blt_in_execute}} & 5'b00001);    
	      
	wire blt_in_execute;
	registerEquivalent(.inputA(d_e_insn_output[31:27]), .inputB(5'b00110), .eq(blt_in_execute));
		                  	
	// ****************************************************************************
	// ****************************************************************************
	// Determining which A Operand is input into the ALU
	wire [31:0] aluAInput; 
	
	// to bypass, need proper r-i commands and need register equivalence
	wire bypassAInputFromMemoryCommandEquivalent;
	wire bypassAInputFromMemoryRegisterEquivalent;
	wire bypassInputFromWritebackCommandEquivalent;
	wire bypassInputFromWritebackRegisterEquivalent;
	
	
	// if read value in execute and write value in memory, there is a possibility of forwarding
	wire readValueInExecute, writeValueInMemory, writeValueInWriteback;
	
	readValueInExecute(.in(d_e_insn_output[31:27]), .out(readValueInExecute));
	writeValueInMemory(.in(e_m_insn_output[31:27]), .out(writeValueInMemory));
	assign bypassInputFromMemoryCommandEquivalent = readValueInExecute & writeValueInMemory;
	
	writeValueInWriteback(.in(m_w_insn_output[31:27]), .out(writeValueInWriteback));
	assign bypassInputFromWritebackCommandEquivalent = readValueInExecute & writeValueInWriteback;
	
	wire bypassAInputFromMemory, bypassAInputFromWriteback;
	assign bypassAInputFromMemory = bypassInputFromMemoryCommandEquivalent & bypassAInputFromMemoryRegisterEquivalent;
	assign bypassAInputFromWriteback = bypassInputFromWritebackCommandEquivalent & bypassAInputFromWritebackRegisterEquivalent;
	
	wire [4:0] A_Operand_Input_A;
	assign A_Operand_Input_A = ({5{~blt_in_execute & ~bne_in_execute}} & d_e_insn_output[21:17]) |
							   ({5{blt_in_execute | bne_in_execute}} & d_e_insn_output[26:22]);
	
	// if the RS register of the read is the same as the RD register of the mem
	registerEquivalent(.inputA(A_Operand_Input_A), .inputB(e_m_insn_output[26:22]), .eq(bypassAInputFromMemoryRegisterEquivalent));
	// if the RS register of the read is the same RD register of the writeback
	registerEquivalent(.inputA(A_Operand_Input_A), .inputB(m_w_insn_output[26:22]), .eq(bypassAInputFromWritebackRegisterEquivalent));
	
	wire nop_in_mem, nop_in_writeback;
	registerEquivalent(.inputA(5'b11111), .inputB(e_m_insn_output[31:27]), .eq(nop_in_mem));
	registerEquivalent(.inputA(5'b11111), .inputB(m_w_insn_output[31:27]), .eq(nop_in_writeback));
	
	// takes input from memory
	TriState32Bit(.in(e_m_aluOutput), .out(aluAInput), .enable(bypassAInputFromMemory & ~nop_in_mem));
	// takes input from writeback
	TriState32Bit(.in(writeback_to_reg), .out(aluAInput), .enable((bypassAInputFromWriteback & ~nop_in_writeback) & ~(bypassAInputFromMemory & ~nop_in_mem)));
	// does not forward
	TriState32Bit(.in(decode_execute_AOutput), .out(aluAInput), .enable(~((bypassAInputFromWriteback & ~nop_in_writeback) | 
                                                                         (bypassAInputFromMemory & ~nop_in_mem))));
	                  
	// ****************************************************************************	 
	// ****************************************************************************   
	
	
	// ****************************************************************************
	// ****************************************************************************
	// Determining which B Operand is input into the ALU    
	// if the RT register of the read is the same as the RD register of the mem
	
	wire storeWordInsnInExecute;
	wire [4:0] inputA_AluInputB;
	registerEquivalent(.inputA(d_e_insn_output[31:27]), .inputB(5'b00111), .eq(storeWordInsnInExecute));
	assign inputA_AluInputB = ({5{storeWordInsnInExecute}} & d_e_insn_output[26:22]) | ({5{~storeWordInsnInExecute}} & d_e_insn_output[16:12]) |
	                           ({5{blt_in_execute | bne_in_execute}} & d_e_insn_output[21:17]);
	
	registerEquivalent(.inputA(inputA_AluInputB), .inputB(e_m_insn_output[26:22]), .eq(bypassBInputFromMemoryRegisterEquivalent)); 
	// if the RT register of the read is the same as the RD register of the writeback
	registerEquivalent(.inputA(inputA_AluInputB), .inputB(m_w_insn_output[26:22]), .eq(bypassBInputFromWritebackRegisterEquivalent));
	
	wire bypassBInputFromMemory, bypassBInputFromWriteback; 
	
	wire [31:0] signExtendedImmediate;
	signExtend(.in(d_e_insn_output[16:0]), .out(signExtendedImmediate));
	
	wire [31:0] signExtendedImmediate26;
	signExtend(.in(d_e_insn_output[26:0]), .out(signExtendedImmediate26));
	
	wire [31:0] rTypeBInput;
	
	
	// to bypass, need proper r-i commands and need register equivalence
	wire bypassBInputFromMemoryRegisterEquivalent;
	wire bypassBInputFromWritebackRegisterEquivalent;
	
	assign bypassBInputFromMemory = bypassInputFromMemoryCommandEquivalent & bypassBInputFromMemoryRegisterEquivalent;
	assign bypassBInputFromWriteback = bypassInputFromWritebackCommandEquivalent & bypassBInputFromWritebackRegisterEquivalent;
	
	
	TriState32Bit(.in(e_m_aluOutput), .out(rTypeBInput), .enable(bypassBInputFromMemory & ~nop_in_mem));
	TriState32Bit(.in(writeback_to_reg), .out(rTypeBInput), .enable((bypassBInputFromWriteback & ~nop_in_writeback) & ~(bypassBInputFromMemory & ~nop_in_mem)));
	TriState32Bit(.in(decode_execute_BOutput), .out(rTypeBInput), .enable(~((bypassBInputFromMemory & ~nop_in_mem) | (bypassBInputFromWriteback & ~nop_in_writeback))));
	
	wire [31:0] aluBInput = ({32{rType_d_e | blt_in_execute | bne_in_execute}} & rTypeBInput) | 
	                        ({32{iType_d_e & ~blt_in_execute & ~bne_in_execute}} & signExtendedImmediate); // i type uses sign extended immediate 
	                    
	// ****************************************************************************
	// **************************************************************************** 
	                  
	wire ab_notequal, a_lessthan_b;
	wire jump_due_to_bne;
	wire jump_due_to_blt;
	wire bne_in_execute;
	assign jump_due_to_bne = ab_notequal & bne_in_execute;
	assign jump_due_to_blt = a_lessthan_b & blt_in_execute;
	
	// bex
	wire bex_in_execute, status_gt_zero;
	wire jump_due_to_bex;
	assign jump_due_to_bex = bex_in_execute & status_gt_zero;
	registerEquivalent(.inputA(d_e_insn_output[31:27]), .inputB(5'b10110), .eq(bex_in_execute));
	compareStatusToZero(.in(outStatus), .gt(status_gt_zero));
	
	registerEquivalent(.inputA(d_e_insn_output[31:27]), .inputB(5'b00010), .eq(bne_in_execute));
	
	// if blt command, must give alu op code a subtract
	
	
	
	wire [31:0] mainALUOutput;
   // tristates choose between output of alu and the multiplier
	TriState32Bit(.in(mainALUOutput), .out(outputOfAlu), .enable(~mul_div_ready));
	TriState32Bit(.in(data_result_of_mult_div_temp), .out(outputOfAlu), .enable(mul_div_ready));
	
	alu_sdh31 mainALU(.data_operandA(aluAInput), 
	                  .data_operandB(aluBInput), .ctrl_ALUopcode(ctrl_ALUopcode),
	                  .ctrl_shiftamt(d_e_insn_output[11:7]), .data_result(mainALUOutput), .isNotEqual(ab_notequal),
	                  .isLessThan(a_lessthan_b));
	                  		
	        // logic for multiplication and division
        
	// first cycle of multiplication enters the execute stage
	output mul_in_execute, div_in_execute;
	output mult_alu_op, div_alu_op;
	output multOperation;
	output divOperation;
	assign mul_in_execute = rType_d_e & mult_alu_op;
	assign div_in_execute = rType_d_e & div_alu_op;
	registerEquivalent(.inputA(d_e_insn_output[6:2]), .inputB(5'b00110), .eq(mult_alu_op)); // mult.
	registerEquivalent(.inputA(d_e_insn_output[6:2]), .inputB(5'b00111), .eq(div_alu_op)); // div.
	// temp is because this value is only temporary. we need to store it in a flip flop so not to lose it
	wire [31:0] data_result_of_mult_div_temp;
	yh91_hw4(.data_operandA(aluAInput), .data_operandB(aluBInput), .ctrl_MULT(mul_in_execute), .ctrl_DIV(div_in_execute), .clock(clock),
                      .data_result(data_result_mult_div_pre_temp), .data_resultRDY(mul_div_ready_temp), .multOperation(multOperation), .divOperation(divOperation));
      
	wire mul_div_ready;
	
	wire [31:0] data_result_mult_div_pre_temp;
	wire [31:0] data_result_mult_div_pre_temp1;
	latch32bit(.d(data_result_mult_div_pre_temp), .ena(~clock), .q(data_result_mult_div_pre_temp1));
	DFF32Bit (.dataInput(data_result_mult_div_pre_temp1), .clock(clock), .write_enable(1), .dataOutput(data_result_of_mult_div_temp));
	
	wire mul_div_ready_temp;
	DFFE (.clk(~clock), .d(mul_div_ready_temp), .ena(1), .q(mul_div_ready));

	wire [31:0] mul_div_insn;
	// stores the value of the mul div insn out a positive clock... allows us to pipe in nops later
   DFF32Bit mul_div_insn_dffe(.dataInput(d_e_insn_output), .clock(clock), .write_enable(mul_in_execute | div_in_execute), .dataOutput(mul_div_insn));
	   

	wire [31:0] e_m_insn_input;
	// CASES
	// 1) on mult or div ready, we pass the mult/div instruction
	// 2) on mult/div not ready and current mult/div is executing, pass nop
	// 3) on neither, pass regular values
	assign e_m_insn_input = ({32{mul_div_ready}} & mul_div_insn) |
                                       ({32{multOperation | divOperation}} & nop) |
                                    ({32{~mul_div_ready & ~multOperation & ~divOperation}} & d_e_insn_output);

		
	wire setx_in_execute;
	wire [26:0] outStatus; // output of status register
	registerEquivalent(.inputA(d_e_insn_output[31:27]), .inputB(5'b10101), .eq(setx_in_execute));
	
	statusRegister(.clock(clock), .enable(setx_in_execute), .statusIn(d_e_insn_output[26:0]), .statusOut(outStatus), .reset(reset));

	wire [31:0] e_m_insn_output, outputOfAlu;
	wire [31:0] e_m_aluOutput, e_m_rTypeBOutput, e_m_pc_output;
			  
	execute_memory_reg(.clock(~clock), .dataInputFromAlu(outputOfAlu), .dataOutputFromAlu(e_m_aluOutput), 
	                   .instructionInput(e_m_insn_input), .instructionOutput(e_m_insn_output), .bValueIn(rTypeBInput), .bValueOut(e_m_rTypeBOutput),
	                   .pcInput(d_e_pc_output), .pcOutput(e_m_pc_output), .reset(reset));
	                   
	                   
	wire [31:0] dmemOutput;
	wire [31:0] dmemDataInput;
	
	wire loadWordInWriteback; // whether load word in writeback stage
	registerEquivalent(.inputA(m_w_insn_output[31:27]), .inputB(5'b01000), .eq(loadWordInWriteback));
	
	wire storeWordInsn;
	registerEquivalent(.inputA(e_m_insn_output[31:27]), .inputB(5'b00111), .eq(storeWordInsn));
	wire same_SW_INSN_RD;
	registerEquivalent(.inputA(e_m_insn_output[26:22]), .inputB(m_w_insn_output[26:22]), .eq(same_SW_INSN_RD));
	wire takenReg_FollowedBy_sw;
	assign takenReg_FollowedBy_sw = storeWordInsn & same_SW_INSN_RD & loadWordInWriteback; // checks if store word in mem. and load word in writeback are same rd
	
	// writeback register is same as RD of store word
	
	
	assign dmemDataInput = ({32{~takenReg_FollowedBy_sw}} & e_m_rTypeBOutput) | ({32{takenReg_FollowedBy_sw}} & writeback_to_reg);
	
	wire [11:0] dmemAddressInput;
	assign dmemAddressInput = e_m_aluOutput[11:0];
	
	
	wire stallInMemory;
	registerEquivalent(.inputA(e_m_insn_output[31:27]), .inputB(5'b11111), .eq(stallInMemory));
	
	dmem mydmem(	.address	(dmemAddressInput),
					.clock		(clock),
					.data		(dmemDataInput),
					.wren		(storeWordInsn & ~stallInMemory),
					.q			(dmemOutput) // change where output q goes...
	);
	
	
	
	memory_writeback_reg(.clock(~clock), .dataInputFromDMem(dmemOutput), .dataOutputFromDMem(m_w_reg_output), 
	                     .instructionInput(e_m_insn_output), .instructionOutput(m_w_insn_output), 
	                     .dataInputFromALU(e_m_aluOutput), .dataOutputFromALU(m_w_alu_output),
	                     .pcInput(e_m_pc_output), .pcOutput(m_w_pcOutput), .reset(reset));
	
	wire [31:0] m_w_reg_output, m_w_alu_output, m_w_pcOutput;
	wire [31:0] m_w_insn_output;
	wire [31:0] writeback_to_reg;
	
	// determines whether insn is load word
	wire loadWordInsn;
	registerEquivalent(.inputA(m_w_insn_output[31:27]), .inputB(5'b01000), .eq(loadWordInsn));
	
	wire needToWriteback, needToWritebackActual;
	shouldWriteback(.in(m_w_insn_output[31:27]), .eq(needToWriteback));
	
	
	wire zeroRegisterInWrite;
	//registerEquivalent(.inputA(m_w_insn_output[26:22]), .inputB(5'b00000), .eq(zeroRegisterInWrite));
	
	assign needToWritebackActual = needToWriteback;
	
	
	registerEquivalent(.inputA(m_w_insn_output[31:27]), .inputB(5'b00000), .eq(rTypeInWriteback));
	
	wire jal_inWriteback;
	registerEquivalent(.inputA(5'b00011), .inputB(m_w_insn_output[31:27]), .eq(jal_inWriteback));
	
	
	// either write alu output to register or dmemOutput of a load word
	assign writeback_to_reg = ({32{~loadWordInsn & ~jal_inWriteback}} & m_w_alu_output) | 
	                          ({32{loadWordInsn & ~jal_inWriteback}} & m_w_reg_output) |
	                          ({32{jal_inWriteback}} & m_w_pcOutput);
		
endmodule


module shouldWriteback(in, eq);
	input [4:0] in;
	output eq;
	
	assign eq = (~in[4] & ~in[3] & ~in[2] & ~in[1] & ~in[0]) |    // r type
				(~in[4] & ~in[3] & ~in[2] & in[1] & in[0])   |    // jal
				(~in[4] & ~in[3] & in[2] & ~in[1] & in[0])   |    // addi
				(~in[4] &  in[3] & ~in[2] & ~in[1] & ~in[0]);     // lw



endmodule

module latch32bit(d, q, ena);
	input [31:0] d;
	input ena;
	output [31:0] q;
	
	genvar i;
	generate
	for (i=0; i<32; i=i+1) begin: loop1
		latch a_dffe(.d(d[i]), .ena(ena), .q(q[i]));
	end
	endgenerate
endmodule


module registerEquivalent(inputA, inputB, eq);
	input [4:0] inputA, inputB;
	output eq;
	
	assign eq = ((inputA[4] & inputB[4]) | (~inputA[4] & ~inputB[4])) & 
	            ((inputA[3] & inputB[3]) | (~inputA[3] & ~inputB[3])) &
	            ((inputA[2] & inputB[2]) | (~inputA[2] & ~inputB[2])) &
	            ((inputA[1] & inputB[1]) | (~inputA[1] & ~inputB[1])) &
	            ((inputA[0] & inputB[0]) | (~inputA[0] & ~inputB[0]));

endmodule

module keyboardEquivalent(inputA, inputB, eq);
	input [7:0] inputA, inputB;
	output eq;
	
	assign eq = ((inputA[7] & inputB[7]) | (~inputA[7] & ~inputB[7])) &
				((inputA[6] & inputB[6]) | (~inputA[6] & ~inputB[6])) &
				((inputA[5] & inputB[5]) | (~inputA[5] & ~inputB[5])) &
				((inputA[4] & inputB[4]) | (~inputA[4] & ~inputB[4])) & 
	            ((inputA[3] & inputB[3]) | (~inputA[3] & ~inputB[3])) &
	            ((inputA[2] & inputB[2]) | (~inputA[2] & ~inputB[2])) &
	            ((inputA[1] & inputB[1]) | (~inputA[1] & ~inputB[1])) &
	            ((inputA[0] & inputB[0]) | (~inputA[0] & ~inputB[0]));

endmodule


module pc_reg(clock, dataInputPC, dataOutputPC, reset);
	input clock, reset;
	input [31:0] dataInputPC;
	output [31:0] dataOutputPC;
	
	DFF32Bit pc(.clock(clock), .write_enable(1), .dataInput(dataInputPC), .dataOutput(dataOutputPC), .reset(reset));

endmodule

module fetch_decode_reg(clock, dataInputPC, instructionInput, dataOutputPC, instructionOutput, reset);
	input clock, reset;
	input [31:0] dataInputPC, instructionInput;
	output [31:0] dataOutputPC, instructionOutput;

	DFF32Bit      pc(.clock(clock), .write_enable(1), .dataInput(dataInputPC), .dataOutput(dataOutputPC), .reset(reset));
	DFF32Bit    insn(.clock(clock), .write_enable(1), .dataInput(instructionInput), .dataOutput(instructionOutput), .reset(reset));

endmodule


module decode_execute_reg(clock, dataInputA, dataInputB, dataInputPC, instructionInput,
						  dataOutputA, dataOutputB, dataOutputPC, instructionOutput, reset);
	input clock, reset;
	input [31:0] dataInputA, dataInputB, dataInputPC, instructionInput;
	output [31:0] dataOutputA, dataOutputB, dataOutputPC, instructionOutput;

	DFF32Bit A_value(.clock(clock), .write_enable(1), .dataInput(dataInputA), .dataOutput(dataOutputA), .reset(reset));
	DFF32Bit B_value(.clock(clock), .write_enable(1), .dataInput(dataInputB), .dataOutput(dataOutputB), .reset(reset));
	DFF32Bit      pc(.clock(clock), .write_enable(1), .dataInput(dataInputPC), .dataOutput(dataOutputPC), .reset(reset));
	DFF32Bit    insn(.clock(clock), .write_enable(1), .dataInput(instructionInput), .dataOutput(instructionOutput), .reset(reset));

endmodule

module execute_memory_reg(clock, dataInputFromAlu, dataOutputFromAlu, instructionInput, instructionOutput, bValueIn, bValueOut,
                          pcInput, pcOutput, reset);
	input clock, reset;
	input [31:0] dataInputFromAlu, instructionInput, bValueIn, pcInput;
	output [31:0] dataOutputFromAlu, instructionOutput, bValueOut, pcOutput;

	DFF32Bit ALU_value(.clock(clock), .write_enable(1), .dataInput(dataInputFromAlu), .dataOutput(dataOutputFromAlu), .reset(reset));
	DFF32Bit    insn(.clock(clock), .write_enable(1), .dataInput(instructionInput), .dataOutput(instructionOutput), .reset(reset));
	DFF32Bit    bVal(.clock(clock), .write_enable(1), .dataInput(bValueIn), .dataOutput(bValueOut), .reset(reset));
	DFF32Bit    pc(.clock(clock), .write_enable(1), .dataInput(pcInput), .dataOutput(pcOutput), .reset(reset));
	
endmodule

module statusRegister(clock, enable, statusIn, statusOut, reset);
	input clock, enable, reset;
	input [26:0] statusIn;
	output [26:0] statusOut;
	DFF32Bit    status(.clock(clock), .dataInput(statusIn), .dataOutput(statusOut), .reset(reset), .write_enable(enable));

endmodule

module memory_writeback_reg(clock, dataInputFromDMem, dataOutputFromDMem, 
                            dataInputFromALU, dataOutputFromALU, instructionInput, instructionOutput,
                            pcInput, pcOutput, reset);
	input clock, reset;
	input [31:0] dataInputFromDMem, instructionInput, dataInputFromALU, pcInput;
	output [31:0] dataOutputFromDMem, instructionOutput, dataOutputFromALU, pcOutput;

	DFF32Bit mem_value(.clock(clock), .write_enable(1), .dataInput(dataInputFromDMem), .dataOutput(dataOutputFromDMem), .reset(reset));
	DFF32Bit    insn(.clock(clock), .write_enable(1), .dataInput(instructionInput), .dataOutput(instructionOutput), .reset(reset));
	DFF32Bit alu_value(.clock(clock), .write_enable(1), .dataInput(dataInputFromALU), .dataOutput(dataOutputFromALU), .reset(reset));
	DFF32Bit pc(.clock(clock), .write_enable(1), .dataInput(pcInput), .dataOutput(pcOutput), .reset(reset));

endmodule

module is_iTypeInsn(insn, eq);
	input [4:0] insn;
	output eq;
	
	assign eq = (~insn[4] & ~insn[3] & insn[2] & ~insn[1] & insn[0]) |
				(~insn[4] & ~insn[3] & ~insn[2] & insn[1] & ~insn[0]) |
				(~insn[4] & ~insn[3] & insn[2] & insn[1] & ~insn[0]) |
				(~insn[4] & ~insn[3] & insn[2] & insn[1] & insn[0]) |
				(~insn[4] & insn[3] & ~insn[2] & ~insn[1] & ~insn[0]);			
endmodule

module readValueInExecute(in, out);
	input [4:0] in;
	output out;
	
	assign out = ~in[4] & ~in[3] & ~in[2] & ~in[1] & ~in[0]   |    // r type
	             ~in[4] & ~in[3] &  in[2] &  ~in[1] & in[0]   |    // addi
	             ~in[4] & ~in[3] & ~in[2] &   in[1] & ~in[0]  |    // bne
	             ~in[4] & ~in[3] & in[2] &    in[1] & ~in[0]  |    // blt
	             ~in[4] & ~in[3] & in[2] &    ~in[1] & ~in[0] |    // jump return
	             ~in[4] &  in[3] & ~in[2] &   ~in[1] & ~in[0] |     // lw
	             ~in[4] & ~in[3] & in[2] &   in[1] & in[0];         // sw

endmodule

module writeValueInMemory(in, out);
	input [4:0] in;
	output out;
	
	assign out =  ~in[4] & ~in[3] & ~in[2] & ~in[1] & ~in[0] |    // r type
	              ~in[4] & ~in[3] & in[2] &  ~in[1] & in[0];      // addi

endmodule

module writeValueInWriteback(in, out);
	input [4:0] in;
	output out;
	
	assign out =  ~in[4] & ~in[3] & ~in[2] & ~in[1] & ~in[0] |    // r type
	              ~in[4] & ~in[3] & in[2] &  ~in[1] & in[0]  |      // addi
	              ~in[4] &  in[3] & ~in[2] &   ~in[1] & ~in[0];     // lw
endmodule

module compareStatusToZero(in, gt);
	input [26:0] in;
	output gt;
	
	assign gt = in[26] | in[25] | in[24] | in[23] | in[22] | in[21] | in[20] | in[19] | in[18] | in[17] |
	            in[16] | in[15] | in[14] | in[13] | in[12] | in[11] | in[10] | in[9] | in[8] | in[7] |
	            in[6]  | in[5]  | in[4]  | in[3]  | in[2]  | in[1]  | in[0];
	            
endmodule
	
	

module signExtend(in, out);
	input [16:0] in;
	output [31:0] out;

	genvar i;
	generate
		for (i = 0; i <= 16; i = i + 1) begin: loop1
			assign out[i] = in[i];
		end
	endgenerate
	
	genvar c;
	generate
		for (c = 17; c <= 31; c = c + 1) begin: loop2
			assign out[c] = in[16];
		end
	endgenerate


endmodule

module signExtend26(in, out);
	input [26:0] in;
	output [31:0] out;

	genvar i;
	generate
		for (i = 0; i <= 26; i = i + 1) begin: loop1
			assign out[i] = in[i];
		end
	endgenerate
	
	genvar c;
	generate
		for (c = 27; c <= 31; c = c + 1) begin: loop2
			assign out[c] = in[16];
		end
	endgenerate


endmodule

module regFile_sdh31(clock, ctrl_writeEnable, ctrl_reset, ctrl_writeReg, ctrl_readRegA, ctrl_readRegB, 
                     data_writeReg, data_readRegA, data_readRegB, registerOneOutput, registerTwoOutput,
                     ctrl_write_reg_one, ctrl_write_reg_two, data_write_reg_one, data_write_reg_two,
							registerThreeOutput, registerFourOutput, registerFiveOutput, registerSixOutput,
							registerSevenOutput, registerTenOutput,
							register11Output, collision, endGameCollision, lives, frogInsideLog);
                     
	input clock, ctrl_writeEnable, ctrl_reset, collision, endGameCollision, frogInsideLog;
	input [4:0] ctrl_writeReg, ctrl_readRegA, ctrl_readRegB;
	input [31:0] data_writeReg;
	output [31:0] data_readRegA, data_readRegB;  
	
	output [31:0] registerOneOutput, registerTwoOutput, registerThreeOutput,
						registerFourOutput, registerFiveOutput, registerSixOutput, registerSevenOutput, registerTenOutput,
						register11Output, lives;  // easy to access outputs for R1 and R2, for our frogger :)
	input [31:0] data_write_reg_one, data_write_reg_two;
	input ctrl_write_reg_one, ctrl_write_reg_two;
	
	     
	wire [31:0] regOneControl;
	assign regOneControl = 32'b00000000000000000000000000000010;
	wire [31:0] regTwoControl;
	assign regTwoControl = 32'b00000000000000000000000000000100;
	wire [31:0] regThreeControl;
	assign regThreeControl = 32'b00000000000000000000000000001000;
	wire [31:0] regFourControl;
	assign regFourControl =  32'b00000000000000000000000000010000;
	wire [31:0] regFiveControl;
	assign regFiveControl =  32'b00000000000000000000000000100000;
	wire [31:0] regSixControl;
	assign regSixControl =  32'b00000000000000000000000001000000;
	wire [31:0] regSevenControl;
	assign regSevenControl =  32'b00000000000000000000000010000000;
	wire [31:0] regTenControl;
	assign regTenControl = 32'b00000000000000000000010000000000;
	
	wire [31:0] reg11Control;
	assign reg11Control = 32'b00000000000000000000100000000000;
	
	wire [31:0] reg28Control;
	assign reg28Control =  32'b00010000000000000000000000000000;
	wire [31:0] reg23Control;
	assign reg23Control =  32'b00000000100000000000000000000000;
	
         
    wire [31:0] decoded_readRegA;
    wire [31:0] decoded_readRegB;
    wire [31:0] decoded_writeReg;
    Decoder5To32 decode1(.in(ctrl_readRegA), .out(decoded_readRegA));
    Decoder5To32 decode2(.in(ctrl_readRegB), .out(decoded_readRegB));
    Decoder5To32 decode3(.in(ctrl_writeReg), .out(decoded_writeReg));
    
   wire [31:0] collisionInput;
	assign collisionInput[31:3] = 29'b0;
	assign collisionInput[2] = frogInsideLog;
	assign collisionInput[1] = endGameCollision;
	assign collisionInput[0] = collision;
	 
	genvar i;
	generate
	for (i=0; i<32; i=i+1) begin: loop1
		wire [31:0] dff32bitout;
		
		wire enableWrite;
		
		// enabling write on a normal write to the register ... or, special write to R1 or R2
		assign enableWrite = (decoded_writeReg[i] & ctrl_writeEnable) |  // normal write
		                     (regOneControl[i] & ctrl_write_reg_one & ~decoded_writeReg[i])  |  // special write to reg one
		                     (regTwoControl[i] & ctrl_write_reg_two & ~decoded_writeReg[i]) |
									(reg28Control[i] & ~ctrl_writeEnable & ~ctrl_write_reg_one & ~ctrl_write_reg_two);		
			
		
		wire [31:0] writeDataInput;

		assign writeDataInput = ({32{(ctrl_writeEnable & decoded_writeReg[i])}} & data_writeReg) |   // normal write data input
								({32{regOneControl[i] & ctrl_write_reg_one & ~decoded_writeReg[i]}} & data_write_reg_one) | // special write to reg 1
								({32{regTwoControl[i] & ctrl_write_reg_two & ~decoded_writeReg[i]}} & data_write_reg_two) |
								({32{reg28Control[i]}} & collisionInput);
						
		DFF32Bit a_dff32(.clock(clock), .write_enable(enableWrite), .dataInput(writeDataInput), 
						.reset(ctrl_reset), .dataOutput(dff32bitout));
							
		TriState32Bit a_tristate(.enable(decoded_readRegA[i]), .out(data_readRegA), .in(dff32bitout));
		TriState32Bit b_tristate(.enable(decoded_readRegB[i]), .out(data_readRegB), .in(dff32bitout));
		
		// giving us direct outputs to register 1 and register 2
		TriState32Bit reg1(.enable(regOneControl[i]), .out(registerOneOutput), .in(dff32bitout));
		TriState32Bit reg2(.enable(regTwoControl[i]), .out(registerTwoOutput), .in(dff32bitout));
		TriState32Bit reg3(.enable(regThreeControl[i]), .out(registerThreeOutput), .in(dff32bitout));
		TriState32Bit reg4(.enable(regFourControl[i]), .out(registerFourOutput), .in(dff32bitout));
		TriState32Bit reg5(.enable(regFiveControl[i]), .out(registerFiveOutput), .in(dff32bitout));
		TriState32Bit reg6(.enable(regSixControl[i]), .out(registerSixOutput), .in(dff32bitout));
		TriState32Bit reg7(.enable(regSevenControl[i]), .out(registerSevenOutput), .in(dff32bitout));
		TriState32Bit reg10(.enable(regTenControl[i]), .out(registerTenOutput), .in(dff32bitout));
		TriState32Bit reg11(.enable(reg11Control[i]), .out(register11Output), .in(dff32bitout));
		TriState32Bit reg23(.enable(reg23Control[i]), .out(lives), .in(dff32bitout));
		
	end
	endgenerate
     
endmodule

module TriState32Bit(in, enable, out);
	input [31:0] in;
	input enable;
	output [31:0] out;

	assign out = enable ? in : 32'bz;
endmodule

module DFF32Bit(clock, write_enable, dataInput, reset, dataOutput);
	input clock, reset, write_enable;
	input [31:0] dataInput;
	output [31:0] dataOutput;
	
	genvar i;
	generate
	for (i=0; i<32; i=i+1) begin: loop1
		DFFE a_dffe(.clk(clock), .d(dataInput[i]), .clrn(~reset), .ena(write_enable), .q(dataOutput[i]));
	end
	endgenerate
endmodule

module Decoder5To32(in, out);
	input [4:0] in;
	output [31:0] out;

	assign out[0] = ~in[4] & ~in[3] & ~in[2] & ~in[1] & ~in[0];
	assign out[1] = ~in[4] & ~in[3] & ~in[2] & ~in[1] & in[0];
	assign out[2] = ~in[4] & ~in[3] & ~in[2] & in[1] & ~in[0];
	assign out[3] = ~in[4] & ~in[3] & ~in[2] & in[1] & in[0];
	
	assign out[4] = ~in[4] & ~in[3] & in[2] & ~in[1] & ~in[0];
	assign out[5] = ~in[4] & ~in[3] & in[2] & ~in[1] & in[0];
	assign out[6] = ~in[4] & ~in[3] & in[2] & in[1] & ~in[0];
	assign out[7] = ~in[4] & ~in[3] & in[2] & in[1] & in[0];
	
	assign out[8] = ~in[4] & in[3] & ~in[2] & ~in[1] & ~in[0];
	assign out[9] = ~in[4] & in[3] & ~in[2] & ~in[1] & in[0];
	assign out[10] = ~in[4] & in[3] & ~in[2] & in[1] & ~in[0];
	assign out[11] = ~in[4] & in[3] & ~in[2] & in[1] & in[0];
	
	assign out[12] = ~in[4] & in[3] & in[2] & ~in[1] & ~in[0];
	assign out[13] = ~in[4] & in[3] & in[2] & ~in[1] & in[0];
	assign out[14] = ~in[4] & in[3] & in[2] & in[1] & ~in[0];
	assign out[15] = ~in[4] & in[3] & in[2] & in[1] & in[0];
	
	assign out[16] = in[4] & ~in[3] & ~in[2] & ~in[1] & ~in[0];
	assign out[17] = in[4] & ~in[3] & ~in[2] & ~in[1] & in[0];
	assign out[18] = in[4] & ~in[3] & ~in[2] & in[1] & ~in[0];
	assign out[19] = in[4] & ~in[3] & ~in[2] & in[1] & in[0];
	
	assign out[20] = in[4] & ~in[3] & in[2] & ~in[1] & ~in[0];
	assign out[21] = in[4] & ~in[3] & in[2] & ~in[1] & in[0];
	assign out[22] = in[4] & ~in[3] & in[2] & in[1] & ~in[0];
	assign out[23] = in[4] & ~in[3] & in[2] & in[1] & in[0];
	
	assign out[24] = in[4] & in[3] & ~in[2] & ~in[1] & ~in[0];
	assign out[25] = in[4] & in[3] & ~in[2] & ~in[1] & in[0];
	assign out[26] = in[4] & in[3] & ~in[2] & in[1] & ~in[0];
	assign out[27] = in[4] & in[3] & ~in[2] & in[1] & in[0];
	
	assign out[28] = in[4] & in[3] & in[2] & ~in[1] & ~in[0];
	assign out[29] = in[4] & in[3] & in[2] & ~in[1] & in[0];
	assign out[30] = in[4] & in[3] & in[2] & in[1] & ~in[0];
	assign out[31] = in[4] & in[3] & in[2] & in[1] & in[0];
endmodule

module alu_sdh31(data_operandA, data_operandB, ctrl_ALUopcode, ctrl_shiftamt, data_result, isNotEqual, isLessThan);

	input [31:0] data_operandA, data_operandB;
	input [4:0] ctrl_ALUopcode, ctrl_shiftamt;
	output [31:0] data_result;
	output isNotEqual, isLessThan;

	wire [31:0] additionOutput;
	wire [31:0] subtractionOutput;
	wire [31:0] andOutput;
	wire [31:0] orOutput;
	wire [31:0] sslOutput;
	wire [31:0] sraOutput;
	
	
	// 0 - ADDITION
	CarryLookaheadAdder2 adder(.x(data_operandA), .y(data_operandB), .sum(additionOutput), .subtractEnable(1'b0));
	
	// 1 - SUBTRACTION
	CarryLookaheadAdder2 subtracter(.x(data_operandA), .y(data_operandB), .sum(subtractionOutput), .subtractEnable(1'b1));
	
	NotEqualZero neqz(.x(subtractionOutput), .out(isNotEqual));
	
	assign isLessThan = subtractionOutput[31] & 1;
	
	// 2 - AND
	assign andOutput = data_operandA & data_operandB;
	// 3 - OR
	assign orOutput = data_operandA | data_operandB;
	
	// 4 - SSL
	ShiftLeftLogical SSL(.in(data_operandA), .shamt(ctrl_shiftamt), .out(sslOutput));
	
	// 5 - SRA
	ShiftRightArithmetic SRA(.in(data_operandA), .shamt(ctrl_shiftamt), .out(sraOutput));
	
	// 6 and 7
	
	wire [31:0] mux0_out;
	wire [31:0] mux1_out;
	wire [31:0] mux2_out;
	wire [31:0] mux3_out;
	wire [31:0] mux4_out;

    Mux32Bit mux0(.A(additionOutput), .B(subtractionOutput), .Enable(ctrl_ALUopcode[0]), .Output(mux0_out));
    Mux32Bit mux1(.A(andOutput), .B(orOutput), .Enable(ctrl_ALUopcode[0]), .Output(mux1_out));
    Mux32Bit mux2(.A(sslOutput), .B(sraOutput), .Enable(ctrl_ALUopcode[0]), .Output(mux2_out));
    
    Mux32Bit mux3(.A(mux0_out), .B(mux1_out), .Enable(ctrl_ALUopcode[1]), .Output(mux3_out));
    Mux32Bit mux4(.A(mux2_out), .B(32'b0), .Enable(ctrl_ALUopcode[1]), .Output(mux4_out));
    
    Mux32Bit mux5(.A(mux3_out), .B(mux4_out), .Enable(ctrl_ALUopcode[2]), .Output(data_result));
    
endmodule

module CarryLookaheadAdder2(x, y, sum, subtractEnable);
	input [31:0] x, y;
	input subtractEnable;
	output [31:0] sum;

	wire [31:0] temp_y;

	genvar k;
	generate
		for (k = 0; k <= 31; k = k + 1) begin: loop0
			xor(temp_y[k], subtractEnable, y[k]);
		end
	endgenerate

	wire P_0, G_0, c_0;	
	assign c_0 = subtractEnable;
	wire P_1, G_1, c_8;
	assign c_8 = G_0 | (P_0 & c_0);
	wire P_2, G_2, c_16;
	assign c_16 = G_1 | (P_1 & G_0) | (P_1 & P_0 & c_0);
	wire P_3, G_3, c_24;
	assign c_24 = G_2 | (P_2 & G_1) | (P_2 & P_1 & G_0) | (P_2  & P_1 & P_0 & c_0); 
	wire c_32;
	assign c_32 = G_3 | (P_3 & G_2) | (P_3 & P_2 & G_1) | (P_3  & P_2 & P_1 & G_0) | (P_3 & P_2 & P_1 & P_0 & c_0); 
	
	EightBitCLA Block0(.x(x[7:0]), .y(temp_y[7:0]), .c_in(c_0), .BlockPropagate(P_0), .BlockGenerate(G_0), .sum(sum[7:0]));
	EightBitCLA Block1(.x(x[15:8]), .y(temp_y[15:8]), .c_in(c_8), .BlockPropagate(P_1), .BlockGenerate(G_1), .sum(sum[15:8]));
	EightBitCLA Block2(.x(x[23:16]), .y(temp_y[23:16]), .c_in(c_16), .BlockPropagate(P_2), .BlockGenerate(G_2), .sum(sum[23:16]));
	EightBitCLA Block3(.x(x[31:24]), .y(temp_y[31:24]), .c_in(c_24), .BlockPropagate(P_3), .BlockGenerate(G_3), .sum(sum[31:24]));

endmodule

module EightBitCLA(x, y, c_in, BlockPropagate, BlockGenerate, sum);
	input [7:0] x, y;
	input c_in;
	output BlockPropagate;
	output BlockGenerate;
	output [7:0] sum;

	wire [7:0] g;
	wire [7:0] p;
	wire [7:0] c;

	assign c[0] = c_in;
	assign g[0] = x[0] & y[0];
	assign p[0] = x[0] | y[0];
	genvar k;
	generate
	for (k = 1; k <= 7; k = k + 1)  begin: loop1
		assign g[k] = x[k] & y[k];
		assign p[k] = x[k] | y[k];
		assign c[k] = g[k-1] | (p[k-1] & c[k-1]); 
	end
	endgenerate
	
	
	assign BlockPropagate = p[7] & p[6] & p[5] &  p[4] & p[3] & p[2] & p[1] & p[0];
	assign BlockGenerate = g[7] | (p[7] & g[6]) | (p[7] & p[6] & g[5]) | (p[7] & p[6] & p[5] & g[4]) | (p[7] & p[6] & p[5] & p[4] & g[3]) |
					(p[7] & p[6] & p[5] & p[4] & p[3] & g[2]) |  (p[7] & p[6] & p[5] & p[4] & p[3] & p[2] & g[1]) |
					(p[7] & p[6] & p[5] & p[4] & p[3] & p[2] & p[1] & g[0]);
					
	
	
	genvar i;
	generate
		for (i = 0; i <= 7; i = i + 1) begin: loop2
			ThreeBitXOR threeBitXOR(.x(x[i]), .y(y[i]), .z(c[i]), .out(sum[i]));
		end
	endgenerate

endmodule

module ShiftLeftLogical(in, shamt, out);
	input [31:0] in;
	input [4:0] shamt;
	output [31:0] out;
	
	
	wire [31:0] temp1;
	wire [31:0] temp2;
	wire [31:0] temp3;
	wire [31:0] temp4;
	
	OneBitLeftShifter(.in(in), .out(temp1), .enable(shamt[0]));
	TwoBitLeftShifter(.in(temp1), .out(temp2), .enable(shamt[1]));
	FourBitLeftShifter(.in(temp2), .out(temp3), .enable(shamt[2]));
	EightBitLeftShifter(.in(temp3), .out(temp4), .enable(shamt[3]));
	SixteenBitLeftShifter(.in(temp4), .out(out), .enable(shamt[4]));
	
endmodule

module OneBitLeftShifter(in, out, enable);
	input [31:0] in;
	input enable;
	output [31:0] out;
	
	
	assign out[0] = in[0] & ~enable;
	
	genvar i;
	generate
		for (i = 1; i <= 31; i = i + 1) begin: loop1
			assign out[i] = (in[i-1] & enable) | (in[i] & ~enable);
		end
	endgenerate
	
endmodule


module TwoBitLeftShifter(in, out, enable);
	input [31:0] in;
	input enable;
	output [31:0] out;
	
	wire [31:0] temp;
	OneBitLeftShifter left1(.in(in), .out(temp), .enable(enable));
	OneBitLeftShifter left2(.in(temp), .out(out), .enable(enable));
	
endmodule

module FourBitLeftShifter(in, out, enable);
	input [31:0] in;
	input enable;
	output [31:0] out;
	
	wire [31:0] temp;
	TwoBitLeftShifter left1(.in(in), .out(temp), .enable(enable));
	TwoBitLeftShifter left2(.in(temp), .out(out), .enable(enable));
	
endmodule

module EightBitLeftShifter(in, out, enable);
	input [31:0] in;
	input enable;
	output [31:0] out;
	
	wire [31:0] temp;
	FourBitLeftShifter left1(.in(in), .out(temp), .enable(enable));
	FourBitLeftShifter left2(.in(temp), .out(out), .enable(enable));
	
endmodule

module SixteenBitLeftShifter(in, out, enable);
	input [31:0] in;
	input enable;
	output [31:0] out;
	
	wire [31:0] temp;
	EightBitLeftShifter left1(.in(in), .out(temp), .enable(enable));
	EightBitLeftShifter left2(.in(temp), .out(out), .enable(enable));
	
endmodule

module ShiftRightArithmetic(in, shamt, out);
	input [31:0] in;
	input [4:0] shamt;
	output [31:0] out;
	
	
	wire [31:0] temp1;
	wire [31:0] temp2;
	wire [31:0] temp3;
	wire [31:0] temp4;
	
	OneBitRightShifter(.in(in), .out(temp1), .enable(shamt[0]));
	TwoBitRightShifter(.in(temp1), .out(temp2), .enable(shamt[1]));
	FourBitRightShifter(.in(temp2), .out(temp3), .enable(shamt[2]));
	EightBitRightShifter(.in(temp3), .out(temp4), .enable(shamt[3]));
	SixteenBitRightShifter(.in(temp4), .out(out), .enable(shamt[4]));
	
endmodule

module OneBitRightShifter(in, out, enable);
	input [31:0] in;
	input enable;
	output [31:0] out;
	
	assign out[31] = in[31];
	
	genvar i;
	generate
		for (i = 30; i >= 0; i = i - 1) begin: loop1
			assign out[i] = (in[i+1] & enable) | (in[i] & ~enable);
		end
	endgenerate
	
endmodule


module TwoBitRightShifter(in, out, enable);
	input [31:0] in;
	input enable;
	output [31:0] out;
	
	wire [31:0] temp;
	OneBitRightShifter right1(.in(in), .out(temp), .enable(enable));
	OneBitRightShifter right2(.in(temp), .out(out), .enable(enable));
	
endmodule

module FourBitRightShifter(in, out, enable);
	input [31:0] in;
	input enable;
	output [31:0] out;
	
	wire [31:0] temp;
	TwoBitRightShifter right1(.in(in), .out(temp), .enable(enable));
	TwoBitRightShifter right2(.in(temp), .out(out), .enable(enable));
	
endmodule

module EightBitRightShifter(in, out, enable);
	input [31:0] in;
	input enable;
	output [31:0] out;
	
	wire [31:0] temp;
	FourBitRightShifter right1(.in(in), .out(temp), .enable(enable));
	FourBitRightShifter right2(.in(temp), .out(out), .enable(enable));
	
endmodule

module SixteenBitRightShifter(in, out, enable);
	input [31:0] in;
	input enable;
	output [31:0] out;
	
	wire [31:0] temp;
	EightBitRightShifter right1(.in(in), .out(temp), .enable(enable));
	EightBitRightShifter right2(.in(temp), .out(out), .enable(enable));
	
endmodule

module Mux32Bit(A, B, Enable, Output);
        input [31:0] A, B;
        input Enable;
        output [31:0] Output;
        assign Output = (B & {32{Enable}} | (A & ~{32{Enable}}));
endmodule

module ThreeBitXOR(x, y, z, out);
	input x, y, z;
	output out;
	
	wire temp;
	
	xor(temp, x, y);
	xor(out, temp, z);
endmodule

module NotEqualZero(x, out);
	input [31:0] x;
	output out;

	
	assign out = x[0] | x[1] | x[2] | x[3] | x[4] | x[5] | x[6] | x[7] | x[8] | x[9] | x[10] | x[11] |
	             x[12] | x[13] | x[14] | x[15] | x[16] | x[17] | x[18] | x[19] | x[20] | x[21] | x[22] |
	             x[23] | x[24] | x[25] | x[26] | x[27] | x[28] | x[29] | x[30] | x[31];

endmodule

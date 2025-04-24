											   


// Define global variables
reg [2:0]current_state;
reg [2:0]next_state;
reg [15:0]IR;
			 


								 

module TheAllSystem (	   // < ------------
    input clk,
    input reset,
	
	//For Instruction Decode
	output reg [15:0] muxOut, // (Bus W) Output data
	output reg [15:0] outputOfRegFilesrc1, // (Bus A )Output data 
     output reg [15:0] outputOfRegFilesrc2, // (Bus B) Output data -	
	output reg [8:0] offset_9bit,

	
	//ALU result 				 
	
	output reg[15:0] ALUResult, //for verification
	
	//ExtenderResult
	output reg[15:0]extended_immediate, 
	reg[5:0] immediate_6bit,
	
    output reg PCSrc0, // sel 1
    output reg PCSrc1,// sel 2 	  
    output reg PCSrc2,// sel 3
    output reg regFilesrc1 ,//first input of a register file 
	output reg regFilesrc2 ,//second input o# Error: VCP2000 datapath.v : (39, 25): Syntax error. Unexpected token: ;.f a register file 
	output reg regDest1,	 
	output reg regDest2, 	
	output reg RegWrite,
	output reg extension_signal,
	output reg ALUSrc1,   
	output reg ALUSrc2, 
	output reg [2:0] ALUOp,
	output reg MemWrite,
	output reg MemRead,
	output reg WriteBack1,
	output reg WriteBack2,	
	output reg [2:0] Function,	  
	output reg [15:0] numOfInst,
	output reg [15:0] numOfLoadInst, 
	output reg [15:0] numOfStoreInst,
	output reg [15:0] numOfAluInst,
	output reg [15:0] numOf_ForInst, 
	output reg [15:0] numOf_CallInst,
	output reg [15:0] numOf_BranchInst,
	output reg [15:0] numOf_JumpInst,
	output reg [15:0] numOf_RetInst,
    output reg [15:0] numOfClkCycle,	 
	output reg [15:0] PC
	


	
			 

);	
//****************************************************************************************
		/// Instruction Fetch

   	 reg [15:0] Imemory [0:31];	  	
	 reg [15:0] Registers[0:9];
	 reg [15:0] DataMemory [63:0];
	 reg [15:0] address,data_in;
     reg [15:0] data_out;
	 reg [15:0] RR;
	 reg [3:0] outputOfDestRegMux;	   
	 reg halted;


	 
	 		
	 //ID
	 reg [3:0] OP_CODE; 	
	 reg [2:0] AddressToRd;	 // Address for Rd
	 reg [2:0] AddressToRs;  // Address for Rs
	 reg [2:0] AddressToRt;  // Address for Rt  	 
  

	 	 
		 
	///ALU Part
	reg zero; // Zero flag indicating if the result is zero	 
	reg forSignal;  

	
	reg [15:0] output_Of_ALUsrc1_mux; 
	reg [15:0] output_Of_ALUsrc2_mux;
	reg[15:0] OperandA;	
	reg[15:0] OperandB;
				   	
	reg[15:0] outputOfBTA;	
	reg[15:0] addressOfReturnAddress; 
	reg[15:0] nextAddress_of_for_inst;
	/////////////////////////////////////////////////////////////////
	
	
	
	 
    // State definitions
    parameter IF = 0, ID = 1, EX = 2, MEM = 3, WB = 4;
    reg [4:0] flag; //
	

	
	

	initial begin	 
		DataMemory[7] = 16'h0009;
		 DataMemory[8] = 16'h0004;
		 DataMemory[12] = 16'h0007;  
		 DataMemory[14] = 16'h0006;
		 DataMemory[15] = 16'h0006;
		 DataMemory[16] = 16'h000A;
		 DataMemory[17] = 16'h0006;
		 DataMemory[27] = 16'h000f;
		 DataMemory[33] = 16'h0006;  
		 DataMemory[35] = 16'h0007;  
		 DataMemory[36] = 16'h0004;  
				  end
	initial begin	 
		
			
		//Program 1
           Imemory[0] = 16'b0000001010111000; // AND   ->  reg(1) =reg(2)& reg(7) -> reg(1)= 3 & 9 -> reg(1) = 1
		   Imemory[1] = 16'b0000010001101001; // ADD   -> reg(2) = reg(1) + reg(5) -> reg(2) = 1+7	-> reg(2) = 8
	 	   Imemory[2] = 16'b0000111010011010;	//SUB -> reg(7) = reg(2) - reg(3) ->  reg(7) = 8-4 -> reg(7) = 4
	       Imemory[3] = 16'b0000011110001011;  //SLL -> (reg(3) = reg(6) << reg(1)) -> (reg(3) = 8 << 1) -> reg(3) = 16	(10 in hexa)						    
		   Imemory[4] = 16'b0000101111001100; // SRL -> reg(5) = reg(7) >> reg(1)	-> reg(5) = 4 >> 1 -> reg(5) = 2
		   
		 
		   Imemory[5] = 16'b1000000101000000;  // for reg(0),reg(5) Rs is 0 and Rt is 2	so loop twice		
			   
			   //the values after the next iteration
			   //reg(1) =reg(2)& reg(7)	-> reg(1) = 8 & 4 = 0
			   //reg(2) = reg(1) + reg(5) -> reg(2) = 0 + 2 = 2
			   //reg(7) = reg(2) - reg(3) -> reg(7) = 2 - 16 = -14 (fffe in hexa)	 
			   //reg(3) = reg(6) << reg(1) -> reg(3) = 8 << 0 = 8 
			   //reg(5) = reg(7) >> reg(1) -> reg(5) = -14 >> 0 = -14 (fffe in hexa)
			   
		   Imemory[6] = 16'b0110110011000010;  // BEQ if(reg(6) == reg(3)) nextPC = BTA else : nextPC=PC+1 (note:imm=2)
		 
		   Imemory[7] = 16'b0011010111001001;	 //	ADDI -> reg(7) = reg(2) + 9 -> reg(7) = 2 + 9 = 11 (b in hexa) (if the branch is not taken but here it is taken) -> it is skipped
				 
		   Imemory[8] = 16'b0100011100001000; // LW -> reg(4) = mem(reg(3)+8) -> reg(4) = mem(8+8) = mem(16) = 10 (A in hexa)
		  
		   Imemory[9] = 16'b0101001101000111;  //SW -> mem(reg(1)+7) = reg(5) -> mem(0+7)= mem(7) -> mem(7) = reg(5) -> mem(7) = fff2
		   
		
		    Imemory[10]= 16'b0010010101000110; //   ANDI	->  reg(5) =reg(2)& 6 -> reg(5) = 2 & 6 = 2 - > reg(5) = 2
		   						 		
			Imemory[11]= 16'b0111000001000011;    //   BNE if(reg(0) != reg(1)) nextPC = BTA else : nextPC=PC+1 (note:imm=3)  
				
		    Imemory[12] = 16'b0000001010111000; // AND   ->  reg(1) =reg(2)& reg(7) -> reg(1)= 2 & fff2 -> reg(1) = 2 (the branch is not taken)
		    Imemory[13] = 16'b0000010001101001; // ADD   -> reg(2) = reg(1) + reg(5) -> reg(2) = 2+2	-> reg(2) = 4 (the branch is not taken)
		
		
	
		    Imemory[14]= 16'b0001000010001000;  // JMP offset nextPC = jumptarget (offset = 17),  nextPC = {PC[15:9],offset} = 17
			
			Imemory[15] = 16'b0011010111001001;	 //	ADDI (skipped because of jump instruction)
		   	Imemory[16] = 16'b0000011110001011;  //SLL (skipped because of jump instruction)
			   
		    Imemory[17]=  16'b0001000010101001;  //  CALL offset nextPC = jumptarget ,PC + 1 stored on the RR (offset = 21),  nextPC = {PC[15:9],offset} = 21		
		    Imemory[18] = 16'b0000001010111000; // AND   ->  reg(1) =reg(2)& reg(7) -> reg(1)= 4 & d -> reg(1) = 4
		    Imemory[19] = 16'b0000011001101001; // ADD   -> reg(3) = reg(1) + reg(5) -> reg(3) = 4+2	-> reg(3) = 6 (consider it the last instruction) 
			Imemory[20] = 16'b1111111111111111;	// exit the program
			Imemory[21] = 16'b0011010111001001;	 //	ADDI -> reg(7) = reg(2) + 9 -> reg(7) = 4 + 9 = 13 (d in hexa)	
		   
		    Imemory[22]=  16'b0001000000000010;  //  RET nextPC = value of the RR ,the 9-bit field is ignored in this instruction (offset = 1) it will return to address 18	 
				   
			
			
			/*
			
			//Program 2
            Imemory[0] = 16'b0000001010111000; // AND   ->  reg(1) =reg(2)& reg(7) -> reg(1)= 3 & 9 -> reg(1) = 1
		   Imemory[1] = 16'b0000010001101001; // ADD   -> reg(2) = reg(1) + reg(5) -> reg(2) = 1+7	-> reg(2) = 8
	 	   Imemory[2] = 16'b0100011100001000;	// LW -> reg(4) = mem(reg(3)+8) -> reg(4) = mem(4+8) = mem(12) = 7 
	       Imemory[3] = 16'b0101001101000111;  //SW -> mem(reg(1)+7) = reg(5) -> mem(1+7)= mem(8) -> mem(8) = reg(5) -> mem(8) = 7							    
		   Imemory[4] = 16'b0000101011001100; // SRL -> reg(5) = reg(3) >> reg(1)	-> reg(5) = 4 >> 1 -> reg(5) = 2
		   
		 
		   Imemory[5] = 16'b1000000101000000;  // for reg(0),reg(5) Rs is 0 and Rt is 2	so loop twice		
			   
			   //the values after the next iteration
			   //reg(1) =reg(2)& reg(7)	-> reg(1) = 8 & 9 = 8
			   //reg(2) = reg(1) + reg(5) -> reg(2) = 8 + 2 = 10(A in hexa)
			   //reg(4) = mem(reg(3)+8) -> reg(4) = mem(4+8) = mem(12) = 7 	 
			   //mem(reg(1)+7) = reg(5) -> mem(8+7)= mem(15) -> mem(15) = reg(5) -> mem(15) = 2
			   //reg(5) = reg(3) >> reg(1) -> reg(5) = 4 >> 8 = 0 
			   
		   Imemory[6] = 16'b0110110011000010;  // BEQ if(reg(6) == reg(3)) nextPC = BTA else : nextPC=PC+1 (note:imm=2 ) ( the branch is not taken 4 != 8)
		 
		  Imemory[7] = 16'b0011010111001001;	 //	ADDI -> reg(7) = reg(2) + 9 -> reg(7) = 10 + 9 = 19 (13 in hexa) 
				 
		   Imemory[8] = 16'b0100111100001000; // LW -> reg(4) = mem(reg(7)+8) -> reg(4) = mem(19+8) = mem(27) = f
		  
		   Imemory[9] = 16'b0101001101000111;  //SW -> mem(reg(1)+7) = reg(5) -> mem(8+7)= mem(15) -> mem(15) = reg(5) -> mem(15) = 0
		   
		
		    Imemory[10]= 16'b0010010101000110; //   ANDI	->  reg(5) =reg(2)& 6 -> reg(5) = 10 & 6 = 4 - > reg(5) = 2
		   						 		
			Imemory[11]= 16'b0111000001000011;    //   BNE if(reg(0) != reg(1)) nextPC = BTA else : nextPC=PC+1 (note:imm=3)  (it is taken 8 != 0)
				
		    Imemory[12] = 16'b0000001010111000; // AND   ->  reg(1) =reg(2)& reg(7) -> reg(1)= A & 13 -> reg(1) = 2 (skipped) 
		    Imemory[13] = 16'b0000010001101001; // ADD   -> reg(2) = reg(1) + reg(5) -> reg(2) = 8+2	-> reg(2) = 10(a)  (skipped)
		
		
	
		    Imemory[14]= 16'b0001000010001000;  // JMP offset nextPC = jumptarget (offset = 17),  nextPC = {PC[15:9],offset} = 17
			
			Imemory[15] = 16'b0011010111001001;	 //	ADDI (skipped because of jump instruction)
		   	Imemory[16] = 16'b0000011110001011;  //SLL (skipped because of jump instruction)
			   
		    Imemory[17]=  16'b0001000010101001;  //  CALL offset nextPC = jumptarget ,PC + 1 stored on the RR (offset = 20),  nextPC = {PC[15:9],offset} = 21		
		    Imemory[18] = 16'b0000001010111000; // AND   ->  reg(1) =reg(2)& reg(7) -> reg(1)= a & 13 -> reg(1) = 2
		    Imemory[19] = 16'b0000011001101001; // ADD   -> reg(3) = reg(1) + reg(5) -> reg(3) = 2+2	-> reg(3) = 4 (consider it the last instruction) 
			Imemory[20] = 16'b1111111111111111;	// exit the program
			Imemory[21] = 16'b0011010111001001;	 //	ADDI -> reg(7) = reg(2) + 9 -> reg(7) = A + 9 = 19 (13 in hexa)	
		   
		    Imemory[22]=  16'b0001000000000010;  //  RET nextPC = value of the RR ,the 9-bit field is ignored in this instruction (offset = 1) it will return to address 18	 
			
			
			*/
			
			
		   
		   
		end

		
		//*****************************************************************************************
          	initial begin		
		// 16 32-bit general-purpose registers: from R0 to R15. 
	  	   Registers[0] = 16'h0000;				
            Registers[1] = 16'h0002;
            Registers[2] = 16'h0003;
            Registers[3] = 16'h0004;
            Registers[4] = 16'h0005;
            Registers[5] = 16'h0007;
            Registers[6] = 16'h0008;
            Registers[7] = 16'h0009;  
		   Registers[8]= 16'h0000; 	  // to store the counter of the for loop
		   Registers[9]= 16'h0000; 	// to store the return address
          
		
			   end
		
		 //////////////////

	
//************************************************************************************************	
   integer j;
   always @(posedge clk) 
        case (current_state)
            IF: next_state = ID;
            ID: next_state = EX;
            EX: next_state = MEM;
            MEM: next_state = WB;
            WB: next_state = IF;
        endcase
    	 
		 
	always @(posedge clk) 
 		current_state = next_state;
		
		
    always @(posedge clk or posedge reset)  
	
        if (reset) begin
			
			// Put A reset Printing Statement Here
			 $display ("Before Entering Stage, We are Resetting Variables.\n"); 
			
            current_state = WB;	   
            PC = 16'h0000;	
		  outputOfDestRegMux = 4'b0000;         
		  outputOfRegFilesrc1 = 16'h0000;
		  outputOfRegFilesrc2 = 16'h0000; 
		  output_Of_ALUsrc1_mux = 16'h0000;		
		  output_Of_ALUsrc2_mux = 16'h0000; 
		  extended_immediate = 16'h0000;
            ALUResult = 16'h0000;	 
			outputOfBTA = 16'h0000;
			flag = 0;
			data_in= 16'h0000; 
			data_out= 16'h0000;
			address= 16'h0000;
			immediate_6bit= 0;   
			muxOut=0;  
			forSignal = 0; 
			halted = 0;		 
			numOfInst = 0;
	        numOfLoadInst = 0; 
	        numOfStoreInst = 0;
	        numOfAluInst = 0;
	        numOf_ForInst = 0; 
			numOf_CallInst =0;
	       numOf_BranchInst = 0;
	       numOf_JumpInst = 0;
	       numOf_RetInst = 0; 
		   numOfClkCycle = 0;	
	
		

     		
		end	   

		
//----------------------------------------------------------------------------------------		
		
       else if (current_state == IF) begin	 
		   	// Put A reset Printing Statement Here
			   $display ("\n\n---------------->We are in FETCH Stage.<---------------\n\n"); 	
			 
		   
		   		
//******************** mux4x1 ********************************** 
               if (!halted) begin
                case ({PCSrc0, PCSrc1,PCSrc2})
                   3'b000: PC = PC + 1;
                   3'b001: PC = {PC[15:9],offset_9bit}; // JMP   
		          3'b010: PC = outputOfBTA; 								
				 3'b011: PC = addressOfReturnAddress; 
				 3'b100: PC = nextAddress_of_for_inst;
				   
                endcase  
				end
         
		IR = Imemory[PC];   
		if (IR == 16'b1111111111111111) begin// HALT
			halted <= 1; 
			current_state = WB;
			
    end	
	   else begin	  
		   numOfInst = numOfInst + 1;	
	       numOfClkCycle = numOfClkCycle + 1;
		  end
 
	$display ("IR: %b", IR); 
	$display ("PC: %b", PC);  
	
	
		  		 end
		
//------------------------------------------------------------------------------------------


  else if (current_state == ID) begin	
	 $display ("\n\n---------------->We are in Decode Stage.<---------------\n\n"); 
	
	 OP_CODE = IR[15:12];  
	 

	
		   
	
		   
//########################################################################			   
		   // R-type
	if 	( OP_CODE == 4'b0000 )
	  begin
		  
		numOfAluInst = numOfAluInst + 1;
	 	AddressToRd = IR[11:9];  	  		 
		AddressToRs = IR[8:6];
		AddressToRt = IR[5:3];
		Function =  IR[2:0];	
		
	 $display ("OP: %b",  OP_CODE); 	  
	 $display ("function: %b",  Function); 	
		
		
	
		  
		// Control Signals  	
		
		  PCSrc0 = 0;  // PC = PC + 1
		  PCSrc1 = 0;	
		  PCSrc2 = 0;
           regFilesrc1 = 0; // Rs 
	      regFilesrc2 = 0; // Rt 
           regDest1 = 0;  // Rd
	      regDest2 = 0;	
	      RegWrite = 1;
	      ALUSrc1 = 0; // Rs   
	      ALUSrc2 = 0; //Rt 
	      MemWrite = 0;
		 MemRead = 0;
		 WriteBack1 = 0; // Alu out
	      WriteBack2 = 0;
		 extension_signal =1'bx;
		
		  
		
				 if (Function == 3'b000)	  begin
					ALUOp = 3'b000;	 // AND	 
				 end
				 
				else if (Function == 3'b001) begin 
					ALUOp = 3'b001; // ADD
					
				end	
				else if (Function == 3'b010) begin 
					ALUOp = 3'b010; // SUB
					
				end
				else if (Function == 3'b011) begin 
					ALUOp = 3'b011; // SLL
					
				end
				
				else
					ALUOp = 3'b100;  begin // SRL
	 
		 		 end	  
		  
				  
	   end  
//####################################################	 
	   	
  
	   // I-TYPE	

	else if ( OP_CODE >= 4'b0010 &&  OP_CODE <= 4'b1000)	 
		
	  begin	  				
		  
		AddressToRs = IR[11:9];  	  		  // we have to do this division for all types
		AddressToRt = IR[8:6];
		immediate_6bit = IR[5:0];
		 
					   
		  
	 if (OP_CODE == 4'b0010 || OP_CODE == 4'b0011)	  begin	   // ANDI & ADDI
  
		 // Control Signals 
		 
		 numOfAluInst = numOfAluInst + 1;
		  PCSrc0 = 0;  // PC = PC + 1
		  PCSrc1 = 0;	
		  PCSrc2 = 0;
           regFilesrc1 = 0; // Rs 
	      regFilesrc2 = 0; // Rt 
           regDest1 = 0;  // dest is Rt
	      regDest2 = 1;	
	      RegWrite = 1;
	      ALUSrc1 = 0; // Rs   
	      ALUSrc2 = 1; //immediate 
	      MemWrite = 0;
		 MemRead = 0;
		 WriteBack1 = 0; // Alu out
	      WriteBack2 = 0;	 

	 
	 	 if (OP_CODE == 4'b0010)	  begin
					ALUOp = 3'b000;	 // ANDi	 
					extension_signal = 1'b0;
				 end
				 
	 	else 
			begin 
					ALUOp = 3'b001; // ADDi
					extension_signal =1'b1;
					
			 end 
	 
	 	end	  
	 
	 
	 if (OP_CODE == 4'b0100 || OP_CODE == 4'b0101)	 // load and store
		 begin
		     // Control Signals  	  
			
		
		  PCSrc0 = 0;  // PC = PC + 1
		  PCSrc1 = 0;	 
		  PCSrc2 = 0;
           regFilesrc1 = 0; // Rs 
	      regFilesrc2 = 0; // Rt 
	      ALUSrc1 = 0; // Rs   
	      ALUSrc2 = 1; //immediate 
		 extension_signal=1'b1; 	 
		  ALUOp = 3'b001; // add  	 
			 
			 	 
			 		
		  if (OP_CODE == 4'b0100)	  begin // LW 	
			   numOfLoadInst = numOfLoadInst + 1;
			     MemRead= 1'b1;
				 MemWrite = 1'b0;   
				 RegWrite =  1'b1; 
				 WriteBack1 = 0; // Mem out
				 WriteBack2 = 1;
				 regDest1 = 0;  // dest is Rt
	               regDest2 = 1;	
				
				 end
				 
		 else	begin				// SW	   
			 
		    	 numOfStoreInst = numOfStoreInst + 1;
		 		 RegWrite =  1'b0;
				 MemWrite = 1'b1;   
				 MemRead =	1'b0; 
                 WriteBack1 = 1'bx; // we dont write back in the store instruction
	             WriteBack2 = 1'bx;	
				 regDest1 = 1'bx;  
	             regDest2 = 1'bx;	
	 
		 		 end	
			 
		 
		 end
	 
		 
		 if (OP_CODE == 4'b0110 || OP_CODE == 4'b0111)		// branches
			 begin		 
		 numOf_BranchInst = numOf_BranchInst + 1;
			
				 // Control Signals   
		
		  PCSrc0 = 1'b0;  // BTA
		  PCSrc1 = 1'b1;	  
		  PCSrc2 = 1'b0;
           regFilesrc1 = 1'b0; // Rs 
	      regFilesrc2 = 1'b0; // Rt 
           regDest1 = 1'bx;  // we dont need a dest reg in the branch
	      regDest2 = 1'bx;	
	      RegWrite = 1'b0;	// we dont write in a regFile
	      ALUSrc1 = 1'b0; // Rs   
	      ALUSrc2 = 1'b0; //Rt
	      MemWrite = 1'b0;
		 MemRead = 1'b0;
		 WriteBack1 = 1'bx; 
	      WriteBack2 = 1'bx;	  
		 extension_signal =1'b1;		 
		  ALUOp = 3'b010; 		 
			 
			 end  
			 
			 
	     
		if (OP_CODE == 4'b1000)		// for instruction
			begin					
		  numOf_ForInst = numOf_ForInst + 1;
			
				 // Control Signals   
		
		  PCSrc0 = 1;  //next instruction after for
           PCSrc1 = 0;	
		  PCSrc2 = 0;
           regFilesrc1 = 0; // Rs 
	      regFilesrc2 = forSignal; // if forsignal is 0 then Rt is the second src else it is the address of the special that contain the loop counter(forSignal is updated in the execution stage)
           regDest1 = 1;  // the destenation is the special reg at address 8 
	      regDest2 = 0;
	      RegWrite = 1;	
	      ALUSrc1 = 1; // Rt   
	      ALUSrc2 = 1; //imm = 1
	      MemWrite = 0;
		 MemRead = 0;
		 WriteBack1 = 0; // Alu out 
	      WriteBack2 = 0;	  
		 extension_signal =1'b1;		 
		 ALUOp = 3'b010; //sub (Rt - 1)	
		 immediate_6bit = 6'b000001;	// we subtract 1 from Rt so 1 is the socend operand
		 
			 
			 end  
			 
		
		 
	 
		 		 
			 if (extension_signal == 1'b0) begin
        		// Zero-extension (extend by 0's)
     			extended_immediate = {10'b0, immediate_6bit};
    		 end
    		 else begin
        		// Sign-extension
     		     extended_immediate = {{10{immediate_6bit[5]}}, immediate_6bit};

  			 end	
			
	  end
	  
//########################################################################	 																	  
	  
    // J-TYPE
	  
	else if 	( OP_CODE == 4'b0001)
	  begin	 
		  
		
		  offset_9bit = IR[11:3];	
		  Function = IR[2:0];  
		  
		  
	if  (Function == 3'b000) // <-- jmp
	 begin  		
		  numOf_JumpInst = numOf_JumpInst + 1;
		
				 // Control Signals   
		
		  PCSrc0 = 1'b0;  //jump address
           PCSrc1 = 1'b0;	
		  PCSrc2 = 1'b1;
           regFilesrc1 = 1'bx; 
	      regFilesrc2 = 1'bx; 
           regDest1 = 1'bx;  
	      regDest2 = 1'bx;
	      RegWrite = 1'b0;	
	      ALUSrc1 = 1'bx;   
	      ALUSrc2 = 1'bx;
	      MemWrite = 1'b0;
		 MemRead = 1'b0;
		 WriteBack1 = 1'bx; 
	      WriteBack2 = 1'bx;	  
		 extension_signal =1'bx;		 
		 ALUOp = 3'bxxx; 	
 
	 
	 current_state = WB;

	 		  
		end  
		  
		  
			  
	if  (Function == 3'b001) // <-- call
	 begin  
	  		numOf_CallInst = numOf_CallInst + 1;
				 // Control Signals   
		
		  PCSrc0 = 1'b0; //jump address 
           PCSrc1 = 1'b0;	
		  PCSrc2 = 1'b1;
           regFilesrc1 = 1'bx; 
	      regFilesrc2 = 1'bx; 
           regDest1 = 1'b1;  // to store the return address at the reg of address 9
	      regDest2 = 1'b1;
	      RegWrite = 1'b1;	
	      ALUSrc1 = 1'bx;   
	      ALUSrc2 = 1'bx;
	      MemWrite = 1'b0;
		 MemRead = 1'b0;
		 WriteBack1 = 1'b1; 
	      WriteBack2 = 1'b0;	  
		 extension_signal =1'bx;		 
		 ALUOp = 3'bxxx; 	
		 
		 addressOfReturnAddress = PC + 1;
		 current_state = MEM;	 
		 
	 

	 		  
		end  
		
	
	if  (Function == 3'b010) // <-- RET
	 begin  	
		 numOf_RetInst = numOf_RetInst + 1;
			 // Control Signals          
		                                 
		  PCSrc0 = 1'b0; //Return address        
		  PCSrc1 = 1'b1;	                  
		  PCSrc2 = 1'b1;                    
		 regFilesrc1 = 1'b1; // we need to access the regFile to get the return address stored in a special reg at address 9              
		 regFilesrc2 = 1'bx;                 
		  regDest1 = 1'bx;                   
		 regDest2 = 1'bx;                   
		 RegWrite = 1'b0;	                  
		 ALUSrc1 = 1'bx;                    
		 ALUSrc2 = 1'bx;                    
		 MemWrite = 1'b0;                   
		 MemRead = 1'b0;                    
		 WriteBack1 = 1'bx;                  
		 WriteBack2 = 1'bx;	              
		 extension_signal =1'bx;		      
		 ALUOp = 3'bxxx; 
		  current_state = WB;
		 
	end  
		
				
	  		  
	  end 
	 
	
//########################################################################	  
	   	case ({regDest1,regDest2})
			
		 2'b00 :  	outputOfDestRegMux =  AddressToRd;	 
		 2'b01 :	     outputOfDestRegMux = AddressToRt;
		 2'b10 : 		outputOfDestRegMux = 4'b1000; // for loop counter special reg
		 2'b11 : 	    outputOfDestRegMux = 4'b1001;   // return address special reg
		endcase   	  
		
		
			
		
		case (regFilesrc1)
			
		 1'b0 :  	 outputOfRegFilesrc1 = Registers[AddressToRs];	 
		 1'b1 :	 outputOfRegFilesrc1 = Registers[4'b1001]; // return address special reg
		endcase  	 
		
		
		
		case (regFilesrc2)
			
		 1'b0 :  	 outputOfRegFilesrc2 = Registers[AddressToRt];	 
		 1'b1 :	 outputOfRegFilesrc2 = Registers[4'b1000]; // for loop counter special reg
		endcase   	     
		$display ("outputOfDestRegMux: %b", outputOfDestRegMux);   
	    $display ("outputOfRegFilesrc1: %b", outputOfRegFilesrc1);     
		$display ("outputOfRegFilesrc2: %b", outputOfRegFilesrc2);  
		numOfClkCycle = numOfClkCycle + 1;
	   
	   end
//------------------------------------------------------------------------------------	



else if (current_state == EX) begin	 
	
			$display ("\n\n---------------->We are in Execution Stage.<---------------\n\n"); 
		    $display ("PC: %b", PC); 	   
			$display ("outputOfBTA: %b", outputOfBTA); 
			$display ("zero flag: %b", zero);

			
	case ({ALUSrc1})	
		0'b0: output_Of_ALUsrc1_mux =  outputOfRegFilesrc1;
		0'b1: output_Of_ALUsrc1_mux =  outputOfRegFilesrc2;	// for (for loop instruction)
	endcase	 
	
	
	case ({ALUSrc2})	
		0'b0: output_Of_ALUsrc2_mux =  outputOfRegFilesrc2;
		0'b1: output_Of_ALUsrc2_mux =  extended_immediate;
	endcase
	
	OperandA = output_Of_ALUsrc1_mux;
	OperandB = output_Of_ALUsrc2_mux;
	
	case(ALUOp)
		3'b000: ALUResult = OperandA & OperandB;    //Anding between the Operands
		3'b001: ALUResult = OperandA + OperandB;       //Adding between Operands
		3'b010: ALUResult = OperandA - OperandB;		 //Subtracting between Operands	
		3'b011: ALUResult = OperandA << OperandB;		 //SLL	
		3'b100: ALUResult = OperandA >> OperandB;		 //SRL
		default: ALUResult = 16'b0;
	endcase	 
	
	
	 if ((OP_CODE >= 4'b0000 && OP_CODE <= 4'b0011) || OP_CODE == 4'b1000) begin
      current_state = MEM;
    end
	
		 // Setting the zero Flag                                                                        
	zero = (ALUResult == 32'b0);                                                                    
	
	// For these Instructions: BEQ, BNE we must move to the next instruction after Execution Stage            
	if (OP_CODE == 4'b0110) begin                                                           
	    if (zero == 1) begin                                                                
	        // Branch Target Address                                                           
	        outputOfBTA = PC + extended_immediate;                                             
	        // PC = outputOfBTA; // Uncomment if you want to modify PC here                                                           
	        current_state = WB;	                                                             
	    end 
		else begin		                                                                        
	        outputOfBTA = PC + 1;                                                              
	        current_state = WB;                                                               
	    end	                                                                                 
	end

		/////////////////////////////////////////////////////////
		// BNE	
		else if (OP_CODE == 4'b0111) begin

			
			if (zero == 0)	begin
				// Branch Target Address
				outputOfBTA = PC + extended_immediate;
				current_state = WB;	
			end
	
		else begin	

				outputOfBTA = PC + 1;
				current_state = WB;
			end	
		end
		
		else if (OP_CODE == 4'b1000) begin                 
		                                              
			                                          
			if (zero == 0)	begin                                       
				nextAddress_of_for_inst = outputOfRegFilesrc1;
				forSignal = 1;
			                  
			end                                        
		                                              
		else begin	                                  
		                                              
			nextAddress_of_for_inst = PC + 1;  
			forSignal = 0;
	                 
			end	                                      
		end                                            
		
		
       numOfClkCycle = numOfClkCycle + 1;
end		  



	
//-------------------------------------------------------------------------------------
 
	
	else if (current_state == MEM) begin  
	$display ("\n\n---------------->We are in Memory Stage.<---------------\n\n"); 
		
	address = ALUResult;
	data_in = outputOfRegFilesrc2;
 	
		 if (MemWrite)  DataMemory[address] = data_in	;				
		
       	else if (MemRead) data_out = DataMemory[address] ;	
			
		 	
		$display ("\n\n--The Address: %h\n",address); 
				
		
	if (OP_CODE == 4'b0101)	begin 
		current_state = WB ;	 //skip write back for store instruction 
		$display ("\n\n--The Memoryin (Datain): %h\n",data_in);	  
		$display ("\n\n--: DataMemory[7] %h\n", DataMemory[7]);
		$display ("\n\n--: DataMemory[8] %h\n", DataMemory[8]);
		$display ("\n\n--: DataMemory[12] %h\n", DataMemory[12]);
		$display ("\n\n--: DataMemory[15] %h\n", DataMemory[15]);
		$display ("\n\n--: DataMemory[27] %h\n", DataMemory[27]);
		end
	else 
		$display ("\n\n--The MemoryOut (DataOut): %h\n",data_out);
	
		
		
		 numOfClkCycle = numOfClkCycle + 1;
	
	end	
	
//----------------------------------------------------------------------------------------------------------------------------------------------------------


else if (current_state == WB) begin	 
		 

		 case ({WriteBack1,WriteBack2})		
		 2'b00 : muxOut =  ALUResult;
		 2'b01 :	muxOut =  data_out;	
		 2'b10 : 	muxOut =  addressOfReturnAddress;
		 endcase 
		 
		 $display ("\n\n---------------->We are in Write Back  Stage.<---------------\n\n"); 
		 
		 
		 $display ("\n:MuxOut : %h\n",muxOut);
		 
		 if ( RegWrite ==  1'b1) begin 
			Registers[outputOfDestRegMux] = muxOut;
	     end  
		 
			$display("Register File Contents:");
	    for (integer i = 0; i < 10; i = i + 1) begin
	      $display("Registers[%0d] : %h", i, Registers[i]);
	    end
							    
		 
		 
		 $display ("\n\n\n***********DONE The INST*****************\n\n\n");	
		 numOfClkCycle = numOfClkCycle + 1;		
	end
	
//----------------------------------------------------------------------------------------------------------------------------------------------------------------	
endmodule	 

module TestBench();
    reg clk, reset;

    // For Instruction Decode
    wire [15:0] muxOut, outputOfRegFilesrc1, outputOfRegFilesrc2;  // Output data from register 3 (Bus W), Output data from register 1 (Bus A), Output data from register 2 (Bus B)
    wire [8:0] offset_9bit;
    
    // ALU result
    wire [15:0] ALUResult;
    
    // ExtenderResult
    wire [15:0] extended_immediate;
    wire [5:0] immediate_6bit;
    
    wire PCSrc0 = 0, PCSrc1 = 0, PCSrc2 = 0;  // sel 1, sel 2, sel 3
    wire regFilesrc1, regFilesrc2;  // Inputs of a register file
    wire regDest1, regDest2;
    wire RegWrite, extension_signal;
    wire ALUSrc1, ALUSrc2;
    wire [2:0] ALUOp;
    wire MemWrite, MemRead;
    wire WriteBack1, WriteBack2;
    wire [2:0] Function;	 
	wire [15:0] numOfInst;
	wire [15:0] numOfLoadInst; 
	wire [15:0] numOfStoreInst;
	wire [15:0] numOfAluInst;
	wire [15:0] numOf_ForInst; 
	wire [15:0] numOf_CallInst;
	wire [15:0] numOf_BranchInst;
     wire [15:0] numOf_JumpInst;
	wire [15:0] numOf_RetInst ; 
	
    wire [15:0] numOfClkCycle;
	wire [15:0] PC;

    // Instantiation of TheAllSystem
    TheAllSystem allss (
        .clk(clk),
        .reset(reset),
        .muxOut(muxOut),
        .outputOfRegFilesrc1(outputOfRegFilesrc1),
        .outputOfRegFilesrc2(outputOfRegFilesrc2),
        .ALUResult(ALUResult),
        .offset_9bit(offset_9bit),
        .extended_immediate(extended_immediate),
        .immediate_6bit(immediate_6bit),
        .PCSrc0(PCSrc0),
        .PCSrc1(PCSrc1),
        .PCSrc2(PCSrc2),
        .regFilesrc1(regFilesrc1),
        .regFilesrc2(regFilesrc2),
        .regDest1(regDest1),
        .regDest2(regDest2),
        .RegWrite(RegWrite),
        .extension_signal(extension_signal),
        .ALUSrc1(ALUSrc1),
        .ALUSrc2(ALUSrc2),
        .ALUOp(ALUOp),
        .Function(Function),
        .MemWrite(MemWrite),
        .MemRead(MemRead),
        .WriteBack1(WriteBack1),
        .WriteBack2(WriteBack2), 
	   .numOfInst(numOfInst),
	   .numOfLoadInst(numOfLoadInst), 
	   .numOfStoreInst(numOfStoreInst),
	   .numOfAluInst(numOfAluInst),
	   .numOf_ForInst(numOf_ForInst), 
        .numOf_CallInst(numOf_CallInst),
        .numOf_BranchInst(numOf_BranchInst),
        .numOf_JumpInst(numOf_JumpInst),
	   .numOf_RetInst(numOf_RetInst), 
	   .numOfClkCycle(numOfClkCycle),
	   .PC(PC)
	   
		
    );

    reg [3:0] current_state;

    initial begin
        current_state = 0;
        clk = 0;
        reset = 1;
        #1ns reset = 0;
    end

    always #2ns clk = ~clk;

    always @(posedge clk) begin
        // Displaying the values with time
        $display("Time=%0t:, Rd=%h, Rs=%h, Rt=%h, ALUResult=%h,extended_immediate=%h, offset_9bit =%h, numOfInst = %d(in decimal), numOfLoadInst = %d(in decimal), numOfStoreInst = %d(in decimal), numOfAluInst = %d(in decimal),numOf_ForInst = %d(in decimal),numOf_CallInst = %d(in decimal),numOf_BranchInst = %d(in decimal),numOf_JumpInst = %d(in decimal),numOf_RetInst = %d(in decimal),numOfClkCycle = %d(in decimal) ",
		$time,  muxOut, outputOfRegFilesrc1, outputOfRegFilesrc2, ALUResult, extended_immediate,offset_9bit,numOfInst,numOfLoadInst,numOfStoreInst,numOfAluInst,numOf_ForInst,numOf_CallInst,numOf_BranchInst,numOf_JumpInst,numOf_RetInst,numOfClkCycle);
	 
    end

    initial #400ns $finish;  // End simulation after 400ns
endmodule

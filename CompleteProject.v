//Leondre Washington
//Final Project
//12/2/20
//Icarus Verilog 0.10.0 is used as the simulation tool for opening EPWave
//Appropriate comments are made throughout the code
//A test bench was also created
//Detailed waveform snapshots of instructions are included in the zip file
//The Top module used is provided by Mark Welker

	

`timescale 1ns / 1ps

// Project module top module
//
//
// THis is the memory map for your reference: These values are NOT used in the top module.
// These parameters are the upper 4 bits of the address.
//parameter MainMemEn = 0;
//parameter RegisterEn = 1;
//parameter InstrMemEn = 2;
//parameter MatrixAluEn = 3;
//parameter IntegerAluEn = 4;
//parameter ExecuteEn = 5;

module top ();

logic [255:0] InstructDataOut;  // instruction memory output
logic [255:0] MemDataOut;      // main memory output
logic [255:0] ExeDataOut; // output from execution engine, input to all other modules
							// This requires the use of the execution engine to transfer data from one module to another.
							// You need to put a register in executable to hold the data from the datamux to send data back to other modules 
logic [255:0] DataMuxOut;  // output from mux to provide data into executionunit from all other units.
logic [255:0] MatAluDataOut; // output from matrix ALU
logic [255:0] IntAluDataOut; // output from Integer ALU
logic nReset,Clk;		// output from teh testbench
logic nRead,nWrite;		// output from the execution engine
logic [15:0] address;	// outputfrom execution engine

InstructionMemory  U1(Clk,InstructDataOut, address, nRead,nReset);

MainMemory  U2(Clk,MemDataOut,ExeDataOut, address, nRead,nWrite, nReset);

Execution  	U3(Clk,DataMuxOut,ExeDataOut, address, nRead,nWrite, nReset);

TestBench  	UTest(Clk,nReset);

DataMux 	U4 (DataMuxOut,Clk,nRead,address,InstructDataOut,MemDataOut,ExeDataOut,MatAluDataOut,IntAluDataOut);

MatrixALU  	U5(Clk,MatAluDataOut,ExeDataOut, address, nRead,nWrite, nReset);

IntegerALU  U6(Clk,IntAluDataOut,ExeDataOut, address, nRead,nWrite, nReset);

  initial begin //. setup to allow waveforms for edaplayground
   $dumpfile("dump.vcd");
   $dumpvars(1);
 end

endmodule

//Top module ends here

//Instruction memory
`timescale 1ns / 1ps

module InstructionMemory(
   input Clk,
   output reg [255:0] InstructDataOut,
    input  [15:0] address,
    input nRead,
    input nReset   
    );
//instruction memory
reg [255:0] instr_mem [0:15]; //we can store a total of 16 insturctions here
                              // this size can be increased based on number instructions
//internal signals
wire [3:0] nEnable;     //enable for instruction memory
reg [3:0] Prm_counter;    //program counter //this size can be increased as per number of instructions to be executed

assign nEnable = address[15:12];  //first four bits of address is enable signal

//program counter
always @*
begin
if (nReset ==1'b1)
Prm_counter = 4'd0;
else Prm_counter = address[3:0];  
end 

initial begin   //initialize instruction memory with required instructions
                //note that here addresses starting with 0 (00,01,02 etc) indicate main memory locations
                //and addresses starting with 1 (11,12 etc) indicate temporary register memory
				//only the least 32 bits are used for instruction
				
				
//                      {Opcode,Dest, Op1 , Op2}
instr_mem [0] = {224'd0,8'h01,8'h02,8'h00,8'h01};  //{0,Opcode,destination,source1,source2}
                                                   //ADD matrix A and B then store result in 02h
instr_mem [1] = {224'd0,8'h10,8'h11,8'h08,8'h09};  //ADD integer data1 and data2 then store result in temporary register 11h 
instr_mem [2] = {224'd0,8'h02,8'h03,8'h02,8'h00};  // Subtract first matrix from result in step1 and storing result in 03h memory
instr_mem [3] = {224'd0,8'h03,8'h04,8'h02,8'h00};  // Transpose matrix result from step1 and store in 04h memory  
instr_mem [4] = {224'd0,8'h04,8'h12,8'h03,8'h11};  //  Scale the result from step 3(03h) with result in step 2 (11h) and store
                                                   //result in temporary matrix register 12h.                                             
instr_mem [5] = {224'd0,8'h00,8'h05,8'h04,8'h03};  //multiply the result from step 4 with result in step 3 and store result in memory 05
instr_mem [6] = {224'd0,8'h12,8'h0A,8'h00,8'h01};  //multiply Integer in memory location 0 with 1 and store result in memory 0Ah.(only last 16 bits)
instr_mem [7] = {224'd0,8'h11,8'h13,8'h0A,8'h01};  //Subtract memory 01 from OA and store result in temporary register 13h
instr_mem [8] = {224'd0,8'h13,8'h0B,8'h0A,8'h13};  //Divide memory location 0A by result in step 8 and store result in 0B.
instr_mem [9] = 256'd0; //no operation
end

//fetching instruction
always @ (posedge Clk)
begin
if (nEnable == 4'd2) begin
if (nRead == 1'b1) 
    InstructDataOut = instr_mem [Prm_counter];
end
end
                              
endmodule

//Instruction memory ends

//main memory
`timescale 1ns / 1ps

module MainMemory(
   input Clk,
   output reg [255:0] MemDataOut,
    input [255:0] ExeDataOut,
    input  [15:0] address,
    input nRead,
    input nWrite,
    input nReset
 );
//Main memory
reg [255:0] main_mem [0:15]; //we can have a total of 16 memory locations here
                              // this size can be changed based on required memory
//internal signals
wire [3:0] nEnable;     //enable for main memory
assign nEnable = address[15:12];

reg [7:0] location; //address location of data to be fetched or to written to memory


integer i;

initial begin   //initialize main memory with required Data

if (nReset == 1'b1)
begin
for (i = 0; i <= 15 ; i++)
  main_mem [i] = 256'd0;
end

//we need to store given A and B matrices here
//each element is 16 bit => 4 * 4 matrix means total 16 elemnets.
// for 16 elements 16 * 16 = 256. so we need 256 bit memory.
main_mem [0] = {16'd3,16'd2,16'd8,16'd34,16'd16,16'd6,16'd11,16'd9,16'd9,16'd2,16'd12,16'd13,16'd2,16'd4,16'd16,16'd3};  //matrix A
main_mem [1] = {16'd6,16'd5,16'd7,16'd5,16'd7,16'd6,16'd4,16'd1,16'd4,16'd9,16'd3,16'd8,16'd2,16'd5,16'd7,16'd6};   //matrix B
//we need to store 16 bit integer data here
//integer is stored in lower 16 bits of memory.
//remaining 240 upper bits are considered 0 as they are not used in integer operations
//integer1 and integer2 are choosen as 20 and 4 randomly
main_mem [8] = {240'd0,16'h02};  //16 bit integer 1 data
main_mem [9] = {240'd0,16'h13};  //16 bit integer 2 data

end

//fetching memory
always @ (posedge Clk)
begin
if (nEnable == 4'd0) begin
location = address[7:0];
if (nRead == 1'b1) begin      //reading from main memory
    MemDataOut = main_mem [location];
    end
else if (nWrite == 1'b1) begin    //writing to main memory
     main_mem [location] =  ExeDataOut;
     end  
end
end
                              
endmodule

//main memory ends

//Data mux 

`timescale 1ns / 1ps

module DataMux(
    output reg [255:0] DataMuxOut,
    input Clk,
    input nRead,
    input [15:0] address,
    input [255:0] InstructDataOut,
    input [255:0] MemDataOut,
    input [255:0] ExeDataOut,
    input [255:0] MatAluDataOut,
    input [255:0] IntAluDataOut
    );
    
//selection signal
//middle 4 bits of address
wire [3:0] select, nEnable;
assign nEnable = address [15:12];  //5 is choosen as enable signal for data mux
assign select = address [11:8];    //selection signal for mux.

//temporary reg for storing data
reg [255:0] DataMuxOut_temp;

//selecting data mux output based on select signal
always @ (posedge Clk)
begin
if (nEnable == 4'd5) begin
case(select)
4'd0: DataMuxOut_temp = InstructDataOut;
4'd1: DataMuxOut_temp = MemDataOut;
4'd2: DataMuxOut_temp = ExeDataOut;
4'd3: DataMuxOut_temp = MatAluDataOut;
4'd4: DataMuxOut_temp = IntAluDataOut;
endcase
end
end

//data mux output
always @*
begin    //data is sent to execution engine when enabled and read is high
if (nRead == 1'b1 & nEnable == 4'd5) begin
DataMuxOut = DataMuxOut_temp;
end
end
    
endmodule
//Data mux ends

//Execution module
`timescale 1ns / 1ps

module Execution(
    input Clk,
    input [255:0] DataMuxOut,
    output reg [255:0] ExeDataOut,
    output  [15:0] address,
    output reg nRead,
    output reg nWrite,
    input nReset
    );
 
 //states declaration using enum
 typedef enum {Idle, InstrFetch1,InstrFetch2,InstrFetch3, Source1, Read1_Source1,Read2_Source1,Write1_to_ALU,
                  Source2,Read1_Source2,Read2_Source2,Write2_to_ALU,get_ALU_result,result_in_mux,
                  result_to_exe,Write_back} state_type;
				  
 state_type state_reg = Idle; //initially in Idle state
 state_type state_next;
 
  //internal signals
 reg [3:0] nEnable;  //Enable signal for each module . upper 4 bits of address
 reg [3:0] nSelect;  //selection input for Data Mux. middle 4 bits of address.
 reg [7:0] nLocation; //address of the instruction or memory or register. lower 8 bits of address
 //final address making
 assign address = {nEnable,nSelect,nLocation};
 
 //internal reg for program counter
 reg [7:0] Prm_counter = 0;
 
 /////////-------------temporary register explanation------------/////////////
 //temporary register for storing data from data mux and temporary results
 //we have created 7 temporary 256 bit registers
 //temp_reg[16] stores the current instruction being fetched
 //temp_reg[20] stores the source1 temporarly, if it is from main memory
 //temp_reg[21] stores the source2 temporarly, if it is from main memory
 //temp_reg[22] stores the Result temporarly, if it is to be stored in main memory
 //other 3 registers are used for storing any results temporarly based on instructions
 reg [255:0] temp_reg [16:22];
 
 //signals for storing operands addresses
 wire [7:0] Op1, Op2, Dest, Opcode; 
 //Op1 => source1
 //Op2 => source2
 //Dest => Destination 
 
 //insutruction decode
 assign {Opcode, Dest, Op1, Op2} = temp_reg[16][31:0];  //least 32 bits are considered as instruction
 
 //main fsm
 always @ (posedge Clk or posedge nReset)
 begin
 if (nReset == 1'b1) begin
        state_reg <= InstrFetch1;   //execution starts from instruction fetch when reset
             Prm_counter = 8'd0;
             end
 else state_reg <= state_next;
 end 
 
 /////////////---------------Finite state machine explanation--------///////////////
 //The state machine will have a total of 15 states and it takes atmost 15 clock cycles to complete one instruction
 //based on type of instruction.
 //Initially state machine waits in Idle state and when reset signal is given then the process starts
 //Based on the value of program counter, instruction is fetched from instruction memory and stored in temporary register 10.
//Depending upon the type of sources (from main memory or from temporary register), we fetch the source data and store them in temporary register in execution engine
//now the two source data values are transfered to either Matrix ALU or Integer ALU one by one . 
//then we wait for one cylce for ALU to finish it's calculations.
//then we fetch the result from ALU 
//depending upon the destination, the result is either stored in temporary register or written back to main memory.
//then program counter increments its value by 1 and fetches the second instruction fro execution.
//this process continues until all the instructions are executed or until the reset signal is given
//if all instruction are completed then state machine goes back to Idle state. and waits for reset signal.


 //next state logic
 always @*
 begin
 //defaults
 nRead = 1'b0;
 nWrite = 1'b0;
 nEnable = 4'b1111;  //nothing is enabled
 nSelect = 4'b1111; //nothing is selected in Data mux
 nLocation = 8'd0;
 case(state_reg)
Idle: begin
      if(nReset == 1'b1) begin
      state_next = InstrFetch1;  //Execution starts when Reset
      end
      end
InstrFetch1: begin    //In execution engine
            nRead = 1'b1; //Read from instruction memory
            nEnable = 4'd2; //instruction memory enabled
            nLocation = Prm_counter; //location of instruction to be fetched
            state_next = InstrFetch2;
            end      
InstrFetch2: begin    // In Instruction memory
            nEnable = 4'd5; //Data mux  enabled
            nSelect = 4'b0000; //Data is selected from instruction memory     
            state_next = InstrFetch3;      
             end     
InstrFetch3: begin    // In Data mux    
              nEnable = 4'd5; //Data mux  enabled
              nRead = 1'b1; //Read from data mux to execution engine   
              temp_reg[16] = DataMuxOut; //stored in temporary register 10.
              
              state_next = Source1;
             end    
//Operands fetching starts from here

//to distinguish between selecting operands from Main Memory and Temporary register, we will use following notation.
//if MSB of source/dest = 0 => select oprand from main memory
//if MSB of source/dest = 1 => select operand from temporary register

//note that our present instruction is stored in temporary register 10h
//Source1
//when source is in temporary register there is no need to fetch data from main memory then 
//Read1_Source1 and Read2_Source1 states are skipped and operation directly goes to write to ALU.
Source1: begin
          
          if (Op1[7:4] == 4'd1) begin    //source 1 is in temp reg

            state_next = Write1_to_ALU;    
            end
          else if (Op1[7:4] == 4'd0) begin   //read data from main memory
               nRead = 1'b1;
               nEnable = 4'd0;
               nLocation = Op1;
               state_next = Read1_Source1;
               end
              if ({Opcode, Dest, Op1, Op2} == 32'd0) begin
                      state_next = Idle;  //end of all instructions
                      end     
          end
Read1_Source1: begin
              nEnable = 4'd5; //Data mux  enabled
              nSelect = 4'b0001; //Data is selected from main memory 
              state_next = Read2_Source1;
              end          
Read2_Source1: begin    // In Data mux    
              nEnable = 4'd5; //Data mux  enabled
              nRead = 1'b1; //Read from data mux to execution engine   
              temp_reg[20] = DataMuxOut; //source 1 is stored in temporary register 14h=20.
              state_next = Write1_to_ALU;
             end     
Write1_to_ALU: begin
             if (Op1[7:4] == 4'd1) begin    //source 1 is in temp reg
           ExeDataOut = temp_reg[Op1];
           end
            else if (Op1[7:4] == 4'd0) ExeDataOut = temp_reg[20];  //source1
            nWrite = 1'b1;  //write to ALU
            nSelect = 4'd0; //source1
            if (Opcode[7:4] == 4'b0000) begin
                   nEnable = 4'd3;  //Matrix ALU is enabled
                end
            else if (Opcode[7:4] == 4'b0001) begin
                   nEnable = 4'd4;  //Integer ALU is Enabled
            end 
            nLocation = Opcode;
            state_next = Source2; 
            end     

//Source2
//when source is in temporary register there is no need to fetch data from main memory then 
//Read1_Source2 and Read2_Source2 states are skipped and operation directly goes to write to ALU.			
Source2: begin
          if (Op2[7:4] == 4'd1) begin    //source 2 is in temp reg

            state_next = Write2_to_ALU;    
            end
          else if (Op2[7:4] == 4'd0) begin   //read data from main memory
               nRead = 1'b1;
               nEnable = 4'd0;
               nLocation = Op2;
               state_next = Read1_Source2;
               end  
          end
Read1_Source2: begin
              nEnable = 4'd5; //Data mux  enabled
              nSelect = 4'b0001; //Data is selected from main memory 
              state_next = Read2_Source2;
              end          
Read2_Source2: begin    // In Data mux    
              nEnable = 4'd5; //Data mux  enabled
              nRead = 1'b1; //Read from data mux to execution engine   
              temp_reg[21] = DataMuxOut; //source 2 is stored in temporary register 15h=21.
              state_next = Write2_to_ALU;
              nLocation = Opcode;
             end     
Write2_to_ALU: begin
          if (Op2[7:4] == 4'd1) begin    //source 2 is in temp reg
            ExeDataOut = temp_reg[Op2];
            end
             else  if (Op2[7:4] == 4'd0) ExeDataOut = temp_reg[21];  //source2
            nWrite = 1'b1;  //write to ALU
            nSelect = 4'd1; //source2
            if (Opcode[7:4] == 4'b0000) begin
                   nEnable = 4'd3;  //Matrix ALU is enabled
                   
                end
            else if (Opcode[7:4] == 4'b0001) begin
                   nEnable = 4'd4;  //Integer ALU is Enabled
            end 
            nLocation = Opcode;
            state_next = get_ALU_result; 
            end             
//Now we get the Result from ALU and store it in either MAIN memory or Temporary register			
get_ALU_result: begin
                nRead = 1'b1; //read from ALU
                if (Opcode[7:4] == 4'b0000) begin
                   nEnable = 4'd3;  //Matrix ALU is enabled
                end
                else if (Opcode[7:4] == 4'b0001) begin
                   nEnable = 4'd4;  //Integer ALU is Enabled
                end    
                state_next = result_in_mux;              
                end      
result_in_mux: begin
               nEnable = 4'd5; //Data mux  enabled
                if (Opcode[7:4] == 4'b0000) begin
                   nSelect = 4'd3;  //Data is selected from MAT ALU
                end
                else if (Opcode[7:4] == 4'b0001) begin
                   nSelect = 4'd4;  //Data is selected from INT ALU
                end   
              state_next = result_to_exe; 
              end
result_to_exe: begin
               nEnable = 4'd5; //Data mux  enabled
              nRead = 1'b1; //Read from data mux to execution engine   
              temp_reg[22] = DataMuxOut; //result is stored in temporary register 16h=22.
              state_next = Write_back;
               end         
Write_back: begin
         //   temp_reg[17] = temp_reg[22];   //write back result to given register
            if (Dest[7:4] == 4'd1) begin   //destination is in temp register
               temp_reg[Dest] = temp_reg[22];   //write back result to given register
               Prm_counter = Prm_counter + 8'd1;  //increment program counter
               
               state_next = InstrFetch1;   //fetches next instruction
             end
             else if (Dest[7:4] == 4'd0) begin   //destination is in main memory
               ExeDataOut = temp_reg[22];
               nEnable = 4'd0; //main memory enabled
               nWrite  = 1'b1; //write to main memory
               nLocation = Dest; //destination register address
               Prm_counter = Prm_counter + 8'd1;  //increment program counter

                state_next = InstrFetch1;   //fetches next instruction
               end
            end                                                            
 endcase
 end
 
 
endmodule


//Execution module ends



//Matrix ALU 
`timescale 1ns / 1ps

module MatrixALU(
    input Clk,
    output reg [255:0] MatAluDataOut,
    input [255:0] ExeDataOut,
    input [15:0] address,
    input nRead,
    input nWrite,
    input nReset
    );
    
//internal enable and source selection signals
wire [3:0] nEnable, nSource;
wire [7:0] Opcode;
assign nEnable = address[15:12]; // first 4 bits of address is enable signal
assign nSource = address[11:8];  //middle 4 bits can be used as selection signal for source 
assign Opcode = address[7:0];  //last 8 bits of address is OPCODE

//internal registers for storing sources in matrix form
reg [15:0] A [0:3] [0:3];
reg [15:0] B [0:3] [0:3];
reg [15:0] R [0:3] [0:3]; //internal for storing result

//writing sources into Matrix ALU
always @ *
begin
if (nEnable == 4'd3) begin
    if (nWrite == 1'b1) begin
       if (nSource == 4'd0) begin    //Data from Execution engine is stored as A matrix
             {A[0][0],A[0][1],A[0][2],A[0][3],A[1][0],A[1][1],A[1][2],A[1][3],A[2][0],A[2][1],A[2][2],A[2][3],A[3][0],A[3][1],A[3][2],A[3][3]} = ExeDataOut;
             end
       else if  (nSource == 4'd1) begin //Data from Execution engine is stored as B matrix
             {B[0][0],B[0][1],B[0][2],B[0][3],B[1][0],B[1][1],B[1][2],B[1][3],B[2][0],B[2][1],B[2][2],B[2][3],B[3][0],B[3][1],B[3][2],B[3][3]} = ExeDataOut;
             end     
    end
end
end   

//ALU operation
//local parameters for opcode
localparam [7:0] MMult       = 8'h00,
                 Madd        = 8'h01,
                 Msub        = 8'h02,
                 Mtranspose  = 8'h03,
                 MScale      = 8'h04,
                 MScaleImm   = 8'h05;

//ALU
integer i,j,k;
always @*
begin
 i = 0;
 j = 0;
 k = 0;
{R[0][0],R[0][1],R[0][2],R[0][3],R[1][0],R[1][1],R[1][2],R[1][3],R[2][0],R[2][1],R[2][2],R[2][3],R[3][0],R[3][1],R[3][2],R[3][3]} = 256'd0;  //default value
case(Opcode)
MMult:  begin    //Matrix multiplication
          for(i=0;i < 4;i=i+1)
            for(j=0;j < 4;j=j+1)
                for(k=0;k < 4;k=k+1)
                    R[i][j] = R[i][j] + (A[i][k] * B[k][j]);
        end            
Madd:  begin    //Matrix addition
          for(i=0;i < 4;i=i+1)
            for(j=0;j < 4;j=j+1)
                    R[i][j] = A[i][j] + B[i][j];
        end  
Msub:  begin    //Matrix subtraction
          for(i=0;i < 4;i=i+1)
            for(j=0;j < 4;j=j+1)
                    R[i][j] = A[i][j] - B[i][j];
        end 
Mtranspose: begin    //Matrix transpose
          for(i=0;i < 4;i=i+1)
            for(j=0;j < 4;j=j+1)
                    R[i][j] = A[j][i];
        end  
        //NOTE
        //for multiplication with scalar and multiplication with immediate number
        //the integer is stored as last 16 bits of Exe data out which is B[3][3] (last element of matrix B)
        //so B[3][3] is used as scalar integer 
MScale:  begin    //Matrix multiplication with scalar
          for(i=0;i < 4;i=i+1)
            for(j=0;j < 4;j=j+1)
                    R[i][j] = A[i][j] * B[3][3];
        end  
MScaleImm:  begin    //Matrix subtraction
          for(i=0;i < 4;i=i+1)
            for(j=0;j < 4;j=j+1)
                    R[i][j] = A[i][j] * B[3][3];
        end   
        
endcase
end  

//reading final result
always @ (posedge Clk)
begin
if (nEnable == 4'd3) begin
if (nReset == 1'b1) begin
    MatAluDataOut = 256'd0;
end
else if (nRead == 1'b1) begin
   MatAluDataOut = {R[0][0],R[0][1],R[0][2],R[0][3],R[1][0],R[1][1],R[1][2],R[1][3],R[2][0],R[2][1],R[2][2],R[2][3],R[3][0],R[3][1],R[3][2],R[3][3]};  
end
end  
end             
                 
endmodule

//MATRIX ALU ENDS

//INTEGER ALU
`timescale 1ns / 1ps
module IntegerALU(
    input Clk,
    output reg [255:0] IntAluDataOut,
    input [255:0] ExeDataOut,
    input [15:0] address,
    input nRead,
    input nWrite,
    input nReset
    );
    
//internal enable and source selection signals
wire [3:0] nEnable, nSource;
reg [7:0] Opcode;
assign nEnable = address[15:12]; // first 4 bits of address is enable signal
assign nSource = address[11:8];  //middle 4 bits can be used as selection signal for source 


//internal registers for storing sources
reg [15:0] int1, int2;
reg [15:0] result;  //internal for storing result

//writing sources into INT ALU
always @ *
begin
if (nEnable == 4'd4) begin
    if (nWrite == 1'b1) begin
       if (nSource == 4'd0) begin    //integer 1 data is last 16 bits of Exe Data
             int1 = ExeDataOut[15:0];
             end
       else if  (nSource == 4'd1) begin //integer 2 data is last 16 bits of Exe Data
             int2 = ExeDataOut[15:0];
             end     
    end
end
end   

//ALU operation
//local parameters for local opcode
localparam [7:0] IntAdd  = 8'h10,
                 IntSub  = 8'h11,
                 IntMult = 8'h12,
                 IntDiv  = 8'h13;

//ALU
always @*
begin
result = 16'd0;  //default value
case(Opcode)
IntAdd:  result = int1 + int2 ; //integer addition
IntSub:  result = int1 - int2 ; //integer subtraction
IntMult: result = int1 * int2 ; //integer multiplication
IntDiv:  result = int1 / int2 ; //integer division
endcase
end  

//reading final result
always @ (posedge Clk)
begin
if (nEnable == 4'd4) begin
Opcode = address[7:0];  //last 8 bits of address is OPCODE
if (nReset == 1'b1) begin
    IntAluDataOut = 256'd0;
end
else if (nRead == 1'b1) begin
   IntAluDataOut = {240'd0, result};  
end
end  
end             
                 
endmodule

//INTEGER ALU ENDS

//TEST BENCH
`timescale 1ns / 1ps

module TestBench(output reg Clk,
                 output reg nReset);


//clock generation
initial begin
Clk = 1'b0;
forever #10 Clk = ~Clk;
end

//reset
initial begin
nReset = 1'b0;
#10 nReset = 1'b1;
#40 nReset = 1'b0;
#3000 $finish;
end

endmodule

//TEST BENCH ENDS

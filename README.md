# Verilog_PE
 This is simplistic verilog processing engine. I created a processor containing a matrix ALU, math ALU, instruction fetch, execution and memory interface.

Consists of matrix multiplication scalermultiplication, addition/subtarction, transposition, and a top module bringing each piece together.

Project Details:
1. All math functions will have a clear input set to zero
2. The register value needs to be able to be written back into memeory location
3. All matricies will be 4x4 16 bit deep
4. Matrix multiplier will multiply two 4x4 matrix and return a 4x4 matrix
5. Scalar multiplication is multiplying a matrix by single number
6. Add and subtract will add and subtract 2 4x4 matrix
7. Transpose will flip a matrix along a diagnol
8. The test bench will Start the clock and toggle reset
9. Execution engine will fetch the first opcode from instruction memory and begin execution
10. The execution engine will direct the transfer of data between the memory, the appropriate matrix modules and memory
11. The execution engine will continue executing programs until is finds a STOP opcode
12. The test bench will display an output waveform to determine correct operation





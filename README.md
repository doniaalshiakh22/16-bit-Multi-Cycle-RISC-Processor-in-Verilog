# Design and Verification of a Multi-Cycle 16-bit RISC Processor (Verilog)

## 1. Project Summary

This project entails the implementation and verification of a simple 16-bit RISC processor using Verilog HDL. The processor adopts a **multi-cycle** execution model, where each instruction is processed across several clock cycles, allowing efficient reuse of datapath components and reducing hardware complexity.

---

## 2. Architecture Overview

### 2.1 Datapath Elements

The processor architecture is modular and built around a classic multi-cycle design. Each instruction passes through distinct stages, with the following key hardware blocks:

- **Instruction Memory** – Responsible for delivering the next instruction to be executed.  
- **Register File** – Contains general-purpose and special-purpose registers.  
- **Arithmetic Logic Unit (ALU)** – Executes arithmetic and logic instructions.  
- **Data Memory** – Used for data storage during load (`LW`) and store (`SW`) operations.  
- **Sign Extender** – Extends 6-bit immediate values to 16-bit format.  
- **Program Counter (PC)** – Maintains the address of the current instruction.  
- **Multiplexers (MUXes)** – Direct data flow based on control logic signals.

### 2.2 Instruction Execution Cycle

The execution of instructions is spread over five clock cycles, corresponding to the following stages:

1. **Instruction Fetch (IF)** – Fetches the next instruction from instruction memory.  
2. **Instruction Decode (ID)** – Decodes the opcode and fetches operands from the register file.  
3. **Execute (EX)** – ALU performs operations or calculates memory addresses.  
4. **Memory Access (MEM)** – Access to data memory, if needed.  
5. **Write Back (WB)** – Result is written back to the destination register.

---

## 3. Instruction Set Architecture (ISA)

The processor supports a basic yet functional instruction set categorized into three types:

### 3.1 R-Type (Register-based Instructions)

| Instruction | Operation        | Opcode | Func |
|-------------|------------------|--------|------|
| `AND`       | Rd = Rs & Rt     | 0000   | 000  |
| `ADD`       | Rd = Rs + Rt     | 0000   | 001  |
| `SUB`       | Rd = Rs - Rt     | 0000   | 010  |
| `SLL`       | Rd = Rs << Rt    | 0000   | 011  |
| `SRL`       | Rd = Rs >> Rt    | 0000   | 100  |

### 3.2 I-Type (Immediate-based Instructions)

| Instruction | Operation                    | Opcode |
|-------------|-------------------------------|--------|
| `ANDI`      | Rt = Rs & Immediate           | 0010   |
| `ADDI`      | Rt = Rs + Immediate           | 0011   |
| `LW`        | Rt = Mem[Rs + Immediate]      | 0100   |
| `SW`        | Mem[Rs + Immediate] = Rt      | 0101   |
| `BEQ`       | Branch if Rs == Rt            | 0110   |
| `BNE`       | Branch if Rs != Rt            | 0111   |
| `FOR`       | Loop operation (Rs = addr, Rt = counter) | 1000   |

### 3.3 J-Type (Jump Instructions)

| Instruction | Operation                            | Opcode | Func |
|-------------|---------------------------------------|--------|------|
| `JMP`       | PC = Jump Target                     | 0001   | 000  |
| `CALL`      | RR = PC + 1; PC = Jump Target        | 0001   | 001  |
| `RET`       | PC = RR                              | 0001   | 010  |

---

## 4. Example Test Sequence

Below is a short demonstration of how instructions move through the processor’s execution pipeline:

- **AND R1, R2, R7**  
  - IF: Fetch instruction  
  - ID: Decode and read R2, R7  
  - EX: Compute R2 & R7  
  - WB: Write result to R1

- **ADDI R5, R2, 6**  
  - IF: Fetch instruction  
  - ID: Decode and read R2  
  - EX: Add 6 to R2  
  - WB: Write result to R5

- **LW R4, 8(R3)**  
  - IF: Fetch instruction  
  - ID: Decode and read R3  
  - EX: Compute R3 + 8  
  - MEM: Load value from memory  
  - WB: Store in R4

- **JMP 17**  
  - IF: Fetch instruction  
  - ID: Decode and get target  
  - EX: Set PC to 17

---

## 5. More Information

For detailed schematics, RTL design, and simulation waveforms, please refer to the full project report and Verilog source files included in this repository.


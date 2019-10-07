#include "Core.h"
#include "Registers.h"

Core *initCore(Instruction_Memory *i_mem) {
  Core *core = (Core *)malloc(sizeof(Core));
  core->clk = 0;
  core->PC = 0;
  core->instr_mem = i_mem;
  core->tick = tickFunc;
  core->ifState = (State *) malloc(sizeof(State));
  core->idState = (State *) malloc(sizeof(State));
  core->exState = (State *) malloc(sizeof(State));
  core->memState = (State *) malloc(sizeof(State));
  core->wbState = (State *) malloc(sizeof(State));
  core->ifState->nop = true;
  core->idState->nop = true;
  core->exState->nop = true;
  core->memState->nop = true;
  core->wbState->nop = true;
  core->ifState->PC = 0;
  core->idState->PC = 0;
  core->exState->PC = 0;
  core->memState->PC = 0;
  core->wbState->PC = 0;


  memset(core->registers, 0, 64*sizeof(core->registers[0]));
  memset(core->dataMemory, 0, NUM_MEM_DBLWDS*sizeof(core->dataMemory[0]));

   //MATRIX
  for (int i = 0; i < 16; i++) {
    core->dataMemory[i] = i;
  }
  
  
  /*//task 0
  core->registers[1] = 0;
  core->registers[2] = 10;
  core->registers[3] = -15;
  core->registers[4] = 20;
  core->registers[5] = 30;
  core->registers[6] = -35;
  core->dataMemory[5] = -63;
  core->dataMemory[6] = 63;
  */

  //task 1
  /*
  core->registers[1] = 8;
  core->registers[3] = -4;
  core->registers[5] = 255;
  core->registers[6] = 1023;
  */

 //task 3 v1
 /*
  core->registers[1] = 0;
  core->registers[2] = -5;
  core->registers[5] = -10;
  core->registers[6] = 25;
  core->registers[7] = 4;
  core->dataMemory[13] = -100;
*/
 //task 3 v2
 /*
  core->registers[1] = 8;
  core->registers[2] = -5;
  core->registers[5] = -10;
  core->registers[6] = 25;
  core->registers[7] = 4;
  core->dataMemory[13] = -100;
*/
  //task 1 v2
  /*
  core->registers[1] = 8;
  core->registers[3] = -15;
  core->registers[5] = 255;
  core->registers[6] = 1023;
  */
  //task 2
  /*
  core->registers[1] = 4;
  core->registers[5] = 26;
  core->registers[6] = -27;
  core->dataMemory[3] = 100;
  */
  return core;

}

void IF(State *state, Core *core) {
  if (state->nop) {
    return;
  }
  // (Step 1) Reading instruction from instruction memory
  state->instruction = core->instr_mem->instructions[state->PC / 4].instruction;
}

// Updates the passed in signals and returns the imm value
void ID(State *state, Core *core) {
  if (state->nop) {
    return;
  }
  // (Step 2) Pass into control, register file, immediate and ALU Control
  state->rs_1 = (state->instruction & (0b11111 << 15)) >> 15;
  state->rs_2 = (state->instruction & (0b11111 << 20)) >> 20;
  state->rs_1_data = core->registers[state->rs_1];
  state->rs_2_data = core->registers[state->rs_2];
  state->rd = (state->instruction & (0b11111 << 7)) >> 7;

  control(state, (state->instruction & 0b1111111));

  state->imm = immGen(state->instruction);
}

void EX(State *state, Core *core, int forwardA, int forwardB, uint64_t result1, uint64_t result2) {
  if (state->nop) {
    return;
  }
  // (Step 3) Pass into mux and from there into ALU
  uint64_t operand_2 = 0;
  uint64_t operand_1 = state->rs_1_data;
  if(state->aluSrc) {
    operand_2 = state->imm;
  } else {
    operand_2 = state->rs_2_data;
  }

  if (forwardA == 1) {
    operand_1 = result1;
  } else if (forwardA == 2) {
    operand_1 = result2;
  }

  if (forwardB == 1) {
    if (!state->aluSrc) {
      operand_2 = result1;
    }
    state->rs_2_data = result1;
  } else if (forwardB == 2 && !state->aluSrc) {
    if (!state->aluSrc) {
      operand_2 = result2; //Only do this if ALU isn't using imm value
    }
    state->rs_2_data = result2;
  }

  state->rs_1_data = operand_1;
  uint8_t alu_ctrl = aluControl(state->aluOp, ((state->instruction & (0b111 << 12)) >> 12), ((state->instruction & (0b1111111 << 25)) >> 25));
  alu(operand_1, operand_2, alu_ctrl, &state->result, &state->zero);
}

void Mem(State *state, Core *core) {
  if (state->nop) {
    return;
  }
  state->temp_mem = 0;
  

  if(state->memWrite) { //Write rs2 to memory if control says so
    if (state->result/8 < NUM_MEM_DBLWDS) {
      core->dataMemory[state->result/8] = core->registers[state->rs_2];
    }
  }
  if(state->memRead) { //read value from memory if control says so
      state->temp_mem = core->dataMemory[state->result/8];
  }
}

void WB(State *state, Core *core) {
  if (state->nop) {
    return;
  }
  int temp_data;
  temp_data = state->result;
  if(state->memToReg) {
    temp_data = state->temp_mem;
  } else if(state->branch && state->regWrite) {
    temp_data = state->PC + 4;
  }

  if(state->rd != 0 && state->regWrite) { //Make sure not to write to x0
    core->registers[state->rd] = temp_data;
  }
}

void printStatus(Core *core) {
  printf("IF = %ld, ", core->ifState->nop ? -1 : core->ifState->PC / 4);
  printf("ID = %ld, ", core->idState->nop ? -1 : core->idState->PC / 4);
  printf("EX = %ld, ", core->exState->nop ? -1 : core->exState->PC / 4);
  printf("MEM = %ld, ", core->memState->nop ? -1 : core->memState->PC / 4);
  printf("WB = %ld \n", core->wbState->nop ? -1 : core->wbState->PC / 4);
  int i;
  printf("REGISTERS:\n");
  for(i = 0; i < 64; i++) {
    if (core->registers[i] != 0) {
      printf("%3s: ", REGISTER_NAME[i]);
      printf("%ld\n", core->registers[i]);
    }
  }
  printf("MEMORY:\n");
  for(i = 0; i < NUM_MEM_DBLWDS; i++) {
    if (core->dataMemory[i] != 0) {
      printf("%d: ", i*8);
      printf("%ld\n", core->dataMemory[i]);
    }
  }
  printf("all other registers and memory vals are 0.\n");
}

bool tickFunc(Core *core) {
  // ----- Forwarding logic -----
  int forwardA = 0;
  int forwardB = 0;

  if (core->exState->rs_1 == core->memState->rd && !core->memState->memWrite && !core->memState->branch) {
    forwardA = 2;
  } else if (core->exState->rs_1 == core->wbState->rd && !core->wbState->memWrite && !core->wbState->branch) {
    forwardA = 1;
  }

  if (core->exState->rs_2 == core->memState->rd && !core->memState->memWrite && !core->memState->branch) {
    forwardB = 2;
  } else if (core->exState->rs_2 == core->wbState->rd && !core->wbState->memWrite && !core->wbState->branch) {
    forwardB = 1;
  }

  int wbResult;
  wbResult = core->wbState->result;
  if(core->wbState->memToReg) {
    wbResult = core->wbState->temp_mem;
  } else if(core->wbState->branch && core->wbState->regWrite) {
    wbResult = core->wbState->PC + 4;
  }

  int memResult;
  memResult = core->memState->result;
  if(core->memState->memRead) {
      core->memState->temp_mem = core->dataMemory[core->memState->result/8];
  }
  if(core->memState->memToReg) {
    memResult = core->memState->temp_mem;
  } else if(core->memState->branch && core->memState->regWrite) {
    memResult = core->memState->PC + 4;
  }




  // (Step 5) Writeback
  WB(core->wbState, core);

  // (Step 4) Memory access and register file writeback
  Mem(core->memState, core);

  // EX (instruction, signals, imm, rs_1_data, rs_2_data) -> instruction, signals, result, zero

  EX(core->exState, core, forwardA, forwardB, wbResult, memResult);

  // ID (instruction) -> signals, imm, instruction, rs_1_data, rs_2_data
  ID(core->idState, core);

  core->ifState = (State *) malloc(sizeof(State));
  core->ifState->PC = core -> PC;
  core->ifState->nop = (core -> PC > core->instr_mem->last->addr);
  // IF (PC) -> instruction
  IF(core->ifState,core);

  //Goal here is to determine the next PC for fetching based on ID. If it's not PC+4 insert a nop

  unsigned branch_PC = core->idState->PC + core->idState->imm *2;

  if(core->idState->jalr) branch_PC = core->idState->rs_1_data + core->idState->imm; //core->exState->result;

  uint64_t nextPC = core->PC += 4;

  // Find zero value (without alu)
  uint8_t funct3 = ((core->idState->instruction & (0b111 << 12)) >> 12);
  uint8_t zero = 0;
  uint64_t operand_2 = 0;
  uint64_t operand_1 = core->idState->rs_1_data;
  if(core->idState->aluSrc) {
    operand_2 = core->idState->imm;
  } else {
    operand_2 = core->idState->rs_2_data;
  }
  forwardA = 0;
  forwardB = 0;

  if (core->idState->rs_1 == core->exState->rd && !core->exState->memWrite && !core->exState->branch) {
    forwardA = 2;
  } else if (core->idState->rs_1 == core->memState->rd && !core->memState->memWrite && !core->memState->branch) {
    forwardA = 1;
  }

  if (core->idState->rs_2 == core->exState->rd && !core->exState->memWrite && !core->exState->branch) {
    forwardB = 2;
  } else if (core->idState->rs_2 == core->memState->rd && !core->memState->memWrite && !core->memState->branch) {
    forwardB = 1;
  }
  if (forwardA == 1) {
    operand_1 = core->memState->result;
  } else if (forwardA == 2) {
    operand_1 = core->exState->result;
  }

  if (forwardB == 1) {
    operand_2 = core->memState->result;
  } else if (forwardB == 2) {
    operand_2 = core->exState->result;
  }

  if(core->idState->aluOp == 1) { //Branch
    if (funct3 == 0b000) {//BEQ
      zero = (operand_1 == operand_2);
    }
    if (funct3 == 0b001) { //BNE
      zero = (operand_1 != operand_2);
    }
    if(funct3 == 0b100) { //BLT
      zero = (operand_1 < operand_2);
    }
    if (funct3 == 0b101) { //BGE
      zero = (operand_1 >= operand_2);
    }
  }

  if (core->idState->branch && zero && !core->idState->nop) {
    nextPC = branch_PC + 4;
    core->ifState->nop = true;
  }


  //Print system state to console
  printStatus(core);

  core->PC = nextPC;




  core->clk++;
  // Should we continue?
  printf("\n");
  if (core->wbState->PC <= core->instr_mem->last->addr ||
      core->memState->PC <= core->instr_mem->last->addr ||
      core->exState->PC <= core->instr_mem->last->addr ||
      core->idState->PC <= core->instr_mem->last->addr ||
      core->ifState->PC <= core->instr_mem->last->addr) {
    core->wbState = core->memState;
    core->memState = core->exState;
    core->exState = core ->idState;
    core->idState = core->ifState;
    return true;
  } else {
    return false;
  }
}

/*
 * Sets the state of all interal ISA signals and writes the results to input
 * signals variable. Uses input opcode and funct3 to determine signals
 */
void control(State *signals, unsigned opcode) {
  signals->branch = 0;
  signals->jalr = 0;
  signals->memRead = 0;
  signals->memToReg = 0;
  signals->aluOp = 0;
  signals->memWrite = 0;
  signals->aluSrc = 0;
  signals->regWrite = 0;
  if (opcode == 0b0000011) { //ld
    signals->memRead = 1;
    signals->memToReg = 1;
    signals->aluSrc = 1;
    signals->regWrite = 1;
  } else if(opcode == 0b0010011) { //I instructions
    signals->aluOp = 2;
    signals->aluSrc = 1;
    signals->regWrite = 1;
  } else if (opcode == 0b0100011) { //sd
    signals->memWrite = 1;
    signals->aluSrc = 1;
  } else if (opcode == 0b0110011) { //R instructions
    signals->aluOp = 2;
    signals->regWrite = 1;
  } else if (opcode == 0b1100011) { //B instructions
    signals->aluOp = 1;
    signals->branch = 1;
  } else if (opcode == 0b1100111) { //jalr
    signals->branch = 1;
    signals->jalr = 1;
    signals->aluSrc = 1;
    signals->regWrite = 1;
  } else if(opcode == 0b1101111) { //jal
    signals->branch = 1;
    signals->regWrite = 1;
  }
}

/*
 * Figures out what the immediate value is for an input instruction. immediate
 * value can be stored in pieces throughout so this is needed to combine those
 * pieces.
 */
int immGen(unsigned instr) {
  int imm = 0;
  unsigned opcode = (instr & 0b1111111);
  if(opcode == 0b0010011 || opcode == 0b0000011 || opcode == 0b1100111) { //I instructions
    imm |= ((instr & (0b111111111111 << 20)) >> 20);
    if(imm & 0x800) {
      imm |= 0xFFFFF000;
    }
  } else if(opcode == 0b0100011) { //S instructions
    imm |= ((instr & (0b11111 << 7)) >> 7);
    imm |= ((instr & (0b1111111 << 25)) >> 20);
    if(imm & 0x800) {
      imm |= 0xFFFFF000;
    }
  } else if(opcode == 0b1100011) { //B instructions
    imm |= ((instr & (0b1 << 7)) << 4);
    imm |= ((instr & (0b1111 << 8)) >> 7);
    imm |= ((instr & (0b111111 << 25)) >> 20);
    imm |= ((instr & (0b1 << 31)) >> 19);
    if(imm & 0x1000) {
      imm |= 0xFFFFF000;
    }
  } else if (opcode == 0b1101111) { //J instructions
    imm |= (instr & 0b11111111000000000000);
    imm |= ((instr & (0b100000000000 << 9)) >> 9);
    imm |= ((instr & (0b11111111110 << 20)) >> 20);
    imm |= ((instr & (0b100000000000000000000 << 11)) >> 11);
    if(imm & 0x100000) {
      imm |= 0xFFF00000;
    }
  }

  return imm;
}

void alu(uint64_t r_data_1, uint64_t r_data_2, uint8_t ctrl_signal, uint64_t *result, uint8_t *zero) {
  *zero = 1;
  *result = 0;
  if (ctrl_signal == 0) { //AND
    *result = r_data_1 & r_data_2;
    return;
  }
  if (ctrl_signal == 1) { //OR
    *result = r_data_1 | r_data_2;
    return;
  }
  if (ctrl_signal == 2) { //XOR
    *result = r_data_1 ^ r_data_2;
    *zero = *result;
    return;
  }
  if (ctrl_signal == 3) { //SHIFT LEFT
    *result = r_data_1 << r_data_2;
    return;
  }
  if (ctrl_signal == 4) { //SHIFT RIGHT
    *result = r_data_1 >> r_data_2;
    return;
  }
  if (ctrl_signal == 5) { //INVERT
    *result = r_data_1 << r_data_2;
    return;
  }
  if (ctrl_signal == 6) { //ADD
    *result = r_data_1 + r_data_2;
    return;
  }
  if (ctrl_signal == 7) { //SUBTRACT
    *result = r_data_1 - r_data_2;
    return;
  }
  if (ctrl_signal == 8) { //LT
    *result = (r_data_1 < r_data_2 );
    *zero = *result;
    return;
  }
  if (ctrl_signal == 9) { //GTE
    *result = (r_data_1 >= r_data_2);
    *zero = *result;
    return;
  }
  if (ctrl_signal == 10) { //EQ
    *result = (r_data_1 == r_data_2);
    *zero = *result;
    return;
  }
  if (ctrl_signal == 11) { //NEQ
    *result = (r_data_1 != r_data_2);
    *zero = *result;
    return;
  }
}

uint8_t aluControl(uint8_t aluOp, uint8_t funct3, uint8_t funct7) {
  if(aluOp == 0) return 6; // ADD
  if(aluOp == 1) { //Branch
    if (funct3 == 0b000) return 10; //BEQ
    if (funct3 == 0b001) return 11; //BNE
    if(funct3 == 0b100) return 8; //BLT
    if (funct3 == 0b101) return 9; //BGE
  }
  if(aluOp == 2) { //Math (I and R instructions)
    if (funct3 == 0) {
      if(funct7 == 0b0000000) return 6; // ADD
      if(funct7 == 0b0100000) return 7; // SUB
    }
    if (funct3 == 0b001) return 3; // SLL
    if (funct3 == 0b010) return 8; // LT
    if (funct3 == 0b100) return 2; // XOR
    if (funct3 == 0b101) return 4; // SRL
    if (funct3 == 0b110) return 1; // OR
    if (funct3 == 0b111) return 0; // AND
  }
  return 0;
}

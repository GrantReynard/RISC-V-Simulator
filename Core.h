#ifndef __CORE_H__
#define __CORE_H__

#include "Instruction_Memory.h"

#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#define NUM_MEM_DBLWDS 32
#define BOOL bool

struct Core;
typedef struct Core Core;
typedef struct State State;
typedef struct Core {
  Tick clk; // Keep track of core clock
  Addr PC; // Keep track of program counter
  uint64_t registers[64];
  uint64_t dataMemory[NUM_MEM_DBLWDS];
  Instruction_Memory *instr_mem;
  State *ifState;
  State *idState;
  State *exState;
  State *memState;
  State *wbState;
  // Simulation function
  bool (*tick)(Core *core);
} Core;

Core *initCore(Instruction_Memory *i_mem);
bool tickFunc(Core *core);
void control(State *ctrl_signals, unsigned opcode);
int immGen(unsigned instr);
void alu(uint64_t r_data_1, uint64_t r_data_2, uint8_t ctrl_signal, uint64_t *result, uint8_t *zero);
uint8_t aluControl(uint8_t aluOp, uint8_t funct3, uint8_t funct7);
typedef struct State {
  Addr PC;
  unsigned instruction;
  int imm;
  uint64_t rs_1_data;
  uint64_t rs_2_data;
  uint64_t result;
  uint8_t zero;
  int64_t temp_mem;
  bool nop;
  uint8_t rs_1;
  uint8_t rs_2;
  uint8_t rd;


  //Signals
  uint8_t branch;
  uint8_t jalr;
  uint8_t memRead;
  uint8_t memToReg;
  uint8_t aluOp;
  uint8_t memWrite;
  uint8_t aluSrc;
  uint8_t regWrite;
} State;
#endif

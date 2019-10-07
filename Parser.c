#include "Parser.h"

void loadInstructions(Instruction_Memory *i_mem, const char *trace) {
  printf("Loading trace file: %s\n", trace);

  FILE *fd = fopen(trace, "r");
  if (fd == NULL) {
    perror("Cannot open trace file. \n");
    exit(EXIT_FAILURE);
  }

  // Iterate all the assembly instructions
  char *line = NULL;
  size_t len = 0;
  ssize_t read;

  Addr PC = 0; // program counter points to the zeroth location initially.
  int IMEM_index = 0;
  while ((read = getline(&line, &len, fd)) != -1) {
    // Assign program counter
    i_mem->instructions[IMEM_index].addr = PC;
    // Extract operation
    char *raw_instr = strtok(line, " ");
    if (strcmp(raw_instr, "add") == 0 ||
    strcmp(raw_instr, "sub") == 0 ||
    strcmp(raw_instr, "sll") == 0 ||
    strcmp(raw_instr, "srl") == 0 ||
    strcmp(raw_instr, "xor") == 0 ||
    strcmp(raw_instr, "or")  == 0 ||
    strcmp(raw_instr, "and") == 0) {
      parseRType(raw_instr, &(i_mem->instructions[IMEM_index]));
      i_mem->last = &(i_mem->instructions[IMEM_index]);
    } else if(strcmp(raw_instr, "ld")   == 0 ||
    strcmp(raw_instr, "addi") == 0 ||
    strcmp(raw_instr, "slli") == 0 ||
    strcmp(raw_instr, "xori") == 0 ||
    strcmp(raw_instr, "srli") == 0 ||
    strcmp(raw_instr, "ori")  == 0 ||
    strcmp(raw_instr, "andi") == 0 ||
    strcmp(raw_instr, "jalr") == 0) {
      parseIType(raw_instr, &(i_mem->instructions[IMEM_index]));
      i_mem->last = &(i_mem->instructions[IMEM_index]);
    } else if(strcmp(raw_instr, "sd") == 0) {
      parseSType(raw_instr, &(i_mem->instructions[IMEM_index]));
      i_mem->last = &(i_mem->instructions[IMEM_index]);
    } else if(strcmp(raw_instr, "beq") == 0 ||
    strcmp(raw_instr, "bne") == 0 ||
    strcmp(raw_instr, "blt") == 0 ||
    strcmp(raw_instr, "bge") == 0) {
      parseBType(raw_instr, &(i_mem->instructions[IMEM_index]));
      i_mem->last = &(i_mem->instructions[IMEM_index]);
    } else if(strcmp(raw_instr, "jal") == 0) {
      parseJType(raw_instr, &(i_mem->instructions[IMEM_index]));
      i_mem->last = &(i_mem->instructions[IMEM_index]);
    }
    IMEM_index++;
    PC += 4;
  }
  fclose(fd);
}

void parseRType(char *opr, Instruction *instr) {
  instr->instruction = 0;
  unsigned opcode = 0;
  unsigned funct3 = 0;
  unsigned funct7 = 0;

  if(strcmp(opr, "add") == 0) {
    opcode = 0b0110011;
    funct3 = 0b000;
    funct7 = 0b0000000;
  } else if(strcmp(opr, "sub") == 0) {
    opcode = 0b0110011;
    funct3 = 0b000;
    funct7 = 0b0100000;
  } else if(strcmp(opr, "sll") == 0) {
    opcode = 0b0110011;
    funct3 = 0b001;
    funct7 = 0b0000000;
  } else if(strcmp(opr, "srl") == 0) {
    opcode = 0b0110011;
    funct3 = 0b101;
    funct7 = 0b0000000;
  } else if(strcmp(opr, "xor") == 0) {
    opcode = 0b0110011;
    funct3 = 0b100;
    funct7 = 0b0000000;
  } else if(strcmp(opr, "or")  == 0) {
    opcode = 0b0110011;
    funct3 = 0b110;
    funct7 = 0b0000000;
  } else if(strcmp(opr, "and") == 0) {
    opcode = 0b0110011;
    funct3 = 0b111;
    funct7 = 0b0000000;
  }
  char *reg = strtok(NULL, ", ");
  unsigned rd = regIndex(reg);

  reg = strtok(NULL, ", ");
  unsigned rs_1 = regIndex(reg);

  reg = strtok(NULL, "\n");
  removeWhitespace(reg);
  unsigned rs_2 = regIndex(reg);

  instr->instruction |= opcode;
  instr->instruction |= (rd << 7);
  instr->instruction |= (funct3 << (7 + 5));
  instr->instruction |= (rs_1 << (7 + 5 + 3));
  instr->instruction |= (rs_2 << (7 + 5 + 3 + 5));
  instr->instruction |= (funct7 << (7 + 5 + 3 + 5 + 5));
}

void parseIType(char *opr, Instruction *instr) {
  instr->instruction = 0;
  unsigned opcode = 0;
  unsigned funct3 = 0;
  unsigned funct7 = 0;

  if(strcmp(opr, "ld") == 0)  {
    opcode = 0b0000011;
    funct3 = 0b011;
    funct7 = 0b0000000;
  } else if(strcmp(opr, "addi") == 0) {
    opcode = 0b0010011;
    funct3 = 0b000;
    funct7 = 0b0000000;
  } else if(strcmp(opr, "slli") == 0) {
    opcode = 0b0010011;
    funct3 = 0b001;
    funct7 = 0b0000000;
  } else if(strcmp(opr, "xori") == 0) {
    opcode = 0b0010011;
    funct3 = 0b100;
    funct7 = 0b0000000;
  } else if(strcmp(opr, "srli") == 0) {
    opcode = 0b0010011;
    funct3 = 0b101;
    funct7 = 0b0000000;
  } else if(strcmp(opr, "ori")  == 0) {
    opcode = 0b0010011;
    funct3 = 0b110;
    funct7 = 0b0000000;
  } else if(strcmp(opr, "andi") == 0) {
    opcode = 0b0010011;
    funct3 = 0b111;
    funct7 = 0b0000000;
  } else if(strcmp(opr, "jalr") == 0) {
    opcode = 0b1100111;
    funct3 = 0b111;
    funct7 = 0b0000000;
  }
  char *reg = strtok(NULL, ", ");
  unsigned rd = regIndex(reg);

  int16_t imm;
  unsigned rs_1;
  if(strcmp(opr, "ld") == 0 || strcmp(opr, "jalr") == 0) {
    char *pEnd;
    imm = strtol(strtok(NULL, "("), &pEnd, 10);
    reg = strtok(NULL, ")");
    rs_1 = regIndex(reg);
  } else {
    reg = strtok(NULL, ", ");
    rs_1 = regIndex(reg);
    char *pEnd;
    imm = strtol(strtok(NULL, "\n"), &pEnd, 10);

  }
  instr->instruction |= opcode;
  instr->instruction |= (rd << 7);
  instr->instruction |= (funct3 << (7 + 5));
  instr->instruction |= (rs_1 << (7 + 5 + 3));
  instr->instruction |= (imm << (7 + 5 + 3 + 5));
}

void parseSType(char *opr, Instruction *instr) {
  instr->instruction = 0;
  unsigned opcode = 0;
  unsigned funct3 = 0;
  if(strcmp(opr, "sd") == 0) {
    opcode = 0b0100011;
    funct3 = 0b011;
  }
  char *reg = strtok(NULL, ", ");
  unsigned rs_2 = regIndex(reg);

  char *pEnd;
  int16_t imm = strtol(strtok(NULL, "("), &pEnd, 10);

  reg = strtok(NULL, ")");
  removeWhitespace(reg);
  unsigned rs_1 = regIndex(reg);

  instr->instruction |= opcode;
  instr->instruction |= ((imm & 0b11111) << 7);
  instr->instruction |= (funct3 << (7 + 5));
  instr->instruction |= (rs_1 << (7 + 5 + 3));
  instr->instruction |= (rs_2 << (7 + 5 + 3 + 5));
  instr->instruction |= ((imm & 0b111111100000) << (7 + 5 + 3 + 5 + 5 - 5));
}

void parseBType(char *opr, Instruction *instr) {
  instr->instruction = 0;
  unsigned opcode = 0;
  unsigned funct3 = 0;

  if (strcmp(opr, "beq") == 0) {
    opcode = 0b1100011;
    funct3 = 0b000;
  } else if(strcmp(opr, "bne") == 0) {
    opcode = 0b1100011;
    funct3 = 0b001;
  } else if(strcmp(opr, "blt") == 0) {
    opcode = 0b1100011;
    funct3 = 0b100;
  } else if(strcmp(opr, "bge") == 0) {
    opcode = 0b1100011;
    funct3 = 0b101;
  }

  char *reg = strtok(NULL, ", ");
  unsigned rs_1 = regIndex(reg);

  reg = strtok(NULL, ", ");
  unsigned rs_2 = regIndex(reg);

  char *pEnd;
  int16_t imm = strtol(strtok(NULL, ", "), &pEnd, 10);

  // Contruct instruction
  instr->instruction |= opcode;
  instr->instruction |= ((imm & 0b100000000000) >> (11 - 7));
  instr->instruction |= ((imm & 0b11110) << (7 + 1 - 1));
  instr->instruction |= (funct3 << (7 + 1 + 4));
  instr->instruction |= (rs_1 << (7 + 1 + 4 + 3));
  instr->instruction |= (rs_2 << (7 + 1 + 4 + 3 + 5));
  instr->instruction |= ((imm & 0b11111100000) << (7 + 1 + 4 + 3 + 5 + 5 - 5));
  instr->instruction |= ((imm & 0b1000000000000) << (7 + 1 + 4 + 3 + 5 + 5 + 6 - 12));
}
void parseJType(char *opr, Instruction *instr) {
  instr->instruction = 0;
  unsigned opcode = 0b0000000;

  if(strcmp(opr, "jal") == 0) {
    opcode = 0b1101111;
  }

  char *reg = strtok(NULL, ", ");
  unsigned rd = regIndex(reg);

  char *pEnd;
  int16_t imm = strtol(strtok(NULL, ", "), &pEnd, 10);

  // Contruct instruction
  instr->instruction |= opcode;
  instr->instruction |= (rd << 7);
  instr->instruction |= (imm & 0b11111111000000000000);
  instr->instruction |= ((imm & 0b100000000000) << (7 + 5 + 8 - 11));
  instr->instruction |= ((imm & 0b11111111110) << (7 + 5 + 8 + 1 - 1));
  instr->instruction |= ((imm & 0b100000000000000000000) << (7 + 5 + 8 + 1 + 10 - 20));
}

int regIndex(char *reg) {
  unsigned i = 0;
  for (; i < NUM_OF_REGS; i++) {
    if (strcmp(REGISTER_NAME[i], reg) == 0) {
      break;
    }
  }
  return i;
}

void removeWhitespace(char *reg) {
  unsigned i = 0;
  while(i < strlen(reg)) {
    if(reg[i] == *" ") {
      memmove(&reg[i], &(reg[i])+1, strlen(reg)-1);
      reg[strlen(reg)-1] = '\0';
    } else {
      i++;
    }
  }
}

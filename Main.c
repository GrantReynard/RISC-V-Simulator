#include <stdio.h>

#include "Core.h"
#include "Parser.h"

int main(int argc, const char *argv[])
{	
    if (argc != 2)
    {
        printf("Usage: %s %s\n", argv[0], "<trace-file>");

        return 0;
    }

    /* Task One */
    Instruction_Memory instr_mem;
    instr_mem.last = NULL;
    loadInstructions(&instr_mem, argv[1]);

    /* Task Two */
    Core *core = initCore(&instr_mem);

    /* Task Three - Simulation */
    while(core->tick(core));

    printf("Simulation is finished.\n");

    free(core);    
}

// TI File $Revision: 
// Checkin $Date: 
//###########################################################################
//
// FILE:    C28M35n_28x_RAM_lnk.cmd
//
// TITLE:   Linker Command File For 28M35n examples that run out of RAM
//
//          This ONLY includes all SARAM blocks on the 28M35n device.
//          This does not include flash or OTP.
//
//          Keep in mind that L0 and L1 are protected by the code
//          security module.
//
//          What this means is in most cases you will want to move to
//          another memory map file which has more memory defined.
//
//###########################################################################
// $TI Release:  $
// $Release Date:  $
//###########################################################################

/* ======================================================
// In addition to this memory linker command file,
// add the header linker command file directly to the project.
// The header linker command file is required to link the
// peripheral structures to the proper locations within
// the memory map.
//
// The header linker files are found in <base>\28M35x_headers\cmd
//
// For BIOS applications add:      F2837x_Headers_BIOS.cmd
// For nonBIOS applications add:   F2837x_Headers_nonBIOS.cmd
========================================================= */



/* Define the memory block start/length for the 28M35n
   PAGE 0 will be used to organize program sections
   PAGE 1 will be used to organize data sections

   Notes:
         Memory blocks on F2837x are uniform (ie same
         physical memory) in both PAGE 0 and PAGE 1.
         That is the same memory region should not be
         defined for both PAGE 0 and PAGE 1.
         Doing so will result in corruption of program
         and/or data.

         L0 block is mirrored - that is it
         can be accessed in high memory or low memory.
         For simplicity only one instance is used in this
         linker file.

         Contiguous SARAM memory blocks can be combined
         if required to create a larger memory block.
*/

MEMORY
{
PAGE 0 :
   /* BEGIN is used for the "boot to SARAM" bootloader mode   */

   BEGIN       : origin = 0x080000, length = 0x000002
   RAMLS0LS1   : origin = 0x008000, length = 0x001000
   RAMLS2      : origin = 0x009000, length = 0x000800
   RESET       : origin = 0x3FFFC0, length = 0x000002
   IQTABLES    : origin = 0x3FE000, length = 0x000B50     /* IQ Math Tables in Boot ROM */
   IQTABLES2   : origin = 0x3FEB50, length = 0x00008C     /* IQ Math Tables in Boot ROM */
   IQTABLES3   : origin = 0x3FEBDC, length = 0x0000AA	  /* IQ Math Tables in Boot ROM */

   FLASHN      : origin = 0x80002, length = 0x001FFD     /* on-chip FLASH */
   FLASHML      : origin = 0x82000, length = 0x004000     /* on-chip FLASH */
   /*FLASHL      : origin = 0x84000, length = 0x002000*/     /* on-chip FLASH */
   FLASHK      : origin = 0x86000, length = 0x002000     /* on-chip FLASH */
   FLASHJ      : origin = 0x88000, length = 0x008000     /* on-chip FLASH */
   FLASHI      : origin = 0x90000, length = 0x008000     /* on-chip FLASH */
   FLASHH      : origin = 0x98000, length = 0x008000     /* on-chip FLASH */
   FLASHG      : origin = 0xA0000, length = 0x008000     /* on-chip FLASH */
   FLASHF      : origin = 0xA8000, length = 0x008000     /* on-chip FLASH */
   FLASHE      : origin = 0xB0000, length = 0x008000      /* on-chip FLASH */
   FLASHD      : origin = 0xB8000, length = 0x002000      /* on-chip FLASH */
   FLASHC      : origin = 0xBA000, length = 0x002000      /* on-chip FLASH */
   FLASHB      : origin = 0xBC000, length = 0x002000     /* on-chip FLASH */
   
   FLASHN1      : origin = 0xC0000, length = 0x002000     /* on-chip FLASH */
   FLASHM1      : origin = 0xC2000, length = 0x002000     /* on-chip FLASH */
   FLASHL1      : origin = 0xC4000, length = 0x002000     /* on-chip FLASH */
   FLASHK1      : origin = 0xC6000, length = 0x002000     /* on-chip FLASH */
   FLASHJ1      : origin = 0xC8000, length = 0x008000     /* on-chip FLASH */
   FLASHI1      : origin = 0xD0000, length = 0x008000     /* on-chip FLASH */
   FLASHH1      : origin = 0xD8000, length = 0x008000     /* on-chip FLASH */
   FLASHG1      : origin = 0xE0000, length = 0x008000     /* on-chip FLASH */
   FLASHF1      : origin = 0xE8000, length = 0x008000     /* on-chip FLASH */
   FLASHE1      : origin = 0xF0000, length = 0x008000      /* on-chip FLASH */
   FLASHD1      : origin = 0xF8000, length = 0x002000      /* on-chip FLASH */
   FLASHC1      : origin = 0xFA000, length = 0x002000      /* on-chip FLASH */
   FLASHB1      : origin = 0xFC000, length = 0x002000     /* on-chip FLASH */
   FLASHA1      : origin = 0xFE000, length = 0x002000     /* on-chip FLASH */



   BOOTROM     : origin = 0x3FF27C, length = 0x000D44


PAGE 1 :

   RAMM0       : origin = 0x000004, length = 0x000250
   RAMM1       : origin = 0x000400, length = 0x000250     /* on-chip RAM block M1 */
   RAMD0       : origin = 0x00B000, length = 0x000250
   RAMD1       : origin = 0x00B800, length = 0x000250

   RAMLS3      : origin = 0x009800, length = 0x000800
   RAMLS4LS5   : origin = 0x00A000, length = 0x001000


   RAMGS0GS1  : origin = 0x00C000, length = 0x002000
   RAMGS3      : origin = 0x00F000, length = 0x000500
   RAMGS4      : origin = 0x010000, length = 0x000500
   RAMGS5      : origin = 0x011000, length = 0x000500
   RAMGS6      : origin = 0x012000, length = 0x000500
   RAMGS7      : origin = 0x013000, length = 0x000500
   RAMGS8      : origin = 0x014000, length = 0x000500
   RAMGS9      : origin = 0x015000, length = 0x000500
   RAMGS10     : origin = 0x016000, length = 0x000500
   RAMGS11     : origin = 0x017000, length = 0x000500
   RAMGS12     : origin = 0x018000, length = 0x000500
   RAMGS13     : origin = 0x019000, length = 0x000500
   RAMGS14     : origin = 0x01A000, length = 0x000500
   RAMGS15     : origin = 0x01B000, length = 0x000500

   FLASHA      : origin = 0xBE000, length = 0x002000      /* on-chip FLASH */

}


SECTIONS
{
   /* Setup for "boot to SARAM" mode:
      The codestart section (found in DSP28_CodeStartBranch.asm)
      re-directs execution to the start of user code.  */
   codestart        : > BEGIN,     PAGE = 0
   ramfuncs         : LOAD = FLASHML,
                      RUN = RAMLS0LS1,
                      LOAD_START(_RamfuncsLoadStart),
                      LOAD_SIZE(_RamfuncsLoadSize),
                      LOAD_END(_RamfuncsLoadEnd),
                      RUN_START(_RamfuncsRunStart),
                      PAGE = 0
   .text            : > FLASHML,    PAGE = 0
   .cinit           : > FLASHN|FLASHML,     PAGE = 0
   .pinit           : > FLASHN,     PAGE = 0
   .switch          : > FLASHML,     PAGE = 0
   .reset           : > RESET,     PAGE = 0, TYPE = DSECT /* not used, */

   .stack           : > RAMLS3,     PAGE = 1
   .ebss            : > RAMGS0GS1,     PAGE = 1
   .econst          : > FLASHML,     PAGE = 0
   .esysmem         : > RAMGS0GS1,     PAGE = 1


   
   //IQmath           : > RAML0,     PAGE = 0
   //IQmathTables     : > IQTABLES,  PAGE = 0, TYPE = NOLOAD

  /* Uncomment the section below if calling the IQNexp() or IQexp()
      functions from the IQMath.lib library in order to utilize the
      relevant IQ Math table in Boot ROM (This saves space and Boot ROM
      is 1 wait-state). If this section is not uncommented, IQmathTables2
      will be loaded into other memory (SARAM, Flash, etc.) and will take
      up space, but 0 wait-state is possible.
   */
   /*
   IQmathTables2    : > IQTABLES2, PAGE = 0, TYPE = NOLOAD
   {

              IQmath.lib<IQNexpTable.obj> (IQmathTablesRam)

   }
   */
   /* Uncomment the section below if calling the IQNasin() or IQasin()
      functions from the IQMath.lib library in order to utilize the
      relevant IQ Math table in Boot ROM (This saves space and Boot ROM
      is 1 wait-state). If this section is not uncommented, IQmathTables2
      will be loaded into other memory (SARAM, Flash, etc.) and will take
      up space, but 0 wait-state is possible.
   */
   /*
   IQmathTables3    : > IQTABLES3, PAGE = 0, TYPE = NOLOAD
   {

              IQmath.lib<IQNasinTable.obj> (IQmathTablesRam)

   }
   */

}

/*
//===========================================================================
// End of file.
//===========================================================================
*/

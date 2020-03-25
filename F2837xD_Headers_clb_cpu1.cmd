
/* this linker command file is to be included if user wants to use the CLB (Configurable Logic Block
 * This linker command file works as an addendum ot the already existing Flash/RAM linker command file
*/ 
MEMORY
{
 PAGE 0:    /* Program Memory */

 PAGE 1:    /* Data Memory */

   CLB1_LOGICCFG      : origin = 0x003000, length = 0x000040
   CLB1_LOGICCTL      : origin = 0x003100, length = 0x000040
   CLB1_DATAXCHG      : origin = 0x003200, length = 0x000200
   CLB2_LOGICCFG      : origin = 0x003400, length = 0x000040
   CLB2_LOGICCTL      : origin = 0x003500, length = 0x000040
   CLB2_DATAXCHG      : origin = 0x003600, length = 0x000200
   CLB3_LOGICCFG      : origin = 0x003800, length = 0x000040
   CLB3_LOGICCTL      : origin = 0x003900, length = 0x000040
   CLB3_DATAXCHG      : origin = 0x003A00, length = 0x000200
   CLB4_LOGICCFG      : origin = 0x003C00, length = 0x000040
   CLB4_LOGICCTL      : origin = 0x003D00, length = 0x000040
   CLB4_DATAXCHG      : origin = 0x003E00, length = 0x000200

}


SECTIONS
{

   Clb1LogicCfgRegsFile	: > CLB1_LOGICCFG,	PAGE = 1
   Clb1LogicCtrlRegsFile : > CLB1_LOGICCTL,	PAGE = 1
   Clb1DataExchgRegsFile : > CLB1_DATAXCHG,	PAGE = 1
   Clb2LogicCfgRegsFile : > CLB2_LOGICCFG,	PAGE = 1
   Clb2LogicCtrlRegsFile : > CLB2_LOGICCTL,	PAGE = 1
   Clb2DataExchgRegsFile : > CLB2_DATAXCHG,	PAGE = 1
   Clb3LogicCfgRegsFile : > CLB3_LOGICCFG,	PAGE = 1
   Clb3LogicCtrlRegsFile : > CLB3_LOGICCTL,	PAGE = 1
   Clb3DataExchgRegsFile : > CLB3_DATAXCHG,	PAGE = 1
   Clb4LogicCfgRegsFile : > CLB4_LOGICCFG,	PAGE = 1
   Clb4LogicCtrlRegsFile : > CLB4_LOGICCTL,	PAGE = 1
   Clb4DataExchgRegsFile : > CLB4_DATAXCHG,	PAGE = 1

}

/*
//===========================================================================
// End of file.
//===========================================================================
*/

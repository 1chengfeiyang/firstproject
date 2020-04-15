/**
  ******************************************************************************
  * @file    InfraRed/IR_Decoding_PWMI/src/ir_commands.c
  * @author  MCD Application Team
  * @version V2.0.0
  * @date    25-January-2012
  * @brief   This file provides all the InfraRed commands.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * FOR MORE INFORMATION PLEASE READ CAREFULLY THE LICENSE AGREEMENT FILE
  * LOCATED IN THE ROOT DIRECTORY OF THIS FIRMWARE PACKAGE.
  *
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "ir_decode.h"
#include "stm32f10x.h"

/** @addtogroup STM32F10x_Infrared_Decoders
  * @{
  */

/** @addtogroup IR_Decoding_PWMI
  * @{
  */

/** @addtogroup IR_COMMANDS
  * @brief IR_COMMANDS driver modules
  * @{
  */

/** @defgroup IR_COMMANDS_Private_TypesDefinitions
  * @{
  */

/**
  * @}
  */

/** @defgroup IR_COMMANDS_Private_Defines
  * @{
  */
/**
  * @}  
  */


/** @defgroup IR_COMMANDS_Private_Macros
  * @{
  */
/**
  * @}
  */

/** @defgroup IR_COMMANDS_Private_Variables
  * @{
  */
#ifdef IR_SIRC_PROTOCOL
/* Table of SIRC address */
uint8_t* IR_devices[32] = {
         "        SAT         ",  /* 0 */
         "         TV         ",  /* 1 */
         "        VTR1        ",  /* 2 */
         "      Teletext      ",  /* 3 */
         "     widescreen     ",  /* 4 */
         "                    ",  /* 5 */
         "     laser Disk     ",  /* 6 */
         "        VTR2        ",  /* 7 */
         "                    ",  /* 8 */
         "                    ",  /* 9 */
         "                    ",  /* 10 */
         "        VTR3        ",  /* 11 */
         "Surround Sound Proc ",  /* 12 */
         "                    ",  /* 13 */
         "                    ",  /* 14 */
         "                    ",  /* 15 */
         "    Audio/Cassette  ",  /* 16 */
         "         CD         ",  /* 17 */
         "     Equalizer      ",  /* 18 */
         "                    ",  /* 19 */
         "                    ",  /* 20 */
         "                    ",  /* 21 */
         "                    ",  /* 22 */
         "                    ",  /* 23 */
         "                    ",  /* 24 */
         "                    ",  /* 25 */
         "      DVD Player    ",  /* 26 */
         "                    ",  /* 27 */
         "                    ",  /* 28 */
         "                    ",  /* 29 */
         "                    ",  /* 30 */
         "                    ",  /* 31 */
};

/* Table of SIRC commands */
uint8_t* IR_Commands[128] = {
        "        Num1        ",   /* 0 */
        "        Num2        ",   /* 1 */
        "        Num3        ",   /* 2 */
        "        Num4        ",   /* 3 */
        "        Num5        ",   /* 4 */
        "        Num6        ",   /* 5 */
        "        Num7        ",   /* 6 */
        "        Num8        ",   /* 7 */
        "        Num9        ",   /* 8 */
        "        Num0        ",   /* 9 */
        "       reserved     ",   /* 10 */
        "        Enter       ",   /* 11 */
        "      reserved      ",   /* 12 */
        "      reserved      ",   /* 13 */
        "      Reserved      ",   /* 14 */
        "      Reserved      ",   /* 15 */
        "     Channel Up     ",   /* 16 */
        "     Channel Down   ",   /* 17 */
        "       Volume Up    ",   /* 18 */
        "      Volume Down   ",   /* 19 */
        "         Mute       ",   /* 20 */
        "        Power       ",   /* 21 */
        "        Reset       ",   /* 22 */
        "      Audio Mode    ",   /* 23 */
        "      Picture Up    ",   /* 24 */
        "     Picture Down   ",   /* 25 */
        "     Colour Up      ",   /* 26 */
        "     Colour Down    ",   /* 27 */
        "      Reserved      ",   /* 28 */
        "      Reserved      ",   /* 29 */
        "    Brightness Up   ",   /* 30 */
        "   Brightness Down  ",   /* 31 */
        "       Hue Up       ",   /* 32 */
        "      Hue Down      ",   /* 33 */
        "    Sharpness Up    ",   /* 34 */
        "   Sharpness Down   ",   /* 35 */
        "   Select TV Tuner  ",   /* 36 */
        "   Balance Left     ",   /* 37 */
        "   Balance Right    ",   /* 38 */
        "      Reserved      ",   /* 39 */
        "      Reserved      ",   /* 40 */
        "    Surround on/off ",   /* 41 */
        "      Reserved      ",   /* 42 */
        "      Reserved      ",   /* 43 */
        "      Reserved      ",   /* 44 */
        "      Reserved      ",   /* 45 */
        "      Reserved      ",   /* 46 */
        "     Power off      ",   /* 47 */
        "     Time Display   ",   /* 48 */
        "      Reserved      ",   /* 49 */
        "      Reserved      ",   /* 50 */
        "      Reserved      ",   /* 51 */
        "      Reserved      ",   /* 52 */
        "      Reserved      ",   /* 53 */
        "     Sleep Timer    ",   /* 54 */
        "      Reserved      ",   /* 55 */
        "      Reserved      ",   /* 56 */
        "      Reserved      ",   /* 57 */
        "   Channel Display  ",   /* 58 */
        "    Channel Jump    ",   /* 59 */
        "      Reserved      ",   /* 60 */
        "      Reserved      ",   /* 61 */
        "      Reserved      ",   /* 62 */
        "      Reserved      ",   /* 63 */
        " Select Input video1",   /* 64 */
        " Select Input video2",   /* 65 */
        " Select Input video3",   /* 66 */
        "      Reserved      ",   /* 67 */
        "      Reserved      ",   /* 68 */
        "      Reserved      ",   /* 69 */
        "      Reserved      ",   /* 70 */
        "      Reserved      ",   /* 71 */
        "      Reserved      ",   /* 72 */
        "      Reserved      ",   /* 73 */
        "noiseReductionon/off",   /* 74 */
        "      Reserved      ",   /* 75 */
        "      Reserved      ",   /* 76 */
        "      Reserved      ",   /* 77 */
        "  Cable/Broadcast   ",   /* 78 */
        " Notch Filter on/off",   /* 79 */
        "      Reserved      ",   /* 80 */
        "      Reserved      ",   /* 81 */
        "      Reserved      ",   /* 82 */
        "      Reserved      ",   /* 83 */
        "      Reserved      ",   /* 84 */
        "      Reserved      ",   /* 85 */
        "      Reserved      ",   /* 86 */
        "      Reserved      ",   /* 87 */
        "   PIP Channel Up   ",   /* 88 */
        "   PIP Channel Down ",   /* 89 */
        "      Reserved      ",   /* 90 */
        "      PIP on        ",   /* 91 */
        "    Freeze Screen   ",   /* 92 */
        "      Reserved      ",   /* 93 */
        "     PIP Position   ",   /* 94 */
        "      PIP Swap      ",   /* 95 */
        "        Guide       ",   /* 96 */
        "     Video Setup    ",   /* 97 */
        "     Audio Setup    ",   /* 98 */
        "     Exit Setup     ",   /* 99 */
        "      Reserved      ",   /* 100 */
        "      Reserved      ",   /* 101 */
        "      Reserved      ",   /* 102 */
        "      Reserved      ",   /* 103 */
        "      Reserved      ",   /* 104 */
        "      Reserved      ",   /* 105 */
        "      Reserved      ",   /* 106 */
        "    Auto Program    ",   /* 107 */
        "      Reserved      ",   /* 108 */
        "      Reserved      ",   /* 109 */
        "      Reserved      ",   /* 110 */
        "       PIP off      ",   /* 111 */
        "       Treble Up    ",   /* 112 */
        "      Treble Down   ",   /* 113 */
        "       Bass Up      ",   /* 114 */
        "      Bass Down     ",   /* 115 */
        "       + Key        ",   /* 116 */
        "       - Key        ",   /* 117 */
        "      Reserved      ",   /* 118 */
        "    Slipt Screen    ",   /* 119 */
        "    Add Channel     ",   /* 120 */
        "   Delete Channel   ",   /* 121 */
        "      Reserved      ",   /* 122 */
        "      Reserved      ",   /* 123 */
        "      Reserved      ",   /* 124 */
        "  Trinitone on/off  ",   /* 125 */
        "      Reserved      ",   /* 126 */
        "Display a red Rtests"    /* 127 */
   };
#elif defined (IR_RC5_PROTOCOL)
/* RC5 address table */
uint8_t* rc5_devices[32] = {
        "         TV1        ",                  /*  0 */
        "         TV2        ",                  /*  1 */
        "      Video Text    ",                  /*  2 */
        "     Extension TV   ",                  /*  3 */
        "   LaserVideoPlayer ",                  /*  4 */
        "         VCR1       ",                  /*  5 */
        "         VCR2       ",                  /*  6 */
        "       Reserved     ",                  /*  7 */
        "         Sat1       ",                  /*  8 */
        "     Extension VCR  ",                  /*  9 */
        "         Sat2       ",                  /* 10 */
        "       Reserved     ",                  /* 11 */
        "       CD Video     ",                  /* 12 */
        "       Reserved     ",                  /* 13 */
        "       CD Photo     ",                  /* 14 */
        "       Reserved     ",                  /* 15 */
        "   Preampli Audio 1 ",                  /* 16 */
        "        Tuner       ",                  /* 17 */
        "    Analog Magneto  ",                  /* 18 */
        "   Preampli Audio 2 ",                  /* 19 */
        "         CD         ",                  /* 20 */
        "      Rack Audio    ",                  /* 21 */
        " Audio Sat Receiver ",                  /* 22 */
        "     DDC Magneto    ",                  /* 23 */
        "      Reserved      ",                  /* 24 */
        "      Reserved      ",                  /* 25 */
        "       CDRW         ",                  /* 26 */
        "      Reserved      ",                  /* 27 */
        "      Reserved      ",                  /* 28 */
        "      Reserved      ",                  /* 29 */
        "      Reserved      ",                  /* 30 */
        "      Reserved      "                   /* 31 */  	             
       };

/* RC5 commands table*/
uint8_t* rc5_Commands[128] = {
        "        Num0        ",                                       /* 0 */
        "        Num1        ",                                       /* 1 */
        "        Num2        ",                                       /* 2 */
        "        Num3        ",                                       /* 3 */
        "        Num4        ",                                       /* 4 */
        "        Num5        ",                                       /* 5 */
        "        Num6        ",                                       /* 6 */
        "        Num7        ",                                       /* 7 */
        "        Num8        ",                                       /* 8 */
        "        Num9        ",                                       /* 9 */
        "      TV Digits     ",                                       /* 10 */
        "       TV Freq      ",                                       /* 11 */
        "      TV StandBy    ",                                       /* 12 */
        "    TV Mute On-Off  ",                                       /* 13 */
        "    TV Preference   ",                                       /* 14 */
        "     TV Display     ",                                       /* 15 */
        "     Volume Up      ",                                       /* 16 */
        "    Volume Down     ",                                       /* 17 */
        "    Brightness Up   ",                                       /* 18 */
        "   Brightness Down  ",                                       /* 19 */
        " Color Saturation Up",                                       /* 20 */
        "ColorSaturation Down",                                       /* 21 */
        "       Bass Up      ",                                       /* 22 */
        "      Bass Down     ",                                       /* 23 */
        "      Treble Up     ",                                       /* 24 */
        "     Treble Down    ",                                       /* 25 */
        "    Balance Right   ",                                       /* 26 */
        "    BalanceLeft     ",                                       /* 27 */
        "   TV Contrast Up   ",                                       /* 28 */
        "  TV Contrast Down  ",                                       /* 29 */
        "   TV Search Up     ",                                       /* 30 */
        "  TV tint-hue Down  ",                                       /* 31 */
        "   TV CH Prog Up    ",                                       /* 32 */
        "   TV CH ProgDown   ",                                       /* 33 */
        "TV Last prog-channel",                                       /* 34 */
        " TV Select language ",                                       /* 35 */
        " TV Spacial Stereo  ",                                       /* 36 */
        "  TV Stereo Mono    ",                                       /* 37 */
        "  TV Sleep Timer    ",                                       /* 38 */
        "   TV tint-hue Up   ",                                       /* 39 */
        "   TV RF Switch     ",                                       /* 40 */
        "   TV Store-VOTE    ",                                       /* 41 */
        "      TV Time       ",                                       /* 42 */
        "   TV Scan Fwd Inc  ",                                       /* 43 */
        "    TV Decrement    ",                                       /* 44 */
        "      Reserved      ",                                       /* 45 */
        "   TV control-menu  ",                                       /* 46 */
        "    TV Show Clock   ",                                       /* 47 */
        "       TV Pause     ",                                       /* 48 */
        "   TV Erase Entry   ",                                       /* 49 */
        "     TV Rewind      ",                                       /* 50 */
        "      TV Goto       ",                                       /* 51 */
        "      TV Wind       ",                                       /* 52 */
        "      TV Play       ",                                       /* 53 */
        "      TV Stop       ",                                       /* 54 */
        "      TV Record     ",                                       /* 55 */
        "    TV External 1   ",                                       /* 56 */
        "    TV External 2   ",                                       /* 57 */
        "      Reserved      ",                                       /* 58 */
        "     TV Advance     ",                                       /* 59 */
        "   TV TXT-TV toggle ",                                       /* 60 */
        "  TV System StandBy ",                                       /* 61 */
        "TV Picture Crispener",                                       /* 62 */
        "    System Select   ",                                       /* 63 */
        "      Reserved      ",                                       /* 64 */
        "      Reserved      ",                                       /* 65 */
        "      Reserved      ",                                       /* 66 */
        "      Reserved      ",                                       /* 67 */
        "      Reserved      ",                                       /* 68 */
        "      Reserved      ",                                       /* 69 */
        "  TV Speech Music   ",                                       /* 70 */
        "  DIM Local Display ",                                       /* 71 */
        "      Reserved      ",                                       /* 72 */
        "      Reserved      ",                                       /* 73 */
        "      Reserved      ",                                       /* 74 */
        "      Reserved      ",                                       /* 75 */
        "      Reserved      ",                                       /* 76 */
        "Inc Control Setting ",                                       /* 77 */
        "Dec Control Setting ",                                       /* 78 */
        "   TV Sound Scroll  ",                                       /* 79 */
        "        Step Up     ",                                       /* 80 */
        "       Step Down    ",                                       /* 81 */
        "        Menu On     ",                                       /* 82 */
        "        Menu Off    ",                                       /* 83 */
        "       AV Status    ",                                       /* 84 */
        "      Step Left     ",                                       /* 85 */
        "      Step Right    ",                                       /* 86 */
        "     Acknowledge    ",                                       /* 87 */
        "      PIP On Off    ",                                       /* 88 */
        "       PIP Shift    ",                                       /* 89 */
        "    PIP Main Swap   ",                                       /* 90 */
        "    Strobe On Off   ",                                       /* 91 */
        "     Multi Strobe   ",                                       /* 92 */
        "      Main Frozen   ",                                       /* 93 */
        "  3div9 Multi scan  ",                                       /* 94 */
        "      PIPSelect     ",                                       /* 95 */
        "      MultiPIP      ",                                       /* 96 */
        "     Picture DNR    ",                                       /* 97 */
        "     Main Stored    ",                                       /* 98 */
        "     PIP Strobe     ",                                       /* 99 */
        "    Stored Picture  ",                                       /* 100 */
        "      PIP Freeze    ",                                       /* 101 */
        "      PIP Step Up   ",                                       /* 102 */
        "    PIP Step Down   ",                                       /* 103 */
        "    TV PIP Size     ",                                       /* 104 */
        "  TV Picture Scroll ",                                       /* 105 */
        " TV Actuate Colored ",                                       /* 106 */
        "        TV Red      ",                                       /* 107 */
        "       TV Green     ",                                       /* 108 */
        "      TV Yellow     ",                                       /* 109 */
        "       TV Cyan      ",                                       /* 110 */
        "    TV Index White  ",                                       /* 111 */
        "        TV Next     ",                                       /* 112 */
        "     TV Previous    ",                                       /* 113 */
        "       Reserved     ",                                       /* 114 */
        "       Reserved     ",                                       /* 115 */
        "       Reserved     ",                                       /* 116 */
        "       Reserved     ",                                       /* 117 */
        "       Sub Mode     ",                                       /* 118 */
        "   Option Sub Mode  ",                                       /* 119 */
        "       Reserved     ",                                       /* 120 */
        "       Reserved     ",                                       /* 121 */
        "TV Store Open Close ",                                       /* 122 */
        "       Connect      ",                                       /* 123 */
        "     Disconnect     ",                                       /* 124 */
        "       Reserved     ",                                       /* 125 */
        "  TV Movie Expand   ",                                       /* 126 */
        "  TV Parental Access"                                        /* 127 */
   };

#endif
/**
  * @}
  */

/** @defgroup IR_COMMANDS_Private_FunctionPrototypes
  * @{
  */
/**
  * @}
  */

/** @defgroup IR_COMMANDS_Private_Functions
  * @{
  */
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */
/******************* (C) COPYRIGHT 2012 STMicroelectronics *****END OF FILE****/

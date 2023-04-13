/***********************************************************************
               Objet decodeur de teleinformation client (TIC)
               format Linky "historique" ou anciens compteurs
               electroniques.

Lit les trames et decode les groupes :   |<--  Switches d'option   -->|
                                         | Base | HPHC | IMono | ITri |
 PAPP   : puissance apparente en VA......|   X  |   X  |   X   |   X  |
 BASE   : index general compteur en Wh...|   X  |      |       |      |
 HCHC   : index heures creuses en Wh.....|      |   X  |       |      |
 HCHP   : index heures pleines en Wh.....|      |   X  |       |      |
 PTEC   : periode tarifaire en cours.....|      |   X  |       |      |
 IINST  : intensite instantanee en A.....|      |      |   X   |      |
 IINST1 : intensite instantanee en A.....|      |      |       |   X  |
 IINST2 : intensite instantanee en A.....|      |      |       |   X  |
 IINST3 : intensite instantanee en A.....|      |      |       |   X  |

Reference : ERDF-NOI-CPT_54E V3

V06 : MicroQuettas mars 2018

V10a : initial version, parametric. Compiled OK on 22/10/19.
V10b : fixed bug in ptecIsNew().
V10c : added LKYSIMINPUT mode
V10d : adapted to Arduino Uno and Mega.

***********************************************************************/
#ifndef _LinkyHistTIC
#define _LinkyHistTIC true

/********************** Processor selection ***************************/
//#define ARDUINOMEGA Serial1    /* Define the serial input used   */
                               /* when running on a Mega,        */
                               /* comment out on an AVR328 (Uno) */
                               /* On a Mega, call the constructor */
                               /* without parameters. If parameters */
                               /* (pin numbers) are given, they will */
                               /* be ignored.                        */

/********************** Configuration switches ************************/
//#define LINKYDEBUG true     /* Verbose debugging mode */
//#define LKYSIMINPUT true    /* Simulated Linky input on Serial */
                              /* AVR328 (Uno) processor only */

/************* tariffs and intensities configuration ******************/
//#define LKY_Base true        /* Exclusif avec LKY_HPHC */
#define LKY_HPHC true        /* Exclusif avec LKY_Base */
//#define LKY_IMono true       /* Exclusif avec LKY_ITri */
//#define LKY_ITri true        /* Exclusif avec LKY_IMono */

/****************************** Autoconf *****************************/
#if (defined (LKY_Base) && defined (LKY_HPHC))
#undef LKY_Base
#endif

#if (defined (LKY_IMono) && defined (LKY_ITri))
#undef LKY_IMono
#endif

#if (defined (LKYSIMINPUT) && defined (ARDUINOMEGA))
#undef ARDUINOMEGA
#endif

#if !(defined (LKYSIMINPUT) || defined (ARDUINOMEGA))
#define LKYSOFTSERIAL true
#endif

/*************************** Includes ********************************/
#ifdef LKYSOFTSERIAL
#include <SoftwareSerial.h>
#endif

/********************** Defines and consts ***************************/
#define CLy_BfSz 24            /* Maximum size of the Rx buffers */

const uint16_t CLy_Bds = 1200; /* Transmission speed in bds */

const uint8_t CpinRx_def = 10;
const uint8_t CpinTx_def = 11;

/******************************** Class *******************************
      LinkyHistTIC : Linky historique TIC (teleinformation client)
***********************************************************************/

class LinkyHistTIC
  {
  public:
    #ifdef LKYSOFTSERIAL
    LinkyHistTIC(uint8_t pin_Rx, uint8_t pin_Tx);    /* Constructor */
    #else
    LinkyHistTIC(uint8_t pin_Rx = CpinRx_def, \
                 uint8_t pin_Tx = CpinTx_def);       /* Constructor */
    #endif    

    void Init(uint16_t BdR = CLy_Bds);  
                        /* Initialisation, call from setup() */
    void Update();      /* Update, call from loop() */

    bool pappIsNew();   /* Returns true if papp has changed */
    uint16_t papp();    /* Returns papp in VA */

    #ifdef LKY_Base
    bool baseIsNew();   /* Returns true if base has changed */
    uint32_t base();    /* Returns base index in Wh */
    #endif

    #ifdef LKY_HPHC
    enum Tarifs:uint8_t {C_HPleines, C_HCreuses};
    bool hchcIsNew();   /* Returns true if hchc has changed */
    uint32_t hchc();    /* Index heures creuses en Wh */
    bool hchpIsNew();   /* Returns true if hchp has changed */
    uint32_t hchp();    /* Index heures pleines en Wh */
    bool ptecIsNew();   /* Returns true if ptec has changed */
    uint8_t ptec();     /* Periode tarifaire en cours (0 = HP, 1 = HC) */
    #endif

    #ifdef LKY_IMono
    bool iinstIsNew();  /* Returns true if iinst has changed */
    uint8_t iinst();    /* Returns iinst in A */
    #endif

    #ifdef LKY_ITri
    enum Phases:uint8_t {C_Phase_1, C_Phase_2, C_Phase_3};
    bool iinstIsNew(uint8_t Ph);  /* Returns true if iinst(Ph)
                                   * has changed */
    uint8_t iinst(uint8_t Ph);    /* Returns iinst(Ph) in A */
    #endif

  private:
    char _BfA[CLy_BfSz];        /* Buffer A */
    char _BfB[CLy_BfSz];        /* Buffer B */

    uint8_t _FR;                /* Flag register */
    uint8_t _DNFR;              /* Data new flag register */

    uint16_t _papp;

    #ifdef LKY_Base
    uint32_t _base;      /* Index base */
    #endif

    #ifdef LKY_HPHC
    uint32_t _hchc;      /* Index heures creuses en Wh */
    uint32_t _hchp;      /* Index heures pleines en Wh */
    uint8_t  _ptec;      /* Periode tarifaire en cours :
                          * 0 = HP ; 1 = HC */
    #endif

    #ifdef LKY_IMono
    uint8_t _iinst;     /* Intensite instantanee */
    #endif

    #ifdef LKY_ITri
    uint8_t _iinst[3];  /* Intensite instantanee pour chaque phase */
    #endif

    #ifdef LKYSOFTSERIAL
    SoftwareSerial _LRx;   /* Needs to be constructed at the same time
                            * as LinkyHistTIC. Cf. Special syntax
                            * (initialisation list) in LinkyHistTIC
                            * constructor */
    uint8_t _pin_Rx;
    uint8_t _pin_Tx;
    #endif

    char *_pRec;     /* Reception pointer in the buffer */
    char *_pDec;     /* Decode pointer in the buffer */
    uint8_t _iRec;   /* Received char index */
    uint8_t _iCks;   /* Index of Cks in the received message */
    uint8_t _GId;    /* Group identification */

  };
  
#endif /* _LinkyHistTIC */
/*************************** End of code ******************************/

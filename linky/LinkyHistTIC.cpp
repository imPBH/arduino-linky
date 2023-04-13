/***********************************************************************
               Objet decodeur de teleinformation client (TIC)
               format Linky "historique" ou anciens compteurs
               electroniques.

Lit les trames et decode les groupes :
  BASE  : (base) index general compteur en Wh,
  IINST : (iinst) intensite instantanee en A,
  PAPP  : (papp) puissance apparente en VA.

Reference : ERDF-NOI-CPT_54E V3

V06 : MicroQuettas mars 2018

V10a : initial version, parametric.
V10b : fixed bug in ptecIsNew().
V10c : added LKYSIMINPUT mode
V10d : adapted to Arduino Uno and Mega.

***********************************************************************/

/***************************** Includes *******************************/
#include <string.h>
#include <Streaming.h>
#include "LinkyHistTIC.h"

/***********************************************************************
                  Objet recepteur TIC Linky historique

 Trame historique :
  - delimiteurs de trame :     <STX> trame <ETX>
    <STX> = 0x02
    <ETX> = 0x03

  - groupes dans une trame : <LF>lmnopqrs<SP>123456789012<SP>C<CR>
    <LR> = 0x0A
    lmnopqrs = label 8 char max
    <SP> = 0x20 ou 0x09 (<HT>)
    123456789012 = data 12 char max
    C = checksum 1 char
    <CR> = 0x0d
       Longueur max : label + data = 7 + 9 = 16

  _FR : flag register

    |  7  |  6  |   5  |   4  |   3  |  2  |   1  |   0  |
    |     |     | _Dec | _GId | _Cks |     | _RxB | _Rec |

     _Rec : receiving
     _RxB : receive in buffer B, decode in buffer A
     _GId : Group identification
     _Dec : decode data

  _DNFR : data available flags

    |  7  |    6    |    5    |    4    |   3   |   2   |   1   |   0   |
    |     | _iinst3 | _iinst2 | _iinst1 | _ptec | _hchc | _hchp | _papp |
    |     |         |         |  _iinst |       |       | _base |       |

                              ********************

  Exemple of group :
       <LF>lmnopqr<SP>123456789<SP>C<CR>
           0123456  7 890123456  7 8  9
     Cks:  xxxxxxx  x xxxxxxxxx    ^
                                   |  iCks
    Minimum length group  :
           <LF>lmno<SP>12<SP>C<CR>
               0123  4 56  7 8  9
     Cks:      xxxx  x xx    ^
                             |  iCks

    The storing stops at CRC (included), ie a max of 19 chars

***********************************************************************/



/****************************** Macros ********************************/
#ifdef LKYSOFTSERIAL
#define _LKY _LRx           /*_LRx = software serial instance */
#else
#ifdef ARDUINOMEGA
#define _LKY ARDUINOMEGA    /* Arduino Mega serial port */
#else
#define _LKY Serial         /* Simulated input trough Serial */
#endif
#endif

#ifndef SetBits
#define SetBits(Data, Mask) \
Data |= Mask
#endif

#ifndef ResetBits
#define ResetBits(Data, Mask) \
Data &= ~Mask
#endif

#ifndef InsertBits
#define InsertBits(Data, Mask, Value) \
Data &= ~Mask;\
Data |= (Value & Mask)
#endif

#ifndef P1
#define P1(Name) const char Name[] PROGMEM
#endif

/************************* Defines and const  **************************/

const uint8_t bLy_Rec = 0x01;  /* Receiving */
const uint8_t bLy_RxB = 0x02;  /* Receive in buffer B */
const uint8_t bLy_Cks = 0x08;  /* Check Cks */
const uint8_t bLy_GId = 0x10;  /* Group identification */
const uint8_t bLy_Dec = 0x20;  /* Decode */

const char Car_SP = 0x20;     /* Char space */
const char Car_HT = 0x09;     /* Horizontal tabulation */

const uint8_t CLy_MinLg = 8;  /* Minimum useful message length */

const char CLy_Sep[] = {Car_SP, Car_HT, '\0'};  /* Separators */

/***  const below are used for _GId and for flag rank in _DNFR ***/
const uint8_t  CLy_papp = 0,  \
  CLy_base = 1, CLy_hchp = 1, CLy_hchc = 2, CLy_ptec = 3,  \
  CLy_iinst = 4, CLy_iinst1 = 4, CLy_iinst2 = 5, CLy_iinst3 = 6;

#ifdef LKY_IMono
enum Phases:uint8_t {C_Phase_1, C_Phase_2, C_Phase_3};
#endif

/************************* Donnees en progmem *************************/
P1(PLy_papp)  = "PAPP";

#ifdef LKY_Base
P1(PLy_base)  = "BASE";
#endif

#ifdef LKY_HPHC
P1(PLy_hchp)  = "HCHP";
P1(PLy_hchc)  = "HCHC";
P1(PLy_ptec)  = "PTEC";
P1(PLy_HC)    = "HC";         /* Tarif HC (default = HP) */
#endif

#if (defined (LKY_IMono) || defined(LKY_ITri))
P1(PLy_iinst) = "IINST";
#endif

/*************** Constructor, methods and properties ******************/
#ifdef LKYSOFTSERIAL
LinkyHistTIC::LinkyHistTIC(uint8_t pin_Rx, uint8_t pin_Tx) \
      : _LRx (pin_Rx, pin_Tx)  /* Software serial constructor
                                * Achtung : special syntax */
#else
LinkyHistTIC::LinkyHistTIC(uint8_t pin_Rx, uint8_t pin_Tx)
#endif

  {
  _FR = 0;
  _DNFR = 0;
  _pRec = _BfA;    /* Receive in A */
  _pDec = _BfB;    /* Decode in B */
  _iRec = 0;
  _iCks = 0;
  _GId = CLy_papp;
  
  #ifdef LKYSOFTSERIAL
  _pin_Rx = pin_Rx;
  _pin_Tx = pin_Tx;
  #endif
  };

void LinkyHistTIC::Init(uint16_t BdR)
  {
  uint8_t i;

  #ifdef LKYSOFTSERIAL
  /* Initialise the SoftwareSerial */
  pinMode (_pin_Rx, INPUT_PULLUP);
  pinMode (_pin_Tx, OUTPUT);
  #endif

  _LKY.begin(BdR);  /* When LKYSIMINPUT is activated, will adjust */
                    /* the Serial Baud rate to that of the Linky */

  /* Clear all data buffers */
  _papp = 0;

  #ifdef LKY_Base
  _base = 0;
  #endif

  #ifdef LKY_HPHC
  _hchc = 0;
  _hchp = 0;
  _ptec = 255;   /* 1st input, whatever, will trigger ptecIsNew() */
  #endif

  #ifdef LKY_IMono
  _iinst = 0;
  #endif

  #ifdef LKY_ITri
  for (i = 0; i < 3; i++)
    {
    _iinst[i] = 0;
    }
  #endif
  }

void LinkyHistTIC::Update()
  {   /* Called from the main loop */
  char c;
  uint8_t cks, i, j;
  uint32_t ba;
  uint16_t pa;
  bool Run = true;

  /* Achtung : actions are in the reverse order to prevent
   *           execution of all actions in the same turn of
   *           the loop */

  /* 1st part, last action : decode information */
  if (_FR & bLy_Dec)
    {
    ResetBits(_FR, bLy_Dec);     /* Clear requesting flag */
    _pDec = strtok(NULL, CLy_Sep);

    switch (_GId)
      {
      case CLy_papp:
        pa = atoi(_pDec);
        if (_papp != pa)
          {  /* New value for papp */
          _papp = pa;
          SetBits(_DNFR, (1<<CLy_papp));
          }
        break;

      #ifdef LKY_Base
      case CLy_base:
        ba = atol(_pDec);
        if (_base != ba)
          {  /* New value for _base */
          _base = ba;
          SetBits(_DNFR, (1<<CLy_base));
          }
        break;
      #endif

      #ifdef LKY_HPHC
      case CLy_hchp:
        ba = atol(_pDec);
        if (_hchp != ba)
          {  /* New value for _hchp */
          _hchp = ba;
          SetBits(_DNFR, (1<<CLy_hchp));
          }
        break;

      case CLy_hchc:
        ba = atol(_pDec);
        if (_hchc != ba)
          {  /* New value for _hchc */
          _hchc = ba;
          SetBits(_DNFR, (1<<CLy_hchc));
          }
        break;

      case CLy_ptec:
        /*  Format PTEC :
         *    HP..    HC..
         *    0123    0123
         *  Just compare the 2 first chars */
        i = C_HPleines;      /* By default HP */
        if (strncmp_P(_pDec, PLy_HC, 2) == 0)
          { /* Tarif HC */
          i = C_HCreuses;
          }
        if (_ptec != i)
          {  /* PTEC has changed */
          _ptec = i;
          SetBits(_DNFR, (1<<CLy_ptec));  /* New value for _ptec */
          }
        break;
      #endif  /* LKY_HPHC */

      #ifdef LKY_IMono
      case CLy_iinst:
        i = (uint8_t) atoi(_pDec);
        if (_iinst != i)
          {  /* New value for _iinst */
          _iinst = i;
          SetBits(_DNFR, (1<<CLy_iinst));
          }
        break;
      #endif

      #ifdef LKY_ITri
      case CLy_iinst1:
      case CLy_iinst2:
      case CLy_iinst3:
        i = (uint8_t) atoi(_pDec);
        j = _GId - CLy_iinst1;
        if (_iinst[j] != i)
          {  /* New value for _iinst[] */
          _iinst[j] = i;
          SetBits(_DNFR, (1<<_GId));
          }
      #endif

      default:
        break;
      }
    }

  /* 2nd part, second action : group identification */
  if (_FR & bLy_GId)
    {
    ResetBits(_FR, bLy_GId);   /* Clear requesting flag */
    _pDec = strtok(_pDec, CLy_Sep);

    if (strcmp_P(_pDec, PLy_papp) == 0)
      {
      Run = false;
      _GId = CLy_papp;
      }

    #ifdef LKY_Base
    if (Run && (strcmp_P(_pDec, PLy_base) == 0))
      {
      Run = false;
      _GId = CLy_base;
      }
    #endif

    #ifdef LKY_HPHC
    if (Run && (strcmp_P(_pDec, PLy_hchp) == 0))
      {
      Run = false;
      _GId = CLy_hchp;
      }
    if (Run && (strcmp_P(_pDec, PLy_hchc) == 0))
      {
      Run = false;
      _GId = CLy_hchc;
      }
    if (Run && (strcmp_P(_pDec, PLy_ptec) == 0))
      {
      Run = false;
      _GId = CLy_ptec;
      }
    #endif

    #if (defined(LKY_IMono) || defined (LKY_ITri))
    if (Run && (strncmp_P(_pDec, PLy_iinst, 4) == 0))
      {   /*  Format :
           *    IINSTx  x = 1, 2, 3 or '\0' for single phase
           *    012345
           */
      switch (*(_pDec+5))
        {
        case '2':   /* Phase 2 */
          _GId = C_Phase_2 + CLy_iinst;
          break;

        case '3':   /* Phase 3 */
          _GId = C_Phase_3 + CLy_iinst;
          break;

        default:   /* Phase 1 or single phase */
          _GId = C_Phase_1 + CLy_iinst;
          break;
        }    /* End switch */

        Run = false;
        }
    #endif

    if (!Run)
      {
      SetBits(_FR, bLy_Dec);   /* Next = decode */
      }
    }

  /* 3rd part, first action : check cks */
  if (_FR & bLy_Cks)
    {
    ResetBits(_FR, bLy_Cks);   /* Clear requesting flag */
    cks = 0;
    if (_iCks >= CLy_MinLg)
      {   /* Message is long enough */
      for (i = 0; i < _iCks - 1; i++)
        {
        cks += *(_pDec + i);
        }
      cks = (cks & 0x3f) + Car_SP;

      #ifdef LINKYDEBUG
      Serial << _pDec << endl;
      #endif

      if (cks == *(_pDec + _iCks))
        {  /* Cks is correct */
        *(_pDec + _iCks-1) = '\0';
                       /* Terminate the string just before the Cks */
        SetBits(_FR, bLy_GId);  /* Next step, group identification */

        #ifdef LINKYDEBUG
        }
        else
        {
        i = *(_pDec + _iCks);
        Serial << F("Error Cks ") << cks << F(" - ") << i << endl;
        #endif
        }   /* Else, Cks error, do nothing */

      }     /* Message too short, do nothing */
    }

  /* 4th part, receiver processing */
  while (_LKY.available())
    {  /* At least 1 char has been received */
    c = _LKY.read() & 0x7f;   /* Read char, exclude parity */

    if (_FR & bLy_Rec)
      {  /* On going reception */
      if (c == '\r')
        {   /* Received end of group char */
        ResetBits(_FR, bLy_Rec);   /* Receiving complete */
        SetBits(_FR, bLy_Cks);     /* Next check Cks */
        _iCks = _iRec-1;           /* Index of Cks in the message */
        *(_pRec + _iRec) = '\0';   /* Terminate the string */

        /* Swap reception and decode buffers */
        if (_FR & bLy_RxB)
          {  /* Receiving in B, Decode in A, swap */
          ResetBits(_FR, bLy_RxB);
          _pRec = _BfA;       /* --> Receive in A */
          _pDec = _BfB;       /* --> Decode in B */
          }
          else
          {  /* Receiving in A, Decode in B, swap */
          SetBits(_FR, bLy_RxB);
          _pRec = _BfB;     /* --> Receive in B */
          _pDec = _BfA;     /* --> Decode in A */
          }

        }  /* End reception complete */
        else
        {  /* Other character */
        *(_pRec+_iRec) = c;   /* Store received character */
        _iRec += 1;
        if (_iRec >= CLy_BfSz-1)
          {  /* Buffer overrun */
          ResetBits(_FR, bLy_Rec); /* Stop reception and do nothing */
          }
        }  /* End other character than '\r' */
      }    /* End on-going reception */
      else
      {    /* Reception not yet started */
      if (c == '\n')
        {   /* Received start of group char */
        _iRec = 0;
        SetBits(_FR, bLy_Rec);   /* Start reception */
        }
      }
    }  /* End while */
  }

bool LinkyHistTIC::pappIsNew()
  {
  bool Res = false;

  if(_DNFR & (1<<CLy_papp))
    {
    Res = true;
    ResetBits(_DNFR, (1<<CLy_papp));
    }
  return Res;
  }

uint16_t LinkyHistTIC::papp()
  {
  return _papp;
  }

#ifdef LKY_Base
bool LinkyHistTIC::baseIsNew()
  {
  bool Res = false;

  if(_DNFR & (1<<CLy_base))
    {
    Res = true;
    ResetBits(_DNFR, (1<<CLy_base));
    }
  return Res;
  }

uint32_t LinkyHistTIC::base()
  {
  return _base;
  }
#endif  /* LKY_Base */

#ifdef LKY_HPHC
bool LinkyHistTIC::hchpIsNew()
  {
  bool Res = false;

  if(_DNFR & (1<<CLy_hchp))
    {
    Res = true;
    ResetBits(_DNFR, (1<<CLy_hchp));
    }
  return Res;
  }
uint32_t LinkyHistTIC::hchp()
  {
  return _hchp;
  }

bool LinkyHistTIC::hchcIsNew()
  {
  bool Res = false;

  if(_DNFR & (1<<CLy_hchc))
    {
    Res = true;
    ResetBits(_DNFR, (1<<CLy_hchc));
    }
  return Res;
  }
uint32_t LinkyHistTIC::hchc()
  {
  return _hchc;
  }

bool LinkyHistTIC::ptecIsNew()
  {
  bool Res = false;

  if(_DNFR & (1<<CLy_ptec))
    {
    Res = true;
    ResetBits(_DNFR, (1<<CLy_ptec));
    }
  return Res;
  }
uint8_t LinkyHistTIC::ptec()
  {
  return _ptec;
  }
#endif  /* LKY_HPHC */

#ifdef LKY_IMono
bool LinkyHistTIC::iinstIsNew()
  {
  bool Res = false;

  if(_DNFR & (1<<CLy_iinst))
    {
    Res = true;
    ResetBits(_DNFR, (1<<CLy_iinst));
    }
  return Res;
  }

uint8_t LinkyHistTIC::iinst()
  {
  return _iinst;
  }
#endif  /* LKY_IMono */

#ifdef LKY_ITri
bool LinkyHistTIC::iinstIsNew(uint8_t Ph)
  {
  bool Res = false;

  if(_DNFR & (1<<(CLy_iinst1 + Ph)))
    {
    Res = true;
    ResetBits(_DNFR, (1<<(CLy_iinst1 + Ph)));
    }
  return Res;
  }

uint8_t LinkyHistTIC::iinst(uint8_t Ph)
  {
  return _iinst[Ph];
  }
#endif  /* LKY_ITri */


/***********************************************************************
               Fin d'objet recepteur TIC Linky historique
***********************************************************************/

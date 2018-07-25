/PROG  RTC_MOVE
/ATTR
OWNER		= MNEDITOR;
COMMENT		= "Run-time";
PROG_SIZE	= 768;
CREATE		= DATE 18-07-18  TIME 23:55:34;
MODIFIED	= DATE 18-07-19  TIME 02:01:16;
FILE_NAME	= ;
VERSION		= 0;
LINE_COUNT	= 43;
MEMORY_SIZE	= 1236;
PROTECT		= READ_WRITE;
TCD:  STACK_SIZE	= 0,
      TASK_PRIORITY	= 50,
      TIME_SLICE	= 0,
      BUSY_LAMP_OFF	= 0,
      ABORT_REQUEST	= 0,
      PAUSE_REQUEST	= 0;
DEFAULT_GROUP	= 1,*,*,*,*;
CONTROL_CODE	= 00000000 00000000;
/MN
   1:  !Init ;
   2:  F[1]=(OFF) ;
   3:  F[2]=(OFF) ;
   4:   ;
   5:  PAYLOAD[1] ;
   6:  UFRAME_NUM=1 ;
   7:  UTOOL_NUM=1 ;
   8:  OVERRIDE=100% ;
   9:   ;
  10:  !Start server ;
  11:  RUN RTC_SERVER ;
  12:   ;
  13:  LBL[10] ;
  14:  !Select mov type ;
  15:  IF (F[3]) THEN ;
  16:   ;
  17:  !Ready for next point ;
  18:  F[1]=(ON) ;
  19:   ;
  20:  !Wait for server cmd ;
  21:  WAIT (F[2])    ;
  22:  LOCK PREG ;
  23:L PR[1] R[1]mm/sec FINE    ;
  24:  UNLOCK PREG ;
  25:  !Moving done ;
  26:  F[2]=(OFF) ;
  27:  JMP LBL[10] ;
  28:   ;
  29:  ELSE ;
  30:   ;
  31:  !Ready for next point ;
  32:  F[1]=(ON) ;
  33:   ;
  34:  !Wait for server cmd ;
  35:  WAIT (F[2])    ;
  36:  LOCK PREG ;
  37:J PR[1] R[1]% FINE    ;
  38:  UNLOCK PREG ;
  39:  !Moving done ;
  40:  F[2]=(OFF) ;
  41:  JMP LBL[10] ;
  42:   ;
  43:  ENDIF ;
/POS
/END

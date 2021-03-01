#define nx_struct struct
#define nx_union union
#define dbg(mode, format, ...) ((void)0)
#define dbg_clear(mode, format, ...) ((void)0)
#define dbg_active(mode) 0
# 150 "/usr/lib/gcc/msp430/4.6.3/include/stddef.h" 3
typedef long int ptrdiff_t;
#line 212
typedef unsigned int size_t;
#line 324
typedef int wchar_t;
# 8 "/usr/local/lib/ncc/deputy_nodeputy.h"
struct __nesc_attr_nonnull {
#line 8
  int dummy;
}  ;
#line 9
struct __nesc_attr_bnd {
#line 9
  void *lo, *hi;
}  ;
#line 10
struct __nesc_attr_bnd_nok {
#line 10
  void *lo, *hi;
}  ;
#line 11
struct __nesc_attr_count {
#line 11
  int n;
}  ;
#line 12
struct __nesc_attr_count_nok {
#line 12
  int n;
}  ;
#line 13
struct __nesc_attr_one {
#line 13
  int dummy;
}  ;
#line 14
struct __nesc_attr_one_nok {
#line 14
  int dummy;
}  ;
#line 15
struct __nesc_attr_dmemset {
#line 15
  int a1, a2, a3;
}  ;
#line 16
struct __nesc_attr_dmemcpy {
#line 16
  int a1, a2, a3;
}  ;
#line 17
struct __nesc_attr_nts {
#line 17
  int dummy;
}  ;
# 38 "/usr/lib/gcc/msp430/4.6.3/../../../../msp430/include/stdint.h" 3
typedef signed char int8_t;
typedef int int16_t;
typedef long int int32_t;
__extension__ 
#line 41
typedef long long int int64_t;

typedef unsigned char uint8_t;
typedef unsigned int uint16_t;
typedef unsigned long int uint32_t;
__extension__ 
#line 46
typedef unsigned long long int uint64_t;





typedef signed char int_least8_t;
typedef int int_least16_t;
typedef long int int_least32_t;
__extension__ 
#line 55
typedef long long int int_least64_t;


typedef unsigned char uint_least8_t;
typedef unsigned int uint_least16_t;
typedef unsigned long int uint_least32_t;
__extension__ 
#line 61
typedef unsigned long long int uint_least64_t;





typedef signed char int_fast8_t;
typedef int int_fast16_t;
typedef long int int_fast32_t;
__extension__ 
#line 70
typedef long long int int_fast64_t;


typedef unsigned char uint_fast8_t;
typedef unsigned int uint_fast16_t;
typedef unsigned long int uint_fast32_t;
__extension__ 
#line 76
typedef unsigned long long int uint_fast64_t;









typedef int16_t intptr_t;
typedef uint16_t uintptr_t;





__extension__ 
#line 93
typedef long long int intmax_t;
__extension__ 
#line 94
typedef unsigned long long int uintmax_t;
# 281 "/usr/local/lib/ncc/nesc_nx.h"
static __inline uint8_t __nesc_ntoh_uint8(const void * source)  ;




static __inline uint8_t __nesc_hton_uint8(void * target, uint8_t value)  ;





static __inline uint8_t __nesc_ntoh_leuint8(const void * source)  ;




static __inline uint8_t __nesc_hton_leuint8(void * target, uint8_t value)  ;





static __inline int8_t __nesc_ntoh_int8(const void * source)  ;
#line 303
static __inline int8_t __nesc_hton_int8(void * target, int8_t value)  ;






static __inline uint16_t __nesc_ntoh_uint16(const void * source)  ;




static __inline uint16_t __nesc_hton_uint16(void * target, uint16_t value)  ;






static __inline uint16_t __nesc_ntoh_leuint16(const void * source)  ;




static __inline uint16_t __nesc_hton_leuint16(void * target, uint16_t value)  ;
#line 340
static __inline uint32_t __nesc_ntoh_uint32(const void * source)  ;






static __inline uint32_t __nesc_hton_uint32(void * target, uint32_t value)  ;
#line 431
typedef struct { unsigned char nxdata[1]; } __attribute__((packed)) nx_int8_t;typedef int8_t __nesc_nxbase_nx_int8_t  ;
typedef struct { unsigned char nxdata[2]; } __attribute__((packed)) nx_int16_t;typedef int16_t __nesc_nxbase_nx_int16_t  ;
typedef struct { unsigned char nxdata[4]; } __attribute__((packed)) nx_int32_t;typedef int32_t __nesc_nxbase_nx_int32_t  ;
typedef struct { unsigned char nxdata[8]; } __attribute__((packed)) nx_int64_t;typedef int64_t __nesc_nxbase_nx_int64_t  ;
typedef struct { unsigned char nxdata[1]; } __attribute__((packed)) nx_uint8_t;typedef uint8_t __nesc_nxbase_nx_uint8_t  ;
typedef struct { unsigned char nxdata[2]; } __attribute__((packed)) nx_uint16_t;typedef uint16_t __nesc_nxbase_nx_uint16_t  ;
typedef struct { unsigned char nxdata[4]; } __attribute__((packed)) nx_uint32_t;typedef uint32_t __nesc_nxbase_nx_uint32_t  ;
typedef struct { unsigned char nxdata[8]; } __attribute__((packed)) nx_uint64_t;typedef uint64_t __nesc_nxbase_nx_uint64_t  ;


typedef struct { unsigned char nxdata[1]; } __attribute__((packed)) nxle_int8_t;typedef int8_t __nesc_nxbase_nxle_int8_t  ;
typedef struct { unsigned char nxdata[2]; } __attribute__((packed)) nxle_int16_t;typedef int16_t __nesc_nxbase_nxle_int16_t  ;
typedef struct { unsigned char nxdata[4]; } __attribute__((packed)) nxle_int32_t;typedef int32_t __nesc_nxbase_nxle_int32_t  ;
typedef struct { unsigned char nxdata[8]; } __attribute__((packed)) nxle_int64_t;typedef int64_t __nesc_nxbase_nxle_int64_t  ;
typedef struct { unsigned char nxdata[1]; } __attribute__((packed)) nxle_uint8_t;typedef uint8_t __nesc_nxbase_nxle_uint8_t  ;
typedef struct { unsigned char nxdata[2]; } __attribute__((packed)) nxle_uint16_t;typedef uint16_t __nesc_nxbase_nxle_uint16_t  ;
typedef struct { unsigned char nxdata[4]; } __attribute__((packed)) nxle_uint32_t;typedef uint32_t __nesc_nxbase_nxle_uint32_t  ;
typedef struct { unsigned char nxdata[8]; } __attribute__((packed)) nxle_uint64_t;typedef uint64_t __nesc_nxbase_nxle_uint64_t  ;
# 48 "/usr/lib/gcc/msp430/4.6.3/../../../../msp430/include/sys/types.h" 3
typedef unsigned char u_char;
typedef unsigned short u_short;
typedef unsigned int u_int;
typedef unsigned long u_long;
typedef unsigned short ushort;
typedef unsigned int uint;

typedef uint8_t u_int8_t;
typedef uint16_t u_int16_t;
typedef uint32_t u_int32_t;
typedef uint64_t u_int64_t;

typedef u_int64_t u_quad_t;
typedef int64_t quad_t;
typedef quad_t *qaddr_t;

typedef char *caddr_t;
typedef const char *c_caddr_t;
typedef volatile char *v_caddr_t;
typedef u_int32_t fixpt_t;
typedef u_int32_t gid_t;
typedef u_int32_t in_addr_t;
typedef u_int16_t in_port_t;
typedef u_int32_t ino_t;
typedef long key_t;
typedef u_int16_t mode_t;
typedef u_int16_t nlink_t;
typedef quad_t rlim_t;
typedef int32_t segsz_t;
typedef int32_t swblk_t;
typedef int32_t ufs_daddr_t;
typedef int32_t ufs_time_t;
typedef u_int32_t uid_t;
# 42 "/usr/lib/gcc/msp430/4.6.3/../../../../msp430/include/string.h" 3
extern void *memcpy(void *arg_0x7f7fee0af110, const void *arg_0x7f7fee0af410, size_t arg_0x7f7fee0af6d0);

extern void *memset(void *arg_0x7f7fee0ac020, int arg_0x7f7fee0ac2a0, size_t arg_0x7f7fee0ac560);
#line 65
extern void *memset(void *arg_0x7f7fee094060, int arg_0x7f7fee0942e0, size_t arg_0x7f7fee0945a0);
# 62 "/usr/lib/gcc/msp430/4.6.3/../../../../msp430/include/stdlib.h" 3
#line 59
typedef struct __nesc_unnamed4242 {
  int quot;
  int rem;
} div_t;






#line 66
typedef struct __nesc_unnamed4243 {
  long int quot;
  long int rem;
} ldiv_t;
# 122 "/usr/lib/gcc/msp430/4.6.3/../../../../msp430/include/sys/config.h" 3
typedef long int __int32_t;
typedef unsigned long int __uint32_t;
# 12 "/usr/lib/gcc/msp430/4.6.3/../../../../msp430/include/sys/_types.h" 3
typedef long _off_t;
typedef long _ssize_t;
# 19 "/usr/lib/gcc/msp430/4.6.3/../../../../msp430/include/sys/reent.h" 3
typedef unsigned long __ULong;
#line 31
struct _glue {

  struct _glue *_next;
  int _niobs;
  struct __sFILE *_iobs;
};

struct _Bigint {

  struct _Bigint *_next;
  int _k, _maxwds, _sign, _wds;
  __ULong _x[1];
};


struct __tm {

  int __tm_sec;
  int __tm_min;
  int __tm_hour;
  int __tm_mday;
  int __tm_mon;
  int __tm_year;
  int __tm_wday;
  int __tm_yday;
  int __tm_isdst;
};







struct _atexit {
  struct _atexit *_next;
  int _ind;
  void (*_fns[32])(void );
};








struct __sbuf {
  unsigned char *_base;
  int _size;
};






typedef long _fpos_t;
#line 116
struct __sFILE {
  unsigned char *_p;
  int _r;
  int _w;
  short _flags;
  short _file;
  struct __sbuf _bf;
  int _lbfsize;


  void *_cookie;

  int (*_read)(void *_cookie, char *_buf, int _n);
  int (*_write)(void *_cookie, const char *_buf, int _n);

  _fpos_t (*_seek)(void *_cookie, _fpos_t _offset, int _whence);
  int (*_close)(void *_cookie);


  struct __sbuf _ub;
  unsigned char *_up;
  int _ur;


  unsigned char _ubuf[3];
  unsigned char _nbuf[1];


  struct __sbuf _lb;


  int _blksize;
  int _offset;

  struct _reent *_data;
};
#line 174
struct _rand48 {
  unsigned short _seed[3];
  unsigned short _mult[3];
  unsigned short _add;
};









struct _reent {


  int _errno;




  struct __sFILE *_stdin, *_stdout, *_stderr;

  int _inc;
  char _emergency[25];

  int _current_category;
  const char *_current_locale;

  int __sdidinit;

  void (*__cleanup)(struct _reent *arg_0x7f7fee057170);


  struct _Bigint *_result;
  int _result_k;
  struct _Bigint *_p5s;
  struct _Bigint **_freelist;


  int _cvtlen;
  char *_cvtbuf;

  union __nesc_unnamed4244 {

    struct __nesc_unnamed4245 {

      unsigned int _unused_rand;
      char *_strtok_last;
      char _asctime_buf[26];
      struct __tm _localtime_buf;
      int _gamma_signgam;
      __extension__ unsigned long long _rand_next;
      struct _rand48 _r48;
    } _reent;



    struct __nesc_unnamed4246 {


      unsigned char *_nextf[30];
      unsigned int _nmalloc[30];
    } _unused;
  } _new;


  struct _atexit *_atexit;
  struct _atexit _atexit0;


  void (**_sig_func)(int arg_0x7f7fee052480);




  struct _glue __sglue;
  struct __sFILE __sf[3];
};
#line 273
struct _reent;
# 18 "/usr/lib/gcc/msp430/4.6.3/../../../../msp430/include/math.h" 3
union __dmath {

  __uint32_t i[2];
  double d;
};




union __dmath;
#line 212
struct exception {


  int type;
  char *name;
  double arg1;
  double arg2;
  double retval;
  int err;
};
#line 265
enum __fdlibm_version {

  __fdlibm_ieee = -1, 
  __fdlibm_svid, 
  __fdlibm_xopen, 
  __fdlibm_posix
};




enum __fdlibm_version;
# 25 "/home/csilab/tinyos/tos/system/tos.h"
typedef uint8_t bool;
enum __nesc_unnamed4247 {
#line 26
  FALSE = 0, TRUE = 1
};
typedef nx_int8_t nx_bool;
uint16_t TOS_NODE_ID = 1;






struct __nesc_attr_atmostonce {
};
#line 37
struct __nesc_attr_atleastonce {
};
#line 38
struct __nesc_attr_exactlyonce {
};
# 51 "/home/csilab/tinyos/tos/types/TinyError.h"
enum __nesc_unnamed4248 {
  SUCCESS = 0, 
  FAIL = 1, 
  ESIZE = 2, 
  ECANCEL = 3, 
  EOFF = 4, 
  EBUSY = 5, 
  EINVAL = 6, 
  ERETRY = 7, 
  ERESERVE = 8, 
  EALREADY = 9, 
  ENOMEM = 10, 
  ENOACK = 11, 
  ELAST = 11
};

typedef uint8_t error_t  ;

static inline error_t ecombine(error_t r1, error_t r2)  ;
# 47 "/usr/lib/gcc/msp430/4.6.3/../../../../msp430/include/intrinsics.h" 3
void __nop(void );



void __dint(void );



void __eint(void );


unsigned int __read_status_register(void );


typedef unsigned int __istate_t;
# 164 "/usr/lib/gcc/msp430/4.6.3/../../../../msp430/include/msp430f1611.h" 3
extern volatile unsigned char ME1 __asm ("__""ME1");
#line 183
extern volatile unsigned char ME2 __asm ("__""ME2");
#line 195
extern volatile unsigned int WDTCTL __asm ("__""WDTCTL");
#line 267
extern volatile unsigned char P1OUT __asm ("__""P1OUT");

extern volatile unsigned char P1DIR __asm ("__""P1DIR");

extern volatile unsigned char P1IFG __asm ("__""P1IFG");

extern volatile unsigned char P1IES __asm ("__""P1IES");

extern volatile unsigned char P1IE __asm ("__""P1IE");

extern volatile unsigned char P1SEL __asm ("__""P1SEL");




extern volatile unsigned char P2OUT __asm ("__""P2OUT");

extern volatile unsigned char P2DIR __asm ("__""P2DIR");

extern volatile unsigned char P2IFG __asm ("__""P2IFG");



extern volatile unsigned char P2IE __asm ("__""P2IE");

extern volatile unsigned char P2SEL __asm ("__""P2SEL");










extern volatile unsigned char P3OUT __asm ("__""P3OUT");

extern volatile unsigned char P3DIR __asm ("__""P3DIR");

extern volatile unsigned char P3SEL __asm ("__""P3SEL");




extern volatile unsigned char P4OUT __asm ("__""P4OUT");

extern volatile unsigned char P4DIR __asm ("__""P4DIR");

extern volatile unsigned char P4SEL __asm ("__""P4SEL");










extern volatile unsigned char P5OUT __asm ("__""P5OUT");

extern volatile unsigned char P5DIR __asm ("__""P5DIR");

extern volatile unsigned char P5SEL __asm ("__""P5SEL");




extern volatile unsigned char P6OUT __asm ("__""P6OUT");

extern volatile unsigned char P6DIR __asm ("__""P6DIR");

extern volatile unsigned char P6SEL __asm ("__""P6SEL");
#line 382
extern volatile unsigned char U0CTL __asm ("__""U0CTL");

extern volatile unsigned char U0TCTL __asm ("__""U0TCTL");



extern volatile unsigned char U0MCTL __asm ("__""U0MCTL");

extern volatile unsigned char U0BR0 __asm ("__""U0BR0");

extern volatile unsigned char U0BR1 __asm ("__""U0BR1");

extern const volatile unsigned char U0RXBUF __asm ("__""U0RXBUF");
#line 439
extern volatile unsigned char U1CTL __asm ("__""U1CTL");

extern volatile unsigned char U1TCTL __asm ("__""U1TCTL");



extern volatile unsigned char U1MCTL __asm ("__""U1MCTL");

extern volatile unsigned char U1BR0 __asm ("__""U1BR0");

extern volatile unsigned char U1BR1 __asm ("__""U1BR1");

extern const volatile unsigned char U1RXBUF __asm ("__""U1RXBUF");
#line 595
extern volatile unsigned int TACTL __asm ("__""TACTL");

extern volatile unsigned int TACCTL0 __asm ("__""TACCTL0");

extern volatile unsigned int TACCTL1 __asm ("__""TACCTL1");

extern volatile unsigned int TACCTL2 __asm ("__""TACCTL2");

extern volatile unsigned int TAR __asm ("__""TAR");
#line 720
extern volatile unsigned int TBCCTL0 __asm ("__""TBCCTL0");
#line 734
extern volatile unsigned int TBR __asm ("__""TBR");

extern volatile unsigned int TBCCR0 __asm ("__""TBCCR0");
#line 849
extern volatile unsigned char DCOCTL __asm ("__""DCOCTL");

extern volatile unsigned char BCSCTL1 __asm ("__""BCSCTL1");

extern volatile unsigned char BCSCTL2 __asm ("__""BCSCTL2");
#line 1021
extern volatile unsigned int ADC12CTL0 __asm ("__""ADC12CTL0");

extern volatile unsigned int ADC12CTL1 __asm ("__""ADC12CTL1");
# 343 "/home/csilab/tinyos/tos/chips/msp430/msp430hardware.h"
static volatile uint8_t U0CTLnr __asm ("0x0070");
static volatile uint8_t I2CTCTLnr __asm ("0x0071");
static volatile uint8_t I2CDCTLnr __asm ("0x0072");
#line 378
typedef uint8_t mcu_power_t  ;
static inline mcu_power_t mcombine(mcu_power_t m1, mcu_power_t m2)  ;


enum __nesc_unnamed4249 {
  MSP430_POWER_ACTIVE = 0, 
  MSP430_POWER_LPM0 = 1, 
  MSP430_POWER_LPM1 = 2, 
  MSP430_POWER_LPM2 = 3, 
  MSP430_POWER_LPM3 = 4, 
  MSP430_POWER_LPM4 = 5
};

static inline void __nesc_disable_interrupt(void )  ;





static inline void __nesc_enable_interrupt(void )  ;




typedef bool __nesc_atomic_t;
__nesc_atomic_t __nesc_atomic_start(void );
void __nesc_atomic_end(__nesc_atomic_t reenable_interrupts);






__nesc_atomic_t __nesc_atomic_start(void )   ;







void __nesc_atomic_end(__nesc_atomic_t reenable_interrupts)   ;
#line 433
typedef struct { unsigned char nxdata[4]; } __attribute__((packed)) nx_float;typedef float __nesc_nxbase_nx_float  ;
#line 448
enum __nesc_unnamed4250 {
  MSP430_PORT_RESISTOR_INVALID, 
  MSP430_PORT_RESISTOR_OFF, 
  MSP430_PORT_RESISTOR_PULLDOWN, 
  MSP430_PORT_RESISTOR_PULLUP
};
# 8 "/home/csilab/tinyos/tos/platforms/telosb/hardware.h"
enum __nesc_unnamed4251 {
  TOS_SLEEP_NONE = MSP430_POWER_ACTIVE
};
#line 36
static inline void TOSH_SET_SIMO0_PIN()  ;
#line 36
static inline void TOSH_CLR_SIMO0_PIN()  ;
#line 36
static inline void TOSH_MAKE_SIMO0_OUTPUT()  ;
static inline void TOSH_SET_UCLK0_PIN()  ;
#line 37
static inline void TOSH_CLR_UCLK0_PIN()  ;
#line 37
static inline void TOSH_MAKE_UCLK0_OUTPUT()  ;
#line 79
enum __nesc_unnamed4252 {

  TOSH_HUMIDITY_ADDR = 5, 
  TOSH_HUMIDTEMP_ADDR = 3, 
  TOSH_HUMIDITY_RESET = 0x1E
};



static inline void TOSH_SET_FLASH_CS_PIN()  ;
#line 88
static inline void TOSH_CLR_FLASH_CS_PIN()  ;
#line 88
static inline void TOSH_MAKE_FLASH_CS_OUTPUT()  ;
static inline void TOSH_SET_FLASH_HOLD_PIN()  ;
#line 89
static inline void TOSH_MAKE_FLASH_HOLD_OUTPUT()  ;
# 6 "/home/csilab/tinyos/tos/types/AM.h"
typedef nx_uint8_t nx_am_id_t;
typedef nx_uint8_t nx_am_group_t;
typedef nx_uint16_t nx_am_addr_t;

typedef uint8_t am_id_t;
typedef uint8_t am_group_t;
typedef uint16_t am_addr_t;

enum __nesc_unnamed4253 {
  AM_BROADCAST_ADDR = 0xffff
};









enum __nesc_unnamed4254 {
  TOS_AM_GROUP = 0x22, 
  TOS_AM_ADDRESS = 1
};
# 83 "/home/csilab/tinyos/tos/lib/serial/Serial.h"
typedef uint8_t uart_id_t;



enum __nesc_unnamed4255 {
  HDLC_FLAG_BYTE = 0x7e, 
  HDLC_CTLESC_BYTE = 0x7d
};



enum __nesc_unnamed4256 {
  TOS_SERIAL_ACTIVE_MESSAGE_ID = 0, 
  TOS_SERIAL_CC1000_ID = 1, 
  TOS_SERIAL_802_15_4_ID = 2, 
  TOS_SERIAL_UNKNOWN_ID = 255
};


enum __nesc_unnamed4257 {
  SERIAL_PROTO_ACK = 67, 
  SERIAL_PROTO_PACKET_ACK = 68, 
  SERIAL_PROTO_PACKET_NOACK = 69, 
  SERIAL_PROTO_PACKET_UNKNOWN = 255
};
#line 121
#line 109
typedef struct radio_stats {
  uint8_t version;
  uint8_t flags;
  uint8_t reserved;
  uint8_t platform;
  uint16_t MTU;
  uint16_t radio_crc_fail;
  uint16_t radio_queue_drops;
  uint16_t serial_crc_fail;
  uint16_t serial_tx_fail;
  uint16_t serial_short_packets;
  uint16_t serial_proto_drops;
} radio_stats_t;







#line 123
typedef nx_struct serial_header {
  nx_am_addr_t dest;
  nx_am_addr_t src;
  nx_uint8_t length;
  nx_am_group_t group;
  nx_am_id_t type;
} __attribute__((packed)) serial_header_t;




#line 131
typedef nx_struct serial_packet {
  serial_header_t header;
  nx_uint8_t data[];
} __attribute__((packed)) serial_packet_t;



#line 136
typedef nx_struct serial_metadata {
  nx_uint8_t ack;
} __attribute__((packed)) serial_metadata_t;
# 39 "/home/csilab/tinyos/tos/chips/cc2420/CC2420.h"
typedef uint8_t cc2420_status_t;
#line 93
#line 87
typedef nx_struct security_header_t {
  unsigned char __nesc_filler0[1];


  nx_uint32_t frameCounter;
  nx_uint8_t keyID[1];
} __attribute__((packed)) security_header_t;
#line 113
#line 95
typedef nx_struct cc2420_header_t {
  nxle_uint8_t length;
  nxle_uint16_t fcf;
  nxle_uint8_t dsn;
  nxle_uint16_t destpan;
  nxle_uint16_t dest;
  nxle_uint16_t src;







  nxle_uint8_t network;


  nxle_uint8_t type;
} __attribute__((packed)) cc2420_header_t;





#line 118
typedef nx_struct cc2420_footer_t {
} __attribute__((packed)) cc2420_footer_t;
#line 143
#line 128
typedef nx_struct cc2420_metadata_t {
  nx_uint8_t rssi;
  nx_uint8_t lqi;
  nx_uint8_t tx_power;
  nx_bool crc;
  nx_bool ack;
  nx_bool timesync;
  nx_uint32_t timestamp;
  nx_uint16_t rxInterval;
} __attribute__((packed)) 





cc2420_metadata_t;





#line 146
typedef nx_struct cc2420_packet_t {
  cc2420_header_t packet;
  nx_uint8_t data[];
} __attribute__((packed)) cc2420_packet_t;
#line 179
enum __nesc_unnamed4258 {

  MAC_HEADER_SIZE = sizeof(cc2420_header_t ) - 1, 

  MAC_FOOTER_SIZE = sizeof(uint16_t ), 

  MAC_PACKET_SIZE = MAC_HEADER_SIZE + 28 + MAC_FOOTER_SIZE, 

  CC2420_SIZE = MAC_HEADER_SIZE + MAC_FOOTER_SIZE
};

enum cc2420_enums {
  CC2420_TIME_ACK_TURNAROUND = 7, 
  CC2420_TIME_VREN = 20, 
  CC2420_TIME_SYMBOL = 2, 
  CC2420_BACKOFF_PERIOD = 20 / CC2420_TIME_SYMBOL, 
  CC2420_MIN_BACKOFF = 20 / CC2420_TIME_SYMBOL, 
  CC2420_ACK_WAIT_DELAY = 256
};

enum cc2420_status_enums {
  CC2420_STATUS_RSSI_VALID = 1 << 1, 
  CC2420_STATUS_LOCK = 1 << 2, 
  CC2420_STATUS_TX_ACTIVE = 1 << 3, 
  CC2420_STATUS_ENC_BUSY = 1 << 4, 
  CC2420_STATUS_TX_UNDERFLOW = 1 << 5, 
  CC2420_STATUS_XOSC16M_STABLE = 1 << 6
};

enum cc2420_config_reg_enums {
  CC2420_SNOP = 0x00, 
  CC2420_SXOSCON = 0x01, 
  CC2420_STXCAL = 0x02, 
  CC2420_SRXON = 0x03, 
  CC2420_STXON = 0x04, 
  CC2420_STXONCCA = 0x05, 
  CC2420_SRFOFF = 0x06, 
  CC2420_SXOSCOFF = 0x07, 
  CC2420_SFLUSHRX = 0x08, 
  CC2420_SFLUSHTX = 0x09, 
  CC2420_SACK = 0x0a, 
  CC2420_SACKPEND = 0x0b, 
  CC2420_SRXDEC = 0x0c, 
  CC2420_STXENC = 0x0d, 
  CC2420_SAES = 0x0e, 
  CC2420_MAIN = 0x10, 
  CC2420_MDMCTRL0 = 0x11, 
  CC2420_MDMCTRL1 = 0x12, 
  CC2420_RSSI = 0x13, 
  CC2420_SYNCWORD = 0x14, 
  CC2420_TXCTRL = 0x15, 
  CC2420_RXCTRL0 = 0x16, 
  CC2420_RXCTRL1 = 0x17, 
  CC2420_FSCTRL = 0x18, 
  CC2420_SECCTRL0 = 0x19, 
  CC2420_SECCTRL1 = 0x1a, 
  CC2420_BATTMON = 0x1b, 
  CC2420_IOCFG0 = 0x1c, 
  CC2420_IOCFG1 = 0x1d, 
  CC2420_MANFIDL = 0x1e, 
  CC2420_MANFIDH = 0x1f, 
  CC2420_FSMTC = 0x20, 
  CC2420_MANAND = 0x21, 
  CC2420_MANOR = 0x22, 
  CC2420_AGCCTRL = 0x23, 
  CC2420_AGCTST0 = 0x24, 
  CC2420_AGCTST1 = 0x25, 
  CC2420_AGCTST2 = 0x26, 
  CC2420_FSTST0 = 0x27, 
  CC2420_FSTST1 = 0x28, 
  CC2420_FSTST2 = 0x29, 
  CC2420_FSTST3 = 0x2a, 
  CC2420_RXBPFTST = 0x2b, 
  CC2420_FMSTATE = 0x2c, 
  CC2420_ADCTST = 0x2d, 
  CC2420_DACTST = 0x2e, 
  CC2420_TOPTST = 0x2f, 
  CC2420_TXFIFO = 0x3e, 
  CC2420_RXFIFO = 0x3f
};

enum cc2420_ram_addr_enums {
  CC2420_RAM_TXFIFO = 0x000, 
  CC2420_RAM_RXFIFO = 0x080, 
  CC2420_RAM_KEY0 = 0x100, 
  CC2420_RAM_RXNONCE = 0x110, 
  CC2420_RAM_SABUF = 0x120, 
  CC2420_RAM_KEY1 = 0x130, 
  CC2420_RAM_TXNONCE = 0x140, 
  CC2420_RAM_CBCSTATE = 0x150, 
  CC2420_RAM_IEEEADR = 0x160, 
  CC2420_RAM_PANID = 0x168, 
  CC2420_RAM_SHORTADR = 0x16a
};

enum cc2420_nonce_enums {
  CC2420_NONCE_BLOCK_COUNTER = 0, 
  CC2420_NONCE_KEY_SEQ_COUNTER = 2, 
  CC2420_NONCE_FRAME_COUNTER = 3, 
  CC2420_NONCE_SOURCE_ADDRESS = 7, 
  CC2420_NONCE_FLAGS = 15
};

enum cc2420_main_enums {
  CC2420_MAIN_RESETn = 15, 
  CC2420_MAIN_ENC_RESETn = 14, 
  CC2420_MAIN_DEMOD_RESETn = 13, 
  CC2420_MAIN_MOD_RESETn = 12, 
  CC2420_MAIN_FS_RESETn = 11, 
  CC2420_MAIN_XOSC16M_BYPASS = 0
};

enum cc2420_mdmctrl0_enums {
  CC2420_MDMCTRL0_RESERVED_FRAME_MODE = 13, 
  CC2420_MDMCTRL0_PAN_COORDINATOR = 12, 
  CC2420_MDMCTRL0_ADR_DECODE = 11, 
  CC2420_MDMCTRL0_CCA_HYST = 8, 
  CC2420_MDMCTRL0_CCA_MOD = 6, 
  CC2420_MDMCTRL0_AUTOCRC = 5, 
  CC2420_MDMCTRL0_AUTOACK = 4, 
  CC2420_MDMCTRL0_PREAMBLE_LENGTH = 0
};

enum cc2420_mdmctrl1_enums {
  CC2420_MDMCTRL1_CORR_THR = 6, 
  CC2420_MDMCTRL1_DEMOD_AVG_MODE = 5, 
  CC2420_MDMCTRL1_MODULATION_MODE = 4, 
  CC2420_MDMCTRL1_TX_MODE = 2, 
  CC2420_MDMCTRL1_RX_MODE = 0
};

enum cc2420_rssi_enums {
  CC2420_RSSI_CCA_THR = 8, 
  CC2420_RSSI_RSSI_VAL = 0
};

enum cc2420_syncword_enums {
  CC2420_SYNCWORD_SYNCWORD = 0
};

enum cc2420_txctrl_enums {
  CC2420_TXCTRL_TXMIXBUF_CUR = 14, 
  CC2420_TXCTRL_TX_TURNAROUND = 13, 
  CC2420_TXCTRL_TXMIX_CAP_ARRAY = 11, 
  CC2420_TXCTRL_TXMIX_CURRENT = 9, 
  CC2420_TXCTRL_PA_CURRENT = 6, 
  CC2420_TXCTRL_RESERVED = 5, 
  CC2420_TXCTRL_PA_LEVEL = 0
};

enum cc2420_rxctrl0_enums {
  CC2420_RXCTRL0_RXMIXBUF_CUR = 12, 
  CC2420_RXCTRL0_HIGH_LNA_GAIN = 10, 
  CC2420_RXCTRL0_MED_LNA_GAIN = 8, 
  CC2420_RXCTRL0_LOW_LNA_GAIN = 6, 
  CC2420_RXCTRL0_HIGH_LNA_CURRENT = 4, 
  CC2420_RXCTRL0_MED_LNA_CURRENT = 2, 
  CC2420_RXCTRL0_LOW_LNA_CURRENT = 0
};

enum cc2420_rxctrl1_enums {
  CC2420_RXCTRL1_RXBPF_LOCUR = 13, 
  CC2420_RXCTRL1_RXBPF_MIDCUR = 12, 
  CC2420_RXCTRL1_LOW_LOWGAIN = 11, 
  CC2420_RXCTRL1_MED_LOWGAIN = 10, 
  CC2420_RXCTRL1_HIGH_HGM = 9, 
  CC2420_RXCTRL1_MED_HGM = 8, 
  CC2420_RXCTRL1_LNA_CAP_ARRAY = 6, 
  CC2420_RXCTRL1_RXMIX_TAIL = 4, 
  CC2420_RXCTRL1_RXMIX_VCM = 2, 
  CC2420_RXCTRL1_RXMIX_CURRENT = 0
};

enum cc2420_rsctrl_enums {
  CC2420_FSCTRL_LOCK_THR = 14, 
  CC2420_FSCTRL_CAL_DONE = 13, 
  CC2420_FSCTRL_CAL_RUNNING = 12, 
  CC2420_FSCTRL_LOCK_LENGTH = 11, 
  CC2420_FSCTRL_LOCK_STATUS = 10, 
  CC2420_FSCTRL_FREQ = 0
};

enum cc2420_secctrl0_enums {
  CC2420_SECCTRL0_RXFIFO_PROTECTION = 9, 
  CC2420_SECCTRL0_SEC_CBC_HEAD = 8, 
  CC2420_SECCTRL0_SEC_SAKEYSEL = 7, 
  CC2420_SECCTRL0_SEC_TXKEYSEL = 6, 
  CC2420_SECCTRL0_SEC_RXKEYSEL = 5, 
  CC2420_SECCTRL0_SEC_M = 2, 
  CC2420_SECCTRL0_SEC_MODE = 0
};

enum cc2420_secctrl1_enums {
  CC2420_SECCTRL1_SEC_TXL = 8, 
  CC2420_SECCTRL1_SEC_RXL = 0
};

enum cc2420_battmon_enums {
  CC2420_BATTMON_BATT_OK = 6, 
  CC2420_BATTMON_BATTMON_EN = 5, 
  CC2420_BATTMON_BATTMON_VOLTAGE = 0
};

enum cc2420_iocfg0_enums {
  CC2420_IOCFG0_BCN_ACCEPT = 11, 
  CC2420_IOCFG0_FIFO_POLARITY = 10, 
  CC2420_IOCFG0_FIFOP_POLARITY = 9, 
  CC2420_IOCFG0_SFD_POLARITY = 8, 
  CC2420_IOCFG0_CCA_POLARITY = 7, 
  CC2420_IOCFG0_FIFOP_THR = 0
};

enum cc2420_iocfg1_enums {
  CC2420_IOCFG1_HSSD_SRC = 10, 
  CC2420_IOCFG1_SFDMUX = 5, 
  CC2420_IOCFG1_CCAMUX = 0
};

enum cc2420_manfidl_enums {
  CC2420_MANFIDL_PARTNUM = 12, 
  CC2420_MANFIDL_MANFID = 0
};

enum cc2420_manfidh_enums {
  CC2420_MANFIDH_VERSION = 12, 
  CC2420_MANFIDH_PARTNUM = 0
};

enum cc2420_fsmtc_enums {
  CC2420_FSMTC_TC_RXCHAIN2RX = 13, 
  CC2420_FSMTC_TC_SWITCH2TX = 10, 
  CC2420_FSMTC_TC_PAON2TX = 6, 
  CC2420_FSMTC_TC_TXEND2SWITCH = 3, 
  CC2420_FSMTC_TC_TXEND2PAOFF = 0
};

enum cc2420_sfdmux_enums {
  CC2420_SFDMUX_SFD = 0, 
  CC2420_SFDMUX_XOSC16M_STABLE = 24
};

enum cc2420_security_enums {
  CC2420_NO_SEC = 0, 
  CC2420_CBC_MAC = 1, 
  CC2420_CTR = 2, 
  CC2420_CCM = 3, 
  NO_SEC = 0, 
  CBC_MAC_4 = 1, 
  CBC_MAC_8 = 2, 
  CBC_MAC_16 = 3, 
  CTR = 4, 
  CCM_4 = 5, 
  CCM_8 = 6, 
  CCM_16 = 7
};


enum __nesc_unnamed4259 {

  CC2420_INVALID_TIMESTAMP = 0x80000000L
};
# 59 "/home/csilab/tinyos/tos/platforms/telosa/platform_message.h"
#line 56
typedef union message_header {
  cc2420_header_t cc2420;
  serial_header_t serial;
} message_header_t;



#line 61
typedef union TOSRadioFooter {
  cc2420_footer_t cc2420;
} message_footer_t;




#line 65
typedef union TOSRadioMetadata {
  cc2420_metadata_t cc2420;
  serial_metadata_t serial;
} message_metadata_t;
# 19 "/home/csilab/tinyos/tos/types/message.h"
#line 14
typedef nx_struct message_t {
  nx_uint8_t header[sizeof(message_header_t )];
  nx_uint8_t data[28];
  nx_uint8_t footer[sizeof(message_footer_t )];
  nx_uint8_t metadata[sizeof(message_metadata_t )];
} __attribute__((packed)) message_t;
# 43 "/home/csilab/tinyos/tos/types/Leds.h"
enum __nesc_unnamed4260 {
  LEDS_LED0 = 1 << 0, 
  LEDS_LED1 = 1 << 1, 
  LEDS_LED2 = 1 << 2, 
  LEDS_LED3 = 1 << 3, 
  LEDS_LED4 = 1 << 4, 
  LEDS_LED5 = 1 << 5, 
  LEDS_LED6 = 1 << 6, 
  LEDS_LED7 = 1 << 7
};
# 91 "/home/csilab/tinyos/tos/system/crc.h"
static uint16_t crcByte(uint16_t crc, uint8_t b);
# 56 "/home/csilab/tinyos/tos/chips/msp430/usart/msp430usart.h"
#line 48
typedef enum __nesc_unnamed4261 {

  USART_NONE = 0, 
  USART_UART = 1, 
  USART_UART_TX = 2, 
  USART_UART_RX = 3, 
  USART_SPI = 4, 
  USART_I2C = 5
} msp430_usartmode_t;










#line 58
typedef struct __nesc_unnamed4262 {
  unsigned int swrst : 1;
  unsigned int mm : 1;
  unsigned int sync : 1;
  unsigned int listen : 1;
  unsigned int clen : 1;
  unsigned int spb : 1;
  unsigned int pev : 1;
  unsigned int pena : 1;
} __attribute((packed))  msp430_uctl_t;









#line 69
typedef struct __nesc_unnamed4263 {
  unsigned int txept : 1;
  unsigned int stc : 1;
  unsigned int txwake : 1;
  unsigned int urxse : 1;
  unsigned int ssel : 2;
  unsigned int ckpl : 1;
  unsigned int ckph : 1;
} __attribute((packed))  msp430_utctl_t;










#line 79
typedef struct __nesc_unnamed4264 {
  unsigned int rxerr : 1;
  unsigned int rxwake : 1;
  unsigned int urxwie : 1;
  unsigned int urxeie : 1;
  unsigned int brk : 1;
  unsigned int oe : 1;
  unsigned int pe : 1;
  unsigned int fe : 1;
} __attribute((packed))  msp430_urctl_t;
#line 116
#line 99
typedef struct __nesc_unnamed4265 {
  unsigned int ubr : 16;

  unsigned int  : 1;
  unsigned int mm : 1;
  unsigned int  : 1;
  unsigned int listen : 1;
  unsigned int clen : 1;
  unsigned int  : 3;

  unsigned int  : 1;
  unsigned int stc : 1;
  unsigned int  : 2;
  unsigned int ssel : 2;
  unsigned int ckpl : 1;
  unsigned int ckph : 1;
  unsigned int  : 0;
} msp430_spi_config_t;





#line 118
typedef struct __nesc_unnamed4266 {
  uint16_t ubr;
  uint8_t uctl;
  uint8_t utctl;
} msp430_spi_registers_t;




#line 124
typedef union __nesc_unnamed4267 {
  msp430_spi_config_t spiConfig;
  msp430_spi_registers_t spiRegisters;
} msp430_spi_union_config_t;

msp430_spi_union_config_t msp430_spi_default_config = { 
{ 
.ubr = 0x0002, 
.ssel = 0x02, 
.clen = 1, 
.listen = 0, 
.mm = 1, 
.ckph = 1, 
.ckpl = 0, 
.stc = 1 } };
#line 169
#line 150
typedef enum __nesc_unnamed4268 {

  UBR_32KHZ_1200 = 0x001B, UMCTL_32KHZ_1200 = 0x94, 
  UBR_32KHZ_1800 = 0x0012, UMCTL_32KHZ_1800 = 0x84, 
  UBR_32KHZ_2400 = 0x000D, UMCTL_32KHZ_2400 = 0x6D, 
  UBR_32KHZ_4800 = 0x0006, UMCTL_32KHZ_4800 = 0x77, 
  UBR_32KHZ_9600 = 0x0003, UMCTL_32KHZ_9600 = 0x29, 

  UBR_1MHZ_1200 = 0x0369, UMCTL_1MHZ_1200 = 0x7B, 
  UBR_1MHZ_1800 = 0x0246, UMCTL_1MHZ_1800 = 0x55, 
  UBR_1MHZ_2400 = 0x01B4, UMCTL_1MHZ_2400 = 0xDF, 
  UBR_1MHZ_4800 = 0x00DA, UMCTL_1MHZ_4800 = 0xAA, 
  UBR_1MHZ_9600 = 0x006D, UMCTL_1MHZ_9600 = 0x44, 
  UBR_1MHZ_19200 = 0x0036, UMCTL_1MHZ_19200 = 0xB5, 
  UBR_1MHZ_38400 = 0x001B, UMCTL_1MHZ_38400 = 0x94, 
  UBR_1MHZ_57600 = 0x0012, UMCTL_1MHZ_57600 = 0x84, 
  UBR_1MHZ_76800 = 0x000D, UMCTL_1MHZ_76800 = 0x6D, 
  UBR_1MHZ_115200 = 0x0009, UMCTL_1MHZ_115200 = 0x10, 
  UBR_1MHZ_230400 = 0x0004, UMCTL_1MHZ_230400 = 0x55
} msp430_uart_rate_t;
#line 200
#line 171
typedef struct __nesc_unnamed4269 {
  unsigned int ubr : 16;

  unsigned int umctl : 8;

  unsigned int  : 1;
  unsigned int mm : 1;
  unsigned int  : 1;
  unsigned int listen : 1;
  unsigned int clen : 1;
  unsigned int spb : 1;
  unsigned int pev : 1;
  unsigned int pena : 1;
  unsigned int  : 0;

  unsigned int  : 3;
  unsigned int urxse : 1;
  unsigned int ssel : 2;
  unsigned int ckpl : 1;
  unsigned int  : 1;

  unsigned int  : 2;
  unsigned int urxwie : 1;
  unsigned int urxeie : 1;
  unsigned int  : 4;
  unsigned int  : 0;

  unsigned int utxe : 1;
  unsigned int urxe : 1;
} msp430_uart_config_t;








#line 202
typedef struct __nesc_unnamed4270 {
  uint16_t ubr;
  uint8_t umctl;
  uint8_t uctl;
  uint8_t utctl;
  uint8_t urctl;
  uint8_t ume;
} msp430_uart_registers_t;




#line 211
typedef union __nesc_unnamed4271 {
  msp430_uart_config_t uartConfig;
  msp430_uart_registers_t uartRegisters;
} msp430_uart_union_config_t;

msp430_uart_union_config_t msp430_uart_default_config = { 
{ 
.utxe = 1, 
.urxe = 1, 
.ubr = UBR_1MHZ_57600, 
.umctl = UMCTL_1MHZ_57600, 
.ssel = 0x02, 
.pena = 0, 
.pev = 0, 
.spb = 0, 
.clen = 1, 
.listen = 0, 
.mm = 0, 
.ckpl = 0, 
.urxse = 0, 
.urxeie = 1, 
.urxwie = 0, 
.utxe = 1, 
.urxe = 1 } };
#line 248
#line 240
typedef struct __nesc_unnamed4272 {
  unsigned int i2cstt : 1;
  unsigned int i2cstp : 1;
  unsigned int i2cstb : 1;
  unsigned int i2cctrx : 1;
  unsigned int i2cssel : 2;
  unsigned int i2ccrm : 1;
  unsigned int i2cword : 1;
} __attribute((packed))  msp430_i2ctctl_t;
#line 276
#line 253
typedef struct __nesc_unnamed4273 {
  unsigned int  : 1;
  unsigned int mst : 1;
  unsigned int  : 1;
  unsigned int listen : 1;
  unsigned int xa : 1;
  unsigned int  : 1;
  unsigned int txdmaen : 1;
  unsigned int rxdmaen : 1;

  unsigned int  : 4;
  unsigned int i2cssel : 2;
  unsigned int i2crm : 1;
  unsigned int i2cword : 1;

  unsigned int i2cpsc : 8;

  unsigned int i2csclh : 8;

  unsigned int i2cscll : 8;

  unsigned int i2coa : 10;
  unsigned int  : 6;
} msp430_i2c_config_t;








#line 278
typedef struct __nesc_unnamed4274 {
  uint8_t uctl;
  uint8_t i2ctctl;
  uint8_t i2cpsc;
  uint8_t i2csclh;
  uint8_t i2cscll;
  uint16_t i2coa;
} msp430_i2c_registers_t;




#line 287
typedef union __nesc_unnamed4275 {
  msp430_i2c_config_t i2cConfig;
  msp430_i2c_registers_t i2cRegisters;
} msp430_i2c_union_config_t;
#line 309
typedef uint8_t uart_speed_t;
typedef uint8_t uart_parity_t;
typedef uint8_t uart_duplex_t;

enum __nesc_unnamed4276 {
  TOS_UART_1200 = 0, 
  TOS_UART_1800 = 1, 
  TOS_UART_2400 = 2, 
  TOS_UART_4800 = 3, 
  TOS_UART_9600 = 4, 
  TOS_UART_19200 = 5, 
  TOS_UART_38400 = 6, 
  TOS_UART_57600 = 7, 
  TOS_UART_76800 = 8, 
  TOS_UART_115200 = 9, 
  TOS_UART_230400 = 10
};

enum __nesc_unnamed4277 {
  TOS_UART_OFF, 
  TOS_UART_RONLY, 
  TOS_UART_TONLY, 
  TOS_UART_DUPLEX
};

enum __nesc_unnamed4278 {
  TOS_UART_PARITY_NONE, 
  TOS_UART_PARITY_EVEN, 
  TOS_UART_PARITY_ODD
};
# 41 "/home/csilab/tinyos/tos/lib/timer/Timer.h"
typedef struct __nesc_unnamed4279 {
#line 41
  int notUsed;
} 
#line 41
TSecond;
typedef struct __nesc_unnamed4280 {
#line 42
  int notUsed;
} 
#line 42
TMilli;
typedef struct __nesc_unnamed4281 {
#line 43
  int notUsed;
} 
#line 43
T32khz;
typedef struct __nesc_unnamed4282 {
#line 44
  int notUsed;
} 
#line 44
TMicro;
# 39 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430Timer.h"
enum __nesc_unnamed4283 {
  MSP430TIMER_CM_NONE = 0, 
  MSP430TIMER_CM_RISING = 1, 
  MSP430TIMER_CM_FALLING = 2, 
  MSP430TIMER_CM_BOTH = 3, 

  MSP430TIMER_STOP_MODE = 0, 
  MSP430TIMER_UP_MODE = 1, 
  MSP430TIMER_CONTINUOUS_MODE = 2, 
  MSP430TIMER_UPDOWN_MODE = 3, 

  MSP430TIMER_TACLK = 0, 
  MSP430TIMER_TBCLK = 0, 
  MSP430TIMER_ACLK = 1, 
  MSP430TIMER_SMCLK = 2, 
  MSP430TIMER_INCLK = 3, 

  MSP430TIMER_CLOCKDIV_1 = 0, 
  MSP430TIMER_CLOCKDIV_2 = 1, 
  MSP430TIMER_CLOCKDIV_4 = 2, 
  MSP430TIMER_CLOCKDIV_8 = 3
};
#line 75
#line 62
typedef struct __nesc_unnamed4284 {

  int ccifg : 1;
  int cov : 1;
  int out : 1;
  int cci : 1;
  int ccie : 1;
  int outmod : 3;
  int cap : 1;
  int clld : 2;
  int scs : 1;
  int ccis : 2;
  int cm : 2;
} msp430_compare_control_t;
#line 87
#line 77
typedef struct __nesc_unnamed4285 {

  int taifg : 1;
  int taie : 1;
  int taclr : 1;
  int _unused0 : 1;
  int mc : 2;
  int id : 2;
  int tassel : 2;
  int _unused1 : 6;
} msp430_timer_a_control_t;
#line 102
#line 89
typedef struct __nesc_unnamed4286 {

  int tbifg : 1;
  int tbie : 1;
  int tbclr : 1;
  int _unused0 : 1;
  int mc : 2;
  int id : 2;
  int tbssel : 2;
  int _unused1 : 1;
  int cntl : 2;
  int tbclgrp : 2;
  int _unused2 : 1;
} msp430_timer_b_control_t;
# 33 "/home/csilab/tinyos/tos/types/Resource.h"
typedef uint8_t resource_client_id_t;
# 40 "/usr/lib/gcc/msp430/4.6.3/include/stdarg.h" 3
typedef __builtin_va_list __gnuc_va_list;
#line 102
typedef __gnuc_va_list va_list;
# 52 "/usr/lib/gcc/msp430/4.6.3/../../../../msp430/include/stdio.h" 3
int __attribute((format(printf, 1, 2))) printf(const char *string, ...);






int putchar(int c);
# 72 "/home/csilab/tinyos/tos/lib/printf/printf.h"
int printfflush();






#line 77
typedef nx_struct printf_msg {
  nx_uint8_t buffer[28];
} __attribute__((packed)) printf_msg_t;

enum __nesc_unnamed4287 {
  AM_PRINTF_MSG = 100
};
# 40 "/home/csilab/tinyos/tos/types/IeeeEui64.h"
enum __nesc_unnamed4288 {
#line 40
  IEEE_EUI64_LENGTH = 8
};


#line 42
typedef struct ieee_eui64 {
  uint8_t data[IEEE_EUI64_LENGTH];
} ieee_eui64_t;
# 47 "/home/csilab/tinyos/tos/types/Ieee154.h"
typedef uint16_t ieee154_panid_t;
typedef uint16_t ieee154_saddr_t;
typedef ieee_eui64_t ieee154_laddr_t;







#line 51
typedef struct __nesc_unnamed4289 {
  uint8_t ieee_mode : 2;
  union __nesc_unnamed4290 {
    ieee154_saddr_t saddr;
    ieee154_laddr_t laddr;
  } ieee_addr;
} ieee154_addr_t;



enum __nesc_unnamed4291 {
  IEEE154_BROADCAST_ADDR = 0xffff, 
  IEEE154_LINK_MTU = 127
};

struct ieee154_frame_addr {
  ieee154_addr_t ieee_src;
  ieee154_addr_t ieee_dst;
  ieee154_panid_t ieee_dstpan;
};

enum __nesc_unnamed4292 {
  IEEE154_MIN_HDR_SZ = 6
};
#line 86
enum ieee154_fcf_enums {
  IEEE154_FCF_FRAME_TYPE = 0, 
  IEEE154_FCF_SECURITY_ENABLED = 3, 
  IEEE154_FCF_FRAME_PENDING = 4, 
  IEEE154_FCF_ACK_REQ = 5, 
  IEEE154_FCF_INTRAPAN = 6, 
  IEEE154_FCF_DEST_ADDR_MODE = 10, 
  IEEE154_FCF_SRC_ADDR_MODE = 14
};

enum ieee154_fcf_type_enums {
  IEEE154_TYPE_BEACON = 0, 
  IEEE154_TYPE_DATA = 1, 
  IEEE154_TYPE_ACK = 2, 
  IEEE154_TYPE_MAC_CMD = 3, 
  IEEE154_TYPE_MASK = 7
};

enum ieee154_fcf_addr_mode_enums {
  IEEE154_ADDR_NONE = 0, 
  IEEE154_ADDR_SHORT = 2, 
  IEEE154_ADDR_EXT = 3, 
  IEEE154_ADDR_MASK = 3
};
# 12 "/home/csilab/tinyos/tos/platforms/epic/chips/ds2411/DallasId48.h"
enum __nesc_unnamed4293 {
  DALLASID48_SERIAL_LENGTH = 6, 
  DALLASID48_DATA_LENGTH = 8
};








#line 17
typedef union dallasid48_serial_t {
  uint8_t data[DALLASID48_DATA_LENGTH];
  struct  {
    uint8_t family_code;
    uint8_t serial[DALLASID48_SERIAL_LENGTH];
    uint8_t crc;
  } ;
} dallasid48_serial_t;




static inline bool dallasid48checkCrc(const dallasid48_serial_t *id);
# 29 "/home/csilab/tinyos/tos/platforms/epic/chips/ds2411/PlatformIeeeEui64.h"
enum __nesc_unnamed4294 {
  IEEE_EUI64_COMPANY_ID_0 = 0x00, 
  IEEE_EUI64_COMPANY_ID_1 = 0x12, 
  IEEE_EUI64_COMPANY_ID_2 = 0x6d, 
  IEEE_EUI64_SERIAL_ID_0 = 'E', 
  IEEE_EUI64_SERIAL_ID_1 = 'P'
};
# 43 "/home/csilab/tinyos/tos/chips/cc2420/CC2420TimeSyncMessage.h"
typedef nx_uint32_t timesync_radio_t;





#line 45
typedef nx_struct timesync_footer_t {

  nx_am_id_t type;
  timesync_radio_t timestamp;
} __attribute__((packed)) timesync_footer_t;
# 38 "/home/csilab/upma/types/LocalTimeExtended.h"
#line 35
typedef nx_struct local_time16 {
  nx_uint16_t sticks;
  nx_uint32_t mticks;
} __attribute__((packed)) local_time16_t;




#line 40
typedef nx_struct local_time32 {
  nx_uint32_t sticks;
  nx_uint32_t mticks;
} __attribute__((packed)) local_time32_t;
# 4 "/home/csilab/upma/system/SenderDispatcher.h"
enum __nesc_unnamed4295 {
  TDMA_SLOT = 0, 
  CSMA_SLOT = 1, 
  BEACON_SLOT = 2, 
  NO_SLOT = 0xff
};
# 23 "/home/csilab/upma/lib/macs/pure-tdma/TestPureTdma.h"
#line 13
typedef nx_struct TdmaMsg {



  nx_uint16_t SequenceNo;
  nx_uint16_t Data;
  nx_uint8_t Schedule[20];
  nx_uint8_t Flowid;
} __attribute__((packed)) 

TdmaMsg_t;









#line 25
typedef nx_struct adaptivehart_msg {

  nx_uint8_t flowid;
  nx_uint8_t priority;
  nx_uint8_t maxtx;
  nx_uint16_t deadline;
  nx_uint16_t seq;
  nx_uint8_t is_changed;
} __attribute__((packed)) adaptivehart_msg_t;
# 31 "/home/csilab/upma/system/BeaconSlotP.nc"
enum __nesc_unnamed4296 {
  SIMPLE_TDMA_SYNC = 123
};
enum /*PlatformSerialC.UartC*/Msp430Uart1C__0____nesc_unnamed4297 {
  Msp430Uart1C__0__CLIENT_ID = 0U
};
typedef T32khz /*Msp430Uart1P.UartP*/Msp430UartP__0__Counter__precision_tag;
typedef uint16_t /*Msp430Uart1P.UartP*/Msp430UartP__0__Counter__size_type;
typedef T32khz /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__frequency_tag;
typedef /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__frequency_tag /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__precision_tag;
typedef uint16_t /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__size_type;
enum /*PlatformSerialC.UartC.UsartC*/Msp430Usart1C__0____nesc_unnamed4298 {
  Msp430Usart1C__0__CLIENT_ID = 0U
};
enum SerialAMQueueP____nesc_unnamed4299 {
  SerialAMQueueP__NUM_CLIENTS = 1U
};
typedef uint8_t /*PrintfC.QueueC*/QueueC__0__queue_t;
typedef /*PrintfC.QueueC*/QueueC__0__queue_t /*PrintfC.QueueC*/QueueC__0__Queue__t;
typedef uint8_t PrintfP__Queue__t;
enum CC2420ActiveMessageC____nesc_unnamed4300 {
  CC2420ActiveMessageC__CC2420_AM_SEND_ID = 0U
};
typedef T32khz CC2420ControlP__StartupTimer__precision_tag;
typedef uint32_t CC2420ControlP__StartupTimer__size_type;
typedef uint16_t CC2420ControlP__ReadRssi__val_t;
enum /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Timer*/Msp430Timer32khzC__0____nesc_unnamed4301 {
  Msp430Timer32khzC__0__ALARM_ID = 0U
};
typedef T32khz /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__frequency_tag;
typedef /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__frequency_tag /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Alarm__precision_tag;
typedef uint16_t /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Alarm__size_type;
typedef T32khz /*Counter32khz32C.Transform*/TransformCounterC__0__to_precision_tag;
typedef uint32_t /*Counter32khz32C.Transform*/TransformCounterC__0__to_size_type;
typedef T32khz /*Counter32khz32C.Transform*/TransformCounterC__0__from_precision_tag;
typedef uint16_t /*Counter32khz32C.Transform*/TransformCounterC__0__from_size_type;
typedef uint16_t /*Counter32khz32C.Transform*/TransformCounterC__0__upper_count_type;
typedef /*Counter32khz32C.Transform*/TransformCounterC__0__from_precision_tag /*Counter32khz32C.Transform*/TransformCounterC__0__CounterFrom__precision_tag;
typedef /*Counter32khz32C.Transform*/TransformCounterC__0__from_size_type /*Counter32khz32C.Transform*/TransformCounterC__0__CounterFrom__size_type;
typedef /*Counter32khz32C.Transform*/TransformCounterC__0__to_precision_tag /*Counter32khz32C.Transform*/TransformCounterC__0__Counter__precision_tag;
typedef /*Counter32khz32C.Transform*/TransformCounterC__0__to_size_type /*Counter32khz32C.Transform*/TransformCounterC__0__Counter__size_type;
typedef T32khz /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__to_precision_tag;
typedef uint32_t /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__to_size_type;
typedef T32khz /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__from_precision_tag;
typedef uint16_t /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__from_size_type;
typedef /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__to_precision_tag /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Alarm__precision_tag;
typedef /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__to_size_type /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Alarm__size_type;
typedef /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__from_precision_tag /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__AlarmFrom__precision_tag;
typedef /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__from_size_type /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__AlarmFrom__size_type;
typedef /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__to_precision_tag /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Counter__precision_tag;
typedef /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__to_size_type /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Counter__size_type;
enum /*CC2420ControlC.Spi*/CC2420SpiC__0____nesc_unnamed4302 {
  CC2420SpiC__0__CLIENT_ID = 0U
};
enum /*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430Spi0C__0____nesc_unnamed4303 {
  Msp430Spi0C__0__CLIENT_ID = 0U
};
enum /*CC2420SpiWireC.HplCC2420SpiC.SpiC.UsartC*/Msp430Usart0C__0____nesc_unnamed4304 {
  Msp430Usart0C__0__CLIENT_ID = 0U
};
enum /*CC2420ControlC.SyncSpiC*/CC2420SpiC__1____nesc_unnamed4305 {
  CC2420SpiC__1__CLIENT_ID = 1U
};
enum /*CC2420ControlC.RssiResource*/CC2420SpiC__2____nesc_unnamed4306 {
  CC2420SpiC__2__CLIENT_ID = 2U
};
typedef TMicro OneWireMasterC__BusyWait__precision_tag;
typedef uint16_t OneWireMasterC__BusyWait__size_type;
typedef TMicro /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__precision_tag;
typedef uint16_t /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__size_type;
typedef /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__precision_tag /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__BusyWait__precision_tag;
typedef /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__size_type /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__BusyWait__size_type;
typedef /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__precision_tag /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__Counter__precision_tag;
typedef /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__size_type /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__Counter__size_type;
typedef TMicro /*Msp430CounterMicroC.Counter*/Msp430CounterC__1__frequency_tag;
typedef /*Msp430CounterMicroC.Counter*/Msp430CounterC__1__frequency_tag /*Msp430CounterMicroC.Counter*/Msp430CounterC__1__Counter__precision_tag;
typedef uint16_t /*Msp430CounterMicroC.Counter*/Msp430CounterC__1__Counter__size_type;
typedef T32khz CC2420TransmitP__PacketTimeStamp__precision_tag;
typedef uint32_t CC2420TransmitP__PacketTimeStamp__size_type;
typedef T32khz CC2420TransmitP__BackoffTimer__precision_tag;
typedef uint32_t CC2420TransmitP__BackoffTimer__size_type;
enum /*CC2420TransmitC.Spi*/CC2420SpiC__3____nesc_unnamed4307 {
  CC2420SpiC__3__CLIENT_ID = 3U
};
typedef T32khz CC2420ReceiveP__PacketTimeStamp__precision_tag;
typedef uint32_t CC2420ReceiveP__PacketTimeStamp__size_type;
typedef T32khz CC2420PacketP__PacketTimeStamp32khz__precision_tag;
typedef uint32_t CC2420PacketP__PacketTimeStamp32khz__size_type;
typedef T32khz CC2420PacketP__LocalTime32khz__precision_tag;
typedef TMilli CC2420PacketP__LocalTimeMilli__precision_tag;
typedef TMilli CC2420PacketP__PacketTimeStampMilli__precision_tag;
typedef uint32_t CC2420PacketP__PacketTimeStampMilli__size_type;
typedef T32khz /*CC2420PacketC.CounterToLocalTimeC*/CounterToLocalTimeC__0__precision_tag;
typedef /*CC2420PacketC.CounterToLocalTimeC*/CounterToLocalTimeC__0__precision_tag /*CC2420PacketC.CounterToLocalTimeC*/CounterToLocalTimeC__0__LocalTime__precision_tag;
typedef /*CC2420PacketC.CounterToLocalTimeC*/CounterToLocalTimeC__0__precision_tag /*CC2420PacketC.CounterToLocalTimeC*/CounterToLocalTimeC__0__Counter__precision_tag;
typedef uint32_t /*CC2420PacketC.CounterToLocalTimeC*/CounterToLocalTimeC__0__Counter__size_type;
enum /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Timer*/Msp430Timer32khzC__1____nesc_unnamed4308 {
  Msp430Timer32khzC__1__ALARM_ID = 1U
};
typedef T32khz /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__frequency_tag;
typedef /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__frequency_tag /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Alarm__precision_tag;
typedef uint16_t /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Alarm__size_type;
typedef TMilli /*CounterMilli32C.Transform*/TransformCounterC__1__to_precision_tag;
typedef uint32_t /*CounterMilli32C.Transform*/TransformCounterC__1__to_size_type;
typedef T32khz /*CounterMilli32C.Transform*/TransformCounterC__1__from_precision_tag;
typedef uint16_t /*CounterMilli32C.Transform*/TransformCounterC__1__from_size_type;
typedef uint32_t /*CounterMilli32C.Transform*/TransformCounterC__1__upper_count_type;
typedef /*CounterMilli32C.Transform*/TransformCounterC__1__from_precision_tag /*CounterMilli32C.Transform*/TransformCounterC__1__CounterFrom__precision_tag;
typedef /*CounterMilli32C.Transform*/TransformCounterC__1__from_size_type /*CounterMilli32C.Transform*/TransformCounterC__1__CounterFrom__size_type;
typedef /*CounterMilli32C.Transform*/TransformCounterC__1__to_precision_tag /*CounterMilli32C.Transform*/TransformCounterC__1__Counter__precision_tag;
typedef /*CounterMilli32C.Transform*/TransformCounterC__1__to_size_type /*CounterMilli32C.Transform*/TransformCounterC__1__Counter__size_type;
typedef TMilli /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__to_precision_tag;
typedef uint32_t /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__to_size_type;
typedef T32khz /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__from_precision_tag;
typedef uint16_t /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__from_size_type;
typedef /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__to_precision_tag /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__Alarm__precision_tag;
typedef /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__Alarm__size_type;
typedef /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__from_precision_tag /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__AlarmFrom__precision_tag;
typedef /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__from_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__AlarmFrom__size_type;
typedef /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__to_precision_tag /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__Counter__precision_tag;
typedef /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__Counter__size_type;
typedef TMilli /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__precision_tag;
typedef /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__precision_tag /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__precision_tag;
typedef uint32_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type;
typedef /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__precision_tag /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__precision_tag;
typedef TMilli /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__precision_tag;
typedef /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__precision_tag /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__precision_tag;
typedef /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__precision_tag /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__precision_tag;
typedef TMilli /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__1__precision_tag;
typedef /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__1__precision_tag /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__1__LocalTime__precision_tag;
typedef /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__1__precision_tag /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__1__Counter__precision_tag;
typedef uint32_t /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__1__Counter__size_type;
enum /*CC2420ReceiveC.Spi*/CC2420SpiC__4____nesc_unnamed4309 {
  CC2420SpiC__4__CLIENT_ID = 4U
};
typedef uint16_t RandomMlcgC__SeedInit__parameter;
enum CC2420TinyosNetworkC____nesc_unnamed4310 {
  CC2420TinyosNetworkC__TINYOS_N_NETWORKS = 0U
};
typedef T32khz PowerCycleP__Time__precision_tag;
typedef local_time16_t PowerCycleP__Time__local_time_t;
typedef T32khz /*LocalTime32khz16C.LocalTimeP*/LocalTime16P__0__counter_t;
typedef /*LocalTime32khz16C.LocalTimeP*/LocalTime16P__0__counter_t /*LocalTime32khz16C.LocalTimeP*/LocalTime16P__0__LocalTime__precision_tag;
typedef local_time16_t /*LocalTime32khz16C.LocalTimeP*/LocalTime16P__0__LocalTime__local_time_t;
typedef /*LocalTime32khz16C.LocalTimeP*/LocalTime16P__0__counter_t /*LocalTime32khz16C.LocalTimeP*/LocalTime16P__0__Counter__precision_tag;
typedef uint16_t /*LocalTime32khz16C.LocalTimeP*/LocalTime16P__0__Counter__size_type;
typedef adaptivehart_msg_t *PureTDMASchedulerP__HIforwardQ__t;
typedef adaptivehart_msg_t *PureTDMASchedulerP__LOforwardQ__t;
typedef adaptivehart_msg_t *PureTDMASchedulerP__RootQ__t;
typedef T32khz PureTDMASchedulerP__TxOffset_alarm__precision_tag;
typedef uint16_t PureTDMASchedulerP__TxOffset_alarm__size_type;
typedef T32khz /*TDMASlotSenderC.TDMASlotSender.GenericSlotSenderP*/GenericSlotSenderP__0__Alarm__precision_tag;
typedef uint16_t /*TDMASlotSenderC.TDMASlotSender.GenericSlotSenderP*/GenericSlotSenderP__0__Alarm__size_type;
enum /*TDMASlotSenderC.TDMASlotSender.Alarm32khz16C.Msp430Timer*/Msp430Timer32khzC__2____nesc_unnamed4311 {
  Msp430Timer32khzC__2__ALARM_ID = 2U
};
typedef T32khz /*TDMASlotSenderC.TDMASlotSender.Alarm32khz16C.Msp430Alarm*/Msp430AlarmC__2__frequency_tag;
typedef /*TDMASlotSenderC.TDMASlotSender.Alarm32khz16C.Msp430Alarm*/Msp430AlarmC__2__frequency_tag /*TDMASlotSenderC.TDMASlotSender.Alarm32khz16C.Msp430Alarm*/Msp430AlarmC__2__Alarm__precision_tag;
typedef uint16_t /*TDMASlotSenderC.TDMASlotSender.Alarm32khz16C.Msp430Alarm*/Msp430AlarmC__2__Alarm__size_type;
typedef T32khz GenericSlotterP__Alarm__precision_tag;
typedef uint32_t GenericSlotterP__Alarm__size_type;
enum /*GenericSlotterC.Alarm32khz32C.AlarmC.Msp430Timer*/Msp430Timer32khzC__3____nesc_unnamed4312 {
  Msp430Timer32khzC__3__ALARM_ID = 3U
};
typedef T32khz /*GenericSlotterC.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__3__frequency_tag;
typedef /*GenericSlotterC.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__3__frequency_tag /*GenericSlotterC.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__3__Alarm__precision_tag;
typedef uint16_t /*GenericSlotterC.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__3__Alarm__size_type;
typedef T32khz /*GenericSlotterC.Alarm32khz32C.Transform*/TransformAlarmC__2__to_precision_tag;
typedef uint32_t /*GenericSlotterC.Alarm32khz32C.Transform*/TransformAlarmC__2__to_size_type;
typedef T32khz /*GenericSlotterC.Alarm32khz32C.Transform*/TransformAlarmC__2__from_precision_tag;
typedef uint16_t /*GenericSlotterC.Alarm32khz32C.Transform*/TransformAlarmC__2__from_size_type;
typedef /*GenericSlotterC.Alarm32khz32C.Transform*/TransformAlarmC__2__to_precision_tag /*GenericSlotterC.Alarm32khz32C.Transform*/TransformAlarmC__2__Alarm__precision_tag;
typedef /*GenericSlotterC.Alarm32khz32C.Transform*/TransformAlarmC__2__to_size_type /*GenericSlotterC.Alarm32khz32C.Transform*/TransformAlarmC__2__Alarm__size_type;
typedef /*GenericSlotterC.Alarm32khz32C.Transform*/TransformAlarmC__2__from_precision_tag /*GenericSlotterC.Alarm32khz32C.Transform*/TransformAlarmC__2__AlarmFrom__precision_tag;
typedef /*GenericSlotterC.Alarm32khz32C.Transform*/TransformAlarmC__2__from_size_type /*GenericSlotterC.Alarm32khz32C.Transform*/TransformAlarmC__2__AlarmFrom__size_type;
typedef /*GenericSlotterC.Alarm32khz32C.Transform*/TransformAlarmC__2__to_precision_tag /*GenericSlotterC.Alarm32khz32C.Transform*/TransformAlarmC__2__Counter__precision_tag;
typedef /*GenericSlotterC.Alarm32khz32C.Transform*/TransformAlarmC__2__to_size_type /*GenericSlotterC.Alarm32khz32C.Transform*/TransformAlarmC__2__Counter__size_type;
typedef T32khz BeaconSlotP__SyncAlarm__precision_tag;
typedef uint32_t BeaconSlotP__SyncAlarm__size_type;
enum /*PureTDMASchedulerC.Alarm32khz32C.AlarmC.Msp430Timer*/Msp430Timer32khzC__4____nesc_unnamed4313 {
  Msp430Timer32khzC__4__ALARM_ID = 4U
};
typedef T32khz /*PureTDMASchedulerC.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__4__frequency_tag;
typedef /*PureTDMASchedulerC.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__4__frequency_tag /*PureTDMASchedulerC.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__4__Alarm__precision_tag;
typedef uint16_t /*PureTDMASchedulerC.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__4__Alarm__size_type;
typedef T32khz /*PureTDMASchedulerC.Alarm32khz32C.Transform*/TransformAlarmC__3__to_precision_tag;
typedef uint32_t /*PureTDMASchedulerC.Alarm32khz32C.Transform*/TransformAlarmC__3__to_size_type;
typedef T32khz /*PureTDMASchedulerC.Alarm32khz32C.Transform*/TransformAlarmC__3__from_precision_tag;
typedef uint16_t /*PureTDMASchedulerC.Alarm32khz32C.Transform*/TransformAlarmC__3__from_size_type;
typedef /*PureTDMASchedulerC.Alarm32khz32C.Transform*/TransformAlarmC__3__to_precision_tag /*PureTDMASchedulerC.Alarm32khz32C.Transform*/TransformAlarmC__3__Alarm__precision_tag;
typedef /*PureTDMASchedulerC.Alarm32khz32C.Transform*/TransformAlarmC__3__to_size_type /*PureTDMASchedulerC.Alarm32khz32C.Transform*/TransformAlarmC__3__Alarm__size_type;
typedef /*PureTDMASchedulerC.Alarm32khz32C.Transform*/TransformAlarmC__3__from_precision_tag /*PureTDMASchedulerC.Alarm32khz32C.Transform*/TransformAlarmC__3__AlarmFrom__precision_tag;
typedef /*PureTDMASchedulerC.Alarm32khz32C.Transform*/TransformAlarmC__3__from_size_type /*PureTDMASchedulerC.Alarm32khz32C.Transform*/TransformAlarmC__3__AlarmFrom__size_type;
typedef /*PureTDMASchedulerC.Alarm32khz32C.Transform*/TransformAlarmC__3__to_precision_tag /*PureTDMASchedulerC.Alarm32khz32C.Transform*/TransformAlarmC__3__Counter__precision_tag;
typedef /*PureTDMASchedulerC.Alarm32khz32C.Transform*/TransformAlarmC__3__to_size_type /*PureTDMASchedulerC.Alarm32khz32C.Transform*/TransformAlarmC__3__Counter__size_type;
typedef adaptivehart_msg_t */*PureTDMASchedulerC.HIQueueC*/QueueC__1__queue_t;
typedef /*PureTDMASchedulerC.HIQueueC*/QueueC__1__queue_t /*PureTDMASchedulerC.HIQueueC*/QueueC__1__Queue__t;
typedef adaptivehart_msg_t */*PureTDMASchedulerC.LOQueueC*/QueueC__2__queue_t;
typedef /*PureTDMASchedulerC.LOQueueC*/QueueC__2__queue_t /*PureTDMASchedulerC.LOQueueC*/QueueC__2__Queue__t;
typedef adaptivehart_msg_t */*PureTDMASchedulerC.RootQueueC*/QueueC__3__queue_t;
typedef /*PureTDMASchedulerC.RootQueueC*/QueueC__3__queue_t /*PureTDMASchedulerC.RootQueueC*/QueueC__3__Queue__t;
enum /*PureTDMASchedulerC.offset_alarm.Msp430Timer*/Msp430Timer32khzC__5____nesc_unnamed4314 {
  Msp430Timer32khzC__5__ALARM_ID = 5U
};
typedef T32khz /*PureTDMASchedulerC.offset_alarm.Msp430Alarm*/Msp430AlarmC__5__frequency_tag;
typedef /*PureTDMASchedulerC.offset_alarm.Msp430Alarm*/Msp430AlarmC__5__frequency_tag /*PureTDMASchedulerC.offset_alarm.Msp430Alarm*/Msp430AlarmC__5__Alarm__precision_tag;
typedef uint16_t /*PureTDMASchedulerC.offset_alarm.Msp430Alarm*/Msp430AlarmC__5__Alarm__size_type;
typedef TMilli /*TestPureTdmaC.SendingApp.App*/SendingC__0__SendTimer__precision_tag;
typedef TMilli /*TestPureTdmaC.SendingApp.App*/SendingC__0__BenchmarkTimer__precision_tag;
typedef T32khz /*TestPureTdmaC.SendingApp.App*/SendingC__0__Counter__precision_tag;
typedef uint32_t /*TestPureTdmaC.SendingApp.App*/SendingC__0__Counter__size_type;
typedef TMilli /*TestPureTdmaC.SendingApp.App*/SendingC__0__StartTimer__precision_tag;
typedef TMilli /*TestPureTdmaC.SendingApp.App*/SendingC__0__FlushTimer__precision_tag;
enum AMQueueP____nesc_unnamed4315 {
  AMQueueP__NUM_CLIENTS = 1U
};
# 100 "/home/csilab/tinyos/tos/interfaces/Send.nc"
static void /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__SubSend__sendDone(
#line 96
message_t * msg, 



error_t error);
# 78 "/home/csilab/tinyos/tos/interfaces/Receive.nc"
static 
#line 74
message_t * 



/*SerialActiveMessageC.AM*/SerialActiveMessageP__0__SubReceive__receive(
#line 71
message_t * msg, 
void * payload, 





uint8_t len);
# 80 "/home/csilab/tinyos/tos/interfaces/AMSend.nc"
static error_t /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMSend__send(
# 36 "/home/csilab/upma/lib/serial/SerialActiveMessageP.nc"
am_id_t arg_0x7f7fedab0c50, 
# 80 "/home/csilab/tinyos/tos/interfaces/AMSend.nc"
am_addr_t addr, 
#line 71
message_t * msg, 








uint8_t len);
# 78 "/home/csilab/tinyos/tos/interfaces/Packet.nc"
static uint8_t /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__Packet__payloadLength(
#line 74
message_t * msg);
#line 126
static 
#line 123
void * 


/*SerialActiveMessageC.AM*/SerialActiveMessageP__0__Packet__getPayload(
#line 121
message_t * msg, 




uint8_t len);
#line 106
static uint8_t /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__Packet__maxPayloadLength(void );
#line 94
static void /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__Packet__setPayloadLength(
#line 90
message_t * msg, 



uint8_t len);
# 78 "/home/csilab/tinyos/tos/interfaces/Receive.nc"
static 
#line 74
message_t * 



/*SerialActiveMessageC.AM*/SerialActiveMessageP__0__Receive__default__receive(
# 37 "/home/csilab/upma/lib/serial/SerialActiveMessageP.nc"
am_id_t arg_0x7f7fedaa2e30, 
# 71 "/home/csilab/tinyos/tos/interfaces/Receive.nc"
message_t * msg, 
void * payload, 





uint8_t len);
# 67 "/home/csilab/upma/interfaces/AMPacket.nc"
static am_addr_t /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMPacket__destination(
#line 63
message_t * amsg);
#line 92
static void /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMPacket__setDestination(
#line 88
message_t * amsg, 



am_addr_t addr);
#line 136
static am_id_t /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMPacket__type(
#line 132
message_t * amsg);
#line 151
static void /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMPacket__setType(
#line 147
message_t * amsg, 



am_id_t t);
# 113 "/home/csilab/tinyos/tos/interfaces/SplitControl.nc"
static void SerialP__SplitControl__default__startDone(error_t error);
#line 138
static void SerialP__SplitControl__default__stopDone(error_t error);
# 75 "/home/csilab/tinyos/tos/interfaces/TaskBasic.nc"
static void SerialP__stopDoneTask__runTask(void );
#line 75
static void SerialP__RunTx__runTask(void );
# 62 "/home/csilab/tinyos/tos/interfaces/Init.nc"
static error_t SerialP__Init__init(void );
# 54 "/home/csilab/tinyos/tos/lib/serial/SerialFlush.nc"
static void SerialP__SerialFlush__flushDone(void );
#line 49
static void SerialP__SerialFlush__default__flush(void );
# 75 "/home/csilab/tinyos/tos/interfaces/TaskBasic.nc"
static void SerialP__startDoneTask__runTask(void );
# 94 "/home/csilab/tinyos/tos/lib/serial/SerialFrameComm.nc"
static void SerialP__SerialFrameComm__dataReceived(uint8_t data);





static void SerialP__SerialFrameComm__putDone(void );
#line 85
static void SerialP__SerialFrameComm__delimiterReceived(void );
# 75 "/home/csilab/tinyos/tos/interfaces/TaskBasic.nc"
static void SerialP__defaultSerialFlushTask__runTask(void );
# 71 "/home/csilab/tinyos/tos/lib/serial/SendBytePacket.nc"
static error_t SerialP__SendBytePacket__completeSend(void );
#line 62
static error_t SerialP__SendBytePacket__startSend(uint8_t first_byte);
# 75 "/home/csilab/tinyos/tos/interfaces/TaskBasic.nc"
static void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTask__runTask(void );
# 75 "/home/csilab/tinyos/tos/interfaces/Send.nc"
static error_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__Send__send(
# 51 "/home/csilab/tinyos/tos/lib/serial/SerialDispatcherP.nc"
uart_id_t arg_0x7f7fed95c9c0, 
# 67 "/home/csilab/tinyos/tos/interfaces/Send.nc"
message_t * msg, 







uint8_t len);
#line 100
static void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__Send__default__sendDone(
# 51 "/home/csilab/tinyos/tos/lib/serial/SerialDispatcherP.nc"
uart_id_t arg_0x7f7fed95c9c0, 
# 96 "/home/csilab/tinyos/tos/interfaces/Send.nc"
message_t * msg, 



error_t error);
# 75 "/home/csilab/tinyos/tos/interfaces/TaskBasic.nc"
static void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__signalSendDone__runTask(void );
# 78 "/home/csilab/tinyos/tos/interfaces/Receive.nc"
static 
#line 74
message_t * 



/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__Receive__default__receive(
# 50 "/home/csilab/tinyos/tos/lib/serial/SerialDispatcherP.nc"
uart_id_t arg_0x7f7fed95fda0, 
# 71 "/home/csilab/tinyos/tos/interfaces/Receive.nc"
message_t * msg, 
void * payload, 





uint8_t len);
# 31 "/home/csilab/tinyos/tos/lib/serial/SerialPacketInfo.nc"
static uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__default__upperLength(
# 54 "/home/csilab/tinyos/tos/lib/serial/SerialDispatcherP.nc"
uart_id_t arg_0x7f7fed95be30, 
# 31 "/home/csilab/tinyos/tos/lib/serial/SerialPacketInfo.nc"
message_t *msg, uint8_t dataLinkLen);
#line 15
static uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__default__offset(
# 54 "/home/csilab/tinyos/tos/lib/serial/SerialDispatcherP.nc"
uart_id_t arg_0x7f7fed95be30);
# 23 "/home/csilab/tinyos/tos/lib/serial/SerialPacketInfo.nc"
static uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__default__dataLinkLength(
# 54 "/home/csilab/tinyos/tos/lib/serial/SerialDispatcherP.nc"
uart_id_t arg_0x7f7fed95be30, 
# 23 "/home/csilab/tinyos/tos/lib/serial/SerialPacketInfo.nc"
message_t *msg, uint8_t upperLen);
# 81 "/home/csilab/tinyos/tos/lib/serial/SendBytePacket.nc"
static uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__SendBytePacket__nextByte(void );









static void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__SendBytePacket__sendCompleted(error_t error);
# 62 "/home/csilab/tinyos/tos/lib/serial/ReceiveBytePacket.nc"
static error_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__ReceiveBytePacket__startPacket(void );






static void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__ReceiveBytePacket__byteReceived(uint8_t data);










static void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__ReceiveBytePacket__endPacket(error_t result);
# 79 "/home/csilab/tinyos/tos/interfaces/UartStream.nc"
static void HdlcTranslateC__UartStream__receivedByte(uint8_t byte);
#line 99
static void HdlcTranslateC__UartStream__receiveDone(
#line 95
uint8_t * buf, 



uint16_t len, error_t error);
#line 57
static void HdlcTranslateC__UartStream__sendDone(
#line 53
uint8_t * buf, 



uint16_t len, error_t error);
# 56 "/home/csilab/tinyos/tos/lib/serial/SerialFrameComm.nc"
static error_t HdlcTranslateC__SerialFrameComm__putDelimiter(void );
#line 79
static void HdlcTranslateC__SerialFrameComm__resetReceive(void );
#line 65
static error_t HdlcTranslateC__SerialFrameComm__putData(uint8_t data);
# 65 "/home/csilab/tinyos/tos/interfaces/ResourceConfigure.nc"
static void /*Msp430Uart1P.UartP*/Msp430UartP__0__ResourceConfigure__unconfigure(
# 44 "/home/csilab/tinyos/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x7f7fed84e6f0);
# 59 "/home/csilab/tinyos/tos/interfaces/ResourceConfigure.nc"
static void /*Msp430Uart1P.UartP*/Msp430UartP__0__ResourceConfigure__configure(
# 44 "/home/csilab/tinyos/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x7f7fed84e6f0);
# 46 "/home/csilab/tinyos/tos/interfaces/UartByte.nc"
static error_t /*Msp430Uart1P.UartP*/Msp430UartP__0__UartByte__send(
# 46 "/home/csilab/tinyos/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x7f7fed84bb50, 
# 46 "/home/csilab/tinyos/tos/interfaces/UartByte.nc"
uint8_t byte);
# 39 "/home/csilab/tinyos/tos/chips/msp430/usart/Msp430UartConfigure.nc"
static msp430_uart_union_config_t */*Msp430Uart1P.UartP*/Msp430UartP__0__Msp430UartConfigure__default__getConfig(
# 49 "/home/csilab/tinyos/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x7f7fed849d40);
# 48 "/home/csilab/tinyos/tos/interfaces/UartStream.nc"
static error_t /*Msp430Uart1P.UartP*/Msp430UartP__0__UartStream__send(
# 45 "/home/csilab/tinyos/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x7f7fed84c4c0, 
# 44 "/home/csilab/tinyos/tos/interfaces/UartStream.nc"
uint8_t * buf, 



uint16_t len);
#line 79
static void /*Msp430Uart1P.UartP*/Msp430UartP__0__UartStream__default__receivedByte(
# 45 "/home/csilab/tinyos/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x7f7fed84c4c0, 
# 79 "/home/csilab/tinyos/tos/interfaces/UartStream.nc"
uint8_t byte);
#line 99
static void /*Msp430Uart1P.UartP*/Msp430UartP__0__UartStream__default__receiveDone(
# 45 "/home/csilab/tinyos/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x7f7fed84c4c0, 
# 95 "/home/csilab/tinyos/tos/interfaces/UartStream.nc"
uint8_t * buf, 



uint16_t len, error_t error);
#line 57
static void /*Msp430Uart1P.UartP*/Msp430UartP__0__UartStream__default__sendDone(
# 45 "/home/csilab/tinyos/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x7f7fed84c4c0, 
# 53 "/home/csilab/tinyos/tos/interfaces/UartStream.nc"
uint8_t * buf, 



uint16_t len, error_t error);
# 82 "/home/csilab/tinyos/tos/lib/timer/Counter.nc"
static void /*Msp430Uart1P.UartP*/Msp430UartP__0__Counter__overflow(void );
# 120 "/home/csilab/tinyos/tos/interfaces/Resource.nc"
static error_t /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartResource__default__release(
# 48 "/home/csilab/tinyos/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x7f7fed84a980);
# 97 "/home/csilab/tinyos/tos/interfaces/Resource.nc"
static error_t /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartResource__default__immediateRequest(
# 48 "/home/csilab/tinyos/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x7f7fed84a980);
# 102 "/home/csilab/tinyos/tos/interfaces/Resource.nc"
static void /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartResource__granted(
# 48 "/home/csilab/tinyos/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x7f7fed84a980);
# 128 "/home/csilab/tinyos/tos/interfaces/Resource.nc"
static bool /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartResource__default__isOwner(
# 48 "/home/csilab/tinyos/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x7f7fed84a980);
# 120 "/home/csilab/tinyos/tos/interfaces/Resource.nc"
static error_t /*Msp430Uart1P.UartP*/Msp430UartP__0__Resource__release(
# 43 "/home/csilab/tinyos/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x7f7fed84f380);
# 97 "/home/csilab/tinyos/tos/interfaces/Resource.nc"
static error_t /*Msp430Uart1P.UartP*/Msp430UartP__0__Resource__immediateRequest(
# 43 "/home/csilab/tinyos/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x7f7fed84f380);
# 102 "/home/csilab/tinyos/tos/interfaces/Resource.nc"
static void /*Msp430Uart1P.UartP*/Msp430UartP__0__Resource__default__granted(
# 43 "/home/csilab/tinyos/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x7f7fed84f380);
# 54 "/home/csilab/tinyos/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
static void /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartInterrupts__rxDone(
# 51 "/home/csilab/tinyos/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x7f7fed8261c0, 
# 54 "/home/csilab/tinyos/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
uint8_t data);
#line 49
static void /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartInterrupts__txDone(
# 51 "/home/csilab/tinyos/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x7f7fed8261c0);
# 143 "/home/csilab/tinyos/tos/chips/msp430/usart/HplMsp430Usart.nc"
static void HplMsp430Usart1P__Usart__enableUartRx(void );
#line 123
static void HplMsp430Usart1P__Usart__enableUart(void );
#line 97
static void HplMsp430Usart1P__Usart__resetUsart(bool reset);
#line 179
static void HplMsp430Usart1P__Usart__disableIntr(void );

static void HplMsp430Usart1P__Usart__enableTxIntr(void );
#line 90
static void HplMsp430Usart1P__Usart__setUmctl(uint8_t umctl);
#line 133
static void HplMsp430Usart1P__Usart__enableUartTx(void );
#line 178
static void HplMsp430Usart1P__Usart__disableTxIntr(void );
#line 148
static void HplMsp430Usart1P__Usart__disableUartRx(void );
#line 182
static void HplMsp430Usart1P__Usart__enableIntr(void );




static bool HplMsp430Usart1P__Usart__isTxIntrPending(void );
#line 207
static void HplMsp430Usart1P__Usart__clrIntr(void );
#line 80
static void HplMsp430Usart1P__Usart__setUbr(uint16_t ubr);
#line 224
static void HplMsp430Usart1P__Usart__tx(uint8_t data);
#line 128
static void HplMsp430Usart1P__Usart__disableUart(void );
#line 174
static void HplMsp430Usart1P__Usart__setModeUart(msp430_uart_union_config_t *config);
#line 202
static void HplMsp430Usart1P__Usart__clrTxIntr(void );
#line 158
static void HplMsp430Usart1P__Usart__disableSpi(void );
#line 138
static void HplMsp430Usart1P__Usart__disableUartTx(void );
# 95 "/home/csilab/tinyos/tos/interfaces/AsyncStdControl.nc"
static error_t HplMsp430Usart1P__AsyncStdControl__start(void );









static error_t HplMsp430Usart1P__AsyncStdControl__stop(void );
# 73 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static bool /*HplMsp430GeneralIOC.P10*/HplMsp430GeneralIOP__0__IO__get(void );
#line 66
static uint8_t /*HplMsp430GeneralIOC.P10*/HplMsp430GeneralIOP__0__IO__getRaw(void );






static bool /*HplMsp430GeneralIOC.P13*/HplMsp430GeneralIOP__3__IO__get(void );
#line 66
static uint8_t /*HplMsp430GeneralIOC.P13*/HplMsp430GeneralIOP__3__IO__getRaw(void );
#line 78
static void /*HplMsp430GeneralIOC.P14*/HplMsp430GeneralIOP__4__IO__makeInput(void );
#line 73
static bool /*HplMsp430GeneralIOC.P14*/HplMsp430GeneralIOP__4__IO__get(void );
#line 66
static uint8_t /*HplMsp430GeneralIOC.P14*/HplMsp430GeneralIOP__4__IO__getRaw(void );
#line 78
static void /*HplMsp430GeneralIOC.P24*/HplMsp430GeneralIOP__12__IO__makeInput(void );






static void /*HplMsp430GeneralIOC.P24*/HplMsp430GeneralIOP__12__IO__makeOutput(void );
#line 73
static bool /*HplMsp430GeneralIOC.P24*/HplMsp430GeneralIOP__12__IO__get(void );
#line 66
static uint8_t /*HplMsp430GeneralIOC.P24*/HplMsp430GeneralIOP__12__IO__getRaw(void );
#line 53
static void /*HplMsp430GeneralIOC.P24*/HplMsp430GeneralIOP__12__IO__clr(void );
#line 99
static void /*HplMsp430GeneralIOC.P31*/HplMsp430GeneralIOP__17__IO__selectIOFunc(void );
#line 92
static void /*HplMsp430GeneralIOC.P31*/HplMsp430GeneralIOP__17__IO__selectModuleFunc(void );






static void /*HplMsp430GeneralIOC.P32*/HplMsp430GeneralIOP__18__IO__selectIOFunc(void );
#line 92
static void /*HplMsp430GeneralIOC.P32*/HplMsp430GeneralIOP__18__IO__selectModuleFunc(void );






static void /*HplMsp430GeneralIOC.P33*/HplMsp430GeneralIOP__19__IO__selectIOFunc(void );
#line 92
static void /*HplMsp430GeneralIOC.P33*/HplMsp430GeneralIOP__19__IO__selectModuleFunc(void );






static void /*HplMsp430GeneralIOC.P34*/HplMsp430GeneralIOP__20__IO__selectIOFunc(void );
#line 99
static void /*HplMsp430GeneralIOC.P35*/HplMsp430GeneralIOP__21__IO__selectIOFunc(void );
#line 99
static void /*HplMsp430GeneralIOC.P36*/HplMsp430GeneralIOP__22__IO__selectIOFunc(void );
#line 92
static void /*HplMsp430GeneralIOC.P36*/HplMsp430GeneralIOP__22__IO__selectModuleFunc(void );






static void /*HplMsp430GeneralIOC.P37*/HplMsp430GeneralIOP__23__IO__selectIOFunc(void );
#line 92
static void /*HplMsp430GeneralIOC.P37*/HplMsp430GeneralIOP__23__IO__selectModuleFunc(void );
#line 78
static void /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__makeInput(void );
#line 73
static bool /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__get(void );
#line 99
static void /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__selectIOFunc(void );
#line 66
static uint8_t /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__getRaw(void );
#line 92
static void /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__selectModuleFunc(void );
#line 85
static void /*HplMsp430GeneralIOC.P42*/HplMsp430GeneralIOP__26__IO__makeOutput(void );
#line 48
static void /*HplMsp430GeneralIOC.P42*/HplMsp430GeneralIOP__26__IO__set(void );




static void /*HplMsp430GeneralIOC.P42*/HplMsp430GeneralIOP__26__IO__clr(void );
#line 85
static void /*HplMsp430GeneralIOC.P45*/HplMsp430GeneralIOP__29__IO__makeOutput(void );
#line 48
static void /*HplMsp430GeneralIOC.P45*/HplMsp430GeneralIOP__29__IO__set(void );




static void /*HplMsp430GeneralIOC.P45*/HplMsp430GeneralIOP__29__IO__clr(void );
#line 85
static void /*HplMsp430GeneralIOC.P46*/HplMsp430GeneralIOP__30__IO__makeOutput(void );
#line 48
static void /*HplMsp430GeneralIOC.P46*/HplMsp430GeneralIOP__30__IO__set(void );




static void /*HplMsp430GeneralIOC.P46*/HplMsp430GeneralIOP__30__IO__clr(void );
#line 99
static void /*HplMsp430GeneralIOC.P51*/HplMsp430GeneralIOP__33__IO__selectIOFunc(void );
#line 99
static void /*HplMsp430GeneralIOC.P52*/HplMsp430GeneralIOP__34__IO__selectIOFunc(void );
#line 99
static void /*HplMsp430GeneralIOC.P53*/HplMsp430GeneralIOP__35__IO__selectIOFunc(void );
#line 85
static void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__makeOutput(void );
#line 48
static void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__set(void );
#line 85
static void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__makeOutput(void );
#line 48
static void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__set(void );
#line 85
static void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__makeOutput(void );
#line 48
static void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__set(void );
# 62 "/home/csilab/tinyos/tos/interfaces/Init.nc"
static error_t PlatformP__Init__init(void );
#line 62
static error_t MotePlatformC__Init__init(void );
# 46 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430ClockInit.nc"
static void Msp430ClockP__Msp430ClockInit__defaultInitClocks(void );
#line 43
static void Msp430ClockP__Msp430ClockInit__default__initTimerB(void );



static void Msp430ClockP__Msp430ClockInit__defaultInitTimerA(void );
#line 42
static void Msp430ClockP__Msp430ClockInit__default__initTimerA(void );





static void Msp430ClockP__Msp430ClockInit__defaultInitTimerB(void );
#line 45
static void Msp430ClockP__Msp430ClockInit__defaultSetupDcoCalibrate(void );
#line 40
static void Msp430ClockP__Msp430ClockInit__default__setupDcoCalibrate(void );
static void Msp430ClockP__Msp430ClockInit__default__initClocks(void );
# 62 "/home/csilab/tinyos/tos/interfaces/McuPowerOverride.nc"
static mcu_power_t Msp430ClockP__McuPowerOverride__lowestState(void );
# 62 "/home/csilab/tinyos/tos/interfaces/Init.nc"
static error_t Msp430ClockP__Init__init(void );
# 39 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__VectorTimerX0__fired(void );
#line 39
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Overflow__fired(void );
#line 39
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__VectorTimerX1__fired(void );
#line 39
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__default__fired(
# 51 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430TimerP.nc"
uint8_t arg_0x7f7fed4c58b0);
# 45 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430Timer.nc"
static uint16_t /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__get(void );
# 39 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__VectorTimerX0__fired(void );
#line 39
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Overflow__fired(void );
#line 39
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__VectorTimerX1__fired(void );
#line 39
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__default__fired(
# 51 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430TimerP.nc"
uint8_t arg_0x7f7fed4c58b0);
# 45 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430Timer.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__get(void );
static bool /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__isOverflowPending(void );
# 44 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__getEvent(void );
#line 86
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__default__captured(uint16_t time);
# 42 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Control__getControl(void );
# 39 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Event__fired(void );
# 45 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Compare__default__fired(void );
# 48 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Timer__overflow(void );
# 44 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__getEvent(void );
#line 86
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__default__captured(uint16_t time);
# 42 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Control__getControl(void );
# 39 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Event__fired(void );
# 45 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Compare__default__fired(void );
# 48 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Timer__overflow(void );
# 44 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__getEvent(void );
#line 86
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__default__captured(uint16_t time);
# 42 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Control__getControl(void );
# 39 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Event__fired(void );
# 45 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Compare__default__fired(void );
# 48 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Timer__overflow(void );
# 44 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__getEvent(void );
#line 86
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__default__captured(uint16_t time);
# 42 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__getControl(void );
#line 57
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__enableEvents(void );
#line 47
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__setControlAsCompare(void );










static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__disableEvents(void );
#line 44
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__clearPendingInterrupt(void );
# 39 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Event__fired(void );
# 41 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__setEvent(uint16_t time);

static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__setEventFromNow(uint16_t delta);
# 48 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Timer__overflow(void );
# 44 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__getEvent(void );
#line 68
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__clearOverflow(void );
# 55 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430TimerControl.nc"
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__setControlAsCapture(uint8_t cm);
#line 42
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__getControl(void );
#line 57
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__enableEvents(void );
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__disableEvents(void );
#line 44
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__clearPendingInterrupt(void );
# 39 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Event__fired(void );
# 45 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Compare__default__fired(void );
# 48 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Timer__overflow(void );
# 44 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__getEvent(void );
#line 86
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__default__captured(uint16_t time);
# 42 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__getControl(void );
#line 57
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__enableEvents(void );
#line 47
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__setControlAsCompare(void );










static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__disableEvents(void );
#line 44
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__clearPendingInterrupt(void );
# 39 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Event__fired(void );
# 41 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__setEvent(uint16_t time);

static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__setEventFromNow(uint16_t delta);
# 48 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Timer__overflow(void );
# 44 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__getEvent(void );
#line 86
static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__default__captured(uint16_t time);
# 42 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Control__getControl(void );
#line 57
static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Control__enableEvents(void );
static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Control__disableEvents(void );
#line 44
static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Control__clearPendingInterrupt(void );
# 39 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Event__fired(void );
# 41 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Compare__setEvent(uint16_t time);

static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Compare__setEventFromNow(uint16_t delta);
# 48 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Timer__overflow(void );
# 44 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__getEvent(void );
#line 86
static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__default__captured(uint16_t time);
# 42 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Control__getControl(void );
#line 57
static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Control__enableEvents(void );
static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Control__disableEvents(void );
#line 44
static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Control__clearPendingInterrupt(void );
# 39 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Event__fired(void );
# 41 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Compare__setEvent(uint16_t time);

static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Compare__setEventFromNow(uint16_t delta);
# 48 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Timer__overflow(void );
# 44 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__getEvent(void );
#line 86
static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__default__captured(uint16_t time);
# 42 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Control__getControl(void );
#line 57
static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Control__enableEvents(void );
static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Control__disableEvents(void );
#line 44
static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Control__clearPendingInterrupt(void );
# 39 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Event__fired(void );
# 41 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Compare__setEvent(uint16_t time);

static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Compare__setEventFromNow(uint16_t delta);
# 48 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Timer__overflow(void );
# 44 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__getEvent(void );
#line 86
static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__default__captured(uint16_t time);
# 42 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Control__getControl(void );
#line 58
static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Control__disableEvents(void );
# 39 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Event__fired(void );
# 48 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Timer__overflow(void );
# 76 "/home/csilab/tinyos/tos/interfaces/McuSleep.nc"
static void McuSleepC__McuSleep__sleep(void );
# 48 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__overflow(void );
# 64 "/home/csilab/tinyos/tos/lib/timer/Counter.nc"
static /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__size_type /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__get(void );






static bool /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__isOverflowPending(void );
# 62 "/home/csilab/tinyos/tos/interfaces/Init.nc"
static error_t LedsP__Init__init(void );
# 46 "/home/csilab/tinyos/tos/interfaces/GeneralIO.nc"
static void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__makeOutput(void );
#line 40
static void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__set(void );





static void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__makeOutput(void );
#line 40
static void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__set(void );





static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__makeOutput(void );
#line 40
static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__set(void );
# 54 "/home/csilab/tinyos/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
static void /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__0__Interrupts__default__rxDone(
# 39 "/home/csilab/tinyos/tos/chips/msp430/usart/Msp430UsartShareP.nc"
uint8_t arg_0x7f7fed2b0600, 
# 54 "/home/csilab/tinyos/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
uint8_t data);
#line 49
static void /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__0__Interrupts__default__txDone(
# 39 "/home/csilab/tinyos/tos/chips/msp430/usart/Msp430UsartShareP.nc"
uint8_t arg_0x7f7fed2b0600);
# 54 "/home/csilab/tinyos/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
static void /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__0__RawInterrupts__rxDone(uint8_t data);
#line 49
static void /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__0__RawInterrupts__txDone(void );
# 67 "/home/csilab/tinyos/tos/interfaces/TaskBasic.nc"
static error_t SchedulerBasicP__TaskBasic__postTask(
# 56 "/home/csilab/tinyos/tos/system/SchedulerBasicP.nc"
uint8_t arg_0x7f7fedb6be60);
# 75 "/home/csilab/tinyos/tos/interfaces/TaskBasic.nc"
static void SchedulerBasicP__TaskBasic__default__runTask(
# 56 "/home/csilab/tinyos/tos/system/SchedulerBasicP.nc"
uint8_t arg_0x7f7fedb6be60);
# 57 "/home/csilab/tinyos/tos/interfaces/Scheduler.nc"
static void SchedulerBasicP__Scheduler__init(void );
#line 72
static void SchedulerBasicP__Scheduler__taskLoop(void );
#line 65
static bool SchedulerBasicP__Scheduler__runNextTask(void );
# 62 "/home/csilab/tinyos/tos/interfaces/Init.nc"
static error_t /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__1__Init__init(void );
# 53 "/home/csilab/tinyos/tos/interfaces/ResourceQueue.nc"
static bool /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__1__FcfsQueue__isEmpty(void );
#line 70
static resource_client_id_t /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__1__FcfsQueue__dequeue(void );
# 61 "/home/csilab/tinyos/tos/interfaces/ResourceRequested.nc"
static void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceRequested__default__immediateRequested(
# 55 "/home/csilab/tinyos/tos/system/ArbiterP.nc"
uint8_t arg_0x7f7fed25e840);
# 65 "/home/csilab/tinyos/tos/interfaces/ResourceConfigure.nc"
static void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__default__unconfigure(
# 60 "/home/csilab/tinyos/tos/system/ArbiterP.nc"
uint8_t arg_0x7f7fed25bc40);
# 59 "/home/csilab/tinyos/tos/interfaces/ResourceConfigure.nc"
static void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__default__configure(
# 60 "/home/csilab/tinyos/tos/system/ArbiterP.nc"
uint8_t arg_0x7f7fed25bc40);
# 56 "/home/csilab/tinyos/tos/interfaces/ResourceDefaultOwner.nc"
static error_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__release(void );
# 120 "/home/csilab/tinyos/tos/interfaces/Resource.nc"
static error_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__Resource__release(
# 54 "/home/csilab/tinyos/tos/system/ArbiterP.nc"
uint8_t arg_0x7f7fed25f4b0);
# 97 "/home/csilab/tinyos/tos/interfaces/Resource.nc"
static error_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__Resource__immediateRequest(
# 54 "/home/csilab/tinyos/tos/system/ArbiterP.nc"
uint8_t arg_0x7f7fed25f4b0);
# 102 "/home/csilab/tinyos/tos/interfaces/Resource.nc"
static void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__Resource__default__granted(
# 54 "/home/csilab/tinyos/tos/system/ArbiterP.nc"
uint8_t arg_0x7f7fed25f4b0);
# 128 "/home/csilab/tinyos/tos/interfaces/Resource.nc"
static bool /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__Resource__isOwner(
# 54 "/home/csilab/tinyos/tos/system/ArbiterP.nc"
uint8_t arg_0x7f7fed25f4b0);
# 90 "/home/csilab/tinyos/tos/interfaces/ArbiterInfo.nc"
static bool /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ArbiterInfo__inUse(void );







static uint8_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ArbiterInfo__userId(void );
# 75 "/home/csilab/tinyos/tos/interfaces/TaskBasic.nc"
static void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__grantedTask__runTask(void );
# 62 "/home/csilab/tinyos/tos/lib/power/PowerDownCleanup.nc"
static void /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__PowerDownCleanup__default__cleanup(void );
# 46 "/home/csilab/tinyos/tos/interfaces/ResourceDefaultOwner.nc"
static void /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__ResourceDefaultOwner__granted(void );
#line 81
static void /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__ResourceDefaultOwner__immediateRequested(void );
# 39 "/home/csilab/tinyos/tos/chips/msp430/usart/Msp430UartConfigure.nc"
static msp430_uart_union_config_t *TelosSerialP__Msp430UartConfigure__getConfig(void );
# 102 "/home/csilab/tinyos/tos/interfaces/Resource.nc"
static void TelosSerialP__Resource__granted(void );
# 95 "/home/csilab/tinyos/tos/interfaces/StdControl.nc"
static error_t TelosSerialP__StdControl__start(void );









static error_t TelosSerialP__StdControl__stop(void );
# 31 "/home/csilab/tinyos/tos/lib/serial/SerialPacketInfo.nc"
static uint8_t SerialPacketInfoActiveMessageP__Info__upperLength(message_t *msg, uint8_t dataLinkLen);
#line 15
static uint8_t SerialPacketInfoActiveMessageP__Info__offset(void );







static uint8_t SerialPacketInfoActiveMessageP__Info__dataLinkLength(message_t *msg, uint8_t upperLen);
# 80 "/home/csilab/tinyos/tos/interfaces/AMSend.nc"
static error_t /*PrintfC.SerialAMSenderC.AMQueueEntryP*/AMQueueEntryP__0__AMSend__send(am_addr_t addr, 
#line 71
message_t * msg, 








uint8_t len);
# 100 "/home/csilab/tinyos/tos/interfaces/Send.nc"
static void /*PrintfC.SerialAMSenderC.AMQueueEntryP*/AMQueueEntryP__0__Send__sendDone(
#line 96
message_t * msg, 



error_t error);
# 110 "/home/csilab/tinyos/tos/interfaces/AMSend.nc"
static void /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__AMSend__sendDone(
# 48 "/home/csilab/tinyos/tos/system/AMQueueImplP.nc"
am_id_t arg_0x7f7fed176840, 
# 103 "/home/csilab/tinyos/tos/interfaces/AMSend.nc"
message_t * msg, 






error_t error);
# 75 "/home/csilab/tinyos/tos/interfaces/Send.nc"
static error_t /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__Send__send(
# 46 "/home/csilab/tinyos/tos/system/AMQueueImplP.nc"
uint8_t arg_0x7f7fed178630, 
# 67 "/home/csilab/tinyos/tos/interfaces/Send.nc"
message_t * msg, 







uint8_t len);
#line 100
static void /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__Send__default__sendDone(
# 46 "/home/csilab/tinyos/tos/system/AMQueueImplP.nc"
uint8_t arg_0x7f7fed178630, 
# 96 "/home/csilab/tinyos/tos/interfaces/Send.nc"
message_t * msg, 



error_t error);
# 75 "/home/csilab/tinyos/tos/interfaces/TaskBasic.nc"
static void /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__errorTask__runTask(void );
#line 75
static void /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__CancelTask__runTask(void );
# 73 "/home/csilab/tinyos/tos/interfaces/Queue.nc"
static 
#line 71
/*PrintfC.QueueC*/QueueC__0__Queue__t  

/*PrintfC.QueueC*/QueueC__0__Queue__head(void );
#line 90
static error_t /*PrintfC.QueueC*/QueueC__0__Queue__enqueue(
#line 86
/*PrintfC.QueueC*/QueueC__0__Queue__t  newVal);
#line 65
static uint8_t /*PrintfC.QueueC*/QueueC__0__Queue__maxSize(void );
#line 81
static 
#line 79
/*PrintfC.QueueC*/QueueC__0__Queue__t  

/*PrintfC.QueueC*/QueueC__0__Queue__dequeue(void );
#line 50
static bool /*PrintfC.QueueC*/QueueC__0__Queue__empty(void );







static uint8_t /*PrintfC.QueueC*/QueueC__0__Queue__size(void );
# 110 "/home/csilab/tinyos/tos/interfaces/AMSend.nc"
static void PrintfP__AMSend__sendDone(
#line 103
message_t * msg, 






error_t error);
# 75 "/home/csilab/tinyos/tos/interfaces/TaskBasic.nc"
static void PrintfP__retrySend__runTask(void );
# 62 "/home/csilab/tinyos/tos/interfaces/Init.nc"
static error_t PrintfP__Init__init(void );
# 49 "/home/csilab/tinyos/tos/lib/printf/Putchar.nc"
static int PrintfP__Putchar__putchar(int c);
# 62 "/home/csilab/tinyos/tos/interfaces/Init.nc"
static error_t PutcharP__Init__init(void );
# 35 "/home/csilab/upma/interfaces/RadioPowerControl.nc"
static error_t CC2420CsmaP__RadioPowerControl__start(void );
# 40 "/home/csilab/upma/interfaces/AsyncSend.nc"
static error_t CC2420CsmaP__Send__send(
#line 34
message_t * msg, 





uint8_t len);
#line 71
static 
#line 69
void * 

CC2420CsmaP__Send__getPayload(
#line 68
message_t * msg, 


uint8_t len);
#line 63
static uint8_t CC2420CsmaP__Send__maxPayloadLength(void );
# 91 "/home/csilab/upma/chips/cc2420/CC2420Transmit.nc"
static void CC2420CsmaP__CC2420Transmit__sendDone(message_t * p_msg, error_t error);
# 75 "/home/csilab/tinyos/tos/interfaces/TaskBasic.nc"
static void CC2420CsmaP__signalStartDone__runTask(void );
#line 75
static void CC2420CsmaP__signalStopDone__runTask(void );
# 76 "/home/csilab/tinyos/tos/chips/cc2420/interfaces/CC2420Power.nc"
static void CC2420CsmaP__CC2420Power__startOscillatorDone(void );
#line 56
static void CC2420CsmaP__CC2420Power__startVRegDone(void );
# 102 "/home/csilab/tinyos/tos/interfaces/Resource.nc"
static void CC2420CsmaP__Resource__granted(void );
# 75 "/home/csilab/tinyos/tos/interfaces/TaskBasic.nc"
static void CC2420CsmaP__stopDone_task__runTask(void );
#line 75
static void CC2420CsmaP__startDone_task__runTask(void );
# 93 "/home/csilab/tinyos/tos/chips/cc2420/interfaces/CC2420Config.nc"
static bool CC2420ControlP__CC2420Config__isAddressRecognitionEnabled(void );
#line 117
static bool CC2420ControlP__CC2420Config__isAutoAckEnabled(void );
#line 112
static bool CC2420ControlP__CC2420Config__isHwAutoAckDefault(void );
#line 71
static uint16_t CC2420ControlP__CC2420Config__getShortAddr(void );
#line 54
static error_t CC2420ControlP__CC2420Config__sync(void );
#line 77
static uint16_t CC2420ControlP__CC2420Config__getPanAddr(void );
# 78 "/home/csilab/tinyos/tos/lib/timer/Alarm.nc"
static void CC2420ControlP__StartupTimer__fired(void );
# 63 "/home/csilab/tinyos/tos/interfaces/Read.nc"
static void CC2420ControlP__ReadRssi__default__readDone(error_t result, CC2420ControlP__ReadRssi__val_t val);
# 75 "/home/csilab/tinyos/tos/interfaces/TaskBasic.nc"
static void CC2420ControlP__syncDone__runTask(void );
# 62 "/home/csilab/tinyos/tos/interfaces/Init.nc"
static error_t CC2420ControlP__Init__init(void );
# 102 "/home/csilab/tinyos/tos/interfaces/Resource.nc"
static void CC2420ControlP__SpiResource__granted(void );
#line 102
static void CC2420ControlP__SyncResource__granted(void );
# 71 "/home/csilab/tinyos/tos/chips/cc2420/interfaces/CC2420Power.nc"
static error_t CC2420ControlP__CC2420Power__startOscillator(void );
#line 90
static error_t CC2420ControlP__CC2420Power__rxOn(void );
#line 51
static error_t CC2420ControlP__CC2420Power__startVReg(void );
#line 63
static error_t CC2420ControlP__CC2420Power__stopVReg(void );
# 75 "/home/csilab/tinyos/tos/interfaces/TaskBasic.nc"
static void CC2420ControlP__sync__runTask(void );
# 120 "/home/csilab/tinyos/tos/interfaces/Resource.nc"
static error_t CC2420ControlP__Resource__release(void );
#line 88
static error_t CC2420ControlP__Resource__request(void );
# 68 "/home/csilab/tinyos/tos/interfaces/GpioInterrupt.nc"
static void CC2420ControlP__InterruptCCA__fired(void );
# 102 "/home/csilab/tinyos/tos/interfaces/Resource.nc"
static void CC2420ControlP__RssiResource__granted(void );
# 45 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__fired(void );
# 48 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430Timer__overflow(void );
# 103 "/home/csilab/tinyos/tos/lib/timer/Alarm.nc"
static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Alarm__startAt(/*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Alarm__size_type t0, /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Alarm__size_type dt);
#line 73
static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Alarm__stop(void );
# 62 "/home/csilab/tinyos/tos/interfaces/Init.nc"
static error_t /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Init__init(void );
# 82 "/home/csilab/tinyos/tos/lib/timer/Counter.nc"
static void /*Counter32khz32C.Transform*/TransformCounterC__0__CounterFrom__overflow(void );
#line 64
static /*Counter32khz32C.Transform*/TransformCounterC__0__Counter__size_type /*Counter32khz32C.Transform*/TransformCounterC__0__Counter__get(void );
# 109 "/home/csilab/tinyos/tos/lib/timer/Alarm.nc"
static /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Alarm__size_type /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Alarm__getNow(void );
#line 103
static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Alarm__startAt(/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Alarm__size_type t0, /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Alarm__size_type dt);
#line 66
static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Alarm__start(/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Alarm__size_type dt);






static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Alarm__stop(void );




static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__AlarmFrom__fired(void );
# 82 "/home/csilab/tinyos/tos/lib/timer/Counter.nc"
static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Counter__overflow(void );
# 44 "/home/csilab/tinyos/tos/interfaces/GeneralIO.nc"
static void /*HplCC2420PinsC.CCAM*/Msp430GpioC__3__GeneralIO__makeInput(void );
#line 43
static bool /*HplCC2420PinsC.CCAM*/Msp430GpioC__3__GeneralIO__get(void );


static void /*HplCC2420PinsC.CSNM*/Msp430GpioC__4__GeneralIO__makeOutput(void );
#line 40
static void /*HplCC2420PinsC.CSNM*/Msp430GpioC__4__GeneralIO__set(void );
static void /*HplCC2420PinsC.CSNM*/Msp430GpioC__4__GeneralIO__clr(void );

static bool /*HplCC2420PinsC.FIFOM*/Msp430GpioC__5__GeneralIO__get(void );
#line 43
static bool /*HplCC2420PinsC.FIFOPM*/Msp430GpioC__6__GeneralIO__get(void );


static void /*HplCC2420PinsC.RSTNM*/Msp430GpioC__7__GeneralIO__makeOutput(void );
#line 40
static void /*HplCC2420PinsC.RSTNM*/Msp430GpioC__7__GeneralIO__set(void );
static void /*HplCC2420PinsC.RSTNM*/Msp430GpioC__7__GeneralIO__clr(void );


static void /*HplCC2420PinsC.SFDM*/Msp430GpioC__8__GeneralIO__makeInput(void );
#line 43
static bool /*HplCC2420PinsC.SFDM*/Msp430GpioC__8__GeneralIO__get(void );


static void /*HplCC2420PinsC.VRENM*/Msp430GpioC__9__GeneralIO__makeOutput(void );
#line 40
static void /*HplCC2420PinsC.VRENM*/Msp430GpioC__9__GeneralIO__set(void );
static void /*HplCC2420PinsC.VRENM*/Msp430GpioC__9__GeneralIO__clr(void );
# 86 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Msp430Capture__captured(uint16_t time);
# 54 "/home/csilab/tinyos/tos/interfaces/GpioCapture.nc"
static error_t /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Capture__captureFallingEdge(void );
#line 66
static void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Capture__disable(void );
#line 53
static error_t /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Capture__captureRisingEdge(void );
# 52 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
static void HplMsp430InterruptP__Port14__clear(void );
#line 47
static void HplMsp430InterruptP__Port14__disable(void );
#line 67
static void HplMsp430InterruptP__Port14__edge(bool low_to_high);
#line 42
static void HplMsp430InterruptP__Port14__enable(void );









static void HplMsp430InterruptP__Port26__clear(void );
#line 72
static void HplMsp430InterruptP__Port26__default__fired(void );
#line 52
static void HplMsp430InterruptP__Port17__clear(void );
#line 72
static void HplMsp430InterruptP__Port17__default__fired(void );
#line 52
static void HplMsp430InterruptP__Port21__clear(void );
#line 72
static void HplMsp430InterruptP__Port21__default__fired(void );
#line 52
static void HplMsp430InterruptP__Port12__clear(void );
#line 72
static void HplMsp430InterruptP__Port12__default__fired(void );
#line 52
static void HplMsp430InterruptP__Port24__clear(void );
#line 72
static void HplMsp430InterruptP__Port24__default__fired(void );
#line 52
static void HplMsp430InterruptP__Port15__clear(void );
#line 72
static void HplMsp430InterruptP__Port15__default__fired(void );
#line 52
static void HplMsp430InterruptP__Port27__clear(void );
#line 72
static void HplMsp430InterruptP__Port27__default__fired(void );
#line 52
static void HplMsp430InterruptP__Port10__clear(void );
#line 47
static void HplMsp430InterruptP__Port10__disable(void );
#line 67
static void HplMsp430InterruptP__Port10__edge(bool low_to_high);
#line 42
static void HplMsp430InterruptP__Port10__enable(void );









static void HplMsp430InterruptP__Port22__clear(void );
#line 72
static void HplMsp430InterruptP__Port22__default__fired(void );
#line 52
static void HplMsp430InterruptP__Port13__clear(void );
#line 72
static void HplMsp430InterruptP__Port13__default__fired(void );
#line 52
static void HplMsp430InterruptP__Port25__clear(void );
#line 72
static void HplMsp430InterruptP__Port25__default__fired(void );
#line 52
static void HplMsp430InterruptP__Port16__clear(void );
#line 72
static void HplMsp430InterruptP__Port16__default__fired(void );
#line 52
static void HplMsp430InterruptP__Port20__clear(void );
#line 72
static void HplMsp430InterruptP__Port20__default__fired(void );
#line 52
static void HplMsp430InterruptP__Port11__clear(void );
#line 72
static void HplMsp430InterruptP__Port11__default__fired(void );
#line 52
static void HplMsp430InterruptP__Port23__clear(void );
#line 72
static void HplMsp430InterruptP__Port23__default__fired(void );
#line 72
static void /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__HplInterrupt__fired(void );
# 61 "/home/csilab/tinyos/tos/interfaces/GpioInterrupt.nc"
static error_t /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__Interrupt__disable(void );
#line 53
static error_t /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__Interrupt__enableRisingEdge(void );
# 72 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
static void /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__HplInterrupt__fired(void );
# 61 "/home/csilab/tinyos/tos/interfaces/GpioInterrupt.nc"
static error_t /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__Interrupt__disable(void );
#line 54
static error_t /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__Interrupt__enableFallingEdge(void );
# 82 "/home/csilab/tinyos/tos/interfaces/SpiPacket.nc"
static void CC2420SpiP__SpiPacket__sendDone(
#line 75
uint8_t * txBuf, 
uint8_t * rxBuf, 





uint16_t len, 
error_t error);
# 62 "/home/csilab/tinyos/tos/chips/cc2420/interfaces/CC2420Fifo.nc"
static error_t CC2420SpiP__Fifo__continueRead(
# 46 "/home/csilab/tinyos/tos/chips/cc2420/spi/CC2420SpiP.nc"
uint8_t arg_0x7f7fecc8f6e0, 
# 62 "/home/csilab/tinyos/tos/chips/cc2420/interfaces/CC2420Fifo.nc"
uint8_t * data, uint8_t length);
#line 91
static void CC2420SpiP__Fifo__default__writeDone(
# 46 "/home/csilab/tinyos/tos/chips/cc2420/spi/CC2420SpiP.nc"
uint8_t arg_0x7f7fecc8f6e0, 
# 91 "/home/csilab/tinyos/tos/chips/cc2420/interfaces/CC2420Fifo.nc"
uint8_t * data, uint8_t length, error_t error);
#line 82
static cc2420_status_t CC2420SpiP__Fifo__write(
# 46 "/home/csilab/tinyos/tos/chips/cc2420/spi/CC2420SpiP.nc"
uint8_t arg_0x7f7fecc8f6e0, 
# 82 "/home/csilab/tinyos/tos/chips/cc2420/interfaces/CC2420Fifo.nc"
uint8_t * data, uint8_t length);
#line 51
static cc2420_status_t CC2420SpiP__Fifo__beginRead(
# 46 "/home/csilab/tinyos/tos/chips/cc2420/spi/CC2420SpiP.nc"
uint8_t arg_0x7f7fecc8f6e0, 
# 51 "/home/csilab/tinyos/tos/chips/cc2420/interfaces/CC2420Fifo.nc"
uint8_t * data, uint8_t length);
#line 71
static void CC2420SpiP__Fifo__default__readDone(
# 46 "/home/csilab/tinyos/tos/chips/cc2420/spi/CC2420SpiP.nc"
uint8_t arg_0x7f7fecc8f6e0, 
# 71 "/home/csilab/tinyos/tos/chips/cc2420/interfaces/CC2420Fifo.nc"
uint8_t * data, uint8_t length, error_t error);
# 31 "/home/csilab/tinyos/tos/chips/cc2420/interfaces/ChipSpiResource.nc"
static void CC2420SpiP__ChipSpiResource__abortRelease(void );







static error_t CC2420SpiP__ChipSpiResource__attemptRelease(void );
# 102 "/home/csilab/tinyos/tos/interfaces/Resource.nc"
static void CC2420SpiP__SpiResource__granted(void );
# 63 "/home/csilab/tinyos/tos/chips/cc2420/interfaces/CC2420Ram.nc"
static cc2420_status_t CC2420SpiP__Ram__write(
# 47 "/home/csilab/tinyos/tos/chips/cc2420/spi/CC2420SpiP.nc"
uint16_t arg_0x7f7fecc8e9f0, 
# 63 "/home/csilab/tinyos/tos/chips/cc2420/interfaces/CC2420Ram.nc"
uint8_t offset, uint8_t * data, uint8_t length);
# 55 "/home/csilab/tinyos/tos/chips/cc2420/interfaces/CC2420Register.nc"
static cc2420_status_t CC2420SpiP__Reg__read(
# 48 "/home/csilab/tinyos/tos/chips/cc2420/spi/CC2420SpiP.nc"
uint8_t arg_0x7f7fecc8c880, 
# 55 "/home/csilab/tinyos/tos/chips/cc2420/interfaces/CC2420Register.nc"
uint16_t *data);







static cc2420_status_t CC2420SpiP__Reg__write(
# 48 "/home/csilab/tinyos/tos/chips/cc2420/spi/CC2420SpiP.nc"
uint8_t arg_0x7f7fecc8c880, 
# 63 "/home/csilab/tinyos/tos/chips/cc2420/interfaces/CC2420Register.nc"
uint16_t data);
# 120 "/home/csilab/tinyos/tos/interfaces/Resource.nc"
static error_t CC2420SpiP__Resource__release(
# 45 "/home/csilab/tinyos/tos/chips/cc2420/spi/CC2420SpiP.nc"
uint8_t arg_0x7f7fecc90360);
# 97 "/home/csilab/tinyos/tos/interfaces/Resource.nc"
static error_t CC2420SpiP__Resource__immediateRequest(
# 45 "/home/csilab/tinyos/tos/chips/cc2420/spi/CC2420SpiP.nc"
uint8_t arg_0x7f7fecc90360);
# 88 "/home/csilab/tinyos/tos/interfaces/Resource.nc"
static error_t CC2420SpiP__Resource__request(
# 45 "/home/csilab/tinyos/tos/chips/cc2420/spi/CC2420SpiP.nc"
uint8_t arg_0x7f7fecc90360);
# 102 "/home/csilab/tinyos/tos/interfaces/Resource.nc"
static void CC2420SpiP__Resource__default__granted(
# 45 "/home/csilab/tinyos/tos/chips/cc2420/spi/CC2420SpiP.nc"
uint8_t arg_0x7f7fecc90360);
# 128 "/home/csilab/tinyos/tos/interfaces/Resource.nc"
static bool CC2420SpiP__Resource__isOwner(
# 45 "/home/csilab/tinyos/tos/chips/cc2420/spi/CC2420SpiP.nc"
uint8_t arg_0x7f7fecc90360);
# 75 "/home/csilab/tinyos/tos/interfaces/TaskBasic.nc"
static void CC2420SpiP__grant__runTask(void );
# 53 "/home/csilab/tinyos/tos/chips/cc2420/interfaces/CC2420Strobe.nc"
static cc2420_status_t CC2420SpiP__Strobe__strobe(
# 49 "/home/csilab/tinyos/tos/chips/cc2420/spi/CC2420SpiP.nc"
uint8_t arg_0x7f7fecc8b700);
# 62 "/home/csilab/tinyos/tos/interfaces/Init.nc"
static error_t StateImplP__Init__init(void );
# 56 "/home/csilab/tinyos/tos/interfaces/State.nc"
static void StateImplP__State__toIdle(
# 67 "/home/csilab/tinyos/tos/system/StateImplP.nc"
uint8_t arg_0x7f7fecc159c0);
# 66 "/home/csilab/tinyos/tos/interfaces/State.nc"
static bool StateImplP__State__isState(
# 67 "/home/csilab/tinyos/tos/system/StateImplP.nc"
uint8_t arg_0x7f7fecc159c0, 
# 66 "/home/csilab/tinyos/tos/interfaces/State.nc"
uint8_t myState);
#line 61
static bool StateImplP__State__isIdle(
# 67 "/home/csilab/tinyos/tos/system/StateImplP.nc"
uint8_t arg_0x7f7fecc159c0);
# 45 "/home/csilab/tinyos/tos/interfaces/State.nc"
static error_t StateImplP__State__requestState(
# 67 "/home/csilab/tinyos/tos/system/StateImplP.nc"
uint8_t arg_0x7f7fecc159c0, 
# 45 "/home/csilab/tinyos/tos/interfaces/State.nc"
uint8_t reqState);





static void StateImplP__State__forceState(
# 67 "/home/csilab/tinyos/tos/system/StateImplP.nc"
uint8_t arg_0x7f7fecc159c0, 
# 51 "/home/csilab/tinyos/tos/interfaces/State.nc"
uint8_t reqState);
# 65 "/home/csilab/tinyos/tos/interfaces/ResourceConfigure.nc"
static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__ResourceConfigure__unconfigure(
# 76 "/home/csilab/tinyos/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x7f7fecbdfd30);
# 59 "/home/csilab/tinyos/tos/interfaces/ResourceConfigure.nc"
static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__ResourceConfigure__configure(
# 76 "/home/csilab/tinyos/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x7f7fecbdfd30);
# 70 "/home/csilab/tinyos/tos/interfaces/SpiPacket.nc"
static error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__SpiPacket__send(
# 79 "/home/csilab/tinyos/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x7f7fecbdb020, 
# 59 "/home/csilab/tinyos/tos/interfaces/SpiPacket.nc"
uint8_t * txBuf, 

uint8_t * rxBuf, 








uint16_t len);
#line 82
static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__SpiPacket__default__sendDone(
# 79 "/home/csilab/tinyos/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x7f7fecbdb020, 
# 75 "/home/csilab/tinyos/tos/interfaces/SpiPacket.nc"
uint8_t * txBuf, 
uint8_t * rxBuf, 





uint16_t len, 
error_t error);
# 39 "/home/csilab/tinyos/tos/chips/msp430/usart/Msp430SpiConfigure.nc"
static msp430_spi_union_config_t */*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Msp430SpiConfigure__default__getConfig(
# 82 "/home/csilab/tinyos/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x7f7fecbd9110);
# 45 "/home/csilab/tinyos/tos/interfaces/SpiByte.nc"
static uint8_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__SpiByte__write(uint8_t tx);
# 120 "/home/csilab/tinyos/tos/interfaces/Resource.nc"
static error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__default__release(
# 81 "/home/csilab/tinyos/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x7f7fecbdbe30);
# 97 "/home/csilab/tinyos/tos/interfaces/Resource.nc"
static error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__default__immediateRequest(
# 81 "/home/csilab/tinyos/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x7f7fecbdbe30);
# 88 "/home/csilab/tinyos/tos/interfaces/Resource.nc"
static error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__default__request(
# 81 "/home/csilab/tinyos/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x7f7fecbdbe30);
# 102 "/home/csilab/tinyos/tos/interfaces/Resource.nc"
static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__granted(
# 81 "/home/csilab/tinyos/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x7f7fecbdbe30);
# 128 "/home/csilab/tinyos/tos/interfaces/Resource.nc"
static bool /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__default__isOwner(
# 81 "/home/csilab/tinyos/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x7f7fecbdbe30);
# 120 "/home/csilab/tinyos/tos/interfaces/Resource.nc"
static error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Resource__release(
# 75 "/home/csilab/tinyos/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x7f7fecbe29f0);
# 97 "/home/csilab/tinyos/tos/interfaces/Resource.nc"
static error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Resource__immediateRequest(
# 75 "/home/csilab/tinyos/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x7f7fecbe29f0);
# 88 "/home/csilab/tinyos/tos/interfaces/Resource.nc"
static error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Resource__request(
# 75 "/home/csilab/tinyos/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x7f7fecbe29f0);
# 102 "/home/csilab/tinyos/tos/interfaces/Resource.nc"
static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Resource__default__granted(
# 75 "/home/csilab/tinyos/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x7f7fecbe29f0);
# 128 "/home/csilab/tinyos/tos/interfaces/Resource.nc"
static bool /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Resource__isOwner(
# 75 "/home/csilab/tinyos/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x7f7fecbe29f0);
# 54 "/home/csilab/tinyos/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartInterrupts__rxDone(uint8_t data);
#line 49
static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartInterrupts__txDone(void );
# 75 "/home/csilab/tinyos/tos/interfaces/TaskBasic.nc"
static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__signalDone_task__runTask(void );
# 180 "/home/csilab/tinyos/tos/chips/msp430/usart/HplMsp430Usart.nc"
static void HplMsp430Usart0P__Usart__enableRxIntr(void );
#line 197
static void HplMsp430Usart0P__Usart__clrRxIntr(void );
#line 97
static void HplMsp430Usart0P__Usart__resetUsart(bool reset);
#line 179
static void HplMsp430Usart0P__Usart__disableIntr(void );
#line 90
static void HplMsp430Usart0P__Usart__setUmctl(uint8_t umctl);
#line 177
static void HplMsp430Usart0P__Usart__disableRxIntr(void );
#line 207
static void HplMsp430Usart0P__Usart__clrIntr(void );
#line 80
static void HplMsp430Usart0P__Usart__setUbr(uint16_t ubr);
#line 224
static void HplMsp430Usart0P__Usart__tx(uint8_t data);
#line 128
static void HplMsp430Usart0P__Usart__disableUart(void );
#line 153
static void HplMsp430Usart0P__Usart__enableSpi(void );
#line 168
static void HplMsp430Usart0P__Usart__setModeSpi(msp430_spi_union_config_t *config);
#line 231
static uint8_t HplMsp430Usart0P__Usart__rx(void );
#line 192
static bool HplMsp430Usart0P__Usart__isRxIntrPending(void );
#line 158
static void HplMsp430Usart0P__Usart__disableSpi(void );
# 54 "/home/csilab/tinyos/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
static void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__1__Interrupts__default__rxDone(
# 39 "/home/csilab/tinyos/tos/chips/msp430/usart/Msp430UsartShareP.nc"
uint8_t arg_0x7f7fed2b0600, 
# 54 "/home/csilab/tinyos/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
uint8_t data);
#line 49
static void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__1__Interrupts__default__txDone(
# 39 "/home/csilab/tinyos/tos/chips/msp430/usart/Msp430UsartShareP.nc"
uint8_t arg_0x7f7fed2b0600);
# 39 "/home/csilab/tinyos/tos/chips/msp430/usart/HplMsp430I2CInterrupts.nc"
static void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__1__RawI2CInterrupts__fired(void );
#line 39
static void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__1__I2CInterrupts__default__fired(
# 40 "/home/csilab/tinyos/tos/chips/msp430/usart/Msp430UsartShareP.nc"
uint8_t arg_0x7f7fed2ae1c0);
# 54 "/home/csilab/tinyos/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
static void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__1__RawInterrupts__rxDone(uint8_t data);
#line 49
static void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__1__RawInterrupts__txDone(void );
# 62 "/home/csilab/tinyos/tos/interfaces/Init.nc"
static error_t /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__2__Init__init(void );
# 79 "/home/csilab/tinyos/tos/interfaces/ResourceQueue.nc"
static error_t /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__2__FcfsQueue__enqueue(resource_client_id_t id);
#line 53
static bool /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__2__FcfsQueue__isEmpty(void );








static bool /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__2__FcfsQueue__isEnqueued(resource_client_id_t id);







static resource_client_id_t /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__2__FcfsQueue__dequeue(void );
# 53 "/home/csilab/tinyos/tos/interfaces/ResourceRequested.nc"
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceRequested__default__requested(
# 55 "/home/csilab/tinyos/tos/system/ArbiterP.nc"
uint8_t arg_0x7f7fed25e840);
# 61 "/home/csilab/tinyos/tos/interfaces/ResourceRequested.nc"
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceRequested__default__immediateRequested(
# 55 "/home/csilab/tinyos/tos/system/ArbiterP.nc"
uint8_t arg_0x7f7fed25e840);
# 65 "/home/csilab/tinyos/tos/interfaces/ResourceConfigure.nc"
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceConfigure__default__unconfigure(
# 60 "/home/csilab/tinyos/tos/system/ArbiterP.nc"
uint8_t arg_0x7f7fed25bc40);
# 59 "/home/csilab/tinyos/tos/interfaces/ResourceConfigure.nc"
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceConfigure__default__configure(
# 60 "/home/csilab/tinyos/tos/system/ArbiterP.nc"
uint8_t arg_0x7f7fed25bc40);
# 56 "/home/csilab/tinyos/tos/interfaces/ResourceDefaultOwner.nc"
static error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceDefaultOwner__release(void );
#line 73
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceDefaultOwner__default__requested(void );
#line 46
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceDefaultOwner__default__granted(void );
#line 81
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceDefaultOwner__default__immediateRequested(void );
# 120 "/home/csilab/tinyos/tos/interfaces/Resource.nc"
static error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__Resource__release(
# 54 "/home/csilab/tinyos/tos/system/ArbiterP.nc"
uint8_t arg_0x7f7fed25f4b0);
# 97 "/home/csilab/tinyos/tos/interfaces/Resource.nc"
static error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__Resource__immediateRequest(
# 54 "/home/csilab/tinyos/tos/system/ArbiterP.nc"
uint8_t arg_0x7f7fed25f4b0);
# 88 "/home/csilab/tinyos/tos/interfaces/Resource.nc"
static error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__Resource__request(
# 54 "/home/csilab/tinyos/tos/system/ArbiterP.nc"
uint8_t arg_0x7f7fed25f4b0);
# 102 "/home/csilab/tinyos/tos/interfaces/Resource.nc"
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__Resource__default__granted(
# 54 "/home/csilab/tinyos/tos/system/ArbiterP.nc"
uint8_t arg_0x7f7fed25f4b0);
# 128 "/home/csilab/tinyos/tos/interfaces/Resource.nc"
static bool /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__Resource__isOwner(
# 54 "/home/csilab/tinyos/tos/system/ArbiterP.nc"
uint8_t arg_0x7f7fed25f4b0);
# 90 "/home/csilab/tinyos/tos/interfaces/ArbiterInfo.nc"
static bool /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ArbiterInfo__inUse(void );







static uint8_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ArbiterInfo__userId(void );
# 75 "/home/csilab/tinyos/tos/interfaces/TaskBasic.nc"
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__grantedTask__runTask(void );
# 7 "/home/csilab/tinyos/tos/chips/msp430/usart/HplMsp430I2C.nc"
static void HplMsp430I2C0P__HplI2C__clearModeI2C(void );
#line 6
static bool HplMsp430I2C0P__HplI2C__isI2C(void );
# 55 "/home/csilab/tinyos/tos/system/ActiveMessageAddressC.nc"
static am_addr_t ActiveMessageAddressC__amAddress(void );
# 50 "/home/csilab/tinyos/tos/interfaces/ActiveMessageAddress.nc"
static am_addr_t ActiveMessageAddressC__ActiveMessageAddress__amAddress(void );




static am_group_t ActiveMessageAddressC__ActiveMessageAddress__amGroup(void );
# 12 "/home/csilab/tinyos/tos/platforms/epic/chips/ds2411/ReadId48.nc"
static error_t Ds2411P__ReadId48__read(uint8_t *id);
# 10 "/home/csilab/tinyos/tos/platforms/epic/chips/ds2411/OneWireStream.nc"
static error_t OneWireMasterC__OneWire__read(uint8_t cmd, uint8_t *buf, uint8_t len);
# 66 "/home/csilab/tinyos/tos/lib/timer/BusyWait.nc"
static void /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__BusyWait__wait(/*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__BusyWait__size_type dt);
# 82 "/home/csilab/tinyos/tos/lib/timer/Counter.nc"
static void /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__Counter__overflow(void );
# 48 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430CounterMicroC.Counter*/Msp430CounterC__1__Msp430Timer__overflow(void );
# 64 "/home/csilab/tinyos/tos/lib/timer/Counter.nc"
static /*Msp430CounterMicroC.Counter*/Msp430CounterC__1__Counter__size_type /*Msp430CounterMicroC.Counter*/Msp430CounterC__1__Counter__get(void );
# 44 "/home/csilab/tinyos/tos/interfaces/GeneralIO.nc"
static void /*Ds2411C.Gpio*/Msp430GpioC__11__GeneralIO__makeInput(void );
#line 43
static bool /*Ds2411C.Gpio*/Msp430GpioC__11__GeneralIO__get(void );


static void /*Ds2411C.Gpio*/Msp430GpioC__11__GeneralIO__makeOutput(void );
#line 41
static void /*Ds2411C.Gpio*/Msp430GpioC__11__GeneralIO__clr(void );
# 48 "/home/csilab/tinyos/tos/interfaces/LocalIeeeEui64.nc"
static ieee_eui64_t DallasId48ToIeeeEui64C__LocalIeeeEui64__getId(void );
# 43 "/home/csilab/tinyos/tos/chips/cc2420/interfaces/ReceiveIndicator.nc"
static bool CC2420TransmitP__EnergyIndicator__isReceiving(void );
# 61 "/home/csilab/tinyos/tos/interfaces/GpioCapture.nc"
static void CC2420TransmitP__CaptureSFD__captured(uint16_t time);
# 78 "/home/csilab/tinyos/tos/lib/timer/Alarm.nc"
static void CC2420TransmitP__BackoffTimer__fired(void );
# 76 "/home/csilab/upma/chips/cc2420/CC2420Transmit.nc"
static error_t CC2420TransmitP__Send__send(message_t * p_msg);
# 24 "/home/csilab/tinyos/tos/chips/cc2420/interfaces/ChipSpiResource.nc"
static void CC2420TransmitP__ChipSpiResource__releasing(void );
# 63 "/home/csilab/tinyos/tos/chips/cc2420/interfaces/CC2420Receive.nc"
static void CC2420TransmitP__CC2420Receive__receive(uint8_t type, message_t * message);
# 62 "/home/csilab/tinyos/tos/interfaces/Init.nc"
static error_t CC2420TransmitP__Init__init(void );
# 102 "/home/csilab/tinyos/tos/interfaces/Resource.nc"
static void CC2420TransmitP__SpiResource__granted(void );
# 95 "/home/csilab/tinyos/tos/interfaces/AsyncStdControl.nc"
static error_t CC2420TransmitP__AsyncStdControl__start(void );









static error_t CC2420TransmitP__AsyncStdControl__stop(void );
# 91 "/home/csilab/tinyos/tos/chips/cc2420/interfaces/CC2420Fifo.nc"
static void CC2420TransmitP__TXFIFO__writeDone(uint8_t * data, uint8_t length, error_t error);
#line 71
static void CC2420TransmitP__TXFIFO__readDone(uint8_t * data, uint8_t length, error_t error);
# 55 "/home/csilab/tinyos/tos/chips/cc2420/interfaces/CC2420Config.nc"
static void CC2420ReceiveP__CC2420Config__syncDone(error_t error);
# 55 "/home/csilab/tinyos/tos/chips/cc2420/interfaces/CC2420Receive.nc"
static void CC2420ReceiveP__CC2420Receive__sfd_dropped(void );
#line 49
static void CC2420ReceiveP__CC2420Receive__sfd(uint32_t time);
# 62 "/home/csilab/tinyos/tos/interfaces/Init.nc"
static error_t CC2420ReceiveP__Init__init(void );
# 52 "/home/csilab/upma/interfaces/AsyncReceive.nc"
static void CC2420ReceiveP__Receive__updateBuffer(
#line 49
message_t * msg);
# 102 "/home/csilab/tinyos/tos/interfaces/Resource.nc"
static void CC2420ReceiveP__SpiResource__granted(void );
# 91 "/home/csilab/tinyos/tos/chips/cc2420/interfaces/CC2420Fifo.nc"
static void CC2420ReceiveP__RXFIFO__writeDone(uint8_t * data, uint8_t length, error_t error);
#line 71
static void CC2420ReceiveP__RXFIFO__readDone(uint8_t * data, uint8_t length, error_t error);
# 43 "/home/csilab/tinyos/tos/chips/cc2420/interfaces/ReceiveIndicator.nc"
static bool CC2420ReceiveP__PacketIndicator__isReceiving(void );
# 68 "/home/csilab/tinyos/tos/interfaces/GpioInterrupt.nc"
static void CC2420ReceiveP__InterruptFIFOP__fired(void );
# 95 "/home/csilab/tinyos/tos/interfaces/AsyncStdControl.nc"
static error_t CC2420ReceiveP__StdControl__start(void );









static error_t CC2420ReceiveP__StdControl__stop(void );
# 77 "/home/csilab/tinyos/tos/chips/cc2420/interfaces/CC2420Packet.nc"
static void CC2420PacketP__CC2420Packet__setNetwork(message_t * p_msg, uint8_t networkId);
#line 75
static uint8_t CC2420PacketP__CC2420Packet__getNetwork(message_t * p_msg);
# 70 "/home/csilab/tinyos/tos/interfaces/PacketTimeStamp.nc"
static void CC2420PacketP__PacketTimeStamp32khz__clear(
#line 66
message_t * msg);
#line 78
static void CC2420PacketP__PacketTimeStamp32khz__set(
#line 73
message_t * msg, 




CC2420PacketP__PacketTimeStamp32khz__size_type value);
# 42 "/home/csilab/tinyos/tos/chips/cc2420/interfaces/CC2420PacketBody.nc"
static cc2420_header_t * CC2420PacketP__CC2420PacketBody__getHeader(message_t * msg);










static cc2420_metadata_t * CC2420PacketP__CC2420PacketBody__getMetadata(message_t * msg);
# 58 "/home/csilab/tinyos/tos/chips/cc2420/interfaces/PacketTimeSyncOffset.nc"
static uint8_t CC2420PacketP__PacketTimeSyncOffset__get(
#line 53
message_t * msg);
#line 50
static bool CC2420PacketP__PacketTimeSyncOffset__isSet(
#line 46
message_t * msg);
# 59 "/home/csilab/tinyos/tos/interfaces/PacketAcknowledgements.nc"
static error_t CC2420PacketP__Acks__requestAck(
#line 53
message_t * msg);
#line 85
static bool CC2420PacketP__Acks__wasAcked(
#line 80
message_t * msg);
# 82 "/home/csilab/tinyos/tos/lib/timer/Counter.nc"
static void /*CC2420PacketC.CounterToLocalTimeC*/CounterToLocalTimeC__0__Counter__overflow(void );
# 45 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430Compare__fired(void );
# 48 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430Timer__overflow(void );
# 103 "/home/csilab/tinyos/tos/lib/timer/Alarm.nc"
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Alarm__startAt(/*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Alarm__size_type t0, /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Alarm__size_type dt);
#line 73
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Alarm__stop(void );
# 62 "/home/csilab/tinyos/tos/interfaces/Init.nc"
static error_t /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Init__init(void );
# 82 "/home/csilab/tinyos/tos/lib/timer/Counter.nc"
static void /*CounterMilli32C.Transform*/TransformCounterC__1__CounterFrom__overflow(void );
#line 64
static /*CounterMilli32C.Transform*/TransformCounterC__1__Counter__size_type /*CounterMilli32C.Transform*/TransformCounterC__1__Counter__get(void );
# 109 "/home/csilab/tinyos/tos/lib/timer/Alarm.nc"
static /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__Alarm__size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__Alarm__getNow(void );
#line 103
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__Alarm__startAt(/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__Alarm__size_type t0, /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__Alarm__size_type dt);
#line 116
static /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__Alarm__size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__Alarm__getAlarm(void );
#line 73
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__Alarm__stop(void );




static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__AlarmFrom__fired(void );
# 82 "/home/csilab/tinyos/tos/lib/timer/Counter.nc"
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__Counter__overflow(void );
# 75 "/home/csilab/tinyos/tos/interfaces/TaskBasic.nc"
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired__runTask(void );
# 78 "/home/csilab/tinyos/tos/lib/timer/Alarm.nc"
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__fired(void );
# 136 "/home/csilab/tinyos/tos/lib/timer/Timer.nc"
static uint32_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__getNow(void );
#line 129
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__startOneShotAt(uint32_t t0, uint32_t dt);
#line 78
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__stop(void );
# 75 "/home/csilab/tinyos/tos/interfaces/TaskBasic.nc"
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer__runTask(void );
# 83 "/home/csilab/tinyos/tos/lib/timer/Timer.nc"
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__fired(void );
#line 83
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__default__fired(
# 48 "/home/csilab/tinyos/tos/lib/timer/VirtualizeTimerC.nc"
uint8_t arg_0x7f7fec5fd950);
# 64 "/home/csilab/tinyos/tos/lib/timer/Timer.nc"
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startPeriodic(
# 48 "/home/csilab/tinyos/tos/lib/timer/VirtualizeTimerC.nc"
uint8_t arg_0x7f7fec5fd950, 
# 64 "/home/csilab/tinyos/tos/lib/timer/Timer.nc"
uint32_t dt);








static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startOneShot(
# 48 "/home/csilab/tinyos/tos/lib/timer/VirtualizeTimerC.nc"
uint8_t arg_0x7f7fec5fd950, 
# 73 "/home/csilab/tinyos/tos/lib/timer/Timer.nc"
uint32_t dt);
# 82 "/home/csilab/tinyos/tos/lib/timer/Counter.nc"
static void /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__1__Counter__overflow(void );
# 52 "/home/csilab/tinyos/tos/interfaces/Random.nc"
static uint16_t RandomMlcgC__Random__rand16(void );
#line 46
static uint32_t RandomMlcgC__Random__rand32(void );
# 62 "/home/csilab/tinyos/tos/interfaces/Init.nc"
static error_t RandomMlcgC__Init__init(void );
# 100 "/home/csilab/tinyos/tos/interfaces/Send.nc"
static void UniqueSendP__SubSend__sendDone(
#line 96
message_t * msg, 



error_t error);
#line 75
static error_t UniqueSendP__Send__send(
#line 67
message_t * msg, 







uint8_t len);
#line 112
static uint8_t UniqueSendP__Send__maxPayloadLength(void );
# 62 "/home/csilab/tinyos/tos/interfaces/Init.nc"
static error_t UniqueSendP__Init__init(void );
# 78 "/home/csilab/tinyos/tos/interfaces/Receive.nc"
static 
#line 74
message_t * 



UniqueReceiveP__SubReceive__receive(
#line 71
message_t * msg, 
void * payload, 





uint8_t len);
# 62 "/home/csilab/tinyos/tos/interfaces/Init.nc"
static error_t UniqueReceiveP__Init__init(void );
# 78 "/home/csilab/tinyos/tos/interfaces/Receive.nc"
static 
#line 74
message_t * 



UniqueReceiveP__DuplicateReceive__default__receive(
#line 71
message_t * msg, 
void * payload, 





uint8_t len);
# 100 "/home/csilab/tinyos/tos/interfaces/Send.nc"
static void CC2420TinyosNetworkP__SubSend__sendDone(
#line 96
message_t * msg, 



error_t error);
# 78 "/home/csilab/tinyos/tos/interfaces/Receive.nc"
static 
#line 74
message_t * 



CC2420TinyosNetworkP__SubReceive__receive(
#line 71
message_t * msg, 
void * payload, 





uint8_t len);
# 75 "/home/csilab/tinyos/tos/interfaces/TaskBasic.nc"
static void CC2420TinyosNetworkP__grantTask__runTask(void );
# 75 "/home/csilab/tinyos/tos/interfaces/Send.nc"
static error_t CC2420TinyosNetworkP__ActiveSend__send(
#line 67
message_t * msg, 







uint8_t len);
#line 125
static 
#line 123
void * 

CC2420TinyosNetworkP__ActiveSend__getPayload(
#line 122
message_t * msg, 


uint8_t len);
#line 112
static uint8_t CC2420TinyosNetworkP__ActiveSend__maxPayloadLength(void );
# 78 "/home/csilab/tinyos/tos/interfaces/Receive.nc"
static 
#line 74
message_t * 



CC2420TinyosNetworkP__BareReceive__default__receive(
#line 71
message_t * msg, 
void * payload, 





uint8_t len);
# 120 "/home/csilab/tinyos/tos/interfaces/Resource.nc"
static error_t CC2420TinyosNetworkP__Resource__release(
# 46 "/home/csilab/tinyos/tos/chips/cc2420/lowpan/CC2420TinyosNetworkP.nc"
uint8_t arg_0x7f7fec4cb600);
# 97 "/home/csilab/tinyos/tos/interfaces/Resource.nc"
static error_t CC2420TinyosNetworkP__Resource__immediateRequest(
# 46 "/home/csilab/tinyos/tos/chips/cc2420/lowpan/CC2420TinyosNetworkP.nc"
uint8_t arg_0x7f7fec4cb600);
# 88 "/home/csilab/tinyos/tos/interfaces/Resource.nc"
static error_t CC2420TinyosNetworkP__Resource__request(
# 46 "/home/csilab/tinyos/tos/chips/cc2420/lowpan/CC2420TinyosNetworkP.nc"
uint8_t arg_0x7f7fec4cb600);
# 102 "/home/csilab/tinyos/tos/interfaces/Resource.nc"
static void CC2420TinyosNetworkP__Resource__default__granted(
# 46 "/home/csilab/tinyos/tos/chips/cc2420/lowpan/CC2420TinyosNetworkP.nc"
uint8_t arg_0x7f7fec4cb600);
# 125 "/home/csilab/tinyos/tos/interfaces/Send.nc"
static 
#line 123
void * 

CC2420TinyosNetworkP__BareSend__getPayload(
#line 122
message_t * msg, 


uint8_t len);
#line 100
static void CC2420TinyosNetworkP__BareSend__default__sendDone(
#line 96
message_t * msg, 



error_t error);
# 62 "/home/csilab/tinyos/tos/interfaces/Init.nc"
static error_t /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__Init__init(void );
# 79 "/home/csilab/tinyos/tos/interfaces/ResourceQueue.nc"
static error_t /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__FcfsQueue__enqueue(resource_client_id_t id);
#line 53
static bool /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__FcfsQueue__isEmpty(void );








static bool /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__FcfsQueue__isEnqueued(resource_client_id_t id);







static resource_client_id_t /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__FcfsQueue__dequeue(void );
# 75 "/home/csilab/tinyos/tos/interfaces/TaskBasic.nc"
static void AsyncReceiveAdapterP__receiveDone_task__runTask(void );
# 41 "/home/csilab/upma/interfaces/AsyncReceive.nc"
static void AsyncReceiveAdapterP__AsyncReceive__receive(
#line 37
message_t * msg, 
void * payload, 


uint8_t len);
# 75 "/home/csilab/tinyos/tos/interfaces/Send.nc"
static error_t AsyncSendAdapterP__Send__send(
#line 67
message_t * msg, 







uint8_t len);
#line 112
static uint8_t AsyncSendAdapterP__Send__maxPayloadLength(void );
# 75 "/home/csilab/tinyos/tos/interfaces/TaskBasic.nc"
static void AsyncSendAdapterP__sendDone_task__runTask(void );
# 36 "/home/csilab/upma/interfaces/RadioPowerControl.nc"
static void PowerCycleP__RadioPowerControl__startDone(error_t error);





static void PowerCycleP__RadioPowerControl__stopDone(error_t error);
# 75 "/home/csilab/tinyos/tos/interfaces/TaskBasic.nc"
static void PowerCycleP__startRadio__runTask(void );
# 44 "/home/csilab/tinyos/tos/chips/cc2420/interfaces/ReceiveIndicator.nc"
static void PowerCycleP__PacketIndicator__anypktreceive(void );
# 75 "/home/csilab/tinyos/tos/interfaces/TaskBasic.nc"
static void PowerCycleP__getCca__runTask(void );
# 94 "/home/csilab/tinyos/tos/interfaces/Leds.nc"
static void NoLedsC__Leds__led2Off(void );
#line 89
static void NoLedsC__Leds__led2On(void );
# 37 "/home/csilab/upma/interfaces/LocalTimeExtended.nc"
static bool /*LocalTime32khz16C.LocalTimeP*/LocalTime16P__0__LocalTime__lessThan(/*LocalTime32khz16C.LocalTimeP*/LocalTime16P__0__LocalTime__local_time_t *t1, /*LocalTime32khz16C.LocalTimeP*/LocalTime16P__0__LocalTime__local_time_t *t2);
#line 34
static /*LocalTime32khz16C.LocalTimeP*/LocalTime16P__0__LocalTime__local_time_t /*LocalTime32khz16C.LocalTimeP*/LocalTime16P__0__LocalTime__add(/*LocalTime32khz16C.LocalTimeP*/LocalTime16P__0__LocalTime__local_time_t *t1, /*LocalTime32khz16C.LocalTimeP*/LocalTime16P__0__LocalTime__local_time_t *t2);
#line 33
static /*LocalTime32khz16C.LocalTimeP*/LocalTime16P__0__LocalTime__local_time_t /*LocalTime32khz16C.LocalTimeP*/LocalTime16P__0__LocalTime__getNow(void );
# 62 "/home/csilab/tinyos/tos/interfaces/Init.nc"
static error_t /*LocalTime32khz16C.LocalTimeP*/LocalTime16P__0__Init__init(void );
# 82 "/home/csilab/tinyos/tos/lib/timer/Counter.nc"
static void /*LocalTime32khz16C.LocalTimeP*/LocalTime16P__0__Counter__overflow(void );
# 31 "/home/csilab/upma/interfaces/Slotter.nc"
static void PureTDMASchedulerP__Slotter__slot(uint8_t num);
# 104 "/home/csilab/tinyos/tos/interfaces/SplitControl.nc"
static error_t PureTDMASchedulerP__SplitControl__start(void );
# 48 "/home/csilab/upma/interfaces/AsyncSend.nc"
static void PureTDMASchedulerP__SubSend__sendDone(
#line 44
message_t * msg, 



error_t error);
# 41 "/home/csilab/upma/interfaces/AsyncReceive.nc"
static void PureTDMASchedulerP__SubReceive__receive(
#line 37
message_t * msg, 
void * payload, 


uint8_t len);
# 36 "/home/csilab/upma/interfaces/RadioPowerControl.nc"
static void PureTDMASchedulerP__RadioPowerControl__startDone(error_t error);





static void PureTDMASchedulerP__RadioPowerControl__stopDone(error_t error);
# 40 "/home/csilab/upma/interfaces/AsyncSend.nc"
static error_t PureTDMASchedulerP__Send__send(
#line 34
message_t * msg, 





uint8_t len);
#line 63
static uint8_t PureTDMASchedulerP__Send__maxPayloadLength(void );
# 62 "/home/csilab/tinyos/tos/interfaces/Init.nc"
static error_t PureTDMASchedulerP__Init__init(void );
# 44 "/home/csilab/tinyos/tos/chips/cc2420/interfaces/ReceiveIndicator.nc"
static void PureTDMASchedulerP__ReceiveIndicator__anypktreceive(void );
# 48 "/home/csilab/upma/interfaces/AsyncSend.nc"
static void PureTDMASchedulerP__BeaconSend__sendDone(
#line 44
message_t * msg, 



error_t error);
# 52 "/home/csilab/upma/interfaces/AsyncReceive.nc"
static void PureTDMASchedulerP__Receive__updateBuffer(
#line 49
message_t * msg);
# 78 "/home/csilab/tinyos/tos/lib/timer/Alarm.nc"
static void PureTDMASchedulerP__TxOffset_alarm__fired(void );
# 48 "/home/csilab/upma/interfaces/AsyncSend.nc"
static void /*TDMASlotSenderC.TDMASlotSender.GenericSlotSenderP*/GenericSlotSenderP__0__SubSend__sendDone(
#line 44
message_t * msg, 



error_t error);
# 50 "/home/csilab/upma/interfaces/CcaControl.nc"
static uint16_t /*TDMASlotSenderC.TDMASlotSender.GenericSlotSenderP*/GenericSlotSenderP__0__SubCcaControl__getCongestionBackoff(message_t *msg, uint16_t defaultBackoff);
#line 39
static uint16_t /*TDMASlotSenderC.TDMASlotSender.GenericSlotSenderP*/GenericSlotSenderP__0__SubCcaControl__getInitialBackoff(message_t *msg, uint16_t defaultbackoff);
#line 61
static bool /*TDMASlotSenderC.TDMASlotSender.GenericSlotSenderP*/GenericSlotSenderP__0__SubCcaControl__getCca(message_t *msg, bool defaultCca);
# 60 "/home/csilab/tinyos/tos/interfaces/Boot.nc"
static void /*TDMASlotSenderC.TDMASlotSender.GenericSlotSenderP*/GenericSlotSenderP__0__Boot__booted(void );
# 40 "/home/csilab/upma/interfaces/AsyncSend.nc"
static error_t /*TDMASlotSenderC.TDMASlotSender.GenericSlotSenderP*/GenericSlotSenderP__0__Send__send(
#line 34
message_t * msg, 





uint8_t len);
#line 71
static 
#line 69
void * 

/*TDMASlotSenderC.TDMASlotSender.GenericSlotSenderP*/GenericSlotSenderP__0__Send__getPayload(
#line 68
message_t * msg, 


uint8_t len);
#line 63
static uint8_t /*TDMASlotSenderC.TDMASlotSender.GenericSlotSenderP*/GenericSlotSenderP__0__Send__maxPayloadLength(void );
# 78 "/home/csilab/tinyos/tos/lib/timer/Alarm.nc"
static void /*TDMASlotSenderC.TDMASlotSender.GenericSlotSenderP*/GenericSlotSenderP__0__Alarm__fired(void );
# 62 "/home/csilab/tinyos/tos/interfaces/Init.nc"
static error_t /*TDMASlotSenderC.TDMASlotSender.GenericSlotSenderP*/GenericSlotSenderP__0__Init__init(void );
# 45 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*TDMASlotSenderC.TDMASlotSender.Alarm32khz16C.Msp430Alarm*/Msp430AlarmC__2__Msp430Compare__fired(void );
# 48 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*TDMASlotSenderC.TDMASlotSender.Alarm32khz16C.Msp430Alarm*/Msp430AlarmC__2__Msp430Timer__overflow(void );
# 109 "/home/csilab/tinyos/tos/lib/timer/Alarm.nc"
static /*TDMASlotSenderC.TDMASlotSender.Alarm32khz16C.Msp430Alarm*/Msp430AlarmC__2__Alarm__size_type /*TDMASlotSenderC.TDMASlotSender.Alarm32khz16C.Msp430Alarm*/Msp430AlarmC__2__Alarm__getNow(void );
#line 103
static void /*TDMASlotSenderC.TDMASlotSender.Alarm32khz16C.Msp430Alarm*/Msp430AlarmC__2__Alarm__startAt(/*TDMASlotSenderC.TDMASlotSender.Alarm32khz16C.Msp430Alarm*/Msp430AlarmC__2__Alarm__size_type t0, /*TDMASlotSenderC.TDMASlotSender.Alarm32khz16C.Msp430Alarm*/Msp430AlarmC__2__Alarm__size_type dt);
#line 66
static void /*TDMASlotSenderC.TDMASlotSender.Alarm32khz16C.Msp430Alarm*/Msp430AlarmC__2__Alarm__start(/*TDMASlotSenderC.TDMASlotSender.Alarm32khz16C.Msp430Alarm*/Msp430AlarmC__2__Alarm__size_type dt);
# 60 "/home/csilab/tinyos/tos/interfaces/Boot.nc"
static void GenericSlotterP__Boot__booted(void );
# 78 "/home/csilab/tinyos/tos/lib/timer/Alarm.nc"
static void GenericSlotterP__Alarm__fired(void );
# 62 "/home/csilab/tinyos/tos/interfaces/Init.nc"
static error_t GenericSlotterP__Init__init(void );
# 95 "/home/csilab/tinyos/tos/interfaces/AsyncStdControl.nc"
static error_t GenericSlotterP__AsyncStdControl__start(void );
# 33 "/home/csilab/upma/interfaces/FrameConfiguration.nc"
static void GenericSlotterP__FrameConfiguration__setFrameLength(uint8_t numSlots);
#line 32
static void GenericSlotterP__FrameConfiguration__setSlotLength(uint32_t slotTimeBms);
# 38 "/home/csilab/upma/interfaces/SlotterControl.nc"
static uint8_t GenericSlotterP__SlotterControl__getSlot(void );
#line 31
static error_t GenericSlotterP__SlotterControl__synchronize(uint8_t toSlot);




static uint32_t GenericSlotterP__SlotterControl__getRemaining(void );
static void GenericSlotterP__SlotterControl__stop(void );
# 45 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*GenericSlotterC.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__3__Msp430Compare__fired(void );
# 48 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*GenericSlotterC.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__3__Msp430Timer__overflow(void );
# 103 "/home/csilab/tinyos/tos/lib/timer/Alarm.nc"
static void /*GenericSlotterC.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__3__Alarm__startAt(/*GenericSlotterC.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__3__Alarm__size_type t0, /*GenericSlotterC.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__3__Alarm__size_type dt);
#line 73
static void /*GenericSlotterC.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__3__Alarm__stop(void );
#line 109
static /*GenericSlotterC.Alarm32khz32C.Transform*/TransformAlarmC__2__Alarm__size_type /*GenericSlotterC.Alarm32khz32C.Transform*/TransformAlarmC__2__Alarm__getNow(void );
#line 103
static void /*GenericSlotterC.Alarm32khz32C.Transform*/TransformAlarmC__2__Alarm__startAt(/*GenericSlotterC.Alarm32khz32C.Transform*/TransformAlarmC__2__Alarm__size_type t0, /*GenericSlotterC.Alarm32khz32C.Transform*/TransformAlarmC__2__Alarm__size_type dt);
#line 66
static void /*GenericSlotterC.Alarm32khz32C.Transform*/TransformAlarmC__2__Alarm__start(/*GenericSlotterC.Alarm32khz32C.Transform*/TransformAlarmC__2__Alarm__size_type dt);






static void /*GenericSlotterC.Alarm32khz32C.Transform*/TransformAlarmC__2__Alarm__stop(void );




static void /*GenericSlotterC.Alarm32khz32C.Transform*/TransformAlarmC__2__AlarmFrom__fired(void );
# 82 "/home/csilab/tinyos/tos/lib/timer/Counter.nc"
static void /*GenericSlotterC.Alarm32khz32C.Transform*/TransformAlarmC__2__Counter__overflow(void );
# 48 "/home/csilab/upma/interfaces/AsyncSend.nc"
static void SenderDispatcherP__SubSend__sendDone(
#line 44
message_t * msg, 



error_t error);
# 50 "/home/csilab/upma/interfaces/CcaControl.nc"
static uint16_t SenderDispatcherP__SubCcaControl__getCongestionBackoff(
# 31 "/home/csilab/upma/system/SenderDispatcherP.nc"
am_id_t arg_0x7f7fec199ce0, 
# 50 "/home/csilab/upma/interfaces/CcaControl.nc"
message_t *msg, uint16_t defaultBackoff);
#line 39
static uint16_t SenderDispatcherP__SubCcaControl__getInitialBackoff(
# 31 "/home/csilab/upma/system/SenderDispatcherP.nc"
am_id_t arg_0x7f7fec199ce0, 
# 39 "/home/csilab/upma/interfaces/CcaControl.nc"
message_t *msg, uint16_t defaultbackoff);
#line 61
static bool SenderDispatcherP__SubCcaControl__getCca(
# 31 "/home/csilab/upma/system/SenderDispatcherP.nc"
am_id_t arg_0x7f7fec199ce0, 
# 61 "/home/csilab/upma/interfaces/CcaControl.nc"
message_t *msg, bool defaultCca);
# 40 "/home/csilab/upma/interfaces/AsyncSend.nc"
static error_t SenderDispatcherP__Send__send(
# 39 "/home/csilab/upma/system/SenderDispatcherP.nc"
uint8_t arg_0x7f7fec194250, 
# 34 "/home/csilab/upma/interfaces/AsyncSend.nc"
message_t * msg, 





uint8_t len);
#line 71
static 
#line 69
void * 

SenderDispatcherP__Send__getPayload(
# 39 "/home/csilab/upma/system/SenderDispatcherP.nc"
uint8_t arg_0x7f7fec194250, 
# 68 "/home/csilab/upma/interfaces/AsyncSend.nc"
message_t * msg, 


uint8_t len);
#line 63
static uint8_t SenderDispatcherP__Send__maxPayloadLength(
# 39 "/home/csilab/upma/system/SenderDispatcherP.nc"
uint8_t arg_0x7f7fec194250);
# 48 "/home/csilab/upma/interfaces/AsyncSend.nc"
static void SenderDispatcherP__Send__default__sendDone(
# 39 "/home/csilab/upma/system/SenderDispatcherP.nc"
uint8_t arg_0x7f7fec194250, 
# 44 "/home/csilab/upma/interfaces/AsyncSend.nc"
message_t * msg, 



error_t error);
# 62 "/home/csilab/tinyos/tos/interfaces/Init.nc"
static error_t SenderDispatcherP__Init__init(void );
# 50 "/home/csilab/upma/interfaces/CcaControl.nc"
static uint16_t SenderDispatcherP__SlotsCcaControl__default__getCongestionBackoff(
# 40 "/home/csilab/upma/system/SenderDispatcherP.nc"
uint8_t arg_0x7f7fec1925b0, 
# 50 "/home/csilab/upma/interfaces/CcaControl.nc"
message_t *msg, uint16_t defaultBackoff);
#line 39
static uint16_t SenderDispatcherP__SlotsCcaControl__default__getInitialBackoff(
# 40 "/home/csilab/upma/system/SenderDispatcherP.nc"
uint8_t arg_0x7f7fec1925b0, 
# 39 "/home/csilab/upma/interfaces/CcaControl.nc"
message_t *msg, uint16_t defaultbackoff);
#line 61
static bool SenderDispatcherP__SlotsCcaControl__default__getCca(
# 40 "/home/csilab/upma/system/SenderDispatcherP.nc"
uint8_t arg_0x7f7fec1925b0, 
# 61 "/home/csilab/upma/interfaces/CcaControl.nc"
message_t *msg, bool defaultCca);
# 53 "/home/csilab/upma/interfaces/ChannelMonitor.nc"
static void DummyChannelMonitorC__ChannelMonitor__busy(void );
#line 48
static void DummyChannelMonitorC__ChannelMonitor__free(void );
# 48 "/home/csilab/upma/interfaces/AsyncSend.nc"
static void BeaconSlotP__SubSend__sendDone(
#line 44
message_t * msg, 



error_t error);
# 41 "/home/csilab/upma/interfaces/AsyncReceive.nc"
static void BeaconSlotP__SubReceive__receive(
#line 37
message_t * msg, 
void * payload, 


uint8_t len);
# 78 "/home/csilab/tinyos/tos/lib/timer/Alarm.nc"
static void BeaconSlotP__SyncAlarm__fired(void );
# 50 "/home/csilab/upma/interfaces/CcaControl.nc"
static uint16_t BeaconSlotP__SubCcaControl__getCongestionBackoff(message_t *msg, uint16_t defaultBackoff);
#line 39
static uint16_t BeaconSlotP__SubCcaControl__getInitialBackoff(message_t *msg, uint16_t defaultbackoff);
#line 61
static bool BeaconSlotP__SubCcaControl__getCca(message_t *msg, bool defaultCca);
# 40 "/home/csilab/upma/interfaces/AsyncSend.nc"
static error_t BeaconSlotP__Send__send(
#line 34
message_t * msg, 





uint8_t len);
# 62 "/home/csilab/tinyos/tos/interfaces/Init.nc"
static error_t BeaconSlotP__Init__init(void );
# 52 "/home/csilab/upma/interfaces/AsyncReceive.nc"
static void BeaconSlotP__Receive__updateBuffer(
#line 49
message_t * msg);
# 45 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*PureTDMASchedulerC.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__4__Msp430Compare__fired(void );
# 48 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*PureTDMASchedulerC.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__4__Msp430Timer__overflow(void );
# 103 "/home/csilab/tinyos/tos/lib/timer/Alarm.nc"
static void /*PureTDMASchedulerC.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__4__Alarm__startAt(/*PureTDMASchedulerC.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__4__Alarm__size_type t0, /*PureTDMASchedulerC.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__4__Alarm__size_type dt);





static /*PureTDMASchedulerC.Alarm32khz32C.Transform*/TransformAlarmC__3__Alarm__size_type /*PureTDMASchedulerC.Alarm32khz32C.Transform*/TransformAlarmC__3__Alarm__getNow(void );
#line 103
static void /*PureTDMASchedulerC.Alarm32khz32C.Transform*/TransformAlarmC__3__Alarm__startAt(/*PureTDMASchedulerC.Alarm32khz32C.Transform*/TransformAlarmC__3__Alarm__size_type t0, /*PureTDMASchedulerC.Alarm32khz32C.Transform*/TransformAlarmC__3__Alarm__size_type dt);
#line 66
static void /*PureTDMASchedulerC.Alarm32khz32C.Transform*/TransformAlarmC__3__Alarm__start(/*PureTDMASchedulerC.Alarm32khz32C.Transform*/TransformAlarmC__3__Alarm__size_type dt);
#line 78
static void /*PureTDMASchedulerC.Alarm32khz32C.Transform*/TransformAlarmC__3__AlarmFrom__fired(void );
# 82 "/home/csilab/tinyos/tos/lib/timer/Counter.nc"
static void /*PureTDMASchedulerC.Alarm32khz32C.Transform*/TransformAlarmC__3__Counter__overflow(void );
# 73 "/home/csilab/tinyos/tos/interfaces/Queue.nc"
static 
#line 71
/*PureTDMASchedulerC.HIQueueC*/QueueC__1__Queue__t  

/*PureTDMASchedulerC.HIQueueC*/QueueC__1__Queue__head(void );
#line 90
static error_t /*PureTDMASchedulerC.HIQueueC*/QueueC__1__Queue__enqueue(
#line 86
/*PureTDMASchedulerC.HIQueueC*/QueueC__1__Queue__t  newVal);
#line 65
static uint8_t /*PureTDMASchedulerC.HIQueueC*/QueueC__1__Queue__maxSize(void );
#line 81
static 
#line 79
/*PureTDMASchedulerC.HIQueueC*/QueueC__1__Queue__t  

/*PureTDMASchedulerC.HIQueueC*/QueueC__1__Queue__dequeue(void );
#line 50
static bool /*PureTDMASchedulerC.HIQueueC*/QueueC__1__Queue__empty(void );







static uint8_t /*PureTDMASchedulerC.HIQueueC*/QueueC__1__Queue__size(void );
#line 73
static 
#line 71
/*PureTDMASchedulerC.LOQueueC*/QueueC__2__Queue__t  

/*PureTDMASchedulerC.LOQueueC*/QueueC__2__Queue__head(void );
#line 90
static error_t /*PureTDMASchedulerC.LOQueueC*/QueueC__2__Queue__enqueue(
#line 86
/*PureTDMASchedulerC.LOQueueC*/QueueC__2__Queue__t  newVal);
#line 65
static uint8_t /*PureTDMASchedulerC.LOQueueC*/QueueC__2__Queue__maxSize(void );
#line 81
static 
#line 79
/*PureTDMASchedulerC.LOQueueC*/QueueC__2__Queue__t  

/*PureTDMASchedulerC.LOQueueC*/QueueC__2__Queue__dequeue(void );
#line 50
static bool /*PureTDMASchedulerC.LOQueueC*/QueueC__2__Queue__empty(void );







static uint8_t /*PureTDMASchedulerC.LOQueueC*/QueueC__2__Queue__size(void );
# 45 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*PureTDMASchedulerC.offset_alarm.Msp430Alarm*/Msp430AlarmC__5__Msp430Compare__fired(void );
# 48 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*PureTDMASchedulerC.offset_alarm.Msp430Alarm*/Msp430AlarmC__5__Msp430Timer__overflow(void );
# 8 "/home/csilab/upma/lib/macs/pure-tdma/AdaptiveHART_Util.nc"
static bool AdaptiveHART_UtilC__AdaptiveHART_Util__is_dest(uint8_t flowid, uint8_t nodeid);
#line 5
static uint8_t AdaptiveHART_UtilC__AdaptiveHART_Util__get_release_offset(uint8_t nodeid);
static uint8_t AdaptiveHART_UtilC__AdaptiveHART_Util__get_flow_id(uint8_t nodeid);
static bool AdaptiveHART_UtilC__AdaptiveHART_Util__is_root(uint8_t nodeid);

static uint8_t AdaptiveHART_UtilC__AdaptiveHART_Util__set_destination(uint8_t flowid, uint8_t nodeid);
static uint8_t AdaptiveHART_UtilC__AdaptiveHART_Util__set_TxOffset(uint8_t flowid, uint8_t nodeid);
# 78 "/home/csilab/tinyos/tos/interfaces/Receive.nc"
static 
#line 74
message_t * 



CC2420ActiveMessageP__SubReceive__receive(
#line 71
message_t * msg, 
void * payload, 





uint8_t len);
# 100 "/home/csilab/tinyos/tos/interfaces/Send.nc"
static void CC2420ActiveMessageP__SubSend__sendDone(
#line 96
message_t * msg, 



error_t error);
# 55 "/home/csilab/tinyos/tos/chips/cc2420/interfaces/CC2420Config.nc"
static void CC2420ActiveMessageP__CC2420Config__syncDone(error_t error);
# 50 "/home/csilab/upma/interfaces/CcaControl.nc"
static uint16_t CC2420ActiveMessageP__SubCcaControl__getCongestionBackoff(
# 79 "/home/csilab/upma/chips/cc2420/CC2420ActiveMessageP.nc"
am_id_t arg_0x7f7fec076740, 
# 50 "/home/csilab/upma/interfaces/CcaControl.nc"
message_t *msg, uint16_t defaultBackoff);
#line 39
static uint16_t CC2420ActiveMessageP__SubCcaControl__getInitialBackoff(
# 79 "/home/csilab/upma/chips/cc2420/CC2420ActiveMessageP.nc"
am_id_t arg_0x7f7fec076740, 
# 39 "/home/csilab/upma/interfaces/CcaControl.nc"
message_t *msg, uint16_t defaultbackoff);
#line 61
static bool CC2420ActiveMessageP__SubCcaControl__getCca(
# 79 "/home/csilab/upma/chips/cc2420/CC2420ActiveMessageP.nc"
am_id_t arg_0x7f7fec076740, 
# 61 "/home/csilab/upma/interfaces/CcaControl.nc"
message_t *msg, bool defaultCca);
# 59 "/home/csilab/tinyos/tos/interfaces/SendNotifier.nc"
static void CC2420ActiveMessageP__SendNotifier__default__aboutToSend(
# 70 "/home/csilab/upma/chips/cc2420/CC2420ActiveMessageP.nc"
am_id_t arg_0x7f7fec07d240, 
# 59 "/home/csilab/tinyos/tos/interfaces/SendNotifier.nc"
am_addr_t dest, 
#line 57
message_t * msg);
# 78 "/home/csilab/tinyos/tos/interfaces/Packet.nc"
static uint8_t CC2420ActiveMessageP__Packet__payloadLength(
#line 74
message_t * msg);
#line 126
static 
#line 123
void * 


CC2420ActiveMessageP__Packet__getPayload(
#line 121
message_t * msg, 




uint8_t len);
#line 106
static uint8_t CC2420ActiveMessageP__Packet__maxPayloadLength(void );
#line 94
static void CC2420ActiveMessageP__Packet__setPayloadLength(
#line 90
message_t * msg, 



uint8_t len);
# 80 "/home/csilab/tinyos/tos/interfaces/AMSend.nc"
static error_t CC2420ActiveMessageP__AMSend__send(
# 64 "/home/csilab/upma/chips/cc2420/CC2420ActiveMessageP.nc"
am_id_t arg_0x7f7fec083320, 
# 80 "/home/csilab/tinyos/tos/interfaces/AMSend.nc"
am_addr_t addr, 
#line 71
message_t * msg, 








uint8_t len);
# 78 "/home/csilab/tinyos/tos/interfaces/Receive.nc"
static 
#line 74
message_t * 



CC2420ActiveMessageP__Snoop__default__receive(
# 66 "/home/csilab/upma/chips/cc2420/CC2420ActiveMessageP.nc"
am_id_t arg_0x7f7fec081280, 
# 71 "/home/csilab/tinyos/tos/interfaces/Receive.nc"
message_t * msg, 
void * payload, 





uint8_t len);
# 50 "/home/csilab/upma/interfaces/CcaControl.nc"
static uint16_t CC2420ActiveMessageP__CcaControl__default__getCongestionBackoff(
# 69 "/home/csilab/upma/chips/cc2420/CC2420ActiveMessageP.nc"
am_id_t arg_0x7f7fec07e320, 
# 50 "/home/csilab/upma/interfaces/CcaControl.nc"
message_t *msg, uint16_t defaultBackoff);
#line 39
static uint16_t CC2420ActiveMessageP__CcaControl__default__getInitialBackoff(
# 69 "/home/csilab/upma/chips/cc2420/CC2420ActiveMessageP.nc"
am_id_t arg_0x7f7fec07e320, 
# 39 "/home/csilab/upma/interfaces/CcaControl.nc"
message_t *msg, uint16_t defaultbackoff);
#line 61
static bool CC2420ActiveMessageP__CcaControl__default__getCca(
# 69 "/home/csilab/upma/chips/cc2420/CC2420ActiveMessageP.nc"
am_id_t arg_0x7f7fec07e320, 
# 61 "/home/csilab/upma/interfaces/CcaControl.nc"
message_t *msg, bool defaultCca);
# 78 "/home/csilab/tinyos/tos/interfaces/Receive.nc"
static 
#line 74
message_t * 



CC2420ActiveMessageP__Receive__default__receive(
# 65 "/home/csilab/upma/chips/cc2420/CC2420ActiveMessageP.nc"
am_id_t arg_0x7f7fec082650, 
# 71 "/home/csilab/tinyos/tos/interfaces/Receive.nc"
message_t * msg, 
void * payload, 





uint8_t len);
# 158 "/home/csilab/upma/interfaces/AMPacket.nc"
static uint8_t CC2420ActiveMessageP__AMPacket__headerSize(void );
#line 77
static am_addr_t CC2420ActiveMessageP__AMPacket__source(
#line 73
message_t * amsg);
#line 57
static am_addr_t CC2420ActiveMessageP__AMPacket__address(void );









static am_addr_t CC2420ActiveMessageP__AMPacket__destination(
#line 63
message_t * amsg);
#line 110
static void CC2420ActiveMessageP__AMPacket__setSource(
#line 106
message_t * amsg, 



am_addr_t addr);
#line 92
static void CC2420ActiveMessageP__AMPacket__setDestination(
#line 88
message_t * amsg, 



am_addr_t addr);
#line 136
static am_id_t CC2420ActiveMessageP__AMPacket__type(
#line 132
message_t * amsg);
#line 151
static void CC2420ActiveMessageP__AMPacket__setType(
#line 147
message_t * amsg, 



am_id_t t);
#line 125
static bool CC2420ActiveMessageP__AMPacket__isForMe(
#line 122
message_t * amsg);
# 102 "/home/csilab/tinyos/tos/interfaces/Resource.nc"
static void CC2420ActiveMessageP__RadioResource__granted(void );
# 49 "/home/csilab/tinyos/tos/lib/printf/Putchar.nc"
static int SerialPrintfP__Putchar__putchar(int c);
# 62 "/home/csilab/tinyos/tos/interfaces/Init.nc"
static error_t SerialPrintfP__Init__init(void );
# 95 "/home/csilab/tinyos/tos/interfaces/StdControl.nc"
static error_t SerialPrintfP__StdControl__start(void );
# 83 "/home/csilab/tinyos/tos/lib/timer/Timer.nc"
static void /*TestPureTdmaC.SendingApp.App*/SendingC__0__SendTimer__fired(void );
#line 83
static void /*TestPureTdmaC.SendingApp.App*/SendingC__0__BenchmarkTimer__fired(void );
# 113 "/home/csilab/tinyos/tos/interfaces/SplitControl.nc"
static void /*TestPureTdmaC.SendingApp.App*/SendingC__0__SplitControl__startDone(error_t error);
#line 138
static void /*TestPureTdmaC.SendingApp.App*/SendingC__0__SplitControl__stopDone(error_t error);
# 60 "/home/csilab/tinyos/tos/interfaces/Boot.nc"
static void /*TestPureTdmaC.SendingApp.App*/SendingC__0__Boot__booted(void );
# 75 "/home/csilab/tinyos/tos/interfaces/TaskBasic.nc"
static void /*TestPureTdmaC.SendingApp.App*/SendingC__0__sendTask__runTask(void );
# 110 "/home/csilab/tinyos/tos/interfaces/AMSend.nc"
static void /*TestPureTdmaC.SendingApp.App*/SendingC__0__AMSender__sendDone(
#line 103
message_t * msg, 






error_t error);
# 75 "/home/csilab/tinyos/tos/interfaces/TaskBasic.nc"
static void /*TestPureTdmaC.SendingApp.App*/SendingC__0__flush__runTask(void );
# 82 "/home/csilab/tinyos/tos/lib/timer/Counter.nc"
static void /*TestPureTdmaC.SendingApp.App*/SendingC__0__Counter__overflow(void );
# 83 "/home/csilab/tinyos/tos/lib/timer/Timer.nc"
static void /*TestPureTdmaC.SendingApp.App*/SendingC__0__StartTimer__fired(void );
# 78 "/home/csilab/tinyos/tos/interfaces/Receive.nc"
static 
#line 74
message_t * 



/*TestPureTdmaC.SendingApp.App*/SendingC__0__AMReceiver__receive(
#line 71
message_t * msg, 
void * payload, 





uint8_t len);
# 83 "/home/csilab/tinyos/tos/lib/timer/Timer.nc"
static void /*TestPureTdmaC.SendingApp.App*/SendingC__0__FlushTimer__fired(void );
# 80 "/home/csilab/tinyos/tos/interfaces/AMSend.nc"
static error_t /*TestPureTdmaC.SendingApp.AMSender.SenderC.AMQueueEntryP*/AMQueueEntryP__1__AMSend__send(am_addr_t addr, 
#line 71
message_t * msg, 








uint8_t len);
# 100 "/home/csilab/tinyos/tos/interfaces/Send.nc"
static void /*TestPureTdmaC.SendingApp.AMSender.SenderC.AMQueueEntryP*/AMQueueEntryP__1__Send__sendDone(
#line 96
message_t * msg, 



error_t error);
# 110 "/home/csilab/tinyos/tos/interfaces/AMSend.nc"
static void /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__AMSend__sendDone(
# 48 "/home/csilab/tinyos/tos/system/AMQueueImplP.nc"
am_id_t arg_0x7f7fed176840, 
# 103 "/home/csilab/tinyos/tos/interfaces/AMSend.nc"
message_t * msg, 






error_t error);
# 75 "/home/csilab/tinyos/tos/interfaces/Send.nc"
static error_t /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__Send__send(
# 46 "/home/csilab/tinyos/tos/system/AMQueueImplP.nc"
uint8_t arg_0x7f7fed178630, 
# 67 "/home/csilab/tinyos/tos/interfaces/Send.nc"
message_t * msg, 







uint8_t len);
#line 100
static void /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__Send__default__sendDone(
# 46 "/home/csilab/tinyos/tos/system/AMQueueImplP.nc"
uint8_t arg_0x7f7fed178630, 
# 96 "/home/csilab/tinyos/tos/interfaces/Send.nc"
message_t * msg, 



error_t error);
# 75 "/home/csilab/tinyos/tos/interfaces/TaskBasic.nc"
static void /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__errorTask__runTask(void );
#line 75
static void /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__CancelTask__runTask(void );
# 75 "/home/csilab/tinyos/tos/interfaces/Send.nc"
static error_t /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__SubSend__send(
#line 67
message_t * msg, 







uint8_t len);
# 110 "/home/csilab/tinyos/tos/interfaces/AMSend.nc"
static void /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMSend__sendDone(
# 36 "/home/csilab/upma/lib/serial/SerialActiveMessageP.nc"
am_id_t arg_0x7f7fedab0c50, 
# 103 "/home/csilab/tinyos/tos/interfaces/AMSend.nc"
message_t * msg, 






error_t error);
# 78 "/home/csilab/tinyos/tos/interfaces/Receive.nc"
static 
#line 74
message_t * 



/*SerialActiveMessageC.AM*/SerialActiveMessageP__0__Receive__receive(
# 37 "/home/csilab/upma/lib/serial/SerialActiveMessageP.nc"
am_id_t arg_0x7f7fedaa2e30, 
# 71 "/home/csilab/tinyos/tos/interfaces/Receive.nc"
message_t * msg, 
void * payload, 





uint8_t len);
# 49 "/home/csilab/upma/lib/serial/SerialActiveMessageP.nc"
static inline serial_header_t * /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__getHeader(message_t * msg);







static error_t /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMSend__send(am_id_t id, am_addr_t dest, 
message_t *msg, 
uint8_t len);
#line 85
static inline void /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__SubSend__sendDone(message_t *msg, error_t result);







static inline message_t */*SerialActiveMessageC.AM*/SerialActiveMessageP__0__Receive__default__receive(uint8_t id, message_t *msg, void *payload, uint8_t len);



static inline message_t */*SerialActiveMessageC.AM*/SerialActiveMessageP__0__SubReceive__receive(message_t *msg, void *payload, uint8_t len);








static inline uint8_t /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__Packet__payloadLength(message_t *msg);




static inline void /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__Packet__setPayloadLength(message_t *msg, uint8_t len);



static inline uint8_t /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__Packet__maxPayloadLength(void );



static void */*SerialActiveMessageC.AM*/SerialActiveMessageP__0__Packet__getPayload(message_t *msg, uint8_t len);
#line 132
static am_addr_t /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMPacket__destination(message_t *amsg);









static inline void /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMPacket__setDestination(message_t *amsg, am_addr_t addr);
#line 156
static am_id_t /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMPacket__type(message_t *amsg);




static inline void /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMPacket__setType(message_t *amsg, am_id_t type);
# 113 "/home/csilab/tinyos/tos/interfaces/SplitControl.nc"
static void SerialP__SplitControl__startDone(error_t error);
#line 138
static void SerialP__SplitControl__stopDone(error_t error);
# 67 "/home/csilab/tinyos/tos/interfaces/TaskBasic.nc"
static error_t SerialP__stopDoneTask__postTask(void );
# 95 "/home/csilab/tinyos/tos/interfaces/StdControl.nc"
static error_t SerialP__SerialControl__start(void );









static error_t SerialP__SerialControl__stop(void );
# 67 "/home/csilab/tinyos/tos/interfaces/TaskBasic.nc"
static error_t SerialP__RunTx__postTask(void );
# 49 "/home/csilab/tinyos/tos/lib/serial/SerialFlush.nc"
static void SerialP__SerialFlush__flush(void );
# 56 "/home/csilab/tinyos/tos/lib/serial/SerialFrameComm.nc"
static error_t SerialP__SerialFrameComm__putDelimiter(void );
#line 79
static void SerialP__SerialFrameComm__resetReceive(void );
#line 65
static error_t SerialP__SerialFrameComm__putData(uint8_t data);
# 67 "/home/csilab/tinyos/tos/interfaces/TaskBasic.nc"
static error_t SerialP__defaultSerialFlushTask__postTask(void );
# 81 "/home/csilab/tinyos/tos/lib/serial/SendBytePacket.nc"
static uint8_t SerialP__SendBytePacket__nextByte(void );









static void SerialP__SendBytePacket__sendCompleted(error_t error);
# 62 "/home/csilab/tinyos/tos/lib/serial/ReceiveBytePacket.nc"
static error_t SerialP__ReceiveBytePacket__startPacket(void );






static void SerialP__ReceiveBytePacket__byteReceived(uint8_t data);










static void SerialP__ReceiveBytePacket__endPacket(error_t result);
# 191 "/home/csilab/tinyos/tos/lib/serial/SerialP.nc"
enum SerialP____nesc_unnamed4316 {
#line 191
  SerialP__RunTx = 0U
};
#line 191
typedef int SerialP____nesc_sillytask_RunTx[SerialP__RunTx];
#line 322
enum SerialP____nesc_unnamed4317 {
#line 322
  SerialP__startDoneTask = 1U
};
#line 322
typedef int SerialP____nesc_sillytask_startDoneTask[SerialP__startDoneTask];









enum SerialP____nesc_unnamed4318 {
#line 332
  SerialP__stopDoneTask = 2U
};
#line 332
typedef int SerialP____nesc_sillytask_stopDoneTask[SerialP__stopDoneTask];








enum SerialP____nesc_unnamed4319 {
#line 341
  SerialP__defaultSerialFlushTask = 3U
};
#line 341
typedef int SerialP____nesc_sillytask_defaultSerialFlushTask[SerialP__defaultSerialFlushTask];
#line 81
enum SerialP____nesc_unnamed4320 {
  SerialP__RX_DATA_BUFFER_SIZE = 2, 
  SerialP__TX_DATA_BUFFER_SIZE = 4, 
  SerialP__SERIAL_MTU = 255, 
  SerialP__SERIAL_VERSION = 1, 
  SerialP__ACK_QUEUE_SIZE = 5
};

enum SerialP____nesc_unnamed4321 {
  SerialP__RXSTATE_NOSYNC, 
  SerialP__RXSTATE_PROTO, 
  SerialP__RXSTATE_TOKEN, 
  SerialP__RXSTATE_INFO, 
  SerialP__RXSTATE_INACTIVE
};

enum SerialP____nesc_unnamed4322 {
  SerialP__TXSTATE_IDLE, 
  SerialP__TXSTATE_PROTO, 
  SerialP__TXSTATE_SEQNO, 
  SerialP__TXSTATE_INFO, 
  SerialP__TXSTATE_FCS1, 
  SerialP__TXSTATE_FCS2, 
  SerialP__TXSTATE_ENDFLAG, 
  SerialP__TXSTATE_ENDWAIT, 
  SerialP__TXSTATE_FINISH, 
  SerialP__TXSTATE_ERROR, 
  SerialP__TXSTATE_INACTIVE
};





#line 111
typedef enum SerialP____nesc_unnamed4323 {
  SerialP__BUFFER_AVAILABLE, 
  SerialP__BUFFER_FILLING, 
  SerialP__BUFFER_COMPLETE
} SerialP__tx_data_buffer_states_t;

enum SerialP____nesc_unnamed4324 {
  SerialP__TX_ACK_INDEX = 0, 
  SerialP__TX_DATA_INDEX = 1, 
  SerialP__TX_BUFFER_COUNT = 2
};






#line 124
typedef struct SerialP____nesc_unnamed4325 {
  uint8_t writePtr;
  uint8_t readPtr;
  uint8_t buf[SerialP__RX_DATA_BUFFER_SIZE + 1];
} SerialP__rx_buf_t;




#line 130
typedef struct SerialP____nesc_unnamed4326 {
  uint8_t state;
  uint8_t buf;
} SerialP__tx_buf_t;





#line 135
typedef struct SerialP____nesc_unnamed4327 {
  uint8_t writePtr;
  uint8_t readPtr;
  uint8_t buf[SerialP__ACK_QUEUE_SIZE + 1];
} SerialP__ack_queue_t;



SerialP__rx_buf_t SerialP__rxBuf;
SerialP__tx_buf_t SerialP__txBuf[SerialP__TX_BUFFER_COUNT];



uint8_t SerialP__rxState;
uint8_t SerialP__rxByteCnt;
uint8_t SerialP__rxProto;
uint8_t SerialP__rxSeqno;
uint16_t SerialP__rxCRC;



uint8_t SerialP__txState;
uint8_t SerialP__txByteCnt;
uint8_t SerialP__txProto;
uint8_t SerialP__txSeqno;
uint16_t SerialP__txCRC;
uint8_t SerialP__txPending;
uint8_t SerialP__txIndex;


SerialP__ack_queue_t SerialP__ackQ;

bool SerialP__offPending = FALSE;



static __inline void SerialP__txInit(void );
static __inline void SerialP__rxInit(void );
static __inline void SerialP__ackInit(void );

static __inline bool SerialP__ack_queue_is_full(void );
static __inline bool SerialP__ack_queue_is_empty(void );
static __inline void SerialP__ack_queue_push(uint8_t token);
static __inline uint8_t SerialP__ack_queue_top(void );
static inline uint8_t SerialP__ack_queue_pop(void );




static __inline void SerialP__rx_buffer_push(uint8_t data);
static __inline uint8_t SerialP__rx_buffer_top(void );
static __inline uint8_t SerialP__rx_buffer_pop(void );
static __inline uint16_t SerialP__rx_current_crc(void );

static void SerialP__rx_state_machine(bool isDelimeter, uint8_t data);
static void SerialP__MaybeScheduleTx(void );




static __inline void SerialP__txInit(void );
#line 207
static __inline void SerialP__rxInit(void );








static __inline void SerialP__ackInit(void );



static inline error_t SerialP__Init__init(void );
#line 234
static __inline bool SerialP__ack_queue_is_full(void );









static __inline bool SerialP__ack_queue_is_empty(void );





static __inline void SerialP__ack_queue_push(uint8_t token);









static __inline uint8_t SerialP__ack_queue_top(void );









static inline uint8_t SerialP__ack_queue_pop(void );
#line 297
static __inline void SerialP__rx_buffer_push(uint8_t data);



static __inline uint8_t SerialP__rx_buffer_top(void );



static __inline uint8_t SerialP__rx_buffer_pop(void );





static __inline uint16_t SerialP__rx_current_crc(void );










static inline void SerialP__startDoneTask__runTask(void );









static inline void SerialP__stopDoneTask__runTask(void );



static inline void SerialP__SerialFlush__flushDone(void );




static inline void SerialP__defaultSerialFlushTask__runTask(void );


static inline void SerialP__SerialFlush__default__flush(void );
#line 357
static void SerialP__testOff(void );
#line 394
static inline void SerialP__SerialFrameComm__delimiterReceived(void );


static inline void SerialP__SerialFrameComm__dataReceived(uint8_t data);



static inline bool SerialP__valid_rx_proto(uint8_t proto);










static void SerialP__rx_state_machine(bool isDelimeter, uint8_t data);
#line 518
static void SerialP__MaybeScheduleTx(void );










static inline error_t SerialP__SendBytePacket__completeSend(void );








static inline error_t SerialP__SendBytePacket__startSend(uint8_t b);
#line 559
static inline void SerialP__RunTx__runTask(void );
#line 668
static inline void SerialP__SerialFrameComm__putDone(void );
#line 748
static inline void SerialP__SplitControl__default__startDone(error_t err);
static inline void SerialP__SplitControl__default__stopDone(error_t err);
# 67 "/home/csilab/tinyos/tos/interfaces/TaskBasic.nc"
static error_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTask__postTask(void );
# 100 "/home/csilab/tinyos/tos/interfaces/Send.nc"
static void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__Send__sendDone(
# 51 "/home/csilab/tinyos/tos/lib/serial/SerialDispatcherP.nc"
uart_id_t arg_0x7f7fed95c9c0, 
# 96 "/home/csilab/tinyos/tos/interfaces/Send.nc"
message_t * msg, 



error_t error);
# 67 "/home/csilab/tinyos/tos/interfaces/TaskBasic.nc"
static error_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__signalSendDone__postTask(void );
# 78 "/home/csilab/tinyos/tos/interfaces/Receive.nc"
static 
#line 74
message_t * 



/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__Receive__receive(
# 50 "/home/csilab/tinyos/tos/lib/serial/SerialDispatcherP.nc"
uart_id_t arg_0x7f7fed95fda0, 
# 71 "/home/csilab/tinyos/tos/interfaces/Receive.nc"
message_t * msg, 
void * payload, 





uint8_t len);
# 31 "/home/csilab/tinyos/tos/lib/serial/SerialPacketInfo.nc"
static uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__upperLength(
# 54 "/home/csilab/tinyos/tos/lib/serial/SerialDispatcherP.nc"
uart_id_t arg_0x7f7fed95be30, 
# 31 "/home/csilab/tinyos/tos/lib/serial/SerialPacketInfo.nc"
message_t *msg, uint8_t dataLinkLen);
#line 15
static uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__offset(
# 54 "/home/csilab/tinyos/tos/lib/serial/SerialDispatcherP.nc"
uart_id_t arg_0x7f7fed95be30);
# 23 "/home/csilab/tinyos/tos/lib/serial/SerialPacketInfo.nc"
static uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__dataLinkLength(
# 54 "/home/csilab/tinyos/tos/lib/serial/SerialDispatcherP.nc"
uart_id_t arg_0x7f7fed95be30, 
# 23 "/home/csilab/tinyos/tos/lib/serial/SerialPacketInfo.nc"
message_t *msg, uint8_t upperLen);
# 71 "/home/csilab/tinyos/tos/lib/serial/SendBytePacket.nc"
static error_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__SendBytePacket__completeSend(void );
#line 62
static error_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__SendBytePacket__startSend(uint8_t first_byte);
# 158 "/home/csilab/tinyos/tos/lib/serial/SerialDispatcherP.nc"
enum /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0____nesc_unnamed4328 {
#line 158
  SerialDispatcherP__0__signalSendDone = 4U
};
#line 158
typedef int /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0____nesc_sillytask_signalSendDone[/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__signalSendDone];
#line 275
enum /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0____nesc_unnamed4329 {
#line 275
  SerialDispatcherP__0__receiveTask = 5U
};
#line 275
typedef int /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0____nesc_sillytask_receiveTask[/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTask];
#line 66
#line 62
typedef enum /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0____nesc_unnamed4330 {
  SerialDispatcherP__0__SEND_STATE_IDLE = 0, 
  SerialDispatcherP__0__SEND_STATE_BEGIN = 1, 
  SerialDispatcherP__0__SEND_STATE_DATA = 2
} /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__send_state_t;

enum /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0____nesc_unnamed4331 {
  SerialDispatcherP__0__RECV_STATE_IDLE = 0, 
  SerialDispatcherP__0__RECV_STATE_BEGIN = 1, 
  SerialDispatcherP__0__RECV_STATE_DATA = 2
};






#line 74
typedef struct /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0____nesc_unnamed4332 {
  uint8_t which : 1;
  uint8_t bufZeroLocked : 1;
  uint8_t bufOneLocked : 1;
  uint8_t state : 2;
} /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__recv_state_t;



/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__recv_state_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveState = { 0, 0, 0, /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__RECV_STATE_IDLE };
uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__recvType = TOS_SERIAL_UNKNOWN_ID;
uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__recvIndex = 0;


message_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__messages[2];
message_t * /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__messagePtrs[2] = { &/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__messages[0], &/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__messages[1] };




uint8_t * /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveBuffer = (uint8_t * )&/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__messages[0];

uint8_t * /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendBuffer = (void *)0;
/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__send_state_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendState = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__SEND_STATE_IDLE;
uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendLen = 0;
uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendIndex = 0;
error_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendError = SUCCESS;
bool /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendCancelled = FALSE;
uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendId = 0;


uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTaskPending = FALSE;
uart_id_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTaskType = 0;
uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTaskWhich;
message_t * /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTaskBuf = (void *)0;
uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTaskSize = 0;

static inline error_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__Send__send(uint8_t id, message_t *msg, uint8_t len);
#line 158
static inline void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__signalSendDone__runTask(void );
#line 178
static inline uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__SendBytePacket__nextByte(void );
#line 194
static inline void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__SendBytePacket__sendCompleted(error_t error);




static inline bool /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__isCurrentBufferLocked(void );



static inline void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__lockCurrentBuffer(void );








static inline void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__unlockBuffer(uint8_t which);








static inline void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveBufferSwap(void );




static inline error_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__ReceiveBytePacket__startPacket(void );
#line 244
static inline void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__ReceiveBytePacket__byteReceived(uint8_t b);
#line 275
static inline void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTask__runTask(void );
#line 296
static void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__ReceiveBytePacket__endPacket(error_t result);
#line 358
static inline uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__default__offset(uart_id_t id);


static inline uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__default__dataLinkLength(uart_id_t id, message_t *msg, 
uint8_t upperLen);


static inline uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__default__upperLength(uart_id_t id, message_t *msg, 
uint8_t dataLinkLen);




static inline message_t */*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__Receive__default__receive(uart_id_t idxxx, message_t *msg, 
void *payload, 
uint8_t len);


static inline void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__Send__default__sendDone(uart_id_t idxxx, message_t *msg, error_t error);
# 48 "/home/csilab/tinyos/tos/interfaces/UartStream.nc"
static error_t HdlcTranslateC__UartStream__send(
#line 44
uint8_t * buf, 



uint16_t len);
# 94 "/home/csilab/tinyos/tos/lib/serial/SerialFrameComm.nc"
static void HdlcTranslateC__SerialFrameComm__dataReceived(uint8_t data);





static void HdlcTranslateC__SerialFrameComm__putDone(void );
#line 85
static void HdlcTranslateC__SerialFrameComm__delimiterReceived(void );
# 59 "/home/csilab/tinyos/tos/lib/serial/HdlcTranslateC.nc"
#line 56
typedef struct HdlcTranslateC____nesc_unnamed4333 {
  uint8_t sendEscape : 1;
  uint8_t receiveEscape : 1;
} HdlcTranslateC__HdlcState;


HdlcTranslateC__HdlcState HdlcTranslateC__state = { 0, 0 };
uint8_t HdlcTranslateC__txTemp;
uint8_t HdlcTranslateC__m_data;


static inline void HdlcTranslateC__SerialFrameComm__resetReceive(void );





static inline void HdlcTranslateC__UartStream__receivedByte(uint8_t data);
#line 98
static error_t HdlcTranslateC__SerialFrameComm__putDelimiter(void );







static error_t HdlcTranslateC__SerialFrameComm__putData(uint8_t data);
#line 118
static void HdlcTranslateC__UartStream__sendDone(uint8_t *buf, uint16_t len, 
error_t error);
#line 132
static inline void HdlcTranslateC__UartStream__receiveDone(uint8_t *buf, uint16_t len, error_t error);
# 39 "/home/csilab/tinyos/tos/chips/msp430/usart/Msp430UartConfigure.nc"
static msp430_uart_union_config_t */*Msp430Uart1P.UartP*/Msp430UartP__0__Msp430UartConfigure__getConfig(
# 49 "/home/csilab/tinyos/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x7f7fed849d40);
# 97 "/home/csilab/tinyos/tos/chips/msp430/usart/HplMsp430Usart.nc"
static void /*Msp430Uart1P.UartP*/Msp430UartP__0__Usart__resetUsart(bool reset);
#line 179
static void /*Msp430Uart1P.UartP*/Msp430UartP__0__Usart__disableIntr(void );

static void /*Msp430Uart1P.UartP*/Msp430UartP__0__Usart__enableTxIntr(void );
#line 178
static void /*Msp430Uart1P.UartP*/Msp430UartP__0__Usart__disableTxIntr(void );



static void /*Msp430Uart1P.UartP*/Msp430UartP__0__Usart__enableIntr(void );




static bool /*Msp430Uart1P.UartP*/Msp430UartP__0__Usart__isTxIntrPending(void );
#line 224
static void /*Msp430Uart1P.UartP*/Msp430UartP__0__Usart__tx(uint8_t data);
#line 128
static void /*Msp430Uart1P.UartP*/Msp430UartP__0__Usart__disableUart(void );
#line 174
static void /*Msp430Uart1P.UartP*/Msp430UartP__0__Usart__setModeUart(msp430_uart_union_config_t *config);
#line 202
static void /*Msp430Uart1P.UartP*/Msp430UartP__0__Usart__clrTxIntr(void );
# 79 "/home/csilab/tinyos/tos/interfaces/UartStream.nc"
static void /*Msp430Uart1P.UartP*/Msp430UartP__0__UartStream__receivedByte(
# 45 "/home/csilab/tinyos/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x7f7fed84c4c0, 
# 79 "/home/csilab/tinyos/tos/interfaces/UartStream.nc"
uint8_t byte);
#line 99
static void /*Msp430Uart1P.UartP*/Msp430UartP__0__UartStream__receiveDone(
# 45 "/home/csilab/tinyos/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x7f7fed84c4c0, 
# 95 "/home/csilab/tinyos/tos/interfaces/UartStream.nc"
uint8_t * buf, 



uint16_t len, error_t error);
#line 57
static void /*Msp430Uart1P.UartP*/Msp430UartP__0__UartStream__sendDone(
# 45 "/home/csilab/tinyos/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x7f7fed84c4c0, 
# 53 "/home/csilab/tinyos/tos/interfaces/UartStream.nc"
uint8_t * buf, 



uint16_t len, error_t error);
# 120 "/home/csilab/tinyos/tos/interfaces/Resource.nc"
static error_t /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartResource__release(
# 48 "/home/csilab/tinyos/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x7f7fed84a980);
# 97 "/home/csilab/tinyos/tos/interfaces/Resource.nc"
static error_t /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartResource__immediateRequest(
# 48 "/home/csilab/tinyos/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x7f7fed84a980);
# 128 "/home/csilab/tinyos/tos/interfaces/Resource.nc"
static bool /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartResource__isOwner(
# 48 "/home/csilab/tinyos/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x7f7fed84a980);
# 102 "/home/csilab/tinyos/tos/interfaces/Resource.nc"
static void /*Msp430Uart1P.UartP*/Msp430UartP__0__Resource__granted(
# 43 "/home/csilab/tinyos/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x7f7fed84f380);
#line 59
uint16_t /*Msp430Uart1P.UartP*/Msp430UartP__0__m_tx_len;
#line 59
uint16_t /*Msp430Uart1P.UartP*/Msp430UartP__0__m_rx_len;
uint8_t * /*Msp430Uart1P.UartP*/Msp430UartP__0__m_tx_buf;
#line 60
uint8_t * /*Msp430Uart1P.UartP*/Msp430UartP__0__m_rx_buf;
uint16_t /*Msp430Uart1P.UartP*/Msp430UartP__0__m_tx_pos;
#line 61
uint16_t /*Msp430Uart1P.UartP*/Msp430UartP__0__m_rx_pos;
uint8_t /*Msp430Uart1P.UartP*/Msp430UartP__0__m_byte_time;
uint8_t /*Msp430Uart1P.UartP*/Msp430UartP__0__current_owner;

static inline error_t /*Msp430Uart1P.UartP*/Msp430UartP__0__Resource__immediateRequest(uint8_t id);
#line 77
static inline error_t /*Msp430Uart1P.UartP*/Msp430UartP__0__Resource__release(uint8_t id);







static void /*Msp430Uart1P.UartP*/Msp430UartP__0__ResourceConfigure__configure(uint8_t id);






static inline void /*Msp430Uart1P.UartP*/Msp430UartP__0__ResourceConfigure__unconfigure(uint8_t id);








static inline void /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartResource__granted(uint8_t id);
#line 134
static inline void /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartInterrupts__rxDone(uint8_t id, uint8_t data);
#line 147
static error_t /*Msp430Uart1P.UartP*/Msp430UartP__0__UartStream__send(uint8_t id, uint8_t *buf, uint16_t len);
#line 162
static inline void /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartInterrupts__txDone(uint8_t id);
#line 178
static inline error_t /*Msp430Uart1P.UartP*/Msp430UartP__0__UartByte__send(uint8_t id, uint8_t data);
#line 208
static inline void /*Msp430Uart1P.UartP*/Msp430UartP__0__Counter__overflow(void );

static inline bool /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartResource__default__isOwner(uint8_t id);

static inline error_t /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartResource__default__immediateRequest(uint8_t id);
static inline error_t /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartResource__default__release(uint8_t id);
static inline msp430_uart_union_config_t */*Msp430Uart1P.UartP*/Msp430UartP__0__Msp430UartConfigure__default__getConfig(uint8_t id);



static inline void /*Msp430Uart1P.UartP*/Msp430UartP__0__Resource__default__granted(uint8_t id);

static inline void /*Msp430Uart1P.UartP*/Msp430UartP__0__UartStream__default__sendDone(uint8_t id, uint8_t *buf, uint16_t len, error_t error);
static inline void /*Msp430Uart1P.UartP*/Msp430UartP__0__UartStream__default__receivedByte(uint8_t id, uint8_t byte);
static inline void /*Msp430Uart1P.UartP*/Msp430UartP__0__UartStream__default__receiveDone(uint8_t id, uint8_t *buf, uint16_t len, error_t error);
# 99 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void HplMsp430Usart1P__UCLK__selectIOFunc(void );
# 54 "/home/csilab/tinyos/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
static void HplMsp430Usart1P__Interrupts__rxDone(uint8_t data);
#line 49
static void HplMsp430Usart1P__Interrupts__txDone(void );
# 99 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void HplMsp430Usart1P__URXD__selectIOFunc(void );
#line 92
static void HplMsp430Usart1P__URXD__selectModuleFunc(void );






static void HplMsp430Usart1P__UTXD__selectIOFunc(void );
#line 92
static void HplMsp430Usart1P__UTXD__selectModuleFunc(void );






static void HplMsp430Usart1P__SOMI__selectIOFunc(void );
#line 99
static void HplMsp430Usart1P__SIMO__selectIOFunc(void );
# 87 "/home/csilab/tinyos/tos/chips/msp430/usart/HplMsp430Usart1P.nc"
static volatile uint8_t HplMsp430Usart1P__IE2 __asm ("0x0001");
static volatile uint8_t HplMsp430Usart1P__ME2 __asm ("0x0005");
static volatile uint8_t HplMsp430Usart1P__IFG2 __asm ("0x0003");
static volatile uint8_t HplMsp430Usart1P__U1TCTL __asm ("0x0079");
static volatile uint8_t HplMsp430Usart1P__U1RCTL __asm ("0x007A");
static volatile uint8_t HplMsp430Usart1P__U1TXBUF __asm ("0x007F");



void sig_UART1RX_VECTOR(void ) __attribute((wakeup)) __attribute((interrupt(0x0006)))  ;




void sig_UART1TX_VECTOR(void ) __attribute((wakeup)) __attribute((interrupt(0x0004)))  ;



static inline error_t HplMsp430Usart1P__AsyncStdControl__start(void );



static inline error_t HplMsp430Usart1P__AsyncStdControl__stop(void );
#line 140
static inline void HplMsp430Usart1P__Usart__setUbr(uint16_t control);










static inline void HplMsp430Usart1P__Usart__setUmctl(uint8_t control);







static inline void HplMsp430Usart1P__Usart__resetUsart(bool reset);
#line 203
static inline void HplMsp430Usart1P__Usart__enableUart(void );







static void HplMsp430Usart1P__Usart__disableUart(void );








static inline void HplMsp430Usart1P__Usart__enableUartTx(void );




static inline void HplMsp430Usart1P__Usart__disableUartTx(void );





static inline void HplMsp430Usart1P__Usart__enableUartRx(void );




static inline void HplMsp430Usart1P__Usart__disableUartRx(void );
#line 251
static void HplMsp430Usart1P__Usart__disableSpi(void );
#line 283
static inline void HplMsp430Usart1P__configUart(msp430_uart_union_config_t *config);









static inline void HplMsp430Usart1P__Usart__setModeUart(msp430_uart_union_config_t *config);
#line 318
static inline bool HplMsp430Usart1P__Usart__isTxIntrPending(void );
#line 339
static inline void HplMsp430Usart1P__Usart__clrTxIntr(void );







static inline void HplMsp430Usart1P__Usart__clrIntr(void );







static inline void HplMsp430Usart1P__Usart__disableTxIntr(void );



static inline void HplMsp430Usart1P__Usart__disableIntr(void );










static inline void HplMsp430Usart1P__Usart__enableTxIntr(void );






static inline void HplMsp430Usart1P__Usart__enableIntr(void );






static inline void HplMsp430Usart1P__Usart__tx(uint8_t data);
# 59 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline uint8_t /*HplMsp430GeneralIOC.P10*/HplMsp430GeneralIOP__0__IO__getRaw(void );
static inline bool /*HplMsp430GeneralIOC.P10*/HplMsp430GeneralIOP__0__IO__get(void );
#line 59
static inline uint8_t /*HplMsp430GeneralIOC.P13*/HplMsp430GeneralIOP__3__IO__getRaw(void );
static inline bool /*HplMsp430GeneralIOC.P13*/HplMsp430GeneralIOP__3__IO__get(void );
#line 59
static inline uint8_t /*HplMsp430GeneralIOC.P14*/HplMsp430GeneralIOP__4__IO__getRaw(void );
static inline bool /*HplMsp430GeneralIOC.P14*/HplMsp430GeneralIOP__4__IO__get(void );
static inline void /*HplMsp430GeneralIOC.P14*/HplMsp430GeneralIOP__4__IO__makeInput(void );
#line 57
static inline void /*HplMsp430GeneralIOC.P24*/HplMsp430GeneralIOP__12__IO__clr(void );

static inline uint8_t /*HplMsp430GeneralIOC.P24*/HplMsp430GeneralIOP__12__IO__getRaw(void );
static inline bool /*HplMsp430GeneralIOC.P24*/HplMsp430GeneralIOP__12__IO__get(void );
static inline void /*HplMsp430GeneralIOC.P24*/HplMsp430GeneralIOP__12__IO__makeInput(void );

static inline void /*HplMsp430GeneralIOC.P24*/HplMsp430GeneralIOP__12__IO__makeOutput(void );

static inline void /*HplMsp430GeneralIOC.P31*/HplMsp430GeneralIOP__17__IO__selectModuleFunc(void );

static inline void /*HplMsp430GeneralIOC.P31*/HplMsp430GeneralIOP__17__IO__selectIOFunc(void );
#line 65
static inline void /*HplMsp430GeneralIOC.P32*/HplMsp430GeneralIOP__18__IO__selectModuleFunc(void );

static inline void /*HplMsp430GeneralIOC.P32*/HplMsp430GeneralIOP__18__IO__selectIOFunc(void );
#line 65
static inline void /*HplMsp430GeneralIOC.P33*/HplMsp430GeneralIOP__19__IO__selectModuleFunc(void );

static inline void /*HplMsp430GeneralIOC.P33*/HplMsp430GeneralIOP__19__IO__selectIOFunc(void );
#line 67
static inline void /*HplMsp430GeneralIOC.P34*/HplMsp430GeneralIOP__20__IO__selectIOFunc(void );
#line 67
static inline void /*HplMsp430GeneralIOC.P35*/HplMsp430GeneralIOP__21__IO__selectIOFunc(void );
#line 65
static inline void /*HplMsp430GeneralIOC.P36*/HplMsp430GeneralIOP__22__IO__selectModuleFunc(void );

static inline void /*HplMsp430GeneralIOC.P36*/HplMsp430GeneralIOP__22__IO__selectIOFunc(void );
#line 65
static inline void /*HplMsp430GeneralIOC.P37*/HplMsp430GeneralIOP__23__IO__selectModuleFunc(void );

static inline void /*HplMsp430GeneralIOC.P37*/HplMsp430GeneralIOP__23__IO__selectIOFunc(void );
#line 59
static inline uint8_t /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__getRaw(void );
static inline bool /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__get(void );
static inline void /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__makeInput(void );



static inline void /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__selectModuleFunc(void );

static inline void /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__selectIOFunc(void );
#line 56
static void /*HplMsp430GeneralIOC.P42*/HplMsp430GeneralIOP__26__IO__set(void );
static void /*HplMsp430GeneralIOC.P42*/HplMsp430GeneralIOP__26__IO__clr(void );





static inline void /*HplMsp430GeneralIOC.P42*/HplMsp430GeneralIOP__26__IO__makeOutput(void );
#line 56
static inline void /*HplMsp430GeneralIOC.P45*/HplMsp430GeneralIOP__29__IO__set(void );
static inline void /*HplMsp430GeneralIOC.P45*/HplMsp430GeneralIOP__29__IO__clr(void );





static inline void /*HplMsp430GeneralIOC.P45*/HplMsp430GeneralIOP__29__IO__makeOutput(void );
#line 56
static void /*HplMsp430GeneralIOC.P46*/HplMsp430GeneralIOP__30__IO__set(void );
static void /*HplMsp430GeneralIOC.P46*/HplMsp430GeneralIOP__30__IO__clr(void );





static inline void /*HplMsp430GeneralIOC.P46*/HplMsp430GeneralIOP__30__IO__makeOutput(void );



static inline void /*HplMsp430GeneralIOC.P51*/HplMsp430GeneralIOP__33__IO__selectIOFunc(void );
#line 67
static inline void /*HplMsp430GeneralIOC.P52*/HplMsp430GeneralIOP__34__IO__selectIOFunc(void );
#line 67
static inline void /*HplMsp430GeneralIOC.P53*/HplMsp430GeneralIOP__35__IO__selectIOFunc(void );
#line 56
static inline void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__set(void );






static inline void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__makeOutput(void );
#line 56
static inline void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__set(void );






static inline void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__makeOutput(void );
#line 56
static inline void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__set(void );






static inline void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__makeOutput(void );
# 62 "/home/csilab/tinyos/tos/interfaces/Init.nc"
static error_t PlatformP__MoteInit__init(void );
#line 62
static error_t PlatformP__MoteClockInit__init(void );
#line 62
static error_t PlatformP__LedsInit__init(void );
# 10 "/home/csilab/tinyos/tos/platforms/telosa/PlatformP.nc"
static inline error_t PlatformP__Init__init(void );
# 6 "/home/csilab/tinyos/tos/platforms/telosb/MotePlatformC.nc"
static __inline void MotePlatformC__uwait(uint16_t u);




static __inline void MotePlatformC__TOSH_wait(void );




static void MotePlatformC__TOSH_FLASH_M25P_DP_bit(bool set);










static inline void MotePlatformC__TOSH_FLASH_M25P_DP(void );
#line 56
static inline error_t MotePlatformC__Init__init(void );
# 43 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430ClockInit.nc"
static void Msp430ClockP__Msp430ClockInit__initTimerB(void );
#line 42
static void Msp430ClockP__Msp430ClockInit__initTimerA(void );
#line 40
static void Msp430ClockP__Msp430ClockInit__setupDcoCalibrate(void );
static void Msp430ClockP__Msp430ClockInit__initClocks(void );
# 51 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430ClockP.nc"
static volatile uint8_t Msp430ClockP__IE1 __asm ("0x0000");
static volatile uint16_t Msp430ClockP__TACTL __asm ("0x0160");
static volatile uint16_t Msp430ClockP__TAIV __asm ("0x012E");
static volatile uint16_t Msp430ClockP__TBCTL __asm ("0x0180");
static volatile uint16_t Msp430ClockP__TBIV __asm ("0x011E");

enum Msp430ClockP____nesc_unnamed4334 {

  Msp430ClockP__ACLK_CALIB_PERIOD = 8, 
  Msp430ClockP__TARGET_DCO_DELTA = 4096 / 32 * Msp430ClockP__ACLK_CALIB_PERIOD
};

static inline mcu_power_t Msp430ClockP__McuPowerOverride__lowestState(void );



static inline void Msp430ClockP__Msp430ClockInit__defaultSetupDcoCalibrate(void );
#line 79
static inline void Msp430ClockP__Msp430ClockInit__defaultInitClocks(void );
#line 100
static inline void Msp430ClockP__Msp430ClockInit__defaultInitTimerA(void );
#line 115
static inline void Msp430ClockP__Msp430ClockInit__defaultInitTimerB(void );
#line 130
static inline void Msp430ClockP__Msp430ClockInit__default__setupDcoCalibrate(void );




static inline void Msp430ClockP__Msp430ClockInit__default__initClocks(void );




static inline void Msp430ClockP__Msp430ClockInit__default__initTimerA(void );




static inline void Msp430ClockP__Msp430ClockInit__default__initTimerB(void );





static inline void Msp430ClockP__startTimerA(void );
#line 163
static inline void Msp430ClockP__startTimerB(void );
#line 175
static void Msp430ClockP__set_dco_calib(int calib);





static inline uint16_t Msp430ClockP__test_calib_busywait_delta(int calib);
#line 204
static inline void Msp430ClockP__busyCalibrateDco(void );
#line 229
static inline error_t Msp430ClockP__Init__init(void );
# 39 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__fired(
# 51 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430TimerP.nc"
uint8_t arg_0x7f7fed4c58b0);
# 48 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__overflow(void );
# 62 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__get(void );
#line 126
static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__VectorTimerX0__fired(void );




static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__VectorTimerX1__fired(void );





static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Overflow__fired(void );








static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__default__fired(uint8_t n);
# 39 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__fired(
# 51 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430TimerP.nc"
uint8_t arg_0x7f7fed4c58b0);
# 48 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__overflow(void );
# 62 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430TimerP.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__get(void );
#line 81
static inline bool /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__isOverflowPending(void );
#line 126
static inline void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__VectorTimerX0__fired(void );




static inline void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__VectorTimerX1__fired(void );





static inline void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Overflow__fired(void );








static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__default__fired(uint8_t n);
# 86 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__captured(uint16_t time);
# 45 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Compare__fired(void );
# 55 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__cc_t;


static inline /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__cc_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__int2CC(uint16_t x)  ;
#line 85
static inline /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__cc_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Control__getControl(void );
#line 150
static inline uint16_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__getEvent(void );
#line 180
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__default__captured(uint16_t n);



static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Compare__default__fired(void );



static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Timer__overflow(void );
# 86 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__captured(uint16_t time);
# 45 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Compare__fired(void );
# 55 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__cc_t;


static inline /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__cc_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__int2CC(uint16_t x)  ;
#line 85
static inline /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__cc_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Control__getControl(void );
#line 150
static inline uint16_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__getEvent(void );
#line 180
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__default__captured(uint16_t n);



static inline void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Compare__default__fired(void );



static inline void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Timer__overflow(void );
# 86 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__captured(uint16_t time);
# 45 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Compare__fired(void );
# 55 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__cc_t;


static inline /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__cc_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__int2CC(uint16_t x)  ;
#line 85
static inline /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__cc_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Control__getControl(void );
#line 150
static inline uint16_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__getEvent(void );
#line 180
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__default__captured(uint16_t n);



static inline void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Compare__default__fired(void );



static inline void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Timer__overflow(void );
# 86 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__captured(uint16_t time);
# 45 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__fired(void );
# 45 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430Timer.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Timer__get(void );
# 55 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t;

static inline uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__CC2int(/*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t x)  ;
static inline /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__int2CC(uint16_t x)  ;

static inline uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__compareControl(void );
#line 85
static inline /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__getControl(void );









static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__clearPendingInterrupt(void );









static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__setControlAsCompare(void );
#line 130
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__enableEvents(void );




static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__disableEvents(void );
#line 150
static inline uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__getEvent(void );




static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__setEvent(uint16_t x);









static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__setEventFromNow(uint16_t x);
#line 180
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__default__captured(uint16_t n);







static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Timer__overflow(void );
# 86 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__captured(uint16_t time);
# 45 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Compare__fired(void );
# 55 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__cc_t;

static inline uint16_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__CC2int(/*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__cc_t x)  ;
static inline /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__cc_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__int2CC(uint16_t x)  ;
#line 72
static inline uint16_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__captureControl(uint8_t l_cm);
#line 85
static inline /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__cc_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__getControl(void );









static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__clearPendingInterrupt(void );
#line 110
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__setControlAsCapture(uint8_t cm);
#line 130
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__enableEvents(void );




static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__disableEvents(void );
#line 150
static inline uint16_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__getEvent(void );
#line 175
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__clearOverflow(void );




static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Event__fired(void );
#line 192
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Compare__default__fired(void );



static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Timer__overflow(void );
# 86 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__captured(uint16_t time);
# 45 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__fired(void );
# 45 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430Timer.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Timer__get(void );
# 55 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__cc_t;

static inline uint16_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__CC2int(/*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__cc_t x)  ;
static inline /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__cc_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__int2CC(uint16_t x)  ;

static inline uint16_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__compareControl(void );
#line 85
static inline /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__cc_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__getControl(void );









static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__clearPendingInterrupt(void );









static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__setControlAsCompare(void );
#line 130
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__enableEvents(void );




static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__disableEvents(void );
#line 150
static inline uint16_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__getEvent(void );




static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__setEvent(uint16_t x);









static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__setEventFromNow(uint16_t x);
#line 180
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__default__captured(uint16_t n);







static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Timer__overflow(void );
# 86 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__captured(uint16_t time);
# 45 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Compare__fired(void );
# 45 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430Timer.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Timer__get(void );
# 55 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__cc_t;


static inline /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__cc_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__int2CC(uint16_t x)  ;
#line 85
static inline /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__cc_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Control__getControl(void );









static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Control__clearPendingInterrupt(void );
#line 130
static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Control__enableEvents(void );




static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Control__disableEvents(void );
#line 150
static inline uint16_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__getEvent(void );




static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Compare__setEvent(uint16_t x);









static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Compare__setEventFromNow(uint16_t x);
#line 180
static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__default__captured(uint16_t n);







static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Timer__overflow(void );
# 86 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__captured(uint16_t time);
# 45 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Compare__fired(void );
# 45 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430Timer.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Timer__get(void );
# 55 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__cc_t;


static inline /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__cc_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__int2CC(uint16_t x)  ;
#line 85
static inline /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__cc_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Control__getControl(void );









static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Control__clearPendingInterrupt(void );
#line 130
static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Control__enableEvents(void );




static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Control__disableEvents(void );
#line 150
static inline uint16_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__getEvent(void );




static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Compare__setEvent(uint16_t x);









static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Compare__setEventFromNow(uint16_t x);
#line 180
static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__default__captured(uint16_t n);







static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Timer__overflow(void );
# 86 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__captured(uint16_t time);
# 45 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Compare__fired(void );
# 45 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430Timer.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Timer__get(void );
# 55 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__cc_t;


static inline /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__cc_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__int2CC(uint16_t x)  ;
#line 85
static inline /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__cc_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Control__getControl(void );









static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Control__clearPendingInterrupt(void );
#line 130
static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Control__enableEvents(void );




static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Control__disableEvents(void );
#line 150
static inline uint16_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__getEvent(void );




static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Compare__setEvent(uint16_t x);









static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Compare__setEventFromNow(uint16_t x);
#line 180
static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__default__captured(uint16_t n);







static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Timer__overflow(void );
# 86 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__captured(uint16_t time);
# 45 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Compare__fired(void );
# 55 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__cc_t;


static inline /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__cc_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__int2CC(uint16_t x)  ;
#line 85
static inline /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__cc_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Control__getControl(void );
#line 135
static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Control__disableEvents(void );
#line 150
static inline uint16_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__getEvent(void );
#line 180
static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__default__captured(uint16_t n);







static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Timer__overflow(void );
# 39 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void Msp430TimerCommonP__VectorTimerB1__fired(void );
#line 39
static void Msp430TimerCommonP__VectorTimerA0__fired(void );
#line 39
static void Msp430TimerCommonP__VectorTimerA1__fired(void );
#line 39
static void Msp430TimerCommonP__VectorTimerB0__fired(void );
# 11 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430TimerCommonP.nc"
void sig_TIMERA0_VECTOR(void ) __attribute((wakeup)) __attribute((interrupt(0x000C)))  ;
void sig_TIMERA1_VECTOR(void ) __attribute((wakeup)) __attribute((interrupt(0x000A)))  ;
void sig_TIMERB0_VECTOR(void ) __attribute((wakeup)) __attribute((interrupt(0x001A)))  ;
void sig_TIMERB1_VECTOR(void ) __attribute((wakeup)) __attribute((interrupt(0x0018)))  ;
# 62 "/home/csilab/tinyos/tos/interfaces/McuPowerOverride.nc"
static mcu_power_t McuSleepC__McuPowerOverride__lowestState(void );
# 59 "/home/csilab/tinyos/tos/chips/msp430/McuSleepC.nc"
bool McuSleepC__dirty = TRUE;
mcu_power_t McuSleepC__powerState = MSP430_POWER_ACTIVE;




const uint16_t McuSleepC__msp430PowerBits[MSP430_POWER_LPM4 + 1] = { 
0, 
0x0010, 
0x0040 + 0x0010, 
0x0080 + 0x0010, 
0x0080 + 0x0040 + 0x0010, 
0x0080 + 0x0040 + 0x0020 + 0x0010 };


static inline mcu_power_t McuSleepC__getPowerState(void );
#line 112
static inline void McuSleepC__computePowerState(void );




static inline void McuSleepC__McuSleep__sleep(void );
# 45 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430Timer.nc"
static uint16_t /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__get(void );
static bool /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__isOverflowPending(void );
# 82 "/home/csilab/tinyos/tos/lib/timer/Counter.nc"
static void /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__overflow(void );
# 49 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430CounterC.nc"
static inline uint16_t /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__get(void );




static inline bool /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__isOverflowPending(void );









static inline void /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__overflow(void );
# 46 "/home/csilab/tinyos/tos/interfaces/GeneralIO.nc"
static void LedsP__Led0__makeOutput(void );
#line 40
static void LedsP__Led0__set(void );





static void LedsP__Led1__makeOutput(void );
#line 40
static void LedsP__Led1__set(void );





static void LedsP__Led2__makeOutput(void );
#line 40
static void LedsP__Led2__set(void );
# 56 "/home/csilab/tinyos/tos/system/LedsP.nc"
static inline error_t LedsP__Init__init(void );
# 85 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__HplGeneralIO__makeOutput(void );
#line 48
static void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__HplGeneralIO__set(void );
# 48 "/home/csilab/tinyos/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__set(void );





static inline void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__makeOutput(void );
# 85 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__HplGeneralIO__makeOutput(void );
#line 48
static void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__HplGeneralIO__set(void );
# 48 "/home/csilab/tinyos/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__set(void );





static inline void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__makeOutput(void );
# 85 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__HplGeneralIO__makeOutput(void );
#line 48
static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__HplGeneralIO__set(void );
# 48 "/home/csilab/tinyos/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__set(void );





static inline void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__makeOutput(void );
# 90 "/home/csilab/tinyos/tos/interfaces/ArbiterInfo.nc"
static bool /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__0__ArbiterInfo__inUse(void );







static uint8_t /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__0__ArbiterInfo__userId(void );
# 54 "/home/csilab/tinyos/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
static void /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__0__Interrupts__rxDone(
# 39 "/home/csilab/tinyos/tos/chips/msp430/usart/Msp430UsartShareP.nc"
uint8_t arg_0x7f7fed2b0600, 
# 54 "/home/csilab/tinyos/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
uint8_t data);
#line 49
static void /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__0__Interrupts__txDone(
# 39 "/home/csilab/tinyos/tos/chips/msp430/usart/Msp430UsartShareP.nc"
uint8_t arg_0x7f7fed2b0600);









static inline void /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__0__RawInterrupts__txDone(void );




static inline void /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__0__RawInterrupts__rxDone(uint8_t data);









static inline void /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__0__Interrupts__default__txDone(uint8_t id);
static inline void /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__0__Interrupts__default__rxDone(uint8_t id, uint8_t data);
# 62 "/home/csilab/tinyos/tos/interfaces/Init.nc"
static error_t RealMainP__SoftwareInit__init(void );
# 60 "/home/csilab/tinyos/tos/interfaces/Boot.nc"
static void RealMainP__Boot__booted(void );
# 62 "/home/csilab/tinyos/tos/interfaces/Init.nc"
static error_t RealMainP__PlatformInit__init(void );
# 57 "/home/csilab/tinyos/tos/interfaces/Scheduler.nc"
static void RealMainP__Scheduler__init(void );
#line 72
static void RealMainP__Scheduler__taskLoop(void );
#line 65
static bool RealMainP__Scheduler__runNextTask(void );
# 63 "/home/csilab/tinyos/tos/system/RealMainP.nc"
int main(void )   ;
# 75 "/home/csilab/tinyos/tos/interfaces/TaskBasic.nc"
static void SchedulerBasicP__TaskBasic__runTask(
# 56 "/home/csilab/tinyos/tos/system/SchedulerBasicP.nc"
uint8_t arg_0x7f7fedb6be60);
# 76 "/home/csilab/tinyos/tos/interfaces/McuSleep.nc"
static void SchedulerBasicP__McuSleep__sleep(void );
# 61 "/home/csilab/tinyos/tos/system/SchedulerBasicP.nc"
enum SchedulerBasicP____nesc_unnamed4335 {

  SchedulerBasicP__NUM_TASKS = 30U, 
  SchedulerBasicP__NO_TASK = 255
};

uint8_t SchedulerBasicP__m_head;
uint8_t SchedulerBasicP__m_tail;
uint8_t SchedulerBasicP__m_next[SchedulerBasicP__NUM_TASKS];








static __inline uint8_t SchedulerBasicP__popTask(void );
#line 97
static inline bool SchedulerBasicP__isWaiting(uint8_t id);




static inline bool SchedulerBasicP__pushTask(uint8_t id);
#line 124
static inline void SchedulerBasicP__Scheduler__init(void );









static bool SchedulerBasicP__Scheduler__runNextTask(void );
#line 149
static inline void SchedulerBasicP__Scheduler__taskLoop(void );
#line 170
static error_t SchedulerBasicP__TaskBasic__postTask(uint8_t id);




static void SchedulerBasicP__TaskBasic__default__runTask(uint8_t id);
# 49 "/home/csilab/tinyos/tos/system/FcfsResourceQueueC.nc"
enum /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__1____nesc_unnamed4336 {
#line 49
  FcfsResourceQueueC__1__NO_ENTRY = 0xFF
};
uint8_t /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__1__resQ[1U];
uint8_t /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__1__qHead = /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__1__NO_ENTRY;
uint8_t /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__1__qTail = /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__1__NO_ENTRY;

static inline error_t /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__1__Init__init(void );




static inline bool /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__1__FcfsQueue__isEmpty(void );







static inline resource_client_id_t /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__1__FcfsQueue__dequeue(void );
# 61 "/home/csilab/tinyos/tos/interfaces/ResourceRequested.nc"
static void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceRequested__immediateRequested(
# 55 "/home/csilab/tinyos/tos/system/ArbiterP.nc"
uint8_t arg_0x7f7fed25e840);
# 65 "/home/csilab/tinyos/tos/interfaces/ResourceConfigure.nc"
static void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__unconfigure(
# 60 "/home/csilab/tinyos/tos/system/ArbiterP.nc"
uint8_t arg_0x7f7fed25bc40);
# 59 "/home/csilab/tinyos/tos/interfaces/ResourceConfigure.nc"
static void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__configure(
# 60 "/home/csilab/tinyos/tos/system/ArbiterP.nc"
uint8_t arg_0x7f7fed25bc40);
# 53 "/home/csilab/tinyos/tos/interfaces/ResourceQueue.nc"
static bool /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__Queue__isEmpty(void );
#line 70
static resource_client_id_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__Queue__dequeue(void );
# 46 "/home/csilab/tinyos/tos/interfaces/ResourceDefaultOwner.nc"
static void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__granted(void );
#line 81
static void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__immediateRequested(void );
# 102 "/home/csilab/tinyos/tos/interfaces/Resource.nc"
static void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__Resource__granted(
# 54 "/home/csilab/tinyos/tos/system/ArbiterP.nc"
uint8_t arg_0x7f7fed25f4b0);
# 67 "/home/csilab/tinyos/tos/interfaces/TaskBasic.nc"
static error_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__grantedTask__postTask(void );
# 75 "/home/csilab/tinyos/tos/system/ArbiterP.nc"
enum /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0____nesc_unnamed4337 {
#line 75
  ArbiterP__0__grantedTask = 6U
};
#line 75
typedef int /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0____nesc_sillytask_grantedTask[/*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__grantedTask];
#line 67
enum /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0____nesc_unnamed4338 {
#line 67
  ArbiterP__0__RES_CONTROLLED, ArbiterP__0__RES_GRANTING, ArbiterP__0__RES_IMM_GRANTING, ArbiterP__0__RES_BUSY
};
#line 68
enum /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0____nesc_unnamed4339 {
#line 68
  ArbiterP__0__default_owner_id = 1U
};
#line 69
enum /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0____nesc_unnamed4340 {
#line 69
  ArbiterP__0__NO_RES = 0xFF
};
uint8_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__state = /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__RES_CONTROLLED;
uint8_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__resId = /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__default_owner_id;
uint8_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__reqResId;
#line 93
static error_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__Resource__immediateRequest(uint8_t id);
#line 111
static inline error_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__Resource__release(uint8_t id);
#line 133
static inline error_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__release(void );
#line 153
static bool /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ArbiterInfo__inUse(void );
#line 166
static uint8_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ArbiterInfo__userId(void );










static bool /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__Resource__isOwner(uint8_t id);
#line 190
static inline void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__grantedTask__runTask(void );
#line 202
static inline void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__Resource__default__granted(uint8_t id);



static inline void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceRequested__default__immediateRequested(uint8_t id);









static inline void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__default__configure(uint8_t id);

static inline void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__default__unconfigure(uint8_t id);
# 62 "/home/csilab/tinyos/tos/lib/power/PowerDownCleanup.nc"
static void /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__PowerDownCleanup__cleanup(void );
# 56 "/home/csilab/tinyos/tos/interfaces/ResourceDefaultOwner.nc"
static error_t /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__ResourceDefaultOwner__release(void );
# 95 "/home/csilab/tinyos/tos/interfaces/AsyncStdControl.nc"
static error_t /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__AsyncStdControl__start(void );









static error_t /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__AsyncStdControl__stop(void );
# 74 "/home/csilab/tinyos/tos/lib/power/AsyncPowerManagerP.nc"
static inline void /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__ResourceDefaultOwner__immediateRequested(void );




static inline void /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__ResourceDefaultOwner__granted(void );




static inline void /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__PowerDownCleanup__default__cleanup(void );
# 120 "/home/csilab/tinyos/tos/interfaces/Resource.nc"
static error_t TelosSerialP__Resource__release(void );
#line 97
static error_t TelosSerialP__Resource__immediateRequest(void );
# 8 "/home/csilab/tinyos/tos/platforms/telosa/TelosSerialP.nc"
msp430_uart_union_config_t TelosSerialP__msp430_uart_telos_config = { { .ubr = UBR_1MHZ_115200, .umctl = UMCTL_1MHZ_115200, .ssel = 0x02, .pena = 0, .pev = 0, .spb = 0, .clen = 1, .listen = 0, .mm = 0, .ckpl = 0, .urxse = 0, .urxeie = 1, .urxwie = 0, .utxe = 1, .urxe = 1 } };

static inline error_t TelosSerialP__StdControl__start(void );


static inline error_t TelosSerialP__StdControl__stop(void );



static inline void TelosSerialP__Resource__granted(void );

static inline msp430_uart_union_config_t *TelosSerialP__Msp430UartConfigure__getConfig(void );
# 51 "/home/csilab/tinyos/tos/lib/serial/SerialPacketInfoActiveMessageP.nc"
static inline uint8_t SerialPacketInfoActiveMessageP__Info__offset(void );


static inline uint8_t SerialPacketInfoActiveMessageP__Info__dataLinkLength(message_t *msg, uint8_t upperLen);


static inline uint8_t SerialPacketInfoActiveMessageP__Info__upperLength(message_t *msg, uint8_t dataLinkLen);
# 110 "/home/csilab/tinyos/tos/interfaces/AMSend.nc"
static void /*PrintfC.SerialAMSenderC.AMQueueEntryP*/AMQueueEntryP__0__AMSend__sendDone(
#line 103
message_t * msg, 






error_t error);
# 75 "/home/csilab/tinyos/tos/interfaces/Send.nc"
static error_t /*PrintfC.SerialAMSenderC.AMQueueEntryP*/AMQueueEntryP__0__Send__send(
#line 67
message_t * msg, 







uint8_t len);
# 92 "/home/csilab/upma/interfaces/AMPacket.nc"
static void /*PrintfC.SerialAMSenderC.AMQueueEntryP*/AMQueueEntryP__0__AMPacket__setDestination(
#line 88
message_t * amsg, 



am_addr_t addr);
#line 151
static void /*PrintfC.SerialAMSenderC.AMQueueEntryP*/AMQueueEntryP__0__AMPacket__setType(
#line 147
message_t * amsg, 



am_id_t t);
# 53 "/home/csilab/tinyos/tos/system/AMQueueEntryP.nc"
static error_t /*PrintfC.SerialAMSenderC.AMQueueEntryP*/AMQueueEntryP__0__AMSend__send(am_addr_t dest, 
message_t *msg, 
uint8_t len);









static inline void /*PrintfC.SerialAMSenderC.AMQueueEntryP*/AMQueueEntryP__0__Send__sendDone(message_t *m, error_t err);
# 80 "/home/csilab/tinyos/tos/interfaces/AMSend.nc"
static error_t /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__AMSend__send(
# 48 "/home/csilab/tinyos/tos/system/AMQueueImplP.nc"
am_id_t arg_0x7f7fed176840, 
# 80 "/home/csilab/tinyos/tos/interfaces/AMSend.nc"
am_addr_t addr, 
#line 71
message_t * msg, 








uint8_t len);
# 100 "/home/csilab/tinyos/tos/interfaces/Send.nc"
static void /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__Send__sendDone(
# 46 "/home/csilab/tinyos/tos/system/AMQueueImplP.nc"
uint8_t arg_0x7f7fed178630, 
# 96 "/home/csilab/tinyos/tos/interfaces/Send.nc"
message_t * msg, 



error_t error);
# 78 "/home/csilab/tinyos/tos/interfaces/Packet.nc"
static uint8_t /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__Packet__payloadLength(
#line 74
message_t * msg);
#line 94
static void /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__Packet__setPayloadLength(
#line 90
message_t * msg, 



uint8_t len);
# 67 "/home/csilab/tinyos/tos/interfaces/TaskBasic.nc"
static error_t /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__errorTask__postTask(void );
# 67 "/home/csilab/upma/interfaces/AMPacket.nc"
static am_addr_t /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__AMPacket__destination(
#line 63
message_t * amsg);
#line 136
static am_id_t /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__AMPacket__type(
#line 132
message_t * amsg);
# 126 "/home/csilab/tinyos/tos/system/AMQueueImplP.nc"
enum /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0____nesc_unnamed4341 {
#line 126
  AMQueueImplP__0__CancelTask = 7U
};
#line 126
typedef int /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0____nesc_sillytask_CancelTask[/*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__CancelTask];
#line 169
enum /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0____nesc_unnamed4342 {
#line 169
  AMQueueImplP__0__errorTask = 8U
};
#line 169
typedef int /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0____nesc_sillytask_errorTask[/*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__errorTask];
#line 57
#line 55
typedef struct /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0____nesc_unnamed4343 {
  message_t * msg;
} /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__queue_entry_t;

uint8_t /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__current = 1;
/*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__queue_entry_t /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__queue[1];
uint8_t /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__cancelMask[1 / 8 + 1];

static inline void /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__tryToSend(void );

static inline void /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__nextPacket(void );
#line 90
static inline error_t /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__Send__send(uint8_t clientId, message_t *msg, 
uint8_t len);
#line 126
static inline void /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__CancelTask__runTask(void );
#line 163
static void /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__sendDone(uint8_t last, message_t * msg, error_t err);





static inline void /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__errorTask__runTask(void );




static inline void /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__tryToSend(void );
#line 189
static inline void /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__AMSend__sendDone(am_id_t id, message_t *msg, error_t err);
#line 215
static inline void /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__Send__default__sendDone(uint8_t id, message_t *msg, error_t err);
# 48 "/home/csilab/tinyos/tos/system/QueueC.nc"
/*PrintfC.QueueC*/QueueC__0__queue_t  /*PrintfC.QueueC*/QueueC__0__queue[250];
uint8_t /*PrintfC.QueueC*/QueueC__0__head = 0;
uint8_t /*PrintfC.QueueC*/QueueC__0__tail = 0;
uint8_t /*PrintfC.QueueC*/QueueC__0__size = 0;

static inline bool /*PrintfC.QueueC*/QueueC__0__Queue__empty(void );



static inline uint8_t /*PrintfC.QueueC*/QueueC__0__Queue__size(void );



static inline uint8_t /*PrintfC.QueueC*/QueueC__0__Queue__maxSize(void );



static inline /*PrintfC.QueueC*/QueueC__0__queue_t /*PrintfC.QueueC*/QueueC__0__Queue__head(void );



static inline void /*PrintfC.QueueC*/QueueC__0__printQueue(void );
#line 85
static inline /*PrintfC.QueueC*/QueueC__0__queue_t /*PrintfC.QueueC*/QueueC__0__Queue__dequeue(void );
#line 97
static inline error_t /*PrintfC.QueueC*/QueueC__0__Queue__enqueue(/*PrintfC.QueueC*/QueueC__0__queue_t newVal);
# 90 "/home/csilab/tinyos/tos/interfaces/Queue.nc"
static error_t PrintfP__Queue__enqueue(
#line 86
PrintfP__Queue__t  newVal);
#line 81
static 
#line 79
PrintfP__Queue__t  

PrintfP__Queue__dequeue(void );
#line 50
static bool PrintfP__Queue__empty(void );







static uint8_t PrintfP__Queue__size(void );
# 80 "/home/csilab/tinyos/tos/interfaces/AMSend.nc"
static error_t PrintfP__AMSend__send(am_addr_t addr, 
#line 71
message_t * msg, 








uint8_t len);
# 126 "/home/csilab/tinyos/tos/interfaces/Packet.nc"
static 
#line 123
void * 


PrintfP__Packet__getPayload(
#line 121
message_t * msg, 




uint8_t len);
# 67 "/home/csilab/tinyos/tos/interfaces/TaskBasic.nc"
static error_t PrintfP__retrySend__postTask(void );
# 114 "/home/csilab/tinyos/tos/lib/printf/PrintfP.nc"
enum PrintfP____nesc_unnamed4344 {
#line 114
  PrintfP__retrySend = 9U
};
#line 114
typedef int PrintfP____nesc_sillytask_retrySend[PrintfP__retrySend];
#line 101
enum PrintfP____nesc_unnamed4345 {
  PrintfP__S_STARTED, 
  PrintfP__S_FLUSHING
};

message_t PrintfP__printfMsg;
uint8_t PrintfP__state = PrintfP__S_STARTED;

static inline error_t PrintfP__Init__init(void );




static inline void PrintfP__retrySend__runTask(void );




static void PrintfP__sendNext(void );










int printfflush(void )   ;
#line 142
static void PrintfP__AMSend__sendDone(message_t *msg, error_t error);









static inline int PrintfP__Putchar__putchar(int c);
# 49 "/home/csilab/tinyos/tos/lib/printf/Putchar.nc"
static int PutcharP__Putchar__putchar(int c);
# 98 "/home/csilab/tinyos/tos/lib/printf/PutcharP.nc"
static inline error_t PutcharP__Init__init(void );








int putchar(int c) __attribute((noinline))   ;
# 36 "/home/csilab/upma/interfaces/RadioPowerControl.nc"
static void CC2420CsmaP__RadioPowerControl__startDone(error_t error);





static void CC2420CsmaP__RadioPowerControl__stopDone(error_t error);
# 48 "/home/csilab/upma/interfaces/AsyncSend.nc"
static void CC2420CsmaP__Send__sendDone(
#line 44
message_t * msg, 



error_t error);
# 76 "/home/csilab/upma/chips/cc2420/CC2420Transmit.nc"
static error_t CC2420CsmaP__CC2420Transmit__send(message_t * p_msg);
# 95 "/home/csilab/tinyos/tos/interfaces/AsyncStdControl.nc"
static error_t CC2420CsmaP__SubControl__start(void );









static error_t CC2420CsmaP__SubControl__stop(void );
# 42 "/home/csilab/tinyos/tos/chips/cc2420/interfaces/CC2420PacketBody.nc"
static cc2420_header_t * CC2420CsmaP__CC2420PacketBody__getHeader(message_t * msg);










static cc2420_metadata_t * CC2420CsmaP__CC2420PacketBody__getMetadata(message_t * msg);
# 71 "/home/csilab/tinyos/tos/chips/cc2420/interfaces/CC2420Power.nc"
static error_t CC2420CsmaP__CC2420Power__startOscillator(void );
#line 90
static error_t CC2420CsmaP__CC2420Power__rxOn(void );
#line 51
static error_t CC2420CsmaP__CC2420Power__startVReg(void );
#line 63
static error_t CC2420CsmaP__CC2420Power__stopVReg(void );
# 120 "/home/csilab/tinyos/tos/interfaces/Resource.nc"
static error_t CC2420CsmaP__Resource__release(void );
#line 88
static error_t CC2420CsmaP__Resource__request(void );
# 66 "/home/csilab/tinyos/tos/interfaces/State.nc"
static bool CC2420CsmaP__SplitControlState__isState(uint8_t myState);
#line 45
static error_t CC2420CsmaP__SplitControlState__requestState(uint8_t reqState);





static void CC2420CsmaP__SplitControlState__forceState(uint8_t reqState);
# 67 "/home/csilab/tinyos/tos/interfaces/TaskBasic.nc"
static error_t CC2420CsmaP__stopDone_task__postTask(void );
#line 67
static error_t CC2420CsmaP__startDone_task__postTask(void );
# 80 "/home/csilab/upma/chips/cc2420/CC2420CsmaP.nc"
enum CC2420CsmaP____nesc_unnamed4346 {
#line 80
  CC2420CsmaP__startDone_task = 10U
};
#line 80
typedef int CC2420CsmaP____nesc_sillytask_startDone_task[CC2420CsmaP__startDone_task];
enum CC2420CsmaP____nesc_unnamed4347 {
#line 81
  CC2420CsmaP__stopDone_task = 11U
};
#line 81
typedef int CC2420CsmaP____nesc_sillytask_stopDone_task[CC2420CsmaP__stopDone_task];





enum CC2420CsmaP____nesc_unnamed4348 {
#line 87
  CC2420CsmaP__signalStartDone = 12U
};
#line 87
typedef int CC2420CsmaP____nesc_sillytask_signalStartDone[CC2420CsmaP__signalStartDone];
#line 110
enum CC2420CsmaP____nesc_unnamed4349 {
#line 110
  CC2420CsmaP__signalStopDone = 13U
};
#line 110
typedef int CC2420CsmaP____nesc_sillytask_signalStopDone[CC2420CsmaP__signalStopDone];
#line 64
enum CC2420CsmaP____nesc_unnamed4350 {
  CC2420CsmaP__S_STOPPED, 
  CC2420CsmaP__S_STARTING, 
  CC2420CsmaP__S_STARTED, 
  CC2420CsmaP__S_STOPPING, 
  CC2420CsmaP__S_TRANSMITTING
};

message_t * CC2420CsmaP__m_msg;

error_t CC2420CsmaP__sendErr = SUCCESS;







static void CC2420CsmaP__sendDone(void );

static inline void CC2420CsmaP__shutdown(void );


static inline void CC2420CsmaP__signalStartDone__runTask(void );



static error_t CC2420CsmaP__RadioPowerControl__start(void );
#line 110
static inline void CC2420CsmaP__signalStopDone__runTask(void );
#line 143
static inline error_t CC2420CsmaP__Send__send(message_t *p_msg, uint8_t len);
#line 179
static void *CC2420CsmaP__Send__getPayload(message_t *m, uint8_t len);








static inline uint8_t CC2420CsmaP__Send__maxPayloadLength(void );




static inline void CC2420CsmaP__CC2420Transmit__sendDone(message_t *p_msg, error_t err);




static inline void CC2420CsmaP__CC2420Power__startVRegDone(void );



static inline void CC2420CsmaP__Resource__granted(void );






static inline void CC2420CsmaP__CC2420Power__startOscillatorDone(void );





static void CC2420CsmaP__sendDone(void );
#line 243
static inline void CC2420CsmaP__startDone_task__runTask(void );










static inline void CC2420CsmaP__stopDone_task__runTask(void );
#line 275
static inline void CC2420CsmaP__shutdown(void );
# 55 "/home/csilab/tinyos/tos/chips/cc2420/interfaces/CC2420Config.nc"
static void CC2420ControlP__CC2420Config__syncDone(error_t error);
# 63 "/home/csilab/tinyos/tos/chips/cc2420/interfaces/CC2420Register.nc"
static cc2420_status_t CC2420ControlP__RXCTRL1__write(uint16_t data);
# 48 "/home/csilab/tinyos/tos/interfaces/LocalIeeeEui64.nc"
static ieee_eui64_t CC2420ControlP__LocalIeeeEui64__getId(void );
# 66 "/home/csilab/tinyos/tos/lib/timer/Alarm.nc"
static void CC2420ControlP__StartupTimer__start(CC2420ControlP__StartupTimer__size_type dt);
# 63 "/home/csilab/tinyos/tos/chips/cc2420/interfaces/CC2420Register.nc"
static cc2420_status_t CC2420ControlP__MDMCTRL0__write(uint16_t data);
# 46 "/home/csilab/tinyos/tos/interfaces/GeneralIO.nc"
static void CC2420ControlP__RSTN__makeOutput(void );
#line 40
static void CC2420ControlP__RSTN__set(void );
static void CC2420ControlP__RSTN__clr(void );
# 63 "/home/csilab/tinyos/tos/interfaces/Read.nc"
static void CC2420ControlP__ReadRssi__readDone(error_t result, CC2420ControlP__ReadRssi__val_t val);
# 67 "/home/csilab/tinyos/tos/interfaces/TaskBasic.nc"
static error_t CC2420ControlP__syncDone__postTask(void );
# 55 "/home/csilab/tinyos/tos/chips/cc2420/interfaces/CC2420Register.nc"
static cc2420_status_t CC2420ControlP__RSSI__read(uint16_t *data);







static cc2420_status_t CC2420ControlP__TXCTRL__write(uint16_t data);
#line 63
static cc2420_status_t CC2420ControlP__IOCFG0__write(uint16_t data);
# 50 "/home/csilab/tinyos/tos/interfaces/ActiveMessageAddress.nc"
static am_addr_t CC2420ControlP__ActiveMessageAddress__amAddress(void );




static am_group_t CC2420ControlP__ActiveMessageAddress__amGroup(void );
# 46 "/home/csilab/tinyos/tos/interfaces/GeneralIO.nc"
static void CC2420ControlP__CSN__makeOutput(void );
#line 40
static void CC2420ControlP__CSN__set(void );
static void CC2420ControlP__CSN__clr(void );




static void CC2420ControlP__VREN__makeOutput(void );
#line 40
static void CC2420ControlP__VREN__set(void );
static void CC2420ControlP__VREN__clr(void );
# 53 "/home/csilab/tinyos/tos/chips/cc2420/interfaces/CC2420Strobe.nc"
static cc2420_status_t CC2420ControlP__SXOSCON__strobe(void );
# 120 "/home/csilab/tinyos/tos/interfaces/Resource.nc"
static error_t CC2420ControlP__SpiResource__release(void );
#line 88
static error_t CC2420ControlP__SpiResource__request(void );
#line 120
static error_t CC2420ControlP__SyncResource__release(void );
#line 88
static error_t CC2420ControlP__SyncResource__request(void );
# 76 "/home/csilab/tinyos/tos/chips/cc2420/interfaces/CC2420Power.nc"
static void CC2420ControlP__CC2420Power__startOscillatorDone(void );
#line 56
static void CC2420ControlP__CC2420Power__startVRegDone(void );
# 63 "/home/csilab/tinyos/tos/chips/cc2420/interfaces/CC2420Register.nc"
static cc2420_status_t CC2420ControlP__IOCFG1__write(uint16_t data);
#line 63
static cc2420_status_t CC2420ControlP__FSCTRL__write(uint16_t data);
# 53 "/home/csilab/tinyos/tos/chips/cc2420/interfaces/CC2420Strobe.nc"
static cc2420_status_t CC2420ControlP__SRXON__strobe(void );
# 102 "/home/csilab/tinyos/tos/interfaces/Resource.nc"
static void CC2420ControlP__Resource__granted(void );
# 63 "/home/csilab/tinyos/tos/chips/cc2420/interfaces/CC2420Ram.nc"
static cc2420_status_t CC2420ControlP__IEEEADR__write(uint8_t offset, uint8_t * data, uint8_t length);
# 61 "/home/csilab/tinyos/tos/interfaces/GpioInterrupt.nc"
static error_t CC2420ControlP__InterruptCCA__disable(void );
#line 53
static error_t CC2420ControlP__InterruptCCA__enableRisingEdge(void );
# 120 "/home/csilab/tinyos/tos/interfaces/Resource.nc"
static error_t CC2420ControlP__RssiResource__release(void );
# 53 "/home/csilab/tinyos/tos/chips/cc2420/interfaces/CC2420Strobe.nc"
static cc2420_status_t CC2420ControlP__SRFOFF__strobe(void );
# 125 "/home/csilab/tinyos/tos/chips/cc2420/control/CC2420ControlP.nc"
enum CC2420ControlP____nesc_unnamed4351 {
#line 125
  CC2420ControlP__sync = 14U
};
#line 125
typedef int CC2420ControlP____nesc_sillytask_sync[CC2420ControlP__sync];
enum CC2420ControlP____nesc_unnamed4352 {
#line 126
  CC2420ControlP__syncDone = 15U
};
#line 126
typedef int CC2420ControlP____nesc_sillytask_syncDone[CC2420ControlP__syncDone];
#line 90
#line 84
typedef enum CC2420ControlP____nesc_unnamed4353 {
  CC2420ControlP__S_VREG_STOPPED, 
  CC2420ControlP__S_VREG_STARTING, 
  CC2420ControlP__S_VREG_STARTED, 
  CC2420ControlP__S_XOSC_STARTING, 
  CC2420ControlP__S_XOSC_STARTED
} CC2420ControlP__cc2420_control_state_t;

uint8_t CC2420ControlP__m_channel;

uint8_t CC2420ControlP__m_tx_power;

uint16_t CC2420ControlP__m_pan;

uint16_t CC2420ControlP__m_short_addr;

ieee_eui64_t CC2420ControlP__m_ext_addr;

bool CC2420ControlP__m_sync_busy;


bool CC2420ControlP__autoAckEnabled;


bool CC2420ControlP__hwAutoAckDefault;


bool CC2420ControlP__addressRecognition;


bool CC2420ControlP__hwAddressRecognition;

CC2420ControlP__cc2420_control_state_t CC2420ControlP__m_state = CC2420ControlP__S_VREG_STOPPED;



static void CC2420ControlP__writeFsctrl(void );
static void CC2420ControlP__writeMdmctrl0(void );
static void CC2420ControlP__writeId(void );
static inline void CC2420ControlP__writeTxctrl(void );





static inline error_t CC2420ControlP__Init__init(void );
#line 188
static inline error_t CC2420ControlP__Resource__request(void );







static inline error_t CC2420ControlP__Resource__release(void );







static inline error_t CC2420ControlP__CC2420Power__startVReg(void );
#line 216
static inline error_t CC2420ControlP__CC2420Power__stopVReg(void );







static inline error_t CC2420ControlP__CC2420Power__startOscillator(void );
#line 268
static inline error_t CC2420ControlP__CC2420Power__rxOn(void );
#line 302
static inline uint16_t CC2420ControlP__CC2420Config__getShortAddr(void );







static inline uint16_t CC2420ControlP__CC2420Config__getPanAddr(void );
#line 323
static inline error_t CC2420ControlP__CC2420Config__sync(void );
#line 355
static inline bool CC2420ControlP__CC2420Config__isAddressRecognitionEnabled(void );
#line 382
static inline bool CC2420ControlP__CC2420Config__isHwAutoAckDefault(void );






static inline bool CC2420ControlP__CC2420Config__isAutoAckEnabled(void );









static inline void CC2420ControlP__SyncResource__granted(void );
#line 413
static inline void CC2420ControlP__SpiResource__granted(void );




static inline void CC2420ControlP__RssiResource__granted(void );
#line 431
static inline void CC2420ControlP__StartupTimer__fired(void );









static inline void CC2420ControlP__InterruptCCA__fired(void );
#line 465
static inline void CC2420ControlP__sync__runTask(void );



static inline void CC2420ControlP__syncDone__runTask(void );









static void CC2420ControlP__writeFsctrl(void );
#line 496
static void CC2420ControlP__writeMdmctrl0(void );
#line 515
static void CC2420ControlP__writeId(void );
#line 533
static inline void CC2420ControlP__writeTxctrl(void );
#line 545
static inline void CC2420ControlP__ReadRssi__default__readDone(error_t error, uint16_t data);
# 41 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__setEvent(uint16_t time);

static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__setEventFromNow(uint16_t delta);
# 45 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430Timer.nc"
static uint16_t /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430Timer__get(void );
# 78 "/home/csilab/tinyos/tos/lib/timer/Alarm.nc"
static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Alarm__fired(void );
# 57 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430TimerControl.nc"
static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__enableEvents(void );
#line 47
static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__setControlAsCompare(void );










static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__disableEvents(void );
#line 44
static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__clearPendingInterrupt(void );
# 53 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline error_t /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Init__init(void );
#line 65
static inline void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Alarm__stop(void );




static inline void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__fired(void );










static inline void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Alarm__startAt(uint16_t t0, uint16_t dt);
#line 114
static inline void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430Timer__overflow(void );
# 64 "/home/csilab/tinyos/tos/lib/timer/Counter.nc"
static /*Counter32khz32C.Transform*/TransformCounterC__0__CounterFrom__size_type /*Counter32khz32C.Transform*/TransformCounterC__0__CounterFrom__get(void );






static bool /*Counter32khz32C.Transform*/TransformCounterC__0__CounterFrom__isOverflowPending(void );










static void /*Counter32khz32C.Transform*/TransformCounterC__0__Counter__overflow(void );
# 67 "/home/csilab/tinyos/tos/lib/timer/TransformCounterC.nc"
/*Counter32khz32C.Transform*/TransformCounterC__0__upper_count_type /*Counter32khz32C.Transform*/TransformCounterC__0__m_upper;

enum /*Counter32khz32C.Transform*/TransformCounterC__0____nesc_unnamed4354 {

  TransformCounterC__0__LOW_SHIFT_RIGHT = 0, 
  TransformCounterC__0__HIGH_SHIFT_LEFT = 8 * sizeof(/*Counter32khz32C.Transform*/TransformCounterC__0__from_size_type ) - /*Counter32khz32C.Transform*/TransformCounterC__0__LOW_SHIFT_RIGHT, 
  TransformCounterC__0__NUM_UPPER_BITS = 8 * sizeof(/*Counter32khz32C.Transform*/TransformCounterC__0__to_size_type ) - 8 * sizeof(/*Counter32khz32C.Transform*/TransformCounterC__0__from_size_type ) + 0, 



  TransformCounterC__0__OVERFLOW_MASK = /*Counter32khz32C.Transform*/TransformCounterC__0__NUM_UPPER_BITS ? ((/*Counter32khz32C.Transform*/TransformCounterC__0__upper_count_type )2 << (/*Counter32khz32C.Transform*/TransformCounterC__0__NUM_UPPER_BITS - 1)) - 1 : 0
};

static /*Counter32khz32C.Transform*/TransformCounterC__0__to_size_type /*Counter32khz32C.Transform*/TransformCounterC__0__Counter__get(void );
#line 133
static inline void /*Counter32khz32C.Transform*/TransformCounterC__0__CounterFrom__overflow(void );
# 78 "/home/csilab/tinyos/tos/lib/timer/Alarm.nc"
static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Alarm__fired(void );
#line 103
static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__AlarmFrom__startAt(/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__AlarmFrom__size_type t0, /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__AlarmFrom__size_type dt);
#line 73
static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__AlarmFrom__stop(void );
# 64 "/home/csilab/tinyos/tos/lib/timer/Counter.nc"
static /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Counter__size_type /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Counter__get(void );
# 77 "/home/csilab/tinyos/tos/lib/timer/TransformAlarmC.nc"
/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__to_size_type /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__m_t0;
/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__to_size_type /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__m_dt;

enum /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0____nesc_unnamed4355 {

  TransformAlarmC__0__MAX_DELAY_LOG2 = 8 * sizeof(/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__from_size_type ) - 1 - 0, 
  TransformAlarmC__0__MAX_DELAY = (/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__to_size_type )1 << /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__MAX_DELAY_LOG2
};

static inline /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__to_size_type /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Alarm__getNow(void );
#line 102
static inline void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Alarm__stop(void );




static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__set_alarm(void );
#line 147
static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Alarm__startAt(/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__to_size_type t0, /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__to_size_type dt);









static inline void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Alarm__start(/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__to_size_type dt);




static inline void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__AlarmFrom__fired(void );
#line 177
static inline void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Counter__overflow(void );
# 78 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*HplCC2420PinsC.CCAM*/Msp430GpioC__3__HplGeneralIO__makeInput(void );
#line 73
static bool /*HplCC2420PinsC.CCAM*/Msp430GpioC__3__HplGeneralIO__get(void );
# 51 "/home/csilab/tinyos/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline bool /*HplCC2420PinsC.CCAM*/Msp430GpioC__3__GeneralIO__get(void );
static inline void /*HplCC2420PinsC.CCAM*/Msp430GpioC__3__GeneralIO__makeInput(void );
# 85 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*HplCC2420PinsC.CSNM*/Msp430GpioC__4__HplGeneralIO__makeOutput(void );
#line 48
static void /*HplCC2420PinsC.CSNM*/Msp430GpioC__4__HplGeneralIO__set(void );




static void /*HplCC2420PinsC.CSNM*/Msp430GpioC__4__HplGeneralIO__clr(void );
# 48 "/home/csilab/tinyos/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplCC2420PinsC.CSNM*/Msp430GpioC__4__GeneralIO__set(void );
static inline void /*HplCC2420PinsC.CSNM*/Msp430GpioC__4__GeneralIO__clr(void );




static inline void /*HplCC2420PinsC.CSNM*/Msp430GpioC__4__GeneralIO__makeOutput(void );
# 73 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static bool /*HplCC2420PinsC.FIFOM*/Msp430GpioC__5__HplGeneralIO__get(void );
# 51 "/home/csilab/tinyos/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline bool /*HplCC2420PinsC.FIFOM*/Msp430GpioC__5__GeneralIO__get(void );
# 73 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static bool /*HplCC2420PinsC.FIFOPM*/Msp430GpioC__6__HplGeneralIO__get(void );
# 51 "/home/csilab/tinyos/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline bool /*HplCC2420PinsC.FIFOPM*/Msp430GpioC__6__GeneralIO__get(void );
# 85 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*HplCC2420PinsC.RSTNM*/Msp430GpioC__7__HplGeneralIO__makeOutput(void );
#line 48
static void /*HplCC2420PinsC.RSTNM*/Msp430GpioC__7__HplGeneralIO__set(void );




static void /*HplCC2420PinsC.RSTNM*/Msp430GpioC__7__HplGeneralIO__clr(void );
# 48 "/home/csilab/tinyos/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplCC2420PinsC.RSTNM*/Msp430GpioC__7__GeneralIO__set(void );
static inline void /*HplCC2420PinsC.RSTNM*/Msp430GpioC__7__GeneralIO__clr(void );




static inline void /*HplCC2420PinsC.RSTNM*/Msp430GpioC__7__GeneralIO__makeOutput(void );
# 78 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*HplCC2420PinsC.SFDM*/Msp430GpioC__8__HplGeneralIO__makeInput(void );
#line 73
static bool /*HplCC2420PinsC.SFDM*/Msp430GpioC__8__HplGeneralIO__get(void );
# 51 "/home/csilab/tinyos/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline bool /*HplCC2420PinsC.SFDM*/Msp430GpioC__8__GeneralIO__get(void );
static inline void /*HplCC2420PinsC.SFDM*/Msp430GpioC__8__GeneralIO__makeInput(void );
# 85 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*HplCC2420PinsC.VRENM*/Msp430GpioC__9__HplGeneralIO__makeOutput(void );
#line 48
static void /*HplCC2420PinsC.VRENM*/Msp430GpioC__9__HplGeneralIO__set(void );




static void /*HplCC2420PinsC.VRENM*/Msp430GpioC__9__HplGeneralIO__clr(void );
# 48 "/home/csilab/tinyos/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplCC2420PinsC.VRENM*/Msp430GpioC__9__GeneralIO__set(void );
static inline void /*HplCC2420PinsC.VRENM*/Msp430GpioC__9__GeneralIO__clr(void );




static inline void /*HplCC2420PinsC.VRENM*/Msp430GpioC__9__GeneralIO__makeOutput(void );
# 68 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Msp430Capture__clearOverflow(void );
# 61 "/home/csilab/tinyos/tos/interfaces/GpioCapture.nc"
static void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Capture__captured(uint16_t time);
# 55 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430TimerControl.nc"
static void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Msp430TimerControl__setControlAsCapture(uint8_t cm);

static void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Msp430TimerControl__enableEvents(void );
static void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Msp430TimerControl__disableEvents(void );
#line 44
static void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Msp430TimerControl__clearPendingInterrupt(void );
# 99 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__GeneralIO__selectIOFunc(void );
#line 92
static void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__GeneralIO__selectModuleFunc(void );
# 49 "/home/csilab/tinyos/tos/chips/msp430/timer/GpioCaptureC.nc"
static error_t /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__enableCapture(uint8_t mode);
#line 61
static inline error_t /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Capture__captureRisingEdge(void );



static inline error_t /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Capture__captureFallingEdge(void );



static inline void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Capture__disable(void );






static inline void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Msp430Capture__captured(uint16_t time);
# 72 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
static void HplMsp430InterruptP__Port14__fired(void );
#line 72
static void HplMsp430InterruptP__Port26__fired(void );
#line 72
static void HplMsp430InterruptP__Port17__fired(void );
#line 72
static void HplMsp430InterruptP__Port21__fired(void );
#line 72
static void HplMsp430InterruptP__Port12__fired(void );
#line 72
static void HplMsp430InterruptP__Port24__fired(void );
#line 72
static void HplMsp430InterruptP__Port15__fired(void );
#line 72
static void HplMsp430InterruptP__Port27__fired(void );
#line 72
static void HplMsp430InterruptP__Port10__fired(void );
#line 72
static void HplMsp430InterruptP__Port22__fired(void );
#line 72
static void HplMsp430InterruptP__Port13__fired(void );
#line 72
static void HplMsp430InterruptP__Port25__fired(void );
#line 72
static void HplMsp430InterruptP__Port16__fired(void );
#line 72
static void HplMsp430InterruptP__Port20__fired(void );
#line 72
static void HplMsp430InterruptP__Port11__fired(void );
#line 72
static void HplMsp430InterruptP__Port23__fired(void );
# 64 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
void sig_PORT1_VECTOR(void ) __attribute((wakeup)) __attribute((interrupt(0x0008)))  ;
#line 79
static inline void HplMsp430InterruptP__Port11__default__fired(void );
static inline void HplMsp430InterruptP__Port12__default__fired(void );
static inline void HplMsp430InterruptP__Port13__default__fired(void );

static inline void HplMsp430InterruptP__Port15__default__fired(void );
static inline void HplMsp430InterruptP__Port16__default__fired(void );
static inline void HplMsp430InterruptP__Port17__default__fired(void );
static inline void HplMsp430InterruptP__Port10__enable(void );



static inline void HplMsp430InterruptP__Port14__enable(void );



static inline void HplMsp430InterruptP__Port10__disable(void );



static inline void HplMsp430InterruptP__Port14__disable(void );



static inline void HplMsp430InterruptP__Port10__clear(void );
static inline void HplMsp430InterruptP__Port11__clear(void );
static inline void HplMsp430InterruptP__Port12__clear(void );
static inline void HplMsp430InterruptP__Port13__clear(void );
static inline void HplMsp430InterruptP__Port14__clear(void );
static inline void HplMsp430InterruptP__Port15__clear(void );
static inline void HplMsp430InterruptP__Port16__clear(void );
static inline void HplMsp430InterruptP__Port17__clear(void );








static inline void HplMsp430InterruptP__Port10__edge(bool l2h);
#line 142
static inline void HplMsp430InterruptP__Port14__edge(bool l2h);
#line 169
void sig_PORT2_VECTOR(void ) __attribute((wakeup)) __attribute((interrupt(0x0002)))  ;
#line 182
static inline void HplMsp430InterruptP__Port20__default__fired(void );
static inline void HplMsp430InterruptP__Port21__default__fired(void );
static inline void HplMsp430InterruptP__Port22__default__fired(void );
static inline void HplMsp430InterruptP__Port23__default__fired(void );
static inline void HplMsp430InterruptP__Port24__default__fired(void );
static inline void HplMsp430InterruptP__Port25__default__fired(void );
static inline void HplMsp430InterruptP__Port26__default__fired(void );
static inline void HplMsp430InterruptP__Port27__default__fired(void );
#line 206
static inline void HplMsp430InterruptP__Port20__clear(void );
static inline void HplMsp430InterruptP__Port21__clear(void );
static inline void HplMsp430InterruptP__Port22__clear(void );
static inline void HplMsp430InterruptP__Port23__clear(void );
static inline void HplMsp430InterruptP__Port24__clear(void );
static inline void HplMsp430InterruptP__Port25__clear(void );
static inline void HplMsp430InterruptP__Port26__clear(void );
static inline void HplMsp430InterruptP__Port27__clear(void );
# 52 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
static void /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__HplInterrupt__clear(void );
#line 47
static void /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__HplInterrupt__disable(void );
#line 67
static void /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__HplInterrupt__edge(bool low_to_high);
#line 42
static void /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__HplInterrupt__enable(void );
# 68 "/home/csilab/tinyos/tos/interfaces/GpioInterrupt.nc"
static void /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__Interrupt__fired(void );
# 52 "/home/csilab/tinyos/tos/chips/msp430/pins/Msp430InterruptC.nc"
static inline error_t /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__enable(bool rising);








static inline error_t /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__Interrupt__enableRisingEdge(void );







static inline error_t /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__Interrupt__disable(void );







static inline void /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__HplInterrupt__fired(void );
# 52 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
static void /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__HplInterrupt__clear(void );
#line 47
static void /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__HplInterrupt__disable(void );
#line 67
static void /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__HplInterrupt__edge(bool low_to_high);
#line 42
static void /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__HplInterrupt__enable(void );
# 68 "/home/csilab/tinyos/tos/interfaces/GpioInterrupt.nc"
static void /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__Interrupt__fired(void );
# 52 "/home/csilab/tinyos/tos/chips/msp430/pins/Msp430InterruptC.nc"
static inline error_t /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__enable(bool rising);
#line 65
static inline error_t /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__Interrupt__enableFallingEdge(void );



static inline error_t /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__Interrupt__disable(void );







static inline void /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__HplInterrupt__fired(void );
# 70 "/home/csilab/tinyos/tos/interfaces/SpiPacket.nc"
static error_t CC2420SpiP__SpiPacket__send(
#line 59
uint8_t * txBuf, 

uint8_t * rxBuf, 








uint16_t len);
# 91 "/home/csilab/tinyos/tos/chips/cc2420/interfaces/CC2420Fifo.nc"
static void CC2420SpiP__Fifo__writeDone(
# 46 "/home/csilab/tinyos/tos/chips/cc2420/spi/CC2420SpiP.nc"
uint8_t arg_0x7f7fecc8f6e0, 
# 91 "/home/csilab/tinyos/tos/chips/cc2420/interfaces/CC2420Fifo.nc"
uint8_t * data, uint8_t length, error_t error);
#line 71
static void CC2420SpiP__Fifo__readDone(
# 46 "/home/csilab/tinyos/tos/chips/cc2420/spi/CC2420SpiP.nc"
uint8_t arg_0x7f7fecc8f6e0, 
# 71 "/home/csilab/tinyos/tos/chips/cc2420/interfaces/CC2420Fifo.nc"
uint8_t * data, uint8_t length, error_t error);
# 24 "/home/csilab/tinyos/tos/chips/cc2420/interfaces/ChipSpiResource.nc"
static void CC2420SpiP__ChipSpiResource__releasing(void );
# 45 "/home/csilab/tinyos/tos/interfaces/SpiByte.nc"
static uint8_t CC2420SpiP__SpiByte__write(uint8_t tx);
# 56 "/home/csilab/tinyos/tos/interfaces/State.nc"
static void CC2420SpiP__WorkingState__toIdle(void );




static bool CC2420SpiP__WorkingState__isIdle(void );
#line 45
static error_t CC2420SpiP__WorkingState__requestState(uint8_t reqState);
# 120 "/home/csilab/tinyos/tos/interfaces/Resource.nc"
static error_t CC2420SpiP__SpiResource__release(void );
#line 97
static error_t CC2420SpiP__SpiResource__immediateRequest(void );
#line 88
static error_t CC2420SpiP__SpiResource__request(void );
#line 128
static bool CC2420SpiP__SpiResource__isOwner(void );
#line 102
static void CC2420SpiP__Resource__granted(
# 45 "/home/csilab/tinyos/tos/chips/cc2420/spi/CC2420SpiP.nc"
uint8_t arg_0x7f7fecc90360);
# 67 "/home/csilab/tinyos/tos/interfaces/TaskBasic.nc"
static error_t CC2420SpiP__grant__postTask(void );
# 88 "/home/csilab/tinyos/tos/chips/cc2420/spi/CC2420SpiP.nc"
enum CC2420SpiP____nesc_unnamed4356 {
#line 88
  CC2420SpiP__grant = 16U
};
#line 88
typedef int CC2420SpiP____nesc_sillytask_grant[CC2420SpiP__grant];
#line 63
enum CC2420SpiP____nesc_unnamed4357 {
  CC2420SpiP__RESOURCE_COUNT = 5U, 
  CC2420SpiP__NO_HOLDER = 0xFF
};


enum CC2420SpiP____nesc_unnamed4358 {
  CC2420SpiP__S_IDLE, 
  CC2420SpiP__S_BUSY
};


uint16_t CC2420SpiP__m_addr;


uint8_t CC2420SpiP__m_requests = 0;


uint8_t CC2420SpiP__m_holder = CC2420SpiP__NO_HOLDER;


bool CC2420SpiP__release;


static error_t CC2420SpiP__attemptRelease(void );







static inline void CC2420SpiP__ChipSpiResource__abortRelease(void );






static inline error_t CC2420SpiP__ChipSpiResource__attemptRelease(void );




static error_t CC2420SpiP__Resource__request(uint8_t id);
#line 126
static error_t CC2420SpiP__Resource__immediateRequest(uint8_t id);
#line 149
static error_t CC2420SpiP__Resource__release(uint8_t id);
#line 178
static inline bool CC2420SpiP__Resource__isOwner(uint8_t id);





static inline void CC2420SpiP__SpiResource__granted(void );




static cc2420_status_t CC2420SpiP__Fifo__beginRead(uint8_t addr, uint8_t *data, 
uint8_t len);
#line 209
static inline error_t CC2420SpiP__Fifo__continueRead(uint8_t addr, uint8_t *data, 
uint8_t len);



static inline cc2420_status_t CC2420SpiP__Fifo__write(uint8_t addr, uint8_t *data, 
uint8_t len);
#line 260
static cc2420_status_t CC2420SpiP__Ram__write(uint16_t addr, uint8_t offset, 
uint8_t *data, 
uint8_t len);
#line 287
static inline cc2420_status_t CC2420SpiP__Reg__read(uint8_t addr, uint16_t *data);
#line 305
static cc2420_status_t CC2420SpiP__Reg__write(uint8_t addr, uint16_t data);
#line 318
static cc2420_status_t CC2420SpiP__Strobe__strobe(uint8_t addr);










static void CC2420SpiP__SpiPacket__sendDone(uint8_t *tx_buf, uint8_t *rx_buf, 
uint16_t len, error_t error);








static error_t CC2420SpiP__attemptRelease(void );
#line 358
static inline void CC2420SpiP__grant__runTask(void );








static inline void CC2420SpiP__Resource__default__granted(uint8_t id);


static inline void CC2420SpiP__Fifo__default__readDone(uint8_t addr, uint8_t *rx_buf, uint8_t rx_len, error_t error);


static inline void CC2420SpiP__Fifo__default__writeDone(uint8_t addr, uint8_t *tx_buf, uint8_t tx_len, error_t error);
# 74 "/home/csilab/tinyos/tos/system/StateImplP.nc"
uint8_t StateImplP__state[3U];

enum StateImplP____nesc_unnamed4359 {
  StateImplP__S_IDLE = 0
};


static inline error_t StateImplP__Init__init(void );
#line 96
static error_t StateImplP__State__requestState(uint8_t id, uint8_t reqState);
#line 111
static inline void StateImplP__State__forceState(uint8_t id, uint8_t reqState);






static inline void StateImplP__State__toIdle(uint8_t id);







static inline bool StateImplP__State__isIdle(uint8_t id);






static bool StateImplP__State__isState(uint8_t id, uint8_t myState);
# 82 "/home/csilab/tinyos/tos/interfaces/SpiPacket.nc"
static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__SpiPacket__sendDone(
# 79 "/home/csilab/tinyos/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x7f7fecbdb020, 
# 75 "/home/csilab/tinyos/tos/interfaces/SpiPacket.nc"
uint8_t * txBuf, 
uint8_t * rxBuf, 





uint16_t len, 
error_t error);
# 39 "/home/csilab/tinyos/tos/chips/msp430/usart/Msp430SpiConfigure.nc"
static msp430_spi_union_config_t */*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Msp430SpiConfigure__getConfig(
# 82 "/home/csilab/tinyos/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x7f7fecbd9110);
# 180 "/home/csilab/tinyos/tos/chips/msp430/usart/HplMsp430Usart.nc"
static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__enableRxIntr(void );
#line 197
static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__clrRxIntr(void );
#line 97
static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__resetUsart(bool reset);
#line 177
static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__disableRxIntr(void );
#line 224
static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__tx(uint8_t data);
#line 168
static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__setModeSpi(msp430_spi_union_config_t *config);
#line 231
static uint8_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__rx(void );
#line 192
static bool /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__isRxIntrPending(void );
#line 158
static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__disableSpi(void );
# 120 "/home/csilab/tinyos/tos/interfaces/Resource.nc"
static error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__release(
# 81 "/home/csilab/tinyos/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x7f7fecbdbe30);
# 97 "/home/csilab/tinyos/tos/interfaces/Resource.nc"
static error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__immediateRequest(
# 81 "/home/csilab/tinyos/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x7f7fecbdbe30);
# 88 "/home/csilab/tinyos/tos/interfaces/Resource.nc"
static error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__request(
# 81 "/home/csilab/tinyos/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x7f7fecbdbe30);
# 128 "/home/csilab/tinyos/tos/interfaces/Resource.nc"
static bool /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__isOwner(
# 81 "/home/csilab/tinyos/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x7f7fecbdbe30);
# 102 "/home/csilab/tinyos/tos/interfaces/Resource.nc"
static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Resource__granted(
# 75 "/home/csilab/tinyos/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x7f7fecbe29f0);
# 67 "/home/csilab/tinyos/tos/interfaces/TaskBasic.nc"
static error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__signalDone_task__postTask(void );
# 102 "/home/csilab/tinyos/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
enum /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0____nesc_unnamed4360 {
#line 102
  Msp430SpiNoDmaP__0__signalDone_task = 17U
};
#line 102
typedef int /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0____nesc_sillytask_signalDone_task[/*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__signalDone_task];
#line 91
enum /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0____nesc_unnamed4361 {
  Msp430SpiNoDmaP__0__SPI_ATOMIC_SIZE = 2
};

uint16_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_len;
uint8_t * /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_tx_buf;
uint8_t * /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_rx_buf;
uint16_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_pos;
uint8_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_client;

static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__signalDone(void );


static inline error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Resource__immediateRequest(uint8_t id);



static inline error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Resource__request(uint8_t id);



static inline bool /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Resource__isOwner(uint8_t id);



static inline error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Resource__release(uint8_t id);



static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__ResourceConfigure__configure(uint8_t id);



static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__ResourceConfigure__unconfigure(uint8_t id);





static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__granted(uint8_t id);



static uint8_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__SpiByte__write(uint8_t tx);
#line 172
static inline bool /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__default__isOwner(uint8_t id);
static inline error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__default__request(uint8_t id);
static inline error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__default__immediateRequest(uint8_t id);
static inline error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__default__release(uint8_t id);
static inline msp430_spi_union_config_t */*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Msp430SpiConfigure__default__getConfig(uint8_t id);



static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Resource__default__granted(uint8_t id);

static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__continueOp(void );
#line 205
static error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__SpiPacket__send(uint8_t id, uint8_t *tx_buf, 
uint8_t *rx_buf, 
uint16_t len);
#line 227
static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__signalDone_task__runTask(void );



static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartInterrupts__rxDone(uint8_t data);
#line 244
static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__signalDone(void );




static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartInterrupts__txDone(void );

static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__SpiPacket__default__sendDone(uint8_t id, uint8_t *tx_buf, uint8_t *rx_buf, uint16_t len, error_t error);
# 99 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void HplMsp430Usart0P__UCLK__selectIOFunc(void );
#line 92
static void HplMsp430Usart0P__UCLK__selectModuleFunc(void );
# 54 "/home/csilab/tinyos/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
static void HplMsp430Usart0P__Interrupts__rxDone(uint8_t data);
#line 49
static void HplMsp430Usart0P__Interrupts__txDone(void );
# 99 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void HplMsp430Usart0P__URXD__selectIOFunc(void );
#line 99
static void HplMsp430Usart0P__UTXD__selectIOFunc(void );
# 7 "/home/csilab/tinyos/tos/chips/msp430/usart/HplMsp430I2C.nc"
static void HplMsp430Usart0P__HplI2C__clearModeI2C(void );
#line 6
static bool HplMsp430Usart0P__HplI2C__isI2C(void );
# 99 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void HplMsp430Usart0P__SOMI__selectIOFunc(void );
#line 92
static void HplMsp430Usart0P__SOMI__selectModuleFunc(void );
# 39 "/home/csilab/tinyos/tos/chips/msp430/usart/HplMsp430I2CInterrupts.nc"
static void HplMsp430Usart0P__I2CInterrupts__fired(void );
# 99 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void HplMsp430Usart0P__SIMO__selectIOFunc(void );
#line 92
static void HplMsp430Usart0P__SIMO__selectModuleFunc(void );
# 89 "/home/csilab/tinyos/tos/chips/msp430/usart/HplMsp430Usart0P.nc"
static volatile uint8_t HplMsp430Usart0P__IE1 __asm ("0x0000");
static volatile uint8_t HplMsp430Usart0P__ME1 __asm ("0x0004");
static volatile uint8_t HplMsp430Usart0P__IFG1 __asm ("0x0002");
static volatile uint8_t HplMsp430Usart0P__U0TCTL __asm ("0x0071");

static volatile uint8_t HplMsp430Usart0P__U0TXBUF __asm ("0x0077");

void sig_UART0RX_VECTOR(void ) __attribute((wakeup)) __attribute((interrupt(0x0012)))  ;




void sig_UART0TX_VECTOR(void ) __attribute((wakeup)) __attribute((interrupt(0x0010)))  ;
#line 132
static inline void HplMsp430Usart0P__Usart__setUbr(uint16_t control);










static inline void HplMsp430Usart0P__Usart__setUmctl(uint8_t control);







static inline void HplMsp430Usart0P__Usart__resetUsart(bool reset);
#line 207
static inline void HplMsp430Usart0P__Usart__disableUart(void );
#line 238
static inline void HplMsp430Usart0P__Usart__enableSpi(void );








static void HplMsp430Usart0P__Usart__disableSpi(void );








static inline void HplMsp430Usart0P__configSpi(msp430_spi_union_config_t *config);








static void HplMsp430Usart0P__Usart__setModeSpi(msp430_spi_union_config_t *config);
#line 330
static inline bool HplMsp430Usart0P__Usart__isRxIntrPending(void );










static inline void HplMsp430Usart0P__Usart__clrRxIntr(void );



static inline void HplMsp430Usart0P__Usart__clrIntr(void );



static inline void HplMsp430Usart0P__Usart__disableRxIntr(void );







static inline void HplMsp430Usart0P__Usart__disableIntr(void );



static inline void HplMsp430Usart0P__Usart__enableRxIntr(void );
#line 382
static inline void HplMsp430Usart0P__Usart__tx(uint8_t data);



static inline uint8_t HplMsp430Usart0P__Usart__rx(void );
# 90 "/home/csilab/tinyos/tos/interfaces/ArbiterInfo.nc"
static bool /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__1__ArbiterInfo__inUse(void );







static uint8_t /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__1__ArbiterInfo__userId(void );
# 54 "/home/csilab/tinyos/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
static void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__1__Interrupts__rxDone(
# 39 "/home/csilab/tinyos/tos/chips/msp430/usart/Msp430UsartShareP.nc"
uint8_t arg_0x7f7fed2b0600, 
# 54 "/home/csilab/tinyos/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
uint8_t data);
#line 49
static void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__1__Interrupts__txDone(
# 39 "/home/csilab/tinyos/tos/chips/msp430/usart/Msp430UsartShareP.nc"
uint8_t arg_0x7f7fed2b0600);
# 39 "/home/csilab/tinyos/tos/chips/msp430/usart/HplMsp430I2CInterrupts.nc"
static void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__1__I2CInterrupts__fired(
# 40 "/home/csilab/tinyos/tos/chips/msp430/usart/Msp430UsartShareP.nc"
uint8_t arg_0x7f7fed2ae1c0);








static inline void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__1__RawInterrupts__txDone(void );




static inline void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__1__RawInterrupts__rxDone(uint8_t data);




static inline void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__1__RawI2CInterrupts__fired(void );




static inline void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__1__Interrupts__default__txDone(uint8_t id);
static inline void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__1__Interrupts__default__rxDone(uint8_t id, uint8_t data);
static inline void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__1__I2CInterrupts__default__fired(uint8_t id);
# 49 "/home/csilab/tinyos/tos/system/FcfsResourceQueueC.nc"
enum /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__2____nesc_unnamed4362 {
#line 49
  FcfsResourceQueueC__2__NO_ENTRY = 0xFF
};
uint8_t /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__2__resQ[1U];
uint8_t /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__2__qHead = /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__2__NO_ENTRY;
uint8_t /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__2__qTail = /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__2__NO_ENTRY;

static inline error_t /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__2__Init__init(void );




static inline bool /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__2__FcfsQueue__isEmpty(void );



static inline bool /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__2__FcfsQueue__isEnqueued(resource_client_id_t id);



static inline resource_client_id_t /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__2__FcfsQueue__dequeue(void );
#line 82
static inline error_t /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__2__FcfsQueue__enqueue(resource_client_id_t id);
# 53 "/home/csilab/tinyos/tos/interfaces/ResourceRequested.nc"
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceRequested__requested(
# 55 "/home/csilab/tinyos/tos/system/ArbiterP.nc"
uint8_t arg_0x7f7fed25e840);
# 61 "/home/csilab/tinyos/tos/interfaces/ResourceRequested.nc"
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceRequested__immediateRequested(
# 55 "/home/csilab/tinyos/tos/system/ArbiterP.nc"
uint8_t arg_0x7f7fed25e840);
# 65 "/home/csilab/tinyos/tos/interfaces/ResourceConfigure.nc"
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceConfigure__unconfigure(
# 60 "/home/csilab/tinyos/tos/system/ArbiterP.nc"
uint8_t arg_0x7f7fed25bc40);
# 59 "/home/csilab/tinyos/tos/interfaces/ResourceConfigure.nc"
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceConfigure__configure(
# 60 "/home/csilab/tinyos/tos/system/ArbiterP.nc"
uint8_t arg_0x7f7fed25bc40);
# 79 "/home/csilab/tinyos/tos/interfaces/ResourceQueue.nc"
static error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__Queue__enqueue(resource_client_id_t id);
#line 53
static bool /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__Queue__isEmpty(void );
#line 70
static resource_client_id_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__Queue__dequeue(void );
# 73 "/home/csilab/tinyos/tos/interfaces/ResourceDefaultOwner.nc"
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceDefaultOwner__requested(void );
#line 46
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceDefaultOwner__granted(void );
#line 81
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceDefaultOwner__immediateRequested(void );
# 102 "/home/csilab/tinyos/tos/interfaces/Resource.nc"
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__Resource__granted(
# 54 "/home/csilab/tinyos/tos/system/ArbiterP.nc"
uint8_t arg_0x7f7fed25f4b0);
# 67 "/home/csilab/tinyos/tos/interfaces/TaskBasic.nc"
static error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__grantedTask__postTask(void );
# 75 "/home/csilab/tinyos/tos/system/ArbiterP.nc"
enum /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1____nesc_unnamed4363 {
#line 75
  ArbiterP__1__grantedTask = 18U
};
#line 75
typedef int /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1____nesc_sillytask_grantedTask[/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__grantedTask];
#line 67
enum /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1____nesc_unnamed4364 {
#line 67
  ArbiterP__1__RES_CONTROLLED, ArbiterP__1__RES_GRANTING, ArbiterP__1__RES_IMM_GRANTING, ArbiterP__1__RES_BUSY
};
#line 68
enum /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1____nesc_unnamed4365 {
#line 68
  ArbiterP__1__default_owner_id = 1U
};
#line 69
enum /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1____nesc_unnamed4366 {
#line 69
  ArbiterP__1__NO_RES = 0xFF
};
uint8_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__state = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__RES_CONTROLLED;
uint8_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__resId = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__default_owner_id;
uint8_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__reqResId;



static inline error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__Resource__request(uint8_t id);
#line 93
static inline error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__Resource__immediateRequest(uint8_t id);
#line 111
static inline error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__Resource__release(uint8_t id);
#line 133
static error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceDefaultOwner__release(void );
#line 153
static bool /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ArbiterInfo__inUse(void );
#line 166
static uint8_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ArbiterInfo__userId(void );










static bool /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__Resource__isOwner(uint8_t id);
#line 190
static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__grantedTask__runTask(void );
#line 202
static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__Resource__default__granted(uint8_t id);

static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceRequested__default__requested(uint8_t id);

static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceRequested__default__immediateRequested(uint8_t id);

static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceDefaultOwner__default__granted(void );

static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceDefaultOwner__default__requested(void );


static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceDefaultOwner__default__immediateRequested(void );


static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceConfigure__default__configure(uint8_t id);

static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceConfigure__default__unconfigure(uint8_t id);
# 97 "/home/csilab/tinyos/tos/chips/msp430/usart/HplMsp430Usart.nc"
static void HplMsp430I2C0P__HplUsart__resetUsart(bool reset);
# 49 "/home/csilab/tinyos/tos/chips/msp430/usart/HplMsp430I2C0P.nc"
static volatile uint8_t HplMsp430I2C0P__U0CTL __asm ("0x0070");





static inline bool HplMsp430I2C0P__HplI2C__isI2C(void );



static inline void HplMsp430I2C0P__HplI2C__clearModeI2C(void );
# 62 "/home/csilab/tinyos/tos/system/ActiveMessageAddressC.nc"
am_addr_t ActiveMessageAddressC__addr = TOS_AM_ADDRESS;


am_group_t ActiveMessageAddressC__group = TOS_AM_GROUP;






static inline am_addr_t ActiveMessageAddressC__ActiveMessageAddress__amAddress(void );
#line 93
static inline am_group_t ActiveMessageAddressC__ActiveMessageAddress__amGroup(void );
#line 106
static am_addr_t ActiveMessageAddressC__amAddress(void );
# 10 "/home/csilab/tinyos/tos/platforms/epic/chips/ds2411/OneWireStream.nc"
static error_t Ds2411P__OneWire__read(uint8_t cmd, uint8_t *buf, uint8_t len);
# 20 "/home/csilab/tinyos/tos/platforms/epic/chips/ds2411/Ds2411P.nc"
bool Ds2411P__haveId = FALSE;
dallasid48_serial_t Ds2411P__ds2411id;

static inline error_t Ds2411P__readId(void );
#line 36
static inline error_t Ds2411P__ReadId48__read(uint8_t *id);
# 66 "/home/csilab/tinyos/tos/lib/timer/BusyWait.nc"
static void OneWireMasterC__BusyWait__wait(OneWireMasterC__BusyWait__size_type dt);
# 44 "/home/csilab/tinyos/tos/interfaces/GeneralIO.nc"
static void OneWireMasterC__Pin__makeInput(void );
#line 43
static bool OneWireMasterC__Pin__get(void );


static void OneWireMasterC__Pin__makeOutput(void );
#line 41
static void OneWireMasterC__Pin__clr(void );
# 25 "/home/csilab/tinyos/tos/platforms/epic/chips/ds2411/OneWireMasterC.nc"
#line 18
typedef enum OneWireMasterC____nesc_unnamed4367 {
  OneWireMasterC__DELAY_5US = 5, 
  OneWireMasterC__RESET_LOW_TIME = 560, 
  OneWireMasterC__DELAY_60US = 60, 
  OneWireMasterC__PRESENCE_DETECT_LOW_TIME = 240, 
  OneWireMasterC__PRESENCE_RESET_HIGH_TIME = 480, 
  OneWireMasterC__SLOT_TIME = 65
} OneWireMasterC__onewiretimes_t;

static inline bool OneWireMasterC__reset(void );
#line 42
static inline void OneWireMasterC__writeOne(void );






static inline void OneWireMasterC__writeZero(void );






static inline bool OneWireMasterC__readBit(void );










static inline void OneWireMasterC__writeByte(uint8_t c);
#line 80
static inline uint8_t OneWireMasterC__readByte(void );










static inline error_t OneWireMasterC__OneWire__read(uint8_t cmd, uint8_t *buf, uint8_t len);
# 64 "/home/csilab/tinyos/tos/lib/timer/Counter.nc"
static /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__Counter__size_type /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__Counter__get(void );
# 58 "/home/csilab/tinyos/tos/lib/timer/BusyWaitCounterC.nc"
enum /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0____nesc_unnamed4368 {

  BusyWaitCounterC__0__HALF_MAX_SIZE_TYPE = (/*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__size_type )1 << (8 * sizeof(/*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__size_type ) - 1)
};

static void /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__BusyWait__wait(/*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__size_type dt);
#line 83
static inline void /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__Counter__overflow(void );
# 45 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430Timer.nc"
static uint16_t /*Msp430CounterMicroC.Counter*/Msp430CounterC__1__Msp430Timer__get(void );
# 82 "/home/csilab/tinyos/tos/lib/timer/Counter.nc"
static void /*Msp430CounterMicroC.Counter*/Msp430CounterC__1__Counter__overflow(void );
# 49 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430CounterC.nc"
static inline uint16_t /*Msp430CounterMicroC.Counter*/Msp430CounterC__1__Counter__get(void );
#line 64
static inline void /*Msp430CounterMicroC.Counter*/Msp430CounterC__1__Msp430Timer__overflow(void );
# 78 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*Ds2411C.Gpio*/Msp430GpioC__11__HplGeneralIO__makeInput(void );






static void /*Ds2411C.Gpio*/Msp430GpioC__11__HplGeneralIO__makeOutput(void );
#line 73
static bool /*Ds2411C.Gpio*/Msp430GpioC__11__HplGeneralIO__get(void );
#line 53
static void /*Ds2411C.Gpio*/Msp430GpioC__11__HplGeneralIO__clr(void );
# 49 "/home/csilab/tinyos/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*Ds2411C.Gpio*/Msp430GpioC__11__GeneralIO__clr(void );

static inline bool /*Ds2411C.Gpio*/Msp430GpioC__11__GeneralIO__get(void );
static inline void /*Ds2411C.Gpio*/Msp430GpioC__11__GeneralIO__makeInput(void );

static inline void /*Ds2411C.Gpio*/Msp430GpioC__11__GeneralIO__makeOutput(void );
# 12 "/home/csilab/tinyos/tos/platforms/epic/chips/ds2411/ReadId48.nc"
static error_t DallasId48ToIeeeEui64C__ReadId48__read(uint8_t *id);
# 8 "/home/csilab/tinyos/tos/platforms/epic/chips/ds2411/DallasId48ToIeeeEui64C.nc"
static ieee_eui64_t DallasId48ToIeeeEui64C__LocalIeeeEui64__getId(void );
# 70 "/home/csilab/tinyos/tos/interfaces/PacketTimeStamp.nc"
static void CC2420TransmitP__PacketTimeStamp__clear(
#line 66
message_t * msg);
#line 78
static void CC2420TransmitP__PacketTimeStamp__set(
#line 73
message_t * msg, 




CC2420TransmitP__PacketTimeStamp__size_type value);
# 53 "/home/csilab/tinyos/tos/chips/cc2420/interfaces/CC2420Strobe.nc"
static cc2420_status_t CC2420TransmitP__STXONCCA__strobe(void );
# 54 "/home/csilab/tinyos/tos/interfaces/GpioCapture.nc"
static error_t CC2420TransmitP__CaptureSFD__captureFallingEdge(void );
#line 66
static void CC2420TransmitP__CaptureSFD__disable(void );
#line 53
static error_t CC2420TransmitP__CaptureSFD__captureRisingEdge(void );
# 50 "/home/csilab/upma/interfaces/CcaControl.nc"
static uint16_t CC2420TransmitP__CcaControl__getCongestionBackoff(
# 77 "/home/csilab/upma/chips/cc2420/CC2420TransmitP.nc"
am_id_t arg_0x7f7fec8771d0, 
# 50 "/home/csilab/upma/interfaces/CcaControl.nc"
message_t *msg, uint16_t defaultBackoff);
#line 39
static uint16_t CC2420TransmitP__CcaControl__getInitialBackoff(
# 77 "/home/csilab/upma/chips/cc2420/CC2420TransmitP.nc"
am_id_t arg_0x7f7fec8771d0, 
# 39 "/home/csilab/upma/interfaces/CcaControl.nc"
message_t *msg, uint16_t defaultbackoff);
#line 61
static bool CC2420TransmitP__CcaControl__getCca(
# 77 "/home/csilab/upma/chips/cc2420/CC2420TransmitP.nc"
am_id_t arg_0x7f7fec8771d0, 
# 61 "/home/csilab/upma/interfaces/CcaControl.nc"
message_t *msg, bool defaultCca);
# 109 "/home/csilab/tinyos/tos/lib/timer/Alarm.nc"
static CC2420TransmitP__BackoffTimer__size_type CC2420TransmitP__BackoffTimer__getNow(void );
#line 66
static void CC2420TransmitP__BackoffTimer__start(CC2420TransmitP__BackoffTimer__size_type dt);






static void CC2420TransmitP__BackoffTimer__stop(void );
# 63 "/home/csilab/tinyos/tos/chips/cc2420/interfaces/CC2420Ram.nc"
static cc2420_status_t CC2420TransmitP__TXFIFO_RAM__write(uint8_t offset, uint8_t * data, uint8_t length);
# 63 "/home/csilab/tinyos/tos/chips/cc2420/interfaces/CC2420Register.nc"
static cc2420_status_t CC2420TransmitP__TXCTRL__write(uint16_t data);
# 91 "/home/csilab/upma/chips/cc2420/CC2420Transmit.nc"
static void CC2420TransmitP__Send__sendDone(message_t * p_msg, error_t error);
# 31 "/home/csilab/tinyos/tos/chips/cc2420/interfaces/ChipSpiResource.nc"
static void CC2420TransmitP__ChipSpiResource__abortRelease(void );







static error_t CC2420TransmitP__ChipSpiResource__attemptRelease(void );
# 53 "/home/csilab/tinyos/tos/chips/cc2420/interfaces/CC2420Strobe.nc"
static cc2420_status_t CC2420TransmitP__SFLUSHTX__strobe(void );
# 55 "/home/csilab/tinyos/tos/chips/cc2420/interfaces/CC2420Receive.nc"
static void CC2420TransmitP__CC2420Receive__sfd_dropped(void );
#line 49
static void CC2420TransmitP__CC2420Receive__sfd(uint32_t time);
# 52 "/home/csilab/tinyos/tos/interfaces/Random.nc"
static uint16_t CC2420TransmitP__Random__rand16(void );
# 46 "/home/csilab/tinyos/tos/interfaces/GeneralIO.nc"
static void CC2420TransmitP__CSN__makeOutput(void );
#line 40
static void CC2420TransmitP__CSN__set(void );
static void CC2420TransmitP__CSN__clr(void );
# 42 "/home/csilab/tinyos/tos/chips/cc2420/interfaces/CC2420PacketBody.nc"
static cc2420_header_t * CC2420TransmitP__CC2420PacketBody__getHeader(message_t * msg);










static cc2420_metadata_t * CC2420TransmitP__CC2420PacketBody__getMetadata(message_t * msg);
# 58 "/home/csilab/tinyos/tos/chips/cc2420/interfaces/PacketTimeSyncOffset.nc"
static uint8_t CC2420TransmitP__PacketTimeSyncOffset__get(
#line 53
message_t * msg);
#line 50
static bool CC2420TransmitP__PacketTimeSyncOffset__isSet(
#line 46
message_t * msg);
# 120 "/home/csilab/tinyos/tos/interfaces/Resource.nc"
static error_t CC2420TransmitP__SpiResource__release(void );
#line 97
static error_t CC2420TransmitP__SpiResource__immediateRequest(void );
#line 88
static error_t CC2420TransmitP__SpiResource__request(void );
# 44 "/home/csilab/tinyos/tos/interfaces/GeneralIO.nc"
static void CC2420TransmitP__CCA__makeInput(void );
#line 43
static bool CC2420TransmitP__CCA__get(void );
# 53 "/home/csilab/tinyos/tos/chips/cc2420/interfaces/CC2420Strobe.nc"
static cc2420_status_t CC2420TransmitP__SNOP__strobe(void );
# 44 "/home/csilab/tinyos/tos/interfaces/GeneralIO.nc"
static void CC2420TransmitP__SFD__makeInput(void );
#line 43
static bool CC2420TransmitP__SFD__get(void );
# 82 "/home/csilab/tinyos/tos/chips/cc2420/interfaces/CC2420Fifo.nc"
static cc2420_status_t CC2420TransmitP__TXFIFO__write(uint8_t * data, uint8_t length);
# 53 "/home/csilab/tinyos/tos/chips/cc2420/interfaces/CC2420Strobe.nc"
static cc2420_status_t CC2420TransmitP__STXON__strobe(void );
# 124 "/home/csilab/upma/chips/cc2420/CC2420TransmitP.nc"
#line 114
typedef enum CC2420TransmitP____nesc_unnamed4369 {
  CC2420TransmitP__S_STOPPED, 
  CC2420TransmitP__S_STARTED, 
  CC2420TransmitP__S_LOAD, 
  CC2420TransmitP__S_SAMPLE_CCA, 
  CC2420TransmitP__S_BEGIN_TRANSMIT, 
  CC2420TransmitP__S_SFD, 
  CC2420TransmitP__S_EFD, 
  CC2420TransmitP__S_ACK_WAIT, 
  CC2420TransmitP__S_CANCEL
} CC2420TransmitP__cc2420_transmit_state_t;





enum CC2420TransmitP____nesc_unnamed4370 {
  CC2420TransmitP__CC2420_ABORT_PERIOD = 320
};

message_t * CC2420TransmitP__m_msg;

bool CC2420TransmitP__m_cca;

uint8_t CC2420TransmitP__m_tx_power;

CC2420TransmitP__cc2420_transmit_state_t CC2420TransmitP__m_state = CC2420TransmitP__S_STOPPED;

bool CC2420TransmitP__m_receiving = FALSE;

uint16_t CC2420TransmitP__m_prev_time;


bool CC2420TransmitP__sfdHigh;


bool CC2420TransmitP__abortSpiRelease;


int8_t CC2420TransmitP__totalCcaChecks;


uint16_t CC2420TransmitP__myInitialBackoff;


uint16_t CC2420TransmitP__myCongestionBackoff;



static inline error_t CC2420TransmitP__send(message_t * p_msg, bool cca);

static void CC2420TransmitP__loadTXFIFO(void );
static void CC2420TransmitP__attemptSend(void );
static inline void CC2420TransmitP__requestInitialBackoff(message_t * msg);
static inline void CC2420TransmitP__requestCongestionBackoff(message_t * msg);
static void CC2420TransmitP__congestionBackoff(void );
static error_t CC2420TransmitP__acquireSpiResource(void );
static inline error_t CC2420TransmitP__releaseSpiResource(void );
static void CC2420TransmitP__signalDone(error_t err);
static inline uint16_t CC2420TransmitP__defaultInitialBackoff(void );
static inline uint16_t CC2420TransmitP__defaultCongestionBackoff(void );
static inline bool CC2420TransmitP__defaultCca(void );



static inline error_t CC2420TransmitP__Init__init(void );







static inline error_t CC2420TransmitP__AsyncStdControl__start(void );










static inline error_t CC2420TransmitP__AsyncStdControl__stop(void );
#line 211
static inline am_id_t CC2420TransmitP__getType(message_t * p_msg);




static inline error_t CC2420TransmitP__Send__send(message_t * p_msg);
#line 251
static inline bool CC2420TransmitP__EnergyIndicator__isReceiving(void );










static __inline uint32_t CC2420TransmitP__getTime32(uint16_t time);
#line 281
static inline void CC2420TransmitP__CaptureSFD__captured(uint16_t time);
#line 377
static inline void CC2420TransmitP__ChipSpiResource__releasing(void );
#line 389
static inline void CC2420TransmitP__CC2420Receive__receive(uint8_t type, message_t *ack_msg);
#line 416
static inline void CC2420TransmitP__SpiResource__granted(void );
#line 454
static inline void CC2420TransmitP__TXFIFO__writeDone(uint8_t *tx_buf, uint8_t tx_len, 
error_t error);
#line 486
static inline void CC2420TransmitP__TXFIFO__readDone(uint8_t *tx_buf, uint8_t tx_len, 
error_t error);










static inline void CC2420TransmitP__BackoffTimer__fired(void );
#line 547
static inline error_t CC2420TransmitP__send(message_t * p_msg, bool cca);
#line 615
static void CC2420TransmitP__attemptSend(void );
#line 653
static inline void CC2420TransmitP__requestInitialBackoff(message_t *msg);



static inline void CC2420TransmitP__requestCongestionBackoff(message_t *msg);






static void CC2420TransmitP__congestionBackoff(void );






static error_t CC2420TransmitP__acquireSpiResource(void );







static inline error_t CC2420TransmitP__releaseSpiResource(void );
#line 701
static void CC2420TransmitP__loadTXFIFO(void );
#line 726
static void CC2420TransmitP__signalDone(error_t err);
#line 759
static inline uint16_t CC2420TransmitP__defaultInitialBackoff(void );




static inline uint16_t CC2420TransmitP__defaultCongestionBackoff(void );



static inline bool CC2420TransmitP__defaultCca(void );
# 43 "/home/csilab/tinyos/tos/interfaces/GeneralIO.nc"
static bool CC2420ReceiveP__FIFO__get(void );
# 93 "/home/csilab/tinyos/tos/chips/cc2420/interfaces/CC2420Config.nc"
static bool CC2420ReceiveP__CC2420Config__isAddressRecognitionEnabled(void );
#line 117
static bool CC2420ReceiveP__CC2420Config__isAutoAckEnabled(void );
#line 112
static bool CC2420ReceiveP__CC2420Config__isHwAutoAckDefault(void );
#line 71
static uint16_t CC2420ReceiveP__CC2420Config__getShortAddr(void );
# 70 "/home/csilab/tinyos/tos/interfaces/PacketTimeStamp.nc"
static void CC2420ReceiveP__PacketTimeStamp__clear(
#line 66
message_t * msg);
#line 78
static void CC2420ReceiveP__PacketTimeStamp__set(
#line 73
message_t * msg, 




CC2420ReceiveP__PacketTimeStamp__size_type value);
# 43 "/home/csilab/tinyos/tos/interfaces/GeneralIO.nc"
static bool CC2420ReceiveP__FIFOP__get(void );
# 63 "/home/csilab/tinyos/tos/chips/cc2420/interfaces/CC2420Receive.nc"
static void CC2420ReceiveP__CC2420Receive__receive(uint8_t type, message_t * message);
# 53 "/home/csilab/tinyos/tos/chips/cc2420/interfaces/CC2420Strobe.nc"
static cc2420_status_t CC2420ReceiveP__SACK__strobe(void );
# 40 "/home/csilab/tinyos/tos/interfaces/GeneralIO.nc"
static void CC2420ReceiveP__CSN__set(void );
static void CC2420ReceiveP__CSN__clr(void );
# 42 "/home/csilab/tinyos/tos/chips/cc2420/interfaces/CC2420PacketBody.nc"
static cc2420_header_t * CC2420ReceiveP__CC2420PacketBody__getHeader(message_t * msg);










static cc2420_metadata_t * CC2420ReceiveP__CC2420PacketBody__getMetadata(message_t * msg);
# 41 "/home/csilab/upma/interfaces/AsyncReceive.nc"
static void CC2420ReceiveP__Receive__receive(
#line 37
message_t * msg, 
void * payload, 


uint8_t len);
# 120 "/home/csilab/tinyos/tos/interfaces/Resource.nc"
static error_t CC2420ReceiveP__SpiResource__release(void );
#line 97
static error_t CC2420ReceiveP__SpiResource__immediateRequest(void );
#line 88
static error_t CC2420ReceiveP__SpiResource__request(void );
#line 128
static bool CC2420ReceiveP__SpiResource__isOwner(void );
# 62 "/home/csilab/tinyos/tos/chips/cc2420/interfaces/CC2420Fifo.nc"
static error_t CC2420ReceiveP__RXFIFO__continueRead(uint8_t * data, uint8_t length);
#line 51
static cc2420_status_t CC2420ReceiveP__RXFIFO__beginRead(uint8_t * data, uint8_t length);
# 44 "/home/csilab/tinyos/tos/chips/cc2420/interfaces/ReceiveIndicator.nc"
static void CC2420ReceiveP__PacketIndicator__anypktreceive(void );
# 61 "/home/csilab/tinyos/tos/interfaces/GpioInterrupt.nc"
static error_t CC2420ReceiveP__InterruptFIFOP__disable(void );
#line 54
static error_t CC2420ReceiveP__InterruptFIFOP__enableFallingEdge(void );
# 53 "/home/csilab/tinyos/tos/chips/cc2420/interfaces/CC2420Strobe.nc"
static cc2420_status_t CC2420ReceiveP__SFLUSHRX__strobe(void );
# 85 "/home/csilab/upma/chips/cc2420/CC2420ReceiveP.nc"
#line 79
typedef enum CC2420ReceiveP____nesc_unnamed4371 {
  CC2420ReceiveP__S_STOPPED, 
  CC2420ReceiveP__S_STARTED, 
  CC2420ReceiveP__S_RX_LENGTH, 
  CC2420ReceiveP__S_RX_FCF, 
  CC2420ReceiveP__S_RX_PAYLOAD
} CC2420ReceiveP__cc2420_receive_state_t;

enum CC2420ReceiveP____nesc_unnamed4372 {
  CC2420ReceiveP__RXFIFO_SIZE = 128, 
  CC2420ReceiveP__TIMESTAMP_QUEUE_SIZE = 8, 
  CC2420ReceiveP__SACK_HEADER_LENGTH = 7
};

uint32_t CC2420ReceiveP__m_timestamp_queue[CC2420ReceiveP__TIMESTAMP_QUEUE_SIZE];

uint8_t CC2420ReceiveP__m_timestamp_head;

uint8_t CC2420ReceiveP__m_timestamp_size;


uint8_t CC2420ReceiveP__m_missed_packets;


bool CC2420ReceiveP__receivingPacket;


uint8_t CC2420ReceiveP__rxFrameLength;

uint8_t CC2420ReceiveP__m_bytes_left;

message_t * CC2420ReceiveP__m_p_rx_buf;

message_t CC2420ReceiveP__m_rx_buf;

CC2420ReceiveP__cc2420_receive_state_t CC2420ReceiveP__m_state;


static void CC2420ReceiveP__reset_state(void );
static void CC2420ReceiveP__beginReceive(void );
static void CC2420ReceiveP__receive(void );
static void CC2420ReceiveP__waitForNextPacket(void );
static void CC2420ReceiveP__flush(void );
static inline bool CC2420ReceiveP__passesAddressCheck(message_t * msg);

static inline void CC2420ReceiveP__receiveDone(void );


static inline error_t CC2420ReceiveP__Init__init(void );





static inline error_t CC2420ReceiveP__StdControl__start(void );
#line 147
static inline error_t CC2420ReceiveP__StdControl__stop(void );
#line 162
static inline void CC2420ReceiveP__CC2420Receive__sfd(uint32_t time);








static inline void CC2420ReceiveP__CC2420Receive__sfd_dropped(void );






static inline bool CC2420ReceiveP__PacketIndicator__isReceiving(void );









static inline void CC2420ReceiveP__InterruptFIFOP__fired(void );










static inline void CC2420ReceiveP__SpiResource__granted(void );








static inline void CC2420ReceiveP__RXFIFO__readDone(uint8_t *rx_buf, uint8_t rx_len, 
error_t error);
#line 339
static inline void CC2420ReceiveP__RXFIFO__writeDone(uint8_t *tx_buf, uint8_t tx_len, error_t error);






static inline void CC2420ReceiveP__receiveDone(void );
#line 366
static void CC2420ReceiveP__Receive__updateBuffer(message_t *msg);






static inline void CC2420ReceiveP__CC2420Config__syncDone(error_t error);






static void CC2420ReceiveP__beginReceive(void );
#line 397
static void CC2420ReceiveP__flush(void );
#line 415
static void CC2420ReceiveP__receive(void );










static void CC2420ReceiveP__waitForNextPacket(void );
#line 465
static void CC2420ReceiveP__reset_state(void );










static inline bool CC2420ReceiveP__passesAddressCheck(message_t *msg);
# 66 "/home/csilab/upma/chips/cc2420/CC2420PacketP.nc"
static inline error_t CC2420PacketP__Acks__requestAck(message_t *p_msg);









static inline bool CC2420PacketP__Acks__wasAcked(message_t *p_msg);
#line 172
static inline uint8_t CC2420PacketP__CC2420Packet__getNetwork(message_t *p_msg);







static inline void CC2420PacketP__CC2420Packet__setNetwork(message_t *p_msg, uint8_t networkId);







static inline cc2420_header_t * CC2420PacketP__CC2420PacketBody__getHeader(message_t * msg);



static inline cc2420_metadata_t *CC2420PacketP__CC2420PacketBody__getMetadata(message_t *msg);
#line 211
static void CC2420PacketP__PacketTimeStamp32khz__clear(message_t *msg);





static inline void CC2420PacketP__PacketTimeStamp32khz__set(message_t *msg, uint32_t value);
#line 250
static inline bool CC2420PacketP__PacketTimeSyncOffset__isSet(message_t *msg);








static inline uint8_t CC2420PacketP__PacketTimeSyncOffset__get(message_t *msg);
# 58 "/home/csilab/tinyos/tos/lib/timer/CounterToLocalTimeC.nc"
static inline void /*CC2420PacketC.CounterToLocalTimeC*/CounterToLocalTimeC__0__Counter__overflow(void );
# 41 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430Compare__setEvent(uint16_t time);

static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430Compare__setEventFromNow(uint16_t delta);
# 45 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430Timer.nc"
static uint16_t /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430Timer__get(void );
# 78 "/home/csilab/tinyos/tos/lib/timer/Alarm.nc"
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Alarm__fired(void );
# 57 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430TimerControl.nc"
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430TimerControl__enableEvents(void );
#line 47
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430TimerControl__setControlAsCompare(void );










static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430TimerControl__disableEvents(void );
#line 44
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430TimerControl__clearPendingInterrupt(void );
# 53 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline error_t /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Init__init(void );
#line 65
static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Alarm__stop(void );




static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430Compare__fired(void );










static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Alarm__startAt(uint16_t t0, uint16_t dt);
#line 114
static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430Timer__overflow(void );
# 64 "/home/csilab/tinyos/tos/lib/timer/Counter.nc"
static /*CounterMilli32C.Transform*/TransformCounterC__1__CounterFrom__size_type /*CounterMilli32C.Transform*/TransformCounterC__1__CounterFrom__get(void );






static bool /*CounterMilli32C.Transform*/TransformCounterC__1__CounterFrom__isOverflowPending(void );










static void /*CounterMilli32C.Transform*/TransformCounterC__1__Counter__overflow(void );
# 67 "/home/csilab/tinyos/tos/lib/timer/TransformCounterC.nc"
/*CounterMilli32C.Transform*/TransformCounterC__1__upper_count_type /*CounterMilli32C.Transform*/TransformCounterC__1__m_upper;

enum /*CounterMilli32C.Transform*/TransformCounterC__1____nesc_unnamed4373 {

  TransformCounterC__1__LOW_SHIFT_RIGHT = 5, 
  TransformCounterC__1__HIGH_SHIFT_LEFT = 8 * sizeof(/*CounterMilli32C.Transform*/TransformCounterC__1__from_size_type ) - /*CounterMilli32C.Transform*/TransformCounterC__1__LOW_SHIFT_RIGHT, 
  TransformCounterC__1__NUM_UPPER_BITS = 8 * sizeof(/*CounterMilli32C.Transform*/TransformCounterC__1__to_size_type ) - 8 * sizeof(/*CounterMilli32C.Transform*/TransformCounterC__1__from_size_type ) + 5, 



  TransformCounterC__1__OVERFLOW_MASK = /*CounterMilli32C.Transform*/TransformCounterC__1__NUM_UPPER_BITS ? ((/*CounterMilli32C.Transform*/TransformCounterC__1__upper_count_type )2 << (/*CounterMilli32C.Transform*/TransformCounterC__1__NUM_UPPER_BITS - 1)) - 1 : 0
};

static /*CounterMilli32C.Transform*/TransformCounterC__1__to_size_type /*CounterMilli32C.Transform*/TransformCounterC__1__Counter__get(void );
#line 133
static inline void /*CounterMilli32C.Transform*/TransformCounterC__1__CounterFrom__overflow(void );
# 78 "/home/csilab/tinyos/tos/lib/timer/Alarm.nc"
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__Alarm__fired(void );
#line 103
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__AlarmFrom__startAt(/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__AlarmFrom__size_type t0, /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__AlarmFrom__size_type dt);
#line 73
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__AlarmFrom__stop(void );
# 64 "/home/csilab/tinyos/tos/lib/timer/Counter.nc"
static /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__Counter__size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__Counter__get(void );
# 77 "/home/csilab/tinyos/tos/lib/timer/TransformAlarmC.nc"
/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__m_t0;
/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__m_dt;

enum /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1____nesc_unnamed4374 {

  TransformAlarmC__1__MAX_DELAY_LOG2 = 8 * sizeof(/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__from_size_type ) - 1 - 5, 
  TransformAlarmC__1__MAX_DELAY = (/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__to_size_type )1 << /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__MAX_DELAY_LOG2
};

static inline /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__Alarm__getNow(void );




static inline /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__Alarm__getAlarm(void );










static inline void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__Alarm__stop(void );




static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__set_alarm(void );
#line 147
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__Alarm__startAt(/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__to_size_type t0, /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__to_size_type dt);
#line 162
static inline void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__AlarmFrom__fired(void );
#line 177
static inline void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__Counter__overflow(void );
# 67 "/home/csilab/tinyos/tos/interfaces/TaskBasic.nc"
static error_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired__postTask(void );
# 109 "/home/csilab/tinyos/tos/lib/timer/Alarm.nc"
static /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__getNow(void );
#line 103
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__startAt(/*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type t0, /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type dt);
#line 116
static /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__getAlarm(void );
#line 73
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__stop(void );
# 83 "/home/csilab/tinyos/tos/lib/timer/Timer.nc"
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__fired(void );
# 74 "/home/csilab/tinyos/tos/lib/timer/AlarmToTimerC.nc"
enum /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0____nesc_unnamed4375 {
#line 74
  AlarmToTimerC__0__fired = 19U
};
#line 74
typedef int /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0____nesc_sillytask_fired[/*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired];
#line 55
uint32_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__m_dt;
bool /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__m_oneshot;

static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__start(uint32_t t0, uint32_t dt, bool oneshot);
#line 71
static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__stop(void );


static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired__runTask(void );






static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__fired(void );
#line 93
static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__startOneShotAt(uint32_t t0, uint32_t dt);


static inline uint32_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__getNow(void );
# 67 "/home/csilab/tinyos/tos/interfaces/TaskBasic.nc"
static error_t /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer__postTask(void );
# 136 "/home/csilab/tinyos/tos/lib/timer/Timer.nc"
static uint32_t /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__getNow(void );
#line 129
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__startOneShotAt(uint32_t t0, uint32_t dt);
#line 78
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__stop(void );




static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__fired(
# 48 "/home/csilab/tinyos/tos/lib/timer/VirtualizeTimerC.nc"
uint8_t arg_0x7f7fec5fd950);
#line 71
enum /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0____nesc_unnamed4376 {
#line 71
  VirtualizeTimerC__0__updateFromTimer = 20U
};
#line 71
typedef int /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0____nesc_sillytask_updateFromTimer[/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer];
#line 53
enum /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0____nesc_unnamed4377 {

  VirtualizeTimerC__0__NUM_TIMERS = 5U, 
  VirtualizeTimerC__0__END_OF_LIST = 255
};








#line 59
typedef struct /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0____nesc_unnamed4378 {

  uint32_t t0;
  uint32_t dt;
  bool isoneshot : 1;
  bool isrunning : 1;
  bool _reserved : 6;
} /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer_t;

/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer_t /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__m_timers[/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__NUM_TIMERS];




static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__fireTimers(uint32_t now);
#line 100
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer__runTask(void );
#line 139
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__fired(void );




static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__startTimer(uint8_t num, uint32_t t0, uint32_t dt, bool isoneshot);









static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startPeriodic(uint8_t num, uint32_t dt);




static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startOneShot(uint8_t num, uint32_t dt);
#line 204
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__default__fired(uint8_t num);
# 58 "/home/csilab/tinyos/tos/lib/timer/CounterToLocalTimeC.nc"
static inline void /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__1__Counter__overflow(void );
# 52 "/home/csilab/tinyos/tos/system/RandomMlcgC.nc"
uint32_t RandomMlcgC__seed;


static inline error_t RandomMlcgC__Init__init(void );
#line 69
static uint32_t RandomMlcgC__Random__rand32(void );
#line 89
static inline uint16_t RandomMlcgC__Random__rand16(void );
# 75 "/home/csilab/tinyos/tos/interfaces/Send.nc"
static error_t UniqueSendP__SubSend__send(
#line 67
message_t * msg, 







uint8_t len);
#line 112
static uint8_t UniqueSendP__SubSend__maxPayloadLength(void );
#line 100
static void UniqueSendP__Send__sendDone(
#line 96
message_t * msg, 



error_t error);
# 52 "/home/csilab/tinyos/tos/interfaces/Random.nc"
static uint16_t UniqueSendP__Random__rand16(void );
# 42 "/home/csilab/tinyos/tos/chips/cc2420/interfaces/CC2420PacketBody.nc"
static cc2420_header_t * UniqueSendP__CC2420PacketBody__getHeader(message_t * msg);
# 56 "/home/csilab/tinyos/tos/interfaces/State.nc"
static void UniqueSendP__State__toIdle(void );
#line 45
static error_t UniqueSendP__State__requestState(uint8_t reqState);
# 54 "/home/csilab/tinyos/tos/chips/cc2420/unique/UniqueSendP.nc"
uint8_t UniqueSendP__localSendId;

enum UniqueSendP____nesc_unnamed4379 {
  UniqueSendP__S_IDLE, 
  UniqueSendP__S_SENDING
};


static inline error_t UniqueSendP__Init__init(void );
#line 75
static inline error_t UniqueSendP__Send__send(message_t *msg, uint8_t len);
#line 95
static inline uint8_t UniqueSendP__Send__maxPayloadLength(void );








static inline void UniqueSendP__SubSend__sendDone(message_t *msg, error_t error);
# 78 "/home/csilab/tinyos/tos/interfaces/Receive.nc"
static 
#line 74
message_t * 



UniqueReceiveP__Receive__receive(
#line 71
message_t * msg, 
void * payload, 





uint8_t len);
# 42 "/home/csilab/tinyos/tos/chips/cc2420/interfaces/CC2420PacketBody.nc"
static cc2420_header_t * UniqueReceiveP__CC2420PacketBody__getHeader(message_t * msg);
# 78 "/home/csilab/tinyos/tos/interfaces/Receive.nc"
static 
#line 74
message_t * 



UniqueReceiveP__DuplicateReceive__receive(
#line 71
message_t * msg, 
void * payload, 





uint8_t len);
# 59 "/home/csilab/tinyos/tos/chips/cc2420/unique/UniqueReceiveP.nc"
#line 56
struct UniqueReceiveP____nesc_unnamed4380 {
  uint16_t source;
  uint8_t dsn;
} UniqueReceiveP__receivedMessages[4];

uint8_t UniqueReceiveP__writeIndex = 0;


uint8_t UniqueReceiveP__recycleSourceElement;

enum UniqueReceiveP____nesc_unnamed4381 {
  UniqueReceiveP__INVALID_ELEMENT = 0xFF
};


static inline error_t UniqueReceiveP__Init__init(void );









static inline bool UniqueReceiveP__hasSeen(uint16_t msgSource, uint8_t msgDsn);
static inline void UniqueReceiveP__insert(uint16_t msgSource, uint8_t msgDsn);
static inline uint16_t UniqueReceiveP__getSourceKey(message_t  *msg);


static inline message_t *UniqueReceiveP__SubReceive__receive(message_t *msg, void *payload, 
uint8_t len);
#line 112
static inline bool UniqueReceiveP__hasSeen(uint16_t msgSource, uint8_t msgDsn);
#line 138
static inline void UniqueReceiveP__insert(uint16_t msgSource, uint8_t msgDsn);
#line 165
static inline uint16_t UniqueReceiveP__getSourceKey(message_t * msg);
#line 192
static inline message_t *UniqueReceiveP__DuplicateReceive__default__receive(message_t *msg, void *payload, uint8_t len);
# 75 "/home/csilab/tinyos/tos/interfaces/Send.nc"
static error_t CC2420TinyosNetworkP__SubSend__send(
#line 67
message_t * msg, 







uint8_t len);
#line 112
static uint8_t CC2420TinyosNetworkP__SubSend__maxPayloadLength(void );
# 67 "/home/csilab/tinyos/tos/interfaces/TaskBasic.nc"
static error_t CC2420TinyosNetworkP__grantTask__postTask(void );
# 77 "/home/csilab/tinyos/tos/chips/cc2420/interfaces/CC2420Packet.nc"
static void CC2420TinyosNetworkP__CC2420Packet__setNetwork(message_t * p_msg, uint8_t networkId);
#line 75
static uint8_t CC2420TinyosNetworkP__CC2420Packet__getNetwork(message_t * p_msg);
# 100 "/home/csilab/tinyos/tos/interfaces/Send.nc"
static void CC2420TinyosNetworkP__ActiveSend__sendDone(
#line 96
message_t * msg, 



error_t error);
# 79 "/home/csilab/tinyos/tos/interfaces/ResourceQueue.nc"
static error_t CC2420TinyosNetworkP__Queue__enqueue(resource_client_id_t id);
#line 53
static bool CC2420TinyosNetworkP__Queue__isEmpty(void );
#line 70
static resource_client_id_t CC2420TinyosNetworkP__Queue__dequeue(void );
# 42 "/home/csilab/tinyos/tos/chips/cc2420/interfaces/CC2420PacketBody.nc"
static cc2420_header_t * CC2420TinyosNetworkP__CC2420PacketBody__getHeader(message_t * msg);










static cc2420_metadata_t * CC2420TinyosNetworkP__CC2420PacketBody__getMetadata(message_t * msg);
# 78 "/home/csilab/tinyos/tos/interfaces/Receive.nc"
static 
#line 74
message_t * 



CC2420TinyosNetworkP__BareReceive__receive(
#line 71
message_t * msg, 
void * payload, 





uint8_t len);
# 102 "/home/csilab/tinyos/tos/interfaces/Resource.nc"
static void CC2420TinyosNetworkP__Resource__granted(
# 46 "/home/csilab/tinyos/tos/chips/cc2420/lowpan/CC2420TinyosNetworkP.nc"
uint8_t arg_0x7f7fec4cb600);
# 100 "/home/csilab/tinyos/tos/interfaces/Send.nc"
static void CC2420TinyosNetworkP__BareSend__sendDone(
#line 96
message_t * msg, 



error_t error);
# 78 "/home/csilab/tinyos/tos/interfaces/Receive.nc"
static 
#line 74
message_t * 



CC2420TinyosNetworkP__ActiveReceive__receive(
#line 71
message_t * msg, 
void * payload, 





uint8_t len);
# 180 "/home/csilab/tinyos/tos/chips/cc2420/lowpan/CC2420TinyosNetworkP.nc"
enum CC2420TinyosNetworkP____nesc_unnamed4382 {
#line 180
  CC2420TinyosNetworkP__grantTask = 21U
};
#line 180
typedef int CC2420TinyosNetworkP____nesc_sillytask_grantTask[CC2420TinyosNetworkP__grantTask];
#line 68
enum CC2420TinyosNetworkP____nesc_unnamed4383 {
  CC2420TinyosNetworkP__OWNER_NONE = 0xff, 
  CC2420TinyosNetworkP__TINYOS_N_NETWORKS = 0U
};




#line 73
enum CC2420TinyosNetworkP____nesc_unnamed4384 {
  CC2420TinyosNetworkP__CLIENT_AM, 
  CC2420TinyosNetworkP__CLIENT_BARE
} CC2420TinyosNetworkP__m_busy_client;

uint8_t CC2420TinyosNetworkP__resource_owner = CC2420TinyosNetworkP__OWNER_NONE;
#line 78
uint8_t CC2420TinyosNetworkP__next_owner;

static error_t CC2420TinyosNetworkP__ActiveSend__send(message_t *msg, uint8_t len);









static inline uint8_t CC2420TinyosNetworkP__ActiveSend__maxPayloadLength(void );



static void *CC2420TinyosNetworkP__ActiveSend__getPayload(message_t *msg, uint8_t len);
#line 138
static inline void *CC2420TinyosNetworkP__BareSend__getPayload(message_t *msg, uint8_t len);









static inline void CC2420TinyosNetworkP__SubSend__sendDone(message_t *msg, error_t error);








static inline message_t *CC2420TinyosNetworkP__SubReceive__receive(message_t *msg, void *payload, uint8_t len);
#line 180
static inline void CC2420TinyosNetworkP__grantTask__runTask(void );
#line 199
static inline error_t CC2420TinyosNetworkP__Resource__request(uint8_t id);
#line 215
static inline error_t CC2420TinyosNetworkP__Resource__immediateRequest(uint8_t id);
#line 229
static inline error_t CC2420TinyosNetworkP__Resource__release(uint8_t id);
#line 241
static inline message_t *CC2420TinyosNetworkP__BareReceive__default__receive(message_t *msg, void *payload, uint8_t len);


static inline void CC2420TinyosNetworkP__BareSend__default__sendDone(message_t *msg, error_t error);








static inline void CC2420TinyosNetworkP__Resource__default__granted(uint8_t client);
# 49 "/home/csilab/tinyos/tos/system/FcfsResourceQueueC.nc"
enum /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0____nesc_unnamed4385 {
#line 49
  FcfsResourceQueueC__0__NO_ENTRY = 0xFF
};
uint8_t /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__resQ[0];
uint8_t /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__qHead = /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__NO_ENTRY;
uint8_t /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__qTail = /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__NO_ENTRY;

static inline error_t /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__Init__init(void );




static bool /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__FcfsQueue__isEmpty(void );



static inline bool /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__FcfsQueue__isEnqueued(resource_client_id_t id);



static inline resource_client_id_t /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__FcfsQueue__dequeue(void );
#line 82
static inline error_t /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__FcfsQueue__enqueue(resource_client_id_t id);
# 67 "/home/csilab/tinyos/tos/interfaces/TaskBasic.nc"
static error_t AsyncReceiveAdapterP__receiveDone_task__postTask(void );
# 78 "/home/csilab/tinyos/tos/interfaces/Receive.nc"
static 
#line 74
message_t * 



AsyncReceiveAdapterP__Receive__receive(
#line 71
message_t * msg, 
void * payload, 





uint8_t len);
# 52 "/home/csilab/upma/interfaces/AsyncReceive.nc"
static void AsyncReceiveAdapterP__AsyncReceive__updateBuffer(
#line 49
message_t * msg);
# 40 "/home/csilab/upma/system/AsyncReceiveAdapterP.nc"
enum AsyncReceiveAdapterP____nesc_unnamed4386 {
#line 40
  AsyncReceiveAdapterP__receiveDone_task = 22U
};
#line 40
typedef int AsyncReceiveAdapterP____nesc_sillytask_receiveDone_task[AsyncReceiveAdapterP__receiveDone_task];
#line 36
message_t * AsyncReceiveAdapterP__msg_;
void * AsyncReceiveAdapterP__payload_;
uint8_t AsyncReceiveAdapterP__len_;



static inline void AsyncReceiveAdapterP__AsyncReceive__receive(message_t *msg, void *payload, uint8_t len);
#line 54
static inline void AsyncReceiveAdapterP__receiveDone_task__runTask(void );
# 100 "/home/csilab/tinyos/tos/interfaces/Send.nc"
static void AsyncSendAdapterP__Send__sendDone(
#line 96
message_t * msg, 



error_t error);
# 40 "/home/csilab/upma/interfaces/AsyncSend.nc"
static error_t AsyncSendAdapterP__AsyncSend__send(
#line 34
message_t * msg, 





uint8_t len);
#line 63
static uint8_t AsyncSendAdapterP__AsyncSend__maxPayloadLength(void );
# 39 "/home/csilab/upma/system/AsyncSendAdapterP.nc"
enum AsyncSendAdapterP____nesc_unnamed4387 {
#line 39
  AsyncSendAdapterP__sendDone_task = 23U
};
#line 39
typedef int AsyncSendAdapterP____nesc_sillytask_sendDone_task[AsyncSendAdapterP__sendDone_task];
#line 36
message_t * AsyncSendAdapterP__msg_;
uint8_t AsyncSendAdapterP__len_;



static inline error_t AsyncSendAdapterP__Send__send(message_t *msg, uint8_t len);
#line 62
static inline void AsyncSendAdapterP__sendDone_task__runTask(void );
#line 77
static inline uint8_t AsyncSendAdapterP__Send__maxPayloadLength(void );
# 43 "/home/csilab/tinyos/tos/chips/cc2420/interfaces/ReceiveIndicator.nc"
static bool PowerCycleP__EnergyIndicator__isReceiving(void );
# 35 "/home/csilab/upma/interfaces/RadioPowerControl.nc"
static error_t PowerCycleP__RadioPowerControl__start(void );
# 67 "/home/csilab/tinyos/tos/interfaces/TaskBasic.nc"
static error_t PowerCycleP__startRadio__postTask(void );
# 53 "/home/csilab/upma/interfaces/ChannelMonitor.nc"
static void PowerCycleP__ChannelMonitor__busy(void );
#line 48
static void PowerCycleP__ChannelMonitor__free(void );
# 43 "/home/csilab/tinyos/tos/chips/cc2420/interfaces/ReceiveIndicator.nc"
static bool PowerCycleP__PacketIndicator__isReceiving(void );
# 94 "/home/csilab/tinyos/tos/interfaces/Leds.nc"
static void PowerCycleP__Leds__led2Off(void );
#line 89
static void PowerCycleP__Leds__led2On(void );
# 37 "/home/csilab/upma/interfaces/LocalTimeExtended.nc"
static bool PowerCycleP__Time__lessThan(PowerCycleP__Time__local_time_t *t1, PowerCycleP__Time__local_time_t *t2);
#line 34
static PowerCycleP__Time__local_time_t PowerCycleP__Time__add(PowerCycleP__Time__local_time_t *t1, PowerCycleP__Time__local_time_t *t2);
#line 33
static PowerCycleP__Time__local_time_t PowerCycleP__Time__getNow(void );
# 67 "/home/csilab/tinyos/tos/interfaces/TaskBasic.nc"
static error_t PowerCycleP__getCca__postTask(void );
# 108 "/home/csilab/upma/chips/cc2420/PowerCycleP.nc"
enum PowerCycleP____nesc_unnamed4388 {
#line 108
  PowerCycleP__startRadio = 24U
};
#line 108
typedef int PowerCycleP____nesc_sillytask_startRadio[PowerCycleP__startRadio];
enum PowerCycleP____nesc_unnamed4389 {
#line 109
  PowerCycleP__getCca = 25U
};
#line 109
typedef int PowerCycleP____nesc_sillytask_getCca[PowerCycleP__getCca];
#line 99
uint32_t PowerCycleP__detected = 0;
#line 99
uint32_t PowerCycleP__free = 0;

enum PowerCycleP____nesc_unnamed4390 {
  PowerCycleP__DEFAULT_CCA_LENGTH = 8 * 32
};

uint16_t PowerCycleP__ccaCheckLength = PowerCycleP__DEFAULT_CCA_LENGTH;







static inline void PowerCycleP__RadioPowerControl__startDone(error_t error);




static inline void PowerCycleP__RadioPowerControl__stopDone(error_t error);
#line 136
static inline void PowerCycleP__startRadio__runTask(void );









static inline void PowerCycleP__getCca__runTask(void );
#line 218
static inline void PowerCycleP__PacketIndicator__anypktreceive(void );
# 61 "/home/csilab/tinyos/tos/system/NoLedsC.nc"
static inline void NoLedsC__Leds__led2On(void );
static inline void NoLedsC__Leds__led2Off(void );
# 64 "/home/csilab/tinyos/tos/lib/timer/Counter.nc"
static /*LocalTime32khz16C.LocalTimeP*/LocalTime16P__0__Counter__size_type /*LocalTime32khz16C.LocalTimeP*/LocalTime16P__0__Counter__get(void );
# 40 "/home/csilab/upma/system/LocalTime16P.nc"
local_time16_t /*LocalTime32khz16C.LocalTimeP*/LocalTime16P__0__g;








static inline error_t /*LocalTime32khz16C.LocalTimeP*/LocalTime16P__0__Init__init(void );




static local_time16_t /*LocalTime32khz16C.LocalTimeP*/LocalTime16P__0__LocalTime__getNow(void );






static inline local_time16_t /*LocalTime32khz16C.LocalTimeP*/LocalTime16P__0__LocalTime__add(local_time16_t *t1, local_time16_t *t2);
#line 104
static inline bool /*LocalTime32khz16C.LocalTimeP*/LocalTime16P__0__LocalTime__lessThan(local_time16_t *t1, local_time16_t *t2);
#line 117
static inline void /*LocalTime32khz16C.LocalTimeP*/LocalTime16P__0__Counter__overflow(void );
# 113 "/home/csilab/tinyos/tos/interfaces/SplitControl.nc"
static void PureTDMASchedulerP__SplitControl__startDone(error_t error);
#line 138
static void PureTDMASchedulerP__SplitControl__stopDone(error_t error);
# 40 "/home/csilab/upma/interfaces/AsyncSend.nc"
static error_t PureTDMASchedulerP__SubSend__send(
#line 34
message_t * msg, 





uint8_t len);
#line 71
static 
#line 69
void * 

PureTDMASchedulerP__SubSend__getPayload(
#line 68
message_t * msg, 


uint8_t len);
#line 63
static uint8_t PureTDMASchedulerP__SubSend__maxPayloadLength(void );
# 52 "/home/csilab/upma/interfaces/AsyncReceive.nc"
static void PureTDMASchedulerP__SubReceive__updateBuffer(
#line 49
message_t * msg);
# 8 "/home/csilab/upma/lib/macs/pure-tdma/AdaptiveHART_Util.nc"
static bool PureTDMASchedulerP__AH_Util__is_dest(uint8_t flowid, uint8_t nodeid);
#line 5
static uint8_t PureTDMASchedulerP__AH_Util__get_release_offset(uint8_t nodeid);
static uint8_t PureTDMASchedulerP__AH_Util__get_flow_id(uint8_t nodeid);
static bool PureTDMASchedulerP__AH_Util__is_root(uint8_t nodeid);

static uint8_t PureTDMASchedulerP__AH_Util__set_destination(uint8_t flowid, uint8_t nodeid);
static uint8_t PureTDMASchedulerP__AH_Util__set_TxOffset(uint8_t flowid, uint8_t nodeid);
# 73 "/home/csilab/tinyos/tos/interfaces/Queue.nc"
static 
#line 71
PureTDMASchedulerP__HIforwardQ__t  

PureTDMASchedulerP__HIforwardQ__head(void );
#line 90
static error_t PureTDMASchedulerP__HIforwardQ__enqueue(
#line 86
PureTDMASchedulerP__HIforwardQ__t  newVal);
#line 81
static 
#line 79
PureTDMASchedulerP__HIforwardQ__t  

PureTDMASchedulerP__HIforwardQ__dequeue(void );
#line 50
static bool PureTDMASchedulerP__HIforwardQ__empty(void );







static uint8_t PureTDMASchedulerP__HIforwardQ__size(void );
# 35 "/home/csilab/upma/interfaces/RadioPowerControl.nc"
static error_t PureTDMASchedulerP__RadioPowerControl__start(void );
# 73 "/home/csilab/tinyos/tos/interfaces/Queue.nc"
static 
#line 71
PureTDMASchedulerP__LOforwardQ__t  

PureTDMASchedulerP__LOforwardQ__head(void );
#line 90
static error_t PureTDMASchedulerP__LOforwardQ__enqueue(
#line 86
PureTDMASchedulerP__LOforwardQ__t  newVal);
#line 81
static 
#line 79
PureTDMASchedulerP__LOforwardQ__t  

PureTDMASchedulerP__LOforwardQ__dequeue(void );
#line 50
static bool PureTDMASchedulerP__LOforwardQ__empty(void );







static uint8_t PureTDMASchedulerP__LOforwardQ__size(void );
# 59 "/home/csilab/tinyos/tos/interfaces/PacketAcknowledgements.nc"
static error_t PureTDMASchedulerP__ACK__requestAck(
#line 53
message_t * msg);
#line 85
static bool PureTDMASchedulerP__ACK__wasAcked(
#line 80
message_t * msg);
# 95 "/home/csilab/tinyos/tos/interfaces/AsyncStdControl.nc"
static error_t PureTDMASchedulerP__GenericSlotter__start(void );
# 40 "/home/csilab/upma/interfaces/AsyncSend.nc"
static error_t PureTDMASchedulerP__BeaconSend__send(
#line 34
message_t * msg, 





uint8_t len);
# 41 "/home/csilab/upma/interfaces/AsyncReceive.nc"
static void PureTDMASchedulerP__Receive__receive(
#line 37
message_t * msg, 
void * payload, 


uint8_t len);
# 33 "/home/csilab/upma/interfaces/FrameConfiguration.nc"
static void PureTDMASchedulerP__FrameConfiguration__setFrameLength(uint8_t numSlots);
#line 32
static void PureTDMASchedulerP__FrameConfiguration__setSlotLength(uint32_t slotTimeBms);
# 77 "/home/csilab/upma/interfaces/AMPacket.nc"
static am_addr_t PureTDMASchedulerP__AMPacket__source(
#line 73
message_t * amsg);
#line 110
static void PureTDMASchedulerP__AMPacket__setSource(
#line 106
message_t * amsg, 



am_addr_t addr);
#line 92
static void PureTDMASchedulerP__AMPacket__setDestination(
#line 88
message_t * amsg, 



am_addr_t addr);
# 38 "/home/csilab/upma/interfaces/SlotterControl.nc"
static uint8_t PureTDMASchedulerP__SlotterControl__getSlot(void );
#line 31
static error_t PureTDMASchedulerP__SlotterControl__synchronize(uint8_t toSlot);
# 72 "/home/csilab/upma/lib/macs/pure-tdma/PureTDMASchedulerP.nc"
enum PureTDMASchedulerP____nesc_unnamed4391 {
  PureTDMASchedulerP__SIMPLE_TDMA_SYNC = 123, 
  PureTDMASchedulerP__MAC_Q = 1, 
  PureTDMASchedulerP__FLOW_SEQ = 20, 
  PureTDMASchedulerP__SCHEDULE_LEN = 20, 
  PureTDMASchedulerP__SCHEDULE_CHAR_LEN = 6
};

bool PureTDMASchedulerP__init;
uint32_t PureTDMASchedulerP__slotSize;
uint32_t PureTDMASchedulerP__bi;
#line 82
uint32_t PureTDMASchedulerP__sd;
#line 82
uint32_t PureTDMASchedulerP__cap;
uint8_t PureTDMASchedulerP__coordinatorId;
message_t *PureTDMASchedulerP__toSend;
uint8_t PureTDMASchedulerP__toSendLen;
uint8_t PureTDMASchedulerP__currentSlot;
bool PureTDMASchedulerP__sync;
bool PureTDMASchedulerP__requestStop;






bool PureTDMASchedulerP__phyLock = FALSE;
uint8_t PureTDMASchedulerP__i;

bool PureTDMASchedulerP__is_root = FALSE;
bool PureTDMASchedulerP__is_dest[3];
uint8_t PureTDMASchedulerP__my_release_offset = 0;
uint8_t PureTDMASchedulerP__my_flow_id = 0;


message_t PureTDMASchedulerP__AH_pkt;
adaptivehart_msg_t *PureTDMASchedulerP__AH_payload;
bool PureTDMASchedulerP__is_AH_pkt_null = TRUE;


message_t PureTDMASchedulerP__HIforwardPktBuffer[PureTDMASchedulerP__MAC_Q];
message_t PureTDMASchedulerP__LOforwardPktBuffer[PureTDMASchedulerP__MAC_Q];

uint8_t PureTDMASchedulerP__HIqueueSize;
uint8_t PureTDMASchedulerP__LOqueueSize;



int8_t PureTDMASchedulerP__tx_opper[3];
uint8_t PureTDMASchedulerP__txing_flowid = 0;
uint16_t PureTDMASchedulerP__rcv_slot[3];


uint16_t PureTDMASchedulerP__prev_seq[3];
bool PureTDMASchedulerP__rcv_indicator = FALSE;
uint8_t PureTDMASchedulerP__my_txoffset[3];


uint8_t PureTDMASchedulerP__my_dest[3];







uint16_t PureTDMASchedulerP__rcv_slots[3][PureTDMASchedulerP__FLOW_SEQ];
uint16_t PureTDMASchedulerP__rcv_srcs[3][PureTDMASchedulerP__FLOW_SEQ];
uint16_t PureTDMASchedulerP__tx_slots[PureTDMASchedulerP__FLOW_SEQ];
uint8_t PureTDMASchedulerP__rx_flow_seq[3];
uint8_t PureTDMASchedulerP__tx_flow_seq = 0;
uint32_t PureTDMASchedulerP__giga_seq = 0;










uint8_t PureTDMASchedulerP__Task_character[3][3] = { 
{ 0, 0, 0 }, 
{ 0, 0, 4 }, 
{ 0, 0, 4 } };

uint8_t PureTDMASchedulerP__TASK_PERIOD = 0;
uint8_t PureTDMASchedulerP__TASK_DEAD = 1;


uint16_t PureTDMASchedulerP__random_slots[10] = { 19, 21, 22, 25, 28, 35, 39, 43, 46, 52 };
uint8_t PureTDMASchedulerP__random_slot_idx = 0;

uint16_t PureTDMASchedulerP__random_slot = 0;
uint8_t PureTDMASchedulerP__original_period[3] = { 0, 250, 250 };
uint8_t PureTDMASchedulerP__original_deadline[3] = { 0, 250, 250 };




uint32_t PureTDMASchedulerP__gigaframe_length = 30;








uint8_t PureTDMASchedulerP__schedule[30][6] = {  };

uint8_t PureTDMASchedulerP__origin_schedule[30][6] = { 
#line 215
{ 2, 1, 3, 1, 1, 4 }, 
{ 3, 1, 3, 1, 1, 3 }, 
{ 4, 1, 3, 1, 1, 2 }, 
{ 5, 1, 3, 1, 1, 1 }, 
{ 6, 3, 5, 1, 1, 4 }, 
{ 7, 3, 5, 1, 1, 3 }, 
{ 8, 3, 5, 1, 1, 2 }, 
{ 9, 3, 5, 1, 1, 1 }, 
{ 10, 5, 6, 1, 1, 4 }, 
{ 11, 5, 6, 1, 1, 3 }, 
{ 12, 5, 6, 1, 1, 2 }, 
{ 13, 5, 6, 1, 1, 1 }, 
{ 14, 2, 3, 2, 2, 4 }, 
{ 15, 2, 3, 2, 2, 3 }, 
{ 16, 2, 3, 2, 2, 2 }, 
{ 17, 2, 3, 2, 2, 1 }, 
{ 18, 3, 5, 2, 2, 4 }, 
{ 19, 3, 5, 2, 2, 3 }, 
{ 20, 3, 5, 2, 2, 2 }, 
{ 21, 3, 5, 2, 2, 1 }, 
{ 22, 5, 7, 2, 2, 4 }, 
{ 23, 5, 7, 2, 2, 3 }, 
{ 24, 5, 7, 2, 2, 2 }, 
{ 25, 5, 7, 2, 2, 1 } };
#line 269
uint8_t PureTDMASchedulerP__schedule_len = 30;
uint8_t PureTDMASchedulerP__schedule_char_len = 6;
uint8_t PureTDMASchedulerP__SLOT = 0;
#line 271
PureTDMASchedulerP__SENDER = 1;
#line 271
PureTDMASchedulerP__RECEIVER = 2;
#line 271
PureTDMASchedulerP__FLOWID = 3;




static void PureTDMASchedulerP__send_AH_Pkt(message_t *packet, am_addr_t dest, uint8_t flowid);
static void PureTDMASchedulerP__set_payload(adaptivehart_msg_t *payload, uint8_t flowid, uint8_t priority, uint8_t maxtx, uint16_t deadline, uint16_t seq);
static inline void PureTDMASchedulerP__Q_reset(uint8_t flowid);
static void PureTDMASchedulerP__reset_var(uint8_t flowid);


static inline void PureTDMASchedulerP__WH_send(uint8_t slot);
static inline void PureTDMASchedulerP__schedule_reset_WH(uint8_t target_schedule[][6], uint8_t col, uint8_t row);






static inline error_t PureTDMASchedulerP__Init__init(void );
#line 350
static inline error_t PureTDMASchedulerP__SplitControl__start(void );
#line 369
static void PureTDMASchedulerP__RadioPowerControl__startDone(error_t error);
#line 388
static inline void PureTDMASchedulerP__RadioPowerControl__stopDone(error_t error);







static inline void PureTDMASchedulerP__Slotter__slot(uint8_t slot);
#line 549
static inline error_t PureTDMASchedulerP__Send__send(message_t *msg, uint8_t len);
#line 561
static inline void PureTDMASchedulerP__BeaconSend__sendDone(message_t *msg, error_t error);



static inline void PureTDMASchedulerP__SubSend__sendDone(message_t *msg, error_t error);
#line 602
static inline void PureTDMASchedulerP__SubReceive__receive(message_t *msg, void *payload, uint8_t len);
#line 682
static inline uint8_t PureTDMASchedulerP__Send__maxPayloadLength(void );




static inline void PureTDMASchedulerP__Receive__updateBuffer(message_t *msg);
#line 703
static inline void PureTDMASchedulerP__TxOffset_alarm__fired(void );
#line 722
static inline void PureTDMASchedulerP__ReceiveIndicator__anypktreceive(void );






static void PureTDMASchedulerP__send_AH_Pkt(message_t *packet, am_addr_t dest, uint8_t flowid);
#line 760
static void PureTDMASchedulerP__set_payload(adaptivehart_msg_t *payload, uint8_t flowid, uint8_t priority, uint8_t maxtx, uint16_t deadline, uint16_t seq);







static inline void PureTDMASchedulerP__Q_reset(uint8_t flowid);
#line 785
static void PureTDMASchedulerP__reset_var(uint8_t flowid);
#line 829
static inline void PureTDMASchedulerP__WH_send(uint8_t slot);
#line 862
static inline void PureTDMASchedulerP__schedule_reset_WH(uint8_t target_schedule[][6], uint8_t col, uint8_t row);
# 40 "/home/csilab/upma/interfaces/AsyncSend.nc"
static error_t /*TDMASlotSenderC.TDMASlotSender.GenericSlotSenderP*/GenericSlotSenderP__0__SubSend__send(
#line 34
message_t * msg, 





uint8_t len);
#line 71
static 
#line 69
void * 

/*TDMASlotSenderC.TDMASlotSender.GenericSlotSenderP*/GenericSlotSenderP__0__SubSend__getPayload(
#line 68
message_t * msg, 


uint8_t len);
#line 63
static uint8_t /*TDMASlotSenderC.TDMASlotSender.GenericSlotSenderP*/GenericSlotSenderP__0__SubSend__maxPayloadLength(void );
#line 48
static void /*TDMASlotSenderC.TDMASlotSender.GenericSlotSenderP*/GenericSlotSenderP__0__Send__sendDone(
#line 44
message_t * msg, 



error_t error);
# 66 "/home/csilab/tinyos/tos/lib/timer/Alarm.nc"
static void /*TDMASlotSenderC.TDMASlotSender.GenericSlotSenderP*/GenericSlotSenderP__0__Alarm__start(/*TDMASlotSenderC.TDMASlotSender.GenericSlotSenderP*/GenericSlotSenderP__0__Alarm__size_type dt);
# 52 "/home/csilab/upma/system/GenericSlotSenderP.nc"
enum /*TDMASlotSenderC.TDMASlotSender.GenericSlotSenderP*/GenericSlotSenderP__0____nesc_unnamed4392 {
  GenericSlotSenderP__0__S_START = 0, 
  GenericSlotSenderP__0__S_OFFSET = 1, 
  GenericSlotSenderP__0__S_SENT = 2, 
  GenericSlotSenderP__0__S_END = 3
};

message_t */*TDMASlotSenderC.TDMASlotSender.GenericSlotSenderP*/GenericSlotSenderP__0__toSend;
uint8_t /*TDMASlotSenderC.TDMASlotSender.GenericSlotSenderP*/GenericSlotSenderP__0__toSendLen;
uint8_t /*TDMASlotSenderC.TDMASlotSender.GenericSlotSenderP*/GenericSlotSenderP__0__state;

static inline void /*TDMASlotSenderC.TDMASlotSender.GenericSlotSenderP*/GenericSlotSenderP__0__Boot__booted(void );




static inline void /*TDMASlotSenderC.TDMASlotSender.GenericSlotSenderP*/GenericSlotSenderP__0__alarm(uint8_t at);


static inline error_t /*TDMASlotSenderC.TDMASlotSender.GenericSlotSenderP*/GenericSlotSenderP__0__Init__init(void );






static inline error_t /*TDMASlotSenderC.TDMASlotSender.GenericSlotSenderP*/GenericSlotSenderP__0__Send__send(message_t *msg, uint8_t len);
#line 119
static inline void /*TDMASlotSenderC.TDMASlotSender.GenericSlotSenderP*/GenericSlotSenderP__0__Alarm__fired(void );
#line 134
static inline void /*TDMASlotSenderC.TDMASlotSender.GenericSlotSenderP*/GenericSlotSenderP__0__SubSend__sendDone(message_t *msg, error_t error);
#line 146
static inline void */*TDMASlotSenderC.TDMASlotSender.GenericSlotSenderP*/GenericSlotSenderP__0__Send__getPayload(message_t *msg, uint8_t len);



static inline uint8_t /*TDMASlotSenderC.TDMASlotSender.GenericSlotSenderP*/GenericSlotSenderP__0__Send__maxPayloadLength(void );



static inline uint16_t /*TDMASlotSenderC.TDMASlotSender.GenericSlotSenderP*/GenericSlotSenderP__0__SubCcaControl__getInitialBackoff(message_t *msg, uint16_t defaultbackoff);



static inline uint16_t /*TDMASlotSenderC.TDMASlotSender.GenericSlotSenderP*/GenericSlotSenderP__0__SubCcaControl__getCongestionBackoff(message_t *msg, uint16_t defaultBackoff);



static inline bool /*TDMASlotSenderC.TDMASlotSender.GenericSlotSenderP*/GenericSlotSenderP__0__SubCcaControl__getCca(message_t *msg, bool defaultCca);
# 41 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*TDMASlotSenderC.TDMASlotSender.Alarm32khz16C.Msp430Alarm*/Msp430AlarmC__2__Msp430Compare__setEvent(uint16_t time);

static void /*TDMASlotSenderC.TDMASlotSender.Alarm32khz16C.Msp430Alarm*/Msp430AlarmC__2__Msp430Compare__setEventFromNow(uint16_t delta);
# 45 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430Timer.nc"
static uint16_t /*TDMASlotSenderC.TDMASlotSender.Alarm32khz16C.Msp430Alarm*/Msp430AlarmC__2__Msp430Timer__get(void );
# 78 "/home/csilab/tinyos/tos/lib/timer/Alarm.nc"
static void /*TDMASlotSenderC.TDMASlotSender.Alarm32khz16C.Msp430Alarm*/Msp430AlarmC__2__Alarm__fired(void );
# 57 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430TimerControl.nc"
static void /*TDMASlotSenderC.TDMASlotSender.Alarm32khz16C.Msp430Alarm*/Msp430AlarmC__2__Msp430TimerControl__enableEvents(void );
static void /*TDMASlotSenderC.TDMASlotSender.Alarm32khz16C.Msp430Alarm*/Msp430AlarmC__2__Msp430TimerControl__disableEvents(void );
#line 44
static void /*TDMASlotSenderC.TDMASlotSender.Alarm32khz16C.Msp430Alarm*/Msp430AlarmC__2__Msp430TimerControl__clearPendingInterrupt(void );
# 60 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*TDMASlotSenderC.TDMASlotSender.Alarm32khz16C.Msp430Alarm*/Msp430AlarmC__2__Alarm__start(uint16_t dt);









static inline void /*TDMASlotSenderC.TDMASlotSender.Alarm32khz16C.Msp430Alarm*/Msp430AlarmC__2__Msp430Compare__fired(void );










static inline void /*TDMASlotSenderC.TDMASlotSender.Alarm32khz16C.Msp430Alarm*/Msp430AlarmC__2__Alarm__startAt(uint16_t t0, uint16_t dt);
#line 104
static inline uint16_t /*TDMASlotSenderC.TDMASlotSender.Alarm32khz16C.Msp430Alarm*/Msp430AlarmC__2__Alarm__getNow(void );









static inline void /*TDMASlotSenderC.TDMASlotSender.Alarm32khz16C.Msp430Alarm*/Msp430AlarmC__2__Msp430Timer__overflow(void );
# 31 "/home/csilab/upma/interfaces/Slotter.nc"
static void GenericSlotterP__Slotter__slot(uint8_t num);
# 109 "/home/csilab/tinyos/tos/lib/timer/Alarm.nc"
static GenericSlotterP__Alarm__size_type GenericSlotterP__Alarm__getNow(void );
#line 66
static void GenericSlotterP__Alarm__start(GenericSlotterP__Alarm__size_type dt);






static void GenericSlotterP__Alarm__stop(void );
# 46 "/home/csilab/upma/system/GenericSlotterP.nc"
uint8_t GenericSlotterP__slot;
uint32_t GenericSlotterP__slotLength;
uint8_t GenericSlotterP__frameLength;
bool GenericSlotterP__sync;
uint32_t GenericSlotterP__nextFire;

static inline error_t GenericSlotterP__Init__init(void );








static inline void GenericSlotterP__Boot__booted(void );




static inline error_t GenericSlotterP__AsyncStdControl__start(void );








static error_t GenericSlotterP__SlotterControl__synchronize(uint8_t toSlot);







static inline void GenericSlotterP__FrameConfiguration__setSlotLength(uint32_t slotTimeBms);



static inline void GenericSlotterP__FrameConfiguration__setFrameLength(uint8_t numSlots);
#line 107
static inline void GenericSlotterP__SlotterControl__stop(void );




static void GenericSlotterP__Alarm__fired(void );







static inline uint8_t GenericSlotterP__SlotterControl__getSlot(void );


static inline uint32_t GenericSlotterP__SlotterControl__getRemaining(void );
# 41 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*GenericSlotterC.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__3__Msp430Compare__setEvent(uint16_t time);

static void /*GenericSlotterC.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__3__Msp430Compare__setEventFromNow(uint16_t delta);
# 45 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430Timer.nc"
static uint16_t /*GenericSlotterC.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__3__Msp430Timer__get(void );
# 78 "/home/csilab/tinyos/tos/lib/timer/Alarm.nc"
static void /*GenericSlotterC.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__3__Alarm__fired(void );
# 57 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430TimerControl.nc"
static void /*GenericSlotterC.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__3__Msp430TimerControl__enableEvents(void );
static void /*GenericSlotterC.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__3__Msp430TimerControl__disableEvents(void );
#line 44
static void /*GenericSlotterC.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__3__Msp430TimerControl__clearPendingInterrupt(void );
# 65 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*GenericSlotterC.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__3__Alarm__stop(void );




static inline void /*GenericSlotterC.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__3__Msp430Compare__fired(void );










static inline void /*GenericSlotterC.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__3__Alarm__startAt(uint16_t t0, uint16_t dt);
#line 114
static inline void /*GenericSlotterC.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__3__Msp430Timer__overflow(void );
# 78 "/home/csilab/tinyos/tos/lib/timer/Alarm.nc"
static void /*GenericSlotterC.Alarm32khz32C.Transform*/TransformAlarmC__2__Alarm__fired(void );
#line 103
static void /*GenericSlotterC.Alarm32khz32C.Transform*/TransformAlarmC__2__AlarmFrom__startAt(/*GenericSlotterC.Alarm32khz32C.Transform*/TransformAlarmC__2__AlarmFrom__size_type t0, /*GenericSlotterC.Alarm32khz32C.Transform*/TransformAlarmC__2__AlarmFrom__size_type dt);
#line 73
static void /*GenericSlotterC.Alarm32khz32C.Transform*/TransformAlarmC__2__AlarmFrom__stop(void );
# 64 "/home/csilab/tinyos/tos/lib/timer/Counter.nc"
static /*GenericSlotterC.Alarm32khz32C.Transform*/TransformAlarmC__2__Counter__size_type /*GenericSlotterC.Alarm32khz32C.Transform*/TransformAlarmC__2__Counter__get(void );
# 77 "/home/csilab/tinyos/tos/lib/timer/TransformAlarmC.nc"
/*GenericSlotterC.Alarm32khz32C.Transform*/TransformAlarmC__2__to_size_type /*GenericSlotterC.Alarm32khz32C.Transform*/TransformAlarmC__2__m_t0;
/*GenericSlotterC.Alarm32khz32C.Transform*/TransformAlarmC__2__to_size_type /*GenericSlotterC.Alarm32khz32C.Transform*/TransformAlarmC__2__m_dt;

enum /*GenericSlotterC.Alarm32khz32C.Transform*/TransformAlarmC__2____nesc_unnamed4393 {

  TransformAlarmC__2__MAX_DELAY_LOG2 = 8 * sizeof(/*GenericSlotterC.Alarm32khz32C.Transform*/TransformAlarmC__2__from_size_type ) - 1 - 0, 
  TransformAlarmC__2__MAX_DELAY = (/*GenericSlotterC.Alarm32khz32C.Transform*/TransformAlarmC__2__to_size_type )1 << /*GenericSlotterC.Alarm32khz32C.Transform*/TransformAlarmC__2__MAX_DELAY_LOG2
};

static inline /*GenericSlotterC.Alarm32khz32C.Transform*/TransformAlarmC__2__to_size_type /*GenericSlotterC.Alarm32khz32C.Transform*/TransformAlarmC__2__Alarm__getNow(void );
#line 102
static inline void /*GenericSlotterC.Alarm32khz32C.Transform*/TransformAlarmC__2__Alarm__stop(void );




static void /*GenericSlotterC.Alarm32khz32C.Transform*/TransformAlarmC__2__set_alarm(void );
#line 147
static inline void /*GenericSlotterC.Alarm32khz32C.Transform*/TransformAlarmC__2__Alarm__startAt(/*GenericSlotterC.Alarm32khz32C.Transform*/TransformAlarmC__2__to_size_type t0, /*GenericSlotterC.Alarm32khz32C.Transform*/TransformAlarmC__2__to_size_type dt);









static inline void /*GenericSlotterC.Alarm32khz32C.Transform*/TransformAlarmC__2__Alarm__start(/*GenericSlotterC.Alarm32khz32C.Transform*/TransformAlarmC__2__to_size_type dt);




static inline void /*GenericSlotterC.Alarm32khz32C.Transform*/TransformAlarmC__2__AlarmFrom__fired(void );
#line 177
static inline void /*GenericSlotterC.Alarm32khz32C.Transform*/TransformAlarmC__2__Counter__overflow(void );
# 40 "/home/csilab/upma/interfaces/AsyncSend.nc"
static error_t SenderDispatcherP__SubSend__send(
#line 34
message_t * msg, 





uint8_t len);
#line 71
static 
#line 69
void * 

SenderDispatcherP__SubSend__getPayload(
#line 68
message_t * msg, 


uint8_t len);
#line 63
static uint8_t SenderDispatcherP__SubSend__maxPayloadLength(void );
#line 48
static void SenderDispatcherP__Send__sendDone(
# 39 "/home/csilab/upma/system/SenderDispatcherP.nc"
uint8_t arg_0x7f7fec194250, 
# 44 "/home/csilab/upma/interfaces/AsyncSend.nc"
message_t * msg, 



error_t error);
# 50 "/home/csilab/upma/interfaces/CcaControl.nc"
static uint16_t SenderDispatcherP__SlotsCcaControl__getCongestionBackoff(
# 40 "/home/csilab/upma/system/SenderDispatcherP.nc"
uint8_t arg_0x7f7fec1925b0, 
# 50 "/home/csilab/upma/interfaces/CcaControl.nc"
message_t *msg, uint16_t defaultBackoff);
#line 39
static uint16_t SenderDispatcherP__SlotsCcaControl__getInitialBackoff(
# 40 "/home/csilab/upma/system/SenderDispatcherP.nc"
uint8_t arg_0x7f7fec1925b0, 
# 39 "/home/csilab/upma/interfaces/CcaControl.nc"
message_t *msg, uint16_t defaultbackoff);
#line 61
static bool SenderDispatcherP__SlotsCcaControl__getCca(
# 40 "/home/csilab/upma/system/SenderDispatcherP.nc"
uint8_t arg_0x7f7fec1925b0, 
# 61 "/home/csilab/upma/interfaces/CcaControl.nc"
message_t *msg, bool defaultCca);
# 42 "/home/csilab/upma/system/SenderDispatcherP.nc"
message_t *SenderDispatcherP__toSend;
uint8_t SenderDispatcherP__toSendLen;
uint8_t SenderDispatcherP__last_id;

static inline error_t SenderDispatcherP__Init__init(void );
#line 59
static error_t SenderDispatcherP__Send__send(uint8_t slotType, message_t *msg, uint8_t len);
#line 94
static inline void *SenderDispatcherP__Send__getPayload(uint8_t slotType, message_t *msg, uint8_t len);



static inline uint8_t SenderDispatcherP__Send__maxPayloadLength(uint8_t slotType);



static inline void SenderDispatcherP__SubSend__sendDone(message_t *msg, error_t error);
#line 121
static inline void SenderDispatcherP__Send__default__sendDone(uint8_t type, message_t *msg, error_t error);


static inline uint16_t SenderDispatcherP__SlotsCcaControl__default__getInitialBackoff(am_id_t id, message_t *msg, uint16_t defaultbackoff);



static inline uint16_t SenderDispatcherP__SlotsCcaControl__default__getCongestionBackoff(am_id_t id, message_t *msg, uint16_t defaultBackoff);



static inline bool SenderDispatcherP__SlotsCcaControl__default__getCca(am_id_t id, message_t *msg, bool defaultCca);



static inline uint16_t SenderDispatcherP__SubCcaControl__getInitialBackoff(am_id_t id, message_t *msg, uint16_t defaultbackoff);





static inline uint16_t SenderDispatcherP__SubCcaControl__getCongestionBackoff(am_id_t id, message_t *msg, uint16_t defaultBackoff);



static inline bool SenderDispatcherP__SubCcaControl__getCca(am_id_t id, message_t *msg, bool defaultCca);
# 34 "/home/csilab/upma/system/DummyChannelMonitorC.nc"
static inline void DummyChannelMonitorC__ChannelMonitor__busy(void );

static inline void DummyChannelMonitorC__ChannelMonitor__free(void );
# 40 "/home/csilab/upma/interfaces/AsyncSend.nc"
static error_t BeaconSlotP__SubSend__send(
#line 34
message_t * msg, 





uint8_t len);
#line 71
static 
#line 69
void * 

BeaconSlotP__SubSend__getPayload(
#line 68
message_t * msg, 


uint8_t len);
# 52 "/home/csilab/upma/interfaces/AsyncReceive.nc"
static void BeaconSlotP__SubReceive__updateBuffer(
#line 49
message_t * msg);
# 66 "/home/csilab/tinyos/tos/lib/timer/Alarm.nc"
static void BeaconSlotP__SyncAlarm__start(BeaconSlotP__SyncAlarm__size_type dt);
# 48 "/home/csilab/upma/interfaces/AsyncSend.nc"
static void BeaconSlotP__Send__sendDone(
#line 44
message_t * msg, 



error_t error);
# 41 "/home/csilab/upma/interfaces/AsyncReceive.nc"
static void BeaconSlotP__Receive__receive(
#line 37
message_t * msg, 
void * payload, 


uint8_t len);
# 158 "/home/csilab/upma/interfaces/AMPacket.nc"
static uint8_t BeaconSlotP__AMPacket__headerSize(void );
#line 110
static void BeaconSlotP__AMPacket__setSource(
#line 106
message_t * amsg, 



am_addr_t addr);
#line 92
static void BeaconSlotP__AMPacket__setDestination(
#line 88
message_t * amsg, 



am_addr_t addr);
#line 136
static am_id_t BeaconSlotP__AMPacket__type(
#line 132
message_t * amsg);
#line 151
static void BeaconSlotP__AMPacket__setType(
#line 147
message_t * amsg, 



am_id_t t);
# 31 "/home/csilab/upma/interfaces/SlotterControl.nc"
static error_t BeaconSlotP__SlotterControl__synchronize(uint8_t toSlot);




static uint32_t BeaconSlotP__SlotterControl__getRemaining(void );
static void BeaconSlotP__SlotterControl__stop(void );
# 51 "/home/csilab/upma/system/BeaconSlotP.nc"
message_t BeaconSlotP__syncPkt;
uint8_t BeaconSlotP__syncPktLen;




static inline void BeaconSlotP__write_timestamp(uint8_t *payload, uint32_t timestamp);





static inline uint32_t BeaconSlotP__read_timestamp(uint8_t *payload);





static inline error_t BeaconSlotP__Init__init(void );








static inline error_t BeaconSlotP__Send__send(message_t *msg, uint8_t len);
#line 93
static inline void BeaconSlotP__SubSend__sendDone(message_t *msg, error_t error);
#line 113
static inline void BeaconSlotP__SubReceive__receive(message_t *msg, void *payload, uint8_t len);
#line 136
static inline void BeaconSlotP__SyncAlarm__fired(void );



static inline void BeaconSlotP__Receive__updateBuffer(message_t *msg);


static inline uint16_t BeaconSlotP__SubCcaControl__getInitialBackoff(message_t *msg, uint16_t defaultbackoff);



static inline uint16_t BeaconSlotP__SubCcaControl__getCongestionBackoff(message_t *msg, uint16_t defaultBackoff);



static inline bool BeaconSlotP__SubCcaControl__getCca(message_t *msg, bool defaultCca);
# 41 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*PureTDMASchedulerC.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__4__Msp430Compare__setEvent(uint16_t time);

static void /*PureTDMASchedulerC.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__4__Msp430Compare__setEventFromNow(uint16_t delta);
# 45 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430Timer.nc"
static uint16_t /*PureTDMASchedulerC.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__4__Msp430Timer__get(void );
# 78 "/home/csilab/tinyos/tos/lib/timer/Alarm.nc"
static void /*PureTDMASchedulerC.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__4__Alarm__fired(void );
# 57 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430TimerControl.nc"
static void /*PureTDMASchedulerC.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__4__Msp430TimerControl__enableEvents(void );
static void /*PureTDMASchedulerC.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__4__Msp430TimerControl__disableEvents(void );
#line 44
static void /*PureTDMASchedulerC.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__4__Msp430TimerControl__clearPendingInterrupt(void );
# 70 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*PureTDMASchedulerC.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__4__Msp430Compare__fired(void );










static inline void /*PureTDMASchedulerC.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__4__Alarm__startAt(uint16_t t0, uint16_t dt);
#line 114
static inline void /*PureTDMASchedulerC.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__4__Msp430Timer__overflow(void );
# 78 "/home/csilab/tinyos/tos/lib/timer/Alarm.nc"
static void /*PureTDMASchedulerC.Alarm32khz32C.Transform*/TransformAlarmC__3__Alarm__fired(void );
#line 103
static void /*PureTDMASchedulerC.Alarm32khz32C.Transform*/TransformAlarmC__3__AlarmFrom__startAt(/*PureTDMASchedulerC.Alarm32khz32C.Transform*/TransformAlarmC__3__AlarmFrom__size_type t0, /*PureTDMASchedulerC.Alarm32khz32C.Transform*/TransformAlarmC__3__AlarmFrom__size_type dt);
# 64 "/home/csilab/tinyos/tos/lib/timer/Counter.nc"
static /*PureTDMASchedulerC.Alarm32khz32C.Transform*/TransformAlarmC__3__Counter__size_type /*PureTDMASchedulerC.Alarm32khz32C.Transform*/TransformAlarmC__3__Counter__get(void );
# 77 "/home/csilab/tinyos/tos/lib/timer/TransformAlarmC.nc"
/*PureTDMASchedulerC.Alarm32khz32C.Transform*/TransformAlarmC__3__to_size_type /*PureTDMASchedulerC.Alarm32khz32C.Transform*/TransformAlarmC__3__m_t0;
/*PureTDMASchedulerC.Alarm32khz32C.Transform*/TransformAlarmC__3__to_size_type /*PureTDMASchedulerC.Alarm32khz32C.Transform*/TransformAlarmC__3__m_dt;

enum /*PureTDMASchedulerC.Alarm32khz32C.Transform*/TransformAlarmC__3____nesc_unnamed4394 {

  TransformAlarmC__3__MAX_DELAY_LOG2 = 8 * sizeof(/*PureTDMASchedulerC.Alarm32khz32C.Transform*/TransformAlarmC__3__from_size_type ) - 1 - 0, 
  TransformAlarmC__3__MAX_DELAY = (/*PureTDMASchedulerC.Alarm32khz32C.Transform*/TransformAlarmC__3__to_size_type )1 << /*PureTDMASchedulerC.Alarm32khz32C.Transform*/TransformAlarmC__3__MAX_DELAY_LOG2
};

static inline /*PureTDMASchedulerC.Alarm32khz32C.Transform*/TransformAlarmC__3__to_size_type /*PureTDMASchedulerC.Alarm32khz32C.Transform*/TransformAlarmC__3__Alarm__getNow(void );
#line 107
static void /*PureTDMASchedulerC.Alarm32khz32C.Transform*/TransformAlarmC__3__set_alarm(void );
#line 147
static inline void /*PureTDMASchedulerC.Alarm32khz32C.Transform*/TransformAlarmC__3__Alarm__startAt(/*PureTDMASchedulerC.Alarm32khz32C.Transform*/TransformAlarmC__3__to_size_type t0, /*PureTDMASchedulerC.Alarm32khz32C.Transform*/TransformAlarmC__3__to_size_type dt);









static inline void /*PureTDMASchedulerC.Alarm32khz32C.Transform*/TransformAlarmC__3__Alarm__start(/*PureTDMASchedulerC.Alarm32khz32C.Transform*/TransformAlarmC__3__to_size_type dt);




static inline void /*PureTDMASchedulerC.Alarm32khz32C.Transform*/TransformAlarmC__3__AlarmFrom__fired(void );
#line 177
static inline void /*PureTDMASchedulerC.Alarm32khz32C.Transform*/TransformAlarmC__3__Counter__overflow(void );
# 48 "/home/csilab/tinyos/tos/system/QueueC.nc"
/*PureTDMASchedulerC.HIQueueC*/QueueC__1__queue_t  /*PureTDMASchedulerC.HIQueueC*/QueueC__1__queue[1];
uint8_t /*PureTDMASchedulerC.HIQueueC*/QueueC__1__head = 0;
uint8_t /*PureTDMASchedulerC.HIQueueC*/QueueC__1__tail = 0;
uint8_t /*PureTDMASchedulerC.HIQueueC*/QueueC__1__size = 0;

static inline bool /*PureTDMASchedulerC.HIQueueC*/QueueC__1__Queue__empty(void );



static inline uint8_t /*PureTDMASchedulerC.HIQueueC*/QueueC__1__Queue__size(void );



static inline uint8_t /*PureTDMASchedulerC.HIQueueC*/QueueC__1__Queue__maxSize(void );



static inline /*PureTDMASchedulerC.HIQueueC*/QueueC__1__queue_t /*PureTDMASchedulerC.HIQueueC*/QueueC__1__Queue__head(void );



static inline void /*PureTDMASchedulerC.HIQueueC*/QueueC__1__printQueue(void );
#line 85
static inline /*PureTDMASchedulerC.HIQueueC*/QueueC__1__queue_t /*PureTDMASchedulerC.HIQueueC*/QueueC__1__Queue__dequeue(void );
#line 97
static inline error_t /*PureTDMASchedulerC.HIQueueC*/QueueC__1__Queue__enqueue(/*PureTDMASchedulerC.HIQueueC*/QueueC__1__queue_t newVal);
#line 48
/*PureTDMASchedulerC.LOQueueC*/QueueC__2__queue_t  /*PureTDMASchedulerC.LOQueueC*/QueueC__2__queue[1];
uint8_t /*PureTDMASchedulerC.LOQueueC*/QueueC__2__head = 0;
uint8_t /*PureTDMASchedulerC.LOQueueC*/QueueC__2__tail = 0;
uint8_t /*PureTDMASchedulerC.LOQueueC*/QueueC__2__size = 0;

static inline bool /*PureTDMASchedulerC.LOQueueC*/QueueC__2__Queue__empty(void );



static inline uint8_t /*PureTDMASchedulerC.LOQueueC*/QueueC__2__Queue__size(void );



static inline uint8_t /*PureTDMASchedulerC.LOQueueC*/QueueC__2__Queue__maxSize(void );



static inline /*PureTDMASchedulerC.LOQueueC*/QueueC__2__queue_t /*PureTDMASchedulerC.LOQueueC*/QueueC__2__Queue__head(void );



static inline void /*PureTDMASchedulerC.LOQueueC*/QueueC__2__printQueue(void );
#line 85
static inline /*PureTDMASchedulerC.LOQueueC*/QueueC__2__queue_t /*PureTDMASchedulerC.LOQueueC*/QueueC__2__Queue__dequeue(void );
#line 97
static inline error_t /*PureTDMASchedulerC.LOQueueC*/QueueC__2__Queue__enqueue(/*PureTDMASchedulerC.LOQueueC*/QueueC__2__queue_t newVal);
# 78 "/home/csilab/tinyos/tos/lib/timer/Alarm.nc"
static void /*PureTDMASchedulerC.offset_alarm.Msp430Alarm*/Msp430AlarmC__5__Alarm__fired(void );
# 58 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430TimerControl.nc"
static void /*PureTDMASchedulerC.offset_alarm.Msp430Alarm*/Msp430AlarmC__5__Msp430TimerControl__disableEvents(void );
# 70 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*PureTDMASchedulerC.offset_alarm.Msp430Alarm*/Msp430AlarmC__5__Msp430Compare__fired(void );
#line 114
static inline void /*PureTDMASchedulerC.offset_alarm.Msp430Alarm*/Msp430AlarmC__5__Msp430Timer__overflow(void );
# 14 "/home/csilab/upma/lib/macs/pure-tdma/AdaptiveHART_UtilC.nc"
uint8_t AdaptiveHART_UtilC__flow_info[3][4] = { 
{ 0, 0, 0, 0 }, 
{ 1, 1, 2, 6 }, 
{ 2, 2, 2, 7 } };


uint8_t AdaptiveHART_UtilC__HI_routing_map[8] = { 0, 3, 0, 5, 0, 6, 0, 0 };
uint8_t AdaptiveHART_UtilC__LO_routing_map[8] = { 0, 0, 3, 5, 0, 7, 0, 0 };

uint8_t AdaptiveHART_UtilC__Offset_1 = 0;
uint8_t AdaptiveHART_UtilC__Offset_2 = 7;
uint8_t AdaptiveHART_UtilC__Offset_3 = 15;


static inline bool AdaptiveHART_UtilC__has_LS_type(uint8_t nodeid);

static inline uint8_t AdaptiveHART_UtilC__AdaptiveHART_Util__get_release_offset(uint8_t nodeid);
#line 43
static inline uint8_t AdaptiveHART_UtilC__AdaptiveHART_Util__get_flow_id(uint8_t nodeid);
#line 57
static inline bool AdaptiveHART_UtilC__AdaptiveHART_Util__is_root(uint8_t nodeid);
#line 70
static inline bool AdaptiveHART_UtilC__AdaptiveHART_Util__is_dest(uint8_t flowid, uint8_t nodeid);







static inline uint8_t AdaptiveHART_UtilC__AdaptiveHART_Util__set_destination(uint8_t flowid, uint8_t nodeid);
#line 91
static inline uint8_t AdaptiveHART_UtilC__AdaptiveHART_Util__set_TxOffset(uint8_t flowid, uint8_t nodeid);
#line 108
static inline bool AdaptiveHART_UtilC__has_LS_type(uint8_t nodeid);
# 75 "/home/csilab/tinyos/tos/interfaces/Send.nc"
static error_t CC2420ActiveMessageP__SubSend__send(
#line 67
message_t * msg, 







uint8_t len);
#line 125
static 
#line 123
void * 

CC2420ActiveMessageP__SubSend__getPayload(
#line 122
message_t * msg, 


uint8_t len);
#line 112
static uint8_t CC2420ActiveMessageP__SubSend__maxPayloadLength(void );
# 77 "/home/csilab/tinyos/tos/chips/cc2420/interfaces/CC2420Config.nc"
static uint16_t CC2420ActiveMessageP__CC2420Config__getPanAddr(void );
# 59 "/home/csilab/tinyos/tos/interfaces/SendNotifier.nc"
static void CC2420ActiveMessageP__SendNotifier__aboutToSend(
# 70 "/home/csilab/upma/chips/cc2420/CC2420ActiveMessageP.nc"
am_id_t arg_0x7f7fec07d240, 
# 59 "/home/csilab/tinyos/tos/interfaces/SendNotifier.nc"
am_addr_t dest, 
#line 57
message_t * msg);
# 110 "/home/csilab/tinyos/tos/interfaces/AMSend.nc"
static void CC2420ActiveMessageP__AMSend__sendDone(
# 64 "/home/csilab/upma/chips/cc2420/CC2420ActiveMessageP.nc"
am_id_t arg_0x7f7fec083320, 
# 103 "/home/csilab/tinyos/tos/interfaces/AMSend.nc"
message_t * msg, 






error_t error);
# 78 "/home/csilab/tinyos/tos/interfaces/Receive.nc"
static 
#line 74
message_t * 



CC2420ActiveMessageP__Snoop__receive(
# 66 "/home/csilab/upma/chips/cc2420/CC2420ActiveMessageP.nc"
am_id_t arg_0x7f7fec081280, 
# 71 "/home/csilab/tinyos/tos/interfaces/Receive.nc"
message_t * msg, 
void * payload, 





uint8_t len);
# 50 "/home/csilab/upma/interfaces/CcaControl.nc"
static uint16_t CC2420ActiveMessageP__CcaControl__getCongestionBackoff(
# 69 "/home/csilab/upma/chips/cc2420/CC2420ActiveMessageP.nc"
am_id_t arg_0x7f7fec07e320, 
# 50 "/home/csilab/upma/interfaces/CcaControl.nc"
message_t *msg, uint16_t defaultBackoff);
#line 39
static uint16_t CC2420ActiveMessageP__CcaControl__getInitialBackoff(
# 69 "/home/csilab/upma/chips/cc2420/CC2420ActiveMessageP.nc"
am_id_t arg_0x7f7fec07e320, 
# 39 "/home/csilab/upma/interfaces/CcaControl.nc"
message_t *msg, uint16_t defaultbackoff);
#line 61
static bool CC2420ActiveMessageP__CcaControl__getCca(
# 69 "/home/csilab/upma/chips/cc2420/CC2420ActiveMessageP.nc"
am_id_t arg_0x7f7fec07e320, 
# 61 "/home/csilab/upma/interfaces/CcaControl.nc"
message_t *msg, bool defaultCca);
# 50 "/home/csilab/tinyos/tos/interfaces/ActiveMessageAddress.nc"
static am_addr_t CC2420ActiveMessageP__ActiveMessageAddress__amAddress(void );
# 42 "/home/csilab/tinyos/tos/chips/cc2420/interfaces/CC2420PacketBody.nc"
static cc2420_header_t * CC2420ActiveMessageP__CC2420PacketBody__getHeader(message_t * msg);
# 78 "/home/csilab/tinyos/tos/interfaces/Receive.nc"
static 
#line 74
message_t * 



CC2420ActiveMessageP__Receive__receive(
# 65 "/home/csilab/upma/chips/cc2420/CC2420ActiveMessageP.nc"
am_id_t arg_0x7f7fec082650, 
# 71 "/home/csilab/tinyos/tos/interfaces/Receive.nc"
message_t * msg, 
void * payload, 





uint8_t len);
# 120 "/home/csilab/tinyos/tos/interfaces/Resource.nc"
static error_t CC2420ActiveMessageP__RadioResource__release(void );
#line 97
static error_t CC2420ActiveMessageP__RadioResource__immediateRequest(void );
#line 88
static error_t CC2420ActiveMessageP__RadioResource__request(void );
# 86 "/home/csilab/upma/chips/cc2420/CC2420ActiveMessageP.nc"
uint16_t CC2420ActiveMessageP__pending_length;
message_t *CC2420ActiveMessageP__pending_message = (void *)0;

static void CC2420ActiveMessageP__RadioResource__granted(void );
#line 102
static error_t CC2420ActiveMessageP__AMSend__send(am_id_t id, am_addr_t addr, 
message_t *msg, 
uint8_t len);
#line 146
static inline am_addr_t CC2420ActiveMessageP__AMPacket__address(void );



static am_addr_t CC2420ActiveMessageP__AMPacket__destination(message_t *amsg);




static am_addr_t CC2420ActiveMessageP__AMPacket__source(message_t *amsg);




static void CC2420ActiveMessageP__AMPacket__setDestination(message_t *amsg, am_addr_t addr);




static inline void CC2420ActiveMessageP__AMPacket__setSource(message_t *amsg, am_addr_t addr);




static inline bool CC2420ActiveMessageP__AMPacket__isForMe(message_t *amsg);




static am_id_t CC2420ActiveMessageP__AMPacket__type(message_t *amsg);




static void CC2420ActiveMessageP__AMPacket__setType(message_t *amsg, am_id_t type);
#line 198
static inline uint8_t CC2420ActiveMessageP__AMPacket__headerSize(void );









static inline uint8_t CC2420ActiveMessageP__Packet__payloadLength(message_t *msg);



static inline void CC2420ActiveMessageP__Packet__setPayloadLength(message_t *msg, uint8_t len);



static inline uint8_t CC2420ActiveMessageP__Packet__maxPayloadLength(void );



static inline void *CC2420ActiveMessageP__Packet__getPayload(message_t *msg, uint8_t len);





static inline void CC2420ActiveMessageP__SubSend__sendDone(message_t *msg, error_t result);






static inline message_t *CC2420ActiveMessageP__SubReceive__receive(message_t *msg, void *payload, uint8_t len);
#line 249
static inline void CC2420ActiveMessageP__CC2420Config__syncDone(error_t error);



static inline uint16_t CC2420ActiveMessageP__SubCcaControl__getInitialBackoff(am_id_t amId, message_t *msg, uint16_t defaultBackoff);




static inline uint16_t CC2420ActiveMessageP__SubCcaControl__getCongestionBackoff(am_id_t amId, message_t *msg, uint16_t defaultBackoff);




static inline bool CC2420ActiveMessageP__SubCcaControl__getCca(am_id_t amId, message_t *msg, bool defaultValue);




static inline message_t *CC2420ActiveMessageP__Receive__default__receive(am_id_t id, message_t *msg, void *payload, uint8_t len);



static inline message_t *CC2420ActiveMessageP__Snoop__default__receive(am_id_t id, message_t *msg, void *payload, uint8_t len);







static inline void CC2420ActiveMessageP__SendNotifier__default__aboutToSend(am_id_t amId, am_addr_t addr, message_t *msg);


static inline uint16_t CC2420ActiveMessageP__CcaControl__default__getInitialBackoff(am_id_t amId, message_t *msg, uint16_t defaultBackoff);



static inline uint16_t CC2420ActiveMessageP__CcaControl__default__getCongestionBackoff(am_id_t amId, message_t *msg, uint16_t defaultBackoff);



static inline bool CC2420ActiveMessageP__CcaControl__default__getCca(am_id_t amId, message_t *msg, bool defaultValue);
# 46 "/home/csilab/tinyos/tos/interfaces/UartByte.nc"
static error_t SerialPrintfP__UartByte__send(uint8_t byte);
# 95 "/home/csilab/tinyos/tos/interfaces/StdControl.nc"
static error_t SerialPrintfP__UartControl__start(void );
# 50 "/home/csilab/tinyos/tos/lib/printf/SerialPrintfP.nc"
static inline error_t SerialPrintfP__Init__init(void );



static inline error_t SerialPrintfP__StdControl__start(void );
#line 69
static inline int SerialPrintfP__Putchar__putchar(int c);
# 64 "/home/csilab/tinyos/tos/lib/timer/Timer.nc"
static void /*TestPureTdmaC.SendingApp.App*/SendingC__0__SendTimer__startPeriodic(uint32_t dt);
#line 64
static void /*TestPureTdmaC.SendingApp.App*/SendingC__0__BenchmarkTimer__startPeriodic(uint32_t dt);
# 104 "/home/csilab/tinyos/tos/interfaces/SplitControl.nc"
static error_t /*TestPureTdmaC.SendingApp.App*/SendingC__0__SplitControl__start(void );
# 126 "/home/csilab/tinyos/tos/interfaces/Packet.nc"
static 
#line 123
void * 


/*TestPureTdmaC.SendingApp.App*/SendingC__0__Packet__getPayload(
#line 121
message_t * msg, 




uint8_t len);
# 67 "/home/csilab/tinyos/tos/interfaces/TaskBasic.nc"
static error_t /*TestPureTdmaC.SendingApp.App*/SendingC__0__sendTask__postTask(void );
# 80 "/home/csilab/tinyos/tos/interfaces/AMSend.nc"
static error_t /*TestPureTdmaC.SendingApp.App*/SendingC__0__AMSender__send(am_addr_t addr, 
#line 71
message_t * msg, 








uint8_t len);
# 67 "/home/csilab/tinyos/tos/interfaces/TaskBasic.nc"
static error_t /*TestPureTdmaC.SendingApp.App*/SendingC__0__flush__postTask(void );
# 64 "/home/csilab/tinyos/tos/lib/timer/Counter.nc"
static /*TestPureTdmaC.SendingApp.App*/SendingC__0__Counter__size_type /*TestPureTdmaC.SendingApp.App*/SendingC__0__Counter__get(void );
# 73 "/home/csilab/tinyos/tos/lib/timer/Timer.nc"
static void /*TestPureTdmaC.SendingApp.App*/SendingC__0__StartTimer__startOneShot(uint32_t dt);
#line 64
static void /*TestPureTdmaC.SendingApp.App*/SendingC__0__FlushTimer__startPeriodic(uint32_t dt);
# 67 "SendingC.nc"
enum /*TestPureTdmaC.SendingApp.App*/SendingC__0____nesc_unnamed4395 {
#line 67
  SendingC__0__sendTask = 26U
};
#line 67
typedef int /*TestPureTdmaC.SendingApp.App*/SendingC__0____nesc_sillytask_sendTask[/*TestPureTdmaC.SendingApp.App*/SendingC__0__sendTask];
#line 113
enum /*TestPureTdmaC.SendingApp.App*/SendingC__0____nesc_unnamed4396 {
#line 113
  SendingC__0__flush = 27U
};
#line 113
typedef int /*TestPureTdmaC.SendingApp.App*/SendingC__0____nesc_sillytask_flush[/*TestPureTdmaC.SendingApp.App*/SendingC__0__flush];
#line 55
enum /*TestPureTdmaC.SendingApp.App*/SendingC__0____nesc_unnamed4397 {

  SendingC__0__BENCHMARK_LENGTH = 10 * 1024
};

message_t /*TestPureTdmaC.SendingApp.App*/SendingC__0__packet;
uint8_t */*TestPureTdmaC.SendingApp.App*/SendingC__0__nodeId;
uint16_t /*TestPureTdmaC.SendingApp.App*/SendingC__0__packetsSent;
#line 62
uint16_t /*TestPureTdmaC.SendingApp.App*/SendingC__0__packetsReceived;
bool /*TestPureTdmaC.SendingApp.App*/SendingC__0__sends;
uint32_t /*TestPureTdmaC.SendingApp.App*/SendingC__0__sendTime;
uint32_t /*TestPureTdmaC.SendingApp.App*/SendingC__0__latency;

static inline void /*TestPureTdmaC.SendingApp.App*/SendingC__0__sendTask__runTask(void );
#line 84
static void /*TestPureTdmaC.SendingApp.App*/SendingC__0__Boot__booted(void );
#line 97
static void /*TestPureTdmaC.SendingApp.App*/SendingC__0__AMSender__sendDone(message_t *bufPtr, error_t error);
#line 113
static inline void /*TestPureTdmaC.SendingApp.App*/SendingC__0__flush__runTask(void );


static inline void /*TestPureTdmaC.SendingApp.App*/SendingC__0__Counter__overflow(void );




static inline void /*TestPureTdmaC.SendingApp.App*/SendingC__0__FlushTimer__fired(void );



static inline void /*TestPureTdmaC.SendingApp.App*/SendingC__0__SendTimer__fired(void );




static inline message_t */*TestPureTdmaC.SendingApp.App*/SendingC__0__AMReceiver__receive(message_t *message, void *payload, uint8_t length);






static void /*TestPureTdmaC.SendingApp.App*/SendingC__0__SplitControl__startDone(error_t err);
#line 150
static inline void /*TestPureTdmaC.SendingApp.App*/SendingC__0__SplitControl__stopDone(error_t err);



static inline void /*TestPureTdmaC.SendingApp.App*/SendingC__0__StartTimer__fired(void );




static inline void /*TestPureTdmaC.SendingApp.App*/SendingC__0__BenchmarkTimer__fired(void );
# 110 "/home/csilab/tinyos/tos/interfaces/AMSend.nc"
static void /*TestPureTdmaC.SendingApp.AMSender.SenderC.AMQueueEntryP*/AMQueueEntryP__1__AMSend__sendDone(
#line 103
message_t * msg, 






error_t error);
# 75 "/home/csilab/tinyos/tos/interfaces/Send.nc"
static error_t /*TestPureTdmaC.SendingApp.AMSender.SenderC.AMQueueEntryP*/AMQueueEntryP__1__Send__send(
#line 67
message_t * msg, 







uint8_t len);
# 92 "/home/csilab/upma/interfaces/AMPacket.nc"
static void /*TestPureTdmaC.SendingApp.AMSender.SenderC.AMQueueEntryP*/AMQueueEntryP__1__AMPacket__setDestination(
#line 88
message_t * amsg, 



am_addr_t addr);
#line 151
static void /*TestPureTdmaC.SendingApp.AMSender.SenderC.AMQueueEntryP*/AMQueueEntryP__1__AMPacket__setType(
#line 147
message_t * amsg, 



am_id_t t);
# 53 "/home/csilab/tinyos/tos/system/AMQueueEntryP.nc"
static inline error_t /*TestPureTdmaC.SendingApp.AMSender.SenderC.AMQueueEntryP*/AMQueueEntryP__1__AMSend__send(am_addr_t dest, 
message_t *msg, 
uint8_t len);









static inline void /*TestPureTdmaC.SendingApp.AMSender.SenderC.AMQueueEntryP*/AMQueueEntryP__1__Send__sendDone(message_t *m, error_t err);
# 80 "/home/csilab/tinyos/tos/interfaces/AMSend.nc"
static error_t /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__AMSend__send(
# 48 "/home/csilab/tinyos/tos/system/AMQueueImplP.nc"
am_id_t arg_0x7f7fed176840, 
# 80 "/home/csilab/tinyos/tos/interfaces/AMSend.nc"
am_addr_t addr, 
#line 71
message_t * msg, 








uint8_t len);
# 100 "/home/csilab/tinyos/tos/interfaces/Send.nc"
static void /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__Send__sendDone(
# 46 "/home/csilab/tinyos/tos/system/AMQueueImplP.nc"
uint8_t arg_0x7f7fed178630, 
# 96 "/home/csilab/tinyos/tos/interfaces/Send.nc"
message_t * msg, 



error_t error);
# 78 "/home/csilab/tinyos/tos/interfaces/Packet.nc"
static uint8_t /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__Packet__payloadLength(
#line 74
message_t * msg);
#line 94
static void /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__Packet__setPayloadLength(
#line 90
message_t * msg, 



uint8_t len);
# 67 "/home/csilab/tinyos/tos/interfaces/TaskBasic.nc"
static error_t /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__errorTask__postTask(void );
# 67 "/home/csilab/upma/interfaces/AMPacket.nc"
static am_addr_t /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__AMPacket__destination(
#line 63
message_t * amsg);
#line 136
static am_id_t /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__AMPacket__type(
#line 132
message_t * amsg);
# 126 "/home/csilab/tinyos/tos/system/AMQueueImplP.nc"
enum /*AMQueueP.AMQueueImplP*/AMQueueImplP__1____nesc_unnamed4398 {
#line 126
  AMQueueImplP__1__CancelTask = 28U
};
#line 126
typedef int /*AMQueueP.AMQueueImplP*/AMQueueImplP__1____nesc_sillytask_CancelTask[/*AMQueueP.AMQueueImplP*/AMQueueImplP__1__CancelTask];
#line 169
enum /*AMQueueP.AMQueueImplP*/AMQueueImplP__1____nesc_unnamed4399 {
#line 169
  AMQueueImplP__1__errorTask = 29U
};
#line 169
typedef int /*AMQueueP.AMQueueImplP*/AMQueueImplP__1____nesc_sillytask_errorTask[/*AMQueueP.AMQueueImplP*/AMQueueImplP__1__errorTask];
#line 57
#line 55
typedef struct /*AMQueueP.AMQueueImplP*/AMQueueImplP__1____nesc_unnamed4400 {
  message_t * msg;
} /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__queue_entry_t;

uint8_t /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__current = 1;
/*AMQueueP.AMQueueImplP*/AMQueueImplP__1__queue_entry_t /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__queue[1];
uint8_t /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__cancelMask[1 / 8 + 1];

static inline void /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__tryToSend(void );

static inline void /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__nextPacket(void );
#line 90
static inline error_t /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__Send__send(uint8_t clientId, message_t *msg, 
uint8_t len);
#line 126
static inline void /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__CancelTask__runTask(void );
#line 163
static void /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__sendDone(uint8_t last, message_t * msg, error_t err);





static inline void /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__errorTask__runTask(void );




static inline void /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__tryToSend(void );
#line 189
static void /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__AMSend__sendDone(am_id_t id, message_t *msg, error_t err);
#line 215
static inline void /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__Send__default__sendDone(uint8_t id, message_t *msg, error_t err);
# 397 "/home/csilab/tinyos/tos/chips/msp430/msp430hardware.h"
static inline  void __nesc_enable_interrupt(void )
{
  __eint();
}

# 98 "/home/csilab/tinyos/tos/interfaces/ArbiterInfo.nc"
inline static uint8_t /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__0__ArbiterInfo__userId(void ){
#line 98
  unsigned char __nesc_result;
#line 98

#line 98
  __nesc_result = /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ArbiterInfo__userId();
#line 98

#line 98
  return __nesc_result;
#line 98
}
#line 98
# 397 "/home/csilab/tinyos/tos/lib/serial/SerialP.nc"
static inline void SerialP__SerialFrameComm__dataReceived(uint8_t data)
#line 397
{
  SerialP__rx_state_machine(FALSE, data);
}

# 94 "/home/csilab/tinyos/tos/lib/serial/SerialFrameComm.nc"
inline static void HdlcTranslateC__SerialFrameComm__dataReceived(uint8_t data){
#line 94
  SerialP__SerialFrameComm__dataReceived(data);
#line 94
}
#line 94
# 394 "/home/csilab/tinyos/tos/lib/serial/SerialP.nc"
static inline void SerialP__SerialFrameComm__delimiterReceived(void )
#line 394
{
  SerialP__rx_state_machine(TRUE, 0);
}

# 85 "/home/csilab/tinyos/tos/lib/serial/SerialFrameComm.nc"
inline static void HdlcTranslateC__SerialFrameComm__delimiterReceived(void ){
#line 85
  SerialP__SerialFrameComm__delimiterReceived();
#line 85
}
#line 85
# 73 "/home/csilab/tinyos/tos/lib/serial/HdlcTranslateC.nc"
static inline void HdlcTranslateC__UartStream__receivedByte(uint8_t data)
#line 73
{






  if (data == HDLC_FLAG_BYTE) {

      HdlcTranslateC__SerialFrameComm__delimiterReceived();
      return;
    }
  else {
#line 85
    if (data == HDLC_CTLESC_BYTE) {

        HdlcTranslateC__state.receiveEscape = 1;
        return;
      }
    else {
#line 90
      if (HdlcTranslateC__state.receiveEscape) {

          HdlcTranslateC__state.receiveEscape = 0;
          data = data ^ 0x20;
        }
      }
    }
#line 95
  HdlcTranslateC__SerialFrameComm__dataReceived(data);
}

# 221 "/home/csilab/tinyos/tos/chips/msp430/usart/Msp430UartP.nc"
static inline void /*Msp430Uart1P.UartP*/Msp430UartP__0__UartStream__default__receivedByte(uint8_t id, uint8_t byte)
#line 221
{
}

# 79 "/home/csilab/tinyos/tos/interfaces/UartStream.nc"
inline static void /*Msp430Uart1P.UartP*/Msp430UartP__0__UartStream__receivedByte(uint8_t arg_0x7f7fed84c4c0, uint8_t byte){
#line 79
  switch (arg_0x7f7fed84c4c0) {
#line 79
    case /*PlatformSerialC.UartC*/Msp430Uart1C__0__CLIENT_ID:
#line 79
      HdlcTranslateC__UartStream__receivedByte(byte);
#line 79
      break;
#line 79
    default:
#line 79
      /*Msp430Uart1P.UartP*/Msp430UartP__0__UartStream__default__receivedByte(arg_0x7f7fed84c4c0, byte);
#line 79
      break;
#line 79
    }
#line 79
}
#line 79
# 132 "/home/csilab/tinyos/tos/lib/serial/HdlcTranslateC.nc"
static inline void HdlcTranslateC__UartStream__receiveDone(uint8_t *buf, uint16_t len, error_t error)
#line 132
{
}

# 222 "/home/csilab/tinyos/tos/chips/msp430/usart/Msp430UartP.nc"
static inline void /*Msp430Uart1P.UartP*/Msp430UartP__0__UartStream__default__receiveDone(uint8_t id, uint8_t *buf, uint16_t len, error_t error)
#line 222
{
}

# 99 "/home/csilab/tinyos/tos/interfaces/UartStream.nc"
inline static void /*Msp430Uart1P.UartP*/Msp430UartP__0__UartStream__receiveDone(uint8_t arg_0x7f7fed84c4c0, uint8_t * buf, uint16_t len, error_t error){
#line 99
  switch (arg_0x7f7fed84c4c0) {
#line 99
    case /*PlatformSerialC.UartC*/Msp430Uart1C__0__CLIENT_ID:
#line 99
      HdlcTranslateC__UartStream__receiveDone(buf, len, error);
#line 99
      break;
#line 99
    default:
#line 99
      /*Msp430Uart1P.UartP*/Msp430UartP__0__UartStream__default__receiveDone(arg_0x7f7fed84c4c0, buf, len, error);
#line 99
      break;
#line 99
    }
#line 99
}
#line 99
# 134 "/home/csilab/tinyos/tos/chips/msp430/usart/Msp430UartP.nc"
static inline void /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartInterrupts__rxDone(uint8_t id, uint8_t data)
#line 134
{
  if (/*Msp430Uart1P.UartP*/Msp430UartP__0__m_rx_buf) {
      /*Msp430Uart1P.UartP*/Msp430UartP__0__m_rx_buf[/*Msp430Uart1P.UartP*/Msp430UartP__0__m_rx_pos++] = data;
      if (/*Msp430Uart1P.UartP*/Msp430UartP__0__m_rx_pos >= /*Msp430Uart1P.UartP*/Msp430UartP__0__m_rx_len) {
          uint8_t *buf = /*Msp430Uart1P.UartP*/Msp430UartP__0__m_rx_buf;

#line 139
          /*Msp430Uart1P.UartP*/Msp430UartP__0__m_rx_buf = (void *)0;
          /*Msp430Uart1P.UartP*/Msp430UartP__0__UartStream__receiveDone(id, buf, /*Msp430Uart1P.UartP*/Msp430UartP__0__m_rx_len, SUCCESS);
        }
    }
  else 
#line 142
    {
      /*Msp430Uart1P.UartP*/Msp430UartP__0__UartStream__receivedByte(id, data);
    }
}

# 65 "/home/csilab/tinyos/tos/chips/msp430/usart/Msp430UsartShareP.nc"
static inline void /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__0__Interrupts__default__rxDone(uint8_t id, uint8_t data)
#line 65
{
}

# 54 "/home/csilab/tinyos/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
inline static void /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__0__Interrupts__rxDone(uint8_t arg_0x7f7fed2b0600, uint8_t data){
#line 54
  switch (arg_0x7f7fed2b0600) {
#line 54
    case /*PlatformSerialC.UartC.UsartC*/Msp430Usart1C__0__CLIENT_ID:
#line 54
      /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartInterrupts__rxDone(/*PlatformSerialC.UartC*/Msp430Uart1C__0__CLIENT_ID, data);
#line 54
      break;
#line 54
    default:
#line 54
      /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__0__Interrupts__default__rxDone(arg_0x7f7fed2b0600, data);
#line 54
      break;
#line 54
    }
#line 54
}
#line 54
# 90 "/home/csilab/tinyos/tos/interfaces/ArbiterInfo.nc"
inline static bool /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__0__ArbiterInfo__inUse(void ){
#line 90
  unsigned char __nesc_result;
#line 90

#line 90
  __nesc_result = /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ArbiterInfo__inUse();
#line 90

#line 90
  return __nesc_result;
#line 90
}
#line 90
# 54 "/home/csilab/tinyos/tos/chips/msp430/usart/Msp430UsartShareP.nc"
static inline void /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__0__RawInterrupts__rxDone(uint8_t data)
#line 54
{
  if (/*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__0__ArbiterInfo__inUse()) {
    /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__0__Interrupts__rxDone(/*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__0__ArbiterInfo__userId(), data);
    }
}

# 54 "/home/csilab/tinyos/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
inline static void HplMsp430Usart1P__Interrupts__rxDone(uint8_t data){
#line 54
  /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__0__RawInterrupts__rxDone(data);
#line 54
}
#line 54
# 401 "/home/csilab/tinyos/tos/lib/serial/SerialP.nc"
static inline bool SerialP__valid_rx_proto(uint8_t proto)
#line 401
{
  switch (proto) {
      case SERIAL_PROTO_PACKET_ACK: 
        return TRUE;
      case SERIAL_PROTO_ACK: 
        case SERIAL_PROTO_PACKET_NOACK: 
          default: 
            return FALSE;
    }
}

# 203 "/home/csilab/tinyos/tos/lib/serial/SerialDispatcherP.nc"
static inline void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__lockCurrentBuffer(void )
#line 203
{
  if (/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveState.which) {
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveState.bufOneLocked = 1;
    }
  else {
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveState.bufZeroLocked = 1;
    }
}

#line 199
static inline bool /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__isCurrentBufferLocked(void )
#line 199
{
  return /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveState.which ? /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveState.bufOneLocked : /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveState.bufZeroLocked;
}

#line 226
static inline error_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__ReceiveBytePacket__startPacket(void )
#line 226
{
  error_t result = SUCCESS;

  /* atomic removed: atomic calls only */
#line 228
  {
    if (!/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__isCurrentBufferLocked()) {


        /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__lockCurrentBuffer();
        /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveState.state = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__RECV_STATE_BEGIN;
        /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__recvIndex = 0;
        /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__recvType = TOS_SERIAL_UNKNOWN_ID;
      }
    else {
        result = EBUSY;
      }
  }
  return result;
}

# 62 "/home/csilab/tinyos/tos/lib/serial/ReceiveBytePacket.nc"
inline static error_t SerialP__ReceiveBytePacket__startPacket(void ){
#line 62
  unsigned char __nesc_result;
#line 62

#line 62
  __nesc_result = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__ReceiveBytePacket__startPacket();
#line 62

#line 62
  return __nesc_result;
#line 62
}
#line 62
# 311 "/home/csilab/tinyos/tos/lib/serial/SerialP.nc"
static __inline uint16_t SerialP__rx_current_crc(void )
#line 311
{
  uint16_t crc;
  uint8_t tmp = SerialP__rxBuf.writePtr;

#line 314
  tmp = tmp == 0 ? SerialP__RX_DATA_BUFFER_SIZE : tmp - 1;
  crc = SerialP__rxBuf.buf[tmp] & 0x00ff;
  crc = (crc << 8) & 0xFF00;
  tmp = tmp == 0 ? SerialP__RX_DATA_BUFFER_SIZE : tmp - 1;
  crc |= SerialP__rxBuf.buf[tmp] & 0x00FF;
  return crc;
}

# 80 "/home/csilab/tinyos/tos/lib/serial/ReceiveBytePacket.nc"
inline static void SerialP__ReceiveBytePacket__endPacket(error_t result){
#line 80
  /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__ReceiveBytePacket__endPacket(result);
#line 80
}
#line 80
# 221 "/home/csilab/tinyos/tos/lib/serial/SerialDispatcherP.nc"
static inline void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveBufferSwap(void )
#line 221
{
  /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveState.which = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveState.which ? 0 : 1;
  /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveBuffer = (uint8_t *)/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__messagePtrs[/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveState.which];
}

# 67 "/home/csilab/tinyos/tos/interfaces/TaskBasic.nc"
inline static error_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTask__postTask(void ){
#line 67
  unsigned char __nesc_result;
#line 67

#line 67
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTask);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 97 "/home/csilab/tinyos/tos/system/SchedulerBasicP.nc"
static inline bool SchedulerBasicP__isWaiting(uint8_t id)
{
  return SchedulerBasicP__m_next[id] != SchedulerBasicP__NO_TASK || SchedulerBasicP__m_tail == id;
}

static inline bool SchedulerBasicP__pushTask(uint8_t id)
{
  if (!SchedulerBasicP__isWaiting(id)) 
    {
      if (SchedulerBasicP__m_head == SchedulerBasicP__NO_TASK) 
        {
          SchedulerBasicP__m_head = id;
          SchedulerBasicP__m_tail = id;
        }
      else 
        {
          SchedulerBasicP__m_next[SchedulerBasicP__m_tail] = id;
          SchedulerBasicP__m_tail = id;
        }
      return TRUE;
    }
  else 
    {
      return FALSE;
    }
}

# 234 "/home/csilab/tinyos/tos/lib/serial/SerialP.nc"
static __inline bool SerialP__ack_queue_is_full(void )
#line 234
{
  uint8_t tmp;
#line 235
  uint8_t tmp2;

  /* atomic removed: atomic calls only */
#line 236
  {
    tmp = SerialP__ackQ.writePtr;
    tmp2 = SerialP__ackQ.readPtr;
  }
  if (++tmp > SerialP__ACK_QUEUE_SIZE) {
#line 240
    tmp = 0;
    }
#line 241
  return tmp == tmp2;
}







static __inline void SerialP__ack_queue_push(uint8_t token)
#line 250
{
  if (!SerialP__ack_queue_is_full()) {
      /* atomic removed: atomic calls only */
#line 252
      {
        SerialP__ackQ.buf[SerialP__ackQ.writePtr] = token;
        if (++ SerialP__ackQ.writePtr > SerialP__ACK_QUEUE_SIZE) {
#line 254
          SerialP__ackQ.writePtr = 0;
          }
      }
#line 256
      SerialP__MaybeScheduleTx();
    }
}

# 67 "/home/csilab/tinyos/tos/interfaces/TaskBasic.nc"
inline static error_t SerialP__RunTx__postTask(void ){
#line 67
  unsigned char __nesc_result;
#line 67

#line 67
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(SerialP__RunTx);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 67 "/home/csilab/tinyos/tos/lib/serial/HdlcTranslateC.nc"
static inline void HdlcTranslateC__SerialFrameComm__resetReceive(void )
#line 67
{
  HdlcTranslateC__state.receiveEscape = 0;
}

# 79 "/home/csilab/tinyos/tos/lib/serial/SerialFrameComm.nc"
inline static void SerialP__SerialFrameComm__resetReceive(void ){
#line 79
  HdlcTranslateC__SerialFrameComm__resetReceive();
#line 79
}
#line 79
# 67 "/home/csilab/tinyos/tos/interfaces/TaskBasic.nc"
inline static error_t SerialP__stopDoneTask__postTask(void ){
#line 67
  unsigned char __nesc_result;
#line 67

#line 67
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(SerialP__stopDoneTask);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 51 "/home/csilab/tinyos/tos/lib/serial/SerialPacketInfoActiveMessageP.nc"
static inline uint8_t SerialPacketInfoActiveMessageP__Info__offset(void )
#line 51
{
  return (uint8_t )(sizeof(message_header_t ) - sizeof(serial_header_t ));
}

# 358 "/home/csilab/tinyos/tos/lib/serial/SerialDispatcherP.nc"
static inline uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__default__offset(uart_id_t id)
#line 358
{
  return 0;
}

# 15 "/home/csilab/tinyos/tos/lib/serial/SerialPacketInfo.nc"
inline static uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__offset(uart_id_t arg_0x7f7fed95be30){
#line 15
  unsigned char __nesc_result;
#line 15

#line 15
  switch (arg_0x7f7fed95be30) {
#line 15
    case TOS_SERIAL_ACTIVE_MESSAGE_ID:
#line 15
      __nesc_result = SerialPacketInfoActiveMessageP__Info__offset();
#line 15
      break;
#line 15
    default:
#line 15
      __nesc_result = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__default__offset(arg_0x7f7fed95be30);
#line 15
      break;
#line 15
    }
#line 15

#line 15
  return __nesc_result;
#line 15
}
#line 15
# 244 "/home/csilab/tinyos/tos/lib/serial/SerialDispatcherP.nc"
static inline void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__ReceiveBytePacket__byteReceived(uint8_t b)
#line 244
{
  /* atomic removed: atomic calls only */
#line 245
  {
    switch (/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveState.state) {
        case /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__RECV_STATE_BEGIN: 
          /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveState.state = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__RECV_STATE_DATA;
        /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__recvIndex = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__offset(b);
        /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__recvType = b;
        break;

        case /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__RECV_STATE_DATA: 
          if (/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__recvIndex < sizeof(message_t )) {
              /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveBuffer[/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__recvIndex] = b;
              /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__recvIndex++;
            }
          else {
            }




        break;

        case /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__RECV_STATE_IDLE: 
          default: 
#line 266
            ;
      }
  }
}

# 69 "/home/csilab/tinyos/tos/lib/serial/ReceiveBytePacket.nc"
inline static void SerialP__ReceiveBytePacket__byteReceived(uint8_t data){
#line 69
  /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__ReceiveBytePacket__byteReceived(data);
#line 69
}
#line 69
# 301 "/home/csilab/tinyos/tos/lib/serial/SerialP.nc"
static __inline uint8_t SerialP__rx_buffer_top(void )
#line 301
{
  uint8_t tmp = SerialP__rxBuf.buf[SerialP__rxBuf.readPtr];

#line 303
  return tmp;
}

#line 305
static __inline uint8_t SerialP__rx_buffer_pop(void )
#line 305
{
  uint8_t tmp = SerialP__rxBuf.buf[SerialP__rxBuf.readPtr];

#line 307
  if (++ SerialP__rxBuf.readPtr > SerialP__RX_DATA_BUFFER_SIZE) {
#line 307
    SerialP__rxBuf.readPtr = 0;
    }
#line 308
  return tmp;
}

#line 297
static __inline void SerialP__rx_buffer_push(uint8_t data)
#line 297
{
  SerialP__rxBuf.buf[SerialP__rxBuf.writePtr] = data;
  if (++ SerialP__rxBuf.writePtr > SerialP__RX_DATA_BUFFER_SIZE) {
#line 299
    SerialP__rxBuf.writePtr = 0;
    }
}

# 220 "/home/csilab/tinyos/tos/chips/msp430/usart/Msp430UartP.nc"
static inline void /*Msp430Uart1P.UartP*/Msp430UartP__0__UartStream__default__sendDone(uint8_t id, uint8_t *buf, uint16_t len, error_t error)
#line 220
{
}

# 57 "/home/csilab/tinyos/tos/interfaces/UartStream.nc"
inline static void /*Msp430Uart1P.UartP*/Msp430UartP__0__UartStream__sendDone(uint8_t arg_0x7f7fed84c4c0, uint8_t * buf, uint16_t len, error_t error){
#line 57
  switch (arg_0x7f7fed84c4c0) {
#line 57
    case /*PlatformSerialC.UartC*/Msp430Uart1C__0__CLIENT_ID:
#line 57
      HdlcTranslateC__UartStream__sendDone(buf, len, error);
#line 57
      break;
#line 57
    default:
#line 57
      /*Msp430Uart1P.UartP*/Msp430UartP__0__UartStream__default__sendDone(arg_0x7f7fed84c4c0, buf, len, error);
#line 57
      break;
#line 57
    }
#line 57
}
#line 57
# 384 "/home/csilab/tinyos/tos/chips/msp430/usart/HplMsp430Usart1P.nc"
static inline void HplMsp430Usart1P__Usart__tx(uint8_t data)
#line 384
{
  HplMsp430Usart1P__U1TXBUF = data;
}

# 224 "/home/csilab/tinyos/tos/chips/msp430/usart/HplMsp430Usart.nc"
inline static void /*Msp430Uart1P.UartP*/Msp430UartP__0__Usart__tx(uint8_t data){
#line 224
  HplMsp430Usart1P__Usart__tx(data);
#line 224
}
#line 224
# 162 "/home/csilab/tinyos/tos/chips/msp430/usart/Msp430UartP.nc"
static inline void /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartInterrupts__txDone(uint8_t id)
#line 162
{
  if (/*Msp430Uart1P.UartP*/Msp430UartP__0__current_owner != id) {
      uint8_t *buf = /*Msp430Uart1P.UartP*/Msp430UartP__0__m_tx_buf;

#line 165
      /*Msp430Uart1P.UartP*/Msp430UartP__0__m_tx_buf = (void *)0;
      /*Msp430Uart1P.UartP*/Msp430UartP__0__UartStream__sendDone(id, buf, /*Msp430Uart1P.UartP*/Msp430UartP__0__m_tx_len, FAIL);
    }
  else {
#line 168
    if (/*Msp430Uart1P.UartP*/Msp430UartP__0__m_tx_pos < /*Msp430Uart1P.UartP*/Msp430UartP__0__m_tx_len) {
        /*Msp430Uart1P.UartP*/Msp430UartP__0__Usart__tx(/*Msp430Uart1P.UartP*/Msp430UartP__0__m_tx_buf[/*Msp430Uart1P.UartP*/Msp430UartP__0__m_tx_pos++]);
      }
    else {
        uint8_t *buf = /*Msp430Uart1P.UartP*/Msp430UartP__0__m_tx_buf;

#line 173
        /*Msp430Uart1P.UartP*/Msp430UartP__0__m_tx_buf = (void *)0;
        /*Msp430Uart1P.UartP*/Msp430UartP__0__UartStream__sendDone(id, buf, /*Msp430Uart1P.UartP*/Msp430UartP__0__m_tx_len, SUCCESS);
      }
    }
}

# 64 "/home/csilab/tinyos/tos/chips/msp430/usart/Msp430UsartShareP.nc"
static inline void /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__0__Interrupts__default__txDone(uint8_t id)
#line 64
{
}

# 49 "/home/csilab/tinyos/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
inline static void /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__0__Interrupts__txDone(uint8_t arg_0x7f7fed2b0600){
#line 49
  switch (arg_0x7f7fed2b0600) {
#line 49
    case /*PlatformSerialC.UartC.UsartC*/Msp430Usart1C__0__CLIENT_ID:
#line 49
      /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartInterrupts__txDone(/*PlatformSerialC.UartC*/Msp430Uart1C__0__CLIENT_ID);
#line 49
      break;
#line 49
    default:
#line 49
      /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__0__Interrupts__default__txDone(arg_0x7f7fed2b0600);
#line 49
      break;
#line 49
    }
#line 49
}
#line 49
# 49 "/home/csilab/tinyos/tos/chips/msp430/usart/Msp430UsartShareP.nc"
static inline void /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__0__RawInterrupts__txDone(void )
#line 49
{
  if (/*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__0__ArbiterInfo__inUse()) {
    /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__0__Interrupts__txDone(/*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__0__ArbiterInfo__userId());
    }
}

# 49 "/home/csilab/tinyos/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
inline static void HplMsp430Usart1P__Interrupts__txDone(void ){
#line 49
  /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__0__RawInterrupts__txDone();
#line 49
}
#line 49
# 48 "/home/csilab/tinyos/tos/interfaces/UartStream.nc"
inline static error_t HdlcTranslateC__UartStream__send(uint8_t * buf, uint16_t len){
#line 48
  unsigned char __nesc_result;
#line 48

#line 48
  __nesc_result = /*Msp430Uart1P.UartP*/Msp430UartP__0__UartStream__send(/*PlatformSerialC.UartC*/Msp430Uart1C__0__CLIENT_ID, buf, len);
#line 48

#line 48
  return __nesc_result;
#line 48
}
#line 48
# 56 "/home/csilab/tinyos/tos/lib/serial/SerialFrameComm.nc"
inline static error_t SerialP__SerialFrameComm__putDelimiter(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = HdlcTranslateC__SerialFrameComm__putDelimiter();
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56









inline static error_t SerialP__SerialFrameComm__putData(uint8_t data){
#line 65
  unsigned char __nesc_result;
#line 65

#line 65
  __nesc_result = HdlcTranslateC__SerialFrameComm__putData(data);
#line 65

#line 65
  return __nesc_result;
#line 65
}
#line 65
# 529 "/home/csilab/tinyos/tos/lib/serial/SerialP.nc"
static inline error_t SerialP__SendBytePacket__completeSend(void )
#line 529
{
  bool ret = FAIL;

  /* atomic removed: atomic calls only */
#line 531
  {
    SerialP__txBuf[SerialP__TX_DATA_INDEX].state = SerialP__BUFFER_COMPLETE;
    ret = SUCCESS;
  }
  return ret;
}

# 71 "/home/csilab/tinyos/tos/lib/serial/SendBytePacket.nc"
inline static error_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__SendBytePacket__completeSend(void ){
#line 71
  unsigned char __nesc_result;
#line 71

#line 71
  __nesc_result = SerialP__SendBytePacket__completeSend();
#line 71

#line 71
  return __nesc_result;
#line 71
}
#line 71
# 178 "/home/csilab/tinyos/tos/lib/serial/SerialDispatcherP.nc"
static inline uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__SendBytePacket__nextByte(void )
#line 178
{
  uint8_t b;
  uint8_t indx;

  /* atomic removed: atomic calls only */
#line 181
  {
    b = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendBuffer[/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendIndex];
    /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendIndex++;
    indx = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendIndex;
  }
  if (indx > /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendLen) {
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__SendBytePacket__completeSend();
      return 0;
    }
  else {
      return b;
    }
}

# 81 "/home/csilab/tinyos/tos/lib/serial/SendBytePacket.nc"
inline static uint8_t SerialP__SendBytePacket__nextByte(void ){
#line 81
  unsigned char __nesc_result;
#line 81

#line 81
  __nesc_result = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__SendBytePacket__nextByte();
#line 81

#line 81
  return __nesc_result;
#line 81
}
#line 81
# 668 "/home/csilab/tinyos/tos/lib/serial/SerialP.nc"
static inline void SerialP__SerialFrameComm__putDone(void )
#line 668
{
  {
    error_t txResult = SUCCESS;

    /* atomic removed: atomic calls only */
#line 671
    {
      switch (SerialP__txState) {

          case SerialP__TXSTATE_PROTO: 

            txResult = SerialP__SerialFrameComm__putData(SerialP__txProto);

          SerialP__txState = SerialP__TXSTATE_INFO;



          SerialP__txCRC = crcByte(SerialP__txCRC, SerialP__txProto);
          break;

          case SerialP__TXSTATE_SEQNO: 
            txResult = SerialP__SerialFrameComm__putData(SerialP__txSeqno);
          SerialP__txState = SerialP__TXSTATE_INFO;
          SerialP__txCRC = crcByte(SerialP__txCRC, SerialP__txSeqno);
          break;

          case SerialP__TXSTATE_INFO: 
            {
              txResult = SerialP__SerialFrameComm__putData(SerialP__txBuf[SerialP__txIndex].buf);
              SerialP__txCRC = crcByte(SerialP__txCRC, SerialP__txBuf[SerialP__txIndex].buf);
              ++SerialP__txByteCnt;

              if (SerialP__txIndex == SerialP__TX_DATA_INDEX) {
                  uint8_t nextByte;

#line 699
                  nextByte = SerialP__SendBytePacket__nextByte();
                  if (SerialP__txBuf[SerialP__txIndex].state == SerialP__BUFFER_COMPLETE || SerialP__txByteCnt >= SerialP__SERIAL_MTU) {
                      SerialP__txState = SerialP__TXSTATE_FCS1;
                    }
                  else {
                      SerialP__txBuf[SerialP__txIndex].buf = nextByte;
                    }
                }
              else {
                  SerialP__txState = SerialP__TXSTATE_FCS1;
                }
            }
          break;

          case SerialP__TXSTATE_FCS1: 
            txResult = SerialP__SerialFrameComm__putData(SerialP__txCRC & 0xff);
          SerialP__txState = SerialP__TXSTATE_FCS2;
          break;

          case SerialP__TXSTATE_FCS2: 
            txResult = SerialP__SerialFrameComm__putData((SerialP__txCRC >> 8) & 0xff);
          SerialP__txState = SerialP__TXSTATE_ENDFLAG;
          break;

          case SerialP__TXSTATE_ENDFLAG: 
            txResult = SerialP__SerialFrameComm__putDelimiter();
          SerialP__txState = SerialP__TXSTATE_ENDWAIT;
          break;

          case SerialP__TXSTATE_ENDWAIT: 
            SerialP__txState = SerialP__TXSTATE_FINISH;
          case SerialP__TXSTATE_FINISH: 
            SerialP__MaybeScheduleTx();
          break;
          case SerialP__TXSTATE_ERROR: 
            default: 
              txResult = FAIL;
          break;
        }

      if (txResult != SUCCESS) {
          SerialP__txState = SerialP__TXSTATE_ERROR;
          SerialP__MaybeScheduleTx();
        }
    }
  }
}

# 100 "/home/csilab/tinyos/tos/lib/serial/SerialFrameComm.nc"
inline static void HdlcTranslateC__SerialFrameComm__putDone(void ){
#line 100
  SerialP__SerialFrameComm__putDone();
#line 100
}
#line 100
# 196 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Timer__overflow(void )
{
}

#line 196
static inline void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Timer__overflow(void )
{
}

#line 196
static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Timer__overflow(void )
{
}

# 83 "/home/csilab/tinyos/tos/lib/timer/BusyWaitCounterC.nc"
static inline void /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__Counter__overflow(void )
{
}

# 82 "/home/csilab/tinyos/tos/lib/timer/Counter.nc"
inline static void /*Msp430CounterMicroC.Counter*/Msp430CounterC__1__Counter__overflow(void ){
#line 82
  /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__Counter__overflow();
#line 82
}
#line 82
# 64 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430CounterC.nc"
static inline void /*Msp430CounterMicroC.Counter*/Msp430CounterC__1__Msp430Timer__overflow(void )
{
  /*Msp430CounterMicroC.Counter*/Msp430CounterC__1__Counter__overflow();
}

# 48 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430Timer.nc"
inline static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__overflow(void ){
#line 48
  /*Msp430CounterMicroC.Counter*/Msp430CounterC__1__Msp430Timer__overflow();
#line 48
  /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Timer__overflow();
#line 48
  /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Timer__overflow();
#line 48
  /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Timer__overflow();
#line 48
}
#line 48
# 137 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Overflow__fired(void )
{
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__overflow();
}





static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__default__fired(uint8_t n)
{
}

# 39 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430TimerEvent.nc"
inline static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__fired(uint8_t arg_0x7f7fed4c58b0){
#line 39
  switch (arg_0x7f7fed4c58b0) {
#line 39
    case 0:
#line 39
      /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Event__fired();
#line 39
      break;
#line 39
    case 1:
#line 39
      /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Event__fired();
#line 39
      break;
#line 39
    case 2:
#line 39
      /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Event__fired();
#line 39
      break;
#line 39
    case 5:
#line 39
      /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Overflow__fired();
#line 39
      break;
#line 39
    default:
#line 39
      /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__default__fired(arg_0x7f7fed4c58b0);
#line 39
      break;
#line 39
    }
#line 39
}
#line 39
# 126 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__VectorTimerX0__fired(void )
{
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__fired(0);
}

# 39 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430TimerEvent.nc"
inline static void Msp430TimerCommonP__VectorTimerA0__fired(void ){
#line 39
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__VectorTimerX0__fired();
#line 39
}
#line 39
# 58 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__cc_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__int2CC(uint16_t x)
#line 58
{
#line 58
  union /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0____nesc_unnamed4401 {
#line 58
    uint16_t f;
#line 58
    /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__cc_t t;
  } 
#line 58
  c = { .f = x };

#line 58
  return c.t;
}

#line 85
static inline /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__cc_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Control__getControl(void )
{
  return /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__int2CC(* (volatile uint16_t * )354U);
}

#line 188
static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__default__captured(uint16_t n)
{
}

# 86 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__captured(uint16_t time){
#line 86
  /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__default__captured(time);
#line 86
}
#line 86
# 150 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__getEvent(void )
{
  return * (volatile uint16_t * )370U;
}

#line 192
static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Compare__default__fired(void )
{
}

# 45 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Compare__fired(void ){
#line 45
  /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Compare__default__fired();
#line 45
}
#line 45
# 58 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__cc_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__int2CC(uint16_t x)
#line 58
{
#line 58
  union /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1____nesc_unnamed4402 {
#line 58
    uint16_t f;
#line 58
    /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__cc_t t;
  } 
#line 58
  c = { .f = x };

#line 58
  return c.t;
}

#line 85
static inline /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__cc_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Control__getControl(void )
{
  return /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__int2CC(* (volatile uint16_t * )356U);
}

#line 188
static inline void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__default__captured(uint16_t n)
{
}

# 86 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__captured(uint16_t time){
#line 86
  /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__default__captured(time);
#line 86
}
#line 86
# 150 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__getEvent(void )
{
  return * (volatile uint16_t * )372U;
}

#line 192
static inline void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Compare__default__fired(void )
{
}

# 45 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Compare__fired(void ){
#line 45
  /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Compare__default__fired();
#line 45
}
#line 45
# 58 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__cc_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__int2CC(uint16_t x)
#line 58
{
#line 58
  union /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2____nesc_unnamed4403 {
#line 58
    uint16_t f;
#line 58
    /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__cc_t t;
  } 
#line 58
  c = { .f = x };

#line 58
  return c.t;
}

#line 85
static inline /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__cc_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Control__getControl(void )
{
  return /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__int2CC(* (volatile uint16_t * )358U);
}

#line 188
static inline void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__default__captured(uint16_t n)
{
}

# 86 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__captured(uint16_t time){
#line 86
  /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__default__captured(time);
#line 86
}
#line 86
# 150 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__getEvent(void )
{
  return * (volatile uint16_t * )374U;
}

#line 192
static inline void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Compare__default__fired(void )
{
}

# 45 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Compare__fired(void ){
#line 45
  /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Compare__default__fired();
#line 45
}
#line 45
# 131 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__VectorTimerX1__fired(void )
{
  uint8_t n = * (volatile uint16_t * )302U;

#line 134
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__fired(n >> 1);
}

# 39 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430TimerEvent.nc"
inline static void Msp430TimerCommonP__VectorTimerA1__fired(void ){
#line 39
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__VectorTimerX1__fired();
#line 39
}
#line 39
# 126 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__VectorTimerX0__fired(void )
{
  /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__fired(0);
}

# 39 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430TimerEvent.nc"
inline static void Msp430TimerCommonP__VectorTimerB0__fired(void ){
#line 39
  /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__VectorTimerX0__fired();
#line 39
}
#line 39
# 196 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Timer__overflow(void )
{
}

#line 196
static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Timer__overflow(void )
{
}

#line 196
static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Timer__overflow(void )
{
}

#line 196
static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Timer__overflow(void )
{
}

#line 196
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Timer__overflow(void )
{
}

#line 196
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Timer__overflow(void )
{
}

#line 196
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Timer__overflow(void )
{
}

# 208 "/home/csilab/tinyos/tos/chips/msp430/usart/Msp430UartP.nc"
static inline void /*Msp430Uart1P.UartP*/Msp430UartP__0__Counter__overflow(void )
#line 208
{
}

# 347 "/usr/local/lib/ncc/nesc_nx.h"
static __inline  uint32_t __nesc_hton_uint32(void * target, uint32_t value)
#line 347
{
  uint8_t *base = target;

#line 349
  base[3] = value;
  base[2] = value >> 8;
  base[1] = value >> 16;
  base[0] = value >> 24;
  return value;
}

#line 340
static __inline  uint32_t __nesc_ntoh_uint32(const void * source)
#line 340
{
  const uint8_t *base = source;

#line 342
  return ((((uint32_t )base[0] << 24) | (
  (uint32_t )base[1] << 16)) | (
  (uint32_t )base[2] << 8)) | base[3];
}

# 117 "/home/csilab/upma/system/LocalTime16P.nc"
static inline void /*LocalTime32khz16C.LocalTimeP*/LocalTime16P__0__Counter__overflow(void )
#line 117
{
  unsigned long __nesc_temp51;
  unsigned char *__nesc_temp50;

  /* atomic removed: atomic calls only */
#line 118
  (__nesc_temp50 = /*LocalTime32khz16C.LocalTimeP*/LocalTime16P__0__g.mticks.nxdata, __nesc_hton_uint32(__nesc_temp50, (__nesc_temp51 = __nesc_ntoh_uint32(__nesc_temp50)) + 1), __nesc_temp51);
}

# 67 "/home/csilab/tinyos/tos/interfaces/TaskBasic.nc"
inline static error_t /*TestPureTdmaC.SendingApp.App*/SendingC__0__flush__postTask(void ){
#line 67
  unsigned char __nesc_result;
#line 67

#line 67
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(/*TestPureTdmaC.SendingApp.App*/SendingC__0__flush);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 116 "SendingC.nc"
static inline void /*TestPureTdmaC.SendingApp.App*/SendingC__0__Counter__overflow(void )
#line 116
{

  /*TestPureTdmaC.SendingApp.App*/SendingC__0__flush__postTask();
}

# 177 "/home/csilab/tinyos/tos/lib/timer/TransformAlarmC.nc"
static inline void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Counter__overflow(void )
{
}

# 58 "/home/csilab/tinyos/tos/lib/timer/CounterToLocalTimeC.nc"
static inline void /*CC2420PacketC.CounterToLocalTimeC*/CounterToLocalTimeC__0__Counter__overflow(void )
{
}

# 177 "/home/csilab/tinyos/tos/lib/timer/TransformAlarmC.nc"
static inline void /*GenericSlotterC.Alarm32khz32C.Transform*/TransformAlarmC__2__Counter__overflow(void )
{
}

#line 177
static inline void /*PureTDMASchedulerC.Alarm32khz32C.Transform*/TransformAlarmC__3__Counter__overflow(void )
{
}

# 82 "/home/csilab/tinyos/tos/lib/timer/Counter.nc"
inline static void /*Counter32khz32C.Transform*/TransformCounterC__0__Counter__overflow(void ){
#line 82
  /*PureTDMASchedulerC.Alarm32khz32C.Transform*/TransformAlarmC__3__Counter__overflow();
#line 82
  /*GenericSlotterC.Alarm32khz32C.Transform*/TransformAlarmC__2__Counter__overflow();
#line 82
  /*CC2420PacketC.CounterToLocalTimeC*/CounterToLocalTimeC__0__Counter__overflow();
#line 82
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Counter__overflow();
#line 82
  /*TestPureTdmaC.SendingApp.App*/SendingC__0__Counter__overflow();
#line 82
}
#line 82
# 133 "/home/csilab/tinyos/tos/lib/timer/TransformCounterC.nc"
static inline void /*Counter32khz32C.Transform*/TransformCounterC__0__CounterFrom__overflow(void )
{
  /* atomic removed: atomic calls only */
  {
    /*Counter32khz32C.Transform*/TransformCounterC__0__m_upper++;
    if ((/*Counter32khz32C.Transform*/TransformCounterC__0__m_upper & /*Counter32khz32C.Transform*/TransformCounterC__0__OVERFLOW_MASK) == 0) {
      /*Counter32khz32C.Transform*/TransformCounterC__0__Counter__overflow();
      }
  }
}

# 58 "/home/csilab/tinyos/tos/lib/timer/CounterToLocalTimeC.nc"
static inline void /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__1__Counter__overflow(void )
{
}

# 177 "/home/csilab/tinyos/tos/lib/timer/TransformAlarmC.nc"
static inline void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__Counter__overflow(void )
{
}

# 82 "/home/csilab/tinyos/tos/lib/timer/Counter.nc"
inline static void /*CounterMilli32C.Transform*/TransformCounterC__1__Counter__overflow(void ){
#line 82
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__Counter__overflow();
#line 82
  /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__1__Counter__overflow();
#line 82
}
#line 82
# 133 "/home/csilab/tinyos/tos/lib/timer/TransformCounterC.nc"
static inline void /*CounterMilli32C.Transform*/TransformCounterC__1__CounterFrom__overflow(void )
{
  /* atomic removed: atomic calls only */
  {
    /*CounterMilli32C.Transform*/TransformCounterC__1__m_upper++;
    if ((/*CounterMilli32C.Transform*/TransformCounterC__1__m_upper & /*CounterMilli32C.Transform*/TransformCounterC__1__OVERFLOW_MASK) == 0) {
      /*CounterMilli32C.Transform*/TransformCounterC__1__Counter__overflow();
      }
  }
}

# 82 "/home/csilab/tinyos/tos/lib/timer/Counter.nc"
inline static void /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__overflow(void ){
#line 82
  /*CounterMilli32C.Transform*/TransformCounterC__1__CounterFrom__overflow();
#line 82
  /*Counter32khz32C.Transform*/TransformCounterC__0__CounterFrom__overflow();
#line 82
  /*LocalTime32khz16C.LocalTimeP*/LocalTime16P__0__Counter__overflow();
#line 82
  /*Msp430Uart1P.UartP*/Msp430UartP__0__Counter__overflow();
#line 82
}
#line 82
# 64 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430CounterC.nc"
static inline void /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__overflow(void )
{
  /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__overflow();
}

# 114 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*PureTDMASchedulerC.offset_alarm.Msp430Alarm*/Msp430AlarmC__5__Msp430Timer__overflow(void )
{
}

#line 114
static inline void /*PureTDMASchedulerC.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__4__Msp430Timer__overflow(void )
{
}

#line 114
static inline void /*GenericSlotterC.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__3__Msp430Timer__overflow(void )
{
}

#line 114
static inline void /*TDMASlotSenderC.TDMASlotSender.Alarm32khz16C.Msp430Alarm*/Msp430AlarmC__2__Msp430Timer__overflow(void )
{
}

#line 114
static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430Timer__overflow(void )
{
}

#line 114
static inline void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430Timer__overflow(void )
{
}

# 48 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430Timer.nc"
inline static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__overflow(void ){
#line 48
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430Timer__overflow();
#line 48
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430Timer__overflow();
#line 48
  /*TDMASlotSenderC.TDMASlotSender.Alarm32khz16C.Msp430Alarm*/Msp430AlarmC__2__Msp430Timer__overflow();
#line 48
  /*GenericSlotterC.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__3__Msp430Timer__overflow();
#line 48
  /*PureTDMASchedulerC.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__4__Msp430Timer__overflow();
#line 48
  /*PureTDMASchedulerC.offset_alarm.Msp430Alarm*/Msp430AlarmC__5__Msp430Timer__overflow();
#line 48
  /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__overflow();
#line 48
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Timer__overflow();
#line 48
  /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Timer__overflow();
#line 48
  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Timer__overflow();
#line 48
  /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Timer__overflow();
#line 48
  /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Timer__overflow();
#line 48
  /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Timer__overflow();
#line 48
  /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Timer__overflow();
#line 48
}
#line 48
# 137 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Overflow__fired(void )
{
  /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__overflow();
}

# 88 "/home/csilab/tinyos/tos/interfaces/Resource.nc"
inline static error_t CC2420ControlP__SpiResource__request(void ){
#line 88
  unsigned char __nesc_result;
#line 88

#line 88
  __nesc_result = CC2420SpiP__Resource__request(/*CC2420ControlC.Spi*/CC2420SpiC__0__CLIENT_ID);
#line 88

#line 88
  return __nesc_result;
#line 88
}
#line 88
# 188 "/home/csilab/tinyos/tos/chips/cc2420/control/CC2420ControlP.nc"
static inline error_t CC2420ControlP__Resource__request(void )
#line 188
{
  return CC2420ControlP__SpiResource__request();
}

# 88 "/home/csilab/tinyos/tos/interfaces/Resource.nc"
inline static error_t CC2420CsmaP__Resource__request(void ){
#line 88
  unsigned char __nesc_result;
#line 88

#line 88
  __nesc_result = CC2420ControlP__Resource__request();
#line 88

#line 88
  return __nesc_result;
#line 88
}
#line 88
# 198 "/home/csilab/upma/chips/cc2420/CC2420CsmaP.nc"
static inline void CC2420CsmaP__CC2420Power__startVRegDone(void )
#line 198
{
  CC2420CsmaP__Resource__request();
}

# 56 "/home/csilab/tinyos/tos/chips/cc2420/interfaces/CC2420Power.nc"
inline static void CC2420ControlP__CC2420Power__startVRegDone(void ){
#line 56
  CC2420CsmaP__CC2420Power__startVRegDone();
#line 56
}
#line 56
# 48 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*HplCC2420PinsC.RSTNM*/Msp430GpioC__7__HplGeneralIO__set(void ){
#line 48
  /*HplMsp430GeneralIOC.P46*/HplMsp430GeneralIOP__30__IO__set();
#line 48
}
#line 48
# 48 "/home/csilab/tinyos/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplCC2420PinsC.RSTNM*/Msp430GpioC__7__GeneralIO__set(void )
#line 48
{
#line 48
  /*HplCC2420PinsC.RSTNM*/Msp430GpioC__7__HplGeneralIO__set();
}

# 40 "/home/csilab/tinyos/tos/interfaces/GeneralIO.nc"
inline static void CC2420ControlP__RSTN__set(void ){
#line 40
  /*HplCC2420PinsC.RSTNM*/Msp430GpioC__7__GeneralIO__set();
#line 40
}
#line 40
# 53 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*HplCC2420PinsC.RSTNM*/Msp430GpioC__7__HplGeneralIO__clr(void ){
#line 53
  /*HplMsp430GeneralIOC.P46*/HplMsp430GeneralIOP__30__IO__clr();
#line 53
}
#line 53
# 49 "/home/csilab/tinyos/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplCC2420PinsC.RSTNM*/Msp430GpioC__7__GeneralIO__clr(void )
#line 49
{
#line 49
  /*HplCC2420PinsC.RSTNM*/Msp430GpioC__7__HplGeneralIO__clr();
}

# 41 "/home/csilab/tinyos/tos/interfaces/GeneralIO.nc"
inline static void CC2420ControlP__RSTN__clr(void ){
#line 41
  /*HplCC2420PinsC.RSTNM*/Msp430GpioC__7__GeneralIO__clr();
#line 41
}
#line 41
# 431 "/home/csilab/tinyos/tos/chips/cc2420/control/CC2420ControlP.nc"
static inline void CC2420ControlP__StartupTimer__fired(void )
#line 431
{
  if (CC2420ControlP__m_state == CC2420ControlP__S_VREG_STARTING) {
      CC2420ControlP__m_state = CC2420ControlP__S_VREG_STARTED;
      CC2420ControlP__RSTN__clr();
      CC2420ControlP__RSTN__set();
      CC2420ControlP__CC2420Power__startVRegDone();
    }
}

# 120 "/home/csilab/tinyos/tos/interfaces/Resource.nc"
inline static error_t CC2420TransmitP__SpiResource__release(void ){
#line 120
  unsigned char __nesc_result;
#line 120

#line 120
  __nesc_result = CC2420SpiP__Resource__release(/*CC2420TransmitC.Spi*/CC2420SpiC__3__CLIENT_ID);
#line 120

#line 120
  return __nesc_result;
#line 120
}
#line 120
# 679 "/home/csilab/upma/chips/cc2420/CC2420TransmitP.nc"
static inline error_t CC2420TransmitP__releaseSpiResource(void )
#line 679
{
  CC2420TransmitP__SpiResource__release();
  return SUCCESS;
}

# 61 "/home/csilab/tinyos/tos/chips/msp430/timer/GpioCaptureC.nc"
static inline error_t /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Capture__captureRisingEdge(void )
#line 61
{
  return /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__enableCapture(MSP430TIMER_CM_RISING);
}

# 53 "/home/csilab/tinyos/tos/interfaces/GpioCapture.nc"
inline static error_t CC2420TransmitP__CaptureSFD__captureRisingEdge(void ){
#line 53
  unsigned char __nesc_result;
#line 53

#line 53
  __nesc_result = /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Capture__captureRisingEdge();
#line 53

#line 53
  return __nesc_result;
#line 53
}
#line 53
# 53 "/home/csilab/tinyos/tos/chips/cc2420/interfaces/CC2420Strobe.nc"
inline static cc2420_status_t CC2420TransmitP__SFLUSHTX__strobe(void ){
#line 53
  unsigned char __nesc_result;
#line 53

#line 53
  __nesc_result = CC2420SpiP__Strobe__strobe(CC2420_SFLUSHTX);
#line 53

#line 53
  return __nesc_result;
#line 53
}
#line 53
# 64 "/home/csilab/tinyos/tos/lib/timer/Counter.nc"
inline static /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Counter__size_type /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Counter__get(void ){
#line 64
  unsigned long __nesc_result;
#line 64

#line 64
  __nesc_result = /*Counter32khz32C.Transform*/TransformCounterC__0__Counter__get();
#line 64

#line 64
  return __nesc_result;
#line 64
}
#line 64
# 86 "/home/csilab/tinyos/tos/lib/timer/TransformAlarmC.nc"
static inline /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__to_size_type /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Alarm__getNow(void )
{
  return /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Counter__get();
}

#line 157
static inline void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Alarm__start(/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__to_size_type dt)
{
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Alarm__startAt(/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Alarm__getNow(), dt);
}

# 66 "/home/csilab/tinyos/tos/lib/timer/Alarm.nc"
inline static void CC2420TransmitP__BackoffTimer__start(CC2420TransmitP__BackoffTimer__size_type dt){
#line 66
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Alarm__start(dt);
#line 66
}
#line 66
# 59 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline uint8_t /*HplMsp430GeneralIOC.P14*/HplMsp430GeneralIOP__4__IO__getRaw(void )
#line 59
{
#line 59
  return * (volatile uint8_t * )32U & (0x01 << 4);
}

#line 60
static inline bool /*HplMsp430GeneralIOC.P14*/HplMsp430GeneralIOP__4__IO__get(void )
#line 60
{
#line 60
  return /*HplMsp430GeneralIOC.P14*/HplMsp430GeneralIOP__4__IO__getRaw() != 0;
}

# 73 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static bool /*HplCC2420PinsC.CCAM*/Msp430GpioC__3__HplGeneralIO__get(void ){
#line 73
  unsigned char __nesc_result;
#line 73

#line 73
  __nesc_result = /*HplMsp430GeneralIOC.P14*/HplMsp430GeneralIOP__4__IO__get();
#line 73

#line 73
  return __nesc_result;
#line 73
}
#line 73
# 51 "/home/csilab/tinyos/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline bool /*HplCC2420PinsC.CCAM*/Msp430GpioC__3__GeneralIO__get(void )
#line 51
{
#line 51
  return /*HplCC2420PinsC.CCAM*/Msp430GpioC__3__HplGeneralIO__get();
}

# 43 "/home/csilab/tinyos/tos/interfaces/GeneralIO.nc"
inline static bool CC2420TransmitP__CCA__get(void ){
#line 43
  unsigned char __nesc_result;
#line 43

#line 43
  __nesc_result = /*HplCC2420PinsC.CCAM*/Msp430GpioC__3__GeneralIO__get();
#line 43

#line 43
  return __nesc_result;
#line 43
}
#line 43
# 498 "/home/csilab/upma/chips/cc2420/CC2420TransmitP.nc"
static inline void CC2420TransmitP__BackoffTimer__fired(void )
#line 498
{
  /* atomic removed: atomic calls only */
#line 499
  {
    switch (CC2420TransmitP__m_state) {

        case CC2420TransmitP__S_SAMPLE_CCA: 


          if (CC2420TransmitP__CCA__get()) {
              CC2420TransmitP__m_state = CC2420TransmitP__S_BEGIN_TRANSMIT;
              CC2420TransmitP__BackoffTimer__start(CC2420_TIME_ACK_TURNAROUND);
            }
          else {
              CC2420TransmitP__congestionBackoff();
            }
        break;

        case CC2420TransmitP__S_BEGIN_TRANSMIT: 
          case CC2420TransmitP__S_CANCEL: 
            if (CC2420TransmitP__acquireSpiResource() == SUCCESS) {
                CC2420TransmitP__attemptSend();
              }
        break;

        case CC2420TransmitP__S_ACK_WAIT: 
          CC2420TransmitP__signalDone(SUCCESS);
        break;

        case CC2420TransmitP__S_SFD: 


          CC2420TransmitP__SFLUSHTX__strobe();
        CC2420TransmitP__CaptureSFD__captureRisingEdge();
        CC2420TransmitP__releaseSpiResource();
        CC2420TransmitP__signalDone(ERETRY);
        break;

        default: 
          break;
      }
  }
}

# 78 "/home/csilab/tinyos/tos/lib/timer/Alarm.nc"
inline static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Alarm__fired(void ){
#line 78
  CC2420TransmitP__BackoffTimer__fired();
#line 78
  CC2420ControlP__StartupTimer__fired();
#line 78
}
#line 78
# 162 "/home/csilab/tinyos/tos/lib/timer/TransformAlarmC.nc"
static inline void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__AlarmFrom__fired(void )
{
  /* atomic removed: atomic calls only */
  {
    if (/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__m_dt == 0) 
      {
        /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Alarm__fired();
      }
    else 
      {
        /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__set_alarm();
      }
  }
}

# 78 "/home/csilab/tinyos/tos/lib/timer/Alarm.nc"
inline static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Alarm__fired(void ){
#line 78
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__AlarmFrom__fired();
#line 78
}
#line 78
# 135 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__disableEvents(void )
{
  * (volatile uint16_t * )386U &= ~0x0010;
}

# 58 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__disableEvents(void ){
#line 58
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__disableEvents();
#line 58
}
#line 58
# 70 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__fired(void )
{
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__disableEvents();
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Alarm__fired();
}

# 45 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__fired(void ){
#line 45
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__fired();
#line 45
}
#line 45
# 150 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__getEvent(void )
{
  return * (volatile uint16_t * )402U;
}

#line 188
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__default__captured(uint16_t n)
{
}

# 86 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__captured(uint16_t time){
#line 86
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__default__captured(time);
#line 86
}
#line 86
# 58 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__int2CC(uint16_t x)
#line 58
{
#line 58
  union /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3____nesc_unnamed4404 {
#line 58
    uint16_t f;
#line 58
    /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t t;
  } 
#line 58
  c = { .f = x };

#line 58
  return c.t;
}

#line 85
static inline /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__getControl(void )
{
  return /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__int2CC(* (volatile uint16_t * )386U);
}

#line 180
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Event__fired(void )
{
  if (/*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__getControl().cap) {
    /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__captured(/*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__getEvent());
    }
  else {
#line 185
    /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__fired();
    }
}

# 45 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430Timer.nc"
inline static uint16_t /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__get(void ){
#line 45
  unsigned int __nesc_result;
#line 45

#line 45
  __nesc_result = /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__get();
#line 45

#line 45
  return __nesc_result;
#line 45
}
#line 45
# 49 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430CounterC.nc"
static inline uint16_t /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__get(void )
{
  return /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__get();
}

# 64 "/home/csilab/tinyos/tos/lib/timer/Counter.nc"
inline static /*Counter32khz32C.Transform*/TransformCounterC__0__CounterFrom__size_type /*Counter32khz32C.Transform*/TransformCounterC__0__CounterFrom__get(void ){
#line 64
  unsigned int __nesc_result;
#line 64

#line 64
  __nesc_result = /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__get();
#line 64

#line 64
  return __nesc_result;
#line 64
}
#line 64
# 81 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline bool /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__isOverflowPending(void )
{
  return * (volatile uint16_t * )384U & 1U;
}

# 46 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430Timer.nc"
inline static bool /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__isOverflowPending(void ){
#line 46
  unsigned char __nesc_result;
#line 46

#line 46
  __nesc_result = /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__isOverflowPending();
#line 46

#line 46
  return __nesc_result;
#line 46
}
#line 46
# 54 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430CounterC.nc"
static inline bool /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__isOverflowPending(void )
{
  return /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__isOverflowPending();
}

# 71 "/home/csilab/tinyos/tos/lib/timer/Counter.nc"
inline static bool /*Counter32khz32C.Transform*/TransformCounterC__0__CounterFrom__isOverflowPending(void ){
#line 71
  unsigned char __nesc_result;
#line 71

#line 71
  __nesc_result = /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__isOverflowPending();
#line 71

#line 71
  return __nesc_result;
#line 71
}
#line 71
# 130 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__enableEvents(void )
{
  * (volatile uint16_t * )386U |= 0x0010;
}

# 57 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__enableEvents(void ){
#line 57
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__enableEvents();
#line 57
}
#line 57
# 95 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__clearPendingInterrupt(void )
{
  * (volatile uint16_t * )386U &= ~0x0001;
}

# 44 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__clearPendingInterrupt(void ){
#line 44
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__clearPendingInterrupt();
#line 44
}
#line 44
# 155 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__setEvent(uint16_t x)
{
  * (volatile uint16_t * )402U = x;
}

# 41 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__setEvent(uint16_t time){
#line 41
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__setEvent(time);
#line 41
}
#line 41
# 45 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430Timer.nc"
inline static uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Timer__get(void ){
#line 45
  unsigned int __nesc_result;
#line 45

#line 45
  __nesc_result = /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__get();
#line 45

#line 45
  return __nesc_result;
#line 45
}
#line 45
# 165 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__setEventFromNow(uint16_t x)
{
  * (volatile uint16_t * )402U = /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Timer__get() + x;
}

# 43 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__setEventFromNow(uint16_t delta){
#line 43
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__setEventFromNow(delta);
#line 43
}
#line 43
# 45 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430Timer.nc"
inline static uint16_t /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430Timer__get(void ){
#line 45
  unsigned int __nesc_result;
#line 45

#line 45
  __nesc_result = /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__get();
#line 45

#line 45
  return __nesc_result;
#line 45
}
#line 45
# 81 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Alarm__startAt(uint16_t t0, uint16_t dt)
{
  /* atomic removed: atomic calls only */
  {
    uint16_t now = /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430Timer__get();
    uint16_t elapsed = now - t0;

#line 87
    if (elapsed >= dt) 
      {
        /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__setEventFromNow(2);
      }
    else 
      {
        uint16_t remaining = dt - elapsed;

#line 94
        if (remaining <= 2) {
          /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__setEventFromNow(2);
          }
        else {
#line 97
          /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__setEvent(now + remaining);
          }
      }
#line 99
    /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__clearPendingInterrupt();
    /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__enableEvents();
  }
}

# 103 "/home/csilab/tinyos/tos/lib/timer/Alarm.nc"
inline static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__AlarmFrom__startAt(/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__AlarmFrom__size_type t0, /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__AlarmFrom__size_type dt){
#line 103
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Alarm__startAt(t0, dt);
#line 103
}
#line 103
# 89 "/home/csilab/tinyos/tos/system/RandomMlcgC.nc"
static inline uint16_t RandomMlcgC__Random__rand16(void )
#line 89
{
  return (uint16_t )RandomMlcgC__Random__rand32();
}

# 52 "/home/csilab/tinyos/tos/interfaces/Random.nc"
inline static uint16_t CC2420TransmitP__Random__rand16(void ){
#line 52
  unsigned int __nesc_result;
#line 52

#line 52
  __nesc_result = RandomMlcgC__Random__rand16();
#line 52

#line 52
  return __nesc_result;
#line 52
}
#line 52
# 764 "/home/csilab/upma/chips/cc2420/CC2420TransmitP.nc"
static inline uint16_t CC2420TransmitP__defaultCongestionBackoff(void )
#line 764
{
  return CC2420TransmitP__Random__rand16() % (0x7 * CC2420_BACKOFF_PERIOD) + CC2420_MIN_BACKOFF;
}

# 292 "/usr/local/lib/ncc/nesc_nx.h"
static __inline  uint8_t __nesc_ntoh_leuint8(const void * source)
#line 292
{
  const uint8_t *base = source;

#line 294
  return base[0];
}

# 211 "/home/csilab/upma/chips/cc2420/CC2420TransmitP.nc"
static inline am_id_t CC2420TransmitP__getType(message_t * p_msg)
#line 211
{
  return __nesc_ntoh_leuint8(((cc2420_header_t * )((uint8_t *)p_msg + (unsigned short )& ((message_t *)0)->data - sizeof(cc2420_header_t )))->type.nxdata);
}

# 287 "/home/csilab/upma/chips/cc2420/CC2420ActiveMessageP.nc"
static inline uint16_t CC2420ActiveMessageP__CcaControl__default__getCongestionBackoff(am_id_t amId, message_t *msg, uint16_t defaultBackoff)
#line 287
{
  return defaultBackoff;
}

# 50 "/home/csilab/upma/interfaces/CcaControl.nc"
inline static uint16_t CC2420ActiveMessageP__CcaControl__getCongestionBackoff(am_id_t arg_0x7f7fec07e320, message_t *msg, uint16_t defaultBackoff){
#line 50
  unsigned int __nesc_result;
#line 50

#line 50
    __nesc_result = CC2420ActiveMessageP__CcaControl__default__getCongestionBackoff(arg_0x7f7fec07e320, msg, defaultBackoff);
#line 50

#line 50
  return __nesc_result;
#line 50
}
#line 50
# 258 "/home/csilab/upma/chips/cc2420/CC2420ActiveMessageP.nc"
static inline uint16_t CC2420ActiveMessageP__SubCcaControl__getCongestionBackoff(am_id_t amId, message_t *msg, uint16_t defaultBackoff)
#line 258
{
  return CC2420ActiveMessageP__CcaControl__getCongestionBackoff(__nesc_ntoh_leuint8(((cc2420_header_t * )((uint8_t *)msg + (unsigned short )& ((message_t *)0)->data - sizeof(cc2420_header_t )))->type.nxdata), msg, defaultBackoff);
}

# 147 "/home/csilab/upma/system/BeaconSlotP.nc"
static inline uint16_t BeaconSlotP__SubCcaControl__getCongestionBackoff(message_t *msg, uint16_t defaultBackoff)
#line 147
{
  return 0;
}

# 158 "/home/csilab/upma/system/GenericSlotSenderP.nc"
static inline uint16_t /*TDMASlotSenderC.TDMASlotSender.GenericSlotSenderP*/GenericSlotSenderP__0__SubCcaControl__getCongestionBackoff(message_t *msg, uint16_t defaultBackoff)
#line 158
{
  return 0;
}

# 128 "/home/csilab/upma/system/SenderDispatcherP.nc"
static inline uint16_t SenderDispatcherP__SlotsCcaControl__default__getCongestionBackoff(am_id_t id, message_t *msg, uint16_t defaultBackoff)
#line 128
{
  return defaultBackoff;
}

# 50 "/home/csilab/upma/interfaces/CcaControl.nc"
inline static uint16_t SenderDispatcherP__SlotsCcaControl__getCongestionBackoff(uint8_t arg_0x7f7fec1925b0, message_t *msg, uint16_t defaultBackoff){
#line 50
  unsigned int __nesc_result;
#line 50

#line 50
  switch (arg_0x7f7fec1925b0) {
#line 50
    case TDMA_SLOT:
#line 50
      __nesc_result = /*TDMASlotSenderC.TDMASlotSender.GenericSlotSenderP*/GenericSlotSenderP__0__SubCcaControl__getCongestionBackoff(msg, defaultBackoff);
#line 50
      break;
#line 50
    case BEACON_SLOT:
#line 50
      __nesc_result = BeaconSlotP__SubCcaControl__getCongestionBackoff(msg, defaultBackoff);
#line 50
      break;
#line 50
    default:
#line 50
      __nesc_result = SenderDispatcherP__SlotsCcaControl__default__getCongestionBackoff(arg_0x7f7fec1925b0, msg, defaultBackoff);
#line 50
      break;
#line 50
    }
#line 50

#line 50
  return __nesc_result;
#line 50
}
#line 50
# 142 "/home/csilab/upma/system/SenderDispatcherP.nc"
static inline uint16_t SenderDispatcherP__SubCcaControl__getCongestionBackoff(am_id_t id, message_t *msg, uint16_t defaultBackoff)
#line 142
{
  return SenderDispatcherP__SlotsCcaControl__getCongestionBackoff(SenderDispatcherP__last_id, msg, defaultBackoff);
}

# 50 "/home/csilab/upma/interfaces/CcaControl.nc"
inline static uint16_t CC2420TransmitP__CcaControl__getCongestionBackoff(am_id_t arg_0x7f7fec8771d0, message_t *msg, uint16_t defaultBackoff){
#line 50
  unsigned int __nesc_result;
#line 50

#line 50
  __nesc_result = SenderDispatcherP__SubCcaControl__getCongestionBackoff(arg_0x7f7fec8771d0, msg, defaultBackoff);
#line 50
  __nesc_result = CC2420ActiveMessageP__SubCcaControl__getCongestionBackoff(arg_0x7f7fec8771d0, msg, defaultBackoff);
#line 50

#line 50
  return __nesc_result;
#line 50
}
#line 50
# 657 "/home/csilab/upma/chips/cc2420/CC2420TransmitP.nc"
static inline void CC2420TransmitP__requestCongestionBackoff(message_t *msg)
#line 657
{
  CC2420TransmitP__myCongestionBackoff = CC2420TransmitP__CcaControl__getCongestionBackoff(CC2420TransmitP__getType(msg), msg, CC2420TransmitP__defaultCongestionBackoff()) + 1;
}

# 97 "/home/csilab/tinyos/tos/interfaces/Resource.nc"
inline static error_t CC2420TransmitP__SpiResource__immediateRequest(void ){
#line 97
  unsigned char __nesc_result;
#line 97

#line 97
  __nesc_result = CC2420SpiP__Resource__immediateRequest(/*CC2420TransmitC.Spi*/CC2420SpiC__3__CLIENT_ID);
#line 97

#line 97
  return __nesc_result;
#line 97
}
#line 97
# 45 "/home/csilab/tinyos/tos/interfaces/State.nc"
inline static error_t CC2420SpiP__WorkingState__requestState(uint8_t reqState){
#line 45
  unsigned char __nesc_result;
#line 45

#line 45
  __nesc_result = StateImplP__State__requestState(0U, reqState);
#line 45

#line 45
  return __nesc_result;
#line 45
}
#line 45
# 172 "/home/csilab/tinyos/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static inline bool /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__default__isOwner(uint8_t id)
#line 172
{
#line 172
  return FALSE;
}

# 128 "/home/csilab/tinyos/tos/interfaces/Resource.nc"
inline static bool /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__isOwner(uint8_t arg_0x7f7fecbdbe30){
#line 128
  unsigned char __nesc_result;
#line 128

#line 128
  switch (arg_0x7f7fecbdbe30) {
#line 128
    case /*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430Spi0C__0__CLIENT_ID:
#line 128
      __nesc_result = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__Resource__isOwner(/*CC2420SpiWireC.HplCC2420SpiC.SpiC.UsartC*/Msp430Usart0C__0__CLIENT_ID);
#line 128
      break;
#line 128
    default:
#line 128
      __nesc_result = /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__default__isOwner(arg_0x7f7fecbdbe30);
#line 128
      break;
#line 128
    }
#line 128

#line 128
  return __nesc_result;
#line 128
}
#line 128
# 112 "/home/csilab/tinyos/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static inline bool /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Resource__isOwner(uint8_t id)
#line 112
{
  return /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__isOwner(id);
}

# 128 "/home/csilab/tinyos/tos/interfaces/Resource.nc"
inline static bool CC2420SpiP__SpiResource__isOwner(void ){
#line 128
  unsigned char __nesc_result;
#line 128

#line 128
  __nesc_result = /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Resource__isOwner(/*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430Spi0C__0__CLIENT_ID);
#line 128

#line 128
  return __nesc_result;
#line 128
}
#line 128
# 176 "/home/csilab/tinyos/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static inline msp430_spi_union_config_t */*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Msp430SpiConfigure__default__getConfig(uint8_t id)
#line 176
{
  return &msp430_spi_default_config;
}

# 39 "/home/csilab/tinyos/tos/chips/msp430/usart/Msp430SpiConfigure.nc"
inline static msp430_spi_union_config_t */*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Msp430SpiConfigure__getConfig(uint8_t arg_0x7f7fecbd9110){
#line 39
  union __nesc_unnamed4267 *__nesc_result;
#line 39

#line 39
    __nesc_result = /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Msp430SpiConfigure__default__getConfig(arg_0x7f7fecbd9110);
#line 39

#line 39
  return __nesc_result;
#line 39
}
#line 39
# 168 "/home/csilab/tinyos/tos/chips/msp430/usart/HplMsp430Usart.nc"
inline static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__setModeSpi(msp430_spi_union_config_t *config){
#line 168
  HplMsp430Usart0P__Usart__setModeSpi(config);
#line 168
}
#line 168
# 120 "/home/csilab/tinyos/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__ResourceConfigure__configure(uint8_t id)
#line 120
{
  /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__setModeSpi(/*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Msp430SpiConfigure__getConfig(id));
}

# 216 "/home/csilab/tinyos/tos/system/ArbiterP.nc"
static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceConfigure__default__configure(uint8_t id)
#line 216
{
}

# 59 "/home/csilab/tinyos/tos/interfaces/ResourceConfigure.nc"
inline static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceConfigure__configure(uint8_t arg_0x7f7fed25bc40){
#line 59
  switch (arg_0x7f7fed25bc40) {
#line 59
    case /*CC2420SpiWireC.HplCC2420SpiC.SpiC.UsartC*/Msp430Usart0C__0__CLIENT_ID:
#line 59
      /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__ResourceConfigure__configure(/*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430Spi0C__0__CLIENT_ID);
#line 59
      break;
#line 59
    default:
#line 59
      /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceConfigure__default__configure(arg_0x7f7fed25bc40);
#line 59
      break;
#line 59
    }
#line 59
}
#line 59
# 213 "/home/csilab/tinyos/tos/system/ArbiterP.nc"
static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceDefaultOwner__default__immediateRequested(void )
#line 213
{
  /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceDefaultOwner__release();
}

# 81 "/home/csilab/tinyos/tos/interfaces/ResourceDefaultOwner.nc"
inline static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceDefaultOwner__immediateRequested(void ){
#line 81
  /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceDefaultOwner__default__immediateRequested();
#line 81
}
#line 81
# 206 "/home/csilab/tinyos/tos/system/ArbiterP.nc"
static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceRequested__default__immediateRequested(uint8_t id)
#line 206
{
}

# 61 "/home/csilab/tinyos/tos/interfaces/ResourceRequested.nc"
inline static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceRequested__immediateRequested(uint8_t arg_0x7f7fed25e840){
#line 61
    /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceRequested__default__immediateRequested(arg_0x7f7fed25e840);
#line 61
}
#line 61
# 93 "/home/csilab/tinyos/tos/system/ArbiterP.nc"
static inline error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__Resource__immediateRequest(uint8_t id)
#line 93
{
  /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceRequested__immediateRequested(/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__resId);
  /* atomic removed: atomic calls only */
#line 95
  {
    if (/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__state == /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__RES_CONTROLLED) {
        /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__state = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__RES_IMM_GRANTING;
        /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__reqResId = id;
      }
    else {
        unsigned char __nesc_temp = 
#line 100
        FAIL;

#line 100
        return __nesc_temp;
      }
  }
#line 102
  /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceDefaultOwner__immediateRequested();
  if (/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__resId == id) {
      /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceConfigure__configure(/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__resId);
      return SUCCESS;
    }
  /* atomic removed: atomic calls only */
#line 107
  /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__state = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__RES_CONTROLLED;
  return FAIL;
}

# 174 "/home/csilab/tinyos/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static inline error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__default__immediateRequest(uint8_t id)
#line 174
{
#line 174
  return FAIL;
}

# 97 "/home/csilab/tinyos/tos/interfaces/Resource.nc"
inline static error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__immediateRequest(uint8_t arg_0x7f7fecbdbe30){
#line 97
  unsigned char __nesc_result;
#line 97

#line 97
  switch (arg_0x7f7fecbdbe30) {
#line 97
    case /*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430Spi0C__0__CLIENT_ID:
#line 97
      __nesc_result = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__Resource__immediateRequest(/*CC2420SpiWireC.HplCC2420SpiC.SpiC.UsartC*/Msp430Usart0C__0__CLIENT_ID);
#line 97
      break;
#line 97
    default:
#line 97
      __nesc_result = /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__default__immediateRequest(arg_0x7f7fecbdbe30);
#line 97
      break;
#line 97
    }
#line 97

#line 97
  return __nesc_result;
#line 97
}
#line 97
# 104 "/home/csilab/tinyos/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static inline error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Resource__immediateRequest(uint8_t id)
#line 104
{
  return /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__immediateRequest(id);
}

# 97 "/home/csilab/tinyos/tos/interfaces/Resource.nc"
inline static error_t CC2420SpiP__SpiResource__immediateRequest(void ){
#line 97
  unsigned char __nesc_result;
#line 97

#line 97
  __nesc_result = /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Resource__immediateRequest(/*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430Spi0C__0__CLIENT_ID);
#line 97

#line 97
  return __nesc_result;
#line 97
}
#line 97
# 151 "/home/csilab/tinyos/tos/chips/msp430/usart/HplMsp430Usart0P.nc"
static inline void HplMsp430Usart0P__Usart__resetUsart(bool reset)
#line 151
{
  if (reset) {
      U0CTL = 0x01;
    }
  else {
      U0CTL &= ~0x01;
    }
}

# 97 "/home/csilab/tinyos/tos/chips/msp430/usart/HplMsp430Usart.nc"
inline static void HplMsp430I2C0P__HplUsart__resetUsart(bool reset){
#line 97
  HplMsp430Usart0P__Usart__resetUsart(reset);
#line 97
}
#line 97
# 59 "/home/csilab/tinyos/tos/chips/msp430/usart/HplMsp430I2C0P.nc"
static inline void HplMsp430I2C0P__HplI2C__clearModeI2C(void )
#line 59
{
  /* atomic removed: atomic calls only */
#line 60
  {
    HplMsp430I2C0P__U0CTL &= ~((0x20 | 0x04) | 0x01);
    HplMsp430I2C0P__HplUsart__resetUsart(TRUE);
  }
}

# 7 "/home/csilab/tinyos/tos/chips/msp430/usart/HplMsp430I2C.nc"
inline static void HplMsp430Usart0P__HplI2C__clearModeI2C(void ){
#line 7
  HplMsp430I2C0P__HplI2C__clearModeI2C();
#line 7
}
#line 7
# 67 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P35*/HplMsp430GeneralIOP__21__IO__selectIOFunc(void )
#line 67
{
  /* atomic removed: atomic calls only */
#line 67
  * (volatile uint8_t * )27U &= ~(0x01 << 5);
}

# 99 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430Usart0P__URXD__selectIOFunc(void ){
#line 99
  /*HplMsp430GeneralIOC.P35*/HplMsp430GeneralIOP__21__IO__selectIOFunc();
#line 99
}
#line 99
# 67 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P34*/HplMsp430GeneralIOP__20__IO__selectIOFunc(void )
#line 67
{
  /* atomic removed: atomic calls only */
#line 67
  * (volatile uint8_t * )27U &= ~(0x01 << 4);
}

# 99 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430Usart0P__UTXD__selectIOFunc(void ){
#line 99
  /*HplMsp430GeneralIOC.P34*/HplMsp430GeneralIOP__20__IO__selectIOFunc();
#line 99
}
#line 99
# 207 "/home/csilab/tinyos/tos/chips/msp430/usart/HplMsp430Usart0P.nc"
static inline void HplMsp430Usart0P__Usart__disableUart(void )
#line 207
{
  /* atomic removed: atomic calls only */
#line 208
  {
    HplMsp430Usart0P__ME1 &= ~(0x80 | 0x40);
    HplMsp430Usart0P__UTXD__selectIOFunc();
    HplMsp430Usart0P__URXD__selectIOFunc();
  }
}

#line 143
static inline void HplMsp430Usart0P__Usart__setUmctl(uint8_t control)
#line 143
{
  U0MCTL = control;
}

#line 132
static inline void HplMsp430Usart0P__Usart__setUbr(uint16_t control)
#line 132
{
  /* atomic removed: atomic calls only */
#line 133
  {
    U0BR0 = control & 0x00FF;
    U0BR1 = (control >> 8) & 0x00FF;
  }
}

#line 256
static inline void HplMsp430Usart0P__configSpi(msp430_spi_union_config_t *config)
#line 256
{

  U0CTL = (config->spiRegisters.uctl | 0x04) | 0x01;
  HplMsp430Usart0P__U0TCTL = config->spiRegisters.utctl;

  HplMsp430Usart0P__Usart__setUbr(config->spiRegisters.ubr);
  HplMsp430Usart0P__Usart__setUmctl(0x00);
}

# 65 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P33*/HplMsp430GeneralIOP__19__IO__selectModuleFunc(void )
#line 65
{
  /* atomic removed: atomic calls only */
#line 65
  * (volatile uint8_t * )27U |= 0x01 << 3;
}

# 92 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430Usart0P__UCLK__selectModuleFunc(void ){
#line 92
  /*HplMsp430GeneralIOC.P33*/HplMsp430GeneralIOP__19__IO__selectModuleFunc();
#line 92
}
#line 92
# 65 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P32*/HplMsp430GeneralIOP__18__IO__selectModuleFunc(void )
#line 65
{
  /* atomic removed: atomic calls only */
#line 65
  * (volatile uint8_t * )27U |= 0x01 << 2;
}

# 92 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430Usart0P__SOMI__selectModuleFunc(void ){
#line 92
  /*HplMsp430GeneralIOC.P32*/HplMsp430GeneralIOP__18__IO__selectModuleFunc();
#line 92
}
#line 92
# 65 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P31*/HplMsp430GeneralIOP__17__IO__selectModuleFunc(void )
#line 65
{
  /* atomic removed: atomic calls only */
#line 65
  * (volatile uint8_t * )27U |= 0x01 << 1;
}

# 92 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430Usart0P__SIMO__selectModuleFunc(void ){
#line 92
  /*HplMsp430GeneralIOC.P31*/HplMsp430GeneralIOP__17__IO__selectModuleFunc();
#line 92
}
#line 92
# 238 "/home/csilab/tinyos/tos/chips/msp430/usart/HplMsp430Usart0P.nc"
static inline void HplMsp430Usart0P__Usart__enableSpi(void )
#line 238
{
  /* atomic removed: atomic calls only */
#line 239
  {
    HplMsp430Usart0P__SIMO__selectModuleFunc();
    HplMsp430Usart0P__SOMI__selectModuleFunc();
    HplMsp430Usart0P__UCLK__selectModuleFunc();
  }
  HplMsp430Usart0P__ME1 |= 0x40;
}

#line 345
static inline void HplMsp430Usart0P__Usart__clrIntr(void )
#line 345
{
  HplMsp430Usart0P__IFG1 &= ~(0x80 | 0x40);
}









static inline void HplMsp430Usart0P__Usart__disableIntr(void )
#line 357
{
  HplMsp430Usart0P__IE1 &= ~(0x80 | 0x40);
}

# 118 "/home/csilab/tinyos/tos/system/StateImplP.nc"
static inline void StateImplP__State__toIdle(uint8_t id)
#line 118
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 119
    StateImplP__state[id] = StateImplP__S_IDLE;
#line 119
    __nesc_atomic_end(__nesc_atomic); }
}

# 56 "/home/csilab/tinyos/tos/interfaces/State.nc"
inline static void CC2420SpiP__WorkingState__toIdle(void ){
#line 56
  StateImplP__State__toIdle(0U);
#line 56
}
#line 56
# 88 "/home/csilab/tinyos/tos/interfaces/Resource.nc"
inline static error_t CC2420TransmitP__SpiResource__request(void ){
#line 88
  unsigned char __nesc_result;
#line 88

#line 88
  __nesc_result = CC2420SpiP__Resource__request(/*CC2420TransmitC.Spi*/CC2420SpiC__3__CLIENT_ID);
#line 88

#line 88
  return __nesc_result;
#line 88
}
#line 88
# 210 "/home/csilab/tinyos/tos/system/ArbiterP.nc"
static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceDefaultOwner__default__requested(void )
#line 210
{
  /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceDefaultOwner__release();
}

# 73 "/home/csilab/tinyos/tos/interfaces/ResourceDefaultOwner.nc"
inline static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceDefaultOwner__requested(void ){
#line 73
  /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceDefaultOwner__default__requested();
#line 73
}
#line 73
# 64 "/home/csilab/tinyos/tos/system/FcfsResourceQueueC.nc"
static inline bool /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__2__FcfsQueue__isEnqueued(resource_client_id_t id)
#line 64
{
  /* atomic removed: atomic calls only */
#line 65
  {
    unsigned char __nesc_temp = 
#line 65
    /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__2__resQ[id] != /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__2__NO_ENTRY || /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__2__qTail == id;

#line 65
    return __nesc_temp;
  }
}

#line 82
static inline error_t /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__2__FcfsQueue__enqueue(resource_client_id_t id)
#line 82
{
  /* atomic removed: atomic calls only */
#line 83
  {
    if (!/*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__2__FcfsQueue__isEnqueued(id)) {
        if (/*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__2__qHead == /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__2__NO_ENTRY) {
          /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__2__qHead = id;
          }
        else {
#line 88
          /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__2__resQ[/*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__2__qTail] = id;
          }
#line 89
        /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__2__qTail = id;
        {
          unsigned char __nesc_temp = 
#line 90
          SUCCESS;

#line 90
          return __nesc_temp;
        }
      }
#line 92
    {
      unsigned char __nesc_temp = 
#line 92
      EBUSY;

#line 92
      return __nesc_temp;
    }
  }
}

# 79 "/home/csilab/tinyos/tos/interfaces/ResourceQueue.nc"
inline static error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__Queue__enqueue(resource_client_id_t id){
#line 79
  unsigned char __nesc_result;
#line 79

#line 79
  __nesc_result = /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__2__FcfsQueue__enqueue(id);
#line 79

#line 79
  return __nesc_result;
#line 79
}
#line 79
# 204 "/home/csilab/tinyos/tos/system/ArbiterP.nc"
static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceRequested__default__requested(uint8_t id)
#line 204
{
}

# 53 "/home/csilab/tinyos/tos/interfaces/ResourceRequested.nc"
inline static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceRequested__requested(uint8_t arg_0x7f7fed25e840){
#line 53
    /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceRequested__default__requested(arg_0x7f7fed25e840);
#line 53
}
#line 53
# 77 "/home/csilab/tinyos/tos/system/ArbiterP.nc"
static inline error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__Resource__request(uint8_t id)
#line 77
{
  /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceRequested__requested(/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__resId);
  /* atomic removed: atomic calls only */
#line 79
  {
    if (/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__state == /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__RES_CONTROLLED) {
        /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__state = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__RES_GRANTING;
        /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__reqResId = id;
      }
    else {
#line 84
      if (/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__reqResId == id) {
          {
            unsigned char __nesc_temp = 
#line 85
            SUCCESS;

#line 85
            return __nesc_temp;
          }
        }
      else 
#line 87
        {
          unsigned char __nesc_temp = 
#line 87
          /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__Queue__enqueue(id);

#line 87
          return __nesc_temp;
        }
      }
  }
#line 89
  /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceDefaultOwner__requested();
  return SUCCESS;
}

# 173 "/home/csilab/tinyos/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static inline error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__default__request(uint8_t id)
#line 173
{
#line 173
  return FAIL;
}

# 88 "/home/csilab/tinyos/tos/interfaces/Resource.nc"
inline static error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__request(uint8_t arg_0x7f7fecbdbe30){
#line 88
  unsigned char __nesc_result;
#line 88

#line 88
  switch (arg_0x7f7fecbdbe30) {
#line 88
    case /*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430Spi0C__0__CLIENT_ID:
#line 88
      __nesc_result = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__Resource__request(/*CC2420SpiWireC.HplCC2420SpiC.SpiC.UsartC*/Msp430Usart0C__0__CLIENT_ID);
#line 88
      break;
#line 88
    default:
#line 88
      __nesc_result = /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__default__request(arg_0x7f7fecbdbe30);
#line 88
      break;
#line 88
    }
#line 88

#line 88
  return __nesc_result;
#line 88
}
#line 88
# 108 "/home/csilab/tinyos/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static inline error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Resource__request(uint8_t id)
#line 108
{
  return /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__request(id);
}

# 88 "/home/csilab/tinyos/tos/interfaces/Resource.nc"
inline static error_t CC2420SpiP__SpiResource__request(void ){
#line 88
  unsigned char __nesc_result;
#line 88

#line 88
  __nesc_result = /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Resource__request(/*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430Spi0C__0__CLIENT_ID);
#line 88

#line 88
  return __nesc_result;
#line 88
}
#line 88
# 382 "/home/csilab/tinyos/tos/chips/msp430/usart/HplMsp430Usart0P.nc"
static inline void HplMsp430Usart0P__Usart__tx(uint8_t data)
#line 382
{
  HplMsp430Usart0P__U0TXBUF = data;
}

# 224 "/home/csilab/tinyos/tos/chips/msp430/usart/HplMsp430Usart.nc"
inline static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__tx(uint8_t data){
#line 224
  HplMsp430Usart0P__Usart__tx(data);
#line 224
}
#line 224
# 330 "/home/csilab/tinyos/tos/chips/msp430/usart/HplMsp430Usart0P.nc"
static inline bool HplMsp430Usart0P__Usart__isRxIntrPending(void )
#line 330
{
  if (HplMsp430Usart0P__IFG1 & 0x40) {
      return TRUE;
    }
  return FALSE;
}

# 192 "/home/csilab/tinyos/tos/chips/msp430/usart/HplMsp430Usart.nc"
inline static bool /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__isRxIntrPending(void ){
#line 192
  unsigned char __nesc_result;
#line 192

#line 192
  __nesc_result = HplMsp430Usart0P__Usart__isRxIntrPending();
#line 192

#line 192
  return __nesc_result;
#line 192
}
#line 192
# 341 "/home/csilab/tinyos/tos/chips/msp430/usart/HplMsp430Usart0P.nc"
static inline void HplMsp430Usart0P__Usart__clrRxIntr(void )
#line 341
{
  HplMsp430Usart0P__IFG1 &= ~0x40;
}

# 197 "/home/csilab/tinyos/tos/chips/msp430/usart/HplMsp430Usart.nc"
inline static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__clrRxIntr(void ){
#line 197
  HplMsp430Usart0P__Usart__clrRxIntr();
#line 197
}
#line 197
# 386 "/home/csilab/tinyos/tos/chips/msp430/usart/HplMsp430Usart0P.nc"
static inline uint8_t HplMsp430Usart0P__Usart__rx(void )
#line 386
{
  return U0RXBUF;
}

# 231 "/home/csilab/tinyos/tos/chips/msp430/usart/HplMsp430Usart.nc"
inline static uint8_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__rx(void ){
#line 231
  unsigned char __nesc_result;
#line 231

#line 231
  __nesc_result = HplMsp430Usart0P__Usart__rx();
#line 231

#line 231
  return __nesc_result;
#line 231
}
#line 231
# 95 "/home/csilab/tinyos/tos/chips/cc2420/spi/CC2420SpiP.nc"
static inline void CC2420SpiP__ChipSpiResource__abortRelease(void )
#line 95
{
  /* atomic removed: atomic calls only */
#line 96
  CC2420SpiP__release = FALSE;
}

# 31 "/home/csilab/tinyos/tos/chips/cc2420/interfaces/ChipSpiResource.nc"
inline static void CC2420TransmitP__ChipSpiResource__abortRelease(void ){
#line 31
  CC2420SpiP__ChipSpiResource__abortRelease();
#line 31
}
#line 31
# 377 "/home/csilab/upma/chips/cc2420/CC2420TransmitP.nc"
static inline void CC2420TransmitP__ChipSpiResource__releasing(void )
#line 377
{
  if (CC2420TransmitP__abortSpiRelease) {
      CC2420TransmitP__ChipSpiResource__abortRelease();
    }
}

# 24 "/home/csilab/tinyos/tos/chips/cc2420/interfaces/ChipSpiResource.nc"
inline static void CC2420SpiP__ChipSpiResource__releasing(void ){
#line 24
  CC2420TransmitP__ChipSpiResource__releasing();
#line 24
}
#line 24
# 208 "/home/csilab/tinyos/tos/system/ArbiterP.nc"
static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceDefaultOwner__default__granted(void )
#line 208
{
}

# 46 "/home/csilab/tinyos/tos/interfaces/ResourceDefaultOwner.nc"
inline static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceDefaultOwner__granted(void ){
#line 46
  /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceDefaultOwner__default__granted();
#line 46
}
#line 46
# 97 "/home/csilab/tinyos/tos/chips/msp430/usart/HplMsp430Usart.nc"
inline static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__resetUsart(bool reset){
#line 97
  HplMsp430Usart0P__Usart__resetUsart(reset);
#line 97
}
#line 97
#line 158
inline static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__disableSpi(void ){
#line 158
  HplMsp430Usart0P__Usart__disableSpi();
#line 158
}
#line 158
# 124 "/home/csilab/tinyos/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__ResourceConfigure__unconfigure(uint8_t id)
#line 124
{
  /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__resetUsart(TRUE);
  /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__disableSpi();
  /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__resetUsart(FALSE);
}

# 218 "/home/csilab/tinyos/tos/system/ArbiterP.nc"
static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceConfigure__default__unconfigure(uint8_t id)
#line 218
{
}

# 65 "/home/csilab/tinyos/tos/interfaces/ResourceConfigure.nc"
inline static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceConfigure__unconfigure(uint8_t arg_0x7f7fed25bc40){
#line 65
  switch (arg_0x7f7fed25bc40) {
#line 65
    case /*CC2420SpiWireC.HplCC2420SpiC.SpiC.UsartC*/Msp430Usart0C__0__CLIENT_ID:
#line 65
      /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__ResourceConfigure__unconfigure(/*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430Spi0C__0__CLIENT_ID);
#line 65
      break;
#line 65
    default:
#line 65
      /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceConfigure__default__unconfigure(arg_0x7f7fed25bc40);
#line 65
      break;
#line 65
    }
#line 65
}
#line 65
# 67 "/home/csilab/tinyos/tos/interfaces/TaskBasic.nc"
inline static error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__grantedTask__postTask(void ){
#line 67
  unsigned char __nesc_result;
#line 67

#line 67
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__grantedTask);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 68 "/home/csilab/tinyos/tos/system/FcfsResourceQueueC.nc"
static inline resource_client_id_t /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__2__FcfsQueue__dequeue(void )
#line 68
{
  /* atomic removed: atomic calls only */
#line 69
  {
    if (/*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__2__qHead != /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__2__NO_ENTRY) {
        uint8_t id = /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__2__qHead;

#line 72
        /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__2__qHead = /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__2__resQ[/*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__2__qHead];
        if (/*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__2__qHead == /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__2__NO_ENTRY) {
          /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__2__qTail = /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__2__NO_ENTRY;
          }
#line 75
        /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__2__resQ[id] = /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__2__NO_ENTRY;
        {
          unsigned char __nesc_temp = 
#line 76
          id;

#line 76
          return __nesc_temp;
        }
      }
#line 78
    {
      unsigned char __nesc_temp = 
#line 78
      /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__2__NO_ENTRY;

#line 78
      return __nesc_temp;
    }
  }
}

# 70 "/home/csilab/tinyos/tos/interfaces/ResourceQueue.nc"
inline static resource_client_id_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__Queue__dequeue(void ){
#line 70
  unsigned char __nesc_result;
#line 70

#line 70
  __nesc_result = /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__2__FcfsQueue__dequeue();
#line 70

#line 70
  return __nesc_result;
#line 70
}
#line 70
# 60 "/home/csilab/tinyos/tos/system/FcfsResourceQueueC.nc"
static inline bool /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__2__FcfsQueue__isEmpty(void )
#line 60
{
  /* atomic removed: atomic calls only */
#line 61
  {
    unsigned char __nesc_temp = 
#line 61
    /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__2__qHead == /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__2__NO_ENTRY;

#line 61
    return __nesc_temp;
  }
}

# 53 "/home/csilab/tinyos/tos/interfaces/ResourceQueue.nc"
inline static bool /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__Queue__isEmpty(void ){
#line 53
  unsigned char __nesc_result;
#line 53

#line 53
  __nesc_result = /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__2__FcfsQueue__isEmpty();
#line 53

#line 53
  return __nesc_result;
#line 53
}
#line 53
# 111 "/home/csilab/tinyos/tos/system/ArbiterP.nc"
static inline error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__Resource__release(uint8_t id)
#line 111
{
  /* atomic removed: atomic calls only */
#line 112
  {
    if (/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__state == /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__RES_BUSY && /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__resId == id) {
        if (/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__Queue__isEmpty() == FALSE) {
            /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__reqResId = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__Queue__dequeue();
            /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__resId = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__NO_RES;
            /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__state = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__RES_GRANTING;
            /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__grantedTask__postTask();
            /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceConfigure__unconfigure(id);
          }
        else {
            /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__resId = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__default_owner_id;
            /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__state = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__RES_CONTROLLED;
            /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceConfigure__unconfigure(id);
            /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceDefaultOwner__granted();
          }
        {
          unsigned char __nesc_temp = 
#line 127
          SUCCESS;

#line 127
          return __nesc_temp;
        }
      }
  }
#line 130
  return FAIL;
}

# 175 "/home/csilab/tinyos/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static inline error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__default__release(uint8_t id)
#line 175
{
#line 175
  return FAIL;
}

# 120 "/home/csilab/tinyos/tos/interfaces/Resource.nc"
inline static error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__release(uint8_t arg_0x7f7fecbdbe30){
#line 120
  unsigned char __nesc_result;
#line 120

#line 120
  switch (arg_0x7f7fecbdbe30) {
#line 120
    case /*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430Spi0C__0__CLIENT_ID:
#line 120
      __nesc_result = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__Resource__release(/*CC2420SpiWireC.HplCC2420SpiC.SpiC.UsartC*/Msp430Usart0C__0__CLIENT_ID);
#line 120
      break;
#line 120
    default:
#line 120
      __nesc_result = /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__default__release(arg_0x7f7fecbdbe30);
#line 120
      break;
#line 120
    }
#line 120

#line 120
  return __nesc_result;
#line 120
}
#line 120
# 116 "/home/csilab/tinyos/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static inline error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Resource__release(uint8_t id)
#line 116
{
  return /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__release(id);
}

# 120 "/home/csilab/tinyos/tos/interfaces/Resource.nc"
inline static error_t CC2420SpiP__SpiResource__release(void ){
#line 120
  unsigned char __nesc_result;
#line 120

#line 120
  __nesc_result = /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Resource__release(/*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430Spi0C__0__CLIENT_ID);
#line 120

#line 120
  return __nesc_result;
#line 120
}
#line 120
# 67 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P31*/HplMsp430GeneralIOP__17__IO__selectIOFunc(void )
#line 67
{
  /* atomic removed: atomic calls only */
#line 67
  * (volatile uint8_t * )27U &= ~(0x01 << 1);
}

# 99 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430Usart0P__SIMO__selectIOFunc(void ){
#line 99
  /*HplMsp430GeneralIOC.P31*/HplMsp430GeneralIOP__17__IO__selectIOFunc();
#line 99
}
#line 99
# 67 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P32*/HplMsp430GeneralIOP__18__IO__selectIOFunc(void )
#line 67
{
  /* atomic removed: atomic calls only */
#line 67
  * (volatile uint8_t * )27U &= ~(0x01 << 2);
}

# 99 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430Usart0P__SOMI__selectIOFunc(void ){
#line 99
  /*HplMsp430GeneralIOC.P32*/HplMsp430GeneralIOP__18__IO__selectIOFunc();
#line 99
}
#line 99
# 67 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P33*/HplMsp430GeneralIOP__19__IO__selectIOFunc(void )
#line 67
{
  /* atomic removed: atomic calls only */
#line 67
  * (volatile uint8_t * )27U &= ~(0x01 << 3);
}

# 99 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430Usart0P__UCLK__selectIOFunc(void ){
#line 99
  /*HplMsp430GeneralIOC.P33*/HplMsp430GeneralIOP__19__IO__selectIOFunc();
#line 99
}
#line 99
# 67 "/home/csilab/tinyos/tos/interfaces/TaskBasic.nc"
inline static error_t CC2420CsmaP__stopDone_task__postTask(void ){
#line 67
  unsigned char __nesc_result;
#line 67

#line 67
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(CC2420CsmaP__stopDone_task);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 57 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P45*/HplMsp430GeneralIOP__29__IO__clr(void )
#line 57
{
#line 57
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 57
    * (volatile uint8_t * )29U &= ~(0x01 << 5);
#line 57
    __nesc_atomic_end(__nesc_atomic); }
}

# 53 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*HplCC2420PinsC.VRENM*/Msp430GpioC__9__HplGeneralIO__clr(void ){
#line 53
  /*HplMsp430GeneralIOC.P45*/HplMsp430GeneralIOP__29__IO__clr();
#line 53
}
#line 53
# 49 "/home/csilab/tinyos/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplCC2420PinsC.VRENM*/Msp430GpioC__9__GeneralIO__clr(void )
#line 49
{
#line 49
  /*HplCC2420PinsC.VRENM*/Msp430GpioC__9__HplGeneralIO__clr();
}

# 41 "/home/csilab/tinyos/tos/interfaces/GeneralIO.nc"
inline static void CC2420ControlP__VREN__clr(void ){
#line 41
  /*HplCC2420PinsC.VRENM*/Msp430GpioC__9__GeneralIO__clr();
#line 41
}
#line 41
# 216 "/home/csilab/tinyos/tos/chips/cc2420/control/CC2420ControlP.nc"
static inline error_t CC2420ControlP__CC2420Power__stopVReg(void )
#line 216
{
  CC2420ControlP__m_state = CC2420ControlP__S_VREG_STOPPED;
  CC2420ControlP__RSTN__clr();
  CC2420ControlP__VREN__clr();
  CC2420ControlP__RSTN__set();
  return SUCCESS;
}

# 63 "/home/csilab/tinyos/tos/chips/cc2420/interfaces/CC2420Power.nc"
inline static error_t CC2420CsmaP__CC2420Power__stopVReg(void ){
#line 63
  unsigned char __nesc_result;
#line 63

#line 63
  __nesc_result = CC2420ControlP__CC2420Power__stopVReg();
#line 63

#line 63
  return __nesc_result;
#line 63
}
#line 63
# 69 "/home/csilab/tinyos/tos/types/TinyError.h"
static inline  error_t ecombine(error_t r1, error_t r2)




{
  return r1 == r2 ? r1 : FAIL;
}

# 102 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port10__clear(void )
#line 102
{
#line 102
  P1IFG &= ~(1 << 0);
}

# 52 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__HplInterrupt__clear(void ){
#line 52
  HplMsp430InterruptP__Port10__clear();
#line 52
}
#line 52
# 94 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port10__disable(void )
#line 94
{
#line 94
  P1IE &= ~(1 << 0);
}

# 47 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__HplInterrupt__disable(void ){
#line 47
  HplMsp430InterruptP__Port10__disable();
#line 47
}
#line 47
# 69 "/home/csilab/tinyos/tos/chips/msp430/pins/Msp430InterruptC.nc"
static inline error_t /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__Interrupt__disable(void )
#line 69
{
  /* atomic removed: atomic calls only */
#line 70
  {
    /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__HplInterrupt__disable();
    /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__HplInterrupt__clear();
  }
  return SUCCESS;
}

# 61 "/home/csilab/tinyos/tos/interfaces/GpioInterrupt.nc"
inline static error_t CC2420ReceiveP__InterruptFIFOP__disable(void ){
#line 61
  unsigned char __nesc_result;
#line 61

#line 61
  __nesc_result = /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__Interrupt__disable();
#line 61

#line 61
  return __nesc_result;
#line 61
}
#line 61
# 48 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*HplCC2420PinsC.CSNM*/Msp430GpioC__4__HplGeneralIO__set(void ){
#line 48
  /*HplMsp430GeneralIOC.P42*/HplMsp430GeneralIOP__26__IO__set();
#line 48
}
#line 48
# 48 "/home/csilab/tinyos/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplCC2420PinsC.CSNM*/Msp430GpioC__4__GeneralIO__set(void )
#line 48
{
#line 48
  /*HplCC2420PinsC.CSNM*/Msp430GpioC__4__HplGeneralIO__set();
}

# 40 "/home/csilab/tinyos/tos/interfaces/GeneralIO.nc"
inline static void CC2420ReceiveP__CSN__set(void ){
#line 40
  /*HplCC2420PinsC.CSNM*/Msp430GpioC__4__GeneralIO__set();
#line 40
}
#line 40
# 147 "/home/csilab/upma/chips/cc2420/CC2420ReceiveP.nc"
static inline error_t CC2420ReceiveP__StdControl__stop(void )
#line 147
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 148
    {
      CC2420ReceiveP__m_state = CC2420ReceiveP__S_STOPPED;
      CC2420ReceiveP__reset_state();
      CC2420ReceiveP__CSN__set();
      CC2420ReceiveP__InterruptFIFOP__disable();
    }
#line 153
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

# 40 "/home/csilab/tinyos/tos/interfaces/GeneralIO.nc"
inline static void CC2420TransmitP__CSN__set(void ){
#line 40
  /*HplCC2420PinsC.CSNM*/Msp430GpioC__4__GeneralIO__set();
#line 40
}
#line 40
# 67 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__selectIOFunc(void )
#line 67
{
  /* atomic removed: atomic calls only */
#line 67
  * (volatile uint8_t * )31U &= ~(0x01 << 1);
}

# 99 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__GeneralIO__selectIOFunc(void ){
#line 99
  /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__selectIOFunc();
#line 99
}
#line 99
# 135 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__disableEvents(void )
{
  * (volatile uint16_t * )388U &= ~0x0010;
}

# 58 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Msp430TimerControl__disableEvents(void ){
#line 58
  /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__disableEvents();
#line 58
}
#line 58
# 69 "/home/csilab/tinyos/tos/chips/msp430/timer/GpioCaptureC.nc"
static inline void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Capture__disable(void )
#line 69
{
  /* atomic removed: atomic calls only */
#line 70
  {
    /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Msp430TimerControl__disableEvents();
    /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__GeneralIO__selectIOFunc();
  }
}

# 66 "/home/csilab/tinyos/tos/interfaces/GpioCapture.nc"
inline static void CC2420TransmitP__CaptureSFD__disable(void ){
#line 66
  /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Capture__disable();
#line 66
}
#line 66
# 65 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Alarm__stop(void )
{
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__disableEvents();
}

# 73 "/home/csilab/tinyos/tos/lib/timer/Alarm.nc"
inline static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__AlarmFrom__stop(void ){
#line 73
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Alarm__stop();
#line 73
}
#line 73
# 102 "/home/csilab/tinyos/tos/lib/timer/TransformAlarmC.nc"
static inline void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Alarm__stop(void )
{
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__AlarmFrom__stop();
}

# 73 "/home/csilab/tinyos/tos/lib/timer/Alarm.nc"
inline static void CC2420TransmitP__BackoffTimer__stop(void ){
#line 73
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Alarm__stop();
#line 73
}
#line 73
# 198 "/home/csilab/upma/chips/cc2420/CC2420TransmitP.nc"
static inline error_t CC2420TransmitP__AsyncStdControl__stop(void )
#line 198
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 199
    {
      CC2420TransmitP__m_state = CC2420TransmitP__S_STOPPED;
      CC2420TransmitP__BackoffTimer__stop();
      CC2420TransmitP__CaptureSFD__disable();
      CC2420TransmitP__SpiResource__release();
      CC2420TransmitP__CSN__set();
    }
#line 205
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

# 105 "/home/csilab/tinyos/tos/interfaces/AsyncStdControl.nc"
inline static error_t CC2420CsmaP__SubControl__stop(void ){
#line 105
  unsigned char __nesc_result;
#line 105

#line 105
  __nesc_result = CC2420TransmitP__AsyncStdControl__stop();
#line 105
  __nesc_result = ecombine(__nesc_result, CC2420ReceiveP__StdControl__stop());
#line 105

#line 105
  return __nesc_result;
#line 105
}
#line 105
# 275 "/home/csilab/upma/chips/cc2420/CC2420CsmaP.nc"
static inline void CC2420CsmaP__shutdown(void )
#line 275
{
  CC2420CsmaP__SubControl__stop();
  CC2420CsmaP__CC2420Power__stopVReg();



  CC2420CsmaP__stopDone_task__postTask();
}

# 561 "/home/csilab/upma/lib/macs/pure-tdma/PureTDMASchedulerP.nc"
static inline void PureTDMASchedulerP__BeaconSend__sendDone(message_t *msg, error_t error)
#line 561
{
}

# 48 "/home/csilab/upma/interfaces/AsyncSend.nc"
inline static void BeaconSlotP__Send__sendDone(message_t * msg, error_t error){
#line 48
  PureTDMASchedulerP__BeaconSend__sendDone(msg, error);
#line 48
}
#line 48
# 93 "/home/csilab/upma/system/BeaconSlotP.nc"
static inline void BeaconSlotP__SubSend__sendDone(message_t *msg, error_t error)
#line 93
{
  if (msg != &BeaconSlotP__syncPkt) {
#line 94
    BeaconSlotP__Send__sendDone(msg, error);
    }
}

# 281 "/usr/local/lib/ncc/nesc_nx.h"
static __inline  uint8_t __nesc_ntoh_uint8(const void * source)
#line 281
{
  const uint8_t *base = source;

#line 283
  return base[0];
}

#line 303
static __inline  int8_t __nesc_ntoh_int8(const void * source)
#line 303
{
#line 303
  return __nesc_ntoh_uint8(source);
}

# 192 "/home/csilab/upma/chips/cc2420/CC2420PacketP.nc"
static inline cc2420_metadata_t *CC2420PacketP__CC2420PacketBody__getMetadata(message_t *msg)
#line 192
{
  return (cc2420_metadata_t *)msg->metadata;
}

#line 76
static inline bool CC2420PacketP__Acks__wasAcked(message_t *p_msg)
#line 76
{
  return __nesc_ntoh_int8(CC2420PacketP__CC2420PacketBody__getMetadata(p_msg)->ack.nxdata);
}

# 85 "/home/csilab/tinyos/tos/interfaces/PacketAcknowledgements.nc"
inline static bool PureTDMASchedulerP__ACK__wasAcked(message_t * msg){
#line 85
  unsigned char __nesc_result;
#line 85

#line 85
  __nesc_result = CC2420PacketP__Acks__wasAcked(msg);
#line 85

#line 85
  return __nesc_result;
#line 85
}
#line 85
# 565 "/home/csilab/upma/lib/macs/pure-tdma/PureTDMASchedulerP.nc"
static inline void PureTDMASchedulerP__SubSend__sendDone(message_t *msg, error_t error)
#line 565
{
  uint8_t tmp_tx_opper;
#line 566
  uint8_t tmp_txing_flowid;

  if (PureTDMASchedulerP__is_root == TRUE) {
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 569
        tmp_txing_flowid = PureTDMASchedulerP__my_flow_id;
#line 569
        __nesc_atomic_end(__nesc_atomic); }
    }
  else 
#line 570
    {
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 571
        tmp_txing_flowid = PureTDMASchedulerP__txing_flowid;
#line 571
        __nesc_atomic_end(__nesc_atomic); }
    }

  if (PureTDMASchedulerP__ACK__wasAcked(msg)) {

      PureTDMASchedulerP__reset_var(tmp_txing_flowid);
    }
  else 
#line 577
    {

      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 579
        tmp_tx_opper = PureTDMASchedulerP__tx_opper[tmp_txing_flowid];
#line 579
        __nesc_atomic_end(__nesc_atomic); }
      if (tmp_tx_opper <= 0) {
          PureTDMASchedulerP__reset_var(tmp_txing_flowid);
        }
    }


  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 586
    PureTDMASchedulerP__txing_flowid = 0;
#line 586
    __nesc_atomic_end(__nesc_atomic); }
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 587
    PureTDMASchedulerP__phyLock = FALSE;
#line 587
    __nesc_atomic_end(__nesc_atomic); }
}

# 48 "/home/csilab/upma/interfaces/AsyncSend.nc"
inline static void /*TDMASlotSenderC.TDMASlotSender.GenericSlotSenderP*/GenericSlotSenderP__0__Send__sendDone(message_t * msg, error_t error){
#line 48
  PureTDMASchedulerP__SubSend__sendDone(msg, error);
#line 48
}
#line 48
# 134 "/home/csilab/upma/system/GenericSlotSenderP.nc"
static inline void /*TDMASlotSenderC.TDMASlotSender.GenericSlotSenderP*/GenericSlotSenderP__0__SubSend__sendDone(message_t *msg, error_t error)
#line 134
{
  if (/*TDMASlotSenderC.TDMASlotSender.GenericSlotSenderP*/GenericSlotSenderP__0__toSend == msg) {
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 136
        /*TDMASlotSenderC.TDMASlotSender.GenericSlotSenderP*/GenericSlotSenderP__0__state = /*TDMASlotSenderC.TDMASlotSender.GenericSlotSenderP*/GenericSlotSenderP__0__S_END;
#line 136
        __nesc_atomic_end(__nesc_atomic); }
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 137
        /*TDMASlotSenderC.TDMASlotSender.GenericSlotSenderP*/GenericSlotSenderP__0__toSend = (void *)0;
#line 137
        __nesc_atomic_end(__nesc_atomic); }
      /*TDMASlotSenderC.TDMASlotSender.GenericSlotSenderP*/GenericSlotSenderP__0__Send__sendDone(msg, error);
    }
}

# 121 "/home/csilab/upma/system/SenderDispatcherP.nc"
static inline void SenderDispatcherP__Send__default__sendDone(uint8_t type, message_t *msg, error_t error)
#line 121
{
}

# 48 "/home/csilab/upma/interfaces/AsyncSend.nc"
inline static void SenderDispatcherP__Send__sendDone(uint8_t arg_0x7f7fec194250, message_t * msg, error_t error){
#line 48
  switch (arg_0x7f7fec194250) {
#line 48
    case TDMA_SLOT:
#line 48
      /*TDMASlotSenderC.TDMASlotSender.GenericSlotSenderP*/GenericSlotSenderP__0__SubSend__sendDone(msg, error);
#line 48
      break;
#line 48
    case BEACON_SLOT:
#line 48
      BeaconSlotP__SubSend__sendDone(msg, error);
#line 48
      break;
#line 48
    default:
#line 48
      SenderDispatcherP__Send__default__sendDone(arg_0x7f7fec194250, msg, error);
#line 48
      break;
#line 48
    }
#line 48
}
#line 48
# 102 "/home/csilab/upma/system/SenderDispatcherP.nc"
static inline void SenderDispatcherP__SubSend__sendDone(message_t *msg, error_t error)
#line 102
{
  uint8_t prev_last_id;

#line 104
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 104
    prev_last_id = SenderDispatcherP__last_id;
#line 104
    __nesc_atomic_end(__nesc_atomic); }

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 106
    {
      if (msg == SenderDispatcherP__toSend) {
          SenderDispatcherP__toSend = (void *)0;
        }
    }
#line 110
    __nesc_atomic_end(__nesc_atomic); }

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 112
    SenderDispatcherP__last_id = NO_SLOT;
#line 112
    __nesc_atomic_end(__nesc_atomic); }





  SenderDispatcherP__Send__sendDone(prev_last_id, msg, error);
}

# 48 "/home/csilab/upma/interfaces/AsyncSend.nc"
inline static void CC2420CsmaP__Send__sendDone(message_t * msg, error_t error){
#line 48
  SenderDispatcherP__SubSend__sendDone(msg, error);
#line 48
}
#line 48
# 69 "/home/csilab/tinyos/tos/system/QueueC.nc"
static inline void /*PureTDMASchedulerC.LOQueueC*/QueueC__2__printQueue(void )
#line 69
{
}

#line 53
static inline bool /*PureTDMASchedulerC.LOQueueC*/QueueC__2__Queue__empty(void )
#line 53
{
  return /*PureTDMASchedulerC.LOQueueC*/QueueC__2__size == 0;
}









static inline /*PureTDMASchedulerC.LOQueueC*/QueueC__2__queue_t /*PureTDMASchedulerC.LOQueueC*/QueueC__2__Queue__head(void )
#line 65
{
  return /*PureTDMASchedulerC.LOQueueC*/QueueC__2__queue[/*PureTDMASchedulerC.LOQueueC*/QueueC__2__head];
}

#line 85
static inline /*PureTDMASchedulerC.LOQueueC*/QueueC__2__queue_t /*PureTDMASchedulerC.LOQueueC*/QueueC__2__Queue__dequeue(void )
#line 85
{
  /*PureTDMASchedulerC.LOQueueC*/QueueC__2__queue_t t = /*PureTDMASchedulerC.LOQueueC*/QueueC__2__Queue__head();

#line 87
  ;
  if (!/*PureTDMASchedulerC.LOQueueC*/QueueC__2__Queue__empty()) {
      /*PureTDMASchedulerC.LOQueueC*/QueueC__2__head++;
      if (/*PureTDMASchedulerC.LOQueueC*/QueueC__2__head == 1) {
#line 90
        /*PureTDMASchedulerC.LOQueueC*/QueueC__2__head = 0;
        }
#line 91
      /*PureTDMASchedulerC.LOQueueC*/QueueC__2__size--;
      /*PureTDMASchedulerC.LOQueueC*/QueueC__2__printQueue();
    }
  return t;
}

# 81 "/home/csilab/tinyos/tos/interfaces/Queue.nc"
inline static PureTDMASchedulerP__LOforwardQ__t  PureTDMASchedulerP__LOforwardQ__dequeue(void ){
#line 81
  nx_struct adaptivehart_msg *__nesc_result;
#line 81

#line 81
  __nesc_result = /*PureTDMASchedulerC.LOQueueC*/QueueC__2__Queue__dequeue();
#line 81

#line 81
  return __nesc_result;
#line 81
}
#line 81
# 57 "/home/csilab/tinyos/tos/system/QueueC.nc"
static inline uint8_t /*PureTDMASchedulerC.LOQueueC*/QueueC__2__Queue__size(void )
#line 57
{
  return /*PureTDMASchedulerC.LOQueueC*/QueueC__2__size;
}

# 58 "/home/csilab/tinyos/tos/interfaces/Queue.nc"
inline static uint8_t PureTDMASchedulerP__LOforwardQ__size(void ){
#line 58
  unsigned char __nesc_result;
#line 58

#line 58
  __nesc_result = /*PureTDMASchedulerC.LOQueueC*/QueueC__2__Queue__size();
#line 58

#line 58
  return __nesc_result;
#line 58
}
#line 58
# 69 "/home/csilab/tinyos/tos/system/QueueC.nc"
static inline void /*PureTDMASchedulerC.HIQueueC*/QueueC__1__printQueue(void )
#line 69
{
}

#line 53
static inline bool /*PureTDMASchedulerC.HIQueueC*/QueueC__1__Queue__empty(void )
#line 53
{
  return /*PureTDMASchedulerC.HIQueueC*/QueueC__1__size == 0;
}









static inline /*PureTDMASchedulerC.HIQueueC*/QueueC__1__queue_t /*PureTDMASchedulerC.HIQueueC*/QueueC__1__Queue__head(void )
#line 65
{
  return /*PureTDMASchedulerC.HIQueueC*/QueueC__1__queue[/*PureTDMASchedulerC.HIQueueC*/QueueC__1__head];
}

#line 85
static inline /*PureTDMASchedulerC.HIQueueC*/QueueC__1__queue_t /*PureTDMASchedulerC.HIQueueC*/QueueC__1__Queue__dequeue(void )
#line 85
{
  /*PureTDMASchedulerC.HIQueueC*/QueueC__1__queue_t t = /*PureTDMASchedulerC.HIQueueC*/QueueC__1__Queue__head();

#line 87
  ;
  if (!/*PureTDMASchedulerC.HIQueueC*/QueueC__1__Queue__empty()) {
      /*PureTDMASchedulerC.HIQueueC*/QueueC__1__head++;
      if (/*PureTDMASchedulerC.HIQueueC*/QueueC__1__head == 1) {
#line 90
        /*PureTDMASchedulerC.HIQueueC*/QueueC__1__head = 0;
        }
#line 91
      /*PureTDMASchedulerC.HIQueueC*/QueueC__1__size--;
      /*PureTDMASchedulerC.HIQueueC*/QueueC__1__printQueue();
    }
  return t;
}

# 81 "/home/csilab/tinyos/tos/interfaces/Queue.nc"
inline static PureTDMASchedulerP__HIforwardQ__t  PureTDMASchedulerP__HIforwardQ__dequeue(void ){
#line 81
  nx_struct adaptivehart_msg *__nesc_result;
#line 81

#line 81
  __nesc_result = /*PureTDMASchedulerC.HIQueueC*/QueueC__1__Queue__dequeue();
#line 81

#line 81
  return __nesc_result;
#line 81
}
#line 81
# 57 "/home/csilab/tinyos/tos/system/QueueC.nc"
static inline uint8_t /*PureTDMASchedulerC.HIQueueC*/QueueC__1__Queue__size(void )
#line 57
{
  return /*PureTDMASchedulerC.HIQueueC*/QueueC__1__size;
}

# 58 "/home/csilab/tinyos/tos/interfaces/Queue.nc"
inline static uint8_t PureTDMASchedulerP__HIforwardQ__size(void ){
#line 58
  unsigned char __nesc_result;
#line 58

#line 58
  __nesc_result = /*PureTDMASchedulerC.HIQueueC*/QueueC__1__Queue__size();
#line 58

#line 58
  return __nesc_result;
#line 58
}
#line 58
# 768 "/home/csilab/upma/lib/macs/pure-tdma/PureTDMASchedulerP.nc"
static inline void PureTDMASchedulerP__Q_reset(uint8_t flowid)
#line 768
{
  uint8_t queueSize;

#line 770
  if (flowid == 1) {
      queueSize = PureTDMASchedulerP__HIforwardQ__size();
      for (PureTDMASchedulerP__i = 0; PureTDMASchedulerP__i < queueSize; PureTDMASchedulerP__i++) {
          PureTDMASchedulerP__HIforwardQ__dequeue();
        }
    }
  else {
#line 776
    if (flowid == 2) {
        queueSize = PureTDMASchedulerP__LOforwardQ__size();
        for (PureTDMASchedulerP__i = 0; PureTDMASchedulerP__i < queueSize; PureTDMASchedulerP__i++) {
            PureTDMASchedulerP__LOforwardQ__dequeue();
          }
      }
    }
}

# 53 "/home/csilab/tinyos/tos/chips/cc2420/interfaces/CC2420Strobe.nc"
inline static cc2420_status_t CC2420TransmitP__STXONCCA__strobe(void ){
#line 53
  unsigned char __nesc_result;
#line 53

#line 53
  __nesc_result = CC2420SpiP__Strobe__strobe(CC2420_STXONCCA);
#line 53

#line 53
  return __nesc_result;
#line 53
}
#line 53
inline static cc2420_status_t CC2420TransmitP__STXON__strobe(void ){
#line 53
  unsigned char __nesc_result;
#line 53

#line 53
  __nesc_result = CC2420SpiP__Strobe__strobe(CC2420_STXON);
#line 53

#line 53
  return __nesc_result;
#line 53
}
#line 53
inline static cc2420_status_t CC2420TransmitP__SNOP__strobe(void ){
#line 53
  unsigned char __nesc_result;
#line 53

#line 53
  __nesc_result = CC2420SpiP__Strobe__strobe(CC2420_SNOP);
#line 53

#line 53
  return __nesc_result;
#line 53
}
#line 53
# 63 "/home/csilab/tinyos/tos/chips/cc2420/interfaces/CC2420Register.nc"
inline static cc2420_status_t CC2420TransmitP__TXCTRL__write(uint16_t data){
#line 63
  unsigned char __nesc_result;
#line 63

#line 63
  __nesc_result = CC2420SpiP__Reg__write(CC2420_TXCTRL, data);
#line 63

#line 63
  return __nesc_result;
#line 63
}
#line 63
# 102 "/home/csilab/tinyos/tos/chips/cc2420/spi/CC2420SpiP.nc"
static inline error_t CC2420SpiP__ChipSpiResource__attemptRelease(void )
#line 102
{
  return CC2420SpiP__attemptRelease();
}

# 39 "/home/csilab/tinyos/tos/chips/cc2420/interfaces/ChipSpiResource.nc"
inline static error_t CC2420TransmitP__ChipSpiResource__attemptRelease(void ){
#line 39
  unsigned char __nesc_result;
#line 39

#line 39
  __nesc_result = CC2420SpiP__ChipSpiResource__attemptRelease();
#line 39

#line 39
  return __nesc_result;
#line 39
}
#line 39
# 65 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__selectModuleFunc(void )
#line 65
{
  /* atomic removed: atomic calls only */
#line 65
  * (volatile uint8_t * )31U |= 0x01 << 1;
}

# 92 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__GeneralIO__selectModuleFunc(void ){
#line 92
  /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__selectModuleFunc();
#line 92
}
#line 92
# 57 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  uint16_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__CC2int(/*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__cc_t x)
#line 57
{
#line 57
  union /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4____nesc_unnamed4405 {
#line 57
    /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__cc_t f;
#line 57
    uint16_t t;
  } 
#line 57
  c = { .f = x };

#line 57
  return c.t;
}

#line 72
static inline uint16_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__captureControl(uint8_t l_cm)
{
  /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__cc_t x = { 
  .cm = l_cm & 0x03, 
  .ccis = 0, 
  .clld = 0, 
  .cap = 1, 
  .scs = 0, 
  .ccie = 0 };

  return /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__CC2int(x);
}

#line 110
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__setControlAsCapture(uint8_t cm)
{
  * (volatile uint16_t * )388U = /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__captureControl(cm);
}

# 55 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Msp430TimerControl__setControlAsCapture(uint8_t cm){
#line 55
  /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__setControlAsCapture(cm);
#line 55
}
#line 55
# 130 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__enableEvents(void )
{
  * (volatile uint16_t * )388U |= 0x0010;
}

# 57 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Msp430TimerControl__enableEvents(void ){
#line 57
  /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__enableEvents();
#line 57
}
#line 57
# 192 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Compare__default__fired(void )
{
}

# 45 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Compare__fired(void ){
#line 45
  /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Compare__default__fired();
#line 45
}
#line 45
# 150 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__getEvent(void )
{
  return * (volatile uint16_t * )404U;
}

# 322 "/usr/local/lib/ncc/nesc_nx.h"
static __inline  uint16_t __nesc_ntoh_leuint16(const void * source)
#line 322
{
  const uint8_t *base = source;

#line 324
  return ((uint16_t )base[1] << 8) | base[0];
}

# 70 "/home/csilab/tinyos/tos/interfaces/PacketTimeStamp.nc"
inline static void CC2420TransmitP__PacketTimeStamp__clear(message_t * msg){
#line 70
  CC2420PacketP__PacketTimeStamp32khz__clear(msg);
#line 70
}
#line 70
# 171 "/home/csilab/upma/chips/cc2420/CC2420ReceiveP.nc"
static inline void CC2420ReceiveP__CC2420Receive__sfd_dropped(void )
#line 171
{
  if (CC2420ReceiveP__m_timestamp_size) {
      CC2420ReceiveP__m_timestamp_size--;
    }
}

# 55 "/home/csilab/tinyos/tos/chips/cc2420/interfaces/CC2420Receive.nc"
inline static void CC2420TransmitP__CC2420Receive__sfd_dropped(void ){
#line 55
  CC2420ReceiveP__CC2420Receive__sfd_dropped();
#line 55
}
#line 55
# 59 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline uint8_t /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__getRaw(void )
#line 59
{
#line 59
  return * (volatile uint8_t * )28U & (0x01 << 1);
}

#line 60
static inline bool /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__get(void )
#line 60
{
#line 60
  return /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__getRaw() != 0;
}

# 73 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static bool /*HplCC2420PinsC.SFDM*/Msp430GpioC__8__HplGeneralIO__get(void ){
#line 73
  unsigned char __nesc_result;
#line 73

#line 73
  __nesc_result = /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__get();
#line 73

#line 73
  return __nesc_result;
#line 73
}
#line 73
# 51 "/home/csilab/tinyos/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline bool /*HplCC2420PinsC.SFDM*/Msp430GpioC__8__GeneralIO__get(void )
#line 51
{
#line 51
  return /*HplCC2420PinsC.SFDM*/Msp430GpioC__8__HplGeneralIO__get();
}

# 43 "/home/csilab/tinyos/tos/interfaces/GeneralIO.nc"
inline static bool CC2420TransmitP__SFD__get(void ){
#line 43
  unsigned char __nesc_result;
#line 43

#line 43
  __nesc_result = /*HplCC2420PinsC.SFDM*/Msp430GpioC__8__GeneralIO__get();
#line 43

#line 43
  return __nesc_result;
#line 43
}
#line 43
# 162 "/home/csilab/upma/chips/cc2420/CC2420ReceiveP.nc"
static inline void CC2420ReceiveP__CC2420Receive__sfd(uint32_t time)
#line 162
{
  if (CC2420ReceiveP__m_timestamp_size < CC2420ReceiveP__TIMESTAMP_QUEUE_SIZE) {
      uint8_t tail = (CC2420ReceiveP__m_timestamp_head + CC2420ReceiveP__m_timestamp_size) % 
      CC2420ReceiveP__TIMESTAMP_QUEUE_SIZE;

#line 166
      CC2420ReceiveP__m_timestamp_queue[tail] = time;
      CC2420ReceiveP__m_timestamp_size++;
    }
}

# 49 "/home/csilab/tinyos/tos/chips/cc2420/interfaces/CC2420Receive.nc"
inline static void CC2420TransmitP__CC2420Receive__sfd(uint32_t time){
#line 49
  CC2420ReceiveP__CC2420Receive__sfd(time);
#line 49
}
#line 49
# 65 "/home/csilab/tinyos/tos/chips/msp430/timer/GpioCaptureC.nc"
static inline error_t /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Capture__captureFallingEdge(void )
#line 65
{
  return /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__enableCapture(MSP430TIMER_CM_FALLING);
}

# 54 "/home/csilab/tinyos/tos/interfaces/GpioCapture.nc"
inline static error_t CC2420TransmitP__CaptureSFD__captureFallingEdge(void ){
#line 54
  unsigned char __nesc_result;
#line 54

#line 54
  __nesc_result = /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Capture__captureFallingEdge();
#line 54

#line 54
  return __nesc_result;
#line 54
}
#line 54
# 188 "/home/csilab/upma/chips/cc2420/CC2420PacketP.nc"
static inline cc2420_header_t * CC2420PacketP__CC2420PacketBody__getHeader(message_t * msg)
#line 188
{
  return (cc2420_header_t * )((uint8_t *)msg + (unsigned short )& ((message_t *)0)->data - sizeof(cc2420_header_t ));
}

# 42 "/home/csilab/tinyos/tos/chips/cc2420/interfaces/CC2420PacketBody.nc"
inline static cc2420_header_t * CC2420TransmitP__CC2420PacketBody__getHeader(message_t * msg){
#line 42
  nx_struct cc2420_header_t *__nesc_result;
#line 42

#line 42
  __nesc_result = CC2420PacketP__CC2420PacketBody__getHeader(msg);
#line 42

#line 42
  return __nesc_result;
#line 42
}
#line 42
# 63 "/home/csilab/tinyos/tos/chips/cc2420/interfaces/CC2420Ram.nc"
inline static cc2420_status_t CC2420TransmitP__TXFIFO_RAM__write(uint8_t offset, uint8_t * data, uint8_t length){
#line 63
  unsigned char __nesc_result;
#line 63

#line 63
  __nesc_result = CC2420SpiP__Ram__write(CC2420_RAM_TXFIFO, offset, data, length);
#line 63

#line 63
  return __nesc_result;
#line 63
}
#line 63
# 53 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*HplCC2420PinsC.CSNM*/Msp430GpioC__4__HplGeneralIO__clr(void ){
#line 53
  /*HplMsp430GeneralIOC.P42*/HplMsp430GeneralIOP__26__IO__clr();
#line 53
}
#line 53
# 49 "/home/csilab/tinyos/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplCC2420PinsC.CSNM*/Msp430GpioC__4__GeneralIO__clr(void )
#line 49
{
#line 49
  /*HplCC2420PinsC.CSNM*/Msp430GpioC__4__HplGeneralIO__clr();
}

# 41 "/home/csilab/tinyos/tos/interfaces/GeneralIO.nc"
inline static void CC2420TransmitP__CSN__clr(void ){
#line 41
  /*HplCC2420PinsC.CSNM*/Msp430GpioC__4__GeneralIO__clr();
#line 41
}
#line 41
# 259 "/home/csilab/upma/chips/cc2420/CC2420PacketP.nc"
static inline uint8_t CC2420PacketP__PacketTimeSyncOffset__get(message_t *msg)
{
  return __nesc_ntoh_leuint8(CC2420PacketP__CC2420PacketBody__getHeader(msg)->length.nxdata)
   + (sizeof(cc2420_header_t ) - MAC_HEADER_SIZE)
   - MAC_FOOTER_SIZE
   - sizeof(timesync_radio_t );
}

# 58 "/home/csilab/tinyos/tos/chips/cc2420/interfaces/PacketTimeSyncOffset.nc"
inline static uint8_t CC2420TransmitP__PacketTimeSyncOffset__get(message_t * msg){
#line 58
  unsigned char __nesc_result;
#line 58

#line 58
  __nesc_result = CC2420PacketP__PacketTimeSyncOffset__get(msg);
#line 58

#line 58
  return __nesc_result;
#line 58
}
#line 58
# 250 "/home/csilab/upma/chips/cc2420/CC2420PacketP.nc"
static inline bool CC2420PacketP__PacketTimeSyncOffset__isSet(message_t *msg)
{
  return __nesc_ntoh_int8(CC2420PacketP__CC2420PacketBody__getMetadata(msg)->timesync.nxdata);
}

# 50 "/home/csilab/tinyos/tos/chips/cc2420/interfaces/PacketTimeSyncOffset.nc"
inline static bool CC2420TransmitP__PacketTimeSyncOffset__isSet(message_t * msg){
#line 50
  unsigned char __nesc_result;
#line 50

#line 50
  __nesc_result = CC2420PacketP__PacketTimeSyncOffset__isSet(msg);
#line 50

#line 50
  return __nesc_result;
#line 50
}
#line 50
# 217 "/home/csilab/upma/chips/cc2420/CC2420PacketP.nc"
static inline void CC2420PacketP__PacketTimeStamp32khz__set(message_t *msg, uint32_t value)
{
  __nesc_hton_uint32(CC2420PacketP__CC2420PacketBody__getMetadata(msg)->timestamp.nxdata, value);
}

# 78 "/home/csilab/tinyos/tos/interfaces/PacketTimeStamp.nc"
inline static void CC2420TransmitP__PacketTimeStamp__set(message_t * msg, CC2420TransmitP__PacketTimeStamp__size_type value){
#line 78
  CC2420PacketP__PacketTimeStamp32khz__set(msg, value);
#line 78
}
#line 78
# 109 "/home/csilab/tinyos/tos/lib/timer/Alarm.nc"
inline static CC2420TransmitP__BackoffTimer__size_type CC2420TransmitP__BackoffTimer__getNow(void ){
#line 109
  unsigned long __nesc_result;
#line 109

#line 109
  __nesc_result = /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Alarm__getNow();
#line 109

#line 109
  return __nesc_result;
#line 109
}
#line 109
# 262 "/home/csilab/upma/chips/cc2420/CC2420TransmitP.nc"
static __inline uint32_t CC2420TransmitP__getTime32(uint16_t time)
{
  uint32_t recent_time = CC2420TransmitP__BackoffTimer__getNow();

#line 265
  return recent_time + (int16_t )(time - recent_time);
}

#line 281
static inline void CC2420TransmitP__CaptureSFD__captured(uint16_t time)
#line 281
{
  unsigned char *__nesc_temp45;
  unsigned char *__nesc_temp44;
#line 282
  uint32_t time32;
  uint8_t sfd_state = 0;

  /* atomic removed: atomic calls only */
#line 284
  {
    time32 = CC2420TransmitP__getTime32(time);
    switch (CC2420TransmitP__m_state) {

        case CC2420TransmitP__S_SFD: 
          CC2420TransmitP__m_state = CC2420TransmitP__S_EFD;
        CC2420TransmitP__sfdHigh = TRUE;


        CC2420TransmitP__m_receiving = FALSE;
        CC2420TransmitP__CaptureSFD__captureFallingEdge();
        CC2420TransmitP__PacketTimeStamp__set(CC2420TransmitP__m_msg, time32);
        if (CC2420TransmitP__PacketTimeSyncOffset__isSet(CC2420TransmitP__m_msg)) {
            uint8_t absOffset = sizeof(message_header_t ) - sizeof(cc2420_header_t ) + CC2420TransmitP__PacketTimeSyncOffset__get(CC2420TransmitP__m_msg);
            timesync_radio_t *timesync = (timesync_radio_t *)((nx_uint8_t *)CC2420TransmitP__m_msg + absOffset);

            (__nesc_temp44 = (*timesync).nxdata, __nesc_hton_uint32(__nesc_temp44, __nesc_ntoh_uint32(__nesc_temp44) - time32));
            CC2420TransmitP__CSN__clr();
            CC2420TransmitP__TXFIFO_RAM__write(absOffset, (uint8_t *)timesync, sizeof(timesync_radio_t ));
            CC2420TransmitP__CSN__set();

            (__nesc_temp45 = (*timesync).nxdata, __nesc_hton_uint32(__nesc_temp45, __nesc_ntoh_uint32(__nesc_temp45) + time32));
          }
        if (__nesc_ntoh_leuint16(CC2420TransmitP__CC2420PacketBody__getHeader(CC2420TransmitP__m_msg)->fcf.nxdata) & (1 << IEEE154_FCF_ACK_REQ)) {

            CC2420TransmitP__abortSpiRelease = TRUE;
          }
        CC2420TransmitP__releaseSpiResource();
        CC2420TransmitP__BackoffTimer__stop();

        if (CC2420TransmitP__SFD__get()) {
            break;
          }


        case CC2420TransmitP__S_EFD: 
          CC2420TransmitP__sfdHigh = FALSE;
        CC2420TransmitP__CaptureSFD__captureRisingEdge();

        if (__nesc_ntoh_leuint16(CC2420TransmitP__CC2420PacketBody__getHeader(CC2420TransmitP__m_msg)->fcf.nxdata) & (1 << IEEE154_FCF_ACK_REQ)) {
            CC2420TransmitP__m_state = CC2420TransmitP__S_ACK_WAIT;
            CC2420TransmitP__BackoffTimer__start(CC2420_ACK_WAIT_DELAY);
          }
        else 
#line 326
          {
            CC2420TransmitP__signalDone(SUCCESS);
          }

        if (!CC2420TransmitP__SFD__get()) {
            break;
          }


        default: 

          if (!CC2420TransmitP__m_receiving && CC2420TransmitP__sfdHigh == FALSE) {
              CC2420TransmitP__sfdHigh = TRUE;
              CC2420TransmitP__CaptureSFD__captureFallingEdge();

              sfd_state = CC2420TransmitP__SFD__get();
              CC2420TransmitP__CC2420Receive__sfd(time32);
              CC2420TransmitP__m_receiving = TRUE;
              CC2420TransmitP__m_prev_time = time;
              if (CC2420TransmitP__SFD__get()) {

                  return;
                }
            }



        if (CC2420TransmitP__sfdHigh == TRUE) {
            CC2420TransmitP__sfdHigh = FALSE;
            CC2420TransmitP__CaptureSFD__captureRisingEdge();
            CC2420TransmitP__m_receiving = FALSE;








            if (sfd_state == 0 && time - CC2420TransmitP__m_prev_time < 10) {
                CC2420TransmitP__CC2420Receive__sfd_dropped();
                if (CC2420TransmitP__m_msg) {
                  CC2420TransmitP__PacketTimeStamp__clear(CC2420TransmitP__m_msg);
                  }
              }
#line 370
            break;
          }
      }
  }
}

# 61 "/home/csilab/tinyos/tos/interfaces/GpioCapture.nc"
inline static void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Capture__captured(uint16_t time){
#line 61
  CC2420TransmitP__CaptureSFD__captured(time);
#line 61
}
#line 61
# 175 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__clearOverflow(void )
{
  * (volatile uint16_t * )388U &= ~0x0002;
}

# 68 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Msp430Capture__clearOverflow(void ){
#line 68
  /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__clearOverflow();
#line 68
}
#line 68
# 95 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__clearPendingInterrupt(void )
{
  * (volatile uint16_t * )388U &= ~0x0001;
}

# 44 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Msp430TimerControl__clearPendingInterrupt(void ){
#line 44
  /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__clearPendingInterrupt();
#line 44
}
#line 44
# 76 "/home/csilab/tinyos/tos/chips/msp430/timer/GpioCaptureC.nc"
static inline void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Msp430Capture__captured(uint16_t time)
#line 76
{
  /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Msp430TimerControl__clearPendingInterrupt();
  /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Msp430Capture__clearOverflow();
  /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Capture__captured(time);
}

# 86 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__captured(uint16_t time){
#line 86
  /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Msp430Capture__captured(time);
#line 86
}
#line 86
# 58 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__cc_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__int2CC(uint16_t x)
#line 58
{
#line 58
  union /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4____nesc_unnamed4406 {
#line 58
    uint16_t f;
#line 58
    /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__cc_t t;
  } 
#line 58
  c = { .f = x };

#line 58
  return c.t;
}

#line 85
static inline /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__cc_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__getControl(void )
{
  return /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__int2CC(* (volatile uint16_t * )388U);
}

#line 180
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Event__fired(void )
{
  if (/*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__getControl().cap) {
    /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__captured(/*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__getEvent());
    }
  else {
#line 185
    /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Compare__fired();
    }
}

# 67 "/home/csilab/tinyos/tos/interfaces/TaskBasic.nc"
inline static error_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired__postTask(void ){
#line 67
  unsigned char __nesc_result;
#line 67

#line 67
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(/*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 81 "/home/csilab/tinyos/tos/lib/timer/AlarmToTimerC.nc"
static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__fired(void )
{
#line 82
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired__postTask();
}

# 78 "/home/csilab/tinyos/tos/lib/timer/Alarm.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__Alarm__fired(void ){
#line 78
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__fired();
#line 78
}
#line 78
# 162 "/home/csilab/tinyos/tos/lib/timer/TransformAlarmC.nc"
static inline void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__AlarmFrom__fired(void )
{
  /* atomic removed: atomic calls only */
  {
    if (/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__m_dt == 0) 
      {
        /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__Alarm__fired();
      }
    else 
      {
        /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__set_alarm();
      }
  }
}

# 78 "/home/csilab/tinyos/tos/lib/timer/Alarm.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Alarm__fired(void ){
#line 78
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__AlarmFrom__fired();
#line 78
}
#line 78
# 135 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__disableEvents(void )
{
  * (volatile uint16_t * )390U &= ~0x0010;
}

# 58 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430TimerControl__disableEvents(void ){
#line 58
  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__disableEvents();
#line 58
}
#line 58
# 70 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430Compare__fired(void )
{
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430TimerControl__disableEvents();
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Alarm__fired();
}

# 45 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__fired(void ){
#line 45
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430Compare__fired();
#line 45
}
#line 45
# 150 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__getEvent(void )
{
  return * (volatile uint16_t * )406U;
}

#line 188
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__default__captured(uint16_t n)
{
}

# 86 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__captured(uint16_t time){
#line 86
  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__default__captured(time);
#line 86
}
#line 86
# 58 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__cc_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__int2CC(uint16_t x)
#line 58
{
#line 58
  union /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5____nesc_unnamed4407 {
#line 58
    uint16_t f;
#line 58
    /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__cc_t t;
  } 
#line 58
  c = { .f = x };

#line 58
  return c.t;
}

#line 85
static inline /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__cc_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__getControl(void )
{
  return /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__int2CC(* (volatile uint16_t * )390U);
}

#line 180
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Event__fired(void )
{
  if (/*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__getControl().cap) {
    /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__captured(/*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__getEvent());
    }
  else {
#line 185
    /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__fired();
    }
}

# 64 "/home/csilab/tinyos/tos/lib/timer/Counter.nc"
inline static /*CounterMilli32C.Transform*/TransformCounterC__1__CounterFrom__size_type /*CounterMilli32C.Transform*/TransformCounterC__1__CounterFrom__get(void ){
#line 64
  unsigned int __nesc_result;
#line 64

#line 64
  __nesc_result = /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__get();
#line 64

#line 64
  return __nesc_result;
#line 64
}
#line 64







inline static bool /*CounterMilli32C.Transform*/TransformCounterC__1__CounterFrom__isOverflowPending(void ){
#line 71
  unsigned char __nesc_result;
#line 71

#line 71
  __nesc_result = /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__isOverflowPending();
#line 71

#line 71
  return __nesc_result;
#line 71
}
#line 71
# 130 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__enableEvents(void )
{
  * (volatile uint16_t * )390U |= 0x0010;
}

# 57 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430TimerControl__enableEvents(void ){
#line 57
  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__enableEvents();
#line 57
}
#line 57
# 95 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__clearPendingInterrupt(void )
{
  * (volatile uint16_t * )390U &= ~0x0001;
}

# 44 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430TimerControl__clearPendingInterrupt(void ){
#line 44
  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__clearPendingInterrupt();
#line 44
}
#line 44
# 155 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__setEvent(uint16_t x)
{
  * (volatile uint16_t * )406U = x;
}

# 41 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430Compare__setEvent(uint16_t time){
#line 41
  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__setEvent(time);
#line 41
}
#line 41
# 45 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430Timer.nc"
inline static uint16_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Timer__get(void ){
#line 45
  unsigned int __nesc_result;
#line 45

#line 45
  __nesc_result = /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__get();
#line 45

#line 45
  return __nesc_result;
#line 45
}
#line 45
# 165 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__setEventFromNow(uint16_t x)
{
  * (volatile uint16_t * )406U = /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Timer__get() + x;
}

# 43 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430Compare__setEventFromNow(uint16_t delta){
#line 43
  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__setEventFromNow(delta);
#line 43
}
#line 43
# 45 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430Timer.nc"
inline static uint16_t /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430Timer__get(void ){
#line 45
  unsigned int __nesc_result;
#line 45

#line 45
  __nesc_result = /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__get();
#line 45

#line 45
  return __nesc_result;
#line 45
}
#line 45
# 81 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Alarm__startAt(uint16_t t0, uint16_t dt)
{
  /* atomic removed: atomic calls only */
  {
    uint16_t now = /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430Timer__get();
    uint16_t elapsed = now - t0;

#line 87
    if (elapsed >= dt) 
      {
        /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430Compare__setEventFromNow(2);
      }
    else 
      {
        uint16_t remaining = dt - elapsed;

#line 94
        if (remaining <= 2) {
          /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430Compare__setEventFromNow(2);
          }
        else {
#line 97
          /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430Compare__setEvent(now + remaining);
          }
      }
#line 99
    /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430TimerControl__clearPendingInterrupt();
    /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430TimerControl__enableEvents();
  }
}

# 103 "/home/csilab/tinyos/tos/lib/timer/Alarm.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__AlarmFrom__startAt(/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__AlarmFrom__size_type t0, /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__AlarmFrom__size_type dt){
#line 103
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Alarm__startAt(t0, dt);
#line 103
}
#line 103
# 40 "/home/csilab/upma/interfaces/AsyncSend.nc"
inline static error_t /*TDMASlotSenderC.TDMASlotSender.GenericSlotSenderP*/GenericSlotSenderP__0__SubSend__send(message_t * msg, uint8_t len){
#line 40
  unsigned char __nesc_result;
#line 40

#line 40
  __nesc_result = SenderDispatcherP__Send__send(TDMA_SLOT, msg, len);
#line 40

#line 40
  return __nesc_result;
#line 40
}
#line 40
# 68 "/home/csilab/upma/system/GenericSlotSenderP.nc"
static inline void /*TDMASlotSenderC.TDMASlotSender.GenericSlotSenderP*/GenericSlotSenderP__0__alarm(uint8_t at)
#line 68
{
}

#line 119
static inline void /*TDMASlotSenderC.TDMASlotSender.GenericSlotSenderP*/GenericSlotSenderP__0__Alarm__fired(void )
#line 119
{
  uint8_t oldState;

  /* atomic removed: atomic calls only */
#line 122
  oldState = /*TDMASlotSenderC.TDMASlotSender.GenericSlotSenderP*/GenericSlotSenderP__0__state;
  if (oldState != /*TDMASlotSenderC.TDMASlotSender.GenericSlotSenderP*/GenericSlotSenderP__0__S_START) {
      /*TDMASlotSenderC.TDMASlotSender.GenericSlotSenderP*/GenericSlotSenderP__0__alarm(2);
    }
  /* atomic removed: atomic calls only */
  /*TDMASlotSenderC.TDMASlotSender.GenericSlotSenderP*/GenericSlotSenderP__0__state = /*TDMASlotSenderC.TDMASlotSender.GenericSlotSenderP*/GenericSlotSenderP__0__S_OFFSET;
  if (/*TDMASlotSenderC.TDMASlotSender.GenericSlotSenderP*/GenericSlotSenderP__0__SubSend__send(/*TDMASlotSenderC.TDMASlotSender.GenericSlotSenderP*/GenericSlotSenderP__0__toSend, /*TDMASlotSenderC.TDMASlotSender.GenericSlotSenderP*/GenericSlotSenderP__0__toSendLen) != SUCCESS) {
      /* atomic removed: atomic calls only */
#line 129
      /*TDMASlotSenderC.TDMASlotSender.GenericSlotSenderP*/GenericSlotSenderP__0__state = /*TDMASlotSenderC.TDMASlotSender.GenericSlotSenderP*/GenericSlotSenderP__0__S_END;
    }
}

# 78 "/home/csilab/tinyos/tos/lib/timer/Alarm.nc"
inline static void /*TDMASlotSenderC.TDMASlotSender.Alarm32khz16C.Msp430Alarm*/Msp430AlarmC__2__Alarm__fired(void ){
#line 78
  /*TDMASlotSenderC.TDMASlotSender.GenericSlotSenderP*/GenericSlotSenderP__0__Alarm__fired();
#line 78
}
#line 78
# 135 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Control__disableEvents(void )
{
  * (volatile uint16_t * )392U &= ~0x0010;
}

# 58 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*TDMASlotSenderC.TDMASlotSender.Alarm32khz16C.Msp430Alarm*/Msp430AlarmC__2__Msp430TimerControl__disableEvents(void ){
#line 58
  /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Control__disableEvents();
#line 58
}
#line 58
# 70 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*TDMASlotSenderC.TDMASlotSender.Alarm32khz16C.Msp430Alarm*/Msp430AlarmC__2__Msp430Compare__fired(void )
{
  /*TDMASlotSenderC.TDMASlotSender.Alarm32khz16C.Msp430Alarm*/Msp430AlarmC__2__Msp430TimerControl__disableEvents();
  /*TDMASlotSenderC.TDMASlotSender.Alarm32khz16C.Msp430Alarm*/Msp430AlarmC__2__Alarm__fired();
}

# 45 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Compare__fired(void ){
#line 45
  /*TDMASlotSenderC.TDMASlotSender.Alarm32khz16C.Msp430Alarm*/Msp430AlarmC__2__Msp430Compare__fired();
#line 45
}
#line 45
# 150 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__getEvent(void )
{
  return * (volatile uint16_t * )408U;
}

#line 188
static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__default__captured(uint16_t n)
{
}

# 86 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__captured(uint16_t time){
#line 86
  /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__default__captured(time);
#line 86
}
#line 86
# 58 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__cc_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__int2CC(uint16_t x)
#line 58
{
#line 58
  union /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6____nesc_unnamed4408 {
#line 58
    uint16_t f;
#line 58
    /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__cc_t t;
  } 
#line 58
  c = { .f = x };

#line 58
  return c.t;
}

#line 85
static inline /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__cc_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Control__getControl(void )
{
  return /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__int2CC(* (volatile uint16_t * )392U);
}

#line 180
static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Event__fired(void )
{
  if (/*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Control__getControl().cap) {
    /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__captured(/*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__getEvent());
    }
  else {
#line 185
    /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Compare__fired();
    }
}

# 286 "/usr/local/lib/ncc/nesc_nx.h"
static __inline  uint8_t __nesc_hton_uint8(void * target, uint8_t value)
#line 286
{
  uint8_t *base = target;

#line 288
  base[0] = value;
  return value;
}

#line 303
static __inline  int8_t __nesc_hton_int8(void * target, int8_t value)
#line 303
{
#line 303
  __nesc_hton_uint8(target, value);
#line 303
  return value;
}

#line 327
static __inline  uint16_t __nesc_hton_leuint16(void * target, uint16_t value)
#line 327
{
  uint8_t *base = target;

#line 329
  base[0] = value;
  base[1] = value >> 8;
  return value;
}

#line 297
static __inline  uint8_t __nesc_hton_leuint8(void * target, uint8_t value)
#line 297
{
  uint8_t *base = target;

#line 299
  base[0] = value;
  return value;
}

# 768 "/home/csilab/upma/chips/cc2420/CC2420TransmitP.nc"
static inline bool CC2420TransmitP__defaultCca(void )
#line 768
{
  return TRUE;
}

# 291 "/home/csilab/upma/chips/cc2420/CC2420ActiveMessageP.nc"
static inline bool CC2420ActiveMessageP__CcaControl__default__getCca(am_id_t amId, message_t *msg, bool defaultValue)
#line 291
{
  return defaultValue;
}

# 61 "/home/csilab/upma/interfaces/CcaControl.nc"
inline static bool CC2420ActiveMessageP__CcaControl__getCca(am_id_t arg_0x7f7fec07e320, message_t *msg, bool defaultCca){
#line 61
  unsigned char __nesc_result;
#line 61

#line 61
    __nesc_result = CC2420ActiveMessageP__CcaControl__default__getCca(arg_0x7f7fec07e320, msg, defaultCca);
#line 61

#line 61
  return __nesc_result;
#line 61
}
#line 61
# 263 "/home/csilab/upma/chips/cc2420/CC2420ActiveMessageP.nc"
static inline bool CC2420ActiveMessageP__SubCcaControl__getCca(am_id_t amId, message_t *msg, bool defaultValue)
#line 263
{
  return CC2420ActiveMessageP__CcaControl__getCca(amId, msg, defaultValue);
}

# 151 "/home/csilab/upma/system/BeaconSlotP.nc"
static inline bool BeaconSlotP__SubCcaControl__getCca(message_t *msg, bool defaultCca)
#line 151
{
  return FALSE;
}

# 162 "/home/csilab/upma/system/GenericSlotSenderP.nc"
static inline bool /*TDMASlotSenderC.TDMASlotSender.GenericSlotSenderP*/GenericSlotSenderP__0__SubCcaControl__getCca(message_t *msg, bool defaultCca)
#line 162
{
  return 0;
}

# 132 "/home/csilab/upma/system/SenderDispatcherP.nc"
static inline bool SenderDispatcherP__SlotsCcaControl__default__getCca(am_id_t id, message_t *msg, bool defaultCca)
#line 132
{
  return defaultCca;
}

# 61 "/home/csilab/upma/interfaces/CcaControl.nc"
inline static bool SenderDispatcherP__SlotsCcaControl__getCca(uint8_t arg_0x7f7fec1925b0, message_t *msg, bool defaultCca){
#line 61
  unsigned char __nesc_result;
#line 61

#line 61
  switch (arg_0x7f7fec1925b0) {
#line 61
    case TDMA_SLOT:
#line 61
      __nesc_result = /*TDMASlotSenderC.TDMASlotSender.GenericSlotSenderP*/GenericSlotSenderP__0__SubCcaControl__getCca(msg, defaultCca);
#line 61
      break;
#line 61
    case BEACON_SLOT:
#line 61
      __nesc_result = BeaconSlotP__SubCcaControl__getCca(msg, defaultCca);
#line 61
      break;
#line 61
    default:
#line 61
      __nesc_result = SenderDispatcherP__SlotsCcaControl__default__getCca(arg_0x7f7fec1925b0, msg, defaultCca);
#line 61
      break;
#line 61
    }
#line 61

#line 61
  return __nesc_result;
#line 61
}
#line 61
# 146 "/home/csilab/upma/system/SenderDispatcherP.nc"
static inline bool SenderDispatcherP__SubCcaControl__getCca(am_id_t id, message_t *msg, bool defaultCca)
#line 146
{
  bool cca;
  uint8_t last_id_;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 150
    last_id_ = SenderDispatcherP__last_id;
#line 150
    __nesc_atomic_end(__nesc_atomic); }
  cca = SenderDispatcherP__SlotsCcaControl__getCca(last_id_, msg, defaultCca);

  return cca;
}

# 61 "/home/csilab/upma/interfaces/CcaControl.nc"
inline static bool CC2420TransmitP__CcaControl__getCca(am_id_t arg_0x7f7fec8771d0, message_t *msg, bool defaultCca){
#line 61
  unsigned char __nesc_result;
#line 61

#line 61
  __nesc_result = SenderDispatcherP__SubCcaControl__getCca(arg_0x7f7fec8771d0, msg, defaultCca);
#line 61
  __nesc_result = CC2420ActiveMessageP__SubCcaControl__getCca(arg_0x7f7fec8771d0, msg, defaultCca);
#line 61

#line 61
  return __nesc_result;
#line 61
}
#line 61
# 547 "/home/csilab/upma/chips/cc2420/CC2420TransmitP.nc"
static inline error_t CC2420TransmitP__send(message_t * p_msg, bool cca)
#line 547
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 548
    {
      if (CC2420TransmitP__m_state == CC2420TransmitP__S_CANCEL) {
          {
            unsigned char __nesc_temp = 
#line 550
            ECANCEL;

            {
#line 550
              __nesc_atomic_end(__nesc_atomic); 
#line 550
              return __nesc_temp;
            }
          }
        }
#line 553
      if (CC2420TransmitP__m_state != CC2420TransmitP__S_STARTED) {
          {
            unsigned char __nesc_temp = 
#line 554
            FAIL;

            {
#line 554
              __nesc_atomic_end(__nesc_atomic); 
#line 554
              return __nesc_temp;
            }
          }
        }
#line 557
      CC2420TransmitP__m_state = CC2420TransmitP__S_LOAD;
      CC2420TransmitP__m_cca = cca;
      CC2420TransmitP__m_msg = p_msg;
      CC2420TransmitP__totalCcaChecks = 0;
    }
#line 561
    __nesc_atomic_end(__nesc_atomic); }

  if (CC2420TransmitP__acquireSpiResource() == SUCCESS) {
      CC2420TransmitP__loadTXFIFO();
    }

  return SUCCESS;
}

#line 216
static inline error_t CC2420TransmitP__Send__send(message_t * p_msg)
#line 216
{
  return CC2420TransmitP__send(p_msg, CC2420TransmitP__CcaControl__getCca(CC2420TransmitP__getType(p_msg), p_msg, CC2420TransmitP__defaultCca()));
}

# 76 "/home/csilab/upma/chips/cc2420/CC2420Transmit.nc"
inline static error_t CC2420CsmaP__CC2420Transmit__send(message_t * p_msg){
#line 76
  unsigned char __nesc_result;
#line 76

#line 76
  __nesc_result = CC2420TransmitP__Send__send(p_msg);
#line 76

#line 76
  return __nesc_result;
#line 76
}
#line 76
# 111 "/home/csilab/tinyos/tos/system/StateImplP.nc"
static inline void StateImplP__State__forceState(uint8_t id, uint8_t reqState)
#line 111
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 112
    StateImplP__state[id] = reqState;
#line 112
    __nesc_atomic_end(__nesc_atomic); }
}

# 51 "/home/csilab/tinyos/tos/interfaces/State.nc"
inline static void CC2420CsmaP__SplitControlState__forceState(uint8_t reqState){
#line 51
  StateImplP__State__forceState(1U, reqState);
#line 51
}
#line 51
#line 66
inline static bool CC2420CsmaP__SplitControlState__isState(uint8_t myState){
#line 66
  unsigned char __nesc_result;
#line 66

#line 66
  __nesc_result = StateImplP__State__isState(1U, myState);
#line 66

#line 66
  return __nesc_result;
#line 66
}
#line 66
# 53 "/home/csilab/tinyos/tos/chips/cc2420/interfaces/CC2420PacketBody.nc"
inline static cc2420_metadata_t * CC2420CsmaP__CC2420PacketBody__getMetadata(message_t * msg){
#line 53
  nx_struct cc2420_metadata_t *__nesc_result;
#line 53

#line 53
  __nesc_result = CC2420PacketP__CC2420PacketBody__getMetadata(msg);
#line 53

#line 53
  return __nesc_result;
#line 53
}
#line 53
#line 42
inline static cc2420_header_t * CC2420CsmaP__CC2420PacketBody__getHeader(message_t * msg){
#line 42
  nx_struct cc2420_header_t *__nesc_result;
#line 42

#line 42
  __nesc_result = CC2420PacketP__CC2420PacketBody__getHeader(msg);
#line 42

#line 42
  return __nesc_result;
#line 42
}
#line 42
# 143 "/home/csilab/upma/chips/cc2420/CC2420CsmaP.nc"
static inline error_t CC2420CsmaP__Send__send(message_t *p_msg, uint8_t len)
#line 143
{
  unsigned char *__nesc_temp43;
  unsigned char *__nesc_temp42;
#line 145
  cc2420_header_t *header = CC2420CsmaP__CC2420PacketBody__getHeader(p_msg);
  cc2420_metadata_t *metadata = CC2420CsmaP__CC2420PacketBody__getMetadata(p_msg);

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 148
    {
      if (!CC2420CsmaP__SplitControlState__isState(CC2420CsmaP__S_STARTED)) {
          {
            unsigned char __nesc_temp = 
#line 150
            FAIL;

            {
#line 150
              __nesc_atomic_end(__nesc_atomic); 
#line 150
              return __nesc_temp;
            }
          }
        }
#line 153
      CC2420CsmaP__SplitControlState__forceState(CC2420CsmaP__S_TRANSMITTING);
      CC2420CsmaP__m_msg = p_msg;
    }
#line 155
    __nesc_atomic_end(__nesc_atomic); }

  __nesc_hton_leuint8(header->length.nxdata, len + CC2420_SIZE);



  (__nesc_temp42 = header->fcf.nxdata, __nesc_hton_leuint16(__nesc_temp42, __nesc_ntoh_leuint16(__nesc_temp42) & (1 << IEEE154_FCF_ACK_REQ)));

  (__nesc_temp43 = header->fcf.nxdata, __nesc_hton_leuint16(__nesc_temp43, __nesc_ntoh_leuint16(__nesc_temp43) | ((((IEEE154_TYPE_DATA << IEEE154_FCF_FRAME_TYPE) | (
  1 << IEEE154_FCF_INTRAPAN)) | (
  IEEE154_ADDR_SHORT << IEEE154_FCF_DEST_ADDR_MODE)) | (
  IEEE154_ADDR_SHORT << IEEE154_FCF_SRC_ADDR_MODE))));

  __nesc_hton_int8(metadata->ack.nxdata, FALSE);
  __nesc_hton_uint8(metadata->rssi.nxdata, 0);
  __nesc_hton_uint8(metadata->lqi.nxdata, 0);

  __nesc_hton_uint32(metadata->timestamp.nxdata, CC2420_INVALID_TIMESTAMP);

  CC2420CsmaP__CC2420Transmit__send(p_msg);
  return SUCCESS;
}

# 40 "/home/csilab/upma/interfaces/AsyncSend.nc"
inline static error_t SenderDispatcherP__SubSend__send(message_t * msg, uint8_t len){
#line 40
  unsigned char __nesc_result;
#line 40

#line 40
  __nesc_result = CC2420CsmaP__Send__send(msg, len);
#line 40

#line 40
  return __nesc_result;
#line 40
}
#line 40
# 70 "/home/csilab/tinyos/tos/interfaces/SpiPacket.nc"
inline static error_t CC2420SpiP__SpiPacket__send(uint8_t * txBuf, uint8_t * rxBuf, uint16_t len){
#line 70
  unsigned char __nesc_result;
#line 70

#line 70
  __nesc_result = /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__SpiPacket__send(/*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430Spi0C__0__CLIENT_ID, txBuf, rxBuf, len);
#line 70

#line 70
  return __nesc_result;
#line 70
}
#line 70
# 45 "/home/csilab/tinyos/tos/interfaces/SpiByte.nc"
inline static uint8_t CC2420SpiP__SpiByte__write(uint8_t tx){
#line 45
  unsigned char __nesc_result;
#line 45

#line 45
  __nesc_result = /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__SpiByte__write(tx);
#line 45

#line 45
  return __nesc_result;
#line 45
}
#line 45
# 126 "/home/csilab/tinyos/tos/system/StateImplP.nc"
static inline bool StateImplP__State__isIdle(uint8_t id)
#line 126
{
  return StateImplP__State__isState(id, StateImplP__S_IDLE);
}

# 61 "/home/csilab/tinyos/tos/interfaces/State.nc"
inline static bool CC2420SpiP__WorkingState__isIdle(void ){
#line 61
  unsigned char __nesc_result;
#line 61

#line 61
  __nesc_result = StateImplP__State__isIdle(0U);
#line 61

#line 61
  return __nesc_result;
#line 61
}
#line 61
# 214 "/home/csilab/tinyos/tos/chips/cc2420/spi/CC2420SpiP.nc"
static inline cc2420_status_t CC2420SpiP__Fifo__write(uint8_t addr, uint8_t *data, 
uint8_t len)
#line 215
{

  uint8_t status = 0;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 219
    {
      if (CC2420SpiP__WorkingState__isIdle()) {
          {
            unsigned char __nesc_temp = 
#line 221
            status;

            {
#line 221
              __nesc_atomic_end(__nesc_atomic); 
#line 221
              return __nesc_temp;
            }
          }
        }
    }
#line 225
    __nesc_atomic_end(__nesc_atomic); }
#line 225
  CC2420SpiP__m_addr = addr;

  status = CC2420SpiP__SpiByte__write(CC2420SpiP__m_addr);
  CC2420SpiP__SpiPacket__send(data, (void *)0, len);

  return status;
}

# 82 "/home/csilab/tinyos/tos/chips/cc2420/interfaces/CC2420Fifo.nc"
inline static cc2420_status_t CC2420TransmitP__TXFIFO__write(uint8_t * data, uint8_t length){
#line 82
  unsigned char __nesc_result;
#line 82

#line 82
  __nesc_result = CC2420SpiP__Fifo__write(CC2420_TXFIFO, data, length);
#line 82

#line 82
  return __nesc_result;
#line 82
}
#line 82
# 361 "/home/csilab/tinyos/tos/chips/msp430/usart/HplMsp430Usart0P.nc"
static inline void HplMsp430Usart0P__Usart__enableRxIntr(void )
#line 361
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 362
    {
      HplMsp430Usart0P__IFG1 &= ~0x40;
      HplMsp430Usart0P__IE1 |= 0x40;
    }
#line 365
    __nesc_atomic_end(__nesc_atomic); }
}

# 180 "/home/csilab/tinyos/tos/chips/msp430/usart/HplMsp430Usart.nc"
inline static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__enableRxIntr(void ){
#line 180
  HplMsp430Usart0P__Usart__enableRxIntr();
#line 180
}
#line 180
# 67 "/home/csilab/tinyos/tos/interfaces/TaskBasic.nc"
inline static error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__signalDone_task__postTask(void ){
#line 67
  unsigned char __nesc_result;
#line 67

#line 67
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(/*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__signalDone_task);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 78 "/home/csilab/tinyos/tos/lib/timer/Alarm.nc"
inline static void /*GenericSlotterC.Alarm32khz32C.Transform*/TransformAlarmC__2__Alarm__fired(void ){
#line 78
  GenericSlotterP__Alarm__fired();
#line 78
}
#line 78
# 162 "/home/csilab/tinyos/tos/lib/timer/TransformAlarmC.nc"
static inline void /*GenericSlotterC.Alarm32khz32C.Transform*/TransformAlarmC__2__AlarmFrom__fired(void )
{
  /* atomic removed: atomic calls only */
  {
    if (/*GenericSlotterC.Alarm32khz32C.Transform*/TransformAlarmC__2__m_dt == 0) 
      {
        /*GenericSlotterC.Alarm32khz32C.Transform*/TransformAlarmC__2__Alarm__fired();
      }
    else 
      {
        /*GenericSlotterC.Alarm32khz32C.Transform*/TransformAlarmC__2__set_alarm();
      }
  }
}

# 78 "/home/csilab/tinyos/tos/lib/timer/Alarm.nc"
inline static void /*GenericSlotterC.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__3__Alarm__fired(void ){
#line 78
  /*GenericSlotterC.Alarm32khz32C.Transform*/TransformAlarmC__2__AlarmFrom__fired();
#line 78
}
#line 78
# 135 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Control__disableEvents(void )
{
  * (volatile uint16_t * )394U &= ~0x0010;
}

# 58 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*GenericSlotterC.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__3__Msp430TimerControl__disableEvents(void ){
#line 58
  /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Control__disableEvents();
#line 58
}
#line 58
# 70 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*GenericSlotterC.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__3__Msp430Compare__fired(void )
{
  /*GenericSlotterC.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__3__Msp430TimerControl__disableEvents();
  /*GenericSlotterC.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__3__Alarm__fired();
}

# 45 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Compare__fired(void ){
#line 45
  /*GenericSlotterC.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__3__Msp430Compare__fired();
#line 45
}
#line 45
# 150 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__getEvent(void )
{
  return * (volatile uint16_t * )410U;
}

#line 188
static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__default__captured(uint16_t n)
{
}

# 86 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__captured(uint16_t time){
#line 86
  /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__default__captured(time);
#line 86
}
#line 86
# 58 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__cc_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__int2CC(uint16_t x)
#line 58
{
#line 58
  union /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7____nesc_unnamed4409 {
#line 58
    uint16_t f;
#line 58
    /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__cc_t t;
  } 
#line 58
  c = { .f = x };

#line 58
  return c.t;
}

#line 85
static inline /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__cc_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Control__getControl(void )
{
  return /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__int2CC(* (volatile uint16_t * )394U);
}

#line 180
static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Event__fired(void )
{
  if (/*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Control__getControl().cap) {
    /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__captured(/*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__getEvent());
    }
  else {
#line 185
    /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Compare__fired();
    }
}

# 310 "/usr/local/lib/ncc/nesc_nx.h"
static __inline  uint16_t __nesc_ntoh_uint16(const void * source)
#line 310
{
  const uint8_t *base = source;

#line 312
  return ((uint16_t )base[0] << 8) | base[1];
}

# 71 "/home/csilab/upma/interfaces/AsyncSend.nc"
inline static void * SenderDispatcherP__SubSend__getPayload(message_t * msg, uint8_t len){
#line 71
  void *__nesc_result;
#line 71

#line 71
  __nesc_result = CC2420CsmaP__Send__getPayload(msg, len);
#line 71

#line 71
  return __nesc_result;
#line 71
}
#line 71
# 94 "/home/csilab/upma/system/SenderDispatcherP.nc"
static inline void *SenderDispatcherP__Send__getPayload(uint8_t slotType, message_t *msg, uint8_t len)
#line 94
{
  return SenderDispatcherP__SubSend__getPayload(msg, len);
}

# 71 "/home/csilab/upma/interfaces/AsyncSend.nc"
inline static void * /*TDMASlotSenderC.TDMASlotSender.GenericSlotSenderP*/GenericSlotSenderP__0__SubSend__getPayload(message_t * msg, uint8_t len){
#line 71
  void *__nesc_result;
#line 71

#line 71
  __nesc_result = SenderDispatcherP__Send__getPayload(TDMA_SLOT, msg, len);
#line 71

#line 71
  return __nesc_result;
#line 71
}
#line 71
# 146 "/home/csilab/upma/system/GenericSlotSenderP.nc"
static inline void */*TDMASlotSenderC.TDMASlotSender.GenericSlotSenderP*/GenericSlotSenderP__0__Send__getPayload(message_t *msg, uint8_t len)
#line 146
{
  return /*TDMASlotSenderC.TDMASlotSender.GenericSlotSenderP*/GenericSlotSenderP__0__SubSend__getPayload(msg, len);
}

# 71 "/home/csilab/upma/interfaces/AsyncSend.nc"
inline static void * PureTDMASchedulerP__SubSend__getPayload(message_t * msg, uint8_t len){
#line 71
  void *__nesc_result;
#line 71

#line 71
  __nesc_result = /*TDMASlotSenderC.TDMASlotSender.GenericSlotSenderP*/GenericSlotSenderP__0__Send__getPayload(msg, len);
#line 71

#line 71
  return __nesc_result;
#line 71
}
#line 71
# 73 "/home/csilab/tinyos/tos/interfaces/Queue.nc"
inline static PureTDMASchedulerP__LOforwardQ__t  PureTDMASchedulerP__LOforwardQ__head(void ){
#line 73
  nx_struct adaptivehart_msg *__nesc_result;
#line 73

#line 73
  __nesc_result = /*PureTDMASchedulerC.LOQueueC*/QueueC__2__Queue__head();
#line 73

#line 73
  return __nesc_result;
#line 73
}
#line 73
#line 50
inline static bool PureTDMASchedulerP__LOforwardQ__empty(void ){
#line 50
  unsigned char __nesc_result;
#line 50

#line 50
  __nesc_result = /*PureTDMASchedulerC.LOQueueC*/QueueC__2__Queue__empty();
#line 50

#line 50
  return __nesc_result;
#line 50
}
#line 50
#line 73
inline static PureTDMASchedulerP__HIforwardQ__t  PureTDMASchedulerP__HIforwardQ__head(void ){
#line 73
  nx_struct adaptivehart_msg *__nesc_result;
#line 73

#line 73
  __nesc_result = /*PureTDMASchedulerC.HIQueueC*/QueueC__1__Queue__head();
#line 73

#line 73
  return __nesc_result;
#line 73
}
#line 73
#line 50
inline static bool PureTDMASchedulerP__HIforwardQ__empty(void ){
#line 50
  unsigned char __nesc_result;
#line 50

#line 50
  __nesc_result = /*PureTDMASchedulerC.HIQueueC*/QueueC__1__Queue__empty();
#line 50

#line 50
  return __nesc_result;
#line 50
}
#line 50
# 829 "/home/csilab/upma/lib/macs/pure-tdma/PureTDMASchedulerP.nc"
static inline void PureTDMASchedulerP__WH_send(uint8_t slot)
#line 829
{
  adaptivehart_msg_t *Q_payload = (adaptivehart_msg_t *)(void *)0;

  for (PureTDMASchedulerP__i = 0; PureTDMASchedulerP__i < PureTDMASchedulerP__schedule_len; PureTDMASchedulerP__i++) {
      if (PureTDMASchedulerP__schedule[PureTDMASchedulerP__i][PureTDMASchedulerP__SLOT] == slot) {
          if (PureTDMASchedulerP__schedule[PureTDMASchedulerP__i][PureTDMASchedulerP__SENDER] == TOS_NODE_ID) {
              if (PureTDMASchedulerP__is_root == TRUE) {
                  PureTDMASchedulerP__AH_payload = (adaptivehart_msg_t *)PureTDMASchedulerP__SubSend__getPayload(&PureTDMASchedulerP__AH_pkt, sizeof(adaptivehart_msg_t ));
                  PureTDMASchedulerP__set_payload(PureTDMASchedulerP__AH_payload, PureTDMASchedulerP__schedule[PureTDMASchedulerP__i][PureTDMASchedulerP__FLOWID], PureTDMASchedulerP__schedule[PureTDMASchedulerP__i][PureTDMASchedulerP__FLOWID], 0, 0, PureTDMASchedulerP__giga_seq);
                  PureTDMASchedulerP__send_AH_Pkt(&PureTDMASchedulerP__AH_pkt, PureTDMASchedulerP__schedule[PureTDMASchedulerP__i][PureTDMASchedulerP__RECEIVER], PureTDMASchedulerP__schedule[PureTDMASchedulerP__i][PureTDMASchedulerP__FLOWID]);
                }
              else 
#line 839
                {
                  if (PureTDMASchedulerP__schedule[PureTDMASchedulerP__i][PureTDMASchedulerP__FLOWID] == 1) {
                      if (!PureTDMASchedulerP__HIforwardQ__empty()) {
                          Q_payload = (adaptivehart_msg_t *)PureTDMASchedulerP__HIforwardQ__head();
                        }
                      else 
#line 843
                        {
                          return;
                        }
                    }
                  else {
#line 846
                    if (PureTDMASchedulerP__schedule[PureTDMASchedulerP__i][PureTDMASchedulerP__FLOWID] == 2) {
                        if (!PureTDMASchedulerP__LOforwardQ__empty()) {
                            Q_payload = (adaptivehart_msg_t *)PureTDMASchedulerP__LOforwardQ__head();
                          }
                        else 
#line 849
                          {
                            return;
                          }
                      }
                    }
#line 853
                  PureTDMASchedulerP__AH_payload = (adaptivehart_msg_t *)PureTDMASchedulerP__SubSend__getPayload(&PureTDMASchedulerP__AH_pkt, sizeof(adaptivehart_msg_t ));
                  PureTDMASchedulerP__set_payload(PureTDMASchedulerP__AH_payload, __nesc_ntoh_uint8(Q_payload->flowid.nxdata), __nesc_ntoh_uint8(Q_payload->priority.nxdata), __nesc_ntoh_uint8(Q_payload->maxtx.nxdata), __nesc_ntoh_uint16(Q_payload->deadline.nxdata), __nesc_ntoh_uint16(Q_payload->seq.nxdata));
                  PureTDMASchedulerP__send_AH_Pkt(&PureTDMASchedulerP__AH_pkt, PureTDMASchedulerP__schedule[PureTDMASchedulerP__i][PureTDMASchedulerP__RECEIVER], PureTDMASchedulerP__schedule[PureTDMASchedulerP__i][PureTDMASchedulerP__FLOWID]);
                }
            }
        }
    }
}

# 40 "/home/csilab/upma/interfaces/AsyncSend.nc"
inline static error_t BeaconSlotP__SubSend__send(message_t * msg, uint8_t len){
#line 40
  unsigned char __nesc_result;
#line 40

#line 40
  __nesc_result = SenderDispatcherP__Send__send(BEACON_SLOT, msg, len);
#line 40

#line 40
  return __nesc_result;
#line 40
}
#line 40
# 57 "/home/csilab/upma/system/BeaconSlotP.nc"
static inline void BeaconSlotP__write_timestamp(uint8_t *payload, uint32_t timestamp)
{
  nx_uint32_t *ptr = (nx_uint32_t *)payload;

#line 60
  __nesc_hton_uint32((*ptr).nxdata, timestamp);
}

# 64 "/home/csilab/tinyos/tos/lib/timer/Counter.nc"
inline static /*GenericSlotterC.Alarm32khz32C.Transform*/TransformAlarmC__2__Counter__size_type /*GenericSlotterC.Alarm32khz32C.Transform*/TransformAlarmC__2__Counter__get(void ){
#line 64
  unsigned long __nesc_result;
#line 64

#line 64
  __nesc_result = /*Counter32khz32C.Transform*/TransformCounterC__0__Counter__get();
#line 64

#line 64
  return __nesc_result;
#line 64
}
#line 64
# 86 "/home/csilab/tinyos/tos/lib/timer/TransformAlarmC.nc"
static inline /*GenericSlotterC.Alarm32khz32C.Transform*/TransformAlarmC__2__to_size_type /*GenericSlotterC.Alarm32khz32C.Transform*/TransformAlarmC__2__Alarm__getNow(void )
{
  return /*GenericSlotterC.Alarm32khz32C.Transform*/TransformAlarmC__2__Counter__get();
}

# 109 "/home/csilab/tinyos/tos/lib/timer/Alarm.nc"
inline static GenericSlotterP__Alarm__size_type GenericSlotterP__Alarm__getNow(void ){
#line 109
  unsigned long __nesc_result;
#line 109

#line 109
  __nesc_result = /*GenericSlotterC.Alarm32khz32C.Transform*/TransformAlarmC__2__Alarm__getNow();
#line 109

#line 109
  return __nesc_result;
#line 109
}
#line 109
# 123 "/home/csilab/upma/system/GenericSlotterP.nc"
static inline uint32_t GenericSlotterP__SlotterControl__getRemaining(void )
#line 123
{
  uint32_t now;
#line 124
  uint32_t diff;

#line 125
  now = GenericSlotterP__Alarm__getNow();
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 126
    {
      if (GenericSlotterP__nextFire > now) {
#line 127
        diff = GenericSlotterP__nextFire - now;
        }
      else {
#line 128
        diff = 0;
        }
    }
#line 130
    __nesc_atomic_end(__nesc_atomic); }
  return diff;
}

# 36 "/home/csilab/upma/interfaces/SlotterControl.nc"
inline static uint32_t BeaconSlotP__SlotterControl__getRemaining(void ){
#line 36
  unsigned long __nesc_result;
#line 36

#line 36
  __nesc_result = GenericSlotterP__SlotterControl__getRemaining();
#line 36

#line 36
  return __nesc_result;
#line 36
}
#line 36
# 71 "/home/csilab/upma/interfaces/AsyncSend.nc"
inline static void * BeaconSlotP__SubSend__getPayload(message_t * msg, uint8_t len){
#line 71
  void *__nesc_result;
#line 71

#line 71
  __nesc_result = SenderDispatcherP__Send__getPayload(BEACON_SLOT, msg, len);
#line 71

#line 71
  return __nesc_result;
#line 71
}
#line 71
# 78 "/home/csilab/upma/system/BeaconSlotP.nc"
static inline error_t BeaconSlotP__Send__send(message_t *msg, uint8_t len)
#line 78
{
  uint8_t *payload;
  error_t status;
  uint32_t remaining;

  payload = BeaconSlotP__SubSend__getPayload(&BeaconSlotP__syncPkt, len);

  remaining = BeaconSlotP__SlotterControl__getRemaining();
  BeaconSlotP__write_timestamp(payload, remaining);

  status = BeaconSlotP__SubSend__send(&BeaconSlotP__syncPkt, BeaconSlotP__syncPktLen);
  return status;
}

# 40 "/home/csilab/upma/interfaces/AsyncSend.nc"
inline static error_t PureTDMASchedulerP__BeaconSend__send(message_t * msg, uint8_t len){
#line 40
  unsigned char __nesc_result;
#line 40

#line 40
  __nesc_result = BeaconSlotP__Send__send(msg, len);
#line 40

#line 40
  return __nesc_result;
#line 40
}
#line 40
# 396 "/home/csilab/upma/lib/macs/pure-tdma/PureTDMASchedulerP.nc"
static inline void PureTDMASchedulerP__Slotter__slot(uint8_t slot)
#line 396
{
  uint8_t tmp_slot;
  int16_t flow_start_slot = -1;
  adaptivehart_msg_t *Q_payload = (adaptivehart_msg_t *)(void *)0;
  uint8_t tmp_tx_opper;
  uint8_t tmp_flowid = 0;

  tmp_slot = slot;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 405
    PureTDMASchedulerP__currentSlot = tmp_slot;
#line 405
    __nesc_atomic_end(__nesc_atomic); }

  if (tmp_slot == PureTDMASchedulerP__gigaframe_length - 1) {
      if (PureTDMASchedulerP__is_root == TRUE) {
          printf("random_slot: %d, Tx: [ %d, %d, %d, %d, %d, %d, %d, %d, %d, %d ]\r\n", PureTDMASchedulerP__random_slot, PureTDMASchedulerP__tx_slots[0], PureTDMASchedulerP__tx_slots[1], PureTDMASchedulerP__tx_slots[2], PureTDMASchedulerP__tx_slots[3], PureTDMASchedulerP__tx_slots[4], PureTDMASchedulerP__tx_slots[5], PureTDMASchedulerP__tx_slots[6], PureTDMASchedulerP__tx_slots[7], PureTDMASchedulerP__tx_slots[8], PureTDMASchedulerP__tx_slots[9]);
          PureTDMASchedulerP__tx_slots[0] = 0;
#line 410
          PureTDMASchedulerP__tx_slots[1] = 0;
#line 410
          PureTDMASchedulerP__tx_slots[2] = 0;
#line 410
          PureTDMASchedulerP__tx_slots[3] = 0;
#line 410
          PureTDMASchedulerP__tx_slots[4] = 0;
#line 410
          PureTDMASchedulerP__tx_slots[5] = 0;
#line 410
          PureTDMASchedulerP__tx_slots[6] = 0;
#line 410
          PureTDMASchedulerP__tx_slots[7] = 0;
#line 410
          PureTDMASchedulerP__tx_slots[8] = 0;
#line 410
          PureTDMASchedulerP__tx_slots[9] = 0;
          PureTDMASchedulerP__tx_flow_seq = 0;
        }
      else 
#line 412
        {
          printf("HI_TASK: Rx: [ %d, %d, %d, %d, %d ]\r\n", PureTDMASchedulerP__rcv_slots[1][0], PureTDMASchedulerP__rcv_slots[1][1], PureTDMASchedulerP__rcv_slots[1][2], PureTDMASchedulerP__rcv_slots[1][3], PureTDMASchedulerP__rcv_slots[1][4]);
          printf("LO_TASK: Rx: [ %d, %d, %d, %d, %d ]\r\n", PureTDMASchedulerP__rcv_slots[2][0], PureTDMASchedulerP__rcv_slots[2][1], PureTDMASchedulerP__rcv_slots[2][2], PureTDMASchedulerP__rcv_slots[2][3], PureTDMASchedulerP__rcv_slots[2][4]);
          PureTDMASchedulerP__rcv_slots[1][0] = 0;
#line 415
          PureTDMASchedulerP__rcv_slots[1][1] = 0;
#line 415
          PureTDMASchedulerP__rcv_slots[1][2] = 0;
#line 415
          PureTDMASchedulerP__rcv_slots[1][3] = 0;
#line 415
          PureTDMASchedulerP__rcv_slots[1][4] = 0;
          PureTDMASchedulerP__rcv_slots[2][0] = 0;
#line 416
          PureTDMASchedulerP__rcv_slots[2][1] = 0;
#line 416
          PureTDMASchedulerP__rcv_slots[2][2] = 0;
#line 416
          PureTDMASchedulerP__rcv_slots[2][3] = 0;
#line 416
          PureTDMASchedulerP__rcv_slots[2][4] = 0;
          PureTDMASchedulerP__rx_flow_seq[1] = 0;
          PureTDMASchedulerP__rx_flow_seq[2] = 0;
        }
      printfflush();
      return;
    }

  if (tmp_slot == 0) {

      if (PureTDMASchedulerP__coordinatorId == TOS_NODE_ID) {
          PureTDMASchedulerP__BeaconSend__send((void *)0, 0);
        }
      return;
    }
  else {
#line 430
    if (tmp_slot == 1) {

        if (TOS_NODE_ID != PureTDMASchedulerP__coordinatorId) {


            printf("<New Frame:%ld>\r\n", PureTDMASchedulerP__giga_seq);
            PureTDMASchedulerP__giga_seq += 1;
            PureTDMASchedulerP__reset_var(1);
            PureTDMASchedulerP__reset_var(2);
#line 474
            { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 474
              PureTDMASchedulerP__phyLock = FALSE;
#line 474
              __nesc_atomic_end(__nesc_atomic); }
          }
        return;
      }
    }







  if (tmp_slot >= 1) {
      PureTDMASchedulerP__WH_send(tmp_slot);
    }
}

# 31 "/home/csilab/upma/interfaces/Slotter.nc"
inline static void GenericSlotterP__Slotter__slot(uint8_t num){
#line 31
  PureTDMASchedulerP__Slotter__slot(num);
#line 31
}
#line 31
# 53 "/home/csilab/tinyos/tos/system/QueueC.nc"
static inline bool /*PrintfC.QueueC*/QueueC__0__Queue__empty(void )
#line 53
{
  return /*PrintfC.QueueC*/QueueC__0__size == 0;
}

# 50 "/home/csilab/tinyos/tos/interfaces/Queue.nc"
inline static bool PrintfP__Queue__empty(void ){
#line 50
  unsigned char __nesc_result;
#line 50

#line 50
  __nesc_result = /*PrintfC.QueueC*/QueueC__0__Queue__empty();
#line 50

#line 50
  return __nesc_result;
#line 50
}
#line 50
# 126 "/home/csilab/tinyos/tos/interfaces/Packet.nc"
inline static void * PrintfP__Packet__getPayload(message_t * msg, uint8_t len){
#line 126
  void *__nesc_result;
#line 126

#line 126
  __nesc_result = /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__Packet__getPayload(msg, len);
#line 126

#line 126
  return __nesc_result;
#line 126
}
#line 126
# 115 "/home/csilab/upma/lib/serial/SerialActiveMessageP.nc"
static inline uint8_t /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__Packet__maxPayloadLength(void )
#line 115
{
  return 28;
}

# 69 "/home/csilab/tinyos/tos/system/QueueC.nc"
static inline void /*PrintfC.QueueC*/QueueC__0__printQueue(void )
#line 69
{
}

#line 65
static inline /*PrintfC.QueueC*/QueueC__0__queue_t /*PrintfC.QueueC*/QueueC__0__Queue__head(void )
#line 65
{
  return /*PrintfC.QueueC*/QueueC__0__queue[/*PrintfC.QueueC*/QueueC__0__head];
}

#line 85
static inline /*PrintfC.QueueC*/QueueC__0__queue_t /*PrintfC.QueueC*/QueueC__0__Queue__dequeue(void )
#line 85
{
  /*PrintfC.QueueC*/QueueC__0__queue_t t = /*PrintfC.QueueC*/QueueC__0__Queue__head();

#line 87
  ;
  if (!/*PrintfC.QueueC*/QueueC__0__Queue__empty()) {
      /*PrintfC.QueueC*/QueueC__0__head++;
      if (/*PrintfC.QueueC*/QueueC__0__head == 250) {
#line 90
        /*PrintfC.QueueC*/QueueC__0__head = 0;
        }
#line 91
      /*PrintfC.QueueC*/QueueC__0__size--;
      /*PrintfC.QueueC*/QueueC__0__printQueue();
    }
  return t;
}

# 81 "/home/csilab/tinyos/tos/interfaces/Queue.nc"
inline static PrintfP__Queue__t  PrintfP__Queue__dequeue(void ){
#line 81
  unsigned char __nesc_result;
#line 81

#line 81
  __nesc_result = /*PrintfC.QueueC*/QueueC__0__Queue__dequeue();
#line 81

#line 81
  return __nesc_result;
#line 81
}
#line 81
# 315 "/usr/local/lib/ncc/nesc_nx.h"
static __inline  uint16_t __nesc_hton_uint16(void * target, uint16_t value)
#line 315
{
  uint8_t *base = target;

#line 317
  base[1] = value;
  base[0] = value >> 8;
  return value;
}

# 49 "/home/csilab/upma/lib/serial/SerialActiveMessageP.nc"
static inline serial_header_t * /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__getHeader(message_t * msg)
#line 49
{
  return (serial_header_t * )((uint8_t *)msg + (unsigned short )& ((message_t *)0)->data - sizeof(serial_header_t ));
}

#line 142
static inline void /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMPacket__setDestination(message_t *amsg, am_addr_t addr)
#line 142
{
  serial_header_t *header = /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__getHeader(amsg);

#line 144
  __nesc_hton_uint16(header->dest.nxdata, addr);
}

# 92 "/home/csilab/upma/interfaces/AMPacket.nc"
inline static void /*PrintfC.SerialAMSenderC.AMQueueEntryP*/AMQueueEntryP__0__AMPacket__setDestination(message_t * amsg, am_addr_t addr){
#line 92
  /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMPacket__setDestination(amsg, addr);
#line 92
}
#line 92
# 161 "/home/csilab/upma/lib/serial/SerialActiveMessageP.nc"
static inline void /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMPacket__setType(message_t *amsg, am_id_t type)
#line 161
{
  serial_header_t *header = /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__getHeader(amsg);

#line 163
  __nesc_hton_uint8(header->type.nxdata, type);
}

# 151 "/home/csilab/upma/interfaces/AMPacket.nc"
inline static void /*PrintfC.SerialAMSenderC.AMQueueEntryP*/AMQueueEntryP__0__AMPacket__setType(message_t * amsg, am_id_t t){
#line 151
  /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMPacket__setType(amsg, t);
#line 151
}
#line 151
# 80 "/home/csilab/tinyos/tos/interfaces/AMSend.nc"
inline static error_t /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__AMSend__send(am_id_t arg_0x7f7fed176840, am_addr_t addr, message_t * msg, uint8_t len){
#line 80
  unsigned char __nesc_result;
#line 80

#line 80
  __nesc_result = /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMSend__send(arg_0x7f7fed176840, addr, msg, len);
#line 80

#line 80
  return __nesc_result;
#line 80
}
#line 80
# 67 "/home/csilab/upma/interfaces/AMPacket.nc"
inline static am_addr_t /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__AMPacket__destination(message_t * amsg){
#line 67
  unsigned int __nesc_result;
#line 67

#line 67
  __nesc_result = /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMPacket__destination(amsg);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
#line 136
inline static am_id_t /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__AMPacket__type(message_t * amsg){
#line 136
  unsigned char __nesc_result;
#line 136

#line 136
  __nesc_result = /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMPacket__type(amsg);
#line 136

#line 136
  return __nesc_result;
#line 136
}
#line 136
# 111 "/home/csilab/upma/lib/serial/SerialActiveMessageP.nc"
static inline void /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__Packet__setPayloadLength(message_t *msg, uint8_t len)
#line 111
{
  __nesc_hton_uint8(/*SerialActiveMessageC.AM*/SerialActiveMessageP__0__getHeader(msg)->length.nxdata, len);
}

# 94 "/home/csilab/tinyos/tos/interfaces/Packet.nc"
inline static void /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__Packet__setPayloadLength(message_t * msg, uint8_t len){
#line 94
  /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__Packet__setPayloadLength(msg, len);
#line 94
}
#line 94
# 90 "/home/csilab/tinyos/tos/system/AMQueueImplP.nc"
static inline error_t /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__Send__send(uint8_t clientId, message_t *msg, 
uint8_t len)
#line 91
{
  if (clientId >= 1) {
      return FAIL;
    }
  if (/*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__queue[clientId].msg != (void *)0) {
      return EBUSY;
    }
  ;

  /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__queue[clientId].msg = msg;
  /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__Packet__setPayloadLength(msg, len);

  if (/*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__current >= 1) {
      error_t err;
      am_id_t amId = /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__AMPacket__type(msg);
      am_addr_t dest = /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__AMPacket__destination(msg);

      ;
      /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__current = clientId;

      err = /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__AMSend__send(amId, dest, msg, len);
      if (err != SUCCESS) {
          ;
          /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__current = 1;
          /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__queue[clientId].msg = (void *)0;
        }

      return err;
    }
  else {
      ;
    }
  return SUCCESS;
}

# 75 "/home/csilab/tinyos/tos/interfaces/Send.nc"
inline static error_t /*PrintfC.SerialAMSenderC.AMQueueEntryP*/AMQueueEntryP__0__Send__send(message_t * msg, uint8_t len){
#line 75
  unsigned char __nesc_result;
#line 75

#line 75
  __nesc_result = /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__Send__send(0U, msg, len);
#line 75

#line 75
  return __nesc_result;
#line 75
}
#line 75
# 538 "/home/csilab/tinyos/tos/lib/serial/SerialP.nc"
static inline error_t SerialP__SendBytePacket__startSend(uint8_t b)
#line 538
{
  bool not_busy = FALSE;

#line 540
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 540
    {
      if (SerialP__txState == SerialP__TXSTATE_INACTIVE) 
        {
          unsigned char __nesc_temp = 
#line 542
          EOFF;

          {
#line 542
            __nesc_atomic_end(__nesc_atomic); 
#line 542
            return __nesc_temp;
          }
        }
    }
#line 545
    __nesc_atomic_end(__nesc_atomic); }
#line 544
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 544
    {
      if (SerialP__txBuf[SerialP__TX_DATA_INDEX].state == SerialP__BUFFER_AVAILABLE) {
          SerialP__txBuf[SerialP__TX_DATA_INDEX].state = SerialP__BUFFER_FILLING;
          SerialP__txBuf[SerialP__TX_DATA_INDEX].buf = b;
          not_busy = TRUE;
        }
    }
#line 550
    __nesc_atomic_end(__nesc_atomic); }
  if (not_busy) {
      SerialP__MaybeScheduleTx();
      return SUCCESS;
    }
  return EBUSY;
}

# 62 "/home/csilab/tinyos/tos/lib/serial/SendBytePacket.nc"
inline static error_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__SendBytePacket__startSend(uint8_t first_byte){
#line 62
  unsigned char __nesc_result;
#line 62

#line 62
  __nesc_result = SerialP__SendBytePacket__startSend(first_byte);
#line 62

#line 62
  return __nesc_result;
#line 62
}
#line 62
# 54 "/home/csilab/tinyos/tos/lib/serial/SerialPacketInfoActiveMessageP.nc"
static inline uint8_t SerialPacketInfoActiveMessageP__Info__dataLinkLength(message_t *msg, uint8_t upperLen)
#line 54
{
  return upperLen + sizeof(serial_header_t );
}

# 361 "/home/csilab/tinyos/tos/lib/serial/SerialDispatcherP.nc"
static inline uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__default__dataLinkLength(uart_id_t id, message_t *msg, 
uint8_t upperLen)
#line 362
{
  return 0;
}

# 23 "/home/csilab/tinyos/tos/lib/serial/SerialPacketInfo.nc"
inline static uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__dataLinkLength(uart_id_t arg_0x7f7fed95be30, message_t *msg, uint8_t upperLen){
#line 23
  unsigned char __nesc_result;
#line 23

#line 23
  switch (arg_0x7f7fed95be30) {
#line 23
    case TOS_SERIAL_ACTIVE_MESSAGE_ID:
#line 23
      __nesc_result = SerialPacketInfoActiveMessageP__Info__dataLinkLength(msg, upperLen);
#line 23
      break;
#line 23
    default:
#line 23
      __nesc_result = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__default__dataLinkLength(arg_0x7f7fed95be30, msg, upperLen);
#line 23
      break;
#line 23
    }
#line 23

#line 23
  return __nesc_result;
#line 23
}
#line 23
# 111 "/home/csilab/tinyos/tos/lib/serial/SerialDispatcherP.nc"
static inline error_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__Send__send(uint8_t id, message_t *msg, uint8_t len)
#line 111
{
  if (/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendState != /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__SEND_STATE_IDLE) {
      return EBUSY;
    }

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 116
    {
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendIndex = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__offset(id);
      if (/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendIndex > sizeof(message_header_t )) {
          {
            unsigned char __nesc_temp = 
#line 119
            ESIZE;

            {
#line 119
              __nesc_atomic_end(__nesc_atomic); 
#line 119
              return __nesc_temp;
            }
          }
        }
#line 122
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendError = SUCCESS;
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendBuffer = (uint8_t *)msg;
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendState = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__SEND_STATE_DATA;
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendId = id;
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendCancelled = FALSE;






      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendLen = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__dataLinkLength(id, msg, len) + /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendIndex;
    }
#line 134
    __nesc_atomic_end(__nesc_atomic); }
  if (/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__SendBytePacket__startSend(id) == SUCCESS) {
      return SUCCESS;
    }
  else {
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendState = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__SEND_STATE_IDLE;
      return FAIL;
    }
}

# 75 "/home/csilab/tinyos/tos/interfaces/Send.nc"
inline static error_t /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__SubSend__send(message_t * msg, uint8_t len){
#line 75
  unsigned char __nesc_result;
#line 75

#line 75
  __nesc_result = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__Send__send(TOS_SERIAL_ACTIVE_MESSAGE_ID, msg, len);
#line 75

#line 75
  return __nesc_result;
#line 75
}
#line 75
# 66 "/home/csilab/upma/chips/cc2420/CC2420PacketP.nc"
static inline error_t CC2420PacketP__Acks__requestAck(message_t *p_msg)
#line 66
{
  unsigned char *__nesc_temp46;

#line 67
  (__nesc_temp46 = CC2420PacketP__CC2420PacketBody__getHeader(p_msg)->fcf.nxdata, __nesc_hton_leuint16(__nesc_temp46, __nesc_ntoh_leuint16(__nesc_temp46) | (1 << IEEE154_FCF_ACK_REQ)));
  return SUCCESS;
}

# 59 "/home/csilab/tinyos/tos/interfaces/PacketAcknowledgements.nc"
inline static error_t PureTDMASchedulerP__ACK__requestAck(message_t * msg){
#line 59
  unsigned char __nesc_result;
#line 59

#line 59
  __nesc_result = CC2420PacketP__Acks__requestAck(msg);
#line 59

#line 59
  return __nesc_result;
#line 59
}
#line 59
# 42 "/home/csilab/tinyos/tos/chips/cc2420/interfaces/CC2420PacketBody.nc"
inline static cc2420_header_t * CC2420ActiveMessageP__CC2420PacketBody__getHeader(message_t * msg){
#line 42
  nx_struct cc2420_header_t *__nesc_result;
#line 42

#line 42
  __nesc_result = CC2420PacketP__CC2420PacketBody__getHeader(msg);
#line 42

#line 42
  return __nesc_result;
#line 42
}
#line 42
# 165 "/home/csilab/upma/chips/cc2420/CC2420ActiveMessageP.nc"
static inline void CC2420ActiveMessageP__AMPacket__setSource(message_t *amsg, am_addr_t addr)
#line 165
{
  cc2420_header_t *header = CC2420ActiveMessageP__CC2420PacketBody__getHeader(amsg);

#line 167
  __nesc_hton_leuint16(header->src.nxdata, addr);
}

# 110 "/home/csilab/upma/interfaces/AMPacket.nc"
inline static void PureTDMASchedulerP__AMPacket__setSource(message_t * amsg, am_addr_t addr){
#line 110
  CC2420ActiveMessageP__AMPacket__setSource(amsg, addr);
#line 110
}
#line 110
#line 92
inline static void PureTDMASchedulerP__AMPacket__setDestination(message_t * amsg, am_addr_t addr){
#line 92
  CC2420ActiveMessageP__AMPacket__setDestination(amsg, addr);
#line 92
}
#line 92
# 45 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430Timer.nc"
inline static uint16_t /*TDMASlotSenderC.TDMASlotSender.Alarm32khz16C.Msp430Alarm*/Msp430AlarmC__2__Msp430Timer__get(void ){
#line 45
  unsigned int __nesc_result;
#line 45

#line 45
  __nesc_result = /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__get();
#line 45

#line 45
  return __nesc_result;
#line 45
}
#line 45
# 104 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline uint16_t /*TDMASlotSenderC.TDMASlotSender.Alarm32khz16C.Msp430Alarm*/Msp430AlarmC__2__Alarm__getNow(void )
{
  return /*TDMASlotSenderC.TDMASlotSender.Alarm32khz16C.Msp430Alarm*/Msp430AlarmC__2__Msp430Timer__get();
}

# 130 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Control__enableEvents(void )
{
  * (volatile uint16_t * )392U |= 0x0010;
}

# 57 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*TDMASlotSenderC.TDMASlotSender.Alarm32khz16C.Msp430Alarm*/Msp430AlarmC__2__Msp430TimerControl__enableEvents(void ){
#line 57
  /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Control__enableEvents();
#line 57
}
#line 57
# 95 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Control__clearPendingInterrupt(void )
{
  * (volatile uint16_t * )392U &= ~0x0001;
}

# 44 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*TDMASlotSenderC.TDMASlotSender.Alarm32khz16C.Msp430Alarm*/Msp430AlarmC__2__Msp430TimerControl__clearPendingInterrupt(void ){
#line 44
  /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Control__clearPendingInterrupt();
#line 44
}
#line 44
# 155 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Compare__setEvent(uint16_t x)
{
  * (volatile uint16_t * )408U = x;
}

# 41 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*TDMASlotSenderC.TDMASlotSender.Alarm32khz16C.Msp430Alarm*/Msp430AlarmC__2__Msp430Compare__setEvent(uint16_t time){
#line 41
  /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Compare__setEvent(time);
#line 41
}
#line 41
# 45 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430Timer.nc"
inline static uint16_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Timer__get(void ){
#line 45
  unsigned int __nesc_result;
#line 45

#line 45
  __nesc_result = /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__get();
#line 45

#line 45
  return __nesc_result;
#line 45
}
#line 45
# 165 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Compare__setEventFromNow(uint16_t x)
{
  * (volatile uint16_t * )408U = /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Timer__get() + x;
}

# 43 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*TDMASlotSenderC.TDMASlotSender.Alarm32khz16C.Msp430Alarm*/Msp430AlarmC__2__Msp430Compare__setEventFromNow(uint16_t delta){
#line 43
  /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Compare__setEventFromNow(delta);
#line 43
}
#line 43
# 81 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*TDMASlotSenderC.TDMASlotSender.Alarm32khz16C.Msp430Alarm*/Msp430AlarmC__2__Alarm__startAt(uint16_t t0, uint16_t dt)
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      uint16_t now = /*TDMASlotSenderC.TDMASlotSender.Alarm32khz16C.Msp430Alarm*/Msp430AlarmC__2__Msp430Timer__get();
      uint16_t elapsed = now - t0;

#line 87
      if (elapsed >= dt) 
        {
          /*TDMASlotSenderC.TDMASlotSender.Alarm32khz16C.Msp430Alarm*/Msp430AlarmC__2__Msp430Compare__setEventFromNow(2);
        }
      else 
        {
          uint16_t remaining = dt - elapsed;

#line 94
          if (remaining <= 2) {
            /*TDMASlotSenderC.TDMASlotSender.Alarm32khz16C.Msp430Alarm*/Msp430AlarmC__2__Msp430Compare__setEventFromNow(2);
            }
          else {
#line 97
            /*TDMASlotSenderC.TDMASlotSender.Alarm32khz16C.Msp430Alarm*/Msp430AlarmC__2__Msp430Compare__setEvent(now + remaining);
            }
        }
#line 99
      /*TDMASlotSenderC.TDMASlotSender.Alarm32khz16C.Msp430Alarm*/Msp430AlarmC__2__Msp430TimerControl__clearPendingInterrupt();
      /*TDMASlotSenderC.TDMASlotSender.Alarm32khz16C.Msp430Alarm*/Msp430AlarmC__2__Msp430TimerControl__enableEvents();
    }
#line 101
    __nesc_atomic_end(__nesc_atomic); }
}

#line 60
static inline void /*TDMASlotSenderC.TDMASlotSender.Alarm32khz16C.Msp430Alarm*/Msp430AlarmC__2__Alarm__start(uint16_t dt)
{
  /*TDMASlotSenderC.TDMASlotSender.Alarm32khz16C.Msp430Alarm*/Msp430AlarmC__2__Alarm__startAt(/*TDMASlotSenderC.TDMASlotSender.Alarm32khz16C.Msp430Alarm*/Msp430AlarmC__2__Alarm__getNow(), dt);
}

# 66 "/home/csilab/tinyos/tos/lib/timer/Alarm.nc"
inline static void /*TDMASlotSenderC.TDMASlotSender.GenericSlotSenderP*/GenericSlotSenderP__0__Alarm__start(/*TDMASlotSenderC.TDMASlotSender.GenericSlotSenderP*/GenericSlotSenderP__0__Alarm__size_type dt){
#line 66
  /*TDMASlotSenderC.TDMASlotSender.Alarm32khz16C.Msp430Alarm*/Msp430AlarmC__2__Alarm__start(dt);
#line 66
}
#line 66
# 78 "/home/csilab/upma/system/GenericSlotSenderP.nc"
static inline error_t /*TDMASlotSenderC.TDMASlotSender.GenericSlotSenderP*/GenericSlotSenderP__0__Send__send(message_t *msg, uint8_t len)
#line 78
{
  error_t err;
  uint8_t state_;

  if (msg == (void *)0) {
#line 82
    return FAIL;
    }
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 84
    {
      state_ = /*TDMASlotSenderC.TDMASlotSender.GenericSlotSenderP*/GenericSlotSenderP__0__state;

      /*TDMASlotSenderC.TDMASlotSender.GenericSlotSenderP*/GenericSlotSenderP__0__toSend = msg;
      /*TDMASlotSenderC.TDMASlotSender.GenericSlotSenderP*/GenericSlotSenderP__0__toSendLen = len;

      if (0 == 0) {
          /*TDMASlotSenderC.TDMASlotSender.GenericSlotSenderP*/GenericSlotSenderP__0__state = /*TDMASlotSenderC.TDMASlotSender.GenericSlotSenderP*/GenericSlotSenderP__0__S_OFFSET;
        }
      else 
#line 92
        {
          /*TDMASlotSenderC.TDMASlotSender.GenericSlotSenderP*/GenericSlotSenderP__0__state = /*TDMASlotSenderC.TDMASlotSender.GenericSlotSenderP*/GenericSlotSenderP__0__S_START;
        }
    }
#line 95
    __nesc_atomic_end(__nesc_atomic); }

  if (state_ != /*TDMASlotSenderC.TDMASlotSender.GenericSlotSenderP*/GenericSlotSenderP__0__S_END) {
      /*TDMASlotSenderC.TDMASlotSender.GenericSlotSenderP*/GenericSlotSenderP__0__alarm(0);
      return FAIL;
    }


  if (/*TDMASlotSenderC.TDMASlotSender.GenericSlotSenderP*/GenericSlotSenderP__0__state == /*TDMASlotSenderC.TDMASlotSender.GenericSlotSenderP*/GenericSlotSenderP__0__S_OFFSET) {
      err = /*TDMASlotSenderC.TDMASlotSender.GenericSlotSenderP*/GenericSlotSenderP__0__SubSend__send(/*TDMASlotSenderC.TDMASlotSender.GenericSlotSenderP*/GenericSlotSenderP__0__toSend, /*TDMASlotSenderC.TDMASlotSender.GenericSlotSenderP*/GenericSlotSenderP__0__toSendLen);
      if (err != SUCCESS) {
          { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 106
            /*TDMASlotSenderC.TDMASlotSender.GenericSlotSenderP*/GenericSlotSenderP__0__state = /*TDMASlotSenderC.TDMASlotSender.GenericSlotSenderP*/GenericSlotSenderP__0__S_END;
#line 106
            __nesc_atomic_end(__nesc_atomic); }
        }
    }
  else 
#line 108
    {
      /*TDMASlotSenderC.TDMASlotSender.GenericSlotSenderP*/GenericSlotSenderP__0__Alarm__start(0);
      err = SUCCESS;
    }



  return err;
}

# 40 "/home/csilab/upma/interfaces/AsyncSend.nc"
inline static error_t PureTDMASchedulerP__SubSend__send(message_t * msg, uint8_t len){
#line 40
  unsigned char __nesc_result;
#line 40

#line 40
  __nesc_result = /*TDMASlotSenderC.TDMASlotSender.GenericSlotSenderP*/GenericSlotSenderP__0__Send__send(msg, len);
#line 40

#line 40
  return __nesc_result;
#line 40
}
#line 40
# 147 "/home/csilab/tinyos/tos/lib/timer/TransformAlarmC.nc"
static inline void /*GenericSlotterC.Alarm32khz32C.Transform*/TransformAlarmC__2__Alarm__startAt(/*GenericSlotterC.Alarm32khz32C.Transform*/TransformAlarmC__2__to_size_type t0, /*GenericSlotterC.Alarm32khz32C.Transform*/TransformAlarmC__2__to_size_type dt)
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      /*GenericSlotterC.Alarm32khz32C.Transform*/TransformAlarmC__2__m_t0 = t0;
      /*GenericSlotterC.Alarm32khz32C.Transform*/TransformAlarmC__2__m_dt = dt;
      /*GenericSlotterC.Alarm32khz32C.Transform*/TransformAlarmC__2__set_alarm();
    }
#line 154
    __nesc_atomic_end(__nesc_atomic); }
}

static inline void /*GenericSlotterC.Alarm32khz32C.Transform*/TransformAlarmC__2__Alarm__start(/*GenericSlotterC.Alarm32khz32C.Transform*/TransformAlarmC__2__to_size_type dt)
{
  /*GenericSlotterC.Alarm32khz32C.Transform*/TransformAlarmC__2__Alarm__startAt(/*GenericSlotterC.Alarm32khz32C.Transform*/TransformAlarmC__2__Alarm__getNow(), dt);
}

# 66 "/home/csilab/tinyos/tos/lib/timer/Alarm.nc"
inline static void GenericSlotterP__Alarm__start(GenericSlotterP__Alarm__size_type dt){
#line 66
  /*GenericSlotterC.Alarm32khz32C.Transform*/TransformAlarmC__2__Alarm__start(dt);
#line 66
}
#line 66
# 130 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Control__enableEvents(void )
{
  * (volatile uint16_t * )394U |= 0x0010;
}

# 57 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*GenericSlotterC.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__3__Msp430TimerControl__enableEvents(void ){
#line 57
  /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Control__enableEvents();
#line 57
}
#line 57
# 95 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Control__clearPendingInterrupt(void )
{
  * (volatile uint16_t * )394U &= ~0x0001;
}

# 44 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*GenericSlotterC.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__3__Msp430TimerControl__clearPendingInterrupt(void ){
#line 44
  /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Control__clearPendingInterrupt();
#line 44
}
#line 44
# 155 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Compare__setEvent(uint16_t x)
{
  * (volatile uint16_t * )410U = x;
}

# 41 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*GenericSlotterC.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__3__Msp430Compare__setEvent(uint16_t time){
#line 41
  /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Compare__setEvent(time);
#line 41
}
#line 41
# 45 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430Timer.nc"
inline static uint16_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Timer__get(void ){
#line 45
  unsigned int __nesc_result;
#line 45

#line 45
  __nesc_result = /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__get();
#line 45

#line 45
  return __nesc_result;
#line 45
}
#line 45
# 165 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Compare__setEventFromNow(uint16_t x)
{
  * (volatile uint16_t * )410U = /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Timer__get() + x;
}

# 43 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*GenericSlotterC.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__3__Msp430Compare__setEventFromNow(uint16_t delta){
#line 43
  /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Compare__setEventFromNow(delta);
#line 43
}
#line 43
# 45 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430Timer.nc"
inline static uint16_t /*GenericSlotterC.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__3__Msp430Timer__get(void ){
#line 45
  unsigned int __nesc_result;
#line 45

#line 45
  __nesc_result = /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__get();
#line 45

#line 45
  return __nesc_result;
#line 45
}
#line 45
# 81 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*GenericSlotterC.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__3__Alarm__startAt(uint16_t t0, uint16_t dt)
{
  /* atomic removed: atomic calls only */
  {
    uint16_t now = /*GenericSlotterC.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__3__Msp430Timer__get();
    uint16_t elapsed = now - t0;

#line 87
    if (elapsed >= dt) 
      {
        /*GenericSlotterC.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__3__Msp430Compare__setEventFromNow(2);
      }
    else 
      {
        uint16_t remaining = dt - elapsed;

#line 94
        if (remaining <= 2) {
          /*GenericSlotterC.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__3__Msp430Compare__setEventFromNow(2);
          }
        else {
#line 97
          /*GenericSlotterC.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__3__Msp430Compare__setEvent(now + remaining);
          }
      }
#line 99
    /*GenericSlotterC.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__3__Msp430TimerControl__clearPendingInterrupt();
    /*GenericSlotterC.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__3__Msp430TimerControl__enableEvents();
  }
}

# 103 "/home/csilab/tinyos/tos/lib/timer/Alarm.nc"
inline static void /*GenericSlotterC.Alarm32khz32C.Transform*/TransformAlarmC__2__AlarmFrom__startAt(/*GenericSlotterC.Alarm32khz32C.Transform*/TransformAlarmC__2__AlarmFrom__size_type t0, /*GenericSlotterC.Alarm32khz32C.Transform*/TransformAlarmC__2__AlarmFrom__size_type dt){
#line 103
  /*GenericSlotterC.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__3__Alarm__startAt(t0, dt);
#line 103
}
#line 103
# 31 "/home/csilab/upma/interfaces/SlotterControl.nc"
inline static error_t BeaconSlotP__SlotterControl__synchronize(uint8_t toSlot){
#line 31
  unsigned char __nesc_result;
#line 31

#line 31
  __nesc_result = GenericSlotterP__SlotterControl__synchronize(toSlot);
#line 31

#line 31
  return __nesc_result;
#line 31
}
#line 31
# 136 "/home/csilab/upma/system/BeaconSlotP.nc"
static inline void BeaconSlotP__SyncAlarm__fired(void )
#line 136
{
  BeaconSlotP__SlotterControl__synchronize(1);
}

# 78 "/home/csilab/tinyos/tos/lib/timer/Alarm.nc"
inline static void /*PureTDMASchedulerC.Alarm32khz32C.Transform*/TransformAlarmC__3__Alarm__fired(void ){
#line 78
  BeaconSlotP__SyncAlarm__fired();
#line 78
}
#line 78
# 162 "/home/csilab/tinyos/tos/lib/timer/TransformAlarmC.nc"
static inline void /*PureTDMASchedulerC.Alarm32khz32C.Transform*/TransformAlarmC__3__AlarmFrom__fired(void )
{
  /* atomic removed: atomic calls only */
  {
    if (/*PureTDMASchedulerC.Alarm32khz32C.Transform*/TransformAlarmC__3__m_dt == 0) 
      {
        /*PureTDMASchedulerC.Alarm32khz32C.Transform*/TransformAlarmC__3__Alarm__fired();
      }
    else 
      {
        /*PureTDMASchedulerC.Alarm32khz32C.Transform*/TransformAlarmC__3__set_alarm();
      }
  }
}

# 78 "/home/csilab/tinyos/tos/lib/timer/Alarm.nc"
inline static void /*PureTDMASchedulerC.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__4__Alarm__fired(void ){
#line 78
  /*PureTDMASchedulerC.Alarm32khz32C.Transform*/TransformAlarmC__3__AlarmFrom__fired();
#line 78
}
#line 78
# 135 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Control__disableEvents(void )
{
  * (volatile uint16_t * )396U &= ~0x0010;
}

# 58 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*PureTDMASchedulerC.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__4__Msp430TimerControl__disableEvents(void ){
#line 58
  /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Control__disableEvents();
#line 58
}
#line 58
# 70 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*PureTDMASchedulerC.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__4__Msp430Compare__fired(void )
{
  /*PureTDMASchedulerC.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__4__Msp430TimerControl__disableEvents();
  /*PureTDMASchedulerC.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__4__Alarm__fired();
}

# 45 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Compare__fired(void ){
#line 45
  /*PureTDMASchedulerC.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__4__Msp430Compare__fired();
#line 45
}
#line 45
# 150 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__getEvent(void )
{
  return * (volatile uint16_t * )412U;
}

#line 188
static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__default__captured(uint16_t n)
{
}

# 86 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__captured(uint16_t time){
#line 86
  /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__default__captured(time);
#line 86
}
#line 86
# 58 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__cc_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__int2CC(uint16_t x)
#line 58
{
#line 58
  union /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8____nesc_unnamed4410 {
#line 58
    uint16_t f;
#line 58
    /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__cc_t t;
  } 
#line 58
  c = { .f = x };

#line 58
  return c.t;
}

#line 85
static inline /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__cc_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Control__getControl(void )
{
  return /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__int2CC(* (volatile uint16_t * )396U);
}

#line 180
static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Event__fired(void )
{
  if (/*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Control__getControl().cap) {
    /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__captured(/*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__getEvent());
    }
  else {
#line 185
    /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Compare__fired();
    }
}

#line 130
static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Control__enableEvents(void )
{
  * (volatile uint16_t * )396U |= 0x0010;
}

# 57 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*PureTDMASchedulerC.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__4__Msp430TimerControl__enableEvents(void ){
#line 57
  /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Control__enableEvents();
#line 57
}
#line 57
# 95 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Control__clearPendingInterrupt(void )
{
  * (volatile uint16_t * )396U &= ~0x0001;
}

# 44 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*PureTDMASchedulerC.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__4__Msp430TimerControl__clearPendingInterrupt(void ){
#line 44
  /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Control__clearPendingInterrupt();
#line 44
}
#line 44
# 155 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Compare__setEvent(uint16_t x)
{
  * (volatile uint16_t * )412U = x;
}

# 41 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*PureTDMASchedulerC.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__4__Msp430Compare__setEvent(uint16_t time){
#line 41
  /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Compare__setEvent(time);
#line 41
}
#line 41
# 45 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430Timer.nc"
inline static uint16_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Timer__get(void ){
#line 45
  unsigned int __nesc_result;
#line 45

#line 45
  __nesc_result = /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__get();
#line 45

#line 45
  return __nesc_result;
#line 45
}
#line 45
# 165 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Compare__setEventFromNow(uint16_t x)
{
  * (volatile uint16_t * )412U = /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Timer__get() + x;
}

# 43 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*PureTDMASchedulerC.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__4__Msp430Compare__setEventFromNow(uint16_t delta){
#line 43
  /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Compare__setEventFromNow(delta);
#line 43
}
#line 43
# 45 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430Timer.nc"
inline static uint16_t /*PureTDMASchedulerC.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__4__Msp430Timer__get(void ){
#line 45
  unsigned int __nesc_result;
#line 45

#line 45
  __nesc_result = /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__get();
#line 45

#line 45
  return __nesc_result;
#line 45
}
#line 45
# 81 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*PureTDMASchedulerC.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__4__Alarm__startAt(uint16_t t0, uint16_t dt)
{
  /* atomic removed: atomic calls only */
  {
    uint16_t now = /*PureTDMASchedulerC.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__4__Msp430Timer__get();
    uint16_t elapsed = now - t0;

#line 87
    if (elapsed >= dt) 
      {
        /*PureTDMASchedulerC.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__4__Msp430Compare__setEventFromNow(2);
      }
    else 
      {
        uint16_t remaining = dt - elapsed;

#line 94
        if (remaining <= 2) {
          /*PureTDMASchedulerC.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__4__Msp430Compare__setEventFromNow(2);
          }
        else {
#line 97
          /*PureTDMASchedulerC.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__4__Msp430Compare__setEvent(now + remaining);
          }
      }
#line 99
    /*PureTDMASchedulerC.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__4__Msp430TimerControl__clearPendingInterrupt();
    /*PureTDMASchedulerC.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__4__Msp430TimerControl__enableEvents();
  }
}

# 103 "/home/csilab/tinyos/tos/lib/timer/Alarm.nc"
inline static void /*PureTDMASchedulerC.Alarm32khz32C.Transform*/TransformAlarmC__3__AlarmFrom__startAt(/*PureTDMASchedulerC.Alarm32khz32C.Transform*/TransformAlarmC__3__AlarmFrom__size_type t0, /*PureTDMASchedulerC.Alarm32khz32C.Transform*/TransformAlarmC__3__AlarmFrom__size_type dt){
#line 103
  /*PureTDMASchedulerC.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__4__Alarm__startAt(t0, dt);
#line 103
}
#line 103
# 703 "/home/csilab/upma/lib/macs/pure-tdma/PureTDMASchedulerP.nc"
static inline void PureTDMASchedulerP__TxOffset_alarm__fired(void )
#line 703
{
  if (PureTDMASchedulerP__is_root == FALSE) {
      if (PureTDMASchedulerP__rcv_indicator == FALSE) {
          PureTDMASchedulerP__send_AH_Pkt(&PureTDMASchedulerP__AH_pkt, PureTDMASchedulerP__my_dest[PureTDMASchedulerP__txing_flowid], PureTDMASchedulerP__txing_flowid);
        }
    }
  else 
#line 708
    {
      PureTDMASchedulerP__send_AH_Pkt(&PureTDMASchedulerP__AH_pkt, PureTDMASchedulerP__my_dest[PureTDMASchedulerP__my_flow_id], PureTDMASchedulerP__my_flow_id);
    }
}

# 78 "/home/csilab/tinyos/tos/lib/timer/Alarm.nc"
inline static void /*PureTDMASchedulerC.offset_alarm.Msp430Alarm*/Msp430AlarmC__5__Alarm__fired(void ){
#line 78
  PureTDMASchedulerP__TxOffset_alarm__fired();
#line 78
}
#line 78
# 135 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Control__disableEvents(void )
{
  * (volatile uint16_t * )398U &= ~0x0010;
}

# 58 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*PureTDMASchedulerC.offset_alarm.Msp430Alarm*/Msp430AlarmC__5__Msp430TimerControl__disableEvents(void ){
#line 58
  /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Control__disableEvents();
#line 58
}
#line 58
# 70 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*PureTDMASchedulerC.offset_alarm.Msp430Alarm*/Msp430AlarmC__5__Msp430Compare__fired(void )
{
  /*PureTDMASchedulerC.offset_alarm.Msp430Alarm*/Msp430AlarmC__5__Msp430TimerControl__disableEvents();
  /*PureTDMASchedulerC.offset_alarm.Msp430Alarm*/Msp430AlarmC__5__Alarm__fired();
}

# 45 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Compare__fired(void ){
#line 45
  /*PureTDMASchedulerC.offset_alarm.Msp430Alarm*/Msp430AlarmC__5__Msp430Compare__fired();
#line 45
}
#line 45
# 150 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__getEvent(void )
{
  return * (volatile uint16_t * )414U;
}

#line 188
static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__default__captured(uint16_t n)
{
}

# 86 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__captured(uint16_t time){
#line 86
  /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__default__captured(time);
#line 86
}
#line 86
# 58 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__cc_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__int2CC(uint16_t x)
#line 58
{
#line 58
  union /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9____nesc_unnamed4411 {
#line 58
    uint16_t f;
#line 58
    /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__cc_t t;
  } 
#line 58
  c = { .f = x };

#line 58
  return c.t;
}

#line 85
static inline /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__cc_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Control__getControl(void )
{
  return /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__int2CC(* (volatile uint16_t * )398U);
}

#line 180
static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Event__fired(void )
{
  if (/*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Control__getControl().cap) {
    /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__captured(/*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__getEvent());
    }
  else {
#line 185
    /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Compare__fired();
    }
}

# 131 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__VectorTimerX1__fired(void )
{
  uint8_t n = * (volatile uint16_t * )286U;

#line 134
  /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__fired(n >> 1);
}

# 39 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430TimerEvent.nc"
inline static void Msp430TimerCommonP__VectorTimerB1__fired(void ){
#line 39
  /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__VectorTimerX1__fired();
#line 39
}
#line 39
# 124 "/home/csilab/tinyos/tos/system/SchedulerBasicP.nc"
static inline void SchedulerBasicP__Scheduler__init(void )
{
  /* atomic removed: atomic calls only */
  {
    memset((void *)SchedulerBasicP__m_next, SchedulerBasicP__NO_TASK, sizeof SchedulerBasicP__m_next);
    SchedulerBasicP__m_head = SchedulerBasicP__NO_TASK;
    SchedulerBasicP__m_tail = SchedulerBasicP__NO_TASK;
  }
}

# 57 "/home/csilab/tinyos/tos/interfaces/Scheduler.nc"
inline static void RealMainP__Scheduler__init(void ){
#line 57
  SchedulerBasicP__Scheduler__init();
#line 57
}
#line 57
# 56 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__set(void )
#line 56
{
  /* atomic removed: atomic calls only */
#line 56
  * (volatile uint8_t * )49U |= 0x01 << 6;
}

# 48 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__HplGeneralIO__set(void ){
#line 48
  /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__set();
#line 48
}
#line 48
# 48 "/home/csilab/tinyos/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__set(void )
#line 48
{
#line 48
  /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__HplGeneralIO__set();
}

# 40 "/home/csilab/tinyos/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led2__set(void ){
#line 40
  /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__set();
#line 40
}
#line 40
# 56 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__set(void )
#line 56
{
  /* atomic removed: atomic calls only */
#line 56
  * (volatile uint8_t * )49U |= 0x01 << 5;
}

# 48 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__HplGeneralIO__set(void ){
#line 48
  /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__set();
#line 48
}
#line 48
# 48 "/home/csilab/tinyos/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__set(void )
#line 48
{
#line 48
  /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__HplGeneralIO__set();
}

# 40 "/home/csilab/tinyos/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led1__set(void ){
#line 40
  /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__set();
#line 40
}
#line 40
# 56 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__set(void )
#line 56
{
  /* atomic removed: atomic calls only */
#line 56
  * (volatile uint8_t * )49U |= 0x01 << 4;
}

# 48 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__HplGeneralIO__set(void ){
#line 48
  /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__set();
#line 48
}
#line 48
# 48 "/home/csilab/tinyos/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__set(void )
#line 48
{
#line 48
  /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__HplGeneralIO__set();
}

# 40 "/home/csilab/tinyos/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led0__set(void ){
#line 40
  /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__set();
#line 40
}
#line 40
# 63 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__makeOutput(void )
#line 63
{
  /* atomic removed: atomic calls only */
#line 63
  * (volatile uint8_t * )50U |= 0x01 << 6;
}

# 85 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__HplGeneralIO__makeOutput(void ){
#line 85
  /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__makeOutput();
#line 85
}
#line 85
# 54 "/home/csilab/tinyos/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__makeOutput(void )
#line 54
{
#line 54
  /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__HplGeneralIO__makeOutput();
}

# 46 "/home/csilab/tinyos/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led2__makeOutput(void ){
#line 46
  /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__makeOutput();
#line 46
}
#line 46
# 63 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__makeOutput(void )
#line 63
{
  /* atomic removed: atomic calls only */
#line 63
  * (volatile uint8_t * )50U |= 0x01 << 5;
}

# 85 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__HplGeneralIO__makeOutput(void ){
#line 85
  /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__makeOutput();
#line 85
}
#line 85
# 54 "/home/csilab/tinyos/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__makeOutput(void )
#line 54
{
#line 54
  /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__HplGeneralIO__makeOutput();
}

# 46 "/home/csilab/tinyos/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led1__makeOutput(void ){
#line 46
  /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__makeOutput();
#line 46
}
#line 46
# 63 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__makeOutput(void )
#line 63
{
  /* atomic removed: atomic calls only */
#line 63
  * (volatile uint8_t * )50U |= 0x01 << 4;
}

# 85 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__HplGeneralIO__makeOutput(void ){
#line 85
  /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__makeOutput();
#line 85
}
#line 85
# 54 "/home/csilab/tinyos/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__makeOutput(void )
#line 54
{
#line 54
  /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__HplGeneralIO__makeOutput();
}

# 46 "/home/csilab/tinyos/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led0__makeOutput(void ){
#line 46
  /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__makeOutput();
#line 46
}
#line 46
# 56 "/home/csilab/tinyos/tos/system/LedsP.nc"
static inline error_t LedsP__Init__init(void )
#line 56
{
  /* atomic removed: atomic calls only */
#line 57
  {
    ;
    LedsP__Led0__makeOutput();
    LedsP__Led1__makeOutput();
    LedsP__Led2__makeOutput();
    LedsP__Led0__set();
    LedsP__Led1__set();
    LedsP__Led2__set();
  }
  return SUCCESS;
}

# 62 "/home/csilab/tinyos/tos/interfaces/Init.nc"
inline static error_t PlatformP__LedsInit__init(void ){
#line 62
  unsigned char __nesc_result;
#line 62

#line 62
  __nesc_result = LedsP__Init__init();
#line 62

#line 62
  return __nesc_result;
#line 62
}
#line 62
# 36 "/home/csilab/tinyos/tos/platforms/telosb/hardware.h"
static inline  void TOSH_SET_SIMO0_PIN()
#line 36
{
#line 36
  static volatile uint8_t r __asm ("0x0019");

#line 36
  r |= 1 << 1;
}

#line 37
static inline  void TOSH_SET_UCLK0_PIN()
#line 37
{
#line 37
  static volatile uint8_t r __asm ("0x0019");

#line 37
  r |= 1 << 3;
}

#line 88
static inline  void TOSH_SET_FLASH_CS_PIN()
#line 88
{
#line 88
  static volatile uint8_t r __asm ("0x001D");

#line 88
  r |= 1 << 4;
}

#line 37
static inline  void TOSH_CLR_UCLK0_PIN()
#line 37
{
#line 37
  static volatile uint8_t r __asm ("0x0019");

#line 37
  r &= ~(1 << 3);
}

#line 88
static inline  void TOSH_CLR_FLASH_CS_PIN()
#line 88
{
#line 88
  static volatile uint8_t r __asm ("0x001D");

#line 88
  r &= ~(1 << 4);
}

# 11 "/home/csilab/tinyos/tos/platforms/telosb/MotePlatformC.nc"
static __inline void MotePlatformC__TOSH_wait(void )
#line 11
{
  __nop();
#line 12
  __nop();
}

# 89 "/home/csilab/tinyos/tos/platforms/telosb/hardware.h"
static inline  void TOSH_SET_FLASH_HOLD_PIN()
#line 89
{
#line 89
  static volatile uint8_t r __asm ("0x001D");

#line 89
  r |= 1 << 7;
}

#line 88
static inline  void TOSH_MAKE_FLASH_CS_OUTPUT()
#line 88
{
#line 88
  static volatile uint8_t r __asm ("0x001E");

#line 88
  r |= 1 << 4;
}

#line 89
static inline  void TOSH_MAKE_FLASH_HOLD_OUTPUT()
#line 89
{
#line 89
  static volatile uint8_t r __asm ("0x001E");

#line 89
  r |= 1 << 7;
}

#line 37
static inline  void TOSH_MAKE_UCLK0_OUTPUT()
#line 37
{
#line 37
  static volatile uint8_t r __asm ("0x001A");

#line 37
  r |= 1 << 3;
}

#line 36
static inline  void TOSH_MAKE_SIMO0_OUTPUT()
#line 36
{
#line 36
  static volatile uint8_t r __asm ("0x001A");

#line 36
  r |= 1 << 1;
}

# 27 "/home/csilab/tinyos/tos/platforms/telosb/MotePlatformC.nc"
static inline void MotePlatformC__TOSH_FLASH_M25P_DP(void )
#line 27
{

  TOSH_MAKE_SIMO0_OUTPUT();
  TOSH_MAKE_UCLK0_OUTPUT();
  TOSH_MAKE_FLASH_HOLD_OUTPUT();
  TOSH_MAKE_FLASH_CS_OUTPUT();
  TOSH_SET_FLASH_HOLD_PIN();
  TOSH_SET_FLASH_CS_PIN();

  MotePlatformC__TOSH_wait();


  TOSH_CLR_FLASH_CS_PIN();
  TOSH_CLR_UCLK0_PIN();

  MotePlatformC__TOSH_FLASH_M25P_DP_bit(TRUE);
  MotePlatformC__TOSH_FLASH_M25P_DP_bit(FALSE);
  MotePlatformC__TOSH_FLASH_M25P_DP_bit(TRUE);
  MotePlatformC__TOSH_FLASH_M25P_DP_bit(TRUE);
  MotePlatformC__TOSH_FLASH_M25P_DP_bit(TRUE);
  MotePlatformC__TOSH_FLASH_M25P_DP_bit(FALSE);
  MotePlatformC__TOSH_FLASH_M25P_DP_bit(FALSE);
  MotePlatformC__TOSH_FLASH_M25P_DP_bit(TRUE);

  TOSH_SET_FLASH_CS_PIN();
  TOSH_SET_UCLK0_PIN();
  TOSH_SET_SIMO0_PIN();
}

#line 6
static __inline void MotePlatformC__uwait(uint16_t u)
#line 6
{
  uint16_t t0 = TAR;

#line 8
  while (TAR - t0 <= u) ;
}

#line 56
static inline error_t MotePlatformC__Init__init(void )
#line 56
{
  /* atomic removed: atomic calls only */

  {
    P1SEL = 0;
    P2SEL = 0;
    P3SEL = 0;
    P4SEL = 0;
    P5SEL = 0;
    P6SEL = 0;

    P1OUT = 0x00;
    P1DIR = 0xe0;

    P2OUT = 0x30;
    P2DIR = 0x7b;

    P3OUT = 0x00;
    P3DIR = 0xf1;

    P4OUT = 0xdd;
    P4DIR = 0xfd;

    P5OUT = 0xff;
    P5DIR = 0xff;

    P6OUT = 0x00;
    P6DIR = 0xff;

    P1IE = 0;
    P2IE = 0;






    MotePlatformC__uwait(1024 * 10);

    MotePlatformC__TOSH_FLASH_M25P_DP();
  }

  return SUCCESS;
}

# 62 "/home/csilab/tinyos/tos/interfaces/Init.nc"
inline static error_t PlatformP__MoteInit__init(void ){
#line 62
  unsigned char __nesc_result;
#line 62

#line 62
  __nesc_result = MotePlatformC__Init__init();
#line 62

#line 62
  return __nesc_result;
#line 62
}
#line 62
# 163 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430ClockP.nc"
static inline void Msp430ClockP__startTimerB(void )
{

  Msp430ClockP__TBCTL = 0x0020 | (Msp430ClockP__TBCTL & ~(0x0020 | 0x0010));
}

#line 151
static inline void Msp430ClockP__startTimerA(void )
{

  Msp430ClockP__TACTL = 0x0020 | (Msp430ClockP__TACTL & ~(0x0020 | 0x0010));
}

#line 115
static inline void Msp430ClockP__Msp430ClockInit__defaultInitTimerB(void )
{
  TBR = 0;









  Msp430ClockP__TBCTL = 0x0100 | 0x0002;
}

#line 145
static inline void Msp430ClockP__Msp430ClockInit__default__initTimerB(void )
{
  Msp430ClockP__Msp430ClockInit__defaultInitTimerB();
}

# 43 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430ClockInit.nc"
inline static void Msp430ClockP__Msp430ClockInit__initTimerB(void ){
#line 43
  Msp430ClockP__Msp430ClockInit__default__initTimerB();
#line 43
}
#line 43
# 100 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430ClockP.nc"
static inline void Msp430ClockP__Msp430ClockInit__defaultInitTimerA(void )
{
  TAR = 0;









  Msp430ClockP__TACTL = 0x0200 | 0x0002;
}

#line 140
static inline void Msp430ClockP__Msp430ClockInit__default__initTimerA(void )
{
  Msp430ClockP__Msp430ClockInit__defaultInitTimerA();
}

# 42 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430ClockInit.nc"
inline static void Msp430ClockP__Msp430ClockInit__initTimerA(void ){
#line 42
  Msp430ClockP__Msp430ClockInit__default__initTimerA();
#line 42
}
#line 42
# 79 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430ClockP.nc"
static inline void Msp430ClockP__Msp430ClockInit__defaultInitClocks(void )
{





  BCSCTL1 = 0x80 | (BCSCTL1 & ((0x04 | 0x02) | 0x01));







  BCSCTL2 = 0x04;


  Msp430ClockP__IE1 &= ~0x02;
}

#line 135
static inline void Msp430ClockP__Msp430ClockInit__default__initClocks(void )
{
  Msp430ClockP__Msp430ClockInit__defaultInitClocks();
}

# 41 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430ClockInit.nc"
inline static void Msp430ClockP__Msp430ClockInit__initClocks(void ){
#line 41
  Msp430ClockP__Msp430ClockInit__default__initClocks();
#line 41
}
#line 41
# 181 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430ClockP.nc"
static inline uint16_t Msp430ClockP__test_calib_busywait_delta(int calib)
{
  int8_t aclk_count = 2;
  uint16_t dco_prev = 0;
  uint16_t dco_curr = 0;

  Msp430ClockP__set_dco_calib(calib);

  while (aclk_count-- > 0) 
    {
      TBCCR0 = TBR + Msp430ClockP__ACLK_CALIB_PERIOD;
      TBCCTL0 &= ~0x0001;
      while ((TBCCTL0 & 0x0001) == 0) ;
      dco_prev = dco_curr;
      dco_curr = TAR;
    }

  return dco_curr - dco_prev;
}




static inline void Msp430ClockP__busyCalibrateDco(void )
{

  int calib;
  int step;






  for (calib = 0, step = 0x800; step != 0; step >>= 1) 
    {

      if (Msp430ClockP__test_calib_busywait_delta(calib | step) <= Msp430ClockP__TARGET_DCO_DELTA) {
        calib |= step;
        }
    }

  if ((calib & 0x0e0) == 0x0e0) {
    calib &= ~0x01f;
    }
  Msp430ClockP__set_dco_calib(calib);
}

#line 67
static inline void Msp430ClockP__Msp430ClockInit__defaultSetupDcoCalibrate(void )
{



  Msp430ClockP__TACTL = 0x0200 | 0x0020;
  Msp430ClockP__TBCTL = 0x0100 | 0x0020;
  BCSCTL1 = 0x80 | 0x04;
  BCSCTL2 = 0;
  TBCCTL0 = 0x4000;
}

#line 130
static inline void Msp430ClockP__Msp430ClockInit__default__setupDcoCalibrate(void )
{
  Msp430ClockP__Msp430ClockInit__defaultSetupDcoCalibrate();
}

# 40 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430ClockInit.nc"
inline static void Msp430ClockP__Msp430ClockInit__setupDcoCalibrate(void ){
#line 40
  Msp430ClockP__Msp430ClockInit__default__setupDcoCalibrate();
#line 40
}
#line 40
# 229 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430ClockP.nc"
static inline error_t Msp430ClockP__Init__init(void )
{

  Msp430ClockP__TACTL = 0x0004;
  Msp430ClockP__TAIV = 0;
  Msp430ClockP__TBCTL = 0x0004;
  Msp430ClockP__TBIV = 0;
  /* atomic removed: atomic calls only */

  {
    Msp430ClockP__Msp430ClockInit__setupDcoCalibrate();
    Msp430ClockP__busyCalibrateDco();
    Msp430ClockP__Msp430ClockInit__initClocks();
    Msp430ClockP__Msp430ClockInit__initTimerA();
    Msp430ClockP__Msp430ClockInit__initTimerB();
    Msp430ClockP__startTimerA();
    Msp430ClockP__startTimerB();
  }

  return SUCCESS;
}

# 62 "/home/csilab/tinyos/tos/interfaces/Init.nc"
inline static error_t PlatformP__MoteClockInit__init(void ){
#line 62
  unsigned char __nesc_result;
#line 62

#line 62
  __nesc_result = Msp430ClockP__Init__init();
#line 62

#line 62
  return __nesc_result;
#line 62
}
#line 62
# 10 "/home/csilab/tinyos/tos/platforms/telosa/PlatformP.nc"
static inline error_t PlatformP__Init__init(void )
#line 10
{
  WDTCTL = 0x5A00 + 0x0080;
  PlatformP__MoteClockInit__init();
  PlatformP__MoteInit__init();
  PlatformP__LedsInit__init();
  return SUCCESS;
}

# 62 "/home/csilab/tinyos/tos/interfaces/Init.nc"
inline static error_t RealMainP__PlatformInit__init(void ){
#line 62
  unsigned char __nesc_result;
#line 62

#line 62
  __nesc_result = PlatformP__Init__init();
#line 62

#line 62
  return __nesc_result;
#line 62
}
#line 62
# 36 "/home/csilab/tinyos/tos/platforms/telosb/hardware.h"
static inline  void TOSH_CLR_SIMO0_PIN()
#line 36
{
#line 36
  static volatile uint8_t r __asm ("0x0019");

#line 36
  r &= ~(1 << 1);
}

# 65 "/home/csilab/tinyos/tos/interfaces/Scheduler.nc"
inline static bool RealMainP__Scheduler__runNextTask(void ){
#line 65
  unsigned char __nesc_result;
#line 65

#line 65
  __nesc_result = SchedulerBasicP__Scheduler__runNextTask();
#line 65

#line 65
  return __nesc_result;
#line 65
}
#line 65
# 110 "/home/csilab/tinyos/tos/interfaces/AMSend.nc"
inline static void /*TestPureTdmaC.SendingApp.AMSender.SenderC.AMQueueEntryP*/AMQueueEntryP__1__AMSend__sendDone(message_t * msg, error_t error){
#line 110
  /*TestPureTdmaC.SendingApp.App*/SendingC__0__AMSender__sendDone(msg, error);
#line 110
}
#line 110
# 65 "/home/csilab/tinyos/tos/system/AMQueueEntryP.nc"
static inline void /*TestPureTdmaC.SendingApp.AMSender.SenderC.AMQueueEntryP*/AMQueueEntryP__1__Send__sendDone(message_t *m, error_t err)
#line 65
{
  /*TestPureTdmaC.SendingApp.AMSender.SenderC.AMQueueEntryP*/AMQueueEntryP__1__AMSend__sendDone(m, err);
}

# 215 "/home/csilab/tinyos/tos/system/AMQueueImplP.nc"
static inline void /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__Send__default__sendDone(uint8_t id, message_t *msg, error_t err)
#line 215
{
}

# 100 "/home/csilab/tinyos/tos/interfaces/Send.nc"
inline static void /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__Send__sendDone(uint8_t arg_0x7f7fed178630, message_t * msg, error_t error){
#line 100
  switch (arg_0x7f7fed178630) {
#line 100
    case 0U:
#line 100
      /*TestPureTdmaC.SendingApp.AMSender.SenderC.AMQueueEntryP*/AMQueueEntryP__1__Send__sendDone(msg, error);
#line 100
      break;
#line 100
    default:
#line 100
      /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__Send__default__sendDone(arg_0x7f7fed178630, msg, error);
#line 100
      break;
#line 100
    }
#line 100
}
#line 100
# 126 "/home/csilab/tinyos/tos/system/AMQueueImplP.nc"
static inline void /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__CancelTask__runTask(void )
#line 126
{
  uint8_t i;
#line 127
  uint8_t j;
#line 127
  uint8_t mask;
#line 127
  uint8_t last;
  message_t *msg;

#line 129
  for (i = 0; i < 1 / 8 + 1; i++) {
      if (/*AMQueueP.AMQueueImplP*/AMQueueImplP__1__cancelMask[i]) {
          for (mask = 1, j = 0; j < 8; j++) {
              if (/*AMQueueP.AMQueueImplP*/AMQueueImplP__1__cancelMask[i] & mask) {
                  last = i * 8 + j;
                  msg = /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__queue[last].msg;
                  /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__queue[last].msg = (void *)0;
                  /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__cancelMask[i] &= ~mask;
                  /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__Send__sendDone(last, msg, ECANCEL);
                }
              mask <<= 1;
            }
        }
    }
}

#line 169
static inline void /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__errorTask__runTask(void )
#line 169
{
  /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__sendDone(/*AMQueueP.AMQueueImplP*/AMQueueImplP__1__current, /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__queue[/*AMQueueP.AMQueueImplP*/AMQueueImplP__1__current].msg, FAIL);
}

# 67 "/home/csilab/tinyos/tos/interfaces/TaskBasic.nc"
inline static error_t /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__errorTask__postTask(void ){
#line 67
  unsigned char __nesc_result;
#line 67

#line 67
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(/*AMQueueP.AMQueueImplP*/AMQueueImplP__1__errorTask);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 80 "/home/csilab/tinyos/tos/interfaces/AMSend.nc"
inline static error_t /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__AMSend__send(am_id_t arg_0x7f7fed176840, am_addr_t addr, message_t * msg, uint8_t len){
#line 80
  unsigned char __nesc_result;
#line 80

#line 80
  __nesc_result = CC2420ActiveMessageP__AMSend__send(arg_0x7f7fed176840, addr, msg, len);
#line 80

#line 80
  return __nesc_result;
#line 80
}
#line 80
# 208 "/home/csilab/upma/chips/cc2420/CC2420ActiveMessageP.nc"
static inline uint8_t CC2420ActiveMessageP__Packet__payloadLength(message_t *msg)
#line 208
{
  return __nesc_ntoh_leuint8(CC2420ActiveMessageP__CC2420PacketBody__getHeader(msg)->length.nxdata) - CC2420_SIZE;
}

# 78 "/home/csilab/tinyos/tos/interfaces/Packet.nc"
inline static uint8_t /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__Packet__payloadLength(message_t * msg){
#line 78
  unsigned char __nesc_result;
#line 78

#line 78
  __nesc_result = CC2420ActiveMessageP__Packet__payloadLength(msg);
#line 78

#line 78
  return __nesc_result;
#line 78
}
#line 78
# 67 "/home/csilab/upma/interfaces/AMPacket.nc"
inline static am_addr_t /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__AMPacket__destination(message_t * amsg){
#line 67
  unsigned int __nesc_result;
#line 67

#line 67
  __nesc_result = CC2420ActiveMessageP__AMPacket__destination(amsg);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
#line 136
inline static am_id_t /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__AMPacket__type(message_t * amsg){
#line 136
  unsigned char __nesc_result;
#line 136

#line 136
  __nesc_result = CC2420ActiveMessageP__AMPacket__type(amsg);
#line 136

#line 136
  return __nesc_result;
#line 136
}
#line 136
# 65 "/home/csilab/tinyos/tos/system/AMQueueImplP.nc"
static inline void /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__nextPacket(void )
#line 65
{
  uint8_t i;

#line 67
  /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__current = (/*AMQueueP.AMQueueImplP*/AMQueueImplP__1__current + 1) % 1;
  for (i = 0; i < 1; i++) {
      if (/*AMQueueP.AMQueueImplP*/AMQueueImplP__1__queue[/*AMQueueP.AMQueueImplP*/AMQueueImplP__1__current].msg == (void *)0 || 
      /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__cancelMask[/*AMQueueP.AMQueueImplP*/AMQueueImplP__1__current / 8] & (1 << /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__current % 8)) 
        {
          /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__current = (/*AMQueueP.AMQueueImplP*/AMQueueImplP__1__current + 1) % 1;
        }
      else {
          break;
        }
    }
  if (i >= 1) {
#line 78
    /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__current = 1;
    }
}

#line 174
static inline void /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__tryToSend(void )
#line 174
{
  /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__nextPacket();
  if (/*AMQueueP.AMQueueImplP*/AMQueueImplP__1__current < 1) {
      error_t nextErr;
      message_t *nextMsg = /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__queue[/*AMQueueP.AMQueueImplP*/AMQueueImplP__1__current].msg;
      am_id_t nextId = /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__AMPacket__type(nextMsg);
      am_addr_t nextDest = /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__AMPacket__destination(nextMsg);
      uint8_t len = /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__Packet__payloadLength(nextMsg);

#line 182
      nextErr = /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__AMSend__send(nextId, nextDest, nextMsg, len);
      if (nextErr != SUCCESS) {
          /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__errorTask__postTask();
        }
    }
}

# 188 "/home/csilab/upma/chips/cc2420/CC2420CsmaP.nc"
static inline uint8_t CC2420CsmaP__Send__maxPayloadLength(void )
#line 188
{
  return 28;
}

# 63 "/home/csilab/upma/interfaces/AsyncSend.nc"
inline static uint8_t SenderDispatcherP__SubSend__maxPayloadLength(void ){
#line 63
  unsigned char __nesc_result;
#line 63

#line 63
  __nesc_result = CC2420CsmaP__Send__maxPayloadLength();
#line 63

#line 63
  return __nesc_result;
#line 63
}
#line 63
# 98 "/home/csilab/upma/system/SenderDispatcherP.nc"
static inline uint8_t SenderDispatcherP__Send__maxPayloadLength(uint8_t slotType)
#line 98
{
  return SenderDispatcherP__SubSend__maxPayloadLength();
}

# 63 "/home/csilab/upma/interfaces/AsyncSend.nc"
inline static uint8_t /*TDMASlotSenderC.TDMASlotSender.GenericSlotSenderP*/GenericSlotSenderP__0__SubSend__maxPayloadLength(void ){
#line 63
  unsigned char __nesc_result;
#line 63

#line 63
  __nesc_result = SenderDispatcherP__Send__maxPayloadLength(TDMA_SLOT);
#line 63

#line 63
  return __nesc_result;
#line 63
}
#line 63
# 150 "/home/csilab/upma/system/GenericSlotSenderP.nc"
static inline uint8_t /*TDMASlotSenderC.TDMASlotSender.GenericSlotSenderP*/GenericSlotSenderP__0__Send__maxPayloadLength(void )
#line 150
{
  return /*TDMASlotSenderC.TDMASlotSender.GenericSlotSenderP*/GenericSlotSenderP__0__SubSend__maxPayloadLength();
}

# 63 "/home/csilab/upma/interfaces/AsyncSend.nc"
inline static uint8_t PureTDMASchedulerP__SubSend__maxPayloadLength(void ){
#line 63
  unsigned char __nesc_result;
#line 63

#line 63
  __nesc_result = /*TDMASlotSenderC.TDMASlotSender.GenericSlotSenderP*/GenericSlotSenderP__0__Send__maxPayloadLength();
#line 63

#line 63
  return __nesc_result;
#line 63
}
#line 63
# 682 "/home/csilab/upma/lib/macs/pure-tdma/PureTDMASchedulerP.nc"
static inline uint8_t PureTDMASchedulerP__Send__maxPayloadLength(void )
#line 682
{
  return PureTDMASchedulerP__SubSend__maxPayloadLength();
}

# 63 "/home/csilab/upma/interfaces/AsyncSend.nc"
inline static uint8_t AsyncSendAdapterP__AsyncSend__maxPayloadLength(void ){
#line 63
  unsigned char __nesc_result;
#line 63

#line 63
  __nesc_result = PureTDMASchedulerP__Send__maxPayloadLength();
#line 63

#line 63
  return __nesc_result;
#line 63
}
#line 63
# 77 "/home/csilab/upma/system/AsyncSendAdapterP.nc"
static inline uint8_t AsyncSendAdapterP__Send__maxPayloadLength(void )
{
  return AsyncSendAdapterP__AsyncSend__maxPayloadLength();
}

# 112 "/home/csilab/tinyos/tos/interfaces/Send.nc"
inline static uint8_t UniqueSendP__SubSend__maxPayloadLength(void ){
#line 112
  unsigned char __nesc_result;
#line 112

#line 112
  __nesc_result = AsyncSendAdapterP__Send__maxPayloadLength();
#line 112

#line 112
  return __nesc_result;
#line 112
}
#line 112
# 95 "/home/csilab/tinyos/tos/chips/cc2420/unique/UniqueSendP.nc"
static inline uint8_t UniqueSendP__Send__maxPayloadLength(void )
#line 95
{
  return UniqueSendP__SubSend__maxPayloadLength();
}

# 112 "/home/csilab/tinyos/tos/interfaces/Send.nc"
inline static uint8_t CC2420TinyosNetworkP__SubSend__maxPayloadLength(void ){
#line 112
  unsigned char __nesc_result;
#line 112

#line 112
  __nesc_result = UniqueSendP__Send__maxPayloadLength();
#line 112

#line 112
  return __nesc_result;
#line 112
}
#line 112
# 90 "/home/csilab/tinyos/tos/chips/cc2420/lowpan/CC2420TinyosNetworkP.nc"
static inline uint8_t CC2420TinyosNetworkP__ActiveSend__maxPayloadLength(void )
#line 90
{
  return CC2420TinyosNetworkP__SubSend__maxPayloadLength();
}

# 112 "/home/csilab/tinyos/tos/interfaces/Send.nc"
inline static uint8_t CC2420ActiveMessageP__SubSend__maxPayloadLength(void ){
#line 112
  unsigned char __nesc_result;
#line 112

#line 112
  __nesc_result = CC2420TinyosNetworkP__ActiveSend__maxPayloadLength();
#line 112

#line 112
  return __nesc_result;
#line 112
}
#line 112
# 216 "/home/csilab/upma/chips/cc2420/CC2420ActiveMessageP.nc"
static inline uint8_t CC2420ActiveMessageP__Packet__maxPayloadLength(void )
#line 216
{
  return CC2420ActiveMessageP__SubSend__maxPayloadLength();
}

# 310 "/home/csilab/tinyos/tos/chips/cc2420/control/CC2420ControlP.nc"
static inline uint16_t CC2420ControlP__CC2420Config__getPanAddr(void )
#line 310
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 311
    {
      unsigned int __nesc_temp = 
#line 311
      CC2420ControlP__m_pan;

      {
#line 311
        __nesc_atomic_end(__nesc_atomic); 
#line 311
        return __nesc_temp;
      }
    }
#line 313
    __nesc_atomic_end(__nesc_atomic); }
}

# 77 "/home/csilab/tinyos/tos/chips/cc2420/interfaces/CC2420Config.nc"
inline static uint16_t CC2420ActiveMessageP__CC2420Config__getPanAddr(void ){
#line 77
  unsigned int __nesc_result;
#line 77

#line 77
  __nesc_result = CC2420ControlP__CC2420Config__getPanAddr();
#line 77

#line 77
  return __nesc_result;
#line 77
}
#line 77
# 53 "/home/csilab/tinyos/tos/interfaces/ResourceQueue.nc"
inline static bool CC2420TinyosNetworkP__Queue__isEmpty(void ){
#line 53
  unsigned char __nesc_result;
#line 53

#line 53
  __nesc_result = /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__FcfsQueue__isEmpty();
#line 53

#line 53
  return __nesc_result;
#line 53
}
#line 53
# 215 "/home/csilab/tinyos/tos/chips/cc2420/lowpan/CC2420TinyosNetworkP.nc"
static inline error_t CC2420TinyosNetworkP__Resource__immediateRequest(uint8_t id)
#line 215
{
  if (CC2420TinyosNetworkP__resource_owner == id) {
#line 216
    return EALREADY;
    }
  if (CC2420TinyosNetworkP__TINYOS_N_NETWORKS > 1) {
      if (CC2420TinyosNetworkP__resource_owner == CC2420TinyosNetworkP__OWNER_NONE && CC2420TinyosNetworkP__Queue__isEmpty()) {
          CC2420TinyosNetworkP__resource_owner = id;
          return SUCCESS;
        }
      return FAIL;
    }
  else 
#line 224
    {
      CC2420TinyosNetworkP__resource_owner = id;
      return SUCCESS;
    }
}

# 97 "/home/csilab/tinyos/tos/interfaces/Resource.nc"
inline static error_t CC2420ActiveMessageP__RadioResource__immediateRequest(void ){
#line 97
  unsigned char __nesc_result;
#line 97

#line 97
  __nesc_result = CC2420TinyosNetworkP__Resource__immediateRequest(CC2420ActiveMessageC__CC2420_AM_SEND_ID);
#line 97

#line 97
  return __nesc_result;
#line 97
}
#line 97
# 280 "/home/csilab/upma/chips/cc2420/CC2420ActiveMessageP.nc"
static inline void CC2420ActiveMessageP__SendNotifier__default__aboutToSend(am_id_t amId, am_addr_t addr, message_t *msg)
#line 280
{
}

# 59 "/home/csilab/tinyos/tos/interfaces/SendNotifier.nc"
inline static void CC2420ActiveMessageP__SendNotifier__aboutToSend(am_id_t arg_0x7f7fec07d240, am_addr_t dest, message_t * msg){
#line 59
    CC2420ActiveMessageP__SendNotifier__default__aboutToSend(arg_0x7f7fec07d240, dest, msg);
#line 59
}
#line 59
# 75 "/home/csilab/tinyos/tos/interfaces/Send.nc"
inline static error_t CC2420ActiveMessageP__SubSend__send(message_t * msg, uint8_t len){
#line 75
  unsigned char __nesc_result;
#line 75

#line 75
  __nesc_result = CC2420TinyosNetworkP__ActiveSend__send(msg, len);
#line 75

#line 75
  return __nesc_result;
#line 75
}
#line 75
# 180 "/home/csilab/upma/chips/cc2420/CC2420PacketP.nc"
static inline void CC2420PacketP__CC2420Packet__setNetwork(message_t *p_msg, uint8_t networkId)
#line 180
{

  __nesc_hton_leuint8(CC2420PacketP__CC2420PacketBody__getHeader(p_msg)->network.nxdata, networkId);
}

# 77 "/home/csilab/tinyos/tos/chips/cc2420/interfaces/CC2420Packet.nc"
inline static void CC2420TinyosNetworkP__CC2420Packet__setNetwork(message_t * p_msg, uint8_t networkId){
#line 77
  CC2420PacketP__CC2420Packet__setNetwork(p_msg, networkId);
#line 77
}
#line 77
# 549 "/home/csilab/upma/lib/macs/pure-tdma/PureTDMASchedulerP.nc"
static inline error_t PureTDMASchedulerP__Send__send(message_t *msg, uint8_t len)
#line 549
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 550
    {
      if (PureTDMASchedulerP__toSend == (void *)0) {
          PureTDMASchedulerP__toSend = msg;
          PureTDMASchedulerP__toSendLen = len;
          {
            unsigned char __nesc_temp = 
#line 554
            SUCCESS;

            {
#line 554
              __nesc_atomic_end(__nesc_atomic); 
#line 554
              return __nesc_temp;
            }
          }
        }
    }
#line 558
    __nesc_atomic_end(__nesc_atomic); }
#line 557
  return FAIL;
}

# 40 "/home/csilab/upma/interfaces/AsyncSend.nc"
inline static error_t AsyncSendAdapterP__AsyncSend__send(message_t * msg, uint8_t len){
#line 40
  unsigned char __nesc_result;
#line 40

#line 40
  __nesc_result = PureTDMASchedulerP__Send__send(msg, len);
#line 40

#line 40
  return __nesc_result;
#line 40
}
#line 40
# 41 "/home/csilab/upma/system/AsyncSendAdapterP.nc"
static inline error_t AsyncSendAdapterP__Send__send(message_t *msg, uint8_t len)
{
  return AsyncSendAdapterP__AsyncSend__send(msg, len);
}

# 75 "/home/csilab/tinyos/tos/interfaces/Send.nc"
inline static error_t UniqueSendP__SubSend__send(message_t * msg, uint8_t len){
#line 75
  unsigned char __nesc_result;
#line 75

#line 75
  __nesc_result = AsyncSendAdapterP__Send__send(msg, len);
#line 75

#line 75
  return __nesc_result;
#line 75
}
#line 75
# 42 "/home/csilab/tinyos/tos/chips/cc2420/interfaces/CC2420PacketBody.nc"
inline static cc2420_header_t * UniqueSendP__CC2420PacketBody__getHeader(message_t * msg){
#line 42
  nx_struct cc2420_header_t *__nesc_result;
#line 42

#line 42
  __nesc_result = CC2420PacketP__CC2420PacketBody__getHeader(msg);
#line 42

#line 42
  return __nesc_result;
#line 42
}
#line 42
# 45 "/home/csilab/tinyos/tos/interfaces/State.nc"
inline static error_t UniqueSendP__State__requestState(uint8_t reqState){
#line 45
  unsigned char __nesc_result;
#line 45

#line 45
  __nesc_result = StateImplP__State__requestState(2U, reqState);
#line 45

#line 45
  return __nesc_result;
#line 45
}
#line 45
# 75 "/home/csilab/tinyos/tos/chips/cc2420/unique/UniqueSendP.nc"
static inline error_t UniqueSendP__Send__send(message_t *msg, uint8_t len)
#line 75
{
  error_t error;

#line 77
  if (UniqueSendP__State__requestState(UniqueSendP__S_SENDING) == SUCCESS) {
      __nesc_hton_leuint8(UniqueSendP__CC2420PacketBody__getHeader(msg)->dsn.nxdata, UniqueSendP__localSendId++);

      if ((error = UniqueSendP__SubSend__send(msg, len)) != SUCCESS) {
          UniqueSendP__State__toIdle();
        }

      return error;
    }

  return EBUSY;
}

# 75 "/home/csilab/tinyos/tos/interfaces/Send.nc"
inline static error_t CC2420TinyosNetworkP__SubSend__send(message_t * msg, uint8_t len){
#line 75
  unsigned char __nesc_result;
#line 75

#line 75
  __nesc_result = UniqueSendP__Send__send(msg, len);
#line 75

#line 75
  return __nesc_result;
#line 75
}
#line 75
# 64 "/home/csilab/tinyos/tos/system/FcfsResourceQueueC.nc"
static inline bool /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__FcfsQueue__isEnqueued(resource_client_id_t id)
#line 64
{
  /* atomic removed: atomic calls only */
#line 65
  {
    unsigned char __nesc_temp = 
#line 65
    /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__resQ[id] != /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__NO_ENTRY || /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__qTail == id;

#line 65
    return __nesc_temp;
  }
}

#line 82
static inline error_t /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__FcfsQueue__enqueue(resource_client_id_t id)
#line 82
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 83
    {
      if (!/*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__FcfsQueue__isEnqueued(id)) {
          if (/*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__qHead == /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__NO_ENTRY) {
            /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__qHead = id;
            }
          else {
#line 88
            /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__resQ[/*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__qTail] = id;
            }
#line 89
          /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__qTail = id;
          {
            unsigned char __nesc_temp = 
#line 90
            SUCCESS;

            {
#line 90
              __nesc_atomic_end(__nesc_atomic); 
#line 90
              return __nesc_temp;
            }
          }
        }
#line 92
      {
        unsigned char __nesc_temp = 
#line 92
        EBUSY;

        {
#line 92
          __nesc_atomic_end(__nesc_atomic); 
#line 92
          return __nesc_temp;
        }
      }
    }
#line 95
    __nesc_atomic_end(__nesc_atomic); }
}

# 79 "/home/csilab/tinyos/tos/interfaces/ResourceQueue.nc"
inline static error_t CC2420TinyosNetworkP__Queue__enqueue(resource_client_id_t id){
#line 79
  unsigned char __nesc_result;
#line 79

#line 79
  __nesc_result = /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__FcfsQueue__enqueue(id);
#line 79

#line 79
  return __nesc_result;
#line 79
}
#line 79
# 67 "/home/csilab/tinyos/tos/interfaces/TaskBasic.nc"
inline static error_t CC2420TinyosNetworkP__grantTask__postTask(void ){
#line 67
  unsigned char __nesc_result;
#line 67

#line 67
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(CC2420TinyosNetworkP__grantTask);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 199 "/home/csilab/tinyos/tos/chips/cc2420/lowpan/CC2420TinyosNetworkP.nc"
static inline error_t CC2420TinyosNetworkP__Resource__request(uint8_t id)
#line 199
{

  CC2420TinyosNetworkP__grantTask__postTask();

  if (CC2420TinyosNetworkP__TINYOS_N_NETWORKS > 1) {
      return CC2420TinyosNetworkP__Queue__enqueue(id);
    }
  else 
#line 205
    {
      if (id == CC2420TinyosNetworkP__resource_owner) {
          return EALREADY;
        }
      else 
#line 208
        {
          CC2420TinyosNetworkP__next_owner = id;
          return SUCCESS;
        }
    }
}

# 88 "/home/csilab/tinyos/tos/interfaces/Resource.nc"
inline static error_t CC2420ActiveMessageP__RadioResource__request(void ){
#line 88
  unsigned char __nesc_result;
#line 88

#line 88
  __nesc_result = CC2420TinyosNetworkP__Resource__request(CC2420ActiveMessageC__CC2420_AM_SEND_ID);
#line 88

#line 88
  return __nesc_result;
#line 88
}
#line 88
# 64 "/home/csilab/tinyos/tos/lib/timer/Counter.nc"
inline static /*TestPureTdmaC.SendingApp.App*/SendingC__0__Counter__size_type /*TestPureTdmaC.SendingApp.App*/SendingC__0__Counter__get(void ){
#line 64
  unsigned long __nesc_result;
#line 64

#line 64
  __nesc_result = /*Counter32khz32C.Transform*/TransformCounterC__0__Counter__get();
#line 64

#line 64
  return __nesc_result;
#line 64
}
#line 64
# 67 "/home/csilab/tinyos/tos/interfaces/TaskBasic.nc"
inline static error_t /*TestPureTdmaC.SendingApp.App*/SendingC__0__sendTask__postTask(void ){
#line 67
  unsigned char __nesc_result;
#line 67

#line 67
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(/*TestPureTdmaC.SendingApp.App*/SendingC__0__sendTask);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 212 "/home/csilab/upma/chips/cc2420/CC2420ActiveMessageP.nc"
static inline void CC2420ActiveMessageP__Packet__setPayloadLength(message_t *msg, uint8_t len)
#line 212
{
  __nesc_hton_leuint8(CC2420ActiveMessageP__CC2420PacketBody__getHeader(msg)->length.nxdata, len + CC2420_SIZE);
}

# 94 "/home/csilab/tinyos/tos/interfaces/Packet.nc"
inline static void /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__Packet__setPayloadLength(message_t * msg, uint8_t len){
#line 94
  CC2420ActiveMessageP__Packet__setPayloadLength(msg, len);
#line 94
}
#line 94
# 90 "/home/csilab/tinyos/tos/system/AMQueueImplP.nc"
static inline error_t /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__Send__send(uint8_t clientId, message_t *msg, 
uint8_t len)
#line 91
{
  if (clientId >= 1) {
      return FAIL;
    }
  if (/*AMQueueP.AMQueueImplP*/AMQueueImplP__1__queue[clientId].msg != (void *)0) {
      return EBUSY;
    }
  ;

  /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__queue[clientId].msg = msg;
  /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__Packet__setPayloadLength(msg, len);

  if (/*AMQueueP.AMQueueImplP*/AMQueueImplP__1__current >= 1) {
      error_t err;
      am_id_t amId = /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__AMPacket__type(msg);
      am_addr_t dest = /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__AMPacket__destination(msg);

      ;
      /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__current = clientId;

      err = /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__AMSend__send(amId, dest, msg, len);
      if (err != SUCCESS) {
          ;
          /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__current = 1;
          /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__queue[clientId].msg = (void *)0;
        }

      return err;
    }
  else {
      ;
    }
  return SUCCESS;
}

# 75 "/home/csilab/tinyos/tos/interfaces/Send.nc"
inline static error_t /*TestPureTdmaC.SendingApp.AMSender.SenderC.AMQueueEntryP*/AMQueueEntryP__1__Send__send(message_t * msg, uint8_t len){
#line 75
  unsigned char __nesc_result;
#line 75

#line 75
  __nesc_result = /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__Send__send(0U, msg, len);
#line 75

#line 75
  return __nesc_result;
#line 75
}
#line 75
# 151 "/home/csilab/upma/interfaces/AMPacket.nc"
inline static void /*TestPureTdmaC.SendingApp.AMSender.SenderC.AMQueueEntryP*/AMQueueEntryP__1__AMPacket__setType(message_t * amsg, am_id_t t){
#line 151
  CC2420ActiveMessageP__AMPacket__setType(amsg, t);
#line 151
}
#line 151
#line 92
inline static void /*TestPureTdmaC.SendingApp.AMSender.SenderC.AMQueueEntryP*/AMQueueEntryP__1__AMPacket__setDestination(message_t * amsg, am_addr_t addr){
#line 92
  CC2420ActiveMessageP__AMPacket__setDestination(amsg, addr);
#line 92
}
#line 92
# 53 "/home/csilab/tinyos/tos/system/AMQueueEntryP.nc"
static inline error_t /*TestPureTdmaC.SendingApp.AMSender.SenderC.AMQueueEntryP*/AMQueueEntryP__1__AMSend__send(am_addr_t dest, 
message_t *msg, 
uint8_t len)
#line 55
{
  /*TestPureTdmaC.SendingApp.AMSender.SenderC.AMQueueEntryP*/AMQueueEntryP__1__AMPacket__setDestination(msg, dest);
  /*TestPureTdmaC.SendingApp.AMSender.SenderC.AMQueueEntryP*/AMQueueEntryP__1__AMPacket__setType(msg, 240);
  return /*TestPureTdmaC.SendingApp.AMSender.SenderC.AMQueueEntryP*/AMQueueEntryP__1__Send__send(msg, len);
}

# 80 "/home/csilab/tinyos/tos/interfaces/AMSend.nc"
inline static error_t /*TestPureTdmaC.SendingApp.App*/SendingC__0__AMSender__send(am_addr_t addr, message_t * msg, uint8_t len){
#line 80
  unsigned char __nesc_result;
#line 80

#line 80
  __nesc_result = /*TestPureTdmaC.SendingApp.AMSender.SenderC.AMQueueEntryP*/AMQueueEntryP__1__AMSend__send(addr, msg, len);
#line 80

#line 80
  return __nesc_result;
#line 80
}
#line 80
# 67 "SendingC.nc"
static inline void /*TestPureTdmaC.SendingApp.App*/SendingC__0__sendTask__runTask(void )
#line 67
{

  if (/*TestPureTdmaC.SendingApp.App*/SendingC__0__sends == FALSE) 
    {



      if (/*TestPureTdmaC.SendingApp.App*/SendingC__0__AMSender__send(AM_BROADCAST_ADDR, &/*TestPureTdmaC.SendingApp.App*/SendingC__0__packet, 2 * sizeof(uint8_t )) != SUCCESS) {
          /*TestPureTdmaC.SendingApp.App*/SendingC__0__sendTask__postTask();
        }
      else 
#line 76
        {
          /*TestPureTdmaC.SendingApp.App*/SendingC__0__sends = TRUE;
          /*TestPureTdmaC.SendingApp.App*/SendingC__0__sendTime = /*TestPureTdmaC.SendingApp.App*/SendingC__0__Counter__get();
        }
    }
}

#line 113
static inline void /*TestPureTdmaC.SendingApp.App*/SendingC__0__flush__runTask(void )
#line 113
{
}

# 36 "/home/csilab/upma/system/DummyChannelMonitorC.nc"
static inline void DummyChannelMonitorC__ChannelMonitor__free(void )
#line 36
{
}

# 48 "/home/csilab/upma/interfaces/ChannelMonitor.nc"
inline static void PowerCycleP__ChannelMonitor__free(void ){
#line 48
  DummyChannelMonitorC__ChannelMonitor__free();
#line 48
}
#line 48
# 34 "/home/csilab/upma/system/DummyChannelMonitorC.nc"
static inline void DummyChannelMonitorC__ChannelMonitor__busy(void )
#line 34
{
}

# 53 "/home/csilab/upma/interfaces/ChannelMonitor.nc"
inline static void PowerCycleP__ChannelMonitor__busy(void ){
#line 53
  DummyChannelMonitorC__ChannelMonitor__busy();
#line 53
}
#line 53
# 251 "/home/csilab/upma/chips/cc2420/CC2420TransmitP.nc"
static inline bool CC2420TransmitP__EnergyIndicator__isReceiving(void )
#line 251
{
  return !CC2420TransmitP__CCA__get();
}

# 43 "/home/csilab/tinyos/tos/chips/cc2420/interfaces/ReceiveIndicator.nc"
inline static bool PowerCycleP__EnergyIndicator__isReceiving(void ){
#line 43
  unsigned char __nesc_result;
#line 43

#line 43
  __nesc_result = CC2420TransmitP__EnergyIndicator__isReceiving();
#line 43

#line 43
  return __nesc_result;
#line 43
}
#line 43
# 178 "/home/csilab/upma/chips/cc2420/CC2420ReceiveP.nc"
static inline bool CC2420ReceiveP__PacketIndicator__isReceiving(void )
#line 178
{
  bool receiving;

#line 180
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 180
    {
      receiving = CC2420ReceiveP__receivingPacket;
    }
#line 182
    __nesc_atomic_end(__nesc_atomic); }
  return receiving;
}

# 43 "/home/csilab/tinyos/tos/chips/cc2420/interfaces/ReceiveIndicator.nc"
inline static bool PowerCycleP__PacketIndicator__isReceiving(void ){
#line 43
  unsigned char __nesc_result;
#line 43

#line 43
  __nesc_result = CC2420ReceiveP__PacketIndicator__isReceiving();
#line 43

#line 43
  return __nesc_result;
#line 43
}
#line 43
# 104 "/home/csilab/upma/system/LocalTime16P.nc"
static inline bool /*LocalTime32khz16C.LocalTimeP*/LocalTime16P__0__LocalTime__lessThan(local_time16_t *t1, local_time16_t *t2)
#line 104
{
  return __nesc_ntoh_uint32(t1->mticks.nxdata) < __nesc_ntoh_uint32(t2->mticks.nxdata) || (__nesc_ntoh_uint32(t1->mticks.nxdata) == __nesc_ntoh_uint32(t2->mticks.nxdata) && __nesc_ntoh_uint16(t1->sticks.nxdata) < __nesc_ntoh_uint16(t2->sticks.nxdata));
}

# 37 "/home/csilab/upma/interfaces/LocalTimeExtended.nc"
inline static bool PowerCycleP__Time__lessThan(PowerCycleP__Time__local_time_t *t1, PowerCycleP__Time__local_time_t *t2){
#line 37
  unsigned char __nesc_result;
#line 37

#line 37
  __nesc_result = /*LocalTime32khz16C.LocalTimeP*/LocalTime16P__0__LocalTime__lessThan(t1, t2);
#line 37

#line 37
  return __nesc_result;
#line 37
}
#line 37
#line 33
inline static PowerCycleP__Time__local_time_t PowerCycleP__Time__getNow(void ){
#line 33
  nx_struct local_time16 __nesc_result;
#line 33

#line 33
  __nesc_result = /*LocalTime32khz16C.LocalTimeP*/LocalTime16P__0__LocalTime__getNow();
#line 33

#line 33
  return __nesc_result;
#line 33
}
#line 33
# 61 "/home/csilab/upma/system/LocalTime16P.nc"
static inline local_time16_t /*LocalTime32khz16C.LocalTimeP*/LocalTime16P__0__LocalTime__add(local_time16_t *t1, local_time16_t *t2)
#line 61
{
  unsigned long __nesc_temp49;
  unsigned char *__nesc_temp48;
#line 62
  local_time16_t added;
  uint32_t sticks = (uint32_t )__nesc_ntoh_uint16(t1->sticks.nxdata) + (uint32_t )__nesc_ntoh_uint16(t2->sticks.nxdata);

  __nesc_hton_uint32(added.mticks.nxdata, __nesc_ntoh_uint32(t1->mticks.nxdata) + __nesc_ntoh_uint32(t2->mticks.nxdata));
  if (sticks > 0xFFFFL) {
    (__nesc_temp48 = added.mticks.nxdata, __nesc_hton_uint32(__nesc_temp48, (__nesc_temp49 = __nesc_ntoh_uint32(__nesc_temp48)) + 1), __nesc_temp49);
    }
#line 68
  __nesc_hton_uint16(added.sticks.nxdata, sticks & 0xFFFFL);

  return added;
}

# 34 "/home/csilab/upma/interfaces/LocalTimeExtended.nc"
inline static PowerCycleP__Time__local_time_t PowerCycleP__Time__add(PowerCycleP__Time__local_time_t *t1, PowerCycleP__Time__local_time_t *t2){
#line 34
  nx_struct local_time16 __nesc_result;
#line 34

#line 34
  __nesc_result = /*LocalTime32khz16C.LocalTimeP*/LocalTime16P__0__LocalTime__add(t1, t2);
#line 34

#line 34
  return __nesc_result;
#line 34
}
#line 34
# 146 "/home/csilab/upma/chips/cc2420/PowerCycleP.nc"
static inline void PowerCycleP__getCca__runTask(void )
#line 146
{
  uint8_t detects = 0;
#line 172
  local_time16_t startAt;
#line 172
  local_time16_t endAt;
#line 172
  local_time16_t length;
#line 172
  local_time16_t now;
  uint16_t count = 0;

  __nesc_hton_uint32(length.mticks.nxdata, 0);
  __nesc_hton_uint16(length.sticks.nxdata, PowerCycleP__ccaCheckLength);
  startAt = PowerCycleP__Time__getNow();
  endAt = PowerCycleP__Time__add(&startAt, &length);

  while (TRUE) {
      count++;
      if (count % 32 == 0) {
          now = PowerCycleP__Time__getNow();
          if (!PowerCycleP__Time__lessThan(&now, &endAt)) {
              break;
            }
        }

      if (PowerCycleP__PacketIndicator__isReceiving()) {
          detects = 3 + 1;
          break;
        }

      if (PowerCycleP__EnergyIndicator__isReceiving()) {
          detects++;
          if (detects > 3) {
              break;
            }
        }
    }


  if (detects > 3) {
      PowerCycleP__detected++;
      PowerCycleP__ChannelMonitor__busy();
    }
  else 
#line 206
    {
      PowerCycleP__free++;
      PowerCycleP__ChannelMonitor__free();
    }
}

# 64 "/home/csilab/tinyos/tos/lib/timer/Counter.nc"
inline static /*LocalTime32khz16C.LocalTimeP*/LocalTime16P__0__Counter__size_type /*LocalTime32khz16C.LocalTimeP*/LocalTime16P__0__Counter__get(void ){
#line 64
  unsigned int __nesc_result;
#line 64

#line 64
  __nesc_result = /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__get();
#line 64

#line 64
  return __nesc_result;
#line 64
}
#line 64
# 67 "/home/csilab/tinyos/tos/interfaces/TaskBasic.nc"
inline static error_t PowerCycleP__startRadio__postTask(void ){
#line 67
  unsigned char __nesc_result;
#line 67

#line 67
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(PowerCycleP__startRadio);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
inline static error_t PowerCycleP__getCca__postTask(void ){
#line 67
  unsigned char __nesc_result;
#line 67

#line 67
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(PowerCycleP__getCca);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 35 "/home/csilab/upma/interfaces/RadioPowerControl.nc"
inline static error_t PowerCycleP__RadioPowerControl__start(void ){
#line 35
  unsigned char __nesc_result;
#line 35

#line 35
  __nesc_result = CC2420CsmaP__RadioPowerControl__start();
#line 35

#line 35
  return __nesc_result;
#line 35
}
#line 35
# 136 "/home/csilab/upma/chips/cc2420/PowerCycleP.nc"
static inline void PowerCycleP__startRadio__runTask(void )
#line 136
{
  error_t err;

#line 138
  err = PowerCycleP__RadioPowerControl__start();
  if (err == EALREADY) {
      PowerCycleP__getCca__postTask();
    }
  else {
#line 141
    if (err != SUCCESS) {
        PowerCycleP__startRadio__postTask();
      }
    }
}

# 45 "/home/csilab/tinyos/tos/interfaces/State.nc"
inline static error_t CC2420CsmaP__SplitControlState__requestState(uint8_t reqState){
#line 45
  unsigned char __nesc_result;
#line 45

#line 45
  __nesc_result = StateImplP__State__requestState(1U, reqState);
#line 45

#line 45
  return __nesc_result;
#line 45
}
#line 45
# 66 "/home/csilab/tinyos/tos/lib/timer/Alarm.nc"
inline static void CC2420ControlP__StartupTimer__start(CC2420ControlP__StartupTimer__size_type dt){
#line 66
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Alarm__start(dt);
#line 66
}
#line 66
# 56 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P45*/HplMsp430GeneralIOP__29__IO__set(void )
#line 56
{
#line 56
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 56
    * (volatile uint8_t * )29U |= 0x01 << 5;
#line 56
    __nesc_atomic_end(__nesc_atomic); }
}

# 48 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*HplCC2420PinsC.VRENM*/Msp430GpioC__9__HplGeneralIO__set(void ){
#line 48
  /*HplMsp430GeneralIOC.P45*/HplMsp430GeneralIOP__29__IO__set();
#line 48
}
#line 48
# 48 "/home/csilab/tinyos/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplCC2420PinsC.VRENM*/Msp430GpioC__9__GeneralIO__set(void )
#line 48
{
#line 48
  /*HplCC2420PinsC.VRENM*/Msp430GpioC__9__HplGeneralIO__set();
}

# 40 "/home/csilab/tinyos/tos/interfaces/GeneralIO.nc"
inline static void CC2420ControlP__VREN__set(void ){
#line 40
  /*HplCC2420PinsC.VRENM*/Msp430GpioC__9__GeneralIO__set();
#line 40
}
#line 40
# 204 "/home/csilab/tinyos/tos/chips/cc2420/control/CC2420ControlP.nc"
static inline error_t CC2420ControlP__CC2420Power__startVReg(void )
#line 204
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 205
    {
      if (CC2420ControlP__m_state != CC2420ControlP__S_VREG_STOPPED) {
          {
            unsigned char __nesc_temp = 
#line 207
            FAIL;

            {
#line 207
              __nesc_atomic_end(__nesc_atomic); 
#line 207
              return __nesc_temp;
            }
          }
        }
#line 209
      CC2420ControlP__m_state = CC2420ControlP__S_VREG_STARTING;
    }
#line 210
    __nesc_atomic_end(__nesc_atomic); }
  CC2420ControlP__VREN__set();
  CC2420ControlP__StartupTimer__start(CC2420_TIME_VREN);
  return SUCCESS;
}

# 51 "/home/csilab/tinyos/tos/chips/cc2420/interfaces/CC2420Power.nc"
inline static error_t CC2420CsmaP__CC2420Power__startVReg(void ){
#line 51
  unsigned char __nesc_result;
#line 51

#line 51
  __nesc_result = CC2420ControlP__CC2420Power__startVReg();
#line 51

#line 51
  return __nesc_result;
#line 51
}
#line 51
# 244 "/home/csilab/tinyos/tos/chips/cc2420/lowpan/CC2420TinyosNetworkP.nc"
static inline void CC2420TinyosNetworkP__BareSend__default__sendDone(message_t *msg, error_t error)
#line 244
{
}

# 100 "/home/csilab/tinyos/tos/interfaces/Send.nc"
inline static void CC2420TinyosNetworkP__BareSend__sendDone(message_t * msg, error_t error){
#line 100
  CC2420TinyosNetworkP__BareSend__default__sendDone(msg, error);
#line 100
}
#line 100
# 110 "/home/csilab/tinyos/tos/interfaces/AMSend.nc"
inline static void CC2420ActiveMessageP__AMSend__sendDone(am_id_t arg_0x7f7fec083320, message_t * msg, error_t error){
#line 110
  /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__AMSend__sendDone(arg_0x7f7fec083320, msg, error);
#line 110
}
#line 110
# 229 "/home/csilab/tinyos/tos/chips/cc2420/lowpan/CC2420TinyosNetworkP.nc"
static inline error_t CC2420TinyosNetworkP__Resource__release(uint8_t id)
#line 229
{
  if (CC2420TinyosNetworkP__TINYOS_N_NETWORKS > 1) {
      CC2420TinyosNetworkP__grantTask__postTask();
    }
  CC2420TinyosNetworkP__resource_owner = CC2420TinyosNetworkP__OWNER_NONE;
  return SUCCESS;
}

# 120 "/home/csilab/tinyos/tos/interfaces/Resource.nc"
inline static error_t CC2420ActiveMessageP__RadioResource__release(void ){
#line 120
  unsigned char __nesc_result;
#line 120

#line 120
  __nesc_result = CC2420TinyosNetworkP__Resource__release(CC2420ActiveMessageC__CC2420_AM_SEND_ID);
#line 120

#line 120
  return __nesc_result;
#line 120
}
#line 120
# 226 "/home/csilab/upma/chips/cc2420/CC2420ActiveMessageP.nc"
static inline void CC2420ActiveMessageP__SubSend__sendDone(message_t *msg, error_t result)
#line 226
{
  CC2420ActiveMessageP__RadioResource__release();
  CC2420ActiveMessageP__AMSend__sendDone(CC2420ActiveMessageP__AMPacket__type(msg), msg, result);
}

# 100 "/home/csilab/tinyos/tos/interfaces/Send.nc"
inline static void CC2420TinyosNetworkP__ActiveSend__sendDone(message_t * msg, error_t error){
#line 100
  CC2420ActiveMessageP__SubSend__sendDone(msg, error);
#line 100
}
#line 100
# 148 "/home/csilab/tinyos/tos/chips/cc2420/lowpan/CC2420TinyosNetworkP.nc"
static inline void CC2420TinyosNetworkP__SubSend__sendDone(message_t *msg, error_t error)
#line 148
{
  if (CC2420TinyosNetworkP__m_busy_client == CC2420TinyosNetworkP__CLIENT_AM) {
      CC2420TinyosNetworkP__ActiveSend__sendDone(msg, error);
    }
  else 
#line 151
    {
      CC2420TinyosNetworkP__BareSend__sendDone(msg, error);
    }
}

# 100 "/home/csilab/tinyos/tos/interfaces/Send.nc"
inline static void UniqueSendP__Send__sendDone(message_t * msg, error_t error){
#line 100
  CC2420TinyosNetworkP__SubSend__sendDone(msg, error);
#line 100
}
#line 100
# 104 "/home/csilab/tinyos/tos/chips/cc2420/unique/UniqueSendP.nc"
static inline void UniqueSendP__SubSend__sendDone(message_t *msg, error_t error)
#line 104
{
  UniqueSendP__State__toIdle();
  UniqueSendP__Send__sendDone(msg, error);
}

# 100 "/home/csilab/tinyos/tos/interfaces/Send.nc"
inline static void AsyncSendAdapterP__Send__sendDone(message_t * msg, error_t error){
#line 100
  UniqueSendP__SubSend__sendDone(msg, error);
#line 100
}
#line 100
# 62 "/home/csilab/upma/system/AsyncSendAdapterP.nc"
static inline void AsyncSendAdapterP__sendDone_task__runTask(void )
{
  message_t *msg;
  uint8_t len;

#line 66
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      msg = AsyncSendAdapterP__msg_;
      len = AsyncSendAdapterP__len_;
    }
#line 70
    __nesc_atomic_end(__nesc_atomic); }


  AsyncSendAdapterP__Send__sendDone(msg, len);
}

# 42 "/home/csilab/tinyos/tos/chips/cc2420/interfaces/CC2420PacketBody.nc"
inline static cc2420_header_t * CC2420TinyosNetworkP__CC2420PacketBody__getHeader(message_t * msg){
#line 42
  nx_struct cc2420_header_t *__nesc_result;
#line 42

#line 42
  __nesc_result = CC2420PacketP__CC2420PacketBody__getHeader(msg);
#line 42

#line 42
  return __nesc_result;
#line 42
}
#line 42
# 138 "/home/csilab/tinyos/tos/chips/cc2420/lowpan/CC2420TinyosNetworkP.nc"
static inline void *CC2420TinyosNetworkP__BareSend__getPayload(message_t *msg, uint8_t len)
#line 138
{

  cc2420_header_t *hdr = CC2420TinyosNetworkP__CC2420PacketBody__getHeader(msg);

#line 141
  return hdr;
}

#line 241
static inline message_t *CC2420TinyosNetworkP__BareReceive__default__receive(message_t *msg, void *payload, uint8_t len)
#line 241
{
  return msg;
}

# 78 "/home/csilab/tinyos/tos/interfaces/Receive.nc"
inline static message_t * CC2420TinyosNetworkP__BareReceive__receive(message_t * msg, void * payload, uint8_t len){
#line 78
  nx_struct message_t *__nesc_result;
#line 78

#line 78
  __nesc_result = CC2420TinyosNetworkP__BareReceive__default__receive(msg, payload, len);
#line 78

#line 78
  return __nesc_result;
#line 78
}
#line 78
# 272 "/home/csilab/upma/chips/cc2420/CC2420ActiveMessageP.nc"
static inline message_t *CC2420ActiveMessageP__Snoop__default__receive(am_id_t id, message_t *msg, void *payload, uint8_t len)
#line 272
{
  return msg;
}

# 78 "/home/csilab/tinyos/tos/interfaces/Receive.nc"
inline static message_t * CC2420ActiveMessageP__Snoop__receive(am_id_t arg_0x7f7fec081280, message_t * msg, void * payload, uint8_t len){
#line 78
  nx_struct message_t *__nesc_result;
#line 78

#line 78
    __nesc_result = CC2420ActiveMessageP__Snoop__default__receive(arg_0x7f7fec081280, msg, payload, len);
#line 78

#line 78
  return __nesc_result;
#line 78
}
#line 78
# 130 "SendingC.nc"
static inline message_t */*TestPureTdmaC.SendingApp.App*/SendingC__0__AMReceiver__receive(message_t *message, void *payload, uint8_t length)
{

  /*TestPureTdmaC.SendingApp.App*/SendingC__0__packetsReceived++;
  return message;
}

# 268 "/home/csilab/upma/chips/cc2420/CC2420ActiveMessageP.nc"
static inline message_t *CC2420ActiveMessageP__Receive__default__receive(am_id_t id, message_t *msg, void *payload, uint8_t len)
#line 268
{
  return msg;
}

# 78 "/home/csilab/tinyos/tos/interfaces/Receive.nc"
inline static message_t * CC2420ActiveMessageP__Receive__receive(am_id_t arg_0x7f7fec082650, message_t * msg, void * payload, uint8_t len){
#line 78
  nx_struct message_t *__nesc_result;
#line 78

#line 78
  switch (arg_0x7f7fec082650) {
#line 78
    case 240:
#line 78
      __nesc_result = /*TestPureTdmaC.SendingApp.App*/SendingC__0__AMReceiver__receive(msg, payload, len);
#line 78
      break;
#line 78
    default:
#line 78
      __nesc_result = CC2420ActiveMessageP__Receive__default__receive(arg_0x7f7fec082650, msg, payload, len);
#line 78
      break;
#line 78
    }
#line 78

#line 78
  return __nesc_result;
#line 78
}
#line 78
# 72 "/home/csilab/tinyos/tos/system/ActiveMessageAddressC.nc"
static inline am_addr_t ActiveMessageAddressC__ActiveMessageAddress__amAddress(void )
#line 72
{
  return ActiveMessageAddressC__amAddress();
}

# 50 "/home/csilab/tinyos/tos/interfaces/ActiveMessageAddress.nc"
inline static am_addr_t CC2420ActiveMessageP__ActiveMessageAddress__amAddress(void ){
#line 50
  unsigned int __nesc_result;
#line 50

#line 50
  __nesc_result = ActiveMessageAddressC__ActiveMessageAddress__amAddress();
#line 50

#line 50
  return __nesc_result;
#line 50
}
#line 50
# 146 "/home/csilab/upma/chips/cc2420/CC2420ActiveMessageP.nc"
static inline am_addr_t CC2420ActiveMessageP__AMPacket__address(void )
#line 146
{
  return CC2420ActiveMessageP__ActiveMessageAddress__amAddress();
}

#line 170
static inline bool CC2420ActiveMessageP__AMPacket__isForMe(message_t *amsg)
#line 170
{
  return CC2420ActiveMessageP__AMPacket__destination(amsg) == CC2420ActiveMessageP__AMPacket__address() || 
  CC2420ActiveMessageP__AMPacket__destination(amsg) == AM_BROADCAST_ADDR;
}

#line 233
static inline message_t *CC2420ActiveMessageP__SubReceive__receive(message_t *msg, void *payload, uint8_t len)
#line 233
{

  if (CC2420ActiveMessageP__AMPacket__isForMe(msg)) {
      return CC2420ActiveMessageP__Receive__receive(CC2420ActiveMessageP__AMPacket__type(msg), msg, payload, len);
    }
  else {
      return CC2420ActiveMessageP__Snoop__receive(CC2420ActiveMessageP__AMPacket__type(msg), msg, payload, len);
    }
}

# 78 "/home/csilab/tinyos/tos/interfaces/Receive.nc"
inline static message_t * CC2420TinyosNetworkP__ActiveReceive__receive(message_t * msg, void * payload, uint8_t len){
#line 78
  nx_struct message_t *__nesc_result;
#line 78

#line 78
  __nesc_result = CC2420ActiveMessageP__SubReceive__receive(msg, payload, len);
#line 78

#line 78
  return __nesc_result;
#line 78
}
#line 78
# 53 "/home/csilab/tinyos/tos/chips/cc2420/interfaces/CC2420PacketBody.nc"
inline static cc2420_metadata_t * CC2420TinyosNetworkP__CC2420PacketBody__getMetadata(message_t * msg){
#line 53
  nx_struct cc2420_metadata_t *__nesc_result;
#line 53

#line 53
  __nesc_result = CC2420PacketP__CC2420PacketBody__getMetadata(msg);
#line 53

#line 53
  return __nesc_result;
#line 53
}
#line 53
# 172 "/home/csilab/upma/chips/cc2420/CC2420PacketP.nc"
static inline uint8_t CC2420PacketP__CC2420Packet__getNetwork(message_t *p_msg)
#line 172
{



  return __nesc_ntoh_leuint8(CC2420PacketP__CC2420PacketBody__getHeader(p_msg)->network.nxdata);
}

# 75 "/home/csilab/tinyos/tos/chips/cc2420/interfaces/CC2420Packet.nc"
inline static uint8_t CC2420TinyosNetworkP__CC2420Packet__getNetwork(message_t * p_msg){
#line 75
  unsigned char __nesc_result;
#line 75

#line 75
  __nesc_result = CC2420PacketP__CC2420Packet__getNetwork(p_msg);
#line 75

#line 75
  return __nesc_result;
#line 75
}
#line 75
# 157 "/home/csilab/tinyos/tos/chips/cc2420/lowpan/CC2420TinyosNetworkP.nc"
static inline message_t *CC2420TinyosNetworkP__SubReceive__receive(message_t *msg, void *payload, uint8_t len)
#line 157
{
  uint8_t network = CC2420TinyosNetworkP__CC2420Packet__getNetwork(msg);

  if (! __nesc_ntoh_int8(CC2420TinyosNetworkP__CC2420PacketBody__getMetadata(msg)->crc.nxdata)) {
      return msg;
    }

  if (network == 0x3f) {
      return CC2420TinyosNetworkP__ActiveReceive__receive(msg, payload, len);
    }
  else 
#line 166
    {
      return CC2420TinyosNetworkP__BareReceive__receive(msg, 
      CC2420TinyosNetworkP__BareSend__getPayload(msg, len), 
      len + sizeof(cc2420_header_t ));
    }
}

# 78 "/home/csilab/tinyos/tos/interfaces/Receive.nc"
inline static message_t * UniqueReceiveP__Receive__receive(message_t * msg, void * payload, uint8_t len){
#line 78
  nx_struct message_t *__nesc_result;
#line 78

#line 78
  __nesc_result = CC2420TinyosNetworkP__SubReceive__receive(msg, payload, len);
#line 78

#line 78
  return __nesc_result;
#line 78
}
#line 78
# 138 "/home/csilab/tinyos/tos/chips/cc2420/unique/UniqueReceiveP.nc"
static inline void UniqueReceiveP__insert(uint16_t msgSource, uint8_t msgDsn)
#line 138
{
  uint8_t element = UniqueReceiveP__recycleSourceElement;
  bool increment = FALSE;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 142
    {
      if (element == UniqueReceiveP__INVALID_ELEMENT || UniqueReceiveP__writeIndex == element) {

          element = UniqueReceiveP__writeIndex;
          increment = TRUE;
        }

      UniqueReceiveP__receivedMessages[element].source = msgSource;
      UniqueReceiveP__receivedMessages[element].dsn = msgDsn;
      if (increment) {
          UniqueReceiveP__writeIndex++;
          UniqueReceiveP__writeIndex %= 4;
        }
    }
#line 155
    __nesc_atomic_end(__nesc_atomic); }
}

#line 192
static inline message_t *UniqueReceiveP__DuplicateReceive__default__receive(message_t *msg, void *payload, uint8_t len)
#line 192
{
  return msg;
}

# 78 "/home/csilab/tinyos/tos/interfaces/Receive.nc"
inline static message_t * UniqueReceiveP__DuplicateReceive__receive(message_t * msg, void * payload, uint8_t len){
#line 78
  nx_struct message_t *__nesc_result;
#line 78

#line 78
  __nesc_result = UniqueReceiveP__DuplicateReceive__default__receive(msg, payload, len);
#line 78

#line 78
  return __nesc_result;
#line 78
}
#line 78
# 112 "/home/csilab/tinyos/tos/chips/cc2420/unique/UniqueReceiveP.nc"
static inline bool UniqueReceiveP__hasSeen(uint16_t msgSource, uint8_t msgDsn)
#line 112
{
  int i;

#line 114
  UniqueReceiveP__recycleSourceElement = UniqueReceiveP__INVALID_ELEMENT;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 116
    {
      for (i = 0; i < 4; i++) {
          if (UniqueReceiveP__receivedMessages[i].source == msgSource) {
              if (UniqueReceiveP__receivedMessages[i].dsn == msgDsn) {

                  {
                    unsigned char __nesc_temp = 
#line 121
                    TRUE;

                    {
#line 121
                      __nesc_atomic_end(__nesc_atomic); 
#line 121
                      return __nesc_temp;
                    }
                  }
                }
#line 124
              UniqueReceiveP__recycleSourceElement = i;
            }
        }
    }
#line 127
    __nesc_atomic_end(__nesc_atomic); }

  return FALSE;
}

# 42 "/home/csilab/tinyos/tos/chips/cc2420/interfaces/CC2420PacketBody.nc"
inline static cc2420_header_t * UniqueReceiveP__CC2420PacketBody__getHeader(message_t * msg){
#line 42
  nx_struct cc2420_header_t *__nesc_result;
#line 42

#line 42
  __nesc_result = CC2420PacketP__CC2420PacketBody__getHeader(msg);
#line 42

#line 42
  return __nesc_result;
#line 42
}
#line 42
# 165 "/home/csilab/tinyos/tos/chips/cc2420/unique/UniqueReceiveP.nc"
static inline uint16_t UniqueReceiveP__getSourceKey(message_t * msg)
#line 165
{
  cc2420_header_t *hdr = UniqueReceiveP__CC2420PacketBody__getHeader(msg);
  int s_mode = (__nesc_ntoh_leuint16(hdr->fcf.nxdata) >> IEEE154_FCF_SRC_ADDR_MODE) & 0x3;
  int d_mode = (__nesc_ntoh_leuint16(hdr->fcf.nxdata) >> IEEE154_FCF_DEST_ADDR_MODE) & 0x3;
  int s_offset = 2;
#line 169
  int s_len = 2;
  uint16_t key = 0;
  uint8_t *current = (uint8_t *)& hdr->dest;
  int i;

  if (s_mode == IEEE154_ADDR_EXT) {
      s_len = 8;
    }
  if (d_mode == IEEE154_ADDR_EXT) {
      s_offset = 8;
    }

  current += s_offset;

  for (i = 0; i < s_len; i++) {
      key += current[i];
    }
  return key;
}

#line 86
static inline message_t *UniqueReceiveP__SubReceive__receive(message_t *msg, void *payload, 
uint8_t len)
#line 87
{

  uint16_t msgSource = UniqueReceiveP__getSourceKey(msg);
  uint8_t msgDsn = __nesc_ntoh_leuint8(UniqueReceiveP__CC2420PacketBody__getHeader(msg)->dsn.nxdata);

  if (UniqueReceiveP__hasSeen(msgSource, msgDsn)) {
      return UniqueReceiveP__DuplicateReceive__receive(msg, payload, len);
    }
  else 
#line 94
    {
      UniqueReceiveP__insert(msgSource, msgDsn);
      return UniqueReceiveP__Receive__receive(msg, payload, len);
    }
}

# 78 "/home/csilab/tinyos/tos/interfaces/Receive.nc"
inline static message_t * AsyncReceiveAdapterP__Receive__receive(message_t * msg, void * payload, uint8_t len){
#line 78
  nx_struct message_t *__nesc_result;
#line 78

#line 78
  __nesc_result = UniqueReceiveP__SubReceive__receive(msg, payload, len);
#line 78

#line 78
  return __nesc_result;
#line 78
}
#line 78
# 52 "/home/csilab/upma/interfaces/AsyncReceive.nc"
inline static void BeaconSlotP__SubReceive__updateBuffer(message_t * msg){
#line 52
  CC2420ReceiveP__Receive__updateBuffer(msg);
#line 52
}
#line 52
# 140 "/home/csilab/upma/system/BeaconSlotP.nc"
static inline void BeaconSlotP__Receive__updateBuffer(message_t *msg)
#line 140
{
#line 140
  return BeaconSlotP__SubReceive__updateBuffer(msg);
}

# 52 "/home/csilab/upma/interfaces/AsyncReceive.nc"
inline static void PureTDMASchedulerP__SubReceive__updateBuffer(message_t * msg){
#line 52
  BeaconSlotP__Receive__updateBuffer(msg);
#line 52
}
#line 52
# 687 "/home/csilab/upma/lib/macs/pure-tdma/PureTDMASchedulerP.nc"
static inline void PureTDMASchedulerP__Receive__updateBuffer(message_t *msg)
#line 687
{
#line 687
  return PureTDMASchedulerP__SubReceive__updateBuffer(msg);
}

# 52 "/home/csilab/upma/interfaces/AsyncReceive.nc"
inline static void AsyncReceiveAdapterP__AsyncReceive__updateBuffer(message_t * msg){
#line 52
  PureTDMASchedulerP__Receive__updateBuffer(msg);
#line 52
}
#line 52
# 54 "/home/csilab/upma/system/AsyncReceiveAdapterP.nc"
static inline void AsyncReceiveAdapterP__receiveDone_task__runTask(void )
{
  message_t *msg;
  void *payload;
  uint8_t len;

#line 59
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      msg = AsyncReceiveAdapterP__msg_;
      payload = AsyncReceiveAdapterP__payload_;
      len = AsyncReceiveAdapterP__len_;
    }
#line 64
    __nesc_atomic_end(__nesc_atomic); }


  AsyncReceiveAdapterP__AsyncReceive__updateBuffer(AsyncReceiveAdapterP__Receive__receive(msg, payload, len));
}

# 178 "/home/csilab/tinyos/tos/chips/cc2420/spi/CC2420SpiP.nc"
static inline bool CC2420SpiP__Resource__isOwner(uint8_t id)
#line 178
{
  /* atomic removed: atomic calls only */
#line 179
  {
    unsigned char __nesc_temp = 
#line 179
    CC2420SpiP__m_holder == id;

#line 179
    return __nesc_temp;
  }
}

# 128 "/home/csilab/tinyos/tos/interfaces/Resource.nc"
inline static bool CC2420ReceiveP__SpiResource__isOwner(void ){
#line 128
  unsigned char __nesc_result;
#line 128

#line 128
  __nesc_result = CC2420SpiP__Resource__isOwner(/*CC2420ReceiveC.Spi*/CC2420SpiC__4__CLIENT_ID);
#line 128

#line 128
  return __nesc_result;
#line 128
}
#line 128
# 218 "/home/csilab/upma/chips/cc2420/PowerCycleP.nc"
static inline void PowerCycleP__PacketIndicator__anypktreceive(void )
#line 218
{
}

# 722 "/home/csilab/upma/lib/macs/pure-tdma/PureTDMASchedulerP.nc"
static inline void PureTDMASchedulerP__ReceiveIndicator__anypktreceive(void )
#line 722
{
  if (PureTDMASchedulerP__rcv_indicator == FALSE && PureTDMASchedulerP__my_txoffset[2] >= 5) {
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 724
        PureTDMASchedulerP__rcv_indicator = TRUE;
#line 724
        __nesc_atomic_end(__nesc_atomic); }
    }
}

# 44 "/home/csilab/tinyos/tos/chips/cc2420/interfaces/ReceiveIndicator.nc"
inline static void CC2420ReceiveP__PacketIndicator__anypktreceive(void ){
#line 44
  PureTDMASchedulerP__ReceiveIndicator__anypktreceive();
#line 44
  PowerCycleP__PacketIndicator__anypktreceive();
#line 44
}
#line 44
# 97 "/home/csilab/tinyos/tos/interfaces/Resource.nc"
inline static error_t CC2420ReceiveP__SpiResource__immediateRequest(void ){
#line 97
  unsigned char __nesc_result;
#line 97

#line 97
  __nesc_result = CC2420SpiP__Resource__immediateRequest(/*CC2420ReceiveC.Spi*/CC2420SpiC__4__CLIENT_ID);
#line 97

#line 97
  return __nesc_result;
#line 97
}
#line 97
#line 88
inline static error_t CC2420ReceiveP__SpiResource__request(void ){
#line 88
  unsigned char __nesc_result;
#line 88

#line 88
  __nesc_result = CC2420SpiP__Resource__request(/*CC2420ReceiveC.Spi*/CC2420SpiC__4__CLIENT_ID);
#line 88

#line 88
  return __nesc_result;
#line 88
}
#line 88
# 253 "/home/csilab/tinyos/tos/chips/cc2420/lowpan/CC2420TinyosNetworkP.nc"
static inline void CC2420TinyosNetworkP__Resource__default__granted(uint8_t client)
#line 253
{
  CC2420TinyosNetworkP__Resource__release(client);
}

# 102 "/home/csilab/tinyos/tos/interfaces/Resource.nc"
inline static void CC2420TinyosNetworkP__Resource__granted(uint8_t arg_0x7f7fec4cb600){
#line 102
  switch (arg_0x7f7fec4cb600) {
#line 102
    case CC2420ActiveMessageC__CC2420_AM_SEND_ID:
#line 102
      CC2420ActiveMessageP__RadioResource__granted();
#line 102
      break;
#line 102
    default:
#line 102
      CC2420TinyosNetworkP__Resource__default__granted(arg_0x7f7fec4cb600);
#line 102
      break;
#line 102
    }
#line 102
}
#line 102
# 68 "/home/csilab/tinyos/tos/system/FcfsResourceQueueC.nc"
static inline resource_client_id_t /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__FcfsQueue__dequeue(void )
#line 68
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 69
    {
      if (/*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__qHead != /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__NO_ENTRY) {
          uint8_t id = /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__qHead;

#line 72
          /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__qHead = /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__resQ[/*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__qHead];
          if (/*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__qHead == /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__NO_ENTRY) {
            /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__qTail = /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__NO_ENTRY;
            }
#line 75
          /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__resQ[id] = /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__NO_ENTRY;
          {
            unsigned char __nesc_temp = 
#line 76
            id;

            {
#line 76
              __nesc_atomic_end(__nesc_atomic); 
#line 76
              return __nesc_temp;
            }
          }
        }
#line 78
      {
        unsigned char __nesc_temp = 
#line 78
        /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__NO_ENTRY;

        {
#line 78
          __nesc_atomic_end(__nesc_atomic); 
#line 78
          return __nesc_temp;
        }
      }
    }
#line 81
    __nesc_atomic_end(__nesc_atomic); }
}

# 70 "/home/csilab/tinyos/tos/interfaces/ResourceQueue.nc"
inline static resource_client_id_t CC2420TinyosNetworkP__Queue__dequeue(void ){
#line 70
  unsigned char __nesc_result;
#line 70

#line 70
  __nesc_result = /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__FcfsQueue__dequeue();
#line 70

#line 70
  return __nesc_result;
#line 70
}
#line 70
# 180 "/home/csilab/tinyos/tos/chips/cc2420/lowpan/CC2420TinyosNetworkP.nc"
static inline void CC2420TinyosNetworkP__grantTask__runTask(void )
#line 180
{


  if (CC2420TinyosNetworkP__TINYOS_N_NETWORKS > 1) {
      if (CC2420TinyosNetworkP__resource_owner == CC2420TinyosNetworkP__OWNER_NONE && !CC2420TinyosNetworkP__Queue__isEmpty()) {
          CC2420TinyosNetworkP__resource_owner = CC2420TinyosNetworkP__Queue__dequeue();

          if (CC2420TinyosNetworkP__resource_owner != CC2420TinyosNetworkP__OWNER_NONE) {
              CC2420TinyosNetworkP__Resource__granted(CC2420TinyosNetworkP__resource_owner);
            }
        }
    }
  else 
#line 191
    {
      if (CC2420TinyosNetworkP__next_owner != CC2420TinyosNetworkP__resource_owner) {
          CC2420TinyosNetworkP__resource_owner = CC2420TinyosNetworkP__next_owner;
          CC2420TinyosNetworkP__Resource__granted(CC2420TinyosNetworkP__resource_owner);
        }
    }
}

# 103 "/home/csilab/tinyos/tos/lib/timer/Alarm.nc"
inline static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__startAt(/*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type t0, /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type dt){
#line 103
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__Alarm__startAt(t0, dt);
#line 103
}
#line 103
# 58 "/home/csilab/tinyos/tos/lib/timer/AlarmToTimerC.nc"
static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__start(uint32_t t0, uint32_t dt, bool oneshot)
{
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__m_dt = dt;
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__m_oneshot = oneshot;
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__startAt(t0, dt);
}

#line 93
static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__startOneShotAt(uint32_t t0, uint32_t dt)
{
#line 94
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__start(t0, dt, TRUE);
}

# 129 "/home/csilab/tinyos/tos/lib/timer/Timer.nc"
inline static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__startOneShotAt(uint32_t t0, uint32_t dt){
#line 129
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__startOneShotAt(t0, dt);
#line 129
}
#line 129
# 65 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Alarm__stop(void )
{
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430TimerControl__disableEvents();
}

# 73 "/home/csilab/tinyos/tos/lib/timer/Alarm.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__AlarmFrom__stop(void ){
#line 73
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Alarm__stop();
#line 73
}
#line 73
# 102 "/home/csilab/tinyos/tos/lib/timer/TransformAlarmC.nc"
static inline void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__Alarm__stop(void )
{
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__AlarmFrom__stop();
}

# 73 "/home/csilab/tinyos/tos/lib/timer/Alarm.nc"
inline static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__stop(void ){
#line 73
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__Alarm__stop();
#line 73
}
#line 73
# 71 "/home/csilab/tinyos/tos/lib/timer/AlarmToTimerC.nc"
static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__stop(void )
{
#line 72
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__stop();
}

# 78 "/home/csilab/tinyos/tos/lib/timer/Timer.nc"
inline static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__stop(void ){
#line 78
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__stop();
#line 78
}
#line 78
# 64 "/home/csilab/tinyos/tos/lib/timer/Counter.nc"
inline static /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__Counter__size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__Counter__get(void ){
#line 64
  unsigned long __nesc_result;
#line 64

#line 64
  __nesc_result = /*CounterMilli32C.Transform*/TransformCounterC__1__Counter__get();
#line 64

#line 64
  return __nesc_result;
#line 64
}
#line 64
# 86 "/home/csilab/tinyos/tos/lib/timer/TransformAlarmC.nc"
static inline /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__Alarm__getNow(void )
{
  return /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__Counter__get();
}

# 109 "/home/csilab/tinyos/tos/lib/timer/Alarm.nc"
inline static /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__getNow(void ){
#line 109
  unsigned long __nesc_result;
#line 109

#line 109
  __nesc_result = /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__Alarm__getNow();
#line 109

#line 109
  return __nesc_result;
#line 109
}
#line 109
# 96 "/home/csilab/tinyos/tos/lib/timer/AlarmToTimerC.nc"
static inline uint32_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__getNow(void )
{
#line 97
  return /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__getNow();
}

# 136 "/home/csilab/tinyos/tos/lib/timer/Timer.nc"
inline static uint32_t /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__getNow(void ){
#line 136
  unsigned long __nesc_result;
#line 136

#line 136
  __nesc_result = /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__getNow();
#line 136

#line 136
  return __nesc_result;
#line 136
}
#line 136
# 100 "/home/csilab/tinyos/tos/lib/timer/VirtualizeTimerC.nc"
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer__runTask(void )
{




  uint32_t now = /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__getNow();
  int32_t min_remaining = (1UL << 31) - 1;
  bool min_remaining_isset = FALSE;
  uint16_t num;

  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__stop();

  for (num = 0; num < /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__NUM_TIMERS; num++) 
    {
      /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer_t *timer = &/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__m_timers[num];

      if (timer->isrunning) 
        {
          uint32_t elapsed = now - timer->t0;
          int32_t remaining = timer->dt - elapsed;

          if (remaining < min_remaining) 
            {
              min_remaining = remaining;
              min_remaining_isset = TRUE;
            }
        }
    }

  if (min_remaining_isset) 
    {
      if (min_remaining <= 0) {
        /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__fireTimers(now);
        }
      else {
#line 135
        /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__startOneShotAt(now, min_remaining);
        }
    }
}

# 125 "SendingC.nc"
static inline void /*TestPureTdmaC.SendingApp.App*/SendingC__0__SendTimer__fired(void )
{
}

#line 121
static inline void /*TestPureTdmaC.SendingApp.App*/SendingC__0__FlushTimer__fired(void )
#line 121
{
}

#line 159
static inline void /*TestPureTdmaC.SendingApp.App*/SendingC__0__BenchmarkTimer__fired(void )
{

  uint32_t avg_latency = /*TestPureTdmaC.SendingApp.App*/SendingC__0__latency / /*TestPureTdmaC.SendingApp.App*/SendingC__0__packetsSent;


  /*TestPureTdmaC.SendingApp.App*/SendingC__0__packetsSent = 0;
  /*TestPureTdmaC.SendingApp.App*/SendingC__0__packetsReceived = 0;
  /*TestPureTdmaC.SendingApp.App*/SendingC__0__latency = 0;
}

# 64 "/home/csilab/tinyos/tos/lib/timer/Timer.nc"
inline static void /*TestPureTdmaC.SendingApp.App*/SendingC__0__BenchmarkTimer__startPeriodic(uint32_t dt){
#line 64
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startPeriodic(3U, dt);
#line 64
}
#line 64
# 154 "SendingC.nc"
static inline void /*TestPureTdmaC.SendingApp.App*/SendingC__0__StartTimer__fired(void )
{
  /*TestPureTdmaC.SendingApp.App*/SendingC__0__BenchmarkTimer__startPeriodic(/*TestPureTdmaC.SendingApp.App*/SendingC__0__BENCHMARK_LENGTH);
}

# 204 "/home/csilab/tinyos/tos/lib/timer/VirtualizeTimerC.nc"
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__default__fired(uint8_t num)
{
}

# 83 "/home/csilab/tinyos/tos/lib/timer/Timer.nc"
inline static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__fired(uint8_t arg_0x7f7fec5fd950){
#line 83
  switch (arg_0x7f7fec5fd950) {
#line 83
    case 1U:
#line 83
      /*TestPureTdmaC.SendingApp.App*/SendingC__0__SendTimer__fired();
#line 83
      break;
#line 83
    case 2U:
#line 83
      /*TestPureTdmaC.SendingApp.App*/SendingC__0__FlushTimer__fired();
#line 83
      break;
#line 83
    case 3U:
#line 83
      /*TestPureTdmaC.SendingApp.App*/SendingC__0__BenchmarkTimer__fired();
#line 83
      break;
#line 83
    case 4U:
#line 83
      /*TestPureTdmaC.SendingApp.App*/SendingC__0__StartTimer__fired();
#line 83
      break;
#line 83
    default:
#line 83
      /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__default__fired(arg_0x7f7fec5fd950);
#line 83
      break;
#line 83
    }
#line 83
}
#line 83
# 67 "/home/csilab/tinyos/tos/interfaces/TaskBasic.nc"
inline static error_t /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer__postTask(void ){
#line 67
  unsigned char __nesc_result;
#line 67

#line 67
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 139 "/home/csilab/tinyos/tos/lib/timer/VirtualizeTimerC.nc"
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__fired(void )
{
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__fireTimers(/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__getNow());
}

# 83 "/home/csilab/tinyos/tos/lib/timer/Timer.nc"
inline static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__fired(void ){
#line 83
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__fired();
#line 83
}
#line 83
# 91 "/home/csilab/tinyos/tos/lib/timer/TransformAlarmC.nc"
static inline /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__Alarm__getAlarm(void )
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 93
    {
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__to_size_type __nesc_temp = 
#line 93
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__m_t0 + /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__m_dt;

      {
#line 93
        __nesc_atomic_end(__nesc_atomic); 
#line 93
        return __nesc_temp;
      }
    }
#line 95
    __nesc_atomic_end(__nesc_atomic); }
}

# 116 "/home/csilab/tinyos/tos/lib/timer/Alarm.nc"
inline static /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__getAlarm(void ){
#line 116
  unsigned long __nesc_result;
#line 116

#line 116
  __nesc_result = /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__Alarm__getAlarm();
#line 116

#line 116
  return __nesc_result;
#line 116
}
#line 116
# 74 "/home/csilab/tinyos/tos/lib/timer/AlarmToTimerC.nc"
static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired__runTask(void )
{
  if (/*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__m_oneshot == FALSE) {
    /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__start(/*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__getAlarm(), /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__m_dt, FALSE);
    }
#line 78
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__fired();
}

# 67 "/home/csilab/tinyos/tos/interfaces/TaskBasic.nc"
inline static error_t CC2420SpiP__grant__postTask(void ){
#line 67
  unsigned char __nesc_result;
#line 67

#line 67
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(CC2420SpiP__grant);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 184 "/home/csilab/tinyos/tos/chips/cc2420/spi/CC2420SpiP.nc"
static inline void CC2420SpiP__SpiResource__granted(void )
#line 184
{
  CC2420SpiP__grant__postTask();
}

# 180 "/home/csilab/tinyos/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Resource__default__granted(uint8_t id)
#line 180
{
}

# 102 "/home/csilab/tinyos/tos/interfaces/Resource.nc"
inline static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Resource__granted(uint8_t arg_0x7f7fecbe29f0){
#line 102
  switch (arg_0x7f7fecbe29f0) {
#line 102
    case /*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430Spi0C__0__CLIENT_ID:
#line 102
      CC2420SpiP__SpiResource__granted();
#line 102
      break;
#line 102
    default:
#line 102
      /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Resource__default__granted(arg_0x7f7fecbe29f0);
#line 102
      break;
#line 102
    }
#line 102
}
#line 102
# 130 "/home/csilab/tinyos/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__granted(uint8_t id)
#line 130
{
  /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Resource__granted(id);
}

# 202 "/home/csilab/tinyos/tos/system/ArbiterP.nc"
static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__Resource__default__granted(uint8_t id)
#line 202
{
}

# 102 "/home/csilab/tinyos/tos/interfaces/Resource.nc"
inline static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__Resource__granted(uint8_t arg_0x7f7fed25f4b0){
#line 102
  switch (arg_0x7f7fed25f4b0) {
#line 102
    case /*CC2420SpiWireC.HplCC2420SpiC.SpiC.UsartC*/Msp430Usart0C__0__CLIENT_ID:
#line 102
      /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__granted(/*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430Spi0C__0__CLIENT_ID);
#line 102
      break;
#line 102
    default:
#line 102
      /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__Resource__default__granted(arg_0x7f7fed25f4b0);
#line 102
      break;
#line 102
    }
#line 102
}
#line 102
# 190 "/home/csilab/tinyos/tos/system/ArbiterP.nc"
static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__grantedTask__runTask(void )
#line 190
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 191
    {
      /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__resId = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__reqResId;
      /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__state = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__RES_BUSY;
    }
#line 194
    __nesc_atomic_end(__nesc_atomic); }
  /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceConfigure__configure(/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__resId);
  /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__Resource__granted(/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__resId);
}

# 251 "/home/csilab/tinyos/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__SpiPacket__default__sendDone(uint8_t id, uint8_t *tx_buf, uint8_t *rx_buf, uint16_t len, error_t error)
#line 251
{
}

# 82 "/home/csilab/tinyos/tos/interfaces/SpiPacket.nc"
inline static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__SpiPacket__sendDone(uint8_t arg_0x7f7fecbdb020, uint8_t * txBuf, uint8_t * rxBuf, uint16_t len, error_t error){
#line 82
  switch (arg_0x7f7fecbdb020) {
#line 82
    case /*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430Spi0C__0__CLIENT_ID:
#line 82
      CC2420SpiP__SpiPacket__sendDone(txBuf, rxBuf, len, error);
#line 82
      break;
#line 82
    default:
#line 82
      /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__SpiPacket__default__sendDone(arg_0x7f7fecbdb020, txBuf, rxBuf, len, error);
#line 82
      break;
#line 82
    }
#line 82
}
#line 82
# 244 "/home/csilab/tinyos/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__signalDone(void )
#line 244
{
  /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__SpiPacket__sendDone(/*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_client, /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_tx_buf, /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_rx_buf, /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_len, 
  SUCCESS);
}

#line 227
static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__signalDone_task__runTask(void )
#line 227
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 228
    /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__signalDone();
#line 228
    __nesc_atomic_end(__nesc_atomic); }
}

# 486 "/home/csilab/upma/chips/cc2420/CC2420TransmitP.nc"
static inline void CC2420TransmitP__TXFIFO__readDone(uint8_t *tx_buf, uint8_t tx_len, 
error_t error)
#line 487
{
}

# 120 "/home/csilab/tinyos/tos/interfaces/Resource.nc"
inline static error_t CC2420ReceiveP__SpiResource__release(void ){
#line 120
  unsigned char __nesc_result;
#line 120

#line 120
  __nesc_result = CC2420SpiP__Resource__release(/*CC2420ReceiveC.Spi*/CC2420SpiC__4__CLIENT_ID);
#line 120

#line 120
  return __nesc_result;
#line 120
}
#line 120
# 67 "/home/csilab/tinyos/tos/interfaces/TaskBasic.nc"
inline static error_t AsyncReceiveAdapterP__receiveDone_task__postTask(void ){
#line 67
  unsigned char __nesc_result;
#line 67

#line 67
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(AsyncReceiveAdapterP__receiveDone_task);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 42 "/home/csilab/upma/system/AsyncReceiveAdapterP.nc"
static inline void AsyncReceiveAdapterP__AsyncReceive__receive(message_t *msg, void *payload, uint8_t len)
{
  /* atomic removed: atomic calls only */
  {
    AsyncReceiveAdapterP__msg_ = msg;
    AsyncReceiveAdapterP__payload_ = payload;
    AsyncReceiveAdapterP__len_ = len;
  }
  AsyncReceiveAdapterP__receiveDone_task__postTask();
}

# 41 "/home/csilab/upma/interfaces/AsyncReceive.nc"
inline static void PureTDMASchedulerP__Receive__receive(message_t * msg, void * payload, uint8_t len){
#line 41
  AsyncReceiveAdapterP__AsyncReceive__receive(msg, payload, len);
#line 41
}
#line 41
# 61 "/home/csilab/tinyos/tos/system/QueueC.nc"
static inline uint8_t /*PureTDMASchedulerC.LOQueueC*/QueueC__2__Queue__maxSize(void )
#line 61
{
  return 1;
}

#line 97
static inline error_t /*PureTDMASchedulerC.LOQueueC*/QueueC__2__Queue__enqueue(/*PureTDMASchedulerC.LOQueueC*/QueueC__2__queue_t newVal)
#line 97
{
  if (/*PureTDMASchedulerC.LOQueueC*/QueueC__2__Queue__size() < /*PureTDMASchedulerC.LOQueueC*/QueueC__2__Queue__maxSize()) {
      ;
      /*PureTDMASchedulerC.LOQueueC*/QueueC__2__queue[/*PureTDMASchedulerC.LOQueueC*/QueueC__2__tail] = newVal;
      /*PureTDMASchedulerC.LOQueueC*/QueueC__2__tail++;
      if (/*PureTDMASchedulerC.LOQueueC*/QueueC__2__tail == 1) {
#line 102
        /*PureTDMASchedulerC.LOQueueC*/QueueC__2__tail = 0;
        }
#line 103
      /*PureTDMASchedulerC.LOQueueC*/QueueC__2__size++;
      /*PureTDMASchedulerC.LOQueueC*/QueueC__2__printQueue();
      return SUCCESS;
    }
  else {
      return FAIL;
    }
}

# 90 "/home/csilab/tinyos/tos/interfaces/Queue.nc"
inline static error_t PureTDMASchedulerP__LOforwardQ__enqueue(PureTDMASchedulerP__LOforwardQ__t  newVal){
#line 90
  unsigned char __nesc_result;
#line 90

#line 90
  __nesc_result = /*PureTDMASchedulerC.LOQueueC*/QueueC__2__Queue__enqueue(newVal);
#line 90

#line 90
  return __nesc_result;
#line 90
}
#line 90
# 61 "/home/csilab/tinyos/tos/system/QueueC.nc"
static inline uint8_t /*PureTDMASchedulerC.HIQueueC*/QueueC__1__Queue__maxSize(void )
#line 61
{
  return 1;
}

#line 97
static inline error_t /*PureTDMASchedulerC.HIQueueC*/QueueC__1__Queue__enqueue(/*PureTDMASchedulerC.HIQueueC*/QueueC__1__queue_t newVal)
#line 97
{
  if (/*PureTDMASchedulerC.HIQueueC*/QueueC__1__Queue__size() < /*PureTDMASchedulerC.HIQueueC*/QueueC__1__Queue__maxSize()) {
      ;
      /*PureTDMASchedulerC.HIQueueC*/QueueC__1__queue[/*PureTDMASchedulerC.HIQueueC*/QueueC__1__tail] = newVal;
      /*PureTDMASchedulerC.HIQueueC*/QueueC__1__tail++;
      if (/*PureTDMASchedulerC.HIQueueC*/QueueC__1__tail == 1) {
#line 102
        /*PureTDMASchedulerC.HIQueueC*/QueueC__1__tail = 0;
        }
#line 103
      /*PureTDMASchedulerC.HIQueueC*/QueueC__1__size++;
      /*PureTDMASchedulerC.HIQueueC*/QueueC__1__printQueue();
      return SUCCESS;
    }
  else {
      return FAIL;
    }
}

# 90 "/home/csilab/tinyos/tos/interfaces/Queue.nc"
inline static error_t PureTDMASchedulerP__HIforwardQ__enqueue(PureTDMASchedulerP__HIforwardQ__t  newVal){
#line 90
  unsigned char __nesc_result;
#line 90

#line 90
  __nesc_result = /*PureTDMASchedulerC.HIQueueC*/QueueC__1__Queue__enqueue(newVal);
#line 90

#line 90
  return __nesc_result;
#line 90
}
#line 90
# 120 "/home/csilab/upma/system/GenericSlotterP.nc"
static inline uint8_t GenericSlotterP__SlotterControl__getSlot(void )
#line 120
{
  return GenericSlotterP__slot;
}

# 38 "/home/csilab/upma/interfaces/SlotterControl.nc"
inline static uint8_t PureTDMASchedulerP__SlotterControl__getSlot(void ){
#line 38
  unsigned char __nesc_result;
#line 38

#line 38
  __nesc_result = GenericSlotterP__SlotterControl__getSlot();
#line 38

#line 38
  return __nesc_result;
#line 38
}
#line 38
# 77 "/home/csilab/upma/interfaces/AMPacket.nc"
inline static am_addr_t PureTDMASchedulerP__AMPacket__source(message_t * amsg){
#line 77
  unsigned int __nesc_result;
#line 77

#line 77
  __nesc_result = CC2420ActiveMessageP__AMPacket__source(amsg);
#line 77

#line 77
  return __nesc_result;
#line 77
}
#line 77
# 602 "/home/csilab/upma/lib/macs/pure-tdma/PureTDMASchedulerP.nc"
static inline void PureTDMASchedulerP__SubReceive__receive(message_t *msg, void *payload, uint8_t len)
#line 602
{
  am_addr_t src = PureTDMASchedulerP__AMPacket__source(msg);
  uint8_t rcv_flow_id;
  uint16_t rcv_flow_seq;
#line 605
  uint16_t tmp_rcv_slot;
  adaptivehart_msg_t *rcv_AH_payload = (adaptivehart_msg_t *)payload;
  adaptivehart_msg_t *tmp_Q_payload = (adaptivehart_msg_t *)(void *)0;
  bool tmp_is_dest;


  rcv_flow_id = __nesc_ntoh_uint8(rcv_AH_payload->flowid.nxdata);
  rcv_flow_seq = __nesc_ntoh_uint16(rcv_AH_payload->seq.nxdata);

  if (PureTDMASchedulerP__prev_seq[rcv_flow_id] != rcv_flow_seq) {


      PureTDMASchedulerP__prev_seq[rcv_flow_id] = rcv_flow_seq;
      tmp_is_dest = PureTDMASchedulerP__is_dest[rcv_flow_id];

      src = (am_addr_t )PureTDMASchedulerP__AMPacket__source(msg);
      tmp_rcv_slot = PureTDMASchedulerP__SlotterControl__getSlot();



      if (PureTDMASchedulerP__rx_flow_seq[rcv_flow_id] <= PureTDMASchedulerP__FLOW_SEQ) {
          PureTDMASchedulerP__rcv_slots[rcv_flow_id][PureTDMASchedulerP__rx_flow_seq[rcv_flow_id]] = tmp_rcv_slot;
          PureTDMASchedulerP__rcv_srcs[rcv_flow_id][PureTDMASchedulerP__rx_flow_seq[rcv_flow_id]] = src;
          PureTDMASchedulerP__rx_flow_seq[rcv_flow_id] += 1;
        }


      if (tmp_is_dest == FALSE) {

          if (rcv_flow_id == 1) {
              PureTDMASchedulerP__HIqueueSize = PureTDMASchedulerP__HIforwardQ__size();
              tmp_Q_payload = (adaptivehart_msg_t *)PureTDMASchedulerP__SubSend__getPayload(&PureTDMASchedulerP__HIforwardPktBuffer[PureTDMASchedulerP__HIqueueSize], sizeof(adaptivehart_msg_t ));
              PureTDMASchedulerP__set_payload(tmp_Q_payload, rcv_flow_id, __nesc_ntoh_uint8(rcv_AH_payload->priority.nxdata), __nesc_ntoh_uint8(rcv_AH_payload->maxtx.nxdata), __nesc_ntoh_uint16(rcv_AH_payload->deadline.nxdata), rcv_flow_seq);
              /* atomic removed: atomic calls only */
#line 638
              PureTDMASchedulerP__tx_opper[rcv_flow_id] = __nesc_ntoh_uint8(rcv_AH_payload->maxtx.nxdata);
              /* atomic removed: atomic calls only */
#line 639
              PureTDMASchedulerP__rcv_slot[rcv_flow_id] = tmp_rcv_slot;
              PureTDMASchedulerP__HIforwardQ__enqueue(tmp_Q_payload);
            }
          else {
#line 641
            if (rcv_flow_id == 2) {
                PureTDMASchedulerP__LOqueueSize = PureTDMASchedulerP__LOforwardQ__size();
                tmp_Q_payload = (adaptivehart_msg_t *)PureTDMASchedulerP__SubSend__getPayload(&PureTDMASchedulerP__LOforwardPktBuffer[PureTDMASchedulerP__LOqueueSize], sizeof(adaptivehart_msg_t ));
                PureTDMASchedulerP__set_payload(tmp_Q_payload, rcv_flow_id, __nesc_ntoh_uint8(rcv_AH_payload->priority.nxdata), __nesc_ntoh_uint8(rcv_AH_payload->maxtx.nxdata), __nesc_ntoh_uint16(rcv_AH_payload->deadline.nxdata), rcv_flow_seq);
                /* atomic removed: atomic calls only */
#line 645
                PureTDMASchedulerP__tx_opper[rcv_flow_id] = __nesc_ntoh_uint8(rcv_AH_payload->maxtx.nxdata);
                /* atomic removed: atomic calls only */
#line 646
                PureTDMASchedulerP__rcv_slot[rcv_flow_id] = tmp_rcv_slot;
                PureTDMASchedulerP__LOforwardQ__enqueue(tmp_Q_payload);
              }
            }
        }
    }

  PureTDMASchedulerP__Receive__receive(msg, payload, len);
}

# 41 "/home/csilab/upma/interfaces/AsyncReceive.nc"
inline static void BeaconSlotP__Receive__receive(message_t * msg, void * payload, uint8_t len){
#line 41
  PureTDMASchedulerP__SubReceive__receive(msg, payload, len);
#line 41
}
#line 41
# 64 "/home/csilab/tinyos/tos/lib/timer/Counter.nc"
inline static /*PureTDMASchedulerC.Alarm32khz32C.Transform*/TransformAlarmC__3__Counter__size_type /*PureTDMASchedulerC.Alarm32khz32C.Transform*/TransformAlarmC__3__Counter__get(void ){
#line 64
  unsigned long __nesc_result;
#line 64

#line 64
  __nesc_result = /*Counter32khz32C.Transform*/TransformCounterC__0__Counter__get();
#line 64

#line 64
  return __nesc_result;
#line 64
}
#line 64
# 86 "/home/csilab/tinyos/tos/lib/timer/TransformAlarmC.nc"
static inline /*PureTDMASchedulerC.Alarm32khz32C.Transform*/TransformAlarmC__3__to_size_type /*PureTDMASchedulerC.Alarm32khz32C.Transform*/TransformAlarmC__3__Alarm__getNow(void )
{
  return /*PureTDMASchedulerC.Alarm32khz32C.Transform*/TransformAlarmC__3__Counter__get();
}

#line 147
static inline void /*PureTDMASchedulerC.Alarm32khz32C.Transform*/TransformAlarmC__3__Alarm__startAt(/*PureTDMASchedulerC.Alarm32khz32C.Transform*/TransformAlarmC__3__to_size_type t0, /*PureTDMASchedulerC.Alarm32khz32C.Transform*/TransformAlarmC__3__to_size_type dt)
{
  /* atomic removed: atomic calls only */
  {
    /*PureTDMASchedulerC.Alarm32khz32C.Transform*/TransformAlarmC__3__m_t0 = t0;
    /*PureTDMASchedulerC.Alarm32khz32C.Transform*/TransformAlarmC__3__m_dt = dt;
    /*PureTDMASchedulerC.Alarm32khz32C.Transform*/TransformAlarmC__3__set_alarm();
  }
}

static inline void /*PureTDMASchedulerC.Alarm32khz32C.Transform*/TransformAlarmC__3__Alarm__start(/*PureTDMASchedulerC.Alarm32khz32C.Transform*/TransformAlarmC__3__to_size_type dt)
{
  /*PureTDMASchedulerC.Alarm32khz32C.Transform*/TransformAlarmC__3__Alarm__startAt(/*PureTDMASchedulerC.Alarm32khz32C.Transform*/TransformAlarmC__3__Alarm__getNow(), dt);
}

# 66 "/home/csilab/tinyos/tos/lib/timer/Alarm.nc"
inline static void BeaconSlotP__SyncAlarm__start(BeaconSlotP__SyncAlarm__size_type dt){
#line 66
  /*PureTDMASchedulerC.Alarm32khz32C.Transform*/TransformAlarmC__3__Alarm__start(dt);
#line 66
}
#line 66
# 63 "/home/csilab/upma/system/BeaconSlotP.nc"
static inline uint32_t BeaconSlotP__read_timestamp(uint8_t *payload)
#line 63
{
  nx_uint32_t *ptr = (nx_uint32_t *)payload;

  return __nesc_ntoh_uint32((*ptr).nxdata);
}

# 65 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*GenericSlotterC.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__3__Alarm__stop(void )
{
  /*GenericSlotterC.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__3__Msp430TimerControl__disableEvents();
}

# 73 "/home/csilab/tinyos/tos/lib/timer/Alarm.nc"
inline static void /*GenericSlotterC.Alarm32khz32C.Transform*/TransformAlarmC__2__AlarmFrom__stop(void ){
#line 73
  /*GenericSlotterC.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__3__Alarm__stop();
#line 73
}
#line 73
# 102 "/home/csilab/tinyos/tos/lib/timer/TransformAlarmC.nc"
static inline void /*GenericSlotterC.Alarm32khz32C.Transform*/TransformAlarmC__2__Alarm__stop(void )
{
  /*GenericSlotterC.Alarm32khz32C.Transform*/TransformAlarmC__2__AlarmFrom__stop();
}

# 73 "/home/csilab/tinyos/tos/lib/timer/Alarm.nc"
inline static void GenericSlotterP__Alarm__stop(void ){
#line 73
  /*GenericSlotterC.Alarm32khz32C.Transform*/TransformAlarmC__2__Alarm__stop();
#line 73
}
#line 73
# 107 "/home/csilab/upma/system/GenericSlotterP.nc"
static inline void GenericSlotterP__SlotterControl__stop(void )
#line 107
{
  GenericSlotterP__Alarm__stop();
}

# 37 "/home/csilab/upma/interfaces/SlotterControl.nc"
inline static void BeaconSlotP__SlotterControl__stop(void ){
#line 37
  GenericSlotterP__SlotterControl__stop();
#line 37
}
#line 37
# 136 "/home/csilab/upma/interfaces/AMPacket.nc"
inline static am_id_t BeaconSlotP__AMPacket__type(message_t * amsg){
#line 136
  unsigned char __nesc_result;
#line 136

#line 136
  __nesc_result = CC2420ActiveMessageP__AMPacket__type(amsg);
#line 136

#line 136
  return __nesc_result;
#line 136
}
#line 136
# 113 "/home/csilab/upma/system/BeaconSlotP.nc"
static inline void BeaconSlotP__SubReceive__receive(message_t *msg, void *payload, uint8_t len)
#line 113
{
  uint32_t alarmTime;

  if (BeaconSlotP__AMPacket__type(msg) == SIMPLE_TDMA_SYNC) {
      BeaconSlotP__SlotterControl__stop();

      alarmTime = BeaconSlotP__read_timestamp(payload);
      if (alarmTime > 37) {
          alarmTime = alarmTime - 37;
          BeaconSlotP__SyncAlarm__start(alarmTime);
        }
      else {


          BeaconSlotP__SlotterControl__synchronize(1);
        }

      BeaconSlotP__SubReceive__updateBuffer(msg);
    }
  else 
#line 131
    {
      BeaconSlotP__Receive__receive(msg, payload, len);
    }
}

# 41 "/home/csilab/upma/interfaces/AsyncReceive.nc"
inline static void CC2420ReceiveP__Receive__receive(message_t * msg, void * payload, uint8_t len){
#line 41
  BeaconSlotP__SubReceive__receive(msg, payload, len);
#line 41
}
#line 41
# 302 "/home/csilab/tinyos/tos/chips/cc2420/control/CC2420ControlP.nc"
static inline uint16_t CC2420ControlP__CC2420Config__getShortAddr(void )
#line 302
{
  /* atomic removed: atomic calls only */
#line 303
  {
    unsigned int __nesc_temp = 
#line 303
    CC2420ControlP__m_short_addr;

#line 303
    return __nesc_temp;
  }
}

# 71 "/home/csilab/tinyos/tos/chips/cc2420/interfaces/CC2420Config.nc"
inline static uint16_t CC2420ReceiveP__CC2420Config__getShortAddr(void ){
#line 71
  unsigned int __nesc_result;
#line 71

#line 71
  __nesc_result = CC2420ControlP__CC2420Config__getShortAddr();
#line 71

#line 71
  return __nesc_result;
#line 71
}
#line 71
# 355 "/home/csilab/tinyos/tos/chips/cc2420/control/CC2420ControlP.nc"
static inline bool CC2420ControlP__CC2420Config__isAddressRecognitionEnabled(void )
#line 355
{
  /* atomic removed: atomic calls only */
#line 356
  {
    unsigned char __nesc_temp = 
#line 356
    CC2420ControlP__addressRecognition;

#line 356
    return __nesc_temp;
  }
}

# 93 "/home/csilab/tinyos/tos/chips/cc2420/interfaces/CC2420Config.nc"
inline static bool CC2420ReceiveP__CC2420Config__isAddressRecognitionEnabled(void ){
#line 93
  unsigned char __nesc_result;
#line 93

#line 93
  __nesc_result = CC2420ControlP__CC2420Config__isAddressRecognitionEnabled();
#line 93

#line 93
  return __nesc_result;
#line 93
}
#line 93
# 42 "/home/csilab/tinyos/tos/chips/cc2420/interfaces/CC2420PacketBody.nc"
inline static cc2420_header_t * CC2420ReceiveP__CC2420PacketBody__getHeader(message_t * msg){
#line 42
  nx_struct cc2420_header_t *__nesc_result;
#line 42

#line 42
  __nesc_result = CC2420PacketP__CC2420PacketBody__getHeader(msg);
#line 42

#line 42
  return __nesc_result;
#line 42
}
#line 42
# 476 "/home/csilab/upma/chips/cc2420/CC2420ReceiveP.nc"
static inline bool CC2420ReceiveP__passesAddressCheck(message_t *msg)
#line 476
{
  cc2420_header_t *header = CC2420ReceiveP__CC2420PacketBody__getHeader(msg);

  if (!CC2420ReceiveP__CC2420Config__isAddressRecognitionEnabled()) {
      return TRUE;
    }

  return __nesc_ntoh_leuint16(header->dest.nxdata) == CC2420ReceiveP__CC2420Config__getShortAddr()
   || __nesc_ntoh_leuint16(header->dest.nxdata) == AM_BROADCAST_ADDR;
}

# 53 "/home/csilab/tinyos/tos/chips/cc2420/interfaces/CC2420PacketBody.nc"
inline static cc2420_metadata_t * CC2420ReceiveP__CC2420PacketBody__getMetadata(message_t * msg){
#line 53
  nx_struct cc2420_metadata_t *__nesc_result;
#line 53

#line 53
  __nesc_result = CC2420PacketP__CC2420PacketBody__getMetadata(msg);
#line 53

#line 53
  return __nesc_result;
#line 53
}
#line 53
# 346 "/home/csilab/upma/chips/cc2420/CC2420ReceiveP.nc"
static inline void CC2420ReceiveP__receiveDone(void )
#line 346
{
  cc2420_metadata_t *metadata = CC2420ReceiveP__CC2420PacketBody__getMetadata(CC2420ReceiveP__m_p_rx_buf);
  cc2420_header_t *header = CC2420ReceiveP__CC2420PacketBody__getHeader(CC2420ReceiveP__m_p_rx_buf);
  uint8_t length = __nesc_ntoh_leuint8(header->length.nxdata);
  uint8_t tmpLen __attribute((unused))  = sizeof(message_t ) - ((unsigned short )& ((message_t *)0)->data - sizeof(cc2420_header_t ));
  uint8_t * buf = (uint8_t * )header;

  __nesc_hton_int8(metadata->crc.nxdata, buf[length] >> 7);
  __nesc_hton_uint8(metadata->lqi.nxdata, buf[length] & 0x7f);
  __nesc_hton_uint8(metadata->rssi.nxdata, buf[length - 1]);

  if (CC2420ReceiveP__passesAddressCheck(CC2420ReceiveP__m_p_rx_buf) && length >= CC2420_SIZE) {
      CC2420ReceiveP__Receive__receive(CC2420ReceiveP__m_p_rx_buf, CC2420ReceiveP__m_p_rx_buf->data, 
      length - CC2420_SIZE);
    }
  else 
#line 360
    {
      /* atomic removed: atomic calls only */
#line 361
      CC2420ReceiveP__receivingPacket = FALSE;
      CC2420ReceiveP__waitForNextPacket();
    }
}

# 53 "/home/csilab/tinyos/tos/chips/cc2420/interfaces/CC2420PacketBody.nc"
inline static cc2420_metadata_t * CC2420TransmitP__CC2420PacketBody__getMetadata(message_t * msg){
#line 53
  nx_struct cc2420_metadata_t *__nesc_result;
#line 53

#line 53
  __nesc_result = CC2420PacketP__CC2420PacketBody__getMetadata(msg);
#line 53

#line 53
  return __nesc_result;
#line 53
}
#line 53
# 389 "/home/csilab/upma/chips/cc2420/CC2420TransmitP.nc"
static inline void CC2420TransmitP__CC2420Receive__receive(uint8_t type, message_t *ack_msg)
#line 389
{
  cc2420_header_t *ack_header;
  cc2420_header_t *msg_header;
  cc2420_metadata_t *msg_metadata;
  uint8_t *ack_buf;
  uint8_t length;

  if (type == IEEE154_TYPE_ACK && CC2420TransmitP__m_msg) {
      ack_header = CC2420TransmitP__CC2420PacketBody__getHeader(ack_msg);
      msg_header = CC2420TransmitP__CC2420PacketBody__getHeader(CC2420TransmitP__m_msg);

      if (CC2420TransmitP__m_state == CC2420TransmitP__S_ACK_WAIT && __nesc_ntoh_leuint8(msg_header->dsn.nxdata) == __nesc_ntoh_leuint8(ack_header->dsn.nxdata)) {
          CC2420TransmitP__BackoffTimer__stop();

          msg_metadata = CC2420TransmitP__CC2420PacketBody__getMetadata(CC2420TransmitP__m_msg);
          ack_buf = (uint8_t *)ack_header;
          length = __nesc_ntoh_leuint8(ack_header->length.nxdata);

          __nesc_hton_int8(msg_metadata->ack.nxdata, TRUE);
          __nesc_hton_uint8(msg_metadata->rssi.nxdata, ack_buf[length - 1]);
          __nesc_hton_uint8(msg_metadata->lqi.nxdata, ack_buf[length] & 0x7f);
          CC2420TransmitP__signalDone(SUCCESS);
        }
    }
}

# 63 "/home/csilab/tinyos/tos/chips/cc2420/interfaces/CC2420Receive.nc"
inline static void CC2420ReceiveP__CC2420Receive__receive(uint8_t type, message_t * message){
#line 63
  CC2420TransmitP__CC2420Receive__receive(type, message);
#line 63
}
#line 63
# 70 "/home/csilab/tinyos/tos/interfaces/PacketTimeStamp.nc"
inline static void CC2420ReceiveP__PacketTimeStamp__clear(message_t * msg){
#line 70
  CC2420PacketP__PacketTimeStamp32khz__clear(msg);
#line 70
}
#line 70








inline static void CC2420ReceiveP__PacketTimeStamp__set(message_t * msg, CC2420ReceiveP__PacketTimeStamp__size_type value){
#line 78
  CC2420PacketP__PacketTimeStamp32khz__set(msg, value);
#line 78
}
#line 78
# 59 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline uint8_t /*HplMsp430GeneralIOC.P10*/HplMsp430GeneralIOP__0__IO__getRaw(void )
#line 59
{
#line 59
  return * (volatile uint8_t * )32U & (0x01 << 0);
}

#line 60
static inline bool /*HplMsp430GeneralIOC.P10*/HplMsp430GeneralIOP__0__IO__get(void )
#line 60
{
#line 60
  return /*HplMsp430GeneralIOC.P10*/HplMsp430GeneralIOP__0__IO__getRaw() != 0;
}

# 73 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static bool /*HplCC2420PinsC.FIFOPM*/Msp430GpioC__6__HplGeneralIO__get(void ){
#line 73
  unsigned char __nesc_result;
#line 73

#line 73
  __nesc_result = /*HplMsp430GeneralIOC.P10*/HplMsp430GeneralIOP__0__IO__get();
#line 73

#line 73
  return __nesc_result;
#line 73
}
#line 73
# 51 "/home/csilab/tinyos/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline bool /*HplCC2420PinsC.FIFOPM*/Msp430GpioC__6__GeneralIO__get(void )
#line 51
{
#line 51
  return /*HplCC2420PinsC.FIFOPM*/Msp430GpioC__6__HplGeneralIO__get();
}

# 43 "/home/csilab/tinyos/tos/interfaces/GeneralIO.nc"
inline static bool CC2420ReceiveP__FIFOP__get(void ){
#line 43
  unsigned char __nesc_result;
#line 43

#line 43
  __nesc_result = /*HplCC2420PinsC.FIFOPM*/Msp430GpioC__6__GeneralIO__get();
#line 43

#line 43
  return __nesc_result;
#line 43
}
#line 43
# 59 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline uint8_t /*HplMsp430GeneralIOC.P13*/HplMsp430GeneralIOP__3__IO__getRaw(void )
#line 59
{
#line 59
  return * (volatile uint8_t * )32U & (0x01 << 3);
}

#line 60
static inline bool /*HplMsp430GeneralIOC.P13*/HplMsp430GeneralIOP__3__IO__get(void )
#line 60
{
#line 60
  return /*HplMsp430GeneralIOC.P13*/HplMsp430GeneralIOP__3__IO__getRaw() != 0;
}

# 73 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static bool /*HplCC2420PinsC.FIFOM*/Msp430GpioC__5__HplGeneralIO__get(void ){
#line 73
  unsigned char __nesc_result;
#line 73

#line 73
  __nesc_result = /*HplMsp430GeneralIOC.P13*/HplMsp430GeneralIOP__3__IO__get();
#line 73

#line 73
  return __nesc_result;
#line 73
}
#line 73
# 51 "/home/csilab/tinyos/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline bool /*HplCC2420PinsC.FIFOM*/Msp430GpioC__5__GeneralIO__get(void )
#line 51
{
#line 51
  return /*HplCC2420PinsC.FIFOM*/Msp430GpioC__5__HplGeneralIO__get();
}

# 43 "/home/csilab/tinyos/tos/interfaces/GeneralIO.nc"
inline static bool CC2420ReceiveP__FIFO__get(void ){
#line 43
  unsigned char __nesc_result;
#line 43

#line 43
  __nesc_result = /*HplCC2420PinsC.FIFOM*/Msp430GpioC__5__GeneralIO__get();
#line 43

#line 43
  return __nesc_result;
#line 43
}
#line 43
# 209 "/home/csilab/tinyos/tos/chips/cc2420/spi/CC2420SpiP.nc"
static inline error_t CC2420SpiP__Fifo__continueRead(uint8_t addr, uint8_t *data, 
uint8_t len)
#line 210
{
  return CC2420SpiP__SpiPacket__send((void *)0, data, len);
}

# 62 "/home/csilab/tinyos/tos/chips/cc2420/interfaces/CC2420Fifo.nc"
inline static error_t CC2420ReceiveP__RXFIFO__continueRead(uint8_t * data, uint8_t length){
#line 62
  unsigned char __nesc_result;
#line 62

#line 62
  __nesc_result = CC2420SpiP__Fifo__continueRead(CC2420_RXFIFO, data, length);
#line 62

#line 62
  return __nesc_result;
#line 62
}
#line 62
#line 51
inline static cc2420_status_t CC2420ReceiveP__RXFIFO__beginRead(uint8_t * data, uint8_t length){
#line 51
  unsigned char __nesc_result;
#line 51

#line 51
  __nesc_result = CC2420SpiP__Fifo__beginRead(CC2420_RXFIFO, data, length);
#line 51

#line 51
  return __nesc_result;
#line 51
}
#line 51
# 41 "/home/csilab/tinyos/tos/interfaces/GeneralIO.nc"
inline static void CC2420ReceiveP__CSN__clr(void ){
#line 41
  /*HplCC2420PinsC.CSNM*/Msp430GpioC__4__GeneralIO__clr();
#line 41
}
#line 41
# 53 "/home/csilab/tinyos/tos/chips/cc2420/interfaces/CC2420Strobe.nc"
inline static cc2420_status_t CC2420ReceiveP__SACK__strobe(void ){
#line 53
  unsigned char __nesc_result;
#line 53

#line 53
  __nesc_result = CC2420SpiP__Strobe__strobe(CC2420_SACK);
#line 53

#line 53
  return __nesc_result;
#line 53
}
#line 53
# 382 "/home/csilab/tinyos/tos/chips/cc2420/control/CC2420ControlP.nc"
static inline bool CC2420ControlP__CC2420Config__isHwAutoAckDefault(void )
#line 382
{
  /* atomic removed: atomic calls only */
#line 383
  {
    unsigned char __nesc_temp = 
#line 383
    CC2420ControlP__hwAutoAckDefault;

#line 383
    return __nesc_temp;
  }
}

# 112 "/home/csilab/tinyos/tos/chips/cc2420/interfaces/CC2420Config.nc"
inline static bool CC2420ReceiveP__CC2420Config__isHwAutoAckDefault(void ){
#line 112
  unsigned char __nesc_result;
#line 112

#line 112
  __nesc_result = CC2420ControlP__CC2420Config__isHwAutoAckDefault();
#line 112

#line 112
  return __nesc_result;
#line 112
}
#line 112
# 389 "/home/csilab/tinyos/tos/chips/cc2420/control/CC2420ControlP.nc"
static inline bool CC2420ControlP__CC2420Config__isAutoAckEnabled(void )
#line 389
{
  /* atomic removed: atomic calls only */
#line 390
  {
    unsigned char __nesc_temp = 
#line 390
    CC2420ControlP__autoAckEnabled;

#line 390
    return __nesc_temp;
  }
}

# 117 "/home/csilab/tinyos/tos/chips/cc2420/interfaces/CC2420Config.nc"
inline static bool CC2420ReceiveP__CC2420Config__isAutoAckEnabled(void ){
#line 117
  unsigned char __nesc_result;
#line 117

#line 117
  __nesc_result = CC2420ControlP__CC2420Config__isAutoAckEnabled();
#line 117

#line 117
  return __nesc_result;
#line 117
}
#line 117
# 208 "/home/csilab/upma/chips/cc2420/CC2420ReceiveP.nc"
static inline void CC2420ReceiveP__RXFIFO__readDone(uint8_t *rx_buf, uint8_t rx_len, 
error_t error)
#line 209
{
  cc2420_header_t *header = CC2420ReceiveP__CC2420PacketBody__getHeader(CC2420ReceiveP__m_p_rx_buf);
  uint8_t tmpLen __attribute((unused))  = sizeof(message_t ) - ((unsigned short )& ((message_t *)0)->data - sizeof(cc2420_header_t ));
  uint8_t * buf = (uint8_t * )header;

#line 213
  CC2420ReceiveP__rxFrameLength = buf[0];

  switch (CC2420ReceiveP__m_state) {

      case CC2420ReceiveP__S_RX_LENGTH: 
        CC2420ReceiveP__m_state = CC2420ReceiveP__S_RX_FCF;
      if (CC2420ReceiveP__rxFrameLength + 1 > CC2420ReceiveP__m_bytes_left) {

          CC2420ReceiveP__flush();
        }
      else {
          if (!CC2420ReceiveP__FIFO__get() && !CC2420ReceiveP__FIFOP__get()) {
              CC2420ReceiveP__m_bytes_left -= CC2420ReceiveP__rxFrameLength + 1;
            }

          if (CC2420ReceiveP__rxFrameLength <= MAC_PACKET_SIZE) {
              if (CC2420ReceiveP__rxFrameLength > 0) {
                  if (CC2420ReceiveP__rxFrameLength > CC2420ReceiveP__SACK_HEADER_LENGTH) {

                      CC2420ReceiveP__RXFIFO__continueRead(buf + 1, CC2420ReceiveP__SACK_HEADER_LENGTH);
                    }
                  else {

                      CC2420ReceiveP__m_state = CC2420ReceiveP__S_RX_PAYLOAD;
                      CC2420ReceiveP__RXFIFO__continueRead(buf + 1, CC2420ReceiveP__rxFrameLength);
                    }
                }
              else {
                  /* atomic removed: atomic calls only */
                  CC2420ReceiveP__receivingPacket = FALSE;
                  CC2420ReceiveP__CSN__set();
                  CC2420ReceiveP__SpiResource__release();
                  CC2420ReceiveP__waitForNextPacket();
                }
            }
          else {

              CC2420ReceiveP__flush();
            }
        }
      break;

      case CC2420ReceiveP__S_RX_FCF: 
        CC2420ReceiveP__m_state = CC2420ReceiveP__S_RX_PAYLOAD;










      if (CC2420ReceiveP__CC2420Config__isAutoAckEnabled() && !CC2420ReceiveP__CC2420Config__isHwAutoAckDefault()) {



          if (((__nesc_ntoh_leuint16(
#line 268
          header->fcf.nxdata) >> IEEE154_FCF_ACK_REQ) & 0x01) == 1
           && (__nesc_ntoh_leuint16(header->dest.nxdata) == CC2420ReceiveP__CC2420Config__getShortAddr()
           || __nesc_ntoh_leuint16(header->dest.nxdata) == AM_BROADCAST_ADDR)
           && ((__nesc_ntoh_leuint16(header->fcf.nxdata) >> IEEE154_FCF_FRAME_TYPE) & 7) == IEEE154_TYPE_DATA) {

              CC2420ReceiveP__CSN__set();
              CC2420ReceiveP__CSN__clr();
              CC2420ReceiveP__SACK__strobe();
              CC2420ReceiveP__CSN__set();
              CC2420ReceiveP__CSN__clr();
              CC2420ReceiveP__RXFIFO__beginRead(buf + 1 + CC2420ReceiveP__SACK_HEADER_LENGTH, 
              CC2420ReceiveP__rxFrameLength - CC2420ReceiveP__SACK_HEADER_LENGTH);
              return;
            }
        }

      CC2420ReceiveP__RXFIFO__continueRead(buf + 1 + CC2420ReceiveP__SACK_HEADER_LENGTH, 
      CC2420ReceiveP__rxFrameLength - CC2420ReceiveP__SACK_HEADER_LENGTH);
      break;

      case CC2420ReceiveP__S_RX_PAYLOAD: 

        CC2420ReceiveP__CSN__set();
      if (!CC2420ReceiveP__m_missed_packets) {

          CC2420ReceiveP__SpiResource__release();
        }




      if ((((
#line 297
      CC2420ReceiveP__m_missed_packets && CC2420ReceiveP__FIFO__get()) || !CC2420ReceiveP__FIFOP__get())
       || !CC2420ReceiveP__m_timestamp_size)
       || CC2420ReceiveP__rxFrameLength <= 10) {
          CC2420ReceiveP__PacketTimeStamp__clear(CC2420ReceiveP__m_p_rx_buf);
        }
      else {
          if (CC2420ReceiveP__m_timestamp_size == 1) {
            CC2420ReceiveP__PacketTimeStamp__set(CC2420ReceiveP__m_p_rx_buf, CC2420ReceiveP__m_timestamp_queue[CC2420ReceiveP__m_timestamp_head]);
            }
#line 305
          CC2420ReceiveP__m_timestamp_head = (CC2420ReceiveP__m_timestamp_head + 1) % CC2420ReceiveP__TIMESTAMP_QUEUE_SIZE;
          CC2420ReceiveP__m_timestamp_size--;

          if (CC2420ReceiveP__m_timestamp_size > 0) {
              CC2420ReceiveP__PacketTimeStamp__clear(CC2420ReceiveP__m_p_rx_buf);
              CC2420ReceiveP__m_timestamp_head = 0;
              CC2420ReceiveP__m_timestamp_size = 0;
            }
        }



      if (buf[CC2420ReceiveP__rxFrameLength] >> 7 && rx_buf) {
          uint8_t type = (__nesc_ntoh_leuint16(header->fcf.nxdata) >> IEEE154_FCF_FRAME_TYPE) & 7;

#line 319
          CC2420ReceiveP__CC2420Receive__receive(type, CC2420ReceiveP__m_p_rx_buf);
          if (type == IEEE154_TYPE_DATA) {
              CC2420ReceiveP__receiveDone();
              return;
            }
        }

      CC2420ReceiveP__waitForNextPacket();
      break;

      default: /* atomic removed: atomic calls only */
        CC2420ReceiveP__receivingPacket = FALSE;
      CC2420ReceiveP__CSN__set();
      CC2420ReceiveP__SpiResource__release();
      break;
    }
}

# 370 "/home/csilab/tinyos/tos/chips/cc2420/spi/CC2420SpiP.nc"
static inline void CC2420SpiP__Fifo__default__readDone(uint8_t addr, uint8_t *rx_buf, uint8_t rx_len, error_t error)
#line 370
{
}

# 71 "/home/csilab/tinyos/tos/chips/cc2420/interfaces/CC2420Fifo.nc"
inline static void CC2420SpiP__Fifo__readDone(uint8_t arg_0x7f7fecc8f6e0, uint8_t * data, uint8_t length, error_t error){
#line 71
  switch (arg_0x7f7fecc8f6e0) {
#line 71
    case CC2420_TXFIFO:
#line 71
      CC2420TransmitP__TXFIFO__readDone(data, length, error);
#line 71
      break;
#line 71
    case CC2420_RXFIFO:
#line 71
      CC2420ReceiveP__RXFIFO__readDone(data, length, error);
#line 71
      break;
#line 71
    default:
#line 71
      CC2420SpiP__Fifo__default__readDone(arg_0x7f7fecc8f6e0, data, length, error);
#line 71
      break;
#line 71
    }
#line 71
}
#line 71
# 53 "/home/csilab/tinyos/tos/chips/cc2420/interfaces/CC2420Strobe.nc"
inline static cc2420_status_t CC2420ReceiveP__SFLUSHRX__strobe(void ){
#line 53
  unsigned char __nesc_result;
#line 53

#line 53
  __nesc_result = CC2420SpiP__Strobe__strobe(CC2420_SFLUSHRX);
#line 53

#line 53
  return __nesc_result;
#line 53
}
#line 53
# 759 "/home/csilab/upma/chips/cc2420/CC2420TransmitP.nc"
static inline uint16_t CC2420TransmitP__defaultInitialBackoff(void )
#line 759
{
  return CC2420TransmitP__Random__rand16()
   % (0x1F * CC2420_BACKOFF_PERIOD) + CC2420_MIN_BACKOFF;
}

# 283 "/home/csilab/upma/chips/cc2420/CC2420ActiveMessageP.nc"
static inline uint16_t CC2420ActiveMessageP__CcaControl__default__getInitialBackoff(am_id_t amId, message_t *msg, uint16_t defaultBackoff)
#line 283
{
  return defaultBackoff;
}

# 39 "/home/csilab/upma/interfaces/CcaControl.nc"
inline static uint16_t CC2420ActiveMessageP__CcaControl__getInitialBackoff(am_id_t arg_0x7f7fec07e320, message_t *msg, uint16_t defaultbackoff){
#line 39
  unsigned int __nesc_result;
#line 39

#line 39
    __nesc_result = CC2420ActiveMessageP__CcaControl__default__getInitialBackoff(arg_0x7f7fec07e320, msg, defaultbackoff);
#line 39

#line 39
  return __nesc_result;
#line 39
}
#line 39
# 253 "/home/csilab/upma/chips/cc2420/CC2420ActiveMessageP.nc"
static inline uint16_t CC2420ActiveMessageP__SubCcaControl__getInitialBackoff(am_id_t amId, message_t *msg, uint16_t defaultBackoff)
#line 253
{
  return CC2420ActiveMessageP__CcaControl__getInitialBackoff(__nesc_ntoh_leuint8(((cc2420_header_t * )((uint8_t *)msg + (unsigned short )& ((message_t *)0)->data - sizeof(cc2420_header_t )))->type.nxdata), msg, defaultBackoff);
}

# 143 "/home/csilab/upma/system/BeaconSlotP.nc"
static inline uint16_t BeaconSlotP__SubCcaControl__getInitialBackoff(message_t *msg, uint16_t defaultbackoff)
#line 143
{
  return 0;
}

# 154 "/home/csilab/upma/system/GenericSlotSenderP.nc"
static inline uint16_t /*TDMASlotSenderC.TDMASlotSender.GenericSlotSenderP*/GenericSlotSenderP__0__SubCcaControl__getInitialBackoff(message_t *msg, uint16_t defaultbackoff)
#line 154
{
  return 0;
}

# 124 "/home/csilab/upma/system/SenderDispatcherP.nc"
static inline uint16_t SenderDispatcherP__SlotsCcaControl__default__getInitialBackoff(am_id_t id, message_t *msg, uint16_t defaultbackoff)
#line 124
{
  return defaultbackoff;
}

# 39 "/home/csilab/upma/interfaces/CcaControl.nc"
inline static uint16_t SenderDispatcherP__SlotsCcaControl__getInitialBackoff(uint8_t arg_0x7f7fec1925b0, message_t *msg, uint16_t defaultbackoff){
#line 39
  unsigned int __nesc_result;
#line 39

#line 39
  switch (arg_0x7f7fec1925b0) {
#line 39
    case TDMA_SLOT:
#line 39
      __nesc_result = /*TDMASlotSenderC.TDMASlotSender.GenericSlotSenderP*/GenericSlotSenderP__0__SubCcaControl__getInitialBackoff(msg, defaultbackoff);
#line 39
      break;
#line 39
    case BEACON_SLOT:
#line 39
      __nesc_result = BeaconSlotP__SubCcaControl__getInitialBackoff(msg, defaultbackoff);
#line 39
      break;
#line 39
    default:
#line 39
      __nesc_result = SenderDispatcherP__SlotsCcaControl__default__getInitialBackoff(arg_0x7f7fec1925b0, msg, defaultbackoff);
#line 39
      break;
#line 39
    }
#line 39

#line 39
  return __nesc_result;
#line 39
}
#line 39
# 136 "/home/csilab/upma/system/SenderDispatcherP.nc"
static inline uint16_t SenderDispatcherP__SubCcaControl__getInitialBackoff(am_id_t id, message_t *msg, uint16_t defaultbackoff)
#line 136
{
  uint16_t backoff;

#line 138
  backoff = SenderDispatcherP__SlotsCcaControl__getInitialBackoff(SenderDispatcherP__last_id, msg, defaultbackoff);
  return backoff;
}

# 39 "/home/csilab/upma/interfaces/CcaControl.nc"
inline static uint16_t CC2420TransmitP__CcaControl__getInitialBackoff(am_id_t arg_0x7f7fec8771d0, message_t *msg, uint16_t defaultbackoff){
#line 39
  unsigned int __nesc_result;
#line 39

#line 39
  __nesc_result = SenderDispatcherP__SubCcaControl__getInitialBackoff(arg_0x7f7fec8771d0, msg, defaultbackoff);
#line 39
  __nesc_result = CC2420ActiveMessageP__SubCcaControl__getInitialBackoff(arg_0x7f7fec8771d0, msg, defaultbackoff);
#line 39

#line 39
  return __nesc_result;
#line 39
}
#line 39
# 653 "/home/csilab/upma/chips/cc2420/CC2420TransmitP.nc"
static inline void CC2420TransmitP__requestInitialBackoff(message_t *msg)
#line 653
{
  CC2420TransmitP__myInitialBackoff = CC2420TransmitP__CcaControl__getInitialBackoff(CC2420TransmitP__getType(msg), msg, CC2420TransmitP__defaultInitialBackoff()) + 1;
}

# 193 "/home/csilab/upma/chips/cc2420/CC2420CsmaP.nc"
static inline void CC2420CsmaP__CC2420Transmit__sendDone(message_t *p_msg, error_t err)
#line 193
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 194
    CC2420CsmaP__sendErr = err;
#line 194
    __nesc_atomic_end(__nesc_atomic); }
  CC2420CsmaP__sendDone();
}

# 91 "/home/csilab/upma/chips/cc2420/CC2420Transmit.nc"
inline static void CC2420TransmitP__Send__sendDone(message_t * p_msg, error_t error){
#line 91
  CC2420CsmaP__CC2420Transmit__sendDone(p_msg, error);
#line 91
}
#line 91
# 454 "/home/csilab/upma/chips/cc2420/CC2420TransmitP.nc"
static inline void CC2420TransmitP__TXFIFO__writeDone(uint8_t *tx_buf, uint8_t tx_len, 
error_t error)
#line 455
{

  CC2420TransmitP__CSN__set();
  if (CC2420TransmitP__m_state == CC2420TransmitP__S_CANCEL) {
      /* atomic removed: atomic calls only */
#line 459
      {
        CC2420TransmitP__CSN__clr();
        CC2420TransmitP__SFLUSHTX__strobe();
        CC2420TransmitP__CSN__set();
      }
      CC2420TransmitP__releaseSpiResource();
      CC2420TransmitP__m_state = CC2420TransmitP__S_STARTED;
      CC2420TransmitP__Send__sendDone(CC2420TransmitP__m_msg, ECANCEL);
    }
  else {
#line 468
    if (!CC2420TransmitP__m_cca) {
        /* atomic removed: atomic calls only */
#line 469
        {
          CC2420TransmitP__m_state = CC2420TransmitP__S_BEGIN_TRANSMIT;
        }
        CC2420TransmitP__attemptSend();
      }
    else {
        CC2420TransmitP__releaseSpiResource();
        /* atomic removed: atomic calls only */
#line 476
        {
          CC2420TransmitP__m_state = CC2420TransmitP__S_SAMPLE_CCA;
        }

        CC2420TransmitP__requestInitialBackoff(CC2420TransmitP__m_msg);
        CC2420TransmitP__BackoffTimer__start(CC2420TransmitP__myInitialBackoff);
      }
    }
}

# 339 "/home/csilab/upma/chips/cc2420/CC2420ReceiveP.nc"
static inline void CC2420ReceiveP__RXFIFO__writeDone(uint8_t *tx_buf, uint8_t tx_len, error_t error)
#line 339
{
}

# 373 "/home/csilab/tinyos/tos/chips/cc2420/spi/CC2420SpiP.nc"
static inline void CC2420SpiP__Fifo__default__writeDone(uint8_t addr, uint8_t *tx_buf, uint8_t tx_len, error_t error)
#line 373
{
}

# 91 "/home/csilab/tinyos/tos/chips/cc2420/interfaces/CC2420Fifo.nc"
inline static void CC2420SpiP__Fifo__writeDone(uint8_t arg_0x7f7fecc8f6e0, uint8_t * data, uint8_t length, error_t error){
#line 91
  switch (arg_0x7f7fecc8f6e0) {
#line 91
    case CC2420_TXFIFO:
#line 91
      CC2420TransmitP__TXFIFO__writeDone(data, length, error);
#line 91
      break;
#line 91
    case CC2420_RXFIFO:
#line 91
      CC2420ReceiveP__RXFIFO__writeDone(data, length, error);
#line 91
      break;
#line 91
    default:
#line 91
      CC2420SpiP__Fifo__default__writeDone(arg_0x7f7fecc8f6e0, data, length, error);
#line 91
      break;
#line 91
    }
#line 91
}
#line 91
# 63 "/home/csilab/tinyos/tos/chips/cc2420/interfaces/CC2420Register.nc"
inline static cc2420_status_t CC2420ControlP__TXCTRL__write(uint16_t data){
#line 63
  unsigned char __nesc_result;
#line 63

#line 63
  __nesc_result = CC2420SpiP__Reg__write(CC2420_TXCTRL, data);
#line 63

#line 63
  return __nesc_result;
#line 63
}
#line 63
# 533 "/home/csilab/tinyos/tos/chips/cc2420/control/CC2420ControlP.nc"
static inline void CC2420ControlP__writeTxctrl(void )
#line 533
{
  /* atomic removed: atomic calls only */
#line 534
  {
    CC2420ControlP__TXCTRL__write((((2 << CC2420_TXCTRL_TXMIXBUF_CUR) | (
    3 << CC2420_TXCTRL_PA_CURRENT)) | (
    1 << CC2420_TXCTRL_RESERVED)) | ((
    31 & 0x1F) << CC2420_TXCTRL_PA_LEVEL));
  }
}

# 63 "/home/csilab/tinyos/tos/chips/cc2420/interfaces/CC2420Register.nc"
inline static cc2420_status_t CC2420ControlP__RXCTRL1__write(uint16_t data){
#line 63
  unsigned char __nesc_result;
#line 63

#line 63
  __nesc_result = CC2420SpiP__Reg__write(CC2420_RXCTRL1, data);
#line 63

#line 63
  return __nesc_result;
#line 63
}
#line 63
inline static cc2420_status_t CC2420ControlP__IOCFG0__write(uint16_t data){
#line 63
  unsigned char __nesc_result;
#line 63

#line 63
  __nesc_result = CC2420SpiP__Reg__write(CC2420_IOCFG0, data);
#line 63

#line 63
  return __nesc_result;
#line 63
}
#line 63
# 53 "/home/csilab/tinyos/tos/chips/cc2420/interfaces/CC2420Strobe.nc"
inline static cc2420_status_t CC2420ControlP__SXOSCON__strobe(void ){
#line 53
  unsigned char __nesc_result;
#line 53

#line 53
  __nesc_result = CC2420SpiP__Strobe__strobe(CC2420_SXOSCON);
#line 53

#line 53
  return __nesc_result;
#line 53
}
#line 53
# 90 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port14__enable(void )
#line 90
{
#line 90
  P1IE |= 1 << 4;
}

# 42 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__HplInterrupt__enable(void ){
#line 42
  HplMsp430InterruptP__Port14__enable();
#line 42
}
#line 42
# 142 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port14__edge(bool l2h)
#line 142
{
  /* atomic removed: atomic calls only */
#line 143
  {
    if (l2h) {
#line 144
      P1IES &= ~(1 << 4);
      }
    else {
#line 145
      P1IES |= 1 << 4;
      }
  }
}

# 67 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__HplInterrupt__edge(bool low_to_high){
#line 67
  HplMsp430InterruptP__Port14__edge(low_to_high);
#line 67
}
#line 67
# 106 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port14__clear(void )
#line 106
{
#line 106
  P1IFG &= ~(1 << 4);
}

# 52 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__HplInterrupt__clear(void ){
#line 52
  HplMsp430InterruptP__Port14__clear();
#line 52
}
#line 52
# 98 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port14__disable(void )
#line 98
{
#line 98
  P1IE &= ~(1 << 4);
}

# 47 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__HplInterrupt__disable(void ){
#line 47
  HplMsp430InterruptP__Port14__disable();
#line 47
}
#line 47
# 69 "/home/csilab/tinyos/tos/chips/msp430/pins/Msp430InterruptC.nc"
static inline error_t /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__Interrupt__disable(void )
#line 69
{
  /* atomic removed: atomic calls only */
#line 70
  {
    /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__HplInterrupt__disable();
    /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__HplInterrupt__clear();
  }
  return SUCCESS;
}

#line 52
static inline error_t /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__enable(bool rising)
#line 52
{
  /* atomic removed: atomic calls only */
#line 53
  {
    /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__Interrupt__disable();
    /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__HplInterrupt__edge(rising);
    /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__HplInterrupt__enable();
  }
  return SUCCESS;
}

static inline error_t /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__Interrupt__enableRisingEdge(void )
#line 61
{
  return /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__enable(TRUE);
}

# 53 "/home/csilab/tinyos/tos/interfaces/GpioInterrupt.nc"
inline static error_t CC2420ControlP__InterruptCCA__enableRisingEdge(void ){
#line 53
  unsigned char __nesc_result;
#line 53

#line 53
  __nesc_result = /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__Interrupt__enableRisingEdge();
#line 53

#line 53
  return __nesc_result;
#line 53
}
#line 53
# 63 "/home/csilab/tinyos/tos/chips/cc2420/interfaces/CC2420Register.nc"
inline static cc2420_status_t CC2420ControlP__IOCFG1__write(uint16_t data){
#line 63
  unsigned char __nesc_result;
#line 63

#line 63
  __nesc_result = CC2420SpiP__Reg__write(CC2420_IOCFG1, data);
#line 63

#line 63
  return __nesc_result;
#line 63
}
#line 63
# 224 "/home/csilab/tinyos/tos/chips/cc2420/control/CC2420ControlP.nc"
static inline error_t CC2420ControlP__CC2420Power__startOscillator(void )
#line 224
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 225
    {
      if (CC2420ControlP__m_state != CC2420ControlP__S_VREG_STARTED) {
          {
            unsigned char __nesc_temp = 
#line 227
            FAIL;

            {
#line 227
              __nesc_atomic_end(__nesc_atomic); 
#line 227
              return __nesc_temp;
            }
          }
        }
#line 230
      CC2420ControlP__m_state = CC2420ControlP__S_XOSC_STARTING;
      CC2420ControlP__IOCFG1__write(CC2420_SFDMUX_XOSC16M_STABLE << 
      CC2420_IOCFG1_CCAMUX);

      CC2420ControlP__InterruptCCA__enableRisingEdge();
      CC2420ControlP__SXOSCON__strobe();

      CC2420ControlP__IOCFG0__write((1 << CC2420_IOCFG0_FIFOP_POLARITY) | (
      127 << CC2420_IOCFG0_FIFOP_THR));

      CC2420ControlP__writeFsctrl();
      CC2420ControlP__writeMdmctrl0();

      CC2420ControlP__RXCTRL1__write(((((((1 << CC2420_RXCTRL1_RXBPF_LOCUR) | (
      1 << CC2420_RXCTRL1_LOW_LOWGAIN)) | (
      1 << CC2420_RXCTRL1_HIGH_HGM)) | (
      1 << CC2420_RXCTRL1_LNA_CAP_ARRAY)) | (
      1 << CC2420_RXCTRL1_RXMIX_TAIL)) | (
      1 << CC2420_RXCTRL1_RXMIX_VCM)) | (
      2 << CC2420_RXCTRL1_RXMIX_CURRENT));

      CC2420ControlP__writeTxctrl();
    }
#line 252
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

# 71 "/home/csilab/tinyos/tos/chips/cc2420/interfaces/CC2420Power.nc"
inline static error_t CC2420CsmaP__CC2420Power__startOscillator(void ){
#line 71
  unsigned char __nesc_result;
#line 71

#line 71
  __nesc_result = CC2420ControlP__CC2420Power__startOscillator();
#line 71

#line 71
  return __nesc_result;
#line 71
}
#line 71
# 202 "/home/csilab/upma/chips/cc2420/CC2420CsmaP.nc"
static inline void CC2420CsmaP__Resource__granted(void )
#line 202
{



  CC2420CsmaP__CC2420Power__startOscillator();
}

# 102 "/home/csilab/tinyos/tos/interfaces/Resource.nc"
inline static void CC2420ControlP__Resource__granted(void ){
#line 102
  CC2420CsmaP__Resource__granted();
#line 102
}
#line 102
# 41 "/home/csilab/tinyos/tos/interfaces/GeneralIO.nc"
inline static void CC2420ControlP__CSN__clr(void ){
#line 41
  /*HplCC2420PinsC.CSNM*/Msp430GpioC__4__GeneralIO__clr();
#line 41
}
#line 41
# 413 "/home/csilab/tinyos/tos/chips/cc2420/control/CC2420ControlP.nc"
static inline void CC2420ControlP__SpiResource__granted(void )
#line 413
{
  CC2420ControlP__CSN__clr();
  CC2420ControlP__Resource__granted();
}

# 67 "/home/csilab/tinyos/tos/interfaces/TaskBasic.nc"
inline static error_t CC2420ControlP__syncDone__postTask(void ){
#line 67
  unsigned char __nesc_result;
#line 67

#line 67
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(CC2420ControlP__syncDone);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 120 "/home/csilab/tinyos/tos/interfaces/Resource.nc"
inline static error_t CC2420ControlP__SyncResource__release(void ){
#line 120
  unsigned char __nesc_result;
#line 120

#line 120
  __nesc_result = CC2420SpiP__Resource__release(/*CC2420ControlC.SyncSpiC*/CC2420SpiC__1__CLIENT_ID);
#line 120

#line 120
  return __nesc_result;
#line 120
}
#line 120
# 40 "/home/csilab/tinyos/tos/interfaces/GeneralIO.nc"
inline static void CC2420ControlP__CSN__set(void ){
#line 40
  /*HplCC2420PinsC.CSNM*/Msp430GpioC__4__GeneralIO__set();
#line 40
}
#line 40
# 53 "/home/csilab/tinyos/tos/chips/cc2420/interfaces/CC2420Strobe.nc"
inline static cc2420_status_t CC2420ControlP__SRXON__strobe(void ){
#line 53
  unsigned char __nesc_result;
#line 53

#line 53
  __nesc_result = CC2420SpiP__Strobe__strobe(CC2420_SRXON);
#line 53

#line 53
  return __nesc_result;
#line 53
}
#line 53
inline static cc2420_status_t CC2420ControlP__SRFOFF__strobe(void ){
#line 53
  unsigned char __nesc_result;
#line 53

#line 53
  __nesc_result = CC2420SpiP__Strobe__strobe(CC2420_SRFOFF);
#line 53

#line 53
  return __nesc_result;
#line 53
}
#line 53
# 399 "/home/csilab/tinyos/tos/chips/cc2420/control/CC2420ControlP.nc"
static inline void CC2420ControlP__SyncResource__granted(void )
#line 399
{
  CC2420ControlP__CSN__clr();
  CC2420ControlP__SRFOFF__strobe();
  CC2420ControlP__writeFsctrl();
  CC2420ControlP__writeMdmctrl0();
  CC2420ControlP__writeId();
  CC2420ControlP__CSN__set();
  CC2420ControlP__CSN__clr();
  CC2420ControlP__SRXON__strobe();
  CC2420ControlP__CSN__set();
  CC2420ControlP__SyncResource__release();
  CC2420ControlP__syncDone__postTask();
}

#line 545
static inline void CC2420ControlP__ReadRssi__default__readDone(error_t error, uint16_t data)
#line 545
{
}

# 63 "/home/csilab/tinyos/tos/interfaces/Read.nc"
inline static void CC2420ControlP__ReadRssi__readDone(error_t result, CC2420ControlP__ReadRssi__val_t val){
#line 63
  CC2420ControlP__ReadRssi__default__readDone(result, val);
#line 63
}
#line 63
# 120 "/home/csilab/tinyos/tos/interfaces/Resource.nc"
inline static error_t CC2420ControlP__RssiResource__release(void ){
#line 120
  unsigned char __nesc_result;
#line 120

#line 120
  __nesc_result = CC2420SpiP__Resource__release(/*CC2420ControlC.RssiResource*/CC2420SpiC__2__CLIENT_ID);
#line 120

#line 120
  return __nesc_result;
#line 120
}
#line 120
# 287 "/home/csilab/tinyos/tos/chips/cc2420/spi/CC2420SpiP.nc"
static inline cc2420_status_t CC2420SpiP__Reg__read(uint8_t addr, uint16_t *data)
#line 287
{

  cc2420_status_t status = 0;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 291
    {
      if (CC2420SpiP__WorkingState__isIdle()) {
          {
            unsigned char __nesc_temp = 
#line 293
            status;

            {
#line 293
              __nesc_atomic_end(__nesc_atomic); 
#line 293
              return __nesc_temp;
            }
          }
        }
    }
#line 297
    __nesc_atomic_end(__nesc_atomic); }
#line 297
  status = CC2420SpiP__SpiByte__write(addr | 0x40);
  *data = (uint16_t )CC2420SpiP__SpiByte__write(0) << 8;
  *data |= CC2420SpiP__SpiByte__write(0);

  return status;
}

# 55 "/home/csilab/tinyos/tos/chips/cc2420/interfaces/CC2420Register.nc"
inline static cc2420_status_t CC2420ControlP__RSSI__read(uint16_t *data){
#line 55
  unsigned char __nesc_result;
#line 55

#line 55
  __nesc_result = CC2420SpiP__Reg__read(CC2420_RSSI, data);
#line 55

#line 55
  return __nesc_result;
#line 55
}
#line 55
# 418 "/home/csilab/tinyos/tos/chips/cc2420/control/CC2420ControlP.nc"
static inline void CC2420ControlP__RssiResource__granted(void )
#line 418
{
  uint16_t data = 0;

#line 420
  CC2420ControlP__CSN__clr();
  CC2420ControlP__RSSI__read(&data);
  CC2420ControlP__CSN__set();

  CC2420ControlP__RssiResource__release();
  data += 0x7f;
  data &= 0x00ff;
  CC2420ControlP__ReadRssi__readDone(SUCCESS, data);
}

# 416 "/home/csilab/upma/chips/cc2420/CC2420TransmitP.nc"
static inline void CC2420TransmitP__SpiResource__granted(void )
#line 416
{
  uint8_t cur_state;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 419
    {
      cur_state = CC2420TransmitP__m_state;
    }
#line 421
    __nesc_atomic_end(__nesc_atomic); }

  switch (cur_state) {
      case CC2420TransmitP__S_LOAD: 
        CC2420TransmitP__loadTXFIFO();
      break;

      case CC2420TransmitP__S_BEGIN_TRANSMIT: 
        CC2420TransmitP__attemptSend();
      break;

      case CC2420TransmitP__S_CANCEL: 
        CC2420TransmitP__CSN__clr();
      CC2420TransmitP__SFLUSHTX__strobe();
      CC2420TransmitP__CSN__set();
      CC2420TransmitP__releaseSpiResource();
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 437
        {
          CC2420TransmitP__m_state = CC2420TransmitP__S_STARTED;
        }
#line 439
        __nesc_atomic_end(__nesc_atomic); }
      CC2420TransmitP__Send__sendDone(CC2420TransmitP__m_msg, ECANCEL);
      break;

      default: 
        CC2420TransmitP__releaseSpiResource();
      break;
    }
}

# 199 "/home/csilab/upma/chips/cc2420/CC2420ReceiveP.nc"
static inline void CC2420ReceiveP__SpiResource__granted(void )
#line 199
{
  CC2420ReceiveP__receive();
}

# 367 "/home/csilab/tinyos/tos/chips/cc2420/spi/CC2420SpiP.nc"
static inline void CC2420SpiP__Resource__default__granted(uint8_t id)
#line 367
{
}

# 102 "/home/csilab/tinyos/tos/interfaces/Resource.nc"
inline static void CC2420SpiP__Resource__granted(uint8_t arg_0x7f7fecc90360){
#line 102
  switch (arg_0x7f7fecc90360) {
#line 102
    case /*CC2420ControlC.Spi*/CC2420SpiC__0__CLIENT_ID:
#line 102
      CC2420ControlP__SpiResource__granted();
#line 102
      break;
#line 102
    case /*CC2420ControlC.SyncSpiC*/CC2420SpiC__1__CLIENT_ID:
#line 102
      CC2420ControlP__SyncResource__granted();
#line 102
      break;
#line 102
    case /*CC2420ControlC.RssiResource*/CC2420SpiC__2__CLIENT_ID:
#line 102
      CC2420ControlP__RssiResource__granted();
#line 102
      break;
#line 102
    case /*CC2420TransmitC.Spi*/CC2420SpiC__3__CLIENT_ID:
#line 102
      CC2420TransmitP__SpiResource__granted();
#line 102
      break;
#line 102
    case /*CC2420ReceiveC.Spi*/CC2420SpiC__4__CLIENT_ID:
#line 102
      CC2420ReceiveP__SpiResource__granted();
#line 102
      break;
#line 102
    default:
#line 102
      CC2420SpiP__Resource__default__granted(arg_0x7f7fecc90360);
#line 102
      break;
#line 102
    }
#line 102
}
#line 102
# 358 "/home/csilab/tinyos/tos/chips/cc2420/spi/CC2420SpiP.nc"
static inline void CC2420SpiP__grant__runTask(void )
#line 358
{
  uint8_t holder;

#line 360
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 360
    {
      holder = CC2420SpiP__m_holder;
    }
#line 362
    __nesc_atomic_end(__nesc_atomic); }
  CC2420SpiP__Resource__granted(holder);
}

# 63 "/home/csilab/tinyos/tos/chips/cc2420/interfaces/CC2420Register.nc"
inline static cc2420_status_t CC2420ControlP__FSCTRL__write(uint16_t data){
#line 63
  unsigned char __nesc_result;
#line 63

#line 63
  __nesc_result = CC2420SpiP__Reg__write(CC2420_FSCTRL, data);
#line 63

#line 63
  return __nesc_result;
#line 63
}
#line 63
inline static cc2420_status_t CC2420ControlP__MDMCTRL0__write(uint16_t data){
#line 63
  unsigned char __nesc_result;
#line 63

#line 63
  __nesc_result = CC2420SpiP__Reg__write(CC2420_MDMCTRL0, data);
#line 63

#line 63
  return __nesc_result;
#line 63
}
#line 63
# 63 "/home/csilab/tinyos/tos/chips/cc2420/interfaces/CC2420Ram.nc"
inline static cc2420_status_t CC2420ControlP__IEEEADR__write(uint8_t offset, uint8_t * data, uint8_t length){
#line 63
  unsigned char __nesc_result;
#line 63

#line 63
  __nesc_result = CC2420SpiP__Ram__write(CC2420_RAM_IEEEADR, offset, data, length);
#line 63

#line 63
  return __nesc_result;
#line 63
}
#line 63
# 249 "/home/csilab/upma/chips/cc2420/CC2420ActiveMessageP.nc"
static inline void CC2420ActiveMessageP__CC2420Config__syncDone(error_t error)
#line 249
{
}

# 373 "/home/csilab/upma/chips/cc2420/CC2420ReceiveP.nc"
static inline void CC2420ReceiveP__CC2420Config__syncDone(error_t error)
#line 373
{
}

# 55 "/home/csilab/tinyos/tos/chips/cc2420/interfaces/CC2420Config.nc"
inline static void CC2420ControlP__CC2420Config__syncDone(error_t error){
#line 55
  CC2420ReceiveP__CC2420Config__syncDone(error);
#line 55
  CC2420ActiveMessageP__CC2420Config__syncDone(error);
#line 55
}
#line 55
# 469 "/home/csilab/tinyos/tos/chips/cc2420/control/CC2420ControlP.nc"
static inline void CC2420ControlP__syncDone__runTask(void )
#line 469
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 470
    CC2420ControlP__m_sync_busy = FALSE;
#line 470
    __nesc_atomic_end(__nesc_atomic); }
  CC2420ControlP__CC2420Config__syncDone(SUCCESS);
}

# 88 "/home/csilab/tinyos/tos/interfaces/Resource.nc"
inline static error_t CC2420ControlP__SyncResource__request(void ){
#line 88
  unsigned char __nesc_result;
#line 88

#line 88
  __nesc_result = CC2420SpiP__Resource__request(/*CC2420ControlC.SyncSpiC*/CC2420SpiC__1__CLIENT_ID);
#line 88

#line 88
  return __nesc_result;
#line 88
}
#line 88
# 323 "/home/csilab/tinyos/tos/chips/cc2420/control/CC2420ControlP.nc"
static inline error_t CC2420ControlP__CC2420Config__sync(void )
#line 323
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 324
    {
      if (CC2420ControlP__m_sync_busy) {
          {
            unsigned char __nesc_temp = 
#line 326
            FAIL;

            {
#line 326
              __nesc_atomic_end(__nesc_atomic); 
#line 326
              return __nesc_temp;
            }
          }
        }
#line 329
      CC2420ControlP__m_sync_busy = TRUE;
      if (CC2420ControlP__m_state == CC2420ControlP__S_XOSC_STARTED) {
          CC2420ControlP__SyncResource__request();
        }
      else 
#line 332
        {
          CC2420ControlP__syncDone__postTask();
        }
    }
#line 335
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

#line 465
static inline void CC2420ControlP__sync__runTask(void )
#line 465
{
  CC2420ControlP__CC2420Config__sync();
}

# 150 "SendingC.nc"
static inline void /*TestPureTdmaC.SendingApp.App*/SendingC__0__SplitControl__stopDone(error_t err)
{
}

# 138 "/home/csilab/tinyos/tos/interfaces/SplitControl.nc"
inline static void PureTDMASchedulerP__SplitControl__stopDone(error_t error){
#line 138
  /*TestPureTdmaC.SendingApp.App*/SendingC__0__SplitControl__stopDone(error);
#line 138
}
#line 138
# 388 "/home/csilab/upma/lib/macs/pure-tdma/PureTDMASchedulerP.nc"
static inline void PureTDMASchedulerP__RadioPowerControl__stopDone(error_t error)
#line 388
{
  if (PureTDMASchedulerP__requestStop) {
      PureTDMASchedulerP__requestStop = FALSE;
      PureTDMASchedulerP__SplitControl__stopDone(error);
    }
}

# 62 "/home/csilab/tinyos/tos/system/NoLedsC.nc"
static inline void NoLedsC__Leds__led2Off(void )
#line 62
{
}

# 94 "/home/csilab/tinyos/tos/interfaces/Leds.nc"
inline static void PowerCycleP__Leds__led2Off(void ){
#line 94
  NoLedsC__Leds__led2Off();
#line 94
}
#line 94
# 118 "/home/csilab/upma/chips/cc2420/PowerCycleP.nc"
static inline void PowerCycleP__RadioPowerControl__stopDone(error_t error)
#line 118
{
  PowerCycleP__Leds__led2Off();
}

# 42 "/home/csilab/upma/interfaces/RadioPowerControl.nc"
inline static void CC2420CsmaP__RadioPowerControl__stopDone(error_t error){
#line 42
  PowerCycleP__RadioPowerControl__stopDone(error);
#line 42
  PureTDMASchedulerP__RadioPowerControl__stopDone(error);
#line 42
}
#line 42
# 110 "/home/csilab/upma/chips/cc2420/CC2420CsmaP.nc"
static inline void CC2420CsmaP__signalStopDone__runTask(void )
#line 110
{
  CC2420CsmaP__RadioPowerControl__stopDone(SUCCESS);
}

# 61 "/home/csilab/tinyos/tos/system/NoLedsC.nc"
static inline void NoLedsC__Leds__led2On(void )
#line 61
{
}

# 89 "/home/csilab/tinyos/tos/interfaces/Leds.nc"
inline static void PowerCycleP__Leds__led2On(void ){
#line 89
  NoLedsC__Leds__led2On();
#line 89
}
#line 89
# 113 "/home/csilab/upma/chips/cc2420/PowerCycleP.nc"
static inline void PowerCycleP__RadioPowerControl__startDone(error_t error)
#line 113
{
  PowerCycleP__Leds__led2On();
  PowerCycleP__getCca__postTask();
}

# 36 "/home/csilab/upma/interfaces/RadioPowerControl.nc"
inline static void CC2420CsmaP__RadioPowerControl__startDone(error_t error){
#line 36
  PowerCycleP__RadioPowerControl__startDone(error);
#line 36
  PureTDMASchedulerP__RadioPowerControl__startDone(error);
#line 36
}
#line 36
# 87 "/home/csilab/upma/chips/cc2420/CC2420CsmaP.nc"
static inline void CC2420CsmaP__signalStartDone__runTask(void )
#line 87
{
  CC2420CsmaP__RadioPowerControl__startDone(SUCCESS);
}

# 113 "/home/csilab/tinyos/tos/interfaces/SplitControl.nc"
inline static void PureTDMASchedulerP__SplitControl__startDone(error_t error){
#line 113
  /*TestPureTdmaC.SendingApp.App*/SendingC__0__SplitControl__startDone(error);
#line 113
}
#line 113
# 125 "/home/csilab/tinyos/tos/interfaces/Send.nc"
inline static void * CC2420ActiveMessageP__SubSend__getPayload(message_t * msg, uint8_t len){
#line 125
  void *__nesc_result;
#line 125

#line 125
  __nesc_result = CC2420TinyosNetworkP__ActiveSend__getPayload(msg, len);
#line 125

#line 125
  return __nesc_result;
#line 125
}
#line 125
# 220 "/home/csilab/upma/chips/cc2420/CC2420ActiveMessageP.nc"
static inline void *CC2420ActiveMessageP__Packet__getPayload(message_t *msg, uint8_t len)
#line 220
{
  return CC2420ActiveMessageP__SubSend__getPayload(msg, len);
}

# 126 "/home/csilab/tinyos/tos/interfaces/Packet.nc"
inline static void * /*TestPureTdmaC.SendingApp.App*/SendingC__0__Packet__getPayload(message_t * msg, uint8_t len){
#line 126
  void *__nesc_result;
#line 126

#line 126
  __nesc_result = CC2420ActiveMessageP__Packet__getPayload(msg, len);
#line 126

#line 126
  return __nesc_result;
#line 126
}
#line 126
# 64 "/home/csilab/tinyos/tos/lib/timer/Timer.nc"
inline static void /*TestPureTdmaC.SendingApp.App*/SendingC__0__SendTimer__startPeriodic(uint32_t dt){
#line 64
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startPeriodic(1U, dt);
#line 64
}
#line 64
# 159 "/home/csilab/tinyos/tos/lib/timer/VirtualizeTimerC.nc"
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startOneShot(uint8_t num, uint32_t dt)
{
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__startTimer(num, /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__getNow(), dt, TRUE);
}

# 73 "/home/csilab/tinyos/tos/lib/timer/Timer.nc"
inline static void /*TestPureTdmaC.SendingApp.App*/SendingC__0__StartTimer__startOneShot(uint32_t dt){
#line 73
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startOneShot(4U, dt);
#line 73
}
#line 73
# 66 "/home/csilab/upma/system/GenericSlotterP.nc"
static inline error_t GenericSlotterP__AsyncStdControl__start(void )
#line 66
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 67
    GenericSlotterP__nextFire = GenericSlotterP__Alarm__getNow() + GenericSlotterP__slotLength;
#line 67
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

# 95 "/home/csilab/tinyos/tos/interfaces/AsyncStdControl.nc"
inline static error_t PureTDMASchedulerP__GenericSlotter__start(void ){
#line 95
  unsigned char __nesc_result;
#line 95

#line 95
  __nesc_result = GenericSlotterP__AsyncStdControl__start();
#line 95

#line 95
  return __nesc_result;
#line 95
}
#line 95
# 31 "/home/csilab/upma/interfaces/SlotterControl.nc"
inline static error_t PureTDMASchedulerP__SlotterControl__synchronize(uint8_t toSlot){
#line 31
  unsigned char __nesc_result;
#line 31

#line 31
  __nesc_result = GenericSlotterP__SlotterControl__synchronize(toSlot);
#line 31

#line 31
  return __nesc_result;
#line 31
}
#line 31
# 254 "/home/csilab/upma/chips/cc2420/CC2420CsmaP.nc"
static inline void CC2420CsmaP__stopDone_task__runTask(void )
#line 254
{
#line 266
  CC2420CsmaP__SplitControlState__forceState(CC2420CsmaP__S_STOPPED);
  CC2420CsmaP__RadioPowerControl__stopDone(SUCCESS);
}

# 120 "/home/csilab/tinyos/tos/interfaces/Resource.nc"
inline static error_t CC2420ControlP__SpiResource__release(void ){
#line 120
  unsigned char __nesc_result;
#line 120

#line 120
  __nesc_result = CC2420SpiP__Resource__release(/*CC2420ControlC.Spi*/CC2420SpiC__0__CLIENT_ID);
#line 120

#line 120
  return __nesc_result;
#line 120
}
#line 120
# 196 "/home/csilab/tinyos/tos/chips/cc2420/control/CC2420ControlP.nc"
static inline error_t CC2420ControlP__Resource__release(void )
#line 196
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 197
    {
      CC2420ControlP__CSN__set();
      {
        unsigned char __nesc_temp = 
#line 199
        CC2420ControlP__SpiResource__release();

        {
#line 199
          __nesc_atomic_end(__nesc_atomic); 
#line 199
          return __nesc_temp;
        }
      }
    }
#line 202
    __nesc_atomic_end(__nesc_atomic); }
}

# 120 "/home/csilab/tinyos/tos/interfaces/Resource.nc"
inline static error_t CC2420CsmaP__Resource__release(void ){
#line 120
  unsigned char __nesc_result;
#line 120

#line 120
  __nesc_result = CC2420ControlP__Resource__release();
#line 120

#line 120
  return __nesc_result;
#line 120
}
#line 120
# 268 "/home/csilab/tinyos/tos/chips/cc2420/control/CC2420ControlP.nc"
static inline error_t CC2420ControlP__CC2420Power__rxOn(void )
#line 268
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 269
    {
      if (CC2420ControlP__m_state != CC2420ControlP__S_XOSC_STARTED) {
          {
            unsigned char __nesc_temp = 
#line 271
            FAIL;

            {
#line 271
              __nesc_atomic_end(__nesc_atomic); 
#line 271
              return __nesc_temp;
            }
          }
        }
#line 273
      CC2420ControlP__SRXON__strobe();
    }
#line 274
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

# 90 "/home/csilab/tinyos/tos/chips/cc2420/interfaces/CC2420Power.nc"
inline static error_t CC2420CsmaP__CC2420Power__rxOn(void ){
#line 90
  unsigned char __nesc_result;
#line 90

#line 90
  __nesc_result = CC2420ControlP__CC2420Power__rxOn();
#line 90

#line 90
  return __nesc_result;
#line 90
}
#line 90
# 86 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port10__enable(void )
#line 86
{
#line 86
  P1IE |= 1 << 0;
}

# 42 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__HplInterrupt__enable(void ){
#line 42
  HplMsp430InterruptP__Port10__enable();
#line 42
}
#line 42
# 118 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port10__edge(bool l2h)
#line 118
{
  /* atomic removed: atomic calls only */
#line 119
  {
    if (l2h) {
#line 120
      P1IES &= ~(1 << 0);
      }
    else {
#line 121
      P1IES |= 1 << 0;
      }
  }
}

# 67 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__HplInterrupt__edge(bool low_to_high){
#line 67
  HplMsp430InterruptP__Port10__edge(low_to_high);
#line 67
}
#line 67
# 52 "/home/csilab/tinyos/tos/chips/msp430/pins/Msp430InterruptC.nc"
static inline error_t /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__enable(bool rising)
#line 52
{
  /* atomic removed: atomic calls only */
#line 53
  {
    /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__Interrupt__disable();
    /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__HplInterrupt__edge(rising);
    /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__HplInterrupt__enable();
  }
  return SUCCESS;
}





static inline error_t /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__Interrupt__enableFallingEdge(void )
#line 65
{
  return /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__enable(FALSE);
}

# 54 "/home/csilab/tinyos/tos/interfaces/GpioInterrupt.nc"
inline static error_t CC2420ReceiveP__InterruptFIFOP__enableFallingEdge(void ){
#line 54
  unsigned char __nesc_result;
#line 54

#line 54
  __nesc_result = /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__Interrupt__enableFallingEdge();
#line 54

#line 54
  return __nesc_result;
#line 54
}
#line 54
# 133 "/home/csilab/upma/chips/cc2420/CC2420ReceiveP.nc"
static inline error_t CC2420ReceiveP__StdControl__start(void )
#line 133
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 134
    {
      CC2420ReceiveP__reset_state();
      CC2420ReceiveP__m_state = CC2420ReceiveP__S_STARTED;
      CC2420ReceiveP__receivingPacket = FALSE;




      CC2420ReceiveP__InterruptFIFOP__enableFallingEdge();
    }
#line 143
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

# 187 "/home/csilab/upma/chips/cc2420/CC2420TransmitP.nc"
static inline error_t CC2420TransmitP__AsyncStdControl__start(void )
#line 187
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 188
    {
      CC2420TransmitP__CaptureSFD__captureRisingEdge();
      CC2420TransmitP__m_state = CC2420TransmitP__S_STARTED;
      CC2420TransmitP__m_receiving = FALSE;
      CC2420TransmitP__abortSpiRelease = FALSE;
      CC2420TransmitP__m_tx_power = 0;
    }
#line 194
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

# 95 "/home/csilab/tinyos/tos/interfaces/AsyncStdControl.nc"
inline static error_t CC2420CsmaP__SubControl__start(void ){
#line 95
  unsigned char __nesc_result;
#line 95

#line 95
  __nesc_result = CC2420TransmitP__AsyncStdControl__start();
#line 95
  __nesc_result = ecombine(__nesc_result, CC2420ReceiveP__StdControl__start());
#line 95

#line 95
  return __nesc_result;
#line 95
}
#line 95
# 243 "/home/csilab/upma/chips/cc2420/CC2420CsmaP.nc"
static inline void CC2420CsmaP__startDone_task__runTask(void )
#line 243
{
  CC2420CsmaP__SubControl__start();



  CC2420CsmaP__CC2420Power__rxOn();
  CC2420CsmaP__Resource__release();
  CC2420CsmaP__SplitControlState__forceState(CC2420CsmaP__S_STARTED);
  CC2420CsmaP__RadioPowerControl__startDone(SUCCESS);
}

# 67 "/home/csilab/tinyos/tos/interfaces/TaskBasic.nc"
inline static error_t PrintfP__retrySend__postTask(void ){
#line 67
  unsigned char __nesc_result;
#line 67

#line 67
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(PrintfP__retrySend);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 80 "/home/csilab/tinyos/tos/interfaces/AMSend.nc"
inline static error_t PrintfP__AMSend__send(am_addr_t addr, message_t * msg, uint8_t len){
#line 80
  unsigned char __nesc_result;
#line 80

#line 80
  __nesc_result = /*PrintfC.SerialAMSenderC.AMQueueEntryP*/AMQueueEntryP__0__AMSend__send(addr, msg, len);
#line 80

#line 80
  return __nesc_result;
#line 80
}
#line 80
# 114 "/home/csilab/tinyos/tos/lib/printf/PrintfP.nc"
static inline void PrintfP__retrySend__runTask(void )
#line 114
{
  if (PrintfP__AMSend__send(AM_BROADCAST_ADDR, &PrintfP__printfMsg, sizeof(printf_msg_t )) != SUCCESS) {
    PrintfP__retrySend__postTask();
    }
}

# 110 "/home/csilab/tinyos/tos/interfaces/AMSend.nc"
inline static void /*PrintfC.SerialAMSenderC.AMQueueEntryP*/AMQueueEntryP__0__AMSend__sendDone(message_t * msg, error_t error){
#line 110
  PrintfP__AMSend__sendDone(msg, error);
#line 110
}
#line 110
# 65 "/home/csilab/tinyos/tos/system/AMQueueEntryP.nc"
static inline void /*PrintfC.SerialAMSenderC.AMQueueEntryP*/AMQueueEntryP__0__Send__sendDone(message_t *m, error_t err)
#line 65
{
  /*PrintfC.SerialAMSenderC.AMQueueEntryP*/AMQueueEntryP__0__AMSend__sendDone(m, err);
}

# 215 "/home/csilab/tinyos/tos/system/AMQueueImplP.nc"
static inline void /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__Send__default__sendDone(uint8_t id, message_t *msg, error_t err)
#line 215
{
}

# 100 "/home/csilab/tinyos/tos/interfaces/Send.nc"
inline static void /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__Send__sendDone(uint8_t arg_0x7f7fed178630, message_t * msg, error_t error){
#line 100
  switch (arg_0x7f7fed178630) {
#line 100
    case 0U:
#line 100
      /*PrintfC.SerialAMSenderC.AMQueueEntryP*/AMQueueEntryP__0__Send__sendDone(msg, error);
#line 100
      break;
#line 100
    default:
#line 100
      /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__Send__default__sendDone(arg_0x7f7fed178630, msg, error);
#line 100
      break;
#line 100
    }
#line 100
}
#line 100
# 126 "/home/csilab/tinyos/tos/system/AMQueueImplP.nc"
static inline void /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__CancelTask__runTask(void )
#line 126
{
  uint8_t i;
#line 127
  uint8_t j;
#line 127
  uint8_t mask;
#line 127
  uint8_t last;
  message_t *msg;

#line 129
  for (i = 0; i < 1 / 8 + 1; i++) {
      if (/*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__cancelMask[i]) {
          for (mask = 1, j = 0; j < 8; j++) {
              if (/*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__cancelMask[i] & mask) {
                  last = i * 8 + j;
                  msg = /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__queue[last].msg;
                  /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__queue[last].msg = (void *)0;
                  /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__cancelMask[i] &= ~mask;
                  /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__Send__sendDone(last, msg, ECANCEL);
                }
              mask <<= 1;
            }
        }
    }
}

#line 169
static inline void /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__errorTask__runTask(void )
#line 169
{
  /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__sendDone(/*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__current, /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__queue[/*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__current].msg, FAIL);
}

# 67 "/home/csilab/tinyos/tos/interfaces/TaskBasic.nc"
inline static error_t /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__errorTask__postTask(void ){
#line 67
  unsigned char __nesc_result;
#line 67

#line 67
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(/*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__errorTask);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 106 "/home/csilab/upma/lib/serial/SerialActiveMessageP.nc"
static inline uint8_t /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__Packet__payloadLength(message_t *msg)
#line 106
{
  serial_header_t *header = /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__getHeader(msg);

#line 108
  return __nesc_ntoh_uint8(header->length.nxdata);
}

# 78 "/home/csilab/tinyos/tos/interfaces/Packet.nc"
inline static uint8_t /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__Packet__payloadLength(message_t * msg){
#line 78
  unsigned char __nesc_result;
#line 78

#line 78
  __nesc_result = /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__Packet__payloadLength(msg);
#line 78

#line 78
  return __nesc_result;
#line 78
}
#line 78
# 65 "/home/csilab/tinyos/tos/system/AMQueueImplP.nc"
static inline void /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__nextPacket(void )
#line 65
{
  uint8_t i;

#line 67
  /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__current = (/*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__current + 1) % 1;
  for (i = 0; i < 1; i++) {
      if (/*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__queue[/*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__current].msg == (void *)0 || 
      /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__cancelMask[/*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__current / 8] & (1 << /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__current % 8)) 
        {
          /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__current = (/*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__current + 1) % 1;
        }
      else {
          break;
        }
    }
  if (i >= 1) {
#line 78
    /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__current = 1;
    }
}

#line 174
static inline void /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__tryToSend(void )
#line 174
{
  /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__nextPacket();
  if (/*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__current < 1) {
      error_t nextErr;
      message_t *nextMsg = /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__queue[/*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__current].msg;
      am_id_t nextId = /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__AMPacket__type(nextMsg);
      am_addr_t nextDest = /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__AMPacket__destination(nextMsg);
      uint8_t len = /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__Packet__payloadLength(nextMsg);

#line 182
      nextErr = /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__AMSend__send(nextId, nextDest, nextMsg, len);
      if (nextErr != SUCCESS) {
          /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__errorTask__postTask();
        }
    }
}

# 17 "/home/csilab/tinyos/tos/platforms/telosa/TelosSerialP.nc"
static inline void TelosSerialP__Resource__granted(void )
#line 17
{
}

# 218 "/home/csilab/tinyos/tos/chips/msp430/usart/Msp430UartP.nc"
static inline void /*Msp430Uart1P.UartP*/Msp430UartP__0__Resource__default__granted(uint8_t id)
#line 218
{
}

# 102 "/home/csilab/tinyos/tos/interfaces/Resource.nc"
inline static void /*Msp430Uart1P.UartP*/Msp430UartP__0__Resource__granted(uint8_t arg_0x7f7fed84f380){
#line 102
  switch (arg_0x7f7fed84f380) {
#line 102
    case /*PlatformSerialC.UartC*/Msp430Uart1C__0__CLIENT_ID:
#line 102
      TelosSerialP__Resource__granted();
#line 102
      break;
#line 102
    default:
#line 102
      /*Msp430Uart1P.UartP*/Msp430UartP__0__Resource__default__granted(arg_0x7f7fed84f380);
#line 102
      break;
#line 102
    }
#line 102
}
#line 102
# 101 "/home/csilab/tinyos/tos/chips/msp430/usart/Msp430UartP.nc"
static inline void /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartResource__granted(uint8_t id)
#line 101
{
  /*Msp430Uart1P.UartP*/Msp430UartP__0__Resource__granted(id);
}

# 202 "/home/csilab/tinyos/tos/system/ArbiterP.nc"
static inline void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__Resource__default__granted(uint8_t id)
#line 202
{
}

# 102 "/home/csilab/tinyos/tos/interfaces/Resource.nc"
inline static void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__Resource__granted(uint8_t arg_0x7f7fed25f4b0){
#line 102
  switch (arg_0x7f7fed25f4b0) {
#line 102
    case /*PlatformSerialC.UartC.UsartC*/Msp430Usart1C__0__CLIENT_ID:
#line 102
      /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartResource__granted(/*PlatformSerialC.UartC*/Msp430Uart1C__0__CLIENT_ID);
#line 102
      break;
#line 102
    default:
#line 102
      /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__Resource__default__granted(arg_0x7f7fed25f4b0);
#line 102
      break;
#line 102
    }
#line 102
}
#line 102
# 216 "/home/csilab/tinyos/tos/system/ArbiterP.nc"
static inline void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__default__configure(uint8_t id)
#line 216
{
}

# 59 "/home/csilab/tinyos/tos/interfaces/ResourceConfigure.nc"
inline static void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__configure(uint8_t arg_0x7f7fed25bc40){
#line 59
  switch (arg_0x7f7fed25bc40) {
#line 59
    case /*PlatformSerialC.UartC.UsartC*/Msp430Usart1C__0__CLIENT_ID:
#line 59
      /*Msp430Uart1P.UartP*/Msp430UartP__0__ResourceConfigure__configure(/*PlatformSerialC.UartC*/Msp430Uart1C__0__CLIENT_ID);
#line 59
      break;
#line 59
    default:
#line 59
      /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__default__configure(arg_0x7f7fed25bc40);
#line 59
      break;
#line 59
    }
#line 59
}
#line 59
# 190 "/home/csilab/tinyos/tos/system/ArbiterP.nc"
static inline void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__grantedTask__runTask(void )
#line 190
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 191
    {
      /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__resId = /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__reqResId;
      /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__state = /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__RES_BUSY;
    }
#line 194
    __nesc_atomic_end(__nesc_atomic); }
  /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__configure(/*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__resId);
  /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__Resource__granted(/*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__resId);
}

# 19 "/home/csilab/tinyos/tos/platforms/telosa/TelosSerialP.nc"
static inline msp430_uart_union_config_t *TelosSerialP__Msp430UartConfigure__getConfig(void )
#line 19
{
  return &TelosSerialP__msp430_uart_telos_config;
}

# 214 "/home/csilab/tinyos/tos/chips/msp430/usart/Msp430UartP.nc"
static inline msp430_uart_union_config_t */*Msp430Uart1P.UartP*/Msp430UartP__0__Msp430UartConfigure__default__getConfig(uint8_t id)
#line 214
{
  return &msp430_uart_default_config;
}

# 39 "/home/csilab/tinyos/tos/chips/msp430/usart/Msp430UartConfigure.nc"
inline static msp430_uart_union_config_t */*Msp430Uart1P.UartP*/Msp430UartP__0__Msp430UartConfigure__getConfig(uint8_t arg_0x7f7fed849d40){
#line 39
  union __nesc_unnamed4271 *__nesc_result;
#line 39

#line 39
  switch (arg_0x7f7fed849d40) {
#line 39
    case /*PlatformSerialC.UartC*/Msp430Uart1C__0__CLIENT_ID:
#line 39
      __nesc_result = TelosSerialP__Msp430UartConfigure__getConfig();
#line 39
      break;
#line 39
    default:
#line 39
      __nesc_result = /*Msp430Uart1P.UartP*/Msp430UartP__0__Msp430UartConfigure__default__getConfig(arg_0x7f7fed849d40);
#line 39
      break;
#line 39
    }
#line 39

#line 39
  return __nesc_result;
#line 39
}
#line 39
# 359 "/home/csilab/tinyos/tos/chips/msp430/usart/HplMsp430Usart1P.nc"
static inline void HplMsp430Usart1P__Usart__disableIntr(void )
#line 359
{
  HplMsp430Usart1P__IE2 &= ~(0x20 | 0x10);
}

#line 347
static inline void HplMsp430Usart1P__Usart__clrIntr(void )
#line 347
{
  HplMsp430Usart1P__IFG2 &= ~(0x20 | 0x10);
}

#line 159
static inline void HplMsp430Usart1P__Usart__resetUsart(bool reset)
#line 159
{
  if (reset) {
    U1CTL = 0x01;
    }
  else {
#line 163
    U1CTL &= ~0x01;
    }
}

# 65 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P36*/HplMsp430GeneralIOP__22__IO__selectModuleFunc(void )
#line 65
{
  /* atomic removed: atomic calls only */
#line 65
  * (volatile uint8_t * )27U |= 0x01 << 6;
}

# 92 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430Usart1P__UTXD__selectModuleFunc(void ){
#line 92
  /*HplMsp430GeneralIOC.P36*/HplMsp430GeneralIOP__22__IO__selectModuleFunc();
#line 92
}
#line 92
# 220 "/home/csilab/tinyos/tos/chips/msp430/usart/HplMsp430Usart1P.nc"
static inline void HplMsp430Usart1P__Usart__enableUartTx(void )
#line 220
{
  HplMsp430Usart1P__UTXD__selectModuleFunc();
  HplMsp430Usart1P__ME2 |= 0x20;
}

# 67 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P37*/HplMsp430GeneralIOP__23__IO__selectIOFunc(void )
#line 67
{
  /* atomic removed: atomic calls only */
#line 67
  * (volatile uint8_t * )27U &= ~(0x01 << 7);
}

# 99 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430Usart1P__URXD__selectIOFunc(void ){
#line 99
  /*HplMsp430GeneralIOC.P37*/HplMsp430GeneralIOP__23__IO__selectIOFunc();
#line 99
}
#line 99
# 236 "/home/csilab/tinyos/tos/chips/msp430/usart/HplMsp430Usart1P.nc"
static inline void HplMsp430Usart1P__Usart__disableUartRx(void )
#line 236
{
  HplMsp430Usart1P__ME2 &= ~0x10;
  HplMsp430Usart1P__URXD__selectIOFunc();
}

# 65 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P37*/HplMsp430GeneralIOP__23__IO__selectModuleFunc(void )
#line 65
{
  /* atomic removed: atomic calls only */
#line 65
  * (volatile uint8_t * )27U |= 0x01 << 7;
}

# 92 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430Usart1P__URXD__selectModuleFunc(void ){
#line 92
  /*HplMsp430GeneralIOC.P37*/HplMsp430GeneralIOP__23__IO__selectModuleFunc();
#line 92
}
#line 92
# 231 "/home/csilab/tinyos/tos/chips/msp430/usart/HplMsp430Usart1P.nc"
static inline void HplMsp430Usart1P__Usart__enableUartRx(void )
#line 231
{
  HplMsp430Usart1P__URXD__selectModuleFunc();
  HplMsp430Usart1P__ME2 |= 0x10;
}

# 67 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P36*/HplMsp430GeneralIOP__22__IO__selectIOFunc(void )
#line 67
{
  /* atomic removed: atomic calls only */
#line 67
  * (volatile uint8_t * )27U &= ~(0x01 << 6);
}

# 99 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430Usart1P__UTXD__selectIOFunc(void ){
#line 99
  /*HplMsp430GeneralIOC.P36*/HplMsp430GeneralIOP__22__IO__selectIOFunc();
#line 99
}
#line 99
# 225 "/home/csilab/tinyos/tos/chips/msp430/usart/HplMsp430Usart1P.nc"
static inline void HplMsp430Usart1P__Usart__disableUartTx(void )
#line 225
{
  HplMsp430Usart1P__ME2 &= ~0x20;
  HplMsp430Usart1P__UTXD__selectIOFunc();
}

#line 203
static inline void HplMsp430Usart1P__Usart__enableUart(void )
#line 203
{
  /* atomic removed: atomic calls only */
#line 204
  {
    HplMsp430Usart1P__UTXD__selectModuleFunc();
    HplMsp430Usart1P__URXD__selectModuleFunc();
  }
  HplMsp430Usart1P__ME2 |= 0x20 | 0x10;
}

#line 151
static inline void HplMsp430Usart1P__Usart__setUmctl(uint8_t control)
#line 151
{
  U1MCTL = control;
}

#line 140
static inline void HplMsp430Usart1P__Usart__setUbr(uint16_t control)
#line 140
{
  /* atomic removed: atomic calls only */
#line 141
  {
    U1BR0 = control & 0x00FF;
    U1BR1 = (control >> 8) & 0x00FF;
  }
}

#line 283
static inline void HplMsp430Usart1P__configUart(msp430_uart_union_config_t *config)
#line 283
{

  U1CTL = (config->uartRegisters.uctl & ~0x04) | 0x01;
  HplMsp430Usart1P__U1TCTL = config->uartRegisters.utctl;
  HplMsp430Usart1P__U1RCTL = config->uartRegisters.urctl;

  HplMsp430Usart1P__Usart__setUbr(config->uartRegisters.ubr);
  HplMsp430Usart1P__Usart__setUmctl(config->uartRegisters.umctl);
}

static inline void HplMsp430Usart1P__Usart__setModeUart(msp430_uart_union_config_t *config)
#line 293
{

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 295
    {
      HplMsp430Usart1P__Usart__resetUsart(TRUE);
      HplMsp430Usart1P__Usart__disableSpi();
      HplMsp430Usart1P__configUart(config);
      if (config->uartConfig.utxe == 1 && config->uartConfig.urxe == 1) {
          HplMsp430Usart1P__Usart__enableUart();
        }
      else {
#line 301
        if (config->uartConfig.utxe == 0 && config->uartConfig.urxe == 1) {
            HplMsp430Usart1P__Usart__disableUartTx();
            HplMsp430Usart1P__Usart__enableUartRx();
          }
        else {
#line 304
          if (config->uartConfig.utxe == 1 && config->uartConfig.urxe == 0) {
              HplMsp430Usart1P__Usart__disableUartRx();
              HplMsp430Usart1P__Usart__enableUartTx();
            }
          else 
#line 307
            {
              HplMsp430Usart1P__Usart__disableUart();
            }
          }
        }
#line 310
      HplMsp430Usart1P__Usart__resetUsart(FALSE);
      HplMsp430Usart1P__Usart__clrIntr();
      HplMsp430Usart1P__Usart__disableIntr();
    }
#line 313
    __nesc_atomic_end(__nesc_atomic); }

  return;
}

# 174 "/home/csilab/tinyos/tos/chips/msp430/usart/HplMsp430Usart.nc"
inline static void /*Msp430Uart1P.UartP*/Msp430UartP__0__Usart__setModeUart(msp430_uart_union_config_t *config){
#line 174
  HplMsp430Usart1P__Usart__setModeUart(config);
#line 174
}
#line 174
# 67 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P51*/HplMsp430GeneralIOP__33__IO__selectIOFunc(void )
#line 67
{
  /* atomic removed: atomic calls only */
#line 67
  * (volatile uint8_t * )51U &= ~(0x01 << 1);
}

# 99 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430Usart1P__SIMO__selectIOFunc(void ){
#line 99
  /*HplMsp430GeneralIOC.P51*/HplMsp430GeneralIOP__33__IO__selectIOFunc();
#line 99
}
#line 99
# 67 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P52*/HplMsp430GeneralIOP__34__IO__selectIOFunc(void )
#line 67
{
  /* atomic removed: atomic calls only */
#line 67
  * (volatile uint8_t * )51U &= ~(0x01 << 2);
}

# 99 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430Usart1P__SOMI__selectIOFunc(void ){
#line 99
  /*HplMsp430GeneralIOC.P52*/HplMsp430GeneralIOP__34__IO__selectIOFunc();
#line 99
}
#line 99
# 67 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P53*/HplMsp430GeneralIOP__35__IO__selectIOFunc(void )
#line 67
{
  /* atomic removed: atomic calls only */
#line 67
  * (volatile uint8_t * )51U &= ~(0x01 << 3);
}

# 99 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430Usart1P__UCLK__selectIOFunc(void ){
#line 99
  /*HplMsp430GeneralIOC.P53*/HplMsp430GeneralIOP__35__IO__selectIOFunc();
#line 99
}
#line 99
# 377 "/home/csilab/tinyos/tos/chips/msp430/usart/HplMsp430Usart1P.nc"
static inline void HplMsp430Usart1P__Usart__enableIntr(void )
#line 377
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 378
    {
      HplMsp430Usart1P__IFG2 &= ~(0x20 | 0x10);
      HplMsp430Usart1P__IE2 |= 0x20 | 0x10;
    }
#line 381
    __nesc_atomic_end(__nesc_atomic); }
}

# 182 "/home/csilab/tinyos/tos/chips/msp430/usart/HplMsp430Usart.nc"
inline static void /*Msp430Uart1P.UartP*/Msp430UartP__0__Usart__enableIntr(void ){
#line 182
  HplMsp430Usart1P__Usart__enableIntr();
#line 182
}
#line 182
# 189 "/home/csilab/tinyos/tos/system/AMQueueImplP.nc"
static inline void /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__AMSend__sendDone(am_id_t id, message_t *msg, error_t err)
#line 189
{





  if (/*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__current >= 1) {
      return;
    }
  if (/*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__queue[/*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__current].msg == msg) {
      /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__sendDone(/*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__current, msg, err);
    }
  else {
      ;
    }
}

# 110 "/home/csilab/tinyos/tos/interfaces/AMSend.nc"
inline static void /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMSend__sendDone(am_id_t arg_0x7f7fedab0c50, message_t * msg, error_t error){
#line 110
  /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__AMSend__sendDone(arg_0x7f7fedab0c50, msg, error);
#line 110
}
#line 110
# 85 "/home/csilab/upma/lib/serial/SerialActiveMessageP.nc"
static inline void /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__SubSend__sendDone(message_t *msg, error_t result)
#line 85
{
  /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMSend__sendDone(/*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMPacket__type(msg), msg, result);
}

# 376 "/home/csilab/tinyos/tos/lib/serial/SerialDispatcherP.nc"
static inline void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__Send__default__sendDone(uart_id_t idxxx, message_t *msg, error_t error)
#line 376
{
  return;
}

# 100 "/home/csilab/tinyos/tos/interfaces/Send.nc"
inline static void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__Send__sendDone(uart_id_t arg_0x7f7fed95c9c0, message_t * msg, error_t error){
#line 100
  switch (arg_0x7f7fed95c9c0) {
#line 100
    case TOS_SERIAL_ACTIVE_MESSAGE_ID:
#line 100
      /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__SubSend__sendDone(msg, error);
#line 100
      break;
#line 100
    default:
#line 100
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__Send__default__sendDone(arg_0x7f7fed95c9c0, msg, error);
#line 100
      break;
#line 100
    }
#line 100
}
#line 100
# 158 "/home/csilab/tinyos/tos/lib/serial/SerialDispatcherP.nc"
static inline void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__signalSendDone__runTask(void )
#line 158
{
  error_t error;

  /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendState = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__SEND_STATE_IDLE;
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 162
    error = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendError;
#line 162
    __nesc_atomic_end(__nesc_atomic); }

  if (/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendCancelled) {
#line 164
    error = ECANCEL;
    }
#line 165
  /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__Send__sendDone(/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendId, (message_t *)/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendBuffer, error);
}

#line 212
static inline void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__unlockBuffer(uint8_t which)
#line 212
{
  if (which) {
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveState.bufOneLocked = 0;
    }
  else {
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveState.bufZeroLocked = 0;
    }
}

# 93 "/home/csilab/upma/lib/serial/SerialActiveMessageP.nc"
static inline message_t */*SerialActiveMessageC.AM*/SerialActiveMessageP__0__Receive__default__receive(uint8_t id, message_t *msg, void *payload, uint8_t len)
#line 93
{
  return msg;
}

# 78 "/home/csilab/tinyos/tos/interfaces/Receive.nc"
inline static message_t * /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__Receive__receive(am_id_t arg_0x7f7fedaa2e30, message_t * msg, void * payload, uint8_t len){
#line 78
  nx_struct message_t *__nesc_result;
#line 78

#line 78
    __nesc_result = /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__Receive__default__receive(arg_0x7f7fedaa2e30, msg, payload, len);
#line 78

#line 78
  return __nesc_result;
#line 78
}
#line 78
# 97 "/home/csilab/upma/lib/serial/SerialActiveMessageP.nc"
static inline message_t */*SerialActiveMessageC.AM*/SerialActiveMessageP__0__SubReceive__receive(message_t *msg, void *payload, uint8_t len)
#line 97
{
  return /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__Receive__receive(/*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMPacket__type(msg), msg, msg->data, len);
}

# 371 "/home/csilab/tinyos/tos/lib/serial/SerialDispatcherP.nc"
static inline message_t */*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__Receive__default__receive(uart_id_t idxxx, message_t *msg, 
void *payload, 
uint8_t len)
#line 373
{
  return msg;
}

# 78 "/home/csilab/tinyos/tos/interfaces/Receive.nc"
inline static message_t * /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__Receive__receive(uart_id_t arg_0x7f7fed95fda0, message_t * msg, void * payload, uint8_t len){
#line 78
  nx_struct message_t *__nesc_result;
#line 78

#line 78
  switch (arg_0x7f7fed95fda0) {
#line 78
    case TOS_SERIAL_ACTIVE_MESSAGE_ID:
#line 78
      __nesc_result = /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__SubReceive__receive(msg, payload, len);
#line 78
      break;
#line 78
    default:
#line 78
      __nesc_result = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__Receive__default__receive(arg_0x7f7fed95fda0, msg, payload, len);
#line 78
      break;
#line 78
    }
#line 78

#line 78
  return __nesc_result;
#line 78
}
#line 78
# 57 "/home/csilab/tinyos/tos/lib/serial/SerialPacketInfoActiveMessageP.nc"
static inline uint8_t SerialPacketInfoActiveMessageP__Info__upperLength(message_t *msg, uint8_t dataLinkLen)
#line 57
{
  return dataLinkLen - sizeof(serial_header_t );
}

# 365 "/home/csilab/tinyos/tos/lib/serial/SerialDispatcherP.nc"
static inline uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__default__upperLength(uart_id_t id, message_t *msg, 
uint8_t dataLinkLen)
#line 366
{
  return 0;
}

# 31 "/home/csilab/tinyos/tos/lib/serial/SerialPacketInfo.nc"
inline static uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__upperLength(uart_id_t arg_0x7f7fed95be30, message_t *msg, uint8_t dataLinkLen){
#line 31
  unsigned char __nesc_result;
#line 31

#line 31
  switch (arg_0x7f7fed95be30) {
#line 31
    case TOS_SERIAL_ACTIVE_MESSAGE_ID:
#line 31
      __nesc_result = SerialPacketInfoActiveMessageP__Info__upperLength(msg, dataLinkLen);
#line 31
      break;
#line 31
    default:
#line 31
      __nesc_result = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__default__upperLength(arg_0x7f7fed95be30, msg, dataLinkLen);
#line 31
      break;
#line 31
    }
#line 31

#line 31
  return __nesc_result;
#line 31
}
#line 31
# 275 "/home/csilab/tinyos/tos/lib/serial/SerialDispatcherP.nc"
static inline void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTask__runTask(void )
#line 275
{
  uart_id_t myType;
  message_t *myBuf;
  uint8_t mySize;
  uint8_t myWhich;

#line 280
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 280
    {
      myType = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTaskType;
      myBuf = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTaskBuf;
      mySize = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTaskSize;
      myWhich = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTaskWhich;
    }
#line 285
    __nesc_atomic_end(__nesc_atomic); }
  mySize -= /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__offset(myType);
  mySize = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__upperLength(myType, myBuf, mySize);
  myBuf = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__Receive__receive(myType, myBuf, myBuf, mySize);
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 289
    {
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__messagePtrs[myWhich] = myBuf;
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__unlockBuffer(myWhich);
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTaskPending = FALSE;
    }
#line 293
    __nesc_atomic_end(__nesc_atomic); }
}

# 749 "/home/csilab/tinyos/tos/lib/serial/SerialP.nc"
static inline void SerialP__SplitControl__default__stopDone(error_t err)
#line 749
{
}

# 138 "/home/csilab/tinyos/tos/interfaces/SplitControl.nc"
inline static void SerialP__SplitControl__stopDone(error_t error){
#line 138
  SerialP__SplitControl__default__stopDone(error);
#line 138
}
#line 138
# 109 "/home/csilab/tinyos/tos/chips/msp430/usart/HplMsp430Usart1P.nc"
static inline error_t HplMsp430Usart1P__AsyncStdControl__stop(void )
#line 109
{
  HplMsp430Usart1P__Usart__disableSpi();
  HplMsp430Usart1P__Usart__disableUart();
  return SUCCESS;
}

# 105 "/home/csilab/tinyos/tos/interfaces/AsyncStdControl.nc"
inline static error_t /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__AsyncStdControl__stop(void ){
#line 105
  unsigned char __nesc_result;
#line 105

#line 105
  __nesc_result = HplMsp430Usart1P__AsyncStdControl__stop();
#line 105

#line 105
  return __nesc_result;
#line 105
}
#line 105
# 84 "/home/csilab/tinyos/tos/lib/power/AsyncPowerManagerP.nc"
static inline void /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__PowerDownCleanup__default__cleanup(void )
#line 84
{
}

# 62 "/home/csilab/tinyos/tos/lib/power/PowerDownCleanup.nc"
inline static void /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__PowerDownCleanup__cleanup(void ){
#line 62
  /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__PowerDownCleanup__default__cleanup();
#line 62
}
#line 62
# 79 "/home/csilab/tinyos/tos/lib/power/AsyncPowerManagerP.nc"
static inline void /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__ResourceDefaultOwner__granted(void )
#line 79
{
  /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__PowerDownCleanup__cleanup();
  /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__AsyncStdControl__stop();
}

# 46 "/home/csilab/tinyos/tos/interfaces/ResourceDefaultOwner.nc"
inline static void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__granted(void ){
#line 46
  /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__ResourceDefaultOwner__granted();
#line 46
}
#line 46
# 128 "/home/csilab/tinyos/tos/chips/msp430/usart/HplMsp430Usart.nc"
inline static void /*Msp430Uart1P.UartP*/Msp430UartP__0__Usart__disableUart(void ){
#line 128
  HplMsp430Usart1P__Usart__disableUart();
#line 128
}
#line 128
#line 179
inline static void /*Msp430Uart1P.UartP*/Msp430UartP__0__Usart__disableIntr(void ){
#line 179
  HplMsp430Usart1P__Usart__disableIntr();
#line 179
}
#line 179
#line 97
inline static void /*Msp430Uart1P.UartP*/Msp430UartP__0__Usart__resetUsart(bool reset){
#line 97
  HplMsp430Usart1P__Usart__resetUsart(reset);
#line 97
}
#line 97
# 92 "/home/csilab/tinyos/tos/chips/msp430/usart/Msp430UartP.nc"
static inline void /*Msp430Uart1P.UartP*/Msp430UartP__0__ResourceConfigure__unconfigure(uint8_t id)
#line 92
{
  /*Msp430Uart1P.UartP*/Msp430UartP__0__Usart__resetUsart(TRUE);
  /*Msp430Uart1P.UartP*/Msp430UartP__0__Usart__disableIntr();
  /*Msp430Uart1P.UartP*/Msp430UartP__0__Usart__disableUart();
}

# 218 "/home/csilab/tinyos/tos/system/ArbiterP.nc"
static inline void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__default__unconfigure(uint8_t id)
#line 218
{
}

# 65 "/home/csilab/tinyos/tos/interfaces/ResourceConfigure.nc"
inline static void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__unconfigure(uint8_t arg_0x7f7fed25bc40){
#line 65
  switch (arg_0x7f7fed25bc40) {
#line 65
    case /*PlatformSerialC.UartC.UsartC*/Msp430Usart1C__0__CLIENT_ID:
#line 65
      /*Msp430Uart1P.UartP*/Msp430UartP__0__ResourceConfigure__unconfigure(/*PlatformSerialC.UartC*/Msp430Uart1C__0__CLIENT_ID);
#line 65
      break;
#line 65
    default:
#line 65
      /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__default__unconfigure(arg_0x7f7fed25bc40);
#line 65
      break;
#line 65
    }
#line 65
}
#line 65
# 67 "/home/csilab/tinyos/tos/interfaces/TaskBasic.nc"
inline static error_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__grantedTask__postTask(void ){
#line 67
  unsigned char __nesc_result;
#line 67

#line 67
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(/*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__grantedTask);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 68 "/home/csilab/tinyos/tos/system/FcfsResourceQueueC.nc"
static inline resource_client_id_t /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__1__FcfsQueue__dequeue(void )
#line 68
{
  /* atomic removed: atomic calls only */
#line 69
  {
    if (/*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__1__qHead != /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__1__NO_ENTRY) {
        uint8_t id = /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__1__qHead;

#line 72
        /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__1__qHead = /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__1__resQ[/*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__1__qHead];
        if (/*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__1__qHead == /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__1__NO_ENTRY) {
          /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__1__qTail = /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__1__NO_ENTRY;
          }
#line 75
        /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__1__resQ[id] = /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__1__NO_ENTRY;
        {
          unsigned char __nesc_temp = 
#line 76
          id;

#line 76
          return __nesc_temp;
        }
      }
#line 78
    {
      unsigned char __nesc_temp = 
#line 78
      /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__1__NO_ENTRY;

#line 78
      return __nesc_temp;
    }
  }
}

# 70 "/home/csilab/tinyos/tos/interfaces/ResourceQueue.nc"
inline static resource_client_id_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__Queue__dequeue(void ){
#line 70
  unsigned char __nesc_result;
#line 70

#line 70
  __nesc_result = /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__1__FcfsQueue__dequeue();
#line 70

#line 70
  return __nesc_result;
#line 70
}
#line 70
# 60 "/home/csilab/tinyos/tos/system/FcfsResourceQueueC.nc"
static inline bool /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__1__FcfsQueue__isEmpty(void )
#line 60
{
  /* atomic removed: atomic calls only */
#line 61
  {
    unsigned char __nesc_temp = 
#line 61
    /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__1__qHead == /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__1__NO_ENTRY;

#line 61
    return __nesc_temp;
  }
}

# 53 "/home/csilab/tinyos/tos/interfaces/ResourceQueue.nc"
inline static bool /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__Queue__isEmpty(void ){
#line 53
  unsigned char __nesc_result;
#line 53

#line 53
  __nesc_result = /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__1__FcfsQueue__isEmpty();
#line 53

#line 53
  return __nesc_result;
#line 53
}
#line 53
# 111 "/home/csilab/tinyos/tos/system/ArbiterP.nc"
static inline error_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__Resource__release(uint8_t id)
#line 111
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 112
    {
      if (/*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__state == /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__RES_BUSY && /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__resId == id) {
          if (/*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__Queue__isEmpty() == FALSE) {
              /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__reqResId = /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__Queue__dequeue();
              /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__resId = /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__NO_RES;
              /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__state = /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__RES_GRANTING;
              /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__grantedTask__postTask();
              /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__unconfigure(id);
            }
          else {
              /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__resId = /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__default_owner_id;
              /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__state = /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__RES_CONTROLLED;
              /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__unconfigure(id);
              /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__granted();
            }
          {
            unsigned char __nesc_temp = 
#line 127
            SUCCESS;

            {
#line 127
              __nesc_atomic_end(__nesc_atomic); 
#line 127
              return __nesc_temp;
            }
          }
        }
    }
#line 131
    __nesc_atomic_end(__nesc_atomic); }
#line 130
  return FAIL;
}

# 213 "/home/csilab/tinyos/tos/chips/msp430/usart/Msp430UartP.nc"
static inline error_t /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartResource__default__release(uint8_t id)
#line 213
{
#line 213
  return FAIL;
}

# 120 "/home/csilab/tinyos/tos/interfaces/Resource.nc"
inline static error_t /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartResource__release(uint8_t arg_0x7f7fed84a980){
#line 120
  unsigned char __nesc_result;
#line 120

#line 120
  switch (arg_0x7f7fed84a980) {
#line 120
    case /*PlatformSerialC.UartC*/Msp430Uart1C__0__CLIENT_ID:
#line 120
      __nesc_result = /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__Resource__release(/*PlatformSerialC.UartC.UsartC*/Msp430Usart1C__0__CLIENT_ID);
#line 120
      break;
#line 120
    default:
#line 120
      __nesc_result = /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartResource__default__release(arg_0x7f7fed84a980);
#line 120
      break;
#line 120
    }
#line 120

#line 120
  return __nesc_result;
#line 120
}
#line 120
# 210 "/home/csilab/tinyos/tos/chips/msp430/usart/Msp430UartP.nc"
static inline bool /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartResource__default__isOwner(uint8_t id)
#line 210
{
#line 210
  return FALSE;
}

# 128 "/home/csilab/tinyos/tos/interfaces/Resource.nc"
inline static bool /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartResource__isOwner(uint8_t arg_0x7f7fed84a980){
#line 128
  unsigned char __nesc_result;
#line 128

#line 128
  switch (arg_0x7f7fed84a980) {
#line 128
    case /*PlatformSerialC.UartC*/Msp430Uart1C__0__CLIENT_ID:
#line 128
      __nesc_result = /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__Resource__isOwner(/*PlatformSerialC.UartC.UsartC*/Msp430Usart1C__0__CLIENT_ID);
#line 128
      break;
#line 128
    default:
#line 128
      __nesc_result = /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartResource__default__isOwner(arg_0x7f7fed84a980);
#line 128
      break;
#line 128
    }
#line 128

#line 128
  return __nesc_result;
#line 128
}
#line 128
# 77 "/home/csilab/tinyos/tos/chips/msp430/usart/Msp430UartP.nc"
static inline error_t /*Msp430Uart1P.UartP*/Msp430UartP__0__Resource__release(uint8_t id)
#line 77
{
  if (/*Msp430Uart1P.UartP*/Msp430UartP__0__UsartResource__isOwner(id) == FALSE) {
    return FAIL;
    }
#line 80
  if (/*Msp430Uart1P.UartP*/Msp430UartP__0__m_rx_buf || /*Msp430Uart1P.UartP*/Msp430UartP__0__m_tx_buf) {
    return EBUSY;
    }
#line 82
  return /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartResource__release(id);
}

# 120 "/home/csilab/tinyos/tos/interfaces/Resource.nc"
inline static error_t TelosSerialP__Resource__release(void ){
#line 120
  unsigned char __nesc_result;
#line 120

#line 120
  __nesc_result = /*Msp430Uart1P.UartP*/Msp430UartP__0__Resource__release(/*PlatformSerialC.UartC*/Msp430Uart1C__0__CLIENT_ID);
#line 120

#line 120
  return __nesc_result;
#line 120
}
#line 120
# 13 "/home/csilab/tinyos/tos/platforms/telosa/TelosSerialP.nc"
static inline error_t TelosSerialP__StdControl__stop(void )
#line 13
{
  TelosSerialP__Resource__release();
  return SUCCESS;
}

# 105 "/home/csilab/tinyos/tos/interfaces/StdControl.nc"
inline static error_t SerialP__SerialControl__stop(void ){
#line 105
  unsigned char __nesc_result;
#line 105

#line 105
  __nesc_result = TelosSerialP__StdControl__stop();
#line 105

#line 105
  return __nesc_result;
#line 105
}
#line 105
# 336 "/home/csilab/tinyos/tos/lib/serial/SerialP.nc"
static inline void SerialP__SerialFlush__flushDone(void )
#line 336
{
  SerialP__SerialControl__stop();
  SerialP__SplitControl__stopDone(SUCCESS);
}

static inline void SerialP__defaultSerialFlushTask__runTask(void )
#line 341
{
  SerialP__SerialFlush__flushDone();
}

# 67 "/home/csilab/tinyos/tos/interfaces/TaskBasic.nc"
inline static error_t SerialP__defaultSerialFlushTask__postTask(void ){
#line 67
  unsigned char __nesc_result;
#line 67

#line 67
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(SerialP__defaultSerialFlushTask);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 344 "/home/csilab/tinyos/tos/lib/serial/SerialP.nc"
static inline void SerialP__SerialFlush__default__flush(void )
#line 344
{
  SerialP__defaultSerialFlushTask__postTask();
}

# 49 "/home/csilab/tinyos/tos/lib/serial/SerialFlush.nc"
inline static void SerialP__SerialFlush__flush(void ){
#line 49
  SerialP__SerialFlush__default__flush();
#line 49
}
#line 49
# 332 "/home/csilab/tinyos/tos/lib/serial/SerialP.nc"
static inline void SerialP__stopDoneTask__runTask(void )
#line 332
{
  SerialP__SerialFlush__flush();
}

#line 748
static inline void SerialP__SplitControl__default__startDone(error_t err)
#line 748
{
}

# 113 "/home/csilab/tinyos/tos/interfaces/SplitControl.nc"
inline static void SerialP__SplitControl__startDone(error_t error){
#line 113
  SerialP__SplitControl__default__startDone(error);
#line 113
}
#line 113
# 212 "/home/csilab/tinyos/tos/chips/msp430/usart/Msp430UartP.nc"
static inline error_t /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartResource__default__immediateRequest(uint8_t id)
#line 212
{
#line 212
  return FAIL;
}

# 97 "/home/csilab/tinyos/tos/interfaces/Resource.nc"
inline static error_t /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartResource__immediateRequest(uint8_t arg_0x7f7fed84a980){
#line 97
  unsigned char __nesc_result;
#line 97

#line 97
  switch (arg_0x7f7fed84a980) {
#line 97
    case /*PlatformSerialC.UartC*/Msp430Uart1C__0__CLIENT_ID:
#line 97
      __nesc_result = /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__Resource__immediateRequest(/*PlatformSerialC.UartC.UsartC*/Msp430Usart1C__0__CLIENT_ID);
#line 97
      break;
#line 97
    default:
#line 97
      __nesc_result = /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartResource__default__immediateRequest(arg_0x7f7fed84a980);
#line 97
      break;
#line 97
    }
#line 97

#line 97
  return __nesc_result;
#line 97
}
#line 97
# 65 "/home/csilab/tinyos/tos/chips/msp430/usart/Msp430UartP.nc"
static inline error_t /*Msp430Uart1P.UartP*/Msp430UartP__0__Resource__immediateRequest(uint8_t id)
#line 65
{
  return /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartResource__immediateRequest(id);
}

# 97 "/home/csilab/tinyos/tos/interfaces/Resource.nc"
inline static error_t TelosSerialP__Resource__immediateRequest(void ){
#line 97
  unsigned char __nesc_result;
#line 97

#line 97
  __nesc_result = /*Msp430Uart1P.UartP*/Msp430UartP__0__Resource__immediateRequest(/*PlatformSerialC.UartC*/Msp430Uart1C__0__CLIENT_ID);
#line 97

#line 97
  return __nesc_result;
#line 97
}
#line 97
# 10 "/home/csilab/tinyos/tos/platforms/telosa/TelosSerialP.nc"
static inline error_t TelosSerialP__StdControl__start(void )
#line 10
{
  return TelosSerialP__Resource__immediateRequest();
}

# 95 "/home/csilab/tinyos/tos/interfaces/StdControl.nc"
inline static error_t SerialP__SerialControl__start(void ){
#line 95
  unsigned char __nesc_result;
#line 95

#line 95
  __nesc_result = TelosSerialP__StdControl__start();
#line 95

#line 95
  return __nesc_result;
#line 95
}
#line 95
# 322 "/home/csilab/tinyos/tos/lib/serial/SerialP.nc"
static inline void SerialP__startDoneTask__runTask(void )
#line 322
{
  SerialP__SerialControl__start();
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 324
    {
      SerialP__txState = SerialP__TXSTATE_IDLE;
      SerialP__rxState = SerialP__RXSTATE_NOSYNC;
    }
#line 327
    __nesc_atomic_end(__nesc_atomic); }
  SerialP__SplitControl__startDone(SUCCESS);
}

# 206 "/home/csilab/tinyos/tos/system/ArbiterP.nc"
static inline void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceRequested__default__immediateRequested(uint8_t id)
#line 206
{
}

# 61 "/home/csilab/tinyos/tos/interfaces/ResourceRequested.nc"
inline static void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceRequested__immediateRequested(uint8_t arg_0x7f7fed25e840){
#line 61
    /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceRequested__default__immediateRequested(arg_0x7f7fed25e840);
#line 61
}
#line 61
# 133 "/home/csilab/tinyos/tos/system/ArbiterP.nc"
static inline error_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__release(void )
#line 133
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 134
    {
      if (/*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__resId == /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__default_owner_id) {
          if (/*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__state == /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__RES_GRANTING) {
              /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__grantedTask__postTask();
              {
                unsigned char __nesc_temp = 
#line 138
                SUCCESS;

                {
#line 138
                  __nesc_atomic_end(__nesc_atomic); 
#line 138
                  return __nesc_temp;
                }
              }
            }
          else {
#line 140
            if (/*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__state == /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__RES_IMM_GRANTING) {
                /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__resId = /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__reqResId;
                /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__state = /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__RES_BUSY;
                {
                  unsigned char __nesc_temp = 
#line 143
                  SUCCESS;

                  {
#line 143
                    __nesc_atomic_end(__nesc_atomic); 
#line 143
                    return __nesc_temp;
                  }
                }
              }
            }
        }
    }
#line 149
    __nesc_atomic_end(__nesc_atomic); }
#line 147
  return FAIL;
}

# 56 "/home/csilab/tinyos/tos/interfaces/ResourceDefaultOwner.nc"
inline static error_t /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__ResourceDefaultOwner__release(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__release();
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 105 "/home/csilab/tinyos/tos/chips/msp430/usart/HplMsp430Usart1P.nc"
static inline error_t HplMsp430Usart1P__AsyncStdControl__start(void )
#line 105
{
  return SUCCESS;
}

# 95 "/home/csilab/tinyos/tos/interfaces/AsyncStdControl.nc"
inline static error_t /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__AsyncStdControl__start(void ){
#line 95
  unsigned char __nesc_result;
#line 95

#line 95
  __nesc_result = HplMsp430Usart1P__AsyncStdControl__start();
#line 95

#line 95
  return __nesc_result;
#line 95
}
#line 95
# 74 "/home/csilab/tinyos/tos/lib/power/AsyncPowerManagerP.nc"
static inline void /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__ResourceDefaultOwner__immediateRequested(void )
#line 74
{
  /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__AsyncStdControl__start();
  /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__ResourceDefaultOwner__release();
}

# 81 "/home/csilab/tinyos/tos/interfaces/ResourceDefaultOwner.nc"
inline static void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__immediateRequested(void ){
#line 81
  /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__ResourceDefaultOwner__immediateRequested();
#line 81
}
#line 81
# 67 "/home/csilab/tinyos/tos/interfaces/TaskBasic.nc"
inline static error_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__signalSendDone__postTask(void ){
#line 67
  unsigned char __nesc_result;
#line 67

#line 67
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__signalSendDone);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 194 "/home/csilab/tinyos/tos/lib/serial/SerialDispatcherP.nc"
static inline void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__SendBytePacket__sendCompleted(error_t error)
#line 194
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 195
    /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendError = error;
#line 195
    __nesc_atomic_end(__nesc_atomic); }
  /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__signalSendDone__postTask();
}

# 91 "/home/csilab/tinyos/tos/lib/serial/SendBytePacket.nc"
inline static void SerialP__SendBytePacket__sendCompleted(error_t error){
#line 91
  /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__SendBytePacket__sendCompleted(error);
#line 91
}
#line 91
# 244 "/home/csilab/tinyos/tos/lib/serial/SerialP.nc"
static __inline bool SerialP__ack_queue_is_empty(void )
#line 244
{
  bool ret;

#line 246
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 246
    ret = SerialP__ackQ.writePtr == SerialP__ackQ.readPtr;
#line 246
    __nesc_atomic_end(__nesc_atomic); }
  return ret;
}











static __inline uint8_t SerialP__ack_queue_top(void )
#line 260
{
  uint8_t tmp = 0;

  /* atomic removed: atomic calls only */
#line 262
  {
    if (!SerialP__ack_queue_is_empty()) {
        tmp = SerialP__ackQ.buf[SerialP__ackQ.readPtr];
      }
  }
  return tmp;
}

static inline uint8_t SerialP__ack_queue_pop(void )
#line 270
{
  uint8_t retval = 0;

  /* atomic removed: atomic calls only */
#line 272
  {
    if (SerialP__ackQ.writePtr != SerialP__ackQ.readPtr) {
        retval = SerialP__ackQ.buf[SerialP__ackQ.readPtr];
        if (++ SerialP__ackQ.readPtr > SerialP__ACK_QUEUE_SIZE) {
#line 275
          SerialP__ackQ.readPtr = 0;
          }
      }
  }
#line 278
  return retval;
}

#line 559
static inline void SerialP__RunTx__runTask(void )
#line 559
{
  uint8_t idle;
  uint8_t done;
  uint8_t fail;









  error_t result = SUCCESS;
  bool send_completed = FALSE;
  bool start_it = FALSE;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 576
    {
      SerialP__txPending = 0;
      idle = SerialP__txState == SerialP__TXSTATE_IDLE;
      done = SerialP__txState == SerialP__TXSTATE_FINISH;
      fail = SerialP__txState == SerialP__TXSTATE_ERROR;
      if (done || fail) {
          SerialP__txState = SerialP__TXSTATE_IDLE;
          SerialP__txBuf[SerialP__txIndex].state = SerialP__BUFFER_AVAILABLE;
        }
    }
#line 585
    __nesc_atomic_end(__nesc_atomic); }


  if (done || fail) {
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 589
        {
          SerialP__txSeqno++;
          if (SerialP__txProto == SERIAL_PROTO_ACK) {
              SerialP__ack_queue_pop();
            }
          else {
              result = done ? SUCCESS : FAIL;
              send_completed = TRUE;
            }
        }
#line 598
        __nesc_atomic_end(__nesc_atomic); }
      idle = TRUE;
    }


  if (idle) {
      bool goInactive;

#line 605
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 605
        goInactive = SerialP__offPending;
#line 605
        __nesc_atomic_end(__nesc_atomic); }
      if (goInactive) {
          { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 607
            SerialP__txState = SerialP__TXSTATE_INACTIVE;
#line 607
            __nesc_atomic_end(__nesc_atomic); }
        }
      else {

          uint8_t myAckState;
          uint8_t myDataState;

#line 613
          { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 613
            {
              myAckState = SerialP__txBuf[SerialP__TX_ACK_INDEX].state;
              myDataState = SerialP__txBuf[SerialP__TX_DATA_INDEX].state;
            }
#line 616
            __nesc_atomic_end(__nesc_atomic); }
          if (!SerialP__ack_queue_is_empty() && myAckState == SerialP__BUFFER_AVAILABLE) {
              { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 618
                {
                  SerialP__txBuf[SerialP__TX_ACK_INDEX].state = SerialP__BUFFER_COMPLETE;
                  SerialP__txBuf[SerialP__TX_ACK_INDEX].buf = SerialP__ack_queue_top();

                  SerialP__txProto = SERIAL_PROTO_ACK;
                  SerialP__txIndex = SerialP__TX_ACK_INDEX;
                  start_it = TRUE;
                }
#line 625
                __nesc_atomic_end(__nesc_atomic); }
            }
          else {
#line 627
            if (myDataState == SerialP__BUFFER_FILLING || myDataState == SerialP__BUFFER_COMPLETE) {
                { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 628
                  {
                    SerialP__txProto = SERIAL_PROTO_PACKET_NOACK;
                    SerialP__txIndex = SerialP__TX_DATA_INDEX;
                    start_it = TRUE;
                  }
#line 632
                  __nesc_atomic_end(__nesc_atomic); }
              }
            else {
              }
            }
        }
    }
  else {
    }


  if (send_completed) {
      SerialP__SendBytePacket__sendCompleted(result);
    }
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 646
    {
      if (SerialP__txState == SerialP__TXSTATE_INACTIVE) {
          SerialP__testOff();
          {
#line 649
            __nesc_atomic_end(__nesc_atomic); 
#line 649
            return;
          }
        }
    }
#line 652
    __nesc_atomic_end(__nesc_atomic); }
  if (start_it) {

      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 655
        {
          SerialP__txCRC = 0;
          SerialP__txByteCnt = 0;
          SerialP__txState = SerialP__TXSTATE_PROTO;
        }
#line 659
        __nesc_atomic_end(__nesc_atomic); }
      if (SerialP__SerialFrameComm__putDelimiter() != SUCCESS) {
          { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 661
            SerialP__txState = SerialP__TXSTATE_ERROR;
#line 661
            __nesc_atomic_end(__nesc_atomic); }
          SerialP__MaybeScheduleTx();
        }
    }
}

#line 216
static __inline void SerialP__ackInit(void )
#line 216
{
  SerialP__ackQ.writePtr = SerialP__ackQ.readPtr = 0;
}

#line 207
static __inline void SerialP__rxInit(void )
#line 207
{
  SerialP__rxBuf.writePtr = SerialP__rxBuf.readPtr = 0;
  SerialP__rxState = SerialP__RXSTATE_INACTIVE;
  SerialP__rxByteCnt = 0;
  SerialP__rxProto = 0;
  SerialP__rxSeqno = 0;
  SerialP__rxCRC = 0;
}

#line 195
static __inline void SerialP__txInit(void )
#line 195
{
  uint8_t i;

  /* atomic removed: atomic calls only */
#line 197
  for (i = 0; i < SerialP__TX_BUFFER_COUNT; i++) SerialP__txBuf[i].state = SerialP__BUFFER_AVAILABLE;
  SerialP__txState = SerialP__TXSTATE_INACTIVE;
  SerialP__txByteCnt = 0;
  SerialP__txProto = 0;
  SerialP__txSeqno = 0;
  SerialP__txCRC = 0;
  SerialP__txPending = FALSE;
  SerialP__txIndex = 0;
}

#line 220
static inline error_t SerialP__Init__init(void )
#line 220
{

  SerialP__txInit();
  SerialP__rxInit();
  SerialP__ackInit();

  return SUCCESS;
}

# 55 "/home/csilab/tinyos/tos/system/FcfsResourceQueueC.nc"
static inline error_t /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__1__Init__init(void )
#line 55
{
  memset(/*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__1__resQ, /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__1__NO_ENTRY, sizeof /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__1__resQ);
  return SUCCESS;
}

# 109 "/home/csilab/tinyos/tos/lib/printf/PrintfP.nc"
static inline error_t PrintfP__Init__init(void )
#line 109
{
  /* atomic removed: atomic calls only */
#line 110
  PrintfP__state = PrintfP__S_STARTED;
  return SUCCESS;
}

# 98 "/home/csilab/tinyos/tos/lib/printf/PutcharP.nc"
static inline error_t PutcharP__Init__init(void )
#line 98
{
  error_t rv = SUCCESS;



  return rv;
}

# 48 "/home/csilab/tinyos/tos/interfaces/LocalIeeeEui64.nc"
inline static ieee_eui64_t CC2420ControlP__LocalIeeeEui64__getId(void ){
#line 48
  struct ieee_eui64 __nesc_result;
#line 48

#line 48
  __nesc_result = DallasId48ToIeeeEui64C__LocalIeeeEui64__getId();
#line 48

#line 48
  return __nesc_result;
#line 48
}
#line 48
# 93 "/home/csilab/tinyos/tos/system/ActiveMessageAddressC.nc"
static inline am_group_t ActiveMessageAddressC__ActiveMessageAddress__amGroup(void )
#line 93
{
  am_group_t myGroup;

  /* atomic removed: atomic calls only */
#line 95
  myGroup = ActiveMessageAddressC__group;
  return myGroup;
}

# 55 "/home/csilab/tinyos/tos/interfaces/ActiveMessageAddress.nc"
inline static am_group_t CC2420ControlP__ActiveMessageAddress__amGroup(void ){
#line 55
  unsigned char __nesc_result;
#line 55

#line 55
  __nesc_result = ActiveMessageAddressC__ActiveMessageAddress__amGroup();
#line 55

#line 55
  return __nesc_result;
#line 55
}
#line 55
#line 50
inline static am_addr_t CC2420ControlP__ActiveMessageAddress__amAddress(void ){
#line 50
  unsigned int __nesc_result;
#line 50

#line 50
  __nesc_result = ActiveMessageAddressC__ActiveMessageAddress__amAddress();
#line 50

#line 50
  return __nesc_result;
#line 50
}
#line 50
# 63 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P45*/HplMsp430GeneralIOP__29__IO__makeOutput(void )
#line 63
{
  /* atomic removed: atomic calls only */
#line 63
  * (volatile uint8_t * )30U |= 0x01 << 5;
}

# 85 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*HplCC2420PinsC.VRENM*/Msp430GpioC__9__HplGeneralIO__makeOutput(void ){
#line 85
  /*HplMsp430GeneralIOC.P45*/HplMsp430GeneralIOP__29__IO__makeOutput();
#line 85
}
#line 85
# 54 "/home/csilab/tinyos/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplCC2420PinsC.VRENM*/Msp430GpioC__9__GeneralIO__makeOutput(void )
#line 54
{
#line 54
  /*HplCC2420PinsC.VRENM*/Msp430GpioC__9__HplGeneralIO__makeOutput();
}

# 46 "/home/csilab/tinyos/tos/interfaces/GeneralIO.nc"
inline static void CC2420ControlP__VREN__makeOutput(void ){
#line 46
  /*HplCC2420PinsC.VRENM*/Msp430GpioC__9__GeneralIO__makeOutput();
#line 46
}
#line 46
# 63 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P46*/HplMsp430GeneralIOP__30__IO__makeOutput(void )
#line 63
{
  /* atomic removed: atomic calls only */
#line 63
  * (volatile uint8_t * )30U |= 0x01 << 6;
}

# 85 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*HplCC2420PinsC.RSTNM*/Msp430GpioC__7__HplGeneralIO__makeOutput(void ){
#line 85
  /*HplMsp430GeneralIOC.P46*/HplMsp430GeneralIOP__30__IO__makeOutput();
#line 85
}
#line 85
# 54 "/home/csilab/tinyos/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplCC2420PinsC.RSTNM*/Msp430GpioC__7__GeneralIO__makeOutput(void )
#line 54
{
#line 54
  /*HplCC2420PinsC.RSTNM*/Msp430GpioC__7__HplGeneralIO__makeOutput();
}

# 46 "/home/csilab/tinyos/tos/interfaces/GeneralIO.nc"
inline static void CC2420ControlP__RSTN__makeOutput(void ){
#line 46
  /*HplCC2420PinsC.RSTNM*/Msp430GpioC__7__GeneralIO__makeOutput();
#line 46
}
#line 46
# 63 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P42*/HplMsp430GeneralIOP__26__IO__makeOutput(void )
#line 63
{
  /* atomic removed: atomic calls only */
#line 63
  * (volatile uint8_t * )30U |= 0x01 << 2;
}

# 85 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*HplCC2420PinsC.CSNM*/Msp430GpioC__4__HplGeneralIO__makeOutput(void ){
#line 85
  /*HplMsp430GeneralIOC.P42*/HplMsp430GeneralIOP__26__IO__makeOutput();
#line 85
}
#line 85
# 54 "/home/csilab/tinyos/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplCC2420PinsC.CSNM*/Msp430GpioC__4__GeneralIO__makeOutput(void )
#line 54
{
#line 54
  /*HplCC2420PinsC.CSNM*/Msp430GpioC__4__HplGeneralIO__makeOutput();
}

# 46 "/home/csilab/tinyos/tos/interfaces/GeneralIO.nc"
inline static void CC2420ControlP__CSN__makeOutput(void ){
#line 46
  /*HplCC2420PinsC.CSNM*/Msp430GpioC__4__GeneralIO__makeOutput();
#line 46
}
#line 46
# 129 "/home/csilab/tinyos/tos/chips/cc2420/control/CC2420ControlP.nc"
static inline error_t CC2420ControlP__Init__init(void )
#line 129
{
  int i;
#line 130
  int t;

#line 131
  CC2420ControlP__CSN__makeOutput();
  CC2420ControlP__RSTN__makeOutput();
  CC2420ControlP__VREN__makeOutput();

  CC2420ControlP__m_short_addr = CC2420ControlP__ActiveMessageAddress__amAddress();
  CC2420ControlP__m_ext_addr = CC2420ControlP__LocalIeeeEui64__getId();
  CC2420ControlP__m_pan = CC2420ControlP__ActiveMessageAddress__amGroup();
  CC2420ControlP__m_tx_power = 31;
  CC2420ControlP__m_channel = 26;

  CC2420ControlP__m_ext_addr = CC2420ControlP__LocalIeeeEui64__getId();
  for (i = 0; i < 4; i++) {
      t = CC2420ControlP__m_ext_addr.data[i];
      CC2420ControlP__m_ext_addr.data[i] = CC2420ControlP__m_ext_addr.data[7 - i];
      CC2420ControlP__m_ext_addr.data[7 - i] = t;
    }





  CC2420ControlP__addressRecognition = TRUE;





  CC2420ControlP__hwAddressRecognition = FALSE;






  CC2420ControlP__autoAckEnabled = TRUE;






  CC2420ControlP__hwAutoAckDefault = FALSE;



  return SUCCESS;
}

# 81 "/home/csilab/tinyos/tos/system/StateImplP.nc"
static inline error_t StateImplP__Init__init(void )
#line 81
{
  int i;

#line 83
  for (i = 0; i < 3U; i++) {
      StateImplP__state[i] = StateImplP__S_IDLE;
    }
  return SUCCESS;
}

# 55 "/home/csilab/tinyos/tos/system/FcfsResourceQueueC.nc"
static inline error_t /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__2__Init__init(void )
#line 55
{
  memset(/*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__2__resQ, /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__2__NO_ENTRY, sizeof /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__2__resQ);
  return SUCCESS;
}

# 57 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__CC2int(/*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t x)
#line 57
{
#line 57
  union /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3____nesc_unnamed4412 {
#line 57
    /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t f;
#line 57
    uint16_t t;
  } 
#line 57
  c = { .f = x };

#line 57
  return c.t;
}

static inline uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__compareControl(void )
{
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t x = { 
  .cm = 1, 
  .ccis = 0, 
  .clld = 0, 
  .cap = 0, 
  .ccie = 0 };

  return /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__CC2int(x);
}

#line 105
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__setControlAsCompare(void )
{
  * (volatile uint16_t * )386U = /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__compareControl();
}

# 47 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__setControlAsCompare(void ){
#line 47
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__setControlAsCompare();
#line 47
}
#line 47
# 53 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline error_t /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Init__init(void )
{
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__disableEvents();
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__setControlAsCompare();
  return SUCCESS;
}

# 61 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__makeInput(void )
#line 61
{
  /* atomic removed: atomic calls only */
#line 61
  * (volatile uint8_t * )30U &= ~(0x01 << 1);
}

# 78 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*HplCC2420PinsC.SFDM*/Msp430GpioC__8__HplGeneralIO__makeInput(void ){
#line 78
  /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__makeInput();
#line 78
}
#line 78
# 52 "/home/csilab/tinyos/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplCC2420PinsC.SFDM*/Msp430GpioC__8__GeneralIO__makeInput(void )
#line 52
{
#line 52
  /*HplCC2420PinsC.SFDM*/Msp430GpioC__8__HplGeneralIO__makeInput();
}

# 44 "/home/csilab/tinyos/tos/interfaces/GeneralIO.nc"
inline static void CC2420TransmitP__SFD__makeInput(void ){
#line 44
  /*HplCC2420PinsC.SFDM*/Msp430GpioC__8__GeneralIO__makeInput();
#line 44
}
#line 44


inline static void CC2420TransmitP__CSN__makeOutput(void ){
#line 46
  /*HplCC2420PinsC.CSNM*/Msp430GpioC__4__GeneralIO__makeOutput();
#line 46
}
#line 46
# 61 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P14*/HplMsp430GeneralIOP__4__IO__makeInput(void )
#line 61
{
  /* atomic removed: atomic calls only */
#line 61
  * (volatile uint8_t * )34U &= ~(0x01 << 4);
}

# 78 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*HplCC2420PinsC.CCAM*/Msp430GpioC__3__HplGeneralIO__makeInput(void ){
#line 78
  /*HplMsp430GeneralIOC.P14*/HplMsp430GeneralIOP__4__IO__makeInput();
#line 78
}
#line 78
# 52 "/home/csilab/tinyos/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplCC2420PinsC.CCAM*/Msp430GpioC__3__GeneralIO__makeInput(void )
#line 52
{
#line 52
  /*HplCC2420PinsC.CCAM*/Msp430GpioC__3__HplGeneralIO__makeInput();
}

# 44 "/home/csilab/tinyos/tos/interfaces/GeneralIO.nc"
inline static void CC2420TransmitP__CCA__makeInput(void ){
#line 44
  /*HplCC2420PinsC.CCAM*/Msp430GpioC__3__GeneralIO__makeInput();
#line 44
}
#line 44
# 179 "/home/csilab/upma/chips/cc2420/CC2420TransmitP.nc"
static inline error_t CC2420TransmitP__Init__init(void )
#line 179
{
  CC2420TransmitP__CCA__makeInput();
  CC2420TransmitP__CSN__makeOutput();
  CC2420TransmitP__SFD__makeInput();
  return SUCCESS;
}

# 127 "/home/csilab/upma/chips/cc2420/CC2420ReceiveP.nc"
static inline error_t CC2420ReceiveP__Init__init(void )
#line 127
{
  CC2420ReceiveP__m_p_rx_buf = &CC2420ReceiveP__m_rx_buf;
  return SUCCESS;
}

# 57 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  uint16_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__CC2int(/*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__cc_t x)
#line 57
{
#line 57
  union /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5____nesc_unnamed4413 {
#line 57
    /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__cc_t f;
#line 57
    uint16_t t;
  } 
#line 57
  c = { .f = x };

#line 57
  return c.t;
}

static inline uint16_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__compareControl(void )
{
  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__cc_t x = { 
  .cm = 1, 
  .ccis = 0, 
  .clld = 0, 
  .cap = 0, 
  .ccie = 0 };

  return /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__CC2int(x);
}

#line 105
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__setControlAsCompare(void )
{
  * (volatile uint16_t * )390U = /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__compareControl();
}

# 47 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430TimerControl__setControlAsCompare(void ){
#line 47
  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__setControlAsCompare();
#line 47
}
#line 47
# 53 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline error_t /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Init__init(void )
{
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430TimerControl__disableEvents();
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430TimerControl__setControlAsCompare();
  return SUCCESS;
}

# 55 "/home/csilab/tinyos/tos/system/RandomMlcgC.nc"
static inline error_t RandomMlcgC__Init__init(void )
#line 55
{
  /* atomic removed: atomic calls only */
#line 56
  RandomMlcgC__seed = (uint32_t )(TOS_NODE_ID + 1);

  return SUCCESS;
}

# 52 "/home/csilab/tinyos/tos/interfaces/Random.nc"
inline static uint16_t UniqueSendP__Random__rand16(void ){
#line 52
  unsigned int __nesc_result;
#line 52

#line 52
  __nesc_result = RandomMlcgC__Random__rand16();
#line 52

#line 52
  return __nesc_result;
#line 52
}
#line 52
# 62 "/home/csilab/tinyos/tos/chips/cc2420/unique/UniqueSendP.nc"
static inline error_t UniqueSendP__Init__init(void )
#line 62
{
  UniqueSendP__localSendId = UniqueSendP__Random__rand16();
  return SUCCESS;
}

# 71 "/home/csilab/tinyos/tos/chips/cc2420/unique/UniqueReceiveP.nc"
static inline error_t UniqueReceiveP__Init__init(void )
#line 71
{
  int i;

#line 73
  for (i = 0; i < 4; i++) {
      UniqueReceiveP__receivedMessages[i].source = (am_addr_t )0xFFFF;
      UniqueReceiveP__receivedMessages[i].dsn = 0;
    }
  return SUCCESS;
}

# 55 "/home/csilab/tinyos/tos/system/FcfsResourceQueueC.nc"
static inline error_t /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__Init__init(void )
#line 55
{
  memset(/*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__resQ, /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__NO_ENTRY, sizeof /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__resQ);
  return SUCCESS;
}

# 49 "/home/csilab/upma/system/LocalTime16P.nc"
static inline error_t /*LocalTime32khz16C.LocalTimeP*/LocalTime16P__0__Init__init(void )
#line 49
{
  memset(&/*LocalTime32khz16C.LocalTimeP*/LocalTime16P__0__g, 0, sizeof /*LocalTime32khz16C.LocalTimeP*/LocalTime16P__0__g);
  return SUCCESS;
}

# 862 "/home/csilab/upma/lib/macs/pure-tdma/PureTDMASchedulerP.nc"
static inline void PureTDMASchedulerP__schedule_reset_WH(uint8_t target_schedule[][6], uint8_t col, uint8_t row)
#line 862
{
  uint8_t i;
  uint8_t j;

#line 865
  for (i = 0; i < col; i++) {
      for (j = 0; j < row; j++) {
          PureTDMASchedulerP__schedule[i][j] = target_schedule[i][j];
        }
    }
}

# 70 "/home/csilab/upma/lib/macs/pure-tdma/AdaptiveHART_UtilC.nc"
static inline bool AdaptiveHART_UtilC__AdaptiveHART_Util__is_dest(uint8_t flowid, uint8_t nodeid)
#line 70
{
  bool result = FALSE;

#line 72
  if (AdaptiveHART_UtilC__flow_info[flowid][3] == nodeid) {
      result = TRUE;
    }
  return result;
}

# 8 "/home/csilab/upma/lib/macs/pure-tdma/AdaptiveHART_Util.nc"
inline static bool PureTDMASchedulerP__AH_Util__is_dest(uint8_t flowid, uint8_t nodeid){
#line 8
  unsigned char __nesc_result;
#line 8

#line 8
  __nesc_result = AdaptiveHART_UtilC__AdaptiveHART_Util__is_dest(flowid, nodeid);
#line 8

#line 8
  return __nesc_result;
#line 8
}
#line 8
# 108 "/home/csilab/upma/lib/macs/pure-tdma/AdaptiveHART_UtilC.nc"
static inline bool AdaptiveHART_UtilC__has_LS_type(uint8_t nodeid)
#line 108
{
  uint8_t i;
#line 109
  uint8_t hi_flow_root;
#line 109
  uint8_t lo_flow_root;
#line 109
  uint8_t dest;
#line 109
  uint8_t sender;
  uint8_t hi_receiver = 0;
  bool result = FALSE;

  hi_flow_root = AdaptiveHART_UtilC__flow_info[1][1];
  lo_flow_root = AdaptiveHART_UtilC__flow_info[2][1];


  for (i = hi_flow_root; i < 8; i++) {
      dest = AdaptiveHART_UtilC__HI_routing_map[i];
      if (dest != 0 && dest == nodeid) {
          hi_receiver = nodeid;
          break;
        }
    }


  for (sender = lo_flow_root; sender < 8; sender++) {
      dest = AdaptiveHART_UtilC__HI_routing_map[sender];
      if (dest != 0 && sender == nodeid && sender == hi_receiver) {
          result = TRUE;
          break;
        }
    }

  return result;
}

#line 91
static inline uint8_t AdaptiveHART_UtilC__AdaptiveHART_Util__set_TxOffset(uint8_t flowid, uint8_t nodeid)
#line 91
{


  if (flowid == 1) {
      return AdaptiveHART_UtilC__Offset_1;
    }
  else {
#line 96
    if (flowid == 2) {
        if (AdaptiveHART_UtilC__has_LS_type(nodeid) == TRUE) {
            return AdaptiveHART_UtilC__Offset_3;
          }
        else 
#line 99
          {
            return AdaptiveHART_UtilC__Offset_2;
          }
      }
    else 
#line 102
      {
        return 0;
      }
    }
}

# 10 "/home/csilab/upma/lib/macs/pure-tdma/AdaptiveHART_Util.nc"
inline static uint8_t PureTDMASchedulerP__AH_Util__set_TxOffset(uint8_t flowid, uint8_t nodeid){
#line 10
  unsigned char __nesc_result;
#line 10

#line 10
  __nesc_result = AdaptiveHART_UtilC__AdaptiveHART_Util__set_TxOffset(flowid, nodeid);
#line 10

#line 10
  return __nesc_result;
#line 10
}
#line 10
# 78 "/home/csilab/upma/lib/macs/pure-tdma/AdaptiveHART_UtilC.nc"
static inline uint8_t AdaptiveHART_UtilC__AdaptiveHART_Util__set_destination(uint8_t flowid, uint8_t nodeid)
#line 78
{
  uint8_t dest;

#line 80
  if (flowid == 1) {
      dest = AdaptiveHART_UtilC__HI_routing_map[nodeid];
    }
  else {
#line 82
    if (flowid == 2) {
        dest = AdaptiveHART_UtilC__LO_routing_map[nodeid];
      }
    else 
#line 84
      {
        dest = 100;
      }
    }
#line 87
  return dest;
}

# 9 "/home/csilab/upma/lib/macs/pure-tdma/AdaptiveHART_Util.nc"
inline static uint8_t PureTDMASchedulerP__AH_Util__set_destination(uint8_t flowid, uint8_t nodeid){
#line 9
  unsigned char __nesc_result;
#line 9

#line 9
  __nesc_result = AdaptiveHART_UtilC__AdaptiveHART_Util__set_destination(flowid, nodeid);
#line 9

#line 9
  return __nesc_result;
#line 9
}
#line 9
# 43 "/home/csilab/upma/lib/macs/pure-tdma/AdaptiveHART_UtilC.nc"
static inline uint8_t AdaptiveHART_UtilC__AdaptiveHART_Util__get_flow_id(uint8_t nodeid)
#line 43
{
  uint8_t i;
  uint8_t flow_id = 0;

  for (i = 1; i < 3; i++) {
      if (AdaptiveHART_UtilC__flow_info[i][1] == nodeid) {
          flow_id = AdaptiveHART_UtilC__flow_info[i][0];
          break;
        }
    }
  return flow_id;
}

# 6 "/home/csilab/upma/lib/macs/pure-tdma/AdaptiveHART_Util.nc"
inline static uint8_t PureTDMASchedulerP__AH_Util__get_flow_id(uint8_t nodeid){
#line 6
  unsigned char __nesc_result;
#line 6

#line 6
  __nesc_result = AdaptiveHART_UtilC__AdaptiveHART_Util__get_flow_id(nodeid);
#line 6

#line 6
  return __nesc_result;
#line 6
}
#line 6
# 30 "/home/csilab/upma/lib/macs/pure-tdma/AdaptiveHART_UtilC.nc"
static inline uint8_t AdaptiveHART_UtilC__AdaptiveHART_Util__get_release_offset(uint8_t nodeid)
#line 30
{
  uint8_t i;
  uint8_t release_offset = 0;

  for (i = 1; i < 3; i++) {
      if (AdaptiveHART_UtilC__flow_info[i][1] == nodeid) {
          release_offset = AdaptiveHART_UtilC__flow_info[i][2];
          break;
        }
    }
  return release_offset;
}

# 5 "/home/csilab/upma/lib/macs/pure-tdma/AdaptiveHART_Util.nc"
inline static uint8_t PureTDMASchedulerP__AH_Util__get_release_offset(uint8_t nodeid){
#line 5
  unsigned char __nesc_result;
#line 5

#line 5
  __nesc_result = AdaptiveHART_UtilC__AdaptiveHART_Util__get_release_offset(nodeid);
#line 5

#line 5
  return __nesc_result;
#line 5
}
#line 5
# 57 "/home/csilab/upma/lib/macs/pure-tdma/AdaptiveHART_UtilC.nc"
static inline bool AdaptiveHART_UtilC__AdaptiveHART_Util__is_root(uint8_t nodeid)
#line 57
{
  uint8_t i;
  bool result = FALSE;

  for (i = 1; i < 3; i++) {
      if (AdaptiveHART_UtilC__flow_info[i][1] == nodeid) {
          result = TRUE;
          break;
        }
    }
  return result;
}

# 7 "/home/csilab/upma/lib/macs/pure-tdma/AdaptiveHART_Util.nc"
inline static bool PureTDMASchedulerP__AH_Util__is_root(uint8_t nodeid){
#line 7
  unsigned char __nesc_result;
#line 7

#line 7
  __nesc_result = AdaptiveHART_UtilC__AdaptiveHART_Util__is_root(nodeid);
#line 7

#line 7
  return __nesc_result;
#line 7
}
#line 7
# 290 "/home/csilab/upma/lib/macs/pure-tdma/PureTDMASchedulerP.nc"
static inline error_t PureTDMASchedulerP__Init__init(void )
#line 290
{
  uint8_t j;

#line 292
  PureTDMASchedulerP__currentSlot = 0xff;
  PureTDMASchedulerP__slotSize = 100 * 32;


  PureTDMASchedulerP__bi = 4000;
  PureTDMASchedulerP__sd = 4000;
  PureTDMASchedulerP__cap = 0;
  PureTDMASchedulerP__coordinatorId = 0;
  PureTDMASchedulerP__init = FALSE;
  PureTDMASchedulerP__toSend = (void *)0;
  PureTDMASchedulerP__toSendLen = 0;
  PureTDMASchedulerP__sync = FALSE;
  /* atomic removed: atomic calls only */
  PureTDMASchedulerP__phyLock = FALSE;

  PureTDMASchedulerP__bi = PureTDMASchedulerP__gigaframe_length;
  PureTDMASchedulerP__sd = PureTDMASchedulerP__gigaframe_length;


  for (PureTDMASchedulerP__i = 0; PureTDMASchedulerP__i < 3; PureTDMASchedulerP__i++) {
      /* atomic removed: atomic calls only */
#line 312
      PureTDMASchedulerP__tx_opper[PureTDMASchedulerP__i] = 0;
      /* atomic removed: atomic calls only */
#line 313
      PureTDMASchedulerP__rcv_slot[PureTDMASchedulerP__i] = 0;
      PureTDMASchedulerP__Task_character[PureTDMASchedulerP__i][PureTDMASchedulerP__TASK_PERIOD] = PureTDMASchedulerP__original_period[PureTDMASchedulerP__i];
      PureTDMASchedulerP__Task_character[PureTDMASchedulerP__i][PureTDMASchedulerP__TASK_DEAD] = PureTDMASchedulerP__original_deadline[PureTDMASchedulerP__i];
    }


  PureTDMASchedulerP__is_root = PureTDMASchedulerP__AH_Util__is_root(TOS_NODE_ID);
  if (PureTDMASchedulerP__is_root == TRUE) {
      PureTDMASchedulerP__my_release_offset = PureTDMASchedulerP__AH_Util__get_release_offset(TOS_NODE_ID);
      /* atomic removed: atomic calls only */
#line 322
      PureTDMASchedulerP__my_flow_id = PureTDMASchedulerP__AH_Util__get_flow_id(TOS_NODE_ID);
    }


  for (PureTDMASchedulerP__i = 1; PureTDMASchedulerP__i < 3; PureTDMASchedulerP__i++) {
      /* atomic removed: atomic calls only */
#line 327
      PureTDMASchedulerP__my_dest[PureTDMASchedulerP__i] = PureTDMASchedulerP__AH_Util__set_destination(PureTDMASchedulerP__i, TOS_NODE_ID);
      /* atomic removed: atomic calls only */
#line 328
      PureTDMASchedulerP__my_txoffset[PureTDMASchedulerP__i] = PureTDMASchedulerP__AH_Util__set_TxOffset(PureTDMASchedulerP__i, TOS_NODE_ID);
      PureTDMASchedulerP__is_dest[PureTDMASchedulerP__i] = PureTDMASchedulerP__AH_Util__is_dest(PureTDMASchedulerP__i, TOS_NODE_ID);


      for (j = 0; j++; j < PureTDMASchedulerP__FLOW_SEQ) {
          PureTDMASchedulerP__rcv_slots[PureTDMASchedulerP__i][j] = 0;
          PureTDMASchedulerP__rcv_srcs[PureTDMASchedulerP__i][j] = 0;
        }
      PureTDMASchedulerP__rx_flow_seq[PureTDMASchedulerP__i] = 0;
    }


  PureTDMASchedulerP__random_slot = PureTDMASchedulerP__random_slots[PureTDMASchedulerP__random_slot_idx];


  PureTDMASchedulerP__schedule_reset_WH(PureTDMASchedulerP__origin_schedule, PureTDMASchedulerP__schedule_len, PureTDMASchedulerP__schedule_char_len);

  printf("dbg: HI_txoffset:%d, and LO_txoffset:%d\r\n", PureTDMASchedulerP__my_txoffset[1], PureTDMASchedulerP__my_txoffset[2]);

  return SUCCESS;
}

# 71 "/home/csilab/upma/system/GenericSlotSenderP.nc"
static inline error_t /*TDMASlotSenderC.TDMASlotSender.GenericSlotSenderP*/GenericSlotSenderP__0__Init__init(void )
#line 71
{
  /*TDMASlotSenderC.TDMASlotSender.GenericSlotSenderP*/GenericSlotSenderP__0__toSend = (void *)0;
  /*TDMASlotSenderC.TDMASlotSender.GenericSlotSenderP*/GenericSlotSenderP__0__toSendLen = 0;
  /*TDMASlotSenderC.TDMASlotSender.GenericSlotSenderP*/GenericSlotSenderP__0__state = /*TDMASlotSenderC.TDMASlotSender.GenericSlotSenderP*/GenericSlotSenderP__0__S_END;
  return SUCCESS;
}

# 52 "/home/csilab/upma/system/GenericSlotterP.nc"
static inline error_t GenericSlotterP__Init__init(void )
#line 52
{
  /* atomic removed: atomic calls only */
#line 53
  {
    GenericSlotterP__slotLength = 320;
    GenericSlotterP__frameLength = 17;
    GenericSlotterP__sync = FALSE;
  }
  return SUCCESS;
}

# 46 "/home/csilab/upma/system/SenderDispatcherP.nc"
static inline error_t SenderDispatcherP__Init__init(void )
#line 46
{
  SenderDispatcherP__toSend = (void *)0;
  SenderDispatcherP__toSendLen = 0;
  SenderDispatcherP__last_id = NO_SLOT;






  return SUCCESS;
}

# 198 "/home/csilab/upma/chips/cc2420/CC2420ActiveMessageP.nc"
static inline uint8_t CC2420ActiveMessageP__AMPacket__headerSize(void )
#line 198
{
  return CC2420_SIZE;
}

# 158 "/home/csilab/upma/interfaces/AMPacket.nc"
inline static uint8_t BeaconSlotP__AMPacket__headerSize(void ){
#line 158
  unsigned char __nesc_result;
#line 158

#line 158
  __nesc_result = CC2420ActiveMessageP__AMPacket__headerSize();
#line 158

#line 158
  return __nesc_result;
#line 158
}
#line 158
#line 110
inline static void BeaconSlotP__AMPacket__setSource(message_t * amsg, am_addr_t addr){
#line 110
  CC2420ActiveMessageP__AMPacket__setSource(amsg, addr);
#line 110
}
#line 110
#line 151
inline static void BeaconSlotP__AMPacket__setType(message_t * amsg, am_id_t t){
#line 151
  CC2420ActiveMessageP__AMPacket__setType(amsg, t);
#line 151
}
#line 151
#line 92
inline static void BeaconSlotP__AMPacket__setDestination(message_t * amsg, am_addr_t addr){
#line 92
  CC2420ActiveMessageP__AMPacket__setDestination(amsg, addr);
#line 92
}
#line 92
# 69 "/home/csilab/upma/system/BeaconSlotP.nc"
static inline error_t BeaconSlotP__Init__init(void )
#line 69
{
  BeaconSlotP__AMPacket__setDestination(&BeaconSlotP__syncPkt, AM_BROADCAST_ADDR);
  BeaconSlotP__AMPacket__setType(&BeaconSlotP__syncPkt, SIMPLE_TDMA_SYNC);
  BeaconSlotP__AMPacket__setSource(&BeaconSlotP__syncPkt, TOS_NODE_ID);
  BeaconSlotP__syncPktLen = BeaconSlotP__AMPacket__headerSize() + sizeof(nx_uint32_t );
  return SUCCESS;
}

# 95 "/home/csilab/tinyos/tos/interfaces/StdControl.nc"
inline static error_t SerialPrintfP__UartControl__start(void ){
#line 95
  unsigned char __nesc_result;
#line 95

#line 95
  __nesc_result = TelosSerialP__StdControl__start();
#line 95

#line 95
  return __nesc_result;
#line 95
}
#line 95
# 54 "/home/csilab/tinyos/tos/lib/printf/SerialPrintfP.nc"
static inline error_t SerialPrintfP__StdControl__start(void )
{
  return SerialPrintfP__UartControl__start();
}

#line 50
static inline error_t SerialPrintfP__Init__init(void )
#line 50
{
  return SerialPrintfP__StdControl__start();
}

# 62 "/home/csilab/tinyos/tos/interfaces/Init.nc"
inline static error_t RealMainP__SoftwareInit__init(void ){
#line 62
  unsigned char __nesc_result;
#line 62

#line 62
  __nesc_result = SerialPrintfP__Init__init();
#line 62
  __nesc_result = ecombine(__nesc_result, BeaconSlotP__Init__init());
#line 62
  __nesc_result = ecombine(__nesc_result, SenderDispatcherP__Init__init());
#line 62
  __nesc_result = ecombine(__nesc_result, GenericSlotterP__Init__init());
#line 62
  __nesc_result = ecombine(__nesc_result, /*TDMASlotSenderC.TDMASlotSender.GenericSlotSenderP*/GenericSlotSenderP__0__Init__init());
#line 62
  __nesc_result = ecombine(__nesc_result, PureTDMASchedulerP__Init__init());
#line 62
  __nesc_result = ecombine(__nesc_result, /*LocalTime32khz16C.LocalTimeP*/LocalTime16P__0__Init__init());
#line 62
  __nesc_result = ecombine(__nesc_result, /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__Init__init());
#line 62
  __nesc_result = ecombine(__nesc_result, UniqueReceiveP__Init__init());
#line 62
  __nesc_result = ecombine(__nesc_result, UniqueSendP__Init__init());
#line 62
  __nesc_result = ecombine(__nesc_result, RandomMlcgC__Init__init());
#line 62
  __nesc_result = ecombine(__nesc_result, /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Init__init());
#line 62
  __nesc_result = ecombine(__nesc_result, CC2420ReceiveP__Init__init());
#line 62
  __nesc_result = ecombine(__nesc_result, CC2420TransmitP__Init__init());
#line 62
  __nesc_result = ecombine(__nesc_result, /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Init__init());
#line 62
  __nesc_result = ecombine(__nesc_result, /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__2__Init__init());
#line 62
  __nesc_result = ecombine(__nesc_result, StateImplP__Init__init());
#line 62
  __nesc_result = ecombine(__nesc_result, CC2420ControlP__Init__init());
#line 62
  __nesc_result = ecombine(__nesc_result, PutcharP__Init__init());
#line 62
  __nesc_result = ecombine(__nesc_result, PrintfP__Init__init());
#line 62
  __nesc_result = ecombine(__nesc_result, /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__1__Init__init());
#line 62
  __nesc_result = ecombine(__nesc_result, SerialP__Init__init());
#line 62

#line 62
  return __nesc_result;
#line 62
}
#line 62
# 29 "/home/csilab/tinyos/tos/platforms/epic/chips/ds2411/DallasId48.h"
static inline bool dallasid48checkCrc(const dallasid48_serial_t *id)
#line 29
{
  uint8_t crc = 0;
  uint8_t idx;

#line 32
  for (idx = 0; idx < DALLASID48_DATA_LENGTH; idx++) {
      uint8_t i;

#line 34
      crc = crc ^ id->data[idx];
      for (i = 0; i < 8; i++) {
          if (crc & 0x01) {
              crc = (crc >> 1) ^ 0x8C;
            }
          else {
              crc >>= 1;
            }
        }
    }
  return crc == 0;
}

# 66 "/home/csilab/tinyos/tos/lib/timer/BusyWait.nc"
inline static void OneWireMasterC__BusyWait__wait(OneWireMasterC__BusyWait__size_type dt){
#line 66
  /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__BusyWait__wait(dt);
#line 66
}
#line 66
# 59 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline uint8_t /*HplMsp430GeneralIOC.P24*/HplMsp430GeneralIOP__12__IO__getRaw(void )
#line 59
{
#line 59
  return * (volatile uint8_t * )40U & (0x01 << 4);
}

#line 60
static inline bool /*HplMsp430GeneralIOC.P24*/HplMsp430GeneralIOP__12__IO__get(void )
#line 60
{
#line 60
  return /*HplMsp430GeneralIOC.P24*/HplMsp430GeneralIOP__12__IO__getRaw() != 0;
}

# 73 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static bool /*Ds2411C.Gpio*/Msp430GpioC__11__HplGeneralIO__get(void ){
#line 73
  unsigned char __nesc_result;
#line 73

#line 73
  __nesc_result = /*HplMsp430GeneralIOC.P24*/HplMsp430GeneralIOP__12__IO__get();
#line 73

#line 73
  return __nesc_result;
#line 73
}
#line 73
# 51 "/home/csilab/tinyos/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline bool /*Ds2411C.Gpio*/Msp430GpioC__11__GeneralIO__get(void )
#line 51
{
#line 51
  return /*Ds2411C.Gpio*/Msp430GpioC__11__HplGeneralIO__get();
}

# 43 "/home/csilab/tinyos/tos/interfaces/GeneralIO.nc"
inline static bool OneWireMasterC__Pin__get(void ){
#line 43
  unsigned char __nesc_result;
#line 43

#line 43
  __nesc_result = /*Ds2411C.Gpio*/Msp430GpioC__11__GeneralIO__get();
#line 43

#line 43
  return __nesc_result;
#line 43
}
#line 43
# 61 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P24*/HplMsp430GeneralIOP__12__IO__makeInput(void )
#line 61
{
  /* atomic removed: atomic calls only */
#line 61
  * (volatile uint8_t * )42U &= ~(0x01 << 4);
}

# 78 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*Ds2411C.Gpio*/Msp430GpioC__11__HplGeneralIO__makeInput(void ){
#line 78
  /*HplMsp430GeneralIOC.P24*/HplMsp430GeneralIOP__12__IO__makeInput();
#line 78
}
#line 78
# 52 "/home/csilab/tinyos/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*Ds2411C.Gpio*/Msp430GpioC__11__GeneralIO__makeInput(void )
#line 52
{
#line 52
  /*Ds2411C.Gpio*/Msp430GpioC__11__HplGeneralIO__makeInput();
}

# 44 "/home/csilab/tinyos/tos/interfaces/GeneralIO.nc"
inline static void OneWireMasterC__Pin__makeInput(void ){
#line 44
  /*Ds2411C.Gpio*/Msp430GpioC__11__GeneralIO__makeInput();
#line 44
}
#line 44
# 63 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P24*/HplMsp430GeneralIOP__12__IO__makeOutput(void )
#line 63
{
  /* atomic removed: atomic calls only */
#line 63
  * (volatile uint8_t * )42U |= 0x01 << 4;
}

# 85 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*Ds2411C.Gpio*/Msp430GpioC__11__HplGeneralIO__makeOutput(void ){
#line 85
  /*HplMsp430GeneralIOC.P24*/HplMsp430GeneralIOP__12__IO__makeOutput();
#line 85
}
#line 85
# 54 "/home/csilab/tinyos/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*Ds2411C.Gpio*/Msp430GpioC__11__GeneralIO__makeOutput(void )
#line 54
{
#line 54
  /*Ds2411C.Gpio*/Msp430GpioC__11__HplGeneralIO__makeOutput();
}

# 46 "/home/csilab/tinyos/tos/interfaces/GeneralIO.nc"
inline static void OneWireMasterC__Pin__makeOutput(void ){
#line 46
  /*Ds2411C.Gpio*/Msp430GpioC__11__GeneralIO__makeOutput();
#line 46
}
#line 46
# 56 "/home/csilab/tinyos/tos/platforms/epic/chips/ds2411/OneWireMasterC.nc"
static inline bool OneWireMasterC__readBit(void )
#line 56
{
  bool bit;

#line 58
  OneWireMasterC__Pin__makeOutput();
  OneWireMasterC__BusyWait__wait(OneWireMasterC__DELAY_5US);
  OneWireMasterC__Pin__makeInput();
  OneWireMasterC__BusyWait__wait(OneWireMasterC__DELAY_5US);
  bit = OneWireMasterC__Pin__get();
  OneWireMasterC__BusyWait__wait(OneWireMasterC__SLOT_TIME);
  return bit;
}

#line 80
static inline uint8_t OneWireMasterC__readByte(void )
#line 80
{
  uint8_t i;
#line 81
  uint8_t c = 0;

#line 82
  for (i = 0; i < 8; i++) {
      c >>= 1;
      if (OneWireMasterC__readBit()) {
          c |= 0x80;
        }
    }
  return c;
}

#line 49
static inline void OneWireMasterC__writeZero(void )
#line 49
{
  OneWireMasterC__Pin__makeOutput();
  OneWireMasterC__BusyWait__wait(OneWireMasterC__DELAY_60US);
  OneWireMasterC__Pin__makeInput();
  OneWireMasterC__BusyWait__wait(OneWireMasterC__DELAY_5US);
}

#line 42
static inline void OneWireMasterC__writeOne(void )
#line 42
{
  OneWireMasterC__Pin__makeOutput();
  OneWireMasterC__BusyWait__wait(OneWireMasterC__DELAY_5US);
  OneWireMasterC__Pin__makeInput();
  OneWireMasterC__BusyWait__wait(OneWireMasterC__SLOT_TIME);
}

#line 67
static inline void OneWireMasterC__writeByte(uint8_t c)
#line 67
{
  uint8_t j;

#line 69
  for (j = 0; j < 8; j++) {
      if (c & 0x01) {
          OneWireMasterC__writeOne();
        }
      else {
          OneWireMasterC__writeZero();
        }
      c >>= 1;
    }
}

# 57 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P24*/HplMsp430GeneralIOP__12__IO__clr(void )
#line 57
{
  /* atomic removed: atomic calls only */
#line 57
  * (volatile uint8_t * )41U &= ~(0x01 << 4);
}

# 53 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*Ds2411C.Gpio*/Msp430GpioC__11__HplGeneralIO__clr(void ){
#line 53
  /*HplMsp430GeneralIOC.P24*/HplMsp430GeneralIOP__12__IO__clr();
#line 53
}
#line 53
# 49 "/home/csilab/tinyos/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*Ds2411C.Gpio*/Msp430GpioC__11__GeneralIO__clr(void )
#line 49
{
#line 49
  /*Ds2411C.Gpio*/Msp430GpioC__11__HplGeneralIO__clr();
}

# 41 "/home/csilab/tinyos/tos/interfaces/GeneralIO.nc"
inline static void OneWireMasterC__Pin__clr(void ){
#line 41
  /*Ds2411C.Gpio*/Msp430GpioC__11__GeneralIO__clr();
#line 41
}
#line 41
# 27 "/home/csilab/tinyos/tos/platforms/epic/chips/ds2411/OneWireMasterC.nc"
static inline bool OneWireMasterC__reset(void )
#line 27
{
  uint16_t i;

#line 29
  OneWireMasterC__Pin__makeInput();
  OneWireMasterC__Pin__clr();
  OneWireMasterC__Pin__makeOutput();
  OneWireMasterC__BusyWait__wait(OneWireMasterC__RESET_LOW_TIME);
  OneWireMasterC__Pin__makeInput();
  OneWireMasterC__BusyWait__wait(OneWireMasterC__DELAY_60US);

  for (i = 0; i < OneWireMasterC__PRESENCE_DETECT_LOW_TIME; i += OneWireMasterC__DELAY_5US, OneWireMasterC__BusyWait__wait(OneWireMasterC__DELAY_5US)) 
    if (!OneWireMasterC__Pin__get()) {
#line 37
      break;
      }
#line 38
  OneWireMasterC__BusyWait__wait(OneWireMasterC__PRESENCE_RESET_HIGH_TIME - OneWireMasterC__DELAY_60US);
  return i < OneWireMasterC__PRESENCE_DETECT_LOW_TIME;
}

#line 91
static inline error_t OneWireMasterC__OneWire__read(uint8_t cmd, uint8_t *buf, uint8_t len)
#line 91
{
  error_t e = SUCCESS;

  /* atomic removed: atomic calls only */
#line 93
  {
    if (OneWireMasterC__reset()) {
        uint8_t i;

#line 96
        OneWireMasterC__writeByte(cmd);
        for (i = 0; i < len; i++) {
            buf[i] = OneWireMasterC__readByte();
          }
      }
    else {
        e = EOFF;
      }
  }
  return e;
}

# 10 "/home/csilab/tinyos/tos/platforms/epic/chips/ds2411/OneWireStream.nc"
inline static error_t Ds2411P__OneWire__read(uint8_t cmd, uint8_t *buf, uint8_t len){
#line 10
  unsigned char __nesc_result;
#line 10

#line 10
  __nesc_result = OneWireMasterC__OneWire__read(cmd, buf, len);
#line 10

#line 10
  return __nesc_result;
#line 10
}
#line 10
# 23 "/home/csilab/tinyos/tos/platforms/epic/chips/ds2411/Ds2411P.nc"
static inline error_t Ds2411P__readId(void )
#line 23
{
  error_t e = Ds2411P__OneWire__read(0x33, Ds2411P__ds2411id.data, DALLASID48_DATA_LENGTH);

#line 25
  if (e == SUCCESS) {
      if (dallasid48checkCrc(&Ds2411P__ds2411id)) {
          Ds2411P__haveId = TRUE;
        }
      else {
          e = EINVAL;
        }
    }
  return e;
}

static inline error_t Ds2411P__ReadId48__read(uint8_t *id)
#line 36
{
  error_t e = SUCCESS;

#line 38
  if (!Ds2411P__haveId) {
      e = Ds2411P__readId();
    }
  if (Ds2411P__haveId) {
      uint8_t i;

#line 43
      for (i = 0; i < DALLASID48_SERIAL_LENGTH; i++) {
          id[i] = Ds2411P__ds2411id.serial[i];
        }
    }
  return e;
}

# 12 "/home/csilab/tinyos/tos/platforms/epic/chips/ds2411/ReadId48.nc"
inline static error_t DallasId48ToIeeeEui64C__ReadId48__read(uint8_t *id){
#line 12
  unsigned char __nesc_result;
#line 12

#line 12
  __nesc_result = Ds2411P__ReadId48__read(id);
#line 12

#line 12
  return __nesc_result;
#line 12
}
#line 12
# 62 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__get(void )
{




  if (0) {
      /* atomic removed: atomic calls only */
#line 69
      {
        uint16_t t0;
        uint16_t t1 = * (volatile uint16_t * )368U;

#line 72
        do {
#line 72
            t0 = t1;
#line 72
            t1 = * (volatile uint16_t * )368U;
          }
        while (
#line 72
        t0 != t1);
        {
          unsigned int __nesc_temp = 
#line 73
          t1;

#line 73
          return __nesc_temp;
        }
      }
    }
  else 
#line 76
    {
      return * (volatile uint16_t * )368U;
    }
}

# 45 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430Timer.nc"
inline static uint16_t /*Msp430CounterMicroC.Counter*/Msp430CounterC__1__Msp430Timer__get(void ){
#line 45
  unsigned int __nesc_result;
#line 45

#line 45
  __nesc_result = /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__get();
#line 45

#line 45
  return __nesc_result;
#line 45
}
#line 45
# 49 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430CounterC.nc"
static inline uint16_t /*Msp430CounterMicroC.Counter*/Msp430CounterC__1__Counter__get(void )
{
  return /*Msp430CounterMicroC.Counter*/Msp430CounterC__1__Msp430Timer__get();
}

# 64 "/home/csilab/tinyos/tos/lib/timer/Counter.nc"
inline static /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__Counter__size_type /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__Counter__get(void ){
#line 64
  unsigned int __nesc_result;
#line 64

#line 64
  __nesc_result = /*Msp430CounterMicroC.Counter*/Msp430CounterC__1__Counter__get();
#line 64

#line 64
  return __nesc_result;
#line 64
}
#line 64
# 63 "/home/csilab/upma/system/GenericSlotSenderP.nc"
static inline void /*TDMASlotSenderC.TDMASlotSender.GenericSlotSenderP*/GenericSlotSenderP__0__Boot__booted(void )
{
}

# 61 "/home/csilab/upma/system/GenericSlotterP.nc"
static inline void GenericSlotterP__Boot__booted(void )
{
}

# 60 "/home/csilab/tinyos/tos/interfaces/Boot.nc"
inline static void RealMainP__Boot__booted(void ){
#line 60
  GenericSlotterP__Boot__booted();
#line 60
  /*TDMASlotSenderC.TDMASlotSender.GenericSlotSenderP*/GenericSlotSenderP__0__Boot__booted();
#line 60
  /*TestPureTdmaC.SendingApp.App*/SendingC__0__Boot__booted();
#line 60
  /*TestPureTdmaC.SendingApp.App*/SendingC__0__Boot__booted();
#line 60
}
#line 60
# 64 "/home/csilab/tinyos/tos/lib/timer/Timer.nc"
inline static void /*TestPureTdmaC.SendingApp.App*/SendingC__0__FlushTimer__startPeriodic(uint32_t dt){
#line 64
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startPeriodic(2U, dt);
#line 64
}
#line 64
# 35 "/home/csilab/upma/interfaces/RadioPowerControl.nc"
inline static error_t PureTDMASchedulerP__RadioPowerControl__start(void ){
#line 35
  unsigned char __nesc_result;
#line 35

#line 35
  __nesc_result = CC2420CsmaP__RadioPowerControl__start();
#line 35

#line 35
  return __nesc_result;
#line 35
}
#line 35
# 87 "/home/csilab/upma/system/GenericSlotterP.nc"
static inline void GenericSlotterP__FrameConfiguration__setFrameLength(uint8_t numSlots)
#line 87
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 88
    GenericSlotterP__frameLength = numSlots;
#line 88
    __nesc_atomic_end(__nesc_atomic); }
}

# 33 "/home/csilab/upma/interfaces/FrameConfiguration.nc"
inline static void PureTDMASchedulerP__FrameConfiguration__setFrameLength(uint8_t numSlots){
#line 33
  GenericSlotterP__FrameConfiguration__setFrameLength(numSlots);
#line 33
}
#line 33
# 83 "/home/csilab/upma/system/GenericSlotterP.nc"
static inline void GenericSlotterP__FrameConfiguration__setSlotLength(uint32_t slotTimeBms)
#line 83
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 84
    GenericSlotterP__slotLength = slotTimeBms;
#line 84
    __nesc_atomic_end(__nesc_atomic); }
}

# 32 "/home/csilab/upma/interfaces/FrameConfiguration.nc"
inline static void PureTDMASchedulerP__FrameConfiguration__setSlotLength(uint32_t slotTimeBms){
#line 32
  GenericSlotterP__FrameConfiguration__setSlotLength(slotTimeBms);
#line 32
}
#line 32
# 350 "/home/csilab/upma/lib/macs/pure-tdma/PureTDMASchedulerP.nc"
static inline error_t PureTDMASchedulerP__SplitControl__start(void )
#line 350
{
  error_t err;

#line 352
  if (PureTDMASchedulerP__init == FALSE) {
      PureTDMASchedulerP__FrameConfiguration__setSlotLength(PureTDMASchedulerP__slotSize);
      PureTDMASchedulerP__FrameConfiguration__setFrameLength(PureTDMASchedulerP__bi + 1);
    }

  err = PureTDMASchedulerP__RadioPowerControl__start();
  return err;
}

# 104 "/home/csilab/tinyos/tos/interfaces/SplitControl.nc"
inline static error_t /*TestPureTdmaC.SendingApp.App*/SendingC__0__SplitControl__start(void ){
#line 104
  unsigned char __nesc_result;
#line 104

#line 104
  __nesc_result = PureTDMASchedulerP__SplitControl__start();
#line 104

#line 104
  return __nesc_result;
#line 104
}
#line 104
# 391 "/home/csilab/tinyos/tos/chips/msp430/msp430hardware.h"
static inline  void __nesc_disable_interrupt(void )
{
  __dint();
  __nop();
}

# 63 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430ClockP.nc"
static inline mcu_power_t Msp430ClockP__McuPowerOverride__lowestState(void )
#line 63
{
  return MSP430_POWER_LPM3;
}

# 62 "/home/csilab/tinyos/tos/interfaces/McuPowerOverride.nc"
inline static mcu_power_t McuSleepC__McuPowerOverride__lowestState(void ){
#line 62
  unsigned char __nesc_result;
#line 62

#line 62
  __nesc_result = Msp430ClockP__McuPowerOverride__lowestState();
#line 62

#line 62
  return __nesc_result;
#line 62
}
#line 62
# 74 "/home/csilab/tinyos/tos/chips/msp430/McuSleepC.nc"
static inline mcu_power_t McuSleepC__getPowerState(void )
#line 74
{
  mcu_power_t pState = MSP430_POWER_LPM4;









  if ((((((
#line 77
  TACCTL0 & 0x0010 || 
  TACCTL1 & 0x0010) || 
  TACCTL2 & 0x0010) && (
  TACTL & 0x0300) == 0x0200) || (
  ME1 & (0x80 | 0x40) && U0TCTL & 0x20)) || (
  ME2 & (0x20 | 0x10) && U1TCTL & 0x20))


   || (U0CTLnr & 0x01 && I2CTCTLnr & 0x20 && 
  I2CDCTLnr & 0x20 && U0CTLnr & 0x04 && U0CTLnr & 0x20)) {


    pState = MSP430_POWER_LPM1;
    }


  if (ADC12CTL0 & 0x010) {
      if (ADC12CTL1 & 0x0010) {

          if (ADC12CTL1 & 0x0008) {
            pState = MSP430_POWER_LPM1;
            }
          else {
#line 99
            pState = MSP430_POWER_ACTIVE;
            }
        }
      else {
#line 100
        if (ADC12CTL1 & 0x0400 && (TACTL & 0x0300) == 0x0200) {



            pState = MSP430_POWER_LPM1;
          }
        }
    }

  return pState;
}

# 379 "/home/csilab/tinyos/tos/chips/msp430/msp430hardware.h"
static inline  mcu_power_t mcombine(mcu_power_t m1, mcu_power_t m2)
#line 379
{
  return m1 < m2 ? m1 : m2;
}

# 112 "/home/csilab/tinyos/tos/chips/msp430/McuSleepC.nc"
static inline void McuSleepC__computePowerState(void )
#line 112
{
  McuSleepC__powerState = mcombine(McuSleepC__getPowerState(), 
  McuSleepC__McuPowerOverride__lowestState());
}

static inline void McuSleepC__McuSleep__sleep(void )
#line 117
{
  uint16_t temp;

#line 119
  if (McuSleepC__dirty) {
      McuSleepC__computePowerState();
    }

  temp = McuSleepC__msp430PowerBits[McuSleepC__powerState] | 0x0008;
   __asm volatile ("bis  %0, r2" :  : "m"(temp));

   __asm volatile ("" :  :  : "memory");
  __nesc_disable_interrupt();
}

# 76 "/home/csilab/tinyos/tos/interfaces/McuSleep.nc"
inline static void SchedulerBasicP__McuSleep__sleep(void ){
#line 76
  McuSleepC__McuSleep__sleep();
#line 76
}
#line 76
# 78 "/home/csilab/tinyos/tos/system/SchedulerBasicP.nc"
static __inline uint8_t SchedulerBasicP__popTask(void )
{
  if (SchedulerBasicP__m_head != SchedulerBasicP__NO_TASK) 
    {
      uint8_t id = SchedulerBasicP__m_head;

#line 83
      SchedulerBasicP__m_head = SchedulerBasicP__m_next[SchedulerBasicP__m_head];
      if (SchedulerBasicP__m_head == SchedulerBasicP__NO_TASK) 
        {
          SchedulerBasicP__m_tail = SchedulerBasicP__NO_TASK;
        }
      SchedulerBasicP__m_next[id] = SchedulerBasicP__NO_TASK;
      return id;
    }
  else 
    {
      return SchedulerBasicP__NO_TASK;
    }
}

#line 149
static inline void SchedulerBasicP__Scheduler__taskLoop(void )
{
  for (; ; ) 
    {
      uint8_t nextTask;

      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
        {
          while ((nextTask = SchedulerBasicP__popTask()) == SchedulerBasicP__NO_TASK) 
            {
              SchedulerBasicP__McuSleep__sleep();
            }
        }
#line 161
        __nesc_atomic_end(__nesc_atomic); }
      SchedulerBasicP__TaskBasic__runTask(nextTask);
    }
}

# 72 "/home/csilab/tinyos/tos/interfaces/Scheduler.nc"
inline static void RealMainP__Scheduler__taskLoop(void ){
#line 72
  SchedulerBasicP__Scheduler__taskLoop();
#line 72
}
#line 72
# 61 "/home/csilab/tinyos/tos/system/QueueC.nc"
static inline uint8_t /*PrintfC.QueueC*/QueueC__0__Queue__maxSize(void )
#line 61
{
  return 250;
}

#line 57
static inline uint8_t /*PrintfC.QueueC*/QueueC__0__Queue__size(void )
#line 57
{
  return /*PrintfC.QueueC*/QueueC__0__size;
}

#line 97
static inline error_t /*PrintfC.QueueC*/QueueC__0__Queue__enqueue(/*PrintfC.QueueC*/QueueC__0__queue_t newVal)
#line 97
{
  if (/*PrintfC.QueueC*/QueueC__0__Queue__size() < /*PrintfC.QueueC*/QueueC__0__Queue__maxSize()) {
      ;
      /*PrintfC.QueueC*/QueueC__0__queue[/*PrintfC.QueueC*/QueueC__0__tail] = newVal;
      /*PrintfC.QueueC*/QueueC__0__tail++;
      if (/*PrintfC.QueueC*/QueueC__0__tail == 250) {
#line 102
        /*PrintfC.QueueC*/QueueC__0__tail = 0;
        }
#line 103
      /*PrintfC.QueueC*/QueueC__0__size++;
      /*PrintfC.QueueC*/QueueC__0__printQueue();
      return SUCCESS;
    }
  else {
      return FAIL;
    }
}

# 90 "/home/csilab/tinyos/tos/interfaces/Queue.nc"
inline static error_t PrintfP__Queue__enqueue(PrintfP__Queue__t  newVal){
#line 90
  unsigned char __nesc_result;
#line 90

#line 90
  __nesc_result = /*PrintfC.QueueC*/QueueC__0__Queue__enqueue(newVal);
#line 90

#line 90
  return __nesc_result;
#line 90
}
#line 90
#line 58
inline static uint8_t PrintfP__Queue__size(void ){
#line 58
  unsigned char __nesc_result;
#line 58

#line 58
  __nesc_result = /*PrintfC.QueueC*/QueueC__0__Queue__size();
#line 58

#line 58
  return __nesc_result;
#line 58
}
#line 58
# 152 "/home/csilab/tinyos/tos/lib/printf/PrintfP.nc"
static inline int PrintfP__Putchar__putchar(int c)
{
  if (PrintfP__state == PrintfP__S_STARTED && PrintfP__Queue__size() >= 250 / 2) {
      PrintfP__state = PrintfP__S_FLUSHING;
      PrintfP__sendNext();
    }
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 158
    {
      if (PrintfP__Queue__enqueue(c) == SUCCESS) 
        {
          int __nesc_temp = 
#line 160
          0;

          {
#line 160
            __nesc_atomic_end(__nesc_atomic); 
#line 160
            return __nesc_temp;
          }
        }
      else 
#line 161
        {
          int __nesc_temp = 
#line 161
          -1;

          {
#line 161
            __nesc_atomic_end(__nesc_atomic); 
#line 161
            return __nesc_temp;
          }
        }
    }
#line 164
    __nesc_atomic_end(__nesc_atomic); }
}

# 370 "/home/csilab/tinyos/tos/chips/msp430/usart/HplMsp430Usart1P.nc"
static inline void HplMsp430Usart1P__Usart__enableTxIntr(void )
#line 370
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 371
    {
      HplMsp430Usart1P__IFG2 &= ~0x20;
      HplMsp430Usart1P__IE2 |= 0x20;
    }
#line 374
    __nesc_atomic_end(__nesc_atomic); }
}

# 181 "/home/csilab/tinyos/tos/chips/msp430/usart/HplMsp430Usart.nc"
inline static void /*Msp430Uart1P.UartP*/Msp430UartP__0__Usart__enableTxIntr(void ){
#line 181
  HplMsp430Usart1P__Usart__enableTxIntr();
#line 181
}
#line 181
# 339 "/home/csilab/tinyos/tos/chips/msp430/usart/HplMsp430Usart1P.nc"
static inline void HplMsp430Usart1P__Usart__clrTxIntr(void )
#line 339
{
  HplMsp430Usart1P__IFG2 &= ~0x20;
}

# 202 "/home/csilab/tinyos/tos/chips/msp430/usart/HplMsp430Usart.nc"
inline static void /*Msp430Uart1P.UartP*/Msp430UartP__0__Usart__clrTxIntr(void ){
#line 202
  HplMsp430Usart1P__Usart__clrTxIntr();
#line 202
}
#line 202
# 318 "/home/csilab/tinyos/tos/chips/msp430/usart/HplMsp430Usart1P.nc"
static inline bool HplMsp430Usart1P__Usart__isTxIntrPending(void )
#line 318
{
  if (HplMsp430Usart1P__IFG2 & 0x20) {
      return TRUE;
    }
  return FALSE;
}

# 187 "/home/csilab/tinyos/tos/chips/msp430/usart/HplMsp430Usart.nc"
inline static bool /*Msp430Uart1P.UartP*/Msp430UartP__0__Usart__isTxIntrPending(void ){
#line 187
  unsigned char __nesc_result;
#line 187

#line 187
  __nesc_result = HplMsp430Usart1P__Usart__isTxIntrPending();
#line 187

#line 187
  return __nesc_result;
#line 187
}
#line 187
# 355 "/home/csilab/tinyos/tos/chips/msp430/usart/HplMsp430Usart1P.nc"
static inline void HplMsp430Usart1P__Usart__disableTxIntr(void )
#line 355
{
  HplMsp430Usart1P__IE2 &= ~0x20;
}

# 178 "/home/csilab/tinyos/tos/chips/msp430/usart/HplMsp430Usart.nc"
inline static void /*Msp430Uart1P.UartP*/Msp430UartP__0__Usart__disableTxIntr(void ){
#line 178
  HplMsp430Usart1P__Usart__disableTxIntr();
#line 178
}
#line 178
# 178 "/home/csilab/tinyos/tos/chips/msp430/usart/Msp430UartP.nc"
static inline error_t /*Msp430Uart1P.UartP*/Msp430UartP__0__UartByte__send(uint8_t id, uint8_t data)
#line 178
{
  if (/*Msp430Uart1P.UartP*/Msp430UartP__0__UsartResource__isOwner(id) == FALSE) {
    return FAIL;
    }
#line 181
  /*Msp430Uart1P.UartP*/Msp430UartP__0__Usart__clrTxIntr();
  /*Msp430Uart1P.UartP*/Msp430UartP__0__Usart__disableTxIntr();
  /*Msp430Uart1P.UartP*/Msp430UartP__0__Usart__tx(data);
  while (!/*Msp430Uart1P.UartP*/Msp430UartP__0__Usart__isTxIntrPending()) ;
  /*Msp430Uart1P.UartP*/Msp430UartP__0__Usart__clrTxIntr();
  /*Msp430Uart1P.UartP*/Msp430UartP__0__Usart__enableTxIntr();
  return SUCCESS;
}

# 46 "/home/csilab/tinyos/tos/interfaces/UartByte.nc"
inline static error_t SerialPrintfP__UartByte__send(uint8_t byte){
#line 46
  unsigned char __nesc_result;
#line 46

#line 46
  __nesc_result = /*Msp430Uart1P.UartP*/Msp430UartP__0__UartByte__send(/*PlatformSerialC.UartC*/Msp430Uart1C__0__CLIENT_ID, byte);
#line 46

#line 46
  return __nesc_result;
#line 46
}
#line 46
# 69 "/home/csilab/tinyos/tos/lib/printf/SerialPrintfP.nc"
static inline int SerialPrintfP__Putchar__putchar(int c)
{
  return SUCCESS == SerialPrintfP__UartByte__send(c) ? c : -1;
}

# 49 "/home/csilab/tinyos/tos/lib/printf/Putchar.nc"
inline static int PutcharP__Putchar__putchar(int c){
#line 49
  int __nesc_result;
#line 49

#line 49
  __nesc_result = SerialPrintfP__Putchar__putchar(c);
#line 49
  __nesc_result = PrintfP__Putchar__putchar(c);
#line 49

#line 49
  return __nesc_result;
#line 49
}
#line 49
# 188 "/home/csilab/upma/chips/cc2420/CC2420ReceiveP.nc"
static inline void CC2420ReceiveP__InterruptFIFOP__fired(void )
#line 188
{
  if (CC2420ReceiveP__m_state == CC2420ReceiveP__S_STARTED) {
      CC2420ReceiveP__beginReceive();
    }
  else {
      CC2420ReceiveP__m_missed_packets++;
    }
}

# 68 "/home/csilab/tinyos/tos/interfaces/GpioInterrupt.nc"
inline static void /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__Interrupt__fired(void ){
#line 68
  CC2420ReceiveP__InterruptFIFOP__fired();
#line 68
}
#line 68
# 77 "/home/csilab/tinyos/tos/chips/msp430/pins/Msp430InterruptC.nc"
static inline void /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__HplInterrupt__fired(void )
#line 77
{
  /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__HplInterrupt__clear();
  /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__Interrupt__fired();
}

# 72 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port10__fired(void ){
#line 72
  /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__HplInterrupt__fired();
#line 72
}
#line 72
# 103 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port11__clear(void )
#line 103
{
#line 103
  P1IFG &= ~(1 << 1);
}

#line 79
static inline void HplMsp430InterruptP__Port11__default__fired(void )
#line 79
{
#line 79
  HplMsp430InterruptP__Port11__clear();
}

# 72 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port11__fired(void ){
#line 72
  HplMsp430InterruptP__Port11__default__fired();
#line 72
}
#line 72
# 104 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port12__clear(void )
#line 104
{
#line 104
  P1IFG &= ~(1 << 2);
}

#line 80
static inline void HplMsp430InterruptP__Port12__default__fired(void )
#line 80
{
#line 80
  HplMsp430InterruptP__Port12__clear();
}

# 72 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port12__fired(void ){
#line 72
  HplMsp430InterruptP__Port12__default__fired();
#line 72
}
#line 72
# 105 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port13__clear(void )
#line 105
{
#line 105
  P1IFG &= ~(1 << 3);
}

#line 81
static inline void HplMsp430InterruptP__Port13__default__fired(void )
#line 81
{
#line 81
  HplMsp430InterruptP__Port13__clear();
}

# 72 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port13__fired(void ){
#line 72
  HplMsp430InterruptP__Port13__default__fired();
#line 72
}
#line 72
# 67 "/home/csilab/tinyos/tos/interfaces/TaskBasic.nc"
inline static error_t CC2420CsmaP__startDone_task__postTask(void ){
#line 67
  unsigned char __nesc_result;
#line 67

#line 67
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(CC2420CsmaP__startDone_task);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 209 "/home/csilab/upma/chips/cc2420/CC2420CsmaP.nc"
static inline void CC2420CsmaP__CC2420Power__startOscillatorDone(void )
#line 209
{
  CC2420CsmaP__startDone_task__postTask();
}

# 76 "/home/csilab/tinyos/tos/chips/cc2420/interfaces/CC2420Power.nc"
inline static void CC2420ControlP__CC2420Power__startOscillatorDone(void ){
#line 76
  CC2420CsmaP__CC2420Power__startOscillatorDone();
#line 76
}
#line 76
# 61 "/home/csilab/tinyos/tos/interfaces/GpioInterrupt.nc"
inline static error_t CC2420ControlP__InterruptCCA__disable(void ){
#line 61
  unsigned char __nesc_result;
#line 61

#line 61
  __nesc_result = /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__Interrupt__disable();
#line 61

#line 61
  return __nesc_result;
#line 61
}
#line 61
# 441 "/home/csilab/tinyos/tos/chips/cc2420/control/CC2420ControlP.nc"
static inline void CC2420ControlP__InterruptCCA__fired(void )
#line 441
{
  CC2420ControlP__m_state = CC2420ControlP__S_XOSC_STARTED;
  CC2420ControlP__InterruptCCA__disable();
  CC2420ControlP__IOCFG1__write(0);
  CC2420ControlP__writeId();
  CC2420ControlP__CSN__set();
  CC2420ControlP__CSN__clr();
  CC2420ControlP__CC2420Power__startOscillatorDone();
}

# 68 "/home/csilab/tinyos/tos/interfaces/GpioInterrupt.nc"
inline static void /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__Interrupt__fired(void ){
#line 68
  CC2420ControlP__InterruptCCA__fired();
#line 68
}
#line 68
# 77 "/home/csilab/tinyos/tos/chips/msp430/pins/Msp430InterruptC.nc"
static inline void /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__HplInterrupt__fired(void )
#line 77
{
  /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__HplInterrupt__clear();
  /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__Interrupt__fired();
}

# 72 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port14__fired(void ){
#line 72
  /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__HplInterrupt__fired();
#line 72
}
#line 72
# 107 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port15__clear(void )
#line 107
{
#line 107
  P1IFG &= ~(1 << 5);
}

#line 83
static inline void HplMsp430InterruptP__Port15__default__fired(void )
#line 83
{
#line 83
  HplMsp430InterruptP__Port15__clear();
}

# 72 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port15__fired(void ){
#line 72
  HplMsp430InterruptP__Port15__default__fired();
#line 72
}
#line 72
# 108 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port16__clear(void )
#line 108
{
#line 108
  P1IFG &= ~(1 << 6);
}

#line 84
static inline void HplMsp430InterruptP__Port16__default__fired(void )
#line 84
{
#line 84
  HplMsp430InterruptP__Port16__clear();
}

# 72 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port16__fired(void ){
#line 72
  HplMsp430InterruptP__Port16__default__fired();
#line 72
}
#line 72
# 109 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port17__clear(void )
#line 109
{
#line 109
  P1IFG &= ~(1 << 7);
}

#line 85
static inline void HplMsp430InterruptP__Port17__default__fired(void )
#line 85
{
#line 85
  HplMsp430InterruptP__Port17__clear();
}

# 72 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port17__fired(void ){
#line 72
  HplMsp430InterruptP__Port17__default__fired();
#line 72
}
#line 72
# 206 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port20__clear(void )
#line 206
{
#line 206
  P2IFG &= ~(1 << 0);
}

#line 182
static inline void HplMsp430InterruptP__Port20__default__fired(void )
#line 182
{
#line 182
  HplMsp430InterruptP__Port20__clear();
}

# 72 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port20__fired(void ){
#line 72
  HplMsp430InterruptP__Port20__default__fired();
#line 72
}
#line 72
# 207 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port21__clear(void )
#line 207
{
#line 207
  P2IFG &= ~(1 << 1);
}

#line 183
static inline void HplMsp430InterruptP__Port21__default__fired(void )
#line 183
{
#line 183
  HplMsp430InterruptP__Port21__clear();
}

# 72 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port21__fired(void ){
#line 72
  HplMsp430InterruptP__Port21__default__fired();
#line 72
}
#line 72
# 208 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port22__clear(void )
#line 208
{
#line 208
  P2IFG &= ~(1 << 2);
}

#line 184
static inline void HplMsp430InterruptP__Port22__default__fired(void )
#line 184
{
#line 184
  HplMsp430InterruptP__Port22__clear();
}

# 72 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port22__fired(void ){
#line 72
  HplMsp430InterruptP__Port22__default__fired();
#line 72
}
#line 72
# 209 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port23__clear(void )
#line 209
{
#line 209
  P2IFG &= ~(1 << 3);
}

#line 185
static inline void HplMsp430InterruptP__Port23__default__fired(void )
#line 185
{
#line 185
  HplMsp430InterruptP__Port23__clear();
}

# 72 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port23__fired(void ){
#line 72
  HplMsp430InterruptP__Port23__default__fired();
#line 72
}
#line 72
# 210 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port24__clear(void )
#line 210
{
#line 210
  P2IFG &= ~(1 << 4);
}

#line 186
static inline void HplMsp430InterruptP__Port24__default__fired(void )
#line 186
{
#line 186
  HplMsp430InterruptP__Port24__clear();
}

# 72 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port24__fired(void ){
#line 72
  HplMsp430InterruptP__Port24__default__fired();
#line 72
}
#line 72
# 211 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port25__clear(void )
#line 211
{
#line 211
  P2IFG &= ~(1 << 5);
}

#line 187
static inline void HplMsp430InterruptP__Port25__default__fired(void )
#line 187
{
#line 187
  HplMsp430InterruptP__Port25__clear();
}

# 72 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port25__fired(void ){
#line 72
  HplMsp430InterruptP__Port25__default__fired();
#line 72
}
#line 72
# 212 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port26__clear(void )
#line 212
{
#line 212
  P2IFG &= ~(1 << 6);
}

#line 188
static inline void HplMsp430InterruptP__Port26__default__fired(void )
#line 188
{
#line 188
  HplMsp430InterruptP__Port26__clear();
}

# 72 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port26__fired(void ){
#line 72
  HplMsp430InterruptP__Port26__default__fired();
#line 72
}
#line 72
# 213 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port27__clear(void )
#line 213
{
#line 213
  P2IFG &= ~(1 << 7);
}

#line 189
static inline void HplMsp430InterruptP__Port27__default__fired(void )
#line 189
{
#line 189
  HplMsp430InterruptP__Port27__clear();
}

# 72 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port27__fired(void ){
#line 72
  HplMsp430InterruptP__Port27__default__fired();
#line 72
}
#line 72
# 98 "/home/csilab/tinyos/tos/interfaces/ArbiterInfo.nc"
inline static uint8_t /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__1__ArbiterInfo__userId(void ){
#line 98
  unsigned char __nesc_result;
#line 98

#line 98
  __nesc_result = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ArbiterInfo__userId();
#line 98

#line 98
  return __nesc_result;
#line 98
}
#line 98
# 349 "/home/csilab/tinyos/tos/chips/msp430/usart/HplMsp430Usart0P.nc"
static inline void HplMsp430Usart0P__Usart__disableRxIntr(void )
#line 349
{
  HplMsp430Usart0P__IE1 &= ~0x40;
}

# 177 "/home/csilab/tinyos/tos/chips/msp430/usart/HplMsp430Usart.nc"
inline static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__disableRxIntr(void ){
#line 177
  HplMsp430Usart0P__Usart__disableRxIntr();
#line 177
}
#line 177
# 231 "/home/csilab/tinyos/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartInterrupts__rxDone(uint8_t data)
#line 231
{

  if (/*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_rx_buf) {
    /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_rx_buf[/*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_pos - 1] = data;
    }
  if (/*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_pos < /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_len) {
    /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__continueOp();
    }
  else 
#line 238
    {
      /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__disableRxIntr();
      /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__signalDone();
    }
}

# 65 "/home/csilab/tinyos/tos/chips/msp430/usart/Msp430UsartShareP.nc"
static inline void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__1__Interrupts__default__rxDone(uint8_t id, uint8_t data)
#line 65
{
}

# 54 "/home/csilab/tinyos/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
inline static void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__1__Interrupts__rxDone(uint8_t arg_0x7f7fed2b0600, uint8_t data){
#line 54
  switch (arg_0x7f7fed2b0600) {
#line 54
    case /*CC2420SpiWireC.HplCC2420SpiC.SpiC.UsartC*/Msp430Usart0C__0__CLIENT_ID:
#line 54
      /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartInterrupts__rxDone(data);
#line 54
      break;
#line 54
    default:
#line 54
      /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__1__Interrupts__default__rxDone(arg_0x7f7fed2b0600, data);
#line 54
      break;
#line 54
    }
#line 54
}
#line 54
# 90 "/home/csilab/tinyos/tos/interfaces/ArbiterInfo.nc"
inline static bool /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__1__ArbiterInfo__inUse(void ){
#line 90
  unsigned char __nesc_result;
#line 90

#line 90
  __nesc_result = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ArbiterInfo__inUse();
#line 90

#line 90
  return __nesc_result;
#line 90
}
#line 90
# 54 "/home/csilab/tinyos/tos/chips/msp430/usart/Msp430UsartShareP.nc"
static inline void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__1__RawInterrupts__rxDone(uint8_t data)
#line 54
{
  if (/*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__1__ArbiterInfo__inUse()) {
    /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__1__Interrupts__rxDone(/*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__1__ArbiterInfo__userId(), data);
    }
}

# 54 "/home/csilab/tinyos/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
inline static void HplMsp430Usart0P__Interrupts__rxDone(uint8_t data){
#line 54
  /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__1__RawInterrupts__rxDone(data);
#line 54
}
#line 54
# 55 "/home/csilab/tinyos/tos/chips/msp430/usart/HplMsp430I2C0P.nc"
static inline bool HplMsp430I2C0P__HplI2C__isI2C(void )
#line 55
{
  /* atomic removed: atomic calls only */
#line 56
  {
    unsigned char __nesc_temp = 
#line 56
    HplMsp430I2C0P__U0CTL & 0x20 && HplMsp430I2C0P__U0CTL & 0x04 && HplMsp430I2C0P__U0CTL & 0x01;

#line 56
    return __nesc_temp;
  }
}

# 6 "/home/csilab/tinyos/tos/chips/msp430/usart/HplMsp430I2C.nc"
inline static bool HplMsp430Usart0P__HplI2C__isI2C(void ){
#line 6
  unsigned char __nesc_result;
#line 6

#line 6
  __nesc_result = HplMsp430I2C0P__HplI2C__isI2C();
#line 6

#line 6
  return __nesc_result;
#line 6
}
#line 6
# 66 "/home/csilab/tinyos/tos/chips/msp430/usart/Msp430UsartShareP.nc"
static inline void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__1__I2CInterrupts__default__fired(uint8_t id)
#line 66
{
}

# 39 "/home/csilab/tinyos/tos/chips/msp430/usart/HplMsp430I2CInterrupts.nc"
inline static void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__1__I2CInterrupts__fired(uint8_t arg_0x7f7fed2ae1c0){
#line 39
    /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__1__I2CInterrupts__default__fired(arg_0x7f7fed2ae1c0);
#line 39
}
#line 39
# 59 "/home/csilab/tinyos/tos/chips/msp430/usart/Msp430UsartShareP.nc"
static inline void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__1__RawI2CInterrupts__fired(void )
#line 59
{
  if (/*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__1__ArbiterInfo__inUse()) {
    /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__1__I2CInterrupts__fired(/*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__1__ArbiterInfo__userId());
    }
}

# 39 "/home/csilab/tinyos/tos/chips/msp430/usart/HplMsp430I2CInterrupts.nc"
inline static void HplMsp430Usart0P__I2CInterrupts__fired(void ){
#line 39
  /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__1__RawI2CInterrupts__fired();
#line 39
}
#line 39
# 249 "/home/csilab/tinyos/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartInterrupts__txDone(void )
#line 249
{
}

# 64 "/home/csilab/tinyos/tos/chips/msp430/usart/Msp430UsartShareP.nc"
static inline void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__1__Interrupts__default__txDone(uint8_t id)
#line 64
{
}

# 49 "/home/csilab/tinyos/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
inline static void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__1__Interrupts__txDone(uint8_t arg_0x7f7fed2b0600){
#line 49
  switch (arg_0x7f7fed2b0600) {
#line 49
    case /*CC2420SpiWireC.HplCC2420SpiC.SpiC.UsartC*/Msp430Usart0C__0__CLIENT_ID:
#line 49
      /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartInterrupts__txDone();
#line 49
      break;
#line 49
    default:
#line 49
      /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__1__Interrupts__default__txDone(arg_0x7f7fed2b0600);
#line 49
      break;
#line 49
    }
#line 49
}
#line 49
# 49 "/home/csilab/tinyos/tos/chips/msp430/usart/Msp430UsartShareP.nc"
static inline void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__1__RawInterrupts__txDone(void )
#line 49
{
  if (/*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__1__ArbiterInfo__inUse()) {
    /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__1__Interrupts__txDone(/*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__1__ArbiterInfo__userId());
    }
}

# 49 "/home/csilab/tinyos/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
inline static void HplMsp430Usart0P__Interrupts__txDone(void ){
#line 49
  /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__1__RawInterrupts__txDone();
#line 49
}
#line 49
# 411 "/home/csilab/tinyos/tos/chips/msp430/msp430hardware.h"
  __nesc_atomic_t __nesc_atomic_start(void )
{
  __nesc_atomic_t result = (__read_status_register() & 0x0008) != 0;

#line 414
  __nesc_disable_interrupt();
   __asm volatile ("" :  :  : "memory");
  return result;
}

  void __nesc_atomic_end(__nesc_atomic_t reenable_interrupts)
{
   __asm volatile ("" :  :  : "memory");
  if (reenable_interrupts) {
    __nesc_enable_interrupt();
    }
}

# 96 "/home/csilab/tinyos/tos/chips/msp430/usart/HplMsp430Usart1P.nc"
__attribute((wakeup)) __attribute((interrupt(0x0006)))  void sig_UART1RX_VECTOR(void )
#line 96
{
  uint8_t temp = U1RXBUF;

#line 98
  HplMsp430Usart1P__Interrupts__rxDone(temp);
}

# 153 "/home/csilab/tinyos/tos/system/ArbiterP.nc"
static bool /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ArbiterInfo__inUse(void )
#line 153
{
  /* atomic removed: atomic calls only */
#line 154
  {
    if (/*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__state == /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__RES_CONTROLLED) 
      {
        unsigned char __nesc_temp = 
#line 156
        FALSE;

#line 156
        return __nesc_temp;
      }
  }
#line 158
  return TRUE;
}

# 412 "/home/csilab/tinyos/tos/lib/serial/SerialP.nc"
static void SerialP__rx_state_machine(bool isDelimeter, uint8_t data)
#line 412
{

  switch (SerialP__rxState) {

      case SerialP__RXSTATE_NOSYNC: 
        if (isDelimeter) {
            SerialP__rxInit();
            SerialP__rxState = SerialP__RXSTATE_PROTO;
          }
      break;

      case SerialP__RXSTATE_PROTO: 
        if (!isDelimeter) {
            SerialP__rxCRC = crcByte(SerialP__rxCRC, data);
            SerialP__rxState = SerialP__RXSTATE_TOKEN;
            SerialP__rxProto = data;
            if (!SerialP__valid_rx_proto(SerialP__rxProto)) {
              goto nosync;
              }
            if (SerialP__rxProto != SERIAL_PROTO_PACKET_ACK) {
                goto nosync;
              }
            if (SerialP__ReceiveBytePacket__startPacket() != SUCCESS) {
                goto nosync;
              }
          }
      break;

      case SerialP__RXSTATE_TOKEN: 
        if (isDelimeter) {
            goto nosync;
          }
        else {
            SerialP__rxSeqno = data;
            SerialP__rxCRC = crcByte(SerialP__rxCRC, SerialP__rxSeqno);
            SerialP__rxState = SerialP__RXSTATE_INFO;
          }
      break;

      case SerialP__RXSTATE_INFO: 
        if (SerialP__rxByteCnt < SerialP__SERIAL_MTU) {
            if (isDelimeter) {
                if (SerialP__rxByteCnt >= 2) {
                    if (SerialP__rx_current_crc() == SerialP__rxCRC) {
                        SerialP__ReceiveBytePacket__endPacket(SUCCESS);
                        SerialP__ack_queue_push(SerialP__rxSeqno);
                        SerialP__rxInit();
                        SerialP__SerialFrameComm__resetReceive();
                        if (SerialP__offPending) {
                            SerialP__rxState = SerialP__RXSTATE_INACTIVE;
                            SerialP__testOff();
                          }
                        goto done;
                      }
                    else {
                        goto nosync;
                      }
                  }
                else {
                    goto nosync;
                  }
              }
            else {
                if (SerialP__rxByteCnt >= 2) {
                    SerialP__ReceiveBytePacket__byteReceived(SerialP__rx_buffer_top());
                    SerialP__rxCRC = crcByte(SerialP__rxCRC, SerialP__rx_buffer_pop());
                  }
                SerialP__rx_buffer_push(data);
                SerialP__rxByteCnt++;
              }
          }
        else 

          {
            goto nosync;
          }
      break;

      default: 
        goto nosync;
    }
  goto done;

  nosync: 

    SerialP__rxInit();
  SerialP__SerialFrameComm__resetReceive();
  SerialP__ReceiveBytePacket__endPacket(FAIL);
  if (SerialP__offPending) {
      SerialP__rxState = SerialP__RXSTATE_INACTIVE;
      SerialP__testOff();
    }
  else {
    if (isDelimeter) {
        SerialP__rxState = SerialP__RXSTATE_PROTO;
      }
    }
  done: ;
}

# 91 "/home/csilab/tinyos/tos/system/crc.h"
static uint16_t crcByte(uint16_t crc, uint8_t b)
#line 91
{
  crc = (uint8_t )(crc >> 8) | (crc << 8);
  crc ^= b;
  crc ^= (uint8_t )(crc & 0xff) >> 4;
  crc ^= crc << 12;
  crc ^= (crc & 0xff) << 5;
  return crc;
}

# 296 "/home/csilab/tinyos/tos/lib/serial/SerialDispatcherP.nc"
static void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__ReceiveBytePacket__endPacket(error_t result)
#line 296
{
  uint8_t postsignalreceive = FALSE;

  /* atomic removed: atomic calls only */
#line 298
  {
    if (!/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTaskPending && result == SUCCESS) {
        postsignalreceive = TRUE;
        /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTaskPending = TRUE;
        /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTaskType = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__recvType;
        /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTaskWhich = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveState.which;
        /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTaskBuf = (message_t *)/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveBuffer;
        /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTaskSize = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__recvIndex;
        /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveBufferSwap();
        /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveState.state = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__RECV_STATE_IDLE;
      }
    else 
#line 308
      {

        /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__unlockBuffer(/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveState.which);
      }
  }
  if (postsignalreceive) {
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTask__postTask();
    }
}

# 170 "/home/csilab/tinyos/tos/system/SchedulerBasicP.nc"
static error_t SchedulerBasicP__TaskBasic__postTask(uint8_t id)
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 172
    {
#line 172
      {
        unsigned char __nesc_temp = 
#line 172
        SchedulerBasicP__pushTask(id) ? SUCCESS : EBUSY;

        {
#line 172
          __nesc_atomic_end(__nesc_atomic); 
#line 172
          return __nesc_temp;
        }
      }
    }
#line 175
    __nesc_atomic_end(__nesc_atomic); }
}

# 518 "/home/csilab/tinyos/tos/lib/serial/SerialP.nc"
static void SerialP__MaybeScheduleTx(void )
#line 518
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 519
    {
      if (SerialP__txPending == 0) {
          if (SerialP__RunTx__postTask() == SUCCESS) {
              SerialP__txPending = 1;
            }
        }
    }
#line 525
    __nesc_atomic_end(__nesc_atomic); }
}

#line 357
static void SerialP__testOff(void )
#line 357
{
  bool turnOff = FALSE;

  /* atomic removed: atomic calls only */
#line 359
  {
    if (SerialP__txState == SerialP__TXSTATE_INACTIVE && 
    SerialP__rxState == SerialP__RXSTATE_INACTIVE) {
        turnOff = TRUE;
      }
  }
  if (turnOff) {
      SerialP__stopDoneTask__postTask();
      /* atomic removed: atomic calls only */
#line 367
      SerialP__offPending = FALSE;
    }
  else {
      /* atomic removed: atomic calls only */
#line 370
      SerialP__offPending = TRUE;
    }
}

# 166 "/home/csilab/tinyos/tos/system/ArbiterP.nc"
static uint8_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ArbiterInfo__userId(void )
#line 166
{
  /* atomic removed: atomic calls only */
#line 167
  {
    if (/*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__state != /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__RES_BUSY) 
      {
        unsigned char __nesc_temp = 
#line 169
        /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__NO_RES;

#line 169
        return __nesc_temp;
      }
#line 170
    {
      unsigned char __nesc_temp = 
#line 170
      /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__resId;

#line 170
      return __nesc_temp;
    }
  }
}

# 101 "/home/csilab/tinyos/tos/chips/msp430/usart/HplMsp430Usart1P.nc"
__attribute((wakeup)) __attribute((interrupt(0x0004)))  void sig_UART1TX_VECTOR(void )
#line 101
{
  HplMsp430Usart1P__Interrupts__txDone();
}

# 118 "/home/csilab/tinyos/tos/lib/serial/HdlcTranslateC.nc"
static void HdlcTranslateC__UartStream__sendDone(uint8_t *buf, uint16_t len, 
error_t error)
#line 119
{
  /* atomic removed: atomic calls only */
#line 120
  {
    if (HdlcTranslateC__state.sendEscape) {
        HdlcTranslateC__state.sendEscape = 0;
        HdlcTranslateC__m_data = HdlcTranslateC__txTemp;
        HdlcTranslateC__UartStream__send(&HdlcTranslateC__m_data, 1);
      }
    else {
        HdlcTranslateC__SerialFrameComm__putDone();
      }
  }
}

# 147 "/home/csilab/tinyos/tos/chips/msp430/usart/Msp430UartP.nc"
static error_t /*Msp430Uart1P.UartP*/Msp430UartP__0__UartStream__send(uint8_t id, uint8_t *buf, uint16_t len)
#line 147
{
  if (/*Msp430Uart1P.UartP*/Msp430UartP__0__UsartResource__isOwner(id) == FALSE) {
    return FAIL;
    }
#line 150
  if (len == 0) {
    return FAIL;
    }
  else {
#line 152
    if (/*Msp430Uart1P.UartP*/Msp430UartP__0__m_tx_buf) {
      return EBUSY;
      }
    }
#line 154
  /*Msp430Uart1P.UartP*/Msp430UartP__0__m_tx_buf = buf;
  /*Msp430Uart1P.UartP*/Msp430UartP__0__m_tx_len = len;
  /*Msp430Uart1P.UartP*/Msp430UartP__0__m_tx_pos = 0;
  /*Msp430Uart1P.UartP*/Msp430UartP__0__current_owner = id;
  /*Msp430Uart1P.UartP*/Msp430UartP__0__Usart__tx(buf[/*Msp430Uart1P.UartP*/Msp430UartP__0__m_tx_pos++]);
  return SUCCESS;
}

# 177 "/home/csilab/tinyos/tos/system/ArbiterP.nc"
static bool /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__Resource__isOwner(uint8_t id)
#line 177
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 178
    {
      if (/*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__resId == id && /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__state == /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__RES_BUSY) {
          unsigned char __nesc_temp = 
#line 179
          TRUE;

          {
#line 179
            __nesc_atomic_end(__nesc_atomic); 
#line 179
            return __nesc_temp;
          }
        }
      else 
#line 180
        {
          unsigned char __nesc_temp = 
#line 180
          FALSE;

          {
#line 180
            __nesc_atomic_end(__nesc_atomic); 
#line 180
            return __nesc_temp;
          }
        }
    }
#line 183
    __nesc_atomic_end(__nesc_atomic); }
}

# 106 "/home/csilab/tinyos/tos/lib/serial/HdlcTranslateC.nc"
static error_t HdlcTranslateC__SerialFrameComm__putData(uint8_t data)
#line 106
{
  if (data == HDLC_CTLESC_BYTE || data == HDLC_FLAG_BYTE) {
      HdlcTranslateC__state.sendEscape = 1;
      HdlcTranslateC__txTemp = data ^ 0x20;
      HdlcTranslateC__m_data = HDLC_CTLESC_BYTE;
    }
  else {
      HdlcTranslateC__m_data = data;
    }
  return HdlcTranslateC__UartStream__send(&HdlcTranslateC__m_data, 1);
}

#line 98
static error_t HdlcTranslateC__SerialFrameComm__putDelimiter(void )
#line 98
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 99
    {
      HdlcTranslateC__state.sendEscape = 0;
      HdlcTranslateC__m_data = HDLC_FLAG_BYTE;
    }
#line 102
    __nesc_atomic_end(__nesc_atomic); }
  return HdlcTranslateC__UartStream__send(&HdlcTranslateC__m_data, 1);
}

# 11 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430TimerCommonP.nc"
__attribute((wakeup)) __attribute((interrupt(0x000C)))  void sig_TIMERA0_VECTOR(void )
#line 11
{
#line 11
  Msp430TimerCommonP__VectorTimerA0__fired();
}

# 180 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Event__fired(void )
{
  if (/*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Control__getControl().cap) {
    /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__captured(/*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__getEvent());
    }
  else {
#line 185
    /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Compare__fired();
    }
}

#line 180
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Event__fired(void )
{
  if (/*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Control__getControl().cap) {
    /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__captured(/*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__getEvent());
    }
  else {
#line 185
    /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Compare__fired();
    }
}

#line 180
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Event__fired(void )
{
  if (/*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Control__getControl().cap) {
    /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__captured(/*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__getEvent());
    }
  else {
#line 185
    /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Compare__fired();
    }
}

# 12 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430TimerCommonP.nc"
__attribute((wakeup)) __attribute((interrupt(0x000A)))  void sig_TIMERA1_VECTOR(void )
#line 12
{
#line 12
  Msp430TimerCommonP__VectorTimerA1__fired();
}

#line 13
__attribute((wakeup)) __attribute((interrupt(0x001A)))  void sig_TIMERB0_VECTOR(void )
#line 13
{
#line 13
  Msp430TimerCommonP__VectorTimerB0__fired();
}

# 146 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430TimerP.nc"
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__default__fired(uint8_t n)
{
}

# 39 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__fired(uint8_t arg_0x7f7fed4c58b0){
#line 39
  switch (arg_0x7f7fed4c58b0) {
#line 39
    case 0:
#line 39
      /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Event__fired();
#line 39
      break;
#line 39
    case 1:
#line 39
      /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Event__fired();
#line 39
      break;
#line 39
    case 2:
#line 39
      /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Event__fired();
#line 39
      break;
#line 39
    case 3:
#line 39
      /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Event__fired();
#line 39
      break;
#line 39
    case 4:
#line 39
      /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Event__fired();
#line 39
      break;
#line 39
    case 5:
#line 39
      /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Event__fired();
#line 39
      break;
#line 39
    case 6:
#line 39
      /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Event__fired();
#line 39
      break;
#line 39
    case 7:
#line 39
      /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Overflow__fired();
#line 39
      break;
#line 39
    default:
#line 39
      /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__default__fired(arg_0x7f7fed4c58b0);
#line 39
      break;
#line 39
    }
#line 39
}
#line 39
# 147 "/home/csilab/tinyos/tos/lib/timer/TransformAlarmC.nc"
static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Alarm__startAt(/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__to_size_type t0, /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__to_size_type dt)
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__m_t0 = t0;
      /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__m_dt = dt;
      /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__set_alarm();
    }
#line 154
    __nesc_atomic_end(__nesc_atomic); }
}

#line 107
static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__set_alarm(void )
{
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__to_size_type now = /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Counter__get();
#line 109
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__to_size_type expires;
#line 109
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__to_size_type remaining;




  expires = /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__m_t0 + /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__m_dt;


  remaining = (/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__to_size_type )(expires - now);


  if (/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__m_t0 <= now) 
    {
      if (expires >= /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__m_t0 && 
      expires <= now) {
        remaining = 0;
        }
    }
  else {
      if (expires >= /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__m_t0 || 
      expires <= now) {
        remaining = 0;
        }
    }
#line 132
  if (remaining > /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__MAX_DELAY) 
    {
      /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__m_t0 = now + /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__MAX_DELAY;
      /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__m_dt = remaining - /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__MAX_DELAY;
      remaining = /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__MAX_DELAY;
    }
  else 
    {
      /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__m_t0 += /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__m_dt;
      /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__m_dt = 0;
    }
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__AlarmFrom__startAt((/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__from_size_type )now << 0, 
  (/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__from_size_type )remaining << 0);
}

# 80 "/home/csilab/tinyos/tos/lib/timer/TransformCounterC.nc"
static /*Counter32khz32C.Transform*/TransformCounterC__0__to_size_type /*Counter32khz32C.Transform*/TransformCounterC__0__Counter__get(void )
{
  /*Counter32khz32C.Transform*/TransformCounterC__0__to_size_type rv = 0;

#line 83
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      /*Counter32khz32C.Transform*/TransformCounterC__0__upper_count_type high = /*Counter32khz32C.Transform*/TransformCounterC__0__m_upper;
      /*Counter32khz32C.Transform*/TransformCounterC__0__from_size_type low = /*Counter32khz32C.Transform*/TransformCounterC__0__CounterFrom__get();

#line 87
      if (/*Counter32khz32C.Transform*/TransformCounterC__0__CounterFrom__isOverflowPending()) 
        {






          high++;
          low = /*Counter32khz32C.Transform*/TransformCounterC__0__CounterFrom__get();
        }
      {
        /*Counter32khz32C.Transform*/TransformCounterC__0__to_size_type high_to = high;
        /*Counter32khz32C.Transform*/TransformCounterC__0__to_size_type low_to = low >> /*Counter32khz32C.Transform*/TransformCounterC__0__LOW_SHIFT_RIGHT;

#line 101
        rv = (high_to << /*Counter32khz32C.Transform*/TransformCounterC__0__HIGH_SHIFT_LEFT) | low_to;
      }
    }
#line 103
    __nesc_atomic_end(__nesc_atomic); }
  return rv;
}

# 62 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430TimerP.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__get(void )
{




  if (1) {
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 69
        {
          uint16_t t0;
          uint16_t t1 = * (volatile uint16_t * )400U;

#line 72
          do {
#line 72
              t0 = t1;
#line 72
              t1 = * (volatile uint16_t * )400U;
            }
          while (
#line 72
          t0 != t1);
          {
            unsigned int __nesc_temp = 
#line 73
            t1;

            {
#line 73
              __nesc_atomic_end(__nesc_atomic); 
#line 73
              return __nesc_temp;
            }
          }
        }
#line 76
        __nesc_atomic_end(__nesc_atomic); }
    }
  else 
#line 76
    {
      return * (volatile uint16_t * )400U;
    }
}

# 664 "/home/csilab/upma/chips/cc2420/CC2420TransmitP.nc"
static void CC2420TransmitP__congestionBackoff(void )
#line 664
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 665
    {
      CC2420TransmitP__requestCongestionBackoff(CC2420TransmitP__m_msg);
      CC2420TransmitP__BackoffTimer__start(CC2420TransmitP__myCongestionBackoff);
    }
#line 668
    __nesc_atomic_end(__nesc_atomic); }
}

# 69 "/home/csilab/tinyos/tos/system/RandomMlcgC.nc"
static uint32_t RandomMlcgC__Random__rand32(void )
#line 69
{
  uint32_t mlcg;
#line 70
  uint32_t p;
#line 70
  uint32_t q;
  uint64_t tmpseed;

  /* atomic removed: atomic calls only */
#line 73
  {
    tmpseed = (uint64_t )33614U * (uint64_t )RandomMlcgC__seed;
    q = tmpseed;
    q = q >> 1;
    p = tmpseed >> 32;
    mlcg = p + q;
    if (mlcg & 0x80000000) {
        mlcg = mlcg & 0x7FFFFFFF;
        mlcg++;
      }
    RandomMlcgC__seed = mlcg;
  }
  return mlcg;
}

# 671 "/home/csilab/upma/chips/cc2420/CC2420TransmitP.nc"
static error_t CC2420TransmitP__acquireSpiResource(void )
#line 671
{
  error_t error = CC2420TransmitP__SpiResource__immediateRequest();

#line 673
  if (error != SUCCESS) {
      CC2420TransmitP__SpiResource__request();
    }
  return error;
}

# 126 "/home/csilab/tinyos/tos/chips/cc2420/spi/CC2420SpiP.nc"
static error_t CC2420SpiP__Resource__immediateRequest(uint8_t id)
#line 126
{
  error_t error;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 129
    {
      if (CC2420SpiP__WorkingState__requestState(CC2420SpiP__S_BUSY) != SUCCESS) {
          {
            unsigned char __nesc_temp = 
#line 131
            EBUSY;

            {
#line 131
              __nesc_atomic_end(__nesc_atomic); 
#line 131
              return __nesc_temp;
            }
          }
        }
      if (CC2420SpiP__SpiResource__isOwner()) {
          CC2420SpiP__m_holder = id;
          error = SUCCESS;
        }
      else {
#line 139
        if ((error = CC2420SpiP__SpiResource__immediateRequest()) == SUCCESS) {
            CC2420SpiP__m_holder = id;
          }
        else {
            CC2420SpiP__WorkingState__toIdle();
          }
        }
    }
#line 146
    __nesc_atomic_end(__nesc_atomic); }
#line 146
  return error;
}

# 96 "/home/csilab/tinyos/tos/system/StateImplP.nc"
static error_t StateImplP__State__requestState(uint8_t id, uint8_t reqState)
#line 96
{
  error_t returnVal = FAIL;

#line 98
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 98
    {
      if (reqState == StateImplP__S_IDLE || StateImplP__state[id] == StateImplP__S_IDLE) {
          StateImplP__state[id] = reqState;
          returnVal = SUCCESS;
        }
    }
#line 103
    __nesc_atomic_end(__nesc_atomic); }
  return returnVal;
}

# 177 "/home/csilab/tinyos/tos/system/ArbiterP.nc"
static bool /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__Resource__isOwner(uint8_t id)
#line 177
{
  /* atomic removed: atomic calls only */
#line 178
  {
    if (/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__resId == id && /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__state == /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__RES_BUSY) {
        unsigned char __nesc_temp = 
#line 179
        TRUE;

#line 179
        return __nesc_temp;
      }
    else 
#line 180
      {
        unsigned char __nesc_temp = 
#line 180
        FALSE;

#line 180
        return __nesc_temp;
      }
  }
}

#line 133
static error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceDefaultOwner__release(void )
#line 133
{
  /* atomic removed: atomic calls only */
#line 134
  {
    if (/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__resId == /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__default_owner_id) {
        if (/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__state == /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__RES_GRANTING) {
            /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__grantedTask__postTask();
            {
              unsigned char __nesc_temp = 
#line 138
              SUCCESS;

#line 138
              return __nesc_temp;
            }
          }
        else {
#line 140
          if (/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__state == /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__RES_IMM_GRANTING) {
              /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__resId = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__reqResId;
              /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__state = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__RES_BUSY;
              {
                unsigned char __nesc_temp = 
#line 143
                SUCCESS;

#line 143
                return __nesc_temp;
              }
            }
          }
      }
  }
#line 147
  return FAIL;
}

# 265 "/home/csilab/tinyos/tos/chips/msp430/usart/HplMsp430Usart0P.nc"
static void HplMsp430Usart0P__Usart__setModeSpi(msp430_spi_union_config_t *config)
#line 265
{

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 267
    {
      HplMsp430Usart0P__Usart__resetUsart(TRUE);
      HplMsp430Usart0P__HplI2C__clearModeI2C();
      HplMsp430Usart0P__Usart__disableUart();
      HplMsp430Usart0P__configSpi(config);
      HplMsp430Usart0P__Usart__enableSpi();
      HplMsp430Usart0P__Usart__resetUsart(FALSE);
      HplMsp430Usart0P__Usart__clrIntr();
      HplMsp430Usart0P__Usart__disableIntr();
    }
#line 276
    __nesc_atomic_end(__nesc_atomic); }
  return;
}

# 107 "/home/csilab/tinyos/tos/chips/cc2420/spi/CC2420SpiP.nc"
static error_t CC2420SpiP__Resource__request(uint8_t id)
#line 107
{

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 109
    {
      if (CC2420SpiP__WorkingState__requestState(CC2420SpiP__S_BUSY) == SUCCESS) {
          CC2420SpiP__m_holder = id;
          if (CC2420SpiP__SpiResource__isOwner()) {
              CC2420SpiP__grant__postTask();
            }
          else {
              CC2420SpiP__SpiResource__request();
            }
        }
      else {
          CC2420SpiP__m_requests |= 1 << id;
        }
    }
#line 122
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

# 615 "/home/csilab/upma/chips/cc2420/CC2420TransmitP.nc"
static void CC2420TransmitP__attemptSend(void )
#line 615
{
  uint8_t status;
  bool congestion = TRUE;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 619
    {
      if (CC2420TransmitP__m_state == CC2420TransmitP__S_CANCEL) {
          CC2420TransmitP__SFLUSHTX__strobe();
          CC2420TransmitP__releaseSpiResource();
          CC2420TransmitP__CSN__set();
          CC2420TransmitP__m_state = CC2420TransmitP__S_STARTED;
          CC2420TransmitP__Send__sendDone(CC2420TransmitP__m_msg, ECANCEL);
          {
#line 626
            __nesc_atomic_end(__nesc_atomic); 
#line 626
            return;
          }
        }

      CC2420TransmitP__CSN__clr();

      status = CC2420TransmitP__m_cca ? CC2420TransmitP__STXONCCA__strobe() : CC2420TransmitP__STXON__strobe();
      if (!(status & CC2420_STATUS_TX_ACTIVE)) {
          status = CC2420TransmitP__SNOP__strobe();
          if (status & CC2420_STATUS_TX_ACTIVE) {
              congestion = FALSE;
            }
        }

      CC2420TransmitP__m_state = congestion ? CC2420TransmitP__S_SAMPLE_CCA : CC2420TransmitP__S_SFD;
      CC2420TransmitP__CSN__set();
    }
#line 642
    __nesc_atomic_end(__nesc_atomic); }

  if (congestion) {
      CC2420TransmitP__totalCcaChecks = 0;
      CC2420TransmitP__releaseSpiResource();
      CC2420TransmitP__congestionBackoff();
    }
  else 
#line 648
    {
      CC2420TransmitP__BackoffTimer__start(CC2420TransmitP__CC2420_ABORT_PERIOD);
    }
}

# 318 "/home/csilab/tinyos/tos/chips/cc2420/spi/CC2420SpiP.nc"
static cc2420_status_t CC2420SpiP__Strobe__strobe(uint8_t addr)
#line 318
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 319
    {
      if (CC2420SpiP__WorkingState__isIdle()) {
          {
            unsigned char __nesc_temp = 
#line 321
            0;

            {
#line 321
              __nesc_atomic_end(__nesc_atomic); 
#line 321
              return __nesc_temp;
            }
          }
        }
    }
#line 325
    __nesc_atomic_end(__nesc_atomic); }
#line 325
  return CC2420SpiP__SpiByte__write(addr);
}

# 133 "/home/csilab/tinyos/tos/system/StateImplP.nc"
static bool StateImplP__State__isState(uint8_t id, uint8_t myState)
#line 133
{
  bool isState;

#line 135
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 135
    isState = StateImplP__state[id] == myState;
#line 135
    __nesc_atomic_end(__nesc_atomic); }
  return isState;
}

# 134 "/home/csilab/tinyos/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static uint8_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__SpiByte__write(uint8_t tx)
#line 134
{
  uint8_t byte;


  /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__tx(tx);
  while (!/*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__isRxIntrPending()) ;
  /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__clrRxIntr();
  byte = /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__rx();

  return byte;
}

# 149 "/home/csilab/tinyos/tos/chips/cc2420/spi/CC2420SpiP.nc"
static error_t CC2420SpiP__Resource__release(uint8_t id)
#line 149
{
  uint8_t i;

#line 151
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 151
    {
      if (CC2420SpiP__m_holder != id) {
          {
            unsigned char __nesc_temp = 
#line 153
            FAIL;

            {
#line 153
              __nesc_atomic_end(__nesc_atomic); 
#line 153
              return __nesc_temp;
            }
          }
        }
#line 156
      CC2420SpiP__m_holder = CC2420SpiP__NO_HOLDER;
      if (!CC2420SpiP__m_requests) {
          CC2420SpiP__WorkingState__toIdle();
          CC2420SpiP__attemptRelease();
        }
      else {
          for (i = CC2420SpiP__m_holder + 1; ; i++) {
              i %= CC2420SpiP__RESOURCE_COUNT;

              if (CC2420SpiP__m_requests & (1 << i)) {
                  CC2420SpiP__m_holder = i;
                  CC2420SpiP__m_requests &= ~(1 << i);
                  CC2420SpiP__grant__postTask();
                  {
                    unsigned char __nesc_temp = 
#line 169
                    SUCCESS;

                    {
#line 169
                      __nesc_atomic_end(__nesc_atomic); 
#line 169
                      return __nesc_temp;
                    }
                  }
                }
            }
        }
    }
#line 175
    __nesc_atomic_end(__nesc_atomic); }
#line 175
  return SUCCESS;
}

#line 339
static error_t CC2420SpiP__attemptRelease(void )
#line 339
{


  if ((
#line 340
  CC2420SpiP__m_requests > 0
   || CC2420SpiP__m_holder != CC2420SpiP__NO_HOLDER)
   || !CC2420SpiP__WorkingState__isIdle()) {
      return FAIL;
    }
  /* atomic removed: atomic calls only */
  CC2420SpiP__release = TRUE;
  CC2420SpiP__ChipSpiResource__releasing();
  /* atomic removed: atomic calls only */
#line 348
  {
    if (CC2420SpiP__release) {
        CC2420SpiP__SpiResource__release();
        {
          unsigned char __nesc_temp = 
#line 351
          SUCCESS;

#line 351
          return __nesc_temp;
        }
      }
  }
  return EBUSY;
}

# 247 "/home/csilab/tinyos/tos/chips/msp430/usart/HplMsp430Usart0P.nc"
static void HplMsp430Usart0P__Usart__disableSpi(void )
#line 247
{
  /* atomic removed: atomic calls only */
#line 248
  {
    HplMsp430Usart0P__ME1 &= ~0x40;
    HplMsp430Usart0P__SIMO__selectIOFunc();
    HplMsp430Usart0P__SOMI__selectIOFunc();
    HplMsp430Usart0P__UCLK__selectIOFunc();
  }
}

# 56 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static void /*HplMsp430GeneralIOC.P42*/HplMsp430GeneralIOP__26__IO__set(void )
#line 56
{
#line 56
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 56
    * (volatile uint8_t * )29U |= 0x01 << 2;
#line 56
    __nesc_atomic_end(__nesc_atomic); }
}

# 215 "/home/csilab/upma/chips/cc2420/CC2420CsmaP.nc"
static void CC2420CsmaP__sendDone(void )
#line 215
{
  error_t packetErr;
  message_t *packet;

#line 218
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 218
    {
      packetErr = CC2420CsmaP__sendErr;
      packet = CC2420CsmaP__m_msg;
    }
#line 221
    __nesc_atomic_end(__nesc_atomic); }
  if (CC2420CsmaP__SplitControlState__isState(CC2420CsmaP__S_STOPPING)) {
      CC2420CsmaP__shutdown();
    }
  else {
      CC2420CsmaP__SplitControlState__forceState(CC2420CsmaP__S_STARTED);
    }

  CC2420CsmaP__Send__sendDone(packet, packetErr);
}

# 465 "/home/csilab/upma/chips/cc2420/CC2420ReceiveP.nc"
static void CC2420ReceiveP__reset_state(void )
#line 465
{
  CC2420ReceiveP__m_bytes_left = CC2420ReceiveP__RXFIFO_SIZE;
  /* atomic removed: atomic calls only */
#line 467
  CC2420ReceiveP__receivingPacket = FALSE;
  CC2420ReceiveP__m_timestamp_head = 0;
  CC2420ReceiveP__m_timestamp_size = 0;
  CC2420ReceiveP__m_missed_packets = 0;
}

# 57 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static void /*HplMsp430GeneralIOC.P46*/HplMsp430GeneralIOP__30__IO__clr(void )
#line 57
{
#line 57
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 57
    * (volatile uint8_t * )29U &= ~(0x01 << 6);
#line 57
    __nesc_atomic_end(__nesc_atomic); }
}

#line 56
static void /*HplMsp430GeneralIOC.P46*/HplMsp430GeneralIOP__30__IO__set(void )
#line 56
{
#line 56
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 56
    * (volatile uint8_t * )29U |= 0x01 << 6;
#line 56
    __nesc_atomic_end(__nesc_atomic); }
}

# 785 "/home/csilab/upma/lib/macs/pure-tdma/PureTDMASchedulerP.nc"
static void PureTDMASchedulerP__reset_var(uint8_t flowid)
#line 785
{
  if (PureTDMASchedulerP__is_root == TRUE) {
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 787
        PureTDMASchedulerP__is_AH_pkt_null = TRUE;
#line 787
        __nesc_atomic_end(__nesc_atomic); }
    }
  else 
#line 788
    {
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 789
        PureTDMASchedulerP__rcv_slot[flowid] = 0;
#line 789
        __nesc_atomic_end(__nesc_atomic); }
      PureTDMASchedulerP__Q_reset(flowid);
    }
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 792
    PureTDMASchedulerP__tx_opper[flowid] = 0;
#line 792
    __nesc_atomic_end(__nesc_atomic); }
}

# 57 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static void /*HplMsp430GeneralIOC.P42*/HplMsp430GeneralIOP__26__IO__clr(void )
#line 57
{
#line 57
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 57
    * (volatile uint8_t * )29U &= ~(0x01 << 2);
#line 57
    __nesc_atomic_end(__nesc_atomic); }
}

# 726 "/home/csilab/upma/chips/cc2420/CC2420TransmitP.nc"
static void CC2420TransmitP__signalDone(error_t err)
#line 726
{
  if (CC2420TransmitP__m_tx_power != 31) {
      CC2420TransmitP__TXCTRL__write((((2 << CC2420_TXCTRL_TXMIXBUF_CUR) | (
      3 << CC2420_TXCTRL_PA_CURRENT)) | (
      1 << CC2420_TXCTRL_RESERVED)) | (
      31 << CC2420_TXCTRL_PA_LEVEL));
      CC2420TransmitP__m_tx_power = 31;
    }
  /* atomic removed: atomic calls only */
  CC2420TransmitP__m_state = CC2420TransmitP__S_STARTED;
  CC2420TransmitP__abortSpiRelease = FALSE;
  CC2420TransmitP__ChipSpiResource__attemptRelease();
  CC2420TransmitP__Send__sendDone(CC2420TransmitP__m_msg, err);
}

# 305 "/home/csilab/tinyos/tos/chips/cc2420/spi/CC2420SpiP.nc"
static cc2420_status_t CC2420SpiP__Reg__write(uint8_t addr, uint16_t data)
#line 305
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 306
    {
      if (CC2420SpiP__WorkingState__isIdle()) {
          {
            unsigned char __nesc_temp = 
#line 308
            0;

            {
#line 308
              __nesc_atomic_end(__nesc_atomic); 
#line 308
              return __nesc_temp;
            }
          }
        }
    }
#line 312
    __nesc_atomic_end(__nesc_atomic); }
#line 311
  CC2420SpiP__SpiByte__write(addr);
  CC2420SpiP__SpiByte__write(data >> 8);
  return CC2420SpiP__SpiByte__write(data & 0xff);
}

# 49 "/home/csilab/tinyos/tos/chips/msp430/timer/GpioCaptureC.nc"
static error_t /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__enableCapture(uint8_t mode)
#line 49
{
  /* atomic removed: atomic calls only */
#line 50
  {
    /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Msp430TimerControl__disableEvents();
    /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__GeneralIO__selectModuleFunc();
    /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Msp430TimerControl__clearPendingInterrupt();
    /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Msp430Capture__clearOverflow();
    /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Msp430TimerControl__setControlAsCapture(mode);
    /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Msp430TimerControl__enableEvents();
  }
  return SUCCESS;
}

# 260 "/home/csilab/tinyos/tos/chips/cc2420/spi/CC2420SpiP.nc"
static cc2420_status_t CC2420SpiP__Ram__write(uint16_t addr, uint8_t offset, 
uint8_t *data, 
uint8_t len)
#line 262
{

  cc2420_status_t status = 0;
  uint8_t tmpLen = len;
  uint8_t * tmpData = (uint8_t * )data;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 268
    {
      if (CC2420SpiP__WorkingState__isIdle()) {
          {
            unsigned char __nesc_temp = 
#line 270
            status;

            {
#line 270
              __nesc_atomic_end(__nesc_atomic); 
#line 270
              return __nesc_temp;
            }
          }
        }
    }
#line 274
    __nesc_atomic_end(__nesc_atomic); }
#line 274
  addr += offset;

  status = CC2420SpiP__SpiByte__write(addr | 0x80);
  CC2420SpiP__SpiByte__write((addr >> 1) & 0xc0);
  for (; len; len--) {
      CC2420SpiP__SpiByte__write(tmpData[tmpLen - len]);
    }

  return status;
}

# 211 "/home/csilab/upma/chips/cc2420/CC2420PacketP.nc"
static void CC2420PacketP__PacketTimeStamp32khz__clear(message_t *msg)
{
  __nesc_hton_int8(CC2420PacketP__CC2420PacketBody__getMetadata(msg)->timesync.nxdata, FALSE);
  __nesc_hton_uint32(CC2420PacketP__CC2420PacketBody__getMetadata(msg)->timestamp.nxdata, CC2420_INVALID_TIMESTAMP);
}

# 107 "/home/csilab/tinyos/tos/lib/timer/TransformAlarmC.nc"
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__set_alarm(void )
{
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__to_size_type now = /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__Counter__get();
#line 109
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__to_size_type expires;
#line 109
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__to_size_type remaining;




  expires = /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__m_t0 + /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__m_dt;


  remaining = (/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__to_size_type )(expires - now);


  if (/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__m_t0 <= now) 
    {
      if (expires >= /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__m_t0 && 
      expires <= now) {
        remaining = 0;
        }
    }
  else {
      if (expires >= /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__m_t0 || 
      expires <= now) {
        remaining = 0;
        }
    }
#line 132
  if (remaining > /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__MAX_DELAY) 
    {
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__m_t0 = now + /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__MAX_DELAY;
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__m_dt = remaining - /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__MAX_DELAY;
      remaining = /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__MAX_DELAY;
    }
  else 
    {
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__m_t0 += /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__m_dt;
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__m_dt = 0;
    }
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__AlarmFrom__startAt((/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__from_size_type )now << 5, 
  (/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__from_size_type )remaining << 5);
}

# 80 "/home/csilab/tinyos/tos/lib/timer/TransformCounterC.nc"
static /*CounterMilli32C.Transform*/TransformCounterC__1__to_size_type /*CounterMilli32C.Transform*/TransformCounterC__1__Counter__get(void )
{
  /*CounterMilli32C.Transform*/TransformCounterC__1__to_size_type rv = 0;

#line 83
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      /*CounterMilli32C.Transform*/TransformCounterC__1__upper_count_type high = /*CounterMilli32C.Transform*/TransformCounterC__1__m_upper;
      /*CounterMilli32C.Transform*/TransformCounterC__1__from_size_type low = /*CounterMilli32C.Transform*/TransformCounterC__1__CounterFrom__get();

#line 87
      if (/*CounterMilli32C.Transform*/TransformCounterC__1__CounterFrom__isOverflowPending()) 
        {






          high++;
          low = /*CounterMilli32C.Transform*/TransformCounterC__1__CounterFrom__get();
        }
      {
        /*CounterMilli32C.Transform*/TransformCounterC__1__to_size_type high_to = high;
        /*CounterMilli32C.Transform*/TransformCounterC__1__to_size_type low_to = low >> /*CounterMilli32C.Transform*/TransformCounterC__1__LOW_SHIFT_RIGHT;

#line 101
        rv = (high_to << /*CounterMilli32C.Transform*/TransformCounterC__1__HIGH_SHIFT_LEFT) | low_to;
      }
    }
#line 103
    __nesc_atomic_end(__nesc_atomic); }
  return rv;
}

# 59 "/home/csilab/upma/system/SenderDispatcherP.nc"
static error_t SenderDispatcherP__Send__send(uint8_t slotType, message_t *msg, uint8_t len)
#line 59
{
  error_t err;
  message_t *prev_toSend;
  uint8_t prev_toSendLen;
#line 62
  uint8_t prev_id;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 64
    {
      prev_toSend = SenderDispatcherP__toSend;
      prev_toSendLen = SenderDispatcherP__toSendLen;
      prev_id = SenderDispatcherP__last_id;
    }
#line 68
    __nesc_atomic_end(__nesc_atomic); }

  if (prev_id != NO_SLOT) {
      return FAIL;
    }

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 74
    {
      SenderDispatcherP__toSend = msg;
      SenderDispatcherP__toSendLen = len;
      SenderDispatcherP__last_id = slotType;
    }
#line 78
    __nesc_atomic_end(__nesc_atomic); }
  err = SenderDispatcherP__SubSend__send(msg, len);
  if (err != SUCCESS) {
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 81
        {
          SenderDispatcherP__toSend = prev_toSend;
          SenderDispatcherP__toSendLen = prev_toSendLen;
          SenderDispatcherP__last_id = prev_id;
        }
#line 85
        __nesc_atomic_end(__nesc_atomic); }
    }
  return err;
}

# 701 "/home/csilab/upma/chips/cc2420/CC2420TransmitP.nc"
static void CC2420TransmitP__loadTXFIFO(void )
#line 701
{
  cc2420_header_t *header = CC2420TransmitP__CC2420PacketBody__getHeader(CC2420TransmitP__m_msg);
  uint8_t tx_power = __nesc_ntoh_uint8(CC2420TransmitP__CC2420PacketBody__getMetadata(CC2420TransmitP__m_msg)->tx_power.nxdata);

  if (!tx_power) {
      tx_power = 31;
    }

  CC2420TransmitP__CSN__clr();

  if (CC2420TransmitP__m_tx_power != tx_power) {
      CC2420TransmitP__TXCTRL__write((((2 << CC2420_TXCTRL_TXMIXBUF_CUR) | (
      3 << CC2420_TXCTRL_PA_CURRENT)) | (
      1 << CC2420_TXCTRL_RESERVED)) | ((
      tx_power & 0x1F) << CC2420_TXCTRL_PA_LEVEL));
    }

  CC2420TransmitP__m_tx_power = tx_power;

  {
    uint8_t tmpLen __attribute((unused))  = __nesc_ntoh_leuint8(header->length.nxdata) - 1;

#line 722
    CC2420TransmitP__TXFIFO__write((uint8_t * )header, __nesc_ntoh_leuint8(header->length.nxdata) - 1);
  }
}

# 205 "/home/csilab/tinyos/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__SpiPacket__send(uint8_t id, uint8_t *tx_buf, 
uint8_t *rx_buf, 
uint16_t len)
#line 207
{

  /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_client = id;
  /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_tx_buf = tx_buf;
  /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_rx_buf = rx_buf;
  /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_len = len;
  /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_pos = 0;

  if (len) {
      /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__enableRxIntr();
      /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__continueOp();
    }
  else {
      /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__signalDone_task__postTask();
    }

  return SUCCESS;
}

#line 182
static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__continueOp(void )
#line 182
{

  uint8_t end;
  uint8_t tmp;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 187
    {
      /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__tx(/*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_tx_buf ? /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_tx_buf[/*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_pos] : 0);

      end = /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_pos + /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__SPI_ATOMIC_SIZE;
      if (end > /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_len) {
        end = /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_len;
        }
      while (++/*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_pos < end) {
          while (!/*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__isRxIntrPending()) ;
          tmp = /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__rx();
          if (/*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_rx_buf) {
            /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_rx_buf[/*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_pos - 1] = tmp;
            }
#line 199
          /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__tx(/*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_tx_buf ? /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_tx_buf[/*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_pos] : 0);
        }
    }
#line 201
    __nesc_atomic_end(__nesc_atomic); }
}

# 112 "/home/csilab/upma/system/GenericSlotterP.nc"
static void GenericSlotterP__Alarm__fired(void )
#line 112
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 113
    GenericSlotterP__nextFire = GenericSlotterP__Alarm__getNow() + GenericSlotterP__slotLength;
#line 113
    __nesc_atomic_end(__nesc_atomic); }
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 114
    GenericSlotterP__slot = (GenericSlotterP__slot + 1) % GenericSlotterP__frameLength;
#line 114
    __nesc_atomic_end(__nesc_atomic); }
  GenericSlotterP__Slotter__slot(GenericSlotterP__slot);

  GenericSlotterP__Alarm__start(GenericSlotterP__slotLength);
}

# 130 "/home/csilab/tinyos/tos/lib/printf/PrintfP.nc"
  int printfflush(void )
#line 130
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 131
    {
      if (PrintfP__state == PrintfP__S_FLUSHING) 
        {
          int __nesc_temp = 
#line 133
          SUCCESS;

          {
#line 133
            __nesc_atomic_end(__nesc_atomic); 
#line 133
            return __nesc_temp;
          }
        }
#line 134
      if (PrintfP__Queue__empty()) 
        {
          int __nesc_temp = 
#line 135
          FAIL;

          {
#line 135
            __nesc_atomic_end(__nesc_atomic); 
#line 135
            return __nesc_temp;
          }
        }
#line 136
      PrintfP__state = PrintfP__S_FLUSHING;
    }
#line 137
    __nesc_atomic_end(__nesc_atomic); }
  PrintfP__sendNext();
  return SUCCESS;
}

#line 119
static void PrintfP__sendNext(void )
#line 119
{
  int i;
  printf_msg_t *m = (printf_msg_t *)PrintfP__Packet__getPayload(&PrintfP__printfMsg, sizeof(printf_msg_t ));
  uint16_t length_to_send = PrintfP__Queue__size() < sizeof(printf_msg_t ) ? PrintfP__Queue__size() : sizeof(printf_msg_t );

#line 123
  memset(m->buffer, 0, sizeof(printf_msg_t ));
  for (i = 0; i < length_to_send; i++) 
    __nesc_hton_uint8(m->buffer[i].nxdata, PrintfP__Queue__dequeue());
  if (PrintfP__AMSend__send(AM_BROADCAST_ADDR, &PrintfP__printfMsg, sizeof(printf_msg_t )) != SUCCESS) {
    PrintfP__retrySend__postTask();
    }
}

# 119 "/home/csilab/upma/lib/serial/SerialActiveMessageP.nc"
static void */*SerialActiveMessageC.AM*/SerialActiveMessageP__0__Packet__getPayload(message_t *msg, uint8_t len)
#line 119
{
  if (len > /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__Packet__maxPayloadLength()) {
      return (void *)0;
    }
  else {
      return (void * )msg->data;
    }
}

# 53 "/home/csilab/tinyos/tos/system/AMQueueEntryP.nc"
static error_t /*PrintfC.SerialAMSenderC.AMQueueEntryP*/AMQueueEntryP__0__AMSend__send(am_addr_t dest, 
message_t *msg, 
uint8_t len)
#line 55
{
  /*PrintfC.SerialAMSenderC.AMQueueEntryP*/AMQueueEntryP__0__AMPacket__setDestination(msg, dest);
  /*PrintfC.SerialAMSenderC.AMQueueEntryP*/AMQueueEntryP__0__AMPacket__setType(msg, 100);
  return /*PrintfC.SerialAMSenderC.AMQueueEntryP*/AMQueueEntryP__0__Send__send(msg, len);
}

# 156 "/home/csilab/upma/lib/serial/SerialActiveMessageP.nc"
static am_id_t /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMPacket__type(message_t *amsg)
#line 156
{
  serial_header_t *header = /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__getHeader(amsg);

#line 158
  return __nesc_ntoh_uint8(header->type.nxdata);
}

#line 132
static am_addr_t /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMPacket__destination(message_t *amsg)
#line 132
{
  serial_header_t *header = /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__getHeader(amsg);

#line 134
  return __nesc_ntoh_uint16(header->dest.nxdata);
}

#line 57
static error_t /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMSend__send(am_id_t id, am_addr_t dest, 
message_t *msg, 
uint8_t len)
#line 59
{
  serial_header_t *header = /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__getHeader(msg);

#line 61
  __nesc_hton_uint16(header->dest.nxdata, dest);





  __nesc_hton_uint8(header->type.nxdata, id);
  __nesc_hton_uint8(header->length.nxdata, len);

  return /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__SubSend__send(msg, len);
}

# 179 "/home/csilab/upma/chips/cc2420/CC2420CsmaP.nc"
static void *CC2420CsmaP__Send__getPayload(message_t *m, uint8_t len)
#line 179
{
  if (len <= CC2420CsmaP__Send__maxPayloadLength()) {
      return (void * )m->data;
    }
  else {
      return (void *)0;
    }
}

# 760 "/home/csilab/upma/lib/macs/pure-tdma/PureTDMASchedulerP.nc"
static void PureTDMASchedulerP__set_payload(adaptivehart_msg_t *payload, uint8_t flowid, uint8_t priority, uint8_t maxtx, uint16_t deadline, uint16_t seq)
#line 760
{
  __nesc_hton_uint8(payload->flowid.nxdata, flowid);
  __nesc_hton_uint8(payload->priority.nxdata, priority);
  __nesc_hton_uint8(payload->maxtx.nxdata, maxtx);
  __nesc_hton_uint16(payload->deadline.nxdata, deadline);
  __nesc_hton_uint16(payload->seq.nxdata, seq);
}

#line 729
static void PureTDMASchedulerP__send_AH_Pkt(message_t *packet, am_addr_t dest, uint8_t flowid)
#line 729
{
  bool tmp_phyLock;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 732
    tmp_phyLock = PureTDMASchedulerP__phyLock;
#line 732
    __nesc_atomic_end(__nesc_atomic); }
  if (tmp_phyLock == FALSE) {
      PureTDMASchedulerP__ACK__requestAck(packet);
      PureTDMASchedulerP__AMPacket__setSource(packet, TOS_NODE_ID);
      PureTDMASchedulerP__AMPacket__setDestination(packet, dest);
      if (PureTDMASchedulerP__SubSend__send(packet, sizeof(adaptivehart_msg_t )) == SUCCESS) {
          { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 738
            PureTDMASchedulerP__phyLock = TRUE;
#line 738
            __nesc_atomic_end(__nesc_atomic); }
          { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 739
            PureTDMASchedulerP__tx_opper[flowid] = PureTDMASchedulerP__tx_opper[flowid] - 1;
#line 739
            __nesc_atomic_end(__nesc_atomic); }

          if (PureTDMASchedulerP__is_root == TRUE && PureTDMASchedulerP__tx_flow_seq <= PureTDMASchedulerP__FLOW_SEQ) {
              PureTDMASchedulerP__tx_slots[PureTDMASchedulerP__tx_flow_seq] = PureTDMASchedulerP__SlotterControl__getSlot();
              PureTDMASchedulerP__tx_flow_seq += 1;
            }
          else {
#line 744
            if (PureTDMASchedulerP__is_root == FALSE && PureTDMASchedulerP__tx_flow_seq <= PureTDMASchedulerP__FLOW_SEQ) {
              }
            }
        }
      else 



        {
          printf("MAC: Pkt fail  \r\n");
        }
    }
  else 
#line 755
    {
      printf("MAC: Radio Locked \r\n");
    }
}

# 160 "/home/csilab/upma/chips/cc2420/CC2420ActiveMessageP.nc"
static void CC2420ActiveMessageP__AMPacket__setDestination(message_t *amsg, am_addr_t addr)
#line 160
{
  cc2420_header_t *header = CC2420ActiveMessageP__CC2420PacketBody__getHeader(amsg);

#line 162
  __nesc_hton_leuint16(header->dest.nxdata, addr);
}

# 107 "/home/csilab/tinyos/tos/lib/timer/TransformAlarmC.nc"
static void /*GenericSlotterC.Alarm32khz32C.Transform*/TransformAlarmC__2__set_alarm(void )
{
  /*GenericSlotterC.Alarm32khz32C.Transform*/TransformAlarmC__2__to_size_type now = /*GenericSlotterC.Alarm32khz32C.Transform*/TransformAlarmC__2__Counter__get();
#line 109
  /*GenericSlotterC.Alarm32khz32C.Transform*/TransformAlarmC__2__to_size_type expires;
#line 109
  /*GenericSlotterC.Alarm32khz32C.Transform*/TransformAlarmC__2__to_size_type remaining;




  expires = /*GenericSlotterC.Alarm32khz32C.Transform*/TransformAlarmC__2__m_t0 + /*GenericSlotterC.Alarm32khz32C.Transform*/TransformAlarmC__2__m_dt;


  remaining = (/*GenericSlotterC.Alarm32khz32C.Transform*/TransformAlarmC__2__to_size_type )(expires - now);


  if (/*GenericSlotterC.Alarm32khz32C.Transform*/TransformAlarmC__2__m_t0 <= now) 
    {
      if (expires >= /*GenericSlotterC.Alarm32khz32C.Transform*/TransformAlarmC__2__m_t0 && 
      expires <= now) {
        remaining = 0;
        }
    }
  else {
      if (expires >= /*GenericSlotterC.Alarm32khz32C.Transform*/TransformAlarmC__2__m_t0 || 
      expires <= now) {
        remaining = 0;
        }
    }
#line 132
  if (remaining > /*GenericSlotterC.Alarm32khz32C.Transform*/TransformAlarmC__2__MAX_DELAY) 
    {
      /*GenericSlotterC.Alarm32khz32C.Transform*/TransformAlarmC__2__m_t0 = now + /*GenericSlotterC.Alarm32khz32C.Transform*/TransformAlarmC__2__MAX_DELAY;
      /*GenericSlotterC.Alarm32khz32C.Transform*/TransformAlarmC__2__m_dt = remaining - /*GenericSlotterC.Alarm32khz32C.Transform*/TransformAlarmC__2__MAX_DELAY;
      remaining = /*GenericSlotterC.Alarm32khz32C.Transform*/TransformAlarmC__2__MAX_DELAY;
    }
  else 
    {
      /*GenericSlotterC.Alarm32khz32C.Transform*/TransformAlarmC__2__m_t0 += /*GenericSlotterC.Alarm32khz32C.Transform*/TransformAlarmC__2__m_dt;
      /*GenericSlotterC.Alarm32khz32C.Transform*/TransformAlarmC__2__m_dt = 0;
    }
  /*GenericSlotterC.Alarm32khz32C.Transform*/TransformAlarmC__2__AlarmFrom__startAt((/*GenericSlotterC.Alarm32khz32C.Transform*/TransformAlarmC__2__from_size_type )now << 0, 
  (/*GenericSlotterC.Alarm32khz32C.Transform*/TransformAlarmC__2__from_size_type )remaining << 0);
}

# 75 "/home/csilab/upma/system/GenericSlotterP.nc"
static error_t GenericSlotterP__SlotterControl__synchronize(uint8_t toSlot)
#line 75
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 76
    GenericSlotterP__slot = toSlot - 1;
#line 76
    __nesc_atomic_end(__nesc_atomic); }

  GenericSlotterP__Alarm__stop();
  GenericSlotterP__Alarm__fired();
  return SUCCESS;
}

# 107 "/home/csilab/tinyos/tos/lib/timer/TransformAlarmC.nc"
static void /*PureTDMASchedulerC.Alarm32khz32C.Transform*/TransformAlarmC__3__set_alarm(void )
{
  /*PureTDMASchedulerC.Alarm32khz32C.Transform*/TransformAlarmC__3__to_size_type now = /*PureTDMASchedulerC.Alarm32khz32C.Transform*/TransformAlarmC__3__Counter__get();
#line 109
  /*PureTDMASchedulerC.Alarm32khz32C.Transform*/TransformAlarmC__3__to_size_type expires;
#line 109
  /*PureTDMASchedulerC.Alarm32khz32C.Transform*/TransformAlarmC__3__to_size_type remaining;




  expires = /*PureTDMASchedulerC.Alarm32khz32C.Transform*/TransformAlarmC__3__m_t0 + /*PureTDMASchedulerC.Alarm32khz32C.Transform*/TransformAlarmC__3__m_dt;


  remaining = (/*PureTDMASchedulerC.Alarm32khz32C.Transform*/TransformAlarmC__3__to_size_type )(expires - now);


  if (/*PureTDMASchedulerC.Alarm32khz32C.Transform*/TransformAlarmC__3__m_t0 <= now) 
    {
      if (expires >= /*PureTDMASchedulerC.Alarm32khz32C.Transform*/TransformAlarmC__3__m_t0 && 
      expires <= now) {
        remaining = 0;
        }
    }
  else {
      if (expires >= /*PureTDMASchedulerC.Alarm32khz32C.Transform*/TransformAlarmC__3__m_t0 || 
      expires <= now) {
        remaining = 0;
        }
    }
#line 132
  if (remaining > /*PureTDMASchedulerC.Alarm32khz32C.Transform*/TransformAlarmC__3__MAX_DELAY) 
    {
      /*PureTDMASchedulerC.Alarm32khz32C.Transform*/TransformAlarmC__3__m_t0 = now + /*PureTDMASchedulerC.Alarm32khz32C.Transform*/TransformAlarmC__3__MAX_DELAY;
      /*PureTDMASchedulerC.Alarm32khz32C.Transform*/TransformAlarmC__3__m_dt = remaining - /*PureTDMASchedulerC.Alarm32khz32C.Transform*/TransformAlarmC__3__MAX_DELAY;
      remaining = /*PureTDMASchedulerC.Alarm32khz32C.Transform*/TransformAlarmC__3__MAX_DELAY;
    }
  else 
    {
      /*PureTDMASchedulerC.Alarm32khz32C.Transform*/TransformAlarmC__3__m_t0 += /*PureTDMASchedulerC.Alarm32khz32C.Transform*/TransformAlarmC__3__m_dt;
      /*PureTDMASchedulerC.Alarm32khz32C.Transform*/TransformAlarmC__3__m_dt = 0;
    }
  /*PureTDMASchedulerC.Alarm32khz32C.Transform*/TransformAlarmC__3__AlarmFrom__startAt((/*PureTDMASchedulerC.Alarm32khz32C.Transform*/TransformAlarmC__3__from_size_type )now << 0, 
  (/*PureTDMASchedulerC.Alarm32khz32C.Transform*/TransformAlarmC__3__from_size_type )remaining << 0);
}

# 14 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430TimerCommonP.nc"
__attribute((wakeup)) __attribute((interrupt(0x0018)))  void sig_TIMERB1_VECTOR(void )
#line 14
{
#line 14
  Msp430TimerCommonP__VectorTimerB1__fired();
}

# 63 "/home/csilab/tinyos/tos/system/RealMainP.nc"
  int main(void )
#line 63
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {





      {
      }
#line 71
      ;

      RealMainP__Scheduler__init();





      RealMainP__PlatformInit__init();
      while (RealMainP__Scheduler__runNextTask()) ;





      RealMainP__SoftwareInit__init();
      while (RealMainP__Scheduler__runNextTask()) ;
    }
#line 88
    __nesc_atomic_end(__nesc_atomic); }


  __nesc_enable_interrupt();

  RealMainP__Boot__booted();


  RealMainP__Scheduler__taskLoop();




  return -1;
}

# 175 "/home/csilab/tinyos/tos/chips/msp430/timer/Msp430ClockP.nc"
static void Msp430ClockP__set_dco_calib(int calib)
{
  BCSCTL1 = (BCSCTL1 & ~0x07) | ((calib >> 8) & 0x07);
  DCOCTL = calib & 0xff;
}

# 16 "/home/csilab/tinyos/tos/platforms/telosb/MotePlatformC.nc"
static void MotePlatformC__TOSH_FLASH_M25P_DP_bit(bool set)
#line 16
{
  if (set) {
    TOSH_SET_SIMO0_PIN();
    }
  else {
#line 20
    TOSH_CLR_SIMO0_PIN();
    }
#line 21
  TOSH_SET_UCLK0_PIN();
  TOSH_CLR_UCLK0_PIN();
}

# 134 "/home/csilab/tinyos/tos/system/SchedulerBasicP.nc"
static bool SchedulerBasicP__Scheduler__runNextTask(void )
{
  uint8_t nextTask;

  /* atomic removed: atomic calls only */
#line 138
  {
    nextTask = SchedulerBasicP__popTask();
    if (nextTask == SchedulerBasicP__NO_TASK) 
      {
        {
          unsigned char __nesc_temp = 
#line 142
          FALSE;

#line 142
          return __nesc_temp;
        }
      }
  }
#line 145
  SchedulerBasicP__TaskBasic__runTask(nextTask);
  return TRUE;
}

#line 175
static void SchedulerBasicP__TaskBasic__default__runTask(uint8_t id)
{
}

# 75 "/home/csilab/tinyos/tos/interfaces/TaskBasic.nc"
static void SchedulerBasicP__TaskBasic__runTask(uint8_t arg_0x7f7fedb6be60){
#line 75
  switch (arg_0x7f7fedb6be60) {
#line 75
    case SerialP__RunTx:
#line 75
      SerialP__RunTx__runTask();
#line 75
      break;
#line 75
    case SerialP__startDoneTask:
#line 75
      SerialP__startDoneTask__runTask();
#line 75
      break;
#line 75
    case SerialP__stopDoneTask:
#line 75
      SerialP__stopDoneTask__runTask();
#line 75
      break;
#line 75
    case SerialP__defaultSerialFlushTask:
#line 75
      SerialP__defaultSerialFlushTask__runTask();
#line 75
      break;
#line 75
    case /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__signalSendDone:
#line 75
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__signalSendDone__runTask();
#line 75
      break;
#line 75
    case /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTask:
#line 75
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTask__runTask();
#line 75
      break;
#line 75
    case /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__grantedTask:
#line 75
      /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__grantedTask__runTask();
#line 75
      break;
#line 75
    case /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__CancelTask:
#line 75
      /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__CancelTask__runTask();
#line 75
      break;
#line 75
    case /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__errorTask:
#line 75
      /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__errorTask__runTask();
#line 75
      break;
#line 75
    case PrintfP__retrySend:
#line 75
      PrintfP__retrySend__runTask();
#line 75
      break;
#line 75
    case CC2420CsmaP__startDone_task:
#line 75
      CC2420CsmaP__startDone_task__runTask();
#line 75
      break;
#line 75
    case CC2420CsmaP__stopDone_task:
#line 75
      CC2420CsmaP__stopDone_task__runTask();
#line 75
      break;
#line 75
    case CC2420CsmaP__signalStartDone:
#line 75
      CC2420CsmaP__signalStartDone__runTask();
#line 75
      break;
#line 75
    case CC2420CsmaP__signalStopDone:
#line 75
      CC2420CsmaP__signalStopDone__runTask();
#line 75
      break;
#line 75
    case CC2420ControlP__sync:
#line 75
      CC2420ControlP__sync__runTask();
#line 75
      break;
#line 75
    case CC2420ControlP__syncDone:
#line 75
      CC2420ControlP__syncDone__runTask();
#line 75
      break;
#line 75
    case CC2420SpiP__grant:
#line 75
      CC2420SpiP__grant__runTask();
#line 75
      break;
#line 75
    case /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__signalDone_task:
#line 75
      /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__signalDone_task__runTask();
#line 75
      break;
#line 75
    case /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__grantedTask:
#line 75
      /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__grantedTask__runTask();
#line 75
      break;
#line 75
    case /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired:
#line 75
      /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired__runTask();
#line 75
      break;
#line 75
    case /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer:
#line 75
      /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer__runTask();
#line 75
      break;
#line 75
    case CC2420TinyosNetworkP__grantTask:
#line 75
      CC2420TinyosNetworkP__grantTask__runTask();
#line 75
      break;
#line 75
    case AsyncReceiveAdapterP__receiveDone_task:
#line 75
      AsyncReceiveAdapterP__receiveDone_task__runTask();
#line 75
      break;
#line 75
    case AsyncSendAdapterP__sendDone_task:
#line 75
      AsyncSendAdapterP__sendDone_task__runTask();
#line 75
      break;
#line 75
    case PowerCycleP__startRadio:
#line 75
      PowerCycleP__startRadio__runTask();
#line 75
      break;
#line 75
    case PowerCycleP__getCca:
#line 75
      PowerCycleP__getCca__runTask();
#line 75
      break;
#line 75
    case /*TestPureTdmaC.SendingApp.App*/SendingC__0__sendTask:
#line 75
      /*TestPureTdmaC.SendingApp.App*/SendingC__0__sendTask__runTask();
#line 75
      break;
#line 75
    case /*TestPureTdmaC.SendingApp.App*/SendingC__0__flush:
#line 75
      /*TestPureTdmaC.SendingApp.App*/SendingC__0__flush__runTask();
#line 75
      break;
#line 75
    case /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__CancelTask:
#line 75
      /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__CancelTask__runTask();
#line 75
      break;
#line 75
    case /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__errorTask:
#line 75
      /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__errorTask__runTask();
#line 75
      break;
#line 75
    default:
#line 75
      SchedulerBasicP__TaskBasic__default__runTask(arg_0x7f7fedb6be60);
#line 75
      break;
#line 75
    }
#line 75
}
#line 75
# 97 "SendingC.nc"
static void /*TestPureTdmaC.SendingApp.App*/SendingC__0__AMSender__sendDone(message_t *bufPtr, error_t error)
{
  if (&/*TestPureTdmaC.SendingApp.App*/SendingC__0__packet == bufPtr) {
      uint32_t now = /*TestPureTdmaC.SendingApp.App*/SendingC__0__Counter__get();

#line 101
      /*TestPureTdmaC.SendingApp.App*/SendingC__0__latency = /*TestPureTdmaC.SendingApp.App*/SendingC__0__latency + (now - /*TestPureTdmaC.SendingApp.App*/SendingC__0__sendTime);




      /*TestPureTdmaC.SendingApp.App*/SendingC__0__packetsSent++;
      /*TestPureTdmaC.SendingApp.App*/SendingC__0__sends = FALSE;

      /*TestPureTdmaC.SendingApp.App*/SendingC__0__sendTask__postTask();
    }
}

# 163 "/home/csilab/tinyos/tos/system/AMQueueImplP.nc"
static void /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__sendDone(uint8_t last, message_t * msg, error_t err)
#line 163
{
  /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__queue[last].msg = (void *)0;
  /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__tryToSend();
  /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__Send__sendDone(last, msg, err);
}

# 175 "/home/csilab/upma/chips/cc2420/CC2420ActiveMessageP.nc"
static am_id_t CC2420ActiveMessageP__AMPacket__type(message_t *amsg)
#line 175
{
  cc2420_header_t *header = CC2420ActiveMessageP__CC2420PacketBody__getHeader(amsg);

#line 177
  return __nesc_ntoh_leuint8(header->type.nxdata);
}

#line 150
static am_addr_t CC2420ActiveMessageP__AMPacket__destination(message_t *amsg)
#line 150
{
  cc2420_header_t *header = CC2420ActiveMessageP__CC2420PacketBody__getHeader(amsg);

#line 152
  return __nesc_ntoh_leuint16(header->dest.nxdata);
}

#line 102
static error_t CC2420ActiveMessageP__AMSend__send(am_id_t id, am_addr_t addr, 
message_t *msg, 
uint8_t len)
#line 104
{
  cc2420_header_t *header = CC2420ActiveMessageP__CC2420PacketBody__getHeader(msg);

  if (len > CC2420ActiveMessageP__Packet__maxPayloadLength()) {
      return ESIZE;
    }

  __nesc_hton_leuint8(header->type.nxdata, id);
  __nesc_hton_leuint16(header->dest.nxdata, addr);
  __nesc_hton_leuint16(header->destpan.nxdata, CC2420ActiveMessageP__CC2420Config__getPanAddr());
  __nesc_hton_leuint16(header->src.nxdata, CC2420ActiveMessageP__AMPacket__address());

  if (CC2420ActiveMessageP__RadioResource__immediateRequest() == SUCCESS) {
      error_t rc;

#line 118
      CC2420ActiveMessageP__SendNotifier__aboutToSend(id, addr, msg);

      rc = CC2420ActiveMessageP__SubSend__send(msg, len);
      if (rc != SUCCESS) {
          CC2420ActiveMessageP__RadioResource__release();
        }

      return rc;
    }
  else 
#line 126
    {
      CC2420ActiveMessageP__pending_length = len;
      CC2420ActiveMessageP__pending_message = msg;
      return CC2420ActiveMessageP__RadioResource__request();
    }
}

# 106 "/home/csilab/tinyos/tos/system/ActiveMessageAddressC.nc"
static am_addr_t ActiveMessageAddressC__amAddress(void )
#line 106
{
  am_addr_t myAddr;

#line 108
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 108
    myAddr = ActiveMessageAddressC__addr;
#line 108
    __nesc_atomic_end(__nesc_atomic); }
  return myAddr;
}

# 60 "/home/csilab/tinyos/tos/system/FcfsResourceQueueC.nc"
static bool /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__FcfsQueue__isEmpty(void )
#line 60
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 61
    {
      unsigned char __nesc_temp = 
#line 61
      /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__qHead == /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__NO_ENTRY;

      {
#line 61
        __nesc_atomic_end(__nesc_atomic); 
#line 61
        return __nesc_temp;
      }
    }
#line 63
    __nesc_atomic_end(__nesc_atomic); }
}

# 80 "/home/csilab/tinyos/tos/chips/cc2420/lowpan/CC2420TinyosNetworkP.nc"
static error_t CC2420TinyosNetworkP__ActiveSend__send(message_t *msg, uint8_t len)
#line 80
{
  CC2420TinyosNetworkP__CC2420Packet__setNetwork(msg, 0x3f);
  CC2420TinyosNetworkP__m_busy_client = CC2420TinyosNetworkP__CLIENT_AM;
  return CC2420TinyosNetworkP__SubSend__send(msg, len);
}

# 56 "/home/csilab/tinyos/tos/interfaces/State.nc"
static void UniqueSendP__State__toIdle(void ){
#line 56
  StateImplP__State__toIdle(2U);
#line 56
}
#line 56
# 180 "/home/csilab/upma/chips/cc2420/CC2420ActiveMessageP.nc"
static void CC2420ActiveMessageP__AMPacket__setType(message_t *amsg, am_id_t type)
#line 180
{
  cc2420_header_t *header = CC2420ActiveMessageP__CC2420PacketBody__getHeader(amsg);

#line 182
  __nesc_hton_leuint8(header->type.nxdata, type);
}

# 54 "/home/csilab/upma/system/LocalTime16P.nc"
static local_time16_t /*LocalTime32khz16C.LocalTimeP*/LocalTime16P__0__LocalTime__getNow(void )
#line 54
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 55
    {
      __nesc_hton_uint16(/*LocalTime32khz16C.LocalTimeP*/LocalTime16P__0__g.sticks.nxdata, /*LocalTime32khz16C.LocalTimeP*/LocalTime16P__0__Counter__get());
      {
        nx_struct local_time16 __nesc_temp = 
#line 57
        /*LocalTime32khz16C.LocalTimeP*/LocalTime16P__0__g;

        {
#line 57
          __nesc_atomic_end(__nesc_atomic); 
#line 57
          return __nesc_temp;
        }
      }
    }
#line 60
    __nesc_atomic_end(__nesc_atomic); }
}

# 91 "/home/csilab/upma/chips/cc2420/CC2420CsmaP.nc"
static error_t CC2420CsmaP__RadioPowerControl__start(void )
#line 91
{
  if (CC2420CsmaP__SplitControlState__requestState(CC2420CsmaP__S_STARTING) == SUCCESS) {




      CC2420CsmaP__CC2420Power__startVReg();
      return SUCCESS;
    }
  else {
#line 100
    if (CC2420CsmaP__SplitControlState__isState(CC2420CsmaP__S_STARTED)) {
        return EALREADY;
      }
    else {
#line 103
      if (CC2420CsmaP__SplitControlState__isState(CC2420CsmaP__S_STARTING)) {
          return SUCCESS;
        }
      }
    }
#line 107
  return EBUSY;
}

# 189 "/home/csilab/tinyos/tos/system/AMQueueImplP.nc"
static void /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__AMSend__sendDone(am_id_t id, message_t *msg, error_t err)
#line 189
{





  if (/*AMQueueP.AMQueueImplP*/AMQueueImplP__1__current >= 1) {
      return;
    }
  if (/*AMQueueP.AMQueueImplP*/AMQueueImplP__1__queue[/*AMQueueP.AMQueueImplP*/AMQueueImplP__1__current].msg == msg) {
      /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__sendDone(/*AMQueueP.AMQueueImplP*/AMQueueImplP__1__current, msg, err);
    }
  else {
      ;
    }
}

# 366 "/home/csilab/upma/chips/cc2420/CC2420ReceiveP.nc"
static void CC2420ReceiveP__Receive__updateBuffer(message_t *msg)
#line 366
{
  CC2420ReceiveP__m_p_rx_buf = msg;
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 368
    CC2420ReceiveP__receivingPacket = FALSE;
#line 368
    __nesc_atomic_end(__nesc_atomic); }
  CC2420ReceiveP__waitForNextPacket();
}

#line 426
static void CC2420ReceiveP__waitForNextPacket(void )
#line 426
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 427
    {
      if (CC2420ReceiveP__m_state == CC2420ReceiveP__S_STOPPED) {
          CC2420ReceiveP__SpiResource__release();
          {
#line 430
            __nesc_atomic_end(__nesc_atomic); 
#line 430
            return;
          }
        }
      CC2420ReceiveP__receivingPacket = FALSE;
#line 445
      if ((CC2420ReceiveP__m_missed_packets && CC2420ReceiveP__FIFO__get()) || !CC2420ReceiveP__FIFOP__get()) {

          if (CC2420ReceiveP__m_missed_packets) {
              CC2420ReceiveP__m_missed_packets--;
            }

          CC2420ReceiveP__beginReceive();
        }
      else {

          CC2420ReceiveP__m_state = CC2420ReceiveP__S_STARTED;
          CC2420ReceiveP__m_missed_packets = 0;
          CC2420ReceiveP__SpiResource__release();
        }
    }
#line 459
    __nesc_atomic_end(__nesc_atomic); }
}

#line 380
static void CC2420ReceiveP__beginReceive(void )
#line 380
{
  CC2420ReceiveP__m_state = CC2420ReceiveP__S_RX_LENGTH;
  /* atomic removed: atomic calls only */
#line 382
  CC2420ReceiveP__receivingPacket = TRUE;
  if (CC2420ReceiveP__SpiResource__isOwner()) {
      CC2420ReceiveP__receive();
    }
  else {
#line 386
    if (CC2420ReceiveP__SpiResource__immediateRequest() == SUCCESS) {
        CC2420ReceiveP__receive();
      }
    else {
        CC2420ReceiveP__SpiResource__request();
      }
    }
}

#line 415
static void CC2420ReceiveP__receive(void )
#line 415
{
  CC2420ReceiveP__PacketIndicator__anypktreceive();
  CC2420ReceiveP__CSN__clr();
  CC2420ReceiveP__RXFIFO__beginRead((uint8_t *)CC2420ReceiveP__CC2420PacketBody__getHeader(CC2420ReceiveP__m_p_rx_buf), 1);
}

# 189 "/home/csilab/tinyos/tos/chips/cc2420/spi/CC2420SpiP.nc"
static cc2420_status_t CC2420SpiP__Fifo__beginRead(uint8_t addr, uint8_t *data, 
uint8_t len)
#line 190
{

  cc2420_status_t status = 0;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 194
    {
      if (CC2420SpiP__WorkingState__isIdle()) {
          {
            unsigned char __nesc_temp = 
#line 196
            status;

            {
#line 196
              __nesc_atomic_end(__nesc_atomic); 
#line 196
              return __nesc_temp;
            }
          }
        }
    }
#line 200
    __nesc_atomic_end(__nesc_atomic); }
#line 200
  CC2420SpiP__m_addr = addr | 0x40;

  status = CC2420SpiP__SpiByte__write(CC2420SpiP__m_addr);
  CC2420SpiP__Fifo__continueRead(addr, data, len);

  return status;
}

# 89 "/home/csilab/upma/chips/cc2420/CC2420ActiveMessageP.nc"
static void CC2420ActiveMessageP__RadioResource__granted(void )
#line 89
{
  uint8_t rc;
  cc2420_header_t *header = CC2420ActiveMessageP__CC2420PacketBody__getHeader(CC2420ActiveMessageP__pending_message);

  CC2420ActiveMessageP__SendNotifier__aboutToSend(__nesc_ntoh_leuint8(header->type.nxdata), __nesc_ntoh_leuint16(header->dest.nxdata), CC2420ActiveMessageP__pending_message);
  rc = CC2420ActiveMessageP__SubSend__send(CC2420ActiveMessageP__pending_message, CC2420ActiveMessageP__pending_length);
  if (rc != SUCCESS) {
      CC2420ActiveMessageP__RadioResource__release();
      CC2420ActiveMessageP__AMSend__sendDone(__nesc_ntoh_leuint8(header->type.nxdata), CC2420ActiveMessageP__pending_message, rc);
    }
}

# 73 "/home/csilab/tinyos/tos/lib/timer/VirtualizeTimerC.nc"
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__fireTimers(uint32_t now)
{
  uint16_t num;

  for (num = 0; num < /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__NUM_TIMERS; num++) 
    {
      /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer_t *timer = &/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__m_timers[num];

      if (timer->isrunning) 
        {
          uint32_t elapsed = now - timer->t0;

          if (elapsed >= timer->dt) 
            {
              if (timer->isoneshot) {
                timer->isrunning = FALSE;
                }
              else {
#line 90
                timer->t0 += timer->dt;
                }
              /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__fired(num);
              break;
            }
        }
    }
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer__postTask();
}

#line 154
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startPeriodic(uint8_t num, uint32_t dt)
{
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__startTimer(num, /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__getNow(), dt, FALSE);
}

#line 144
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__startTimer(uint8_t num, uint32_t t0, uint32_t dt, bool isoneshot)
{
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer_t *timer = &/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__m_timers[num];

#line 147
  timer->t0 = t0;
  timer->dt = dt;
  timer->isoneshot = isoneshot;
  timer->isrunning = TRUE;
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer__postTask();
}

# 147 "/home/csilab/tinyos/tos/lib/timer/TransformAlarmC.nc"
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__Alarm__startAt(/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__to_size_type t0, /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__to_size_type dt)
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__m_t0 = t0;
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__m_dt = dt;
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__set_alarm();
    }
#line 154
    __nesc_atomic_end(__nesc_atomic); }
}

# 329 "/home/csilab/tinyos/tos/chips/cc2420/spi/CC2420SpiP.nc"
static void CC2420SpiP__SpiPacket__sendDone(uint8_t *tx_buf, uint8_t *rx_buf, 
uint16_t len, error_t error)
#line 330
{
  if (CC2420SpiP__m_addr & 0x40) {
      CC2420SpiP__Fifo__readDone(CC2420SpiP__m_addr & ~0x40, rx_buf, len, error);
    }
  else 
#line 333
    {
      CC2420SpiP__Fifo__writeDone(CC2420SpiP__m_addr, tx_buf, len, error);
    }
}

# 397 "/home/csilab/upma/chips/cc2420/CC2420ReceiveP.nc"
static void CC2420ReceiveP__flush(void )
#line 397
{
  CC2420ReceiveP__reset_state();

  CC2420ReceiveP__CSN__set();
  CC2420ReceiveP__CSN__clr();
  CC2420ReceiveP__SFLUSHRX__strobe();
  CC2420ReceiveP__SFLUSHRX__strobe();
  CC2420ReceiveP__CSN__set();
  CC2420ReceiveP__SpiResource__release();
  CC2420ReceiveP__waitForNextPacket();
}

# 155 "/home/csilab/upma/chips/cc2420/CC2420ActiveMessageP.nc"
static am_addr_t CC2420ActiveMessageP__AMPacket__source(message_t *amsg)
#line 155
{
  cc2420_header_t *header = CC2420ActiveMessageP__CC2420PacketBody__getHeader(amsg);

#line 157
  return __nesc_ntoh_leuint16(header->src.nxdata);
}

# 479 "/home/csilab/tinyos/tos/chips/cc2420/control/CC2420ControlP.nc"
static void CC2420ControlP__writeFsctrl(void )
#line 479
{
  uint8_t channel;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 482
    {
      channel = CC2420ControlP__m_channel;
    }
#line 484
    __nesc_atomic_end(__nesc_atomic); }

  CC2420ControlP__FSCTRL__write((1 << CC2420_FSCTRL_LOCK_THR) | (((
  channel - 11) * 5 + 357) << CC2420_FSCTRL_FREQ));
}







static void CC2420ControlP__writeMdmctrl0(void )
#line 496
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 497
    {
      CC2420ControlP__MDMCTRL0__write((((((((1 << CC2420_MDMCTRL0_RESERVED_FRAME_MODE) | ((
      CC2420ControlP__addressRecognition && CC2420ControlP__hwAddressRecognition ? 1 : 0) << CC2420_MDMCTRL0_ADR_DECODE)) | (
      2 << CC2420_MDMCTRL0_CCA_HYST)) | (
      3 << CC2420_MDMCTRL0_CCA_MOD)) | (
      1 << CC2420_MDMCTRL0_AUTOCRC)) | ((
      CC2420ControlP__autoAckEnabled && CC2420ControlP__hwAutoAckDefault) << CC2420_MDMCTRL0_AUTOACK)) | (
      0 << CC2420_MDMCTRL0_AUTOACK)) | (
      2 << CC2420_MDMCTRL0_PREAMBLE_LENGTH));
    }
#line 506
    __nesc_atomic_end(__nesc_atomic); }
}







static void CC2420ControlP__writeId(void )
#line 515
{
  nxle_uint16_t id[6];

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 518
    {

      memcpy((uint8_t *)id, CC2420ControlP__m_ext_addr.data, 8);
      __nesc_hton_leuint16(id[4].nxdata, CC2420ControlP__m_pan);
      __nesc_hton_leuint16(id[5].nxdata, CC2420ControlP__m_short_addr);
    }
#line 523
    __nesc_atomic_end(__nesc_atomic); }

  CC2420ControlP__IEEEADR__write(0, (uint8_t *)&id, 12);
}

# 369 "/home/csilab/upma/lib/macs/pure-tdma/PureTDMASchedulerP.nc"
static void PureTDMASchedulerP__RadioPowerControl__startDone(error_t error)
#line 369
{

  if (PureTDMASchedulerP__coordinatorId == TOS_NODE_ID) {
      if (PureTDMASchedulerP__init == FALSE) {
          PureTDMASchedulerP__SplitControl__startDone(error);
          PureTDMASchedulerP__GenericSlotter__start();
          PureTDMASchedulerP__SlotterControl__synchronize(0);
          PureTDMASchedulerP__init = TRUE;
        }
    }
  else 
#line 378
    {
      if (PureTDMASchedulerP__init == FALSE) {
          PureTDMASchedulerP__SplitControl__startDone(error);
          PureTDMASchedulerP__init = TRUE;
        }
    }
}

# 137 "SendingC.nc"
static void /*TestPureTdmaC.SendingApp.App*/SendingC__0__SplitControl__startDone(error_t err)
{
  /*TestPureTdmaC.SendingApp.App*/SendingC__0__nodeId = (uint8_t *)/*TestPureTdmaC.SendingApp.App*/SendingC__0__Packet__getPayload(&/*TestPureTdmaC.SendingApp.App*/SendingC__0__packet, sizeof(uint8_t ));
  */*TestPureTdmaC.SendingApp.App*/SendingC__0__nodeId = TOS_NODE_ID;



  if (TOS_NODE_ID != 0) {
      /*TestPureTdmaC.SendingApp.App*/SendingC__0__SendTimer__startPeriodic(3000);
    }
  /*TestPureTdmaC.SendingApp.App*/SendingC__0__StartTimer__startOneShot(1000);
}

# 94 "/home/csilab/tinyos/tos/chips/cc2420/lowpan/CC2420TinyosNetworkP.nc"
static void *CC2420TinyosNetworkP__ActiveSend__getPayload(message_t *msg, uint8_t len)
#line 94
{
  if (len <= CC2420TinyosNetworkP__ActiveSend__maxPayloadLength()) {
      return msg->data;
    }
  else 
#line 97
    {
      return (void *)0;
    }
}

# 142 "/home/csilab/tinyos/tos/lib/printf/PrintfP.nc"
static void PrintfP__AMSend__sendDone(message_t *msg, error_t error)
#line 142
{
  if (error == SUCCESS) {
      if (PrintfP__Queue__size() > 0) {
        PrintfP__sendNext();
        }
      else {
#line 146
        PrintfP__state = PrintfP__S_STARTED;
        }
    }
  else {
#line 148
    PrintfP__retrySend__postTask();
    }
}

# 163 "/home/csilab/tinyos/tos/system/AMQueueImplP.nc"
static void /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__sendDone(uint8_t last, message_t * msg, error_t err)
#line 163
{
  /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__queue[last].msg = (void *)0;
  /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__tryToSend();
  /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__Send__sendDone(last, msg, err);
}

# 85 "/home/csilab/tinyos/tos/chips/msp430/usart/Msp430UartP.nc"
static void /*Msp430Uart1P.UartP*/Msp430UartP__0__ResourceConfigure__configure(uint8_t id)
#line 85
{
  msp430_uart_union_config_t *config = /*Msp430Uart1P.UartP*/Msp430UartP__0__Msp430UartConfigure__getConfig(id);

#line 87
  /*Msp430Uart1P.UartP*/Msp430UartP__0__m_byte_time = config->uartConfig.ubr / 2;
  /*Msp430Uart1P.UartP*/Msp430UartP__0__Usart__setModeUart(config);
  /*Msp430Uart1P.UartP*/Msp430UartP__0__Usart__enableIntr();
}

# 251 "/home/csilab/tinyos/tos/chips/msp430/usart/HplMsp430Usart1P.nc"
static void HplMsp430Usart1P__Usart__disableSpi(void )
#line 251
{
  /* atomic removed: atomic calls only */
#line 252
  {
    HplMsp430Usart1P__ME2 &= ~0x10;
    HplMsp430Usart1P__SIMO__selectIOFunc();
    HplMsp430Usart1P__SOMI__selectIOFunc();
    HplMsp430Usart1P__UCLK__selectIOFunc();
  }
}

#line 211
static void HplMsp430Usart1P__Usart__disableUart(void )
#line 211
{
  /* atomic removed: atomic calls only */
#line 212
  {
    HplMsp430Usart1P__ME2 &= ~(0x20 | 0x10);
    HplMsp430Usart1P__UTXD__selectIOFunc();
    HplMsp430Usart1P__URXD__selectIOFunc();
  }
}

# 93 "/home/csilab/tinyos/tos/system/ArbiterP.nc"
static error_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__Resource__immediateRequest(uint8_t id)
#line 93
{
  /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceRequested__immediateRequested(/*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__resId);
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 95
    {
      if (/*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__state == /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__RES_CONTROLLED) {
          /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__state = /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__RES_IMM_GRANTING;
          /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__reqResId = id;
        }
      else {
          unsigned char __nesc_temp = 
#line 100
          FAIL;

          {
#line 100
            __nesc_atomic_end(__nesc_atomic); 
#line 100
            return __nesc_temp;
          }
        }
    }
#line 103
    __nesc_atomic_end(__nesc_atomic); }
#line 102
  /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__immediateRequested();
  if (/*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__resId == id) {
      /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__configure(/*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__resId);
      return SUCCESS;
    }
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 107
    /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__state = /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__RES_CONTROLLED;
#line 107
    __nesc_atomic_end(__nesc_atomic); }
  return FAIL;
}

# 8 "/home/csilab/tinyos/tos/platforms/epic/chips/ds2411/DallasId48ToIeeeEui64C.nc"
static ieee_eui64_t DallasId48ToIeeeEui64C__LocalIeeeEui64__getId(void )
#line 8
{
  uint8_t id[6];
  ieee_eui64_t eui;

#line 11
  if (DallasId48ToIeeeEui64C__ReadId48__read(id) != SUCCESS) {
      memset(eui.data, 0, 8);
      goto done;
    }

  eui.data[0] = IEEE_EUI64_COMPANY_ID_0;
  eui.data[1] = IEEE_EUI64_COMPANY_ID_1;
  eui.data[2] = IEEE_EUI64_COMPANY_ID_2;



  eui.data[3] = IEEE_EUI64_SERIAL_ID_0;
  eui.data[4] = IEEE_EUI64_SERIAL_ID_1;


  eui.data[5] = id[2];
  eui.data[6] = id[1];
  eui.data[7] = id[0];

  done: 
    return eui;
}

# 63 "/home/csilab/tinyos/tos/lib/timer/BusyWaitCounterC.nc"
static void /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__BusyWait__wait(/*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__size_type dt)
{
  /* atomic removed: atomic calls only */
  {


    /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__size_type t0 = /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__Counter__get();

    if (dt > /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__HALF_MAX_SIZE_TYPE) 
      {
        dt -= /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__HALF_MAX_SIZE_TYPE;
        while (/*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__Counter__get() - t0 <= dt) ;
        t0 += dt;
        dt = /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__HALF_MAX_SIZE_TYPE;
      }

    while (/*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__Counter__get() - t0 <= dt) ;
  }
}

# 84 "SendingC.nc"
static void /*TestPureTdmaC.SendingApp.App*/SendingC__0__Boot__booted(void )
{
  /*TestPureTdmaC.SendingApp.App*/SendingC__0__sends = FALSE;
  /*TestPureTdmaC.SendingApp.App*/SendingC__0__packetsSent = 0;
  /*TestPureTdmaC.SendingApp.App*/SendingC__0__packetsReceived = 0;
  /*TestPureTdmaC.SendingApp.App*/SendingC__0__latency = 0;
  /*TestPureTdmaC.SendingApp.App*/SendingC__0__sendTask__postTask();
  /*TestPureTdmaC.SendingApp.App*/SendingC__0__FlushTimer__startPeriodic(250);
  /*TestPureTdmaC.SendingApp.App*/SendingC__0__SplitControl__start();
}

# 107 "/home/csilab/tinyos/tos/lib/printf/PutcharP.nc"
__attribute((noinline))   int putchar(int c)
#line 107
{
#line 119
  return PutcharP__Putchar__putchar(c);
}

# 64 "/home/csilab/tinyos/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
__attribute((wakeup)) __attribute((interrupt(0x0008)))  void sig_PORT1_VECTOR(void )
{
  volatile int n = P1IFG & P1IE;

  if (n & (1 << 0)) {
#line 68
      HplMsp430InterruptP__Port10__fired();
#line 68
      return;
    }
#line 69
  if (n & (1 << 1)) {
#line 69
      HplMsp430InterruptP__Port11__fired();
#line 69
      return;
    }
#line 70
  if (n & (1 << 2)) {
#line 70
      HplMsp430InterruptP__Port12__fired();
#line 70
      return;
    }
#line 71
  if (n & (1 << 3)) {
#line 71
      HplMsp430InterruptP__Port13__fired();
#line 71
      return;
    }
#line 72
  if (n & (1 << 4)) {
#line 72
      HplMsp430InterruptP__Port14__fired();
#line 72
      return;
    }
#line 73
  if (n & (1 << 5)) {
#line 73
      HplMsp430InterruptP__Port15__fired();
#line 73
      return;
    }
#line 74
  if (n & (1 << 6)) {
#line 74
      HplMsp430InterruptP__Port16__fired();
#line 74
      return;
    }
#line 75
  if (n & (1 << 7)) {
#line 75
      HplMsp430InterruptP__Port17__fired();
#line 75
      return;
    }
}

#line 169
__attribute((wakeup)) __attribute((interrupt(0x0002)))  void sig_PORT2_VECTOR(void )
{
  volatile int n = P2IFG & P2IE;

  if (n & (1 << 0)) {
#line 173
      HplMsp430InterruptP__Port20__fired();
#line 173
      return;
    }
#line 174
  if (n & (1 << 1)) {
#line 174
      HplMsp430InterruptP__Port21__fired();
#line 174
      return;
    }
#line 175
  if (n & (1 << 2)) {
#line 175
      HplMsp430InterruptP__Port22__fired();
#line 175
      return;
    }
#line 176
  if (n & (1 << 3)) {
#line 176
      HplMsp430InterruptP__Port23__fired();
#line 176
      return;
    }
#line 177
  if (n & (1 << 4)) {
#line 177
      HplMsp430InterruptP__Port24__fired();
#line 177
      return;
    }
#line 178
  if (n & (1 << 5)) {
#line 178
      HplMsp430InterruptP__Port25__fired();
#line 178
      return;
    }
#line 179
  if (n & (1 << 6)) {
#line 179
      HplMsp430InterruptP__Port26__fired();
#line 179
      return;
    }
#line 180
  if (n & (1 << 7)) {
#line 180
      HplMsp430InterruptP__Port27__fired();
#line 180
      return;
    }
}

# 96 "/home/csilab/tinyos/tos/chips/msp430/usart/HplMsp430Usart0P.nc"
__attribute((wakeup)) __attribute((interrupt(0x0012)))  void sig_UART0RX_VECTOR(void )
#line 96
{
  uint8_t temp = U0RXBUF;

#line 98
  HplMsp430Usart0P__Interrupts__rxDone(temp);
}

# 153 "/home/csilab/tinyos/tos/system/ArbiterP.nc"
static bool /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ArbiterInfo__inUse(void )
#line 153
{
  /* atomic removed: atomic calls only */
#line 154
  {
    if (/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__state == /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__RES_CONTROLLED) 
      {
        unsigned char __nesc_temp = 
#line 156
        FALSE;

#line 156
        return __nesc_temp;
      }
  }
#line 158
  return TRUE;
}






static uint8_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ArbiterInfo__userId(void )
#line 166
{
  /* atomic removed: atomic calls only */
#line 167
  {
    if (/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__state != /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__RES_BUSY) 
      {
        unsigned char __nesc_temp = 
#line 169
        /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__NO_RES;

#line 169
        return __nesc_temp;
      }
#line 170
    {
      unsigned char __nesc_temp = 
#line 170
      /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__resId;

#line 170
      return __nesc_temp;
    }
  }
}

# 101 "/home/csilab/tinyos/tos/chips/msp430/usart/HplMsp430Usart0P.nc"
__attribute((wakeup)) __attribute((interrupt(0x0010)))  void sig_UART0TX_VECTOR(void )
#line 101
{
  if (HplMsp430Usart0P__HplI2C__isI2C()) {
    HplMsp430Usart0P__I2CInterrupts__fired();
    }
  else {
#line 105
    HplMsp430Usart0P__Interrupts__txDone();
    }
}


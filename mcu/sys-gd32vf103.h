#define BOARD_ID_LONGAN_NANO       0xe65dace7
#define BOARD_ID_CQ_STARM          0xc5480875
#define BOARD_ID_FST_01_00         0x613870a9
#define BOARD_ID_FST_01            0x696886af
#define BOARD_ID_FST_01G           0x8801277f
#define BOARD_ID_FST_01SZ          0x7e6fb084
#define BOARD_ID_MAPLE_MINI        0x7a445272
#define BOARD_ID_OLIMEX_STM32_H103 0xf92bb594
#define BOARD_ID_STBEE_MINI        0x1f341961
#define BOARD_ID_STBEE             0x945c37e8
#define BOARD_ID_STM32_PRIMER2     0x21e5798d
#define BOARD_ID_STM8S_DISCOVERY   0x2f0976bb
#define BOARD_ID_ST_DONGLE         0x2cd4e471
#define BOARD_ID_ST_NUCLEO_F103    0x9b87c16d
#define BOARD_ID_NITROKEY_START    0xad1e7ebd
#define BOARD_ID_GNUKEY_DS         0x67ee65a3

extern const uint8_t sys_version[8];
#if defined(USE_SYS3) || defined(USE_SYS_BOARD_ID)
extern const uint32_t sys_board_id;
extern const uint8_t sys_board_name[];
# define SYS_BOARD_ID sys_board_id
#else
# define SYS_BOARD_ID BOARD_ID
#endif

static inline const uint8_t *
unique_device_id (void)
{
  /* GD32VF103/STM32F103 has 96-bit unique device identifier */
  const uint8_t *addr = (const uint8_t *)0x1ffff7e8;

  return addr;
}

void set_led (int on);

void flash_unlock (void);

int flash_program_halfword (uintptr_t addr, uint16_t data);

int flash_erase_page (uintptr_t addr);

int flash_check_blank (const uint8_t *p_start, size_t size);

int flash_write (uintptr_t dst_addr, const uint8_t *src, size_t len);

int flash_protect (void);

void __attribute__((noreturn))
flash_erase_all_and_exec (void (*entry)(void));

void usb_lld_sys_init (void);

void usb_lld_sys_shutdown (void);

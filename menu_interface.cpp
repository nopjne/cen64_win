#include <Windows.h>
#include <stdint.h>
#include <stdio.h>

#define DAISY_BASE_REGISTER 0x08040000
#define DAISY_MENU_INTERRUPT (1 << EXTI3_IRQn)
enum DAISY_FW_FUNCTION {
    GET_FW_VERSION,
    ENABLE_MENU_FUNCTIONS,
    DISABLE_MENU_FUNCTIONS,
    SD_CARD_READ_SECTOR,
    SD_CARD_WRITE_SECTOR,
    UPLOAD_ROM,
    UPLOAD_ROM_EX,
    SET_SAVE_TYPE,
};

#define DAISY_STATUS_BIT_ROM_LOADING 0x00000001
#define DAISY_STATUS_BIT_SD_BUSY     0x00000002
#define DAISY_STATUS_BIT_MENU_MODE   0x00000004
#define DAISY_STATUS_BIT_DMA_BUSY    0x00000008
#define DAISY_STATUS_BIT_DMA_TIMEOUT 0x00000010
#define DAISY_STATUS_BIT_SD_ERROR    0x00000020
#define DCFG_FIFO_TO_RAM 0
#define DCFG_RAM_TO_FIFO 1

enum DAISY_REGISTERS {
    REG_STATUS,
    REG_EXECUTE_FUNCTION,
    REG_FUNCTION_PARAMETER,
    REG_DMA_CFG,
    REG_DMA_LEN,
    REG_DMA_RAM_ADDR, // There are 512 bytes past this register to receive or send DMA.
    REG_DMA_DATA
};

#define SAVE_TYPE_OFF 0
#define SAVE_TYPE_SRAM 1
#define SAVE_TYPE_SRAM96 2
#define SAVE_TYPE_EEP4k 3
#define SAVE_TYPE_EEP16k 4
#define SAVE_TYPE_FLASH 5

//extern uint32_t* MenuBase;
#define F_OK 0
BYTE* FlashRamStorage[1024];
#if 0


void InitMenuFunctions(void);

#include "daisy_seed.h"
#include "stm32h7xx_hal.h"
#include "sys/system.h"
#include "stm32h7xx_hal_dma.h"
#include "stm32h7xx_ll_tim.h"
#include "stm32h7xx_ll_bus.h"
#include "n64common.h"
#include "daisydrive64.h"
#include "flashram.h"
#include "diskio.h"
#include "menu.h"
#include "menurom.h"

void LoadRom(const char* Name);

using namespace daisy;
SdmmcHandler::Config sd_cfg;
extern SdmmcHandler   sd;
#endif

uint32_t* MenuBase = (uint32_t*)FlashRamStorage;

void InitMenuFunctions(void)
{
#if 0
    // Setup interrupt priority.
    HAL_NVIC_SetPriority(EXTI3_IRQn, 11, 0);
    HAL_NVIC_EnableIRQ(EXTI3_IRQn);

    // Enable software interrupts.
    EXTI->IMR1 |= DAISY_MENU_INTERRUPT;
    EXTI->EMR1 |= DAISY_MENU_INTERRUPT;
    //memcpy(ram, menurom, sizeof(menurom));
#endif
}

int retCode = 0;
HANDLE device = NULL;

void EnableMenu(void) {
#if 0
    sd_cfg.Defaults();
    sd_cfg.speed = SdmmcHandler::Speed::VERY_FAST;
    sd.Init(sd_cfg);

    // Mount SD Card
    DSTATUS Status = disk_initialize(0);
    if (RES_OK != Status) {
        BlinkAndDie(500, 100);
    }
#endif
    
    if (device == NULL) {
    device = CreateFile("\\\\.\\E:",    // Drive to open
    //device = CreateFile("\\.\Disk 1",    // Drive to open
        GENERIC_READ,           // Access mode
        FILE_SHARE_WRITE,
        //FILE_SHARE_READ | FILE_SHARE_WRITE,        // Share Mode
        NULL,                   // Security Descriptor
        OPEN_EXISTING,          // How to create
        0,                      // File attributes
        NULL);                  // Handle to template

        if (device == NULL) {
            volatile int a = 0;
            a = 1;
        }

    }
}

#define DRESULT DWORD
DRESULT disk_read(
    BYTE pdrv,		/* Physical drive nmuber to identify the drive */
    BYTE* buff,		/* Data buffer to store read data */
    DWORD sector,	        /* Sector address in LBA */
    UINT count		/* Number of sectors to read */
)
{
    DWORD bytesRead;

    // low level diskread of d: drive.
    if (device == INVALID_HANDLE_VALUE)
    {
        return 1;
    }

    LONG sectorOffsetLow = ((uint64_t)sector * 512);
    LONG sectorOffsetHigh = ((uint64_t)sector * 512) >> 32;
    SetFilePointer(device, sectorOffsetLow, &sectorOffsetHigh, FILE_BEGIN);
    //SetFilePointer(device, sectorOffsetLow, NULL, FILE_BEGIN);

    if (!ReadFile(device, buff, 512 * count, &bytesRead, NULL))
    {
        return 1;
    }

    return F_OK;
}

DRESULT disk_write(
    BYTE pdrv,		/* Physical drive nmuber to identify the drive */
    const BYTE* buff,	/* Data to be written */
    DWORD sector,		/* Sector address in LBA */
    UINT count        	/* Number of sectors to write */
    )
{

   //if (device != NULL) {
   //    CloseHandle(device);
   //    device = CreateFile("\\\\.\\D:",    // Drive to open
   //    //device = CreateFile("\\.\Disk 1",    // Drive to open
   //        0,           // Access mode
   //        0,
   //        //FILE_SHARE_READ | FILE_SHARE_WRITE,        // Share Mode
   //        NULL,                   // Security Descriptor
   //        OPEN_EXISTING,          // How to create
   //        FILE_ATTRIBUTE_NORMAL,                      // File attributes
   //        NULL);                  // Handle to template
   //
   //    if (device == INVALID_HANDLE_VALUE) {
   //        volatile int a = 0;
   //        a = 1;
   //    }
   //
   //}

    // low level diskwrite of d: drive.
    LONG sectorOffsetLow = ((uint64_t)sector * 512);
    LONG sectorOffsetHigh = ((uint64_t)sector * 512) >> 32;
    SetFilePointer(device, sector * 512, &sectorOffsetHigh, FILE_BEGIN);

    DWORD bytesWritten = 0;
    if (!WriteFile(device, buff, 512 * count, &bytesWritten, NULL))
    {
        return 0;
    }

    if (bytesWritten != 512 * count) {
        return 1;
    }

    //if (device != NULL) {
    //    CloseHandle(device);
    //    device = CreateFile("\\\\.\\D:",    // Drive to open
    //    //device = CreateFile("\\.\Disk 1",    // Drive to open
    //        GENERIC_READ,           // Access mode
    //        FILE_SHARE_READ,
    //        //FILE_SHARE_READ | FILE_SHARE_WRITE,        // Share Mode
    //        NULL,                   // Security Descriptor
    //        OPEN_EXISTING,          // How to create
    //        0,                      // File attributes
    //        NULL);                  // Handle to template
    //
    //    if (device == NULL) {
    //        volatile int a = 0;
    //        a = 1;
    //    }
    //
    //}

    return F_OK;
}

extern "C" struct rom_file cart;
extern "C" int close_rom_file(const struct rom_file* file);
extern "C" int open_rom_file(const char* path, struct rom_file* file);

void LoadRom(char *Name)
{
    // Swap the rom that is presented on the PI bus.
    CloseHandle(device);
    char FileName[MAX_PATH];
    sprintf(FileName, "d:\\%s", Name);
    for (size_t i = 0; i < MAX_PATH; i += 1) {
        if (FileName[i] == '/') {
            FileName[i] = '\\';
        }
    }

    close_rom_file(&cart);
    open_rom_file(FileName, &cart);
}

inline void HandleExecute(void)
{
    switch (MenuBase[REG_EXECUTE_FUNCTION]) {
    case GET_FW_VERSION:
        MenuBase[REG_STATUS] |= DAISY_STATUS_BIT_DMA_BUSY;
        MenuBase[REG_FUNCTION_PARAMETER] = 0x666;
        MenuBase[REG_STATUS] &= ~DAISY_STATUS_BIT_DMA_BUSY;
        break;
    case ENABLE_MENU_FUNCTIONS:
        MenuBase[REG_STATUS] |= DAISY_STATUS_BIT_SD_BUSY;
        EnableMenu();
        MenuBase[REG_STATUS] &= ~DAISY_STATUS_BIT_SD_BUSY;
        break;
    case DISABLE_MENU_FUNCTIONS:
        MenuBase[REG_STATUS] &= ~DAISY_STATUS_BIT_SD_BUSY;
        break;
    case SD_CARD_READ_SECTOR:
    {
        MenuBase[REG_STATUS] |= DAISY_STATUS_BIT_SD_BUSY;
        MenuBase[REG_STATUS] &= ~DAISY_STATUS_BIT_SD_ERROR;
        DRESULT result = disk_read(0, (BYTE*)&MenuBase[REG_DMA_DATA], MenuBase[REG_DMA_RAM_ADDR], 1);
        if (result != F_OK) {
            MenuBase[REG_STATUS] |= DAISY_STATUS_BIT_SD_ERROR;
        }
        MenuBase[REG_STATUS] &= ~DAISY_STATUS_BIT_SD_BUSY;
    }
    break;
    case SD_CARD_WRITE_SECTOR:
    {
        MenuBase[REG_STATUS] |= DAISY_STATUS_BIT_SD_BUSY;
        MenuBase[REG_STATUS] &= ~DAISY_STATUS_BIT_SD_ERROR;
        DRESULT result = disk_write(0, (BYTE*)&MenuBase[REG_DMA_DATA], MenuBase[REG_DMA_RAM_ADDR], 1);
        if (result != F_OK) {
            MenuBase[REG_STATUS] |= DAISY_STATUS_BIT_SD_ERROR;
        }

        MenuBase[REG_STATUS] &= ~DAISY_STATUS_BIT_SD_BUSY;
    }
    break;
    case UPLOAD_ROM:
        MenuBase[REG_STATUS] |= DAISY_STATUS_BIT_SD_BUSY;
        LoadRom((char*)&MenuBase[REG_DMA_DATA]);
        MenuBase[REG_STATUS] &= ~DAISY_STATUS_BIT_SD_BUSY;
        // TODO: cause the reset here, notify n64 that load is done.

        break;
    case UPLOAD_ROM_EX:
        MenuBase[REG_STATUS] |= DAISY_STATUS_BIT_SD_BUSY;
        LoadRom((char*)&MenuBase[REG_DMA_DATA]);
        MenuBase[REG_STATUS] &= ~DAISY_STATUS_BIT_SD_BUSY;
    break;
    case SET_SAVE_TYPE:

        break;
    }

    MenuBase[REG_STATUS] &= ~(DAISY_STATUS_BIT_DMA_BUSY | DAISY_STATUS_BIT_SD_BUSY);
}

extern "C" void HandleMenuRead(DWORD addr, DWORD* data)
{
    addr -= DAISY_BASE_REGISTER;
    addr >>= 2;
    *data = MenuBase[addr];
}

extern "C" void HandleMenuWrite(void *Menu, DWORD addr, DWORD data)
{
    addr -= DAISY_BASE_REGISTER;
    addr >>= 2;
    MenuBase[addr] = data;
    if (addr == REG_EXECUTE_FUNCTION) {
        MenuBase[REG_STATUS] |= DAISY_STATUS_BIT_DMA_BUSY | DAISY_STATUS_BIT_SD_BUSY;
        if (MenuBase[REG_EXECUTE_FUNCTION] == ENABLE_MENU_FUNCTIONS) {
            memcpy(Menu, &MenuBase, sizeof(Menu));
        }

        HandleExecute();
    }
}

#if 0
extern "C"
ITCM_FUNCTION
void EXTI3_IRQHandler(void)
{
    EXTI->PR1 = DAISY_MENU_INTERRUPT;
    HandleExecute();
}
#endif


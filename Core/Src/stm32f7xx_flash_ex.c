/**
 * NOTE:
 *      Every bit in the flash memory is set to 1 by default. And it can be changed from 1 to 0 only.
 *      To change it back to 1, the entire sector has to be erased.
 */

#include "stm32f7xx_flash_ex.h"
#include "grbl.h"

#define FLASH_GET_NEXT_SECTOR_ADDR() (currentFlashSectorAddress == FLASH_BASE_ADDR_1 ? FLASH_BASE_ADDR_2 : FLASH_BASE_ADDR_1)

extern volatile uint32_t currentFlashSectorAddress;

uint8_t currentVersion = 0;

/**
 * Function Prototypes
 */
HAL_StatusTypeDef flashEaraseSector(uint32_t sector, uint32_t num_sectors);

uint32_t flashSearchCurrentSectorAddress()
{
    // get first byte of sectors to determine which sector is going to be used
    uint8_t version1 = flashGetByte(FLASH_BASE_ADDR_1);
    uint8_t version2 = flashGetByte(FLASH_BASE_ADDR_2);

    if (version1 == 0xFF)
    {
        // flashEaraseSector(FLASH_SECTOR_NUM_1, 1);
        currentFlashSectorAddress = FLASH_BASE_ADDR_2;
        currentVersion = version2;
    }
    
    if (version2 == 0xFF)
    {
        // flashEaraseSector(FLASH_SECTOR_NUM_2, 1);
        currentFlashSectorAddress = FLASH_BASE_ADDR_1;
        currentVersion = version1;
    }

    if (version1 != 0xFF && version2 != 0xFF)
    {
        // both sectors are used
        // check which sector has the latest version.
        if (version1 > version2)
        {
            flashEaraseSector(FLASH_SECTOR_NUM_2, 1);
            currentFlashSectorAddress = FLASH_BASE_ADDR_1;
            currentVersion = version1;
        }
        else
        {
            flashEaraseSector(FLASH_SECTOR_NUM_1, 1);
            currentFlashSectorAddress = FLASH_BASE_ADDR_2;
            currentVersion = version2;
        }
    }

    return currentFlashSectorAddress;
}

void flashInit()
{
    flashSearchCurrentSectorAddress();
}

HAL_StatusTypeDef flashPutByte(uint32_t address, uint8_t data)
{
    HAL_FLASH_Unlock();

    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS); 

    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, address, (uint64_t)data) == HAL_OK)
    {
        HAL_FLASH_Lock(); 
        return HAL_OK;
    }
    else
    {
        HAL_FLASH_Lock(); 
        return HAL_FLASH_GetError ();
    }
}

uint8_t flashGetByte(uint32_t address)
{
  return *(__IO uint8_t*)address;
}

void flashMemcpyToEepromWithChecksumPrime(unsigned int destination, char *source, unsigned int size) {
  unsigned char checksum = 0;
  for(; size > 0; size--) { 
	//checksum = (checksum << 1) || (checksum >> 7);
	// The above line is incorrect. It should be:
    checksum = (checksum << 1) | (checksum >> 7);
    checksum += *source;
    flashPutByte(destination++, *(source++)); 
  }
  flashPutByte(destination, checksum);
}

HAL_StatusTypeDef flashEaraseSector(uint32_t sector, uint32_t num_sectors)
{
    HAL_FLASH_Unlock();

    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS); 

    FLASH_EraseInitTypeDef EraseInitStruct;
    EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
    EraseInitStruct.Sector = sector;
    EraseInitStruct.NbSectors = num_sectors;
    EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;

    uint32_t SectorError = 0;
    if (HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) == HAL_OK)
    {
        HAL_FLASH_Lock(); 
        return HAL_OK;
    }
    else
    {
        HAL_FLASH_Lock(); 
        return HAL_FLASH_GetError ();
    }
}

HAL_StatusTypeDef flashWriteVersion(uint8_t new_version)
{
    currentVersion = new_version;

    return HAL_OK;
}

HAL_StatusTypeDef flashWriteGlobalSettings(uint32_t target_address, uint8_t *data, uint32_t size)
{
    uint32_t nextSectorAddress = FLASH_GET_NEXT_SECTOR_ADDR();
    uint8_t settings_buf[sizeof(settings_t)+1];
    uint8_t* settings;

    // write version first
    flashPutByte((nextSectorAddress + EEPROM_ADDR_VERSION_OFFSET), currentVersion);

    // check if reading data from current sector is needed
    if (EEPROM_ADDR_GLOBAL != target_address)
    {
        // need to read data from current sector first
        settings = settings_buf;
        if (!(memcpy_from_eeprom_with_checksum((char*)settings, EEPROM_ADDR_GLOBAL, sizeof(settings_t))))
        {
            // failed to read data from current sector
            return HAL_ERROR;
        }
    }
    else 
    {
        settings = data;
    }

    // write data to next sector
    flashMemcpyToEepromWithChecksumPrime((nextSectorAddress + EEPROM_ADDR_GLOBAL_OFFSET), (char*)settings, sizeof(settings_t));

    return HAL_OK;
}

HAL_StatusTypeDef flashWriteParameters(uint32_t target_address, uint8_t *data, uint32_t size)
{
    uint32_t nextSectorAddress = FLASH_GET_NEXT_SECTOR_ADDR();
    uint8_t coordSelect;

    for (coordSelect = 0; coordSelect <= SETTING_INDEX_NCOORD; coordSelect++)
    {
        uint32_t addrOffset = coordSelect * (sizeof(float) * N_AXIS + 1);
        uint32_t addr = addrOffset + EEPROM_ADDR_PARAMETERS;
        uint8_t coordData_buf[sizeof(float) * N_AXIS + 1];
        uint8_t* coordData;

        if (addr != target_address)
        {
            // need to read data from current sector first
            coordData = coordData_buf;
            if (!(memcpy_from_eeprom_with_checksum((char*)coordData, addr, sizeof(float) * N_AXIS)))
            {
                // failed to read data from current sector
                return HAL_ERROR;
            }
        }
        else 
        {
            coordData = data;
        }

        // write data to next sector
        flashMemcpyToEepromWithChecksumPrime((nextSectorAddress + EEPROM_ADDR_PARAMETERS_OFFSET + addrOffset), (char*)coordData, sizeof(float) * N_AXIS);
    }
    
    return HAL_OK;
}

HAL_StatusTypeDef flashWriteStartupBlock(uint32_t target_address, uint8_t *data, uint32_t size)
{
    uint32_t nextSectorAddress = FLASH_GET_NEXT_SECTOR_ADDR();
    
    for (uint8_t n = 0; n < N_STARTUP_LINE; n++)
    {
        uint32_t addrOffset = n*(LINE_BUFFER_SIZE+1);
        uint32_t addr = addrOffset + EEPROM_ADDR_STARTUP_BLOCK;
        uint8_t startupLineData_buf[LINE_BUFFER_SIZE];
        uint8_t* startupLineData;

        if (addr != target_address)
        {
            // need to read data from current sector first
            startupLineData = startupLineData_buf;
            if (!(memcpy_from_eeprom_with_checksum((char*)startupLineData, addr, LINE_BUFFER_SIZE)))
            {
                // failed to read data from current sector
                return HAL_ERROR;
            }
        }
        else 
        {
            startupLineData = data;
        }

        // write data to next sector
        flashMemcpyToEepromWithChecksumPrime((nextSectorAddress + EEPROM_ADDR_STARTUP_BLOCK_OFFSET + addrOffset), (char*)startupLineData, LINE_BUFFER_SIZE);
    }

    return HAL_OK;
}

HAL_StatusTypeDef flashWriteBuildInfo(uint32_t target_address, uint8_t *data, uint32_t size)
{
    uint32_t nextSectorAddress = FLASH_GET_NEXT_SECTOR_ADDR();
    uint8_t buildInfo_buf[LINE_BUFFER_SIZE];
    uint8_t* buildInfo;

    // check if reading data from current sector is needed
    if (EEPROM_ADDR_BUILD_INFO != target_address)
    {
        // need to read data from current sector first
        buildInfo = buildInfo_buf;
        if (!(memcpy_from_eeprom_with_checksum((char*)buildInfo, EEPROM_ADDR_BUILD_INFO, LINE_BUFFER_SIZE)))
        {
            // failed to read data from current sector
            return HAL_ERROR;
        }
    }
    else 
    {
        buildInfo = data;
    }

    // write data to next sector
    flashMemcpyToEepromWithChecksumPrime((nextSectorAddress + EEPROM_ADDR_BUILD_INFO_OFFSET), (char*)buildInfo, LINE_BUFFER_SIZE);

    return HAL_OK;
}

void flashMemcpyToEepromWithChecksum(uint32_t destination, char *source, uint32_t size)
{
    // determine the next sector address
    uint32_t nextSectorAddress = FLASH_GET_NEXT_SECTOR_ADDR();

    // write data to next sector for each category of settings
    // flashWriteVersion(destination, (uint8_t)(source[0])); // Note: version update is triggered by the write_global_settings function
    flashWriteGlobalSettings(destination, (uint8_t*)source, size);
    flashWriteParameters(destination, (uint8_t*)source, size);
    flashWriteStartupBlock(destination, (uint8_t*)source, size);
    flashWriteBuildInfo(destination, (uint8_t*)source, size);

    // erase the current sector
    if(currentFlashSectorAddress == FLASH_BASE_ADDR_1)
    {
        flashEaraseSector(FLASH_SECTOR_NUM_1, 1);
    }
    else if(currentFlashSectorAddress == FLASH_BASE_ADDR_2)
    {
        flashEaraseSector(FLASH_SECTOR_NUM_2, 1);
    }

    // update the current sector address
    currentFlashSectorAddress = nextSectorAddress;
}

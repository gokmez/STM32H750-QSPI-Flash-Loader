/*
 * qspi.c
 *
 *  Created on: Nov 10, 2025
 *      Author: steve
 */


#include "qspi.h"


// Public entrypoint
void MT25Q_RunSelfTest(void)
{
    printf("\n[MT25Q] Self-test start\n");

    // 0) Hardware sanity: try to reset the flash
    if (QSPI_SendSimple(CMD_RESET_ENABLE) != HAL_OK || QSPI_SendSimple(CMD_RESET_MEMORY) != HAL_OK) {
        printf("[MT25Q] Reset command failed\n");
        return;
    }
    HAL_Delay(1); // tRST is small but give it a moment

    // 1) Read JEDEC ID
    uint8_t id[3] = {0};
    if (QSPI_ReadID(id) != HAL_OK) {
        printf("[MT25Q] Read ID failed\n");
        return;
    }
    printf("[MT25Q] JEDEC ID: %02X %02X %02X\n", id[0], id[1], id[2]);
    // Expect manufacturer=Micron (0x20), device type/capacity vary by exact sub-family.
    // We just print what we see.

    // 2) Enter 4-byte address mode (mandatory for >16MB addressing on 512Mbit parts)
    if (QSPI_Enable4ByteAddressing() != HAL_OK) {
        printf("[MT25Q] Enter 4-byte address mode failed\n");
        return;
    }
    // Confirm not busy
    if (QSPI_WaitWhileBusy(100) != HAL_OK) {
        printf("[MT25Q] Device stayed busy after 4B mode\n");
        return;
    }

    // 3) Erase a 4KB subsector at TEST_ADDR
    printf("[MT25Q] Erasing 4KB @ 0x%08lX ...\n", (unsigned long)TEST_ADDR);
    if (QSPI_Erase4K_4B(TEST_ADDR) != HAL_OK) {
        printf("[MT25Q] 4KB erase failed\n");
        return;
    }
    if (QSPI_WaitWhileBusy(5000) != HAL_OK) { // subsector erase can take a while
        printf("[MT25Q] Timeout waiting for erase\n");
        return;
    }
    // quick verify erased (read back and check all 0xFF)
    uint8_t verify_buf[TEST_PAGE_SIZE];
    if (QSPI_Read_4B(TEST_ADDR, verify_buf, sizeof(verify_buf)) != HAL_OK) {
        printf("[MT25Q] Read after erase failed\n");
        return;
    }
    for (size_t i = 0; i < sizeof(verify_buf); ++i) {
        if (verify_buf[i] != 0xFF) {
            printf("[MT25Q] Erase verify failed @+%lu (0x%02X)\n", (unsigned long)i, verify_buf[i]);
            return;
        }
    }
    printf("[MT25Q] Erase OK\n");

    // 4) Program one page with a pattern
    uint8_t out[TEST_PAGE_SIZE];
    for (uint32_t i = 0; i < TEST_PAGE_SIZE; ++i) out[i] = (uint8_t)(i ^ 0xA5);
    printf("[MT25Q] Programming %u bytes @ 0x%08lX ...\n", TEST_PAGE_SIZE, (unsigned long)TEST_ADDR);
    if (QSPI_PageProgram_4B(TEST_ADDR, out, TEST_PAGE_SIZE) != HAL_OK) {
        printf("[MT25Q] Page program failed\n");
        return;
    }
    if (QSPI_WaitWhileBusy(100) != HAL_OK) {
        printf("[MT25Q] Timeout waiting for program\n");
        return;
    }

    // 5) Read back & compare
    uint8_t in[TEST_PAGE_SIZE];
    if (QSPI_Read_4B(TEST_ADDR, in, TEST_PAGE_SIZE) != HAL_OK) {
        printf("[MT25Q] Read after program failed\n");
        return;
    }
    if (memcmp(out, in, TEST_PAGE_SIZE) != 0) {
        for (uint32_t i = 0; i < TEST_PAGE_SIZE; ++i) {
            if (out[i] != in[i]) {
                printf("[MT25Q] Verify mismatch @+%lu: wrote %02X read %02X\n",
                       (unsigned long)i, out[i], in[i]);
                break;
            }
        }
        return;
    }

    printf("[MT25Q] Program/verify OK. Connection & basic ops look good ✅\n");
}

// ------------------- Low-level helpers -------------------

static HAL_StatusTypeDef QSPI_SendSimple(uint8_t instruction)
{
    QSPI_CommandTypeDef cmd = {0};
    cmd.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
    cmd.Instruction       = instruction;
    cmd.AddressMode       = QSPI_ADDRESS_NONE;
    cmd.DataMode          = QSPI_DATA_NONE;
    cmd.DummyCycles       = 0;
    cmd.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;
    cmd.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
    return HAL_QSPI_Command(&hqspi, &cmd, QSPI_TIMEOUT);
}

static HAL_StatusTypeDef QSPI_ReadStatus(uint8_t *sr)
{
    QSPI_CommandTypeDef cmd = {0};
    cmd.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
    cmd.Instruction       = CMD_READ_STATUS_REG;
    cmd.AddressMode       = QSPI_ADDRESS_NONE;
    cmd.DataMode          = QSPI_DATA_1_LINE;
    cmd.NbData            = 1;
    cmd.DummyCycles       = 0;
    cmd.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;
    if (HAL_QSPI_Command(&hqspi, &cmd, QSPI_TIMEOUT) != HAL_OK) return HAL_ERROR;
    return HAL_QSPI_Receive(&hqspi, sr, QSPI_TIMEOUT);
}

static HAL_StatusTypeDef QSPI_WaitWhileBusy(uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();
    uint8_t sr = 0;
    do {
        if (QSPI_ReadStatus(&sr) != HAL_OK) return HAL_ERROR;
        if ((sr & SR_WIP_MASK) == 0) return HAL_OK;
    } while ((HAL_GetTick() - start) < timeout_ms);
    return HAL_TIMEOUT;
}

static HAL_StatusTypeDef QSPI_WriteEnable(void)
{
    if (QSPI_SendSimple(CMD_WRITE_ENABLE) != HAL_OK) return HAL_ERROR;

    // Confirm WEL set
    uint8_t sr = 0;
    uint32_t start = HAL_GetTick();
    do {
        if (QSPI_ReadStatus(&sr) != HAL_OK) return HAL_ERROR;
        if (sr & SR_WEL_MASK) return HAL_OK;
    } while ((HAL_GetTick() - start) < QSPI_TIMEOUT);
    return HAL_TIMEOUT;
}

static HAL_StatusTypeDef QSPI_ReadID(uint8_t id[3])
{
    QSPI_CommandTypeDef cmd = {0};
    cmd.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
    cmd.Instruction       = CMD_READ_ID;
    cmd.AddressMode       = QSPI_ADDRESS_NONE;
    cmd.DataMode          = QSPI_DATA_1_LINE;
    cmd.NbData            = 3;
    cmd.DummyCycles       = 0;
    cmd.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

    if (HAL_QSPI_Command(&hqspi, &cmd, QSPI_TIMEOUT) != HAL_OK) return HAL_ERROR;
    return HAL_QSPI_Receive(&hqspi, id, QSPI_TIMEOUT);
}

static HAL_StatusTypeDef QSPI_Enable4ByteAddressing(void)
{
    // Enter 4-byte address mode (0xB7). No data, no address.
    if (QSPI_WriteEnable() != HAL_OK){
    	return HAL_ERROR;
    }
    if (QSPI_SendSimple(CMD_ENABLE_4BYTE_ADDR) != HAL_OK){
    	return HAL_ERROR;
    }
    return HAL_OK;
}

static HAL_StatusTypeDef QSPI_Erase4K_4B(uint32_t addr)
{
    if (QSPI_WriteEnable() != HAL_OK){
    	return HAL_ERROR;
    }

    QSPI_CommandTypeDef cmd = {0};
    cmd.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
    cmd.Instruction       = CMD_SUBSECTOR_ERASE_4K_4B; // 4-byte opcode 0x21
    cmd.AddressMode       = QSPI_ADDRESS_1_LINE;
    cmd.AddressSize       = QSPI_ADDRESS_32_BITS;
    cmd.Address           = addr;
    cmd.DataMode          = QSPI_DATA_NONE;
    cmd.DummyCycles       = 0;
    cmd.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

    return HAL_QSPI_Command(&hqspi, &cmd, QSPI_TIMEOUT);
}

static HAL_StatusTypeDef QSPI_PageProgram_4B(uint32_t addr, const uint8_t *data, uint32_t size)
{
    // size must not cross a page boundary, typical page = 256 bytes
    if (size == 0 || size > TEST_PAGE_SIZE){
    	return HAL_ERROR;
    }

    if (QSPI_WriteEnable() != HAL_OK){
    	return HAL_ERROR;
    }

    QSPI_CommandTypeDef cmd = {0};
    cmd.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
    cmd.Instruction       = CMD_PAGE_PROGRAM_4B; // 0x12
    cmd.AddressMode       = QSPI_ADDRESS_1_LINE;
    cmd.AddressSize       = QSPI_ADDRESS_32_BITS;
    cmd.Address           = addr;
    cmd.DataMode          = QSPI_DATA_1_LINE;
    cmd.NbData            = size;
    cmd.DummyCycles       = 0;
    cmd.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

    if (HAL_QSPI_Command(&hqspi, &cmd, QSPI_TIMEOUT) != HAL_OK) return HAL_ERROR;

    return HAL_QSPI_Transmit(&hqspi, (uint8_t*)data, QSPI_TIMEOUT);
}

static HAL_StatusTypeDef QSPI_Read_4B(uint32_t addr, uint8_t *data, uint32_t size)
{
    QSPI_CommandTypeDef cmd = {0};
    cmd.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
    cmd.Instruction       = CMD_READ_DATA_4B; // 0x13 (slow read, no dummy)
    cmd.AddressMode       = QSPI_ADDRESS_1_LINE;
    cmd.AddressSize       = QSPI_ADDRESS_32_BITS;
    cmd.Address           = addr;
    cmd.DataMode          = QSPI_DATA_1_LINE;
    cmd.NbData            = size;
    cmd.DummyCycles       = 0;
    cmd.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

    if (HAL_QSPI_Command(&hqspi, &cmd, QSPI_TIMEOUT) != HAL_OK) return HAL_ERROR;
    return HAL_QSPI_Receive(&hqspi, data, QSPI_TIMEOUT);
}


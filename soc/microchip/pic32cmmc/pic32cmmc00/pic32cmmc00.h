/*
 * Copyright (c) 2025 GP Orcullo
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _SOC_MICROCHIP_PIC32CMMC00_H_
#define _SOC_MICROCHIP_PIC32CMMC00_H_

#define GCLK          DT_REG_ADDR(DT_INST(0, atmel_sam0_gclk))
#define GCLK_GENCTRL0 (GCLK + 0x20)
#define GCLK_SYNCBUSY (GCLK + 0x04)

#define GCLK_GENCTRL_SRC_MASK GENMASK(4, 0)
#define GCLK_GENCTRL_GENEN    BIT(8)
#define GCLK_GENCTRL_DIV_MASK GENMASK(31, 16)

#define GCLK_PCHCTRL_GEN(n) FIELD_PREP(GENMASK(3, 0), n)
#define GCLK_PCHCTRL_CHEN   BIT(6)

#define GCLK_GENCTRL_SRC_OSC48M_VAL 6
#define GCLK_PCHCTRL_GEN_GCLK0      0
#define GCLK_SYNCBUSY_GENCTRL0_BIT  2

#define MCLK_CPUDIV                 (DT_REG_ADDR(DT_INST(0, atmel_sam0_mclk)) + 0x04)
#define MCLK_CPUDIV_CPUDIV_DIV1_VAL 1

#define NVMCTRL_CTRLB          (DT_REG_ADDR(DT_INST(0, atmel_sam0_nvmctrl)) + 0x04)
#define NVMCTRL_CTRLB_RWS_MASK GENMASK(4, 1)

#define NVM_SW_CAL_ADDR        0x00806020
#define NVM_SW_CAL_CAL48M_MASK GENMASK64(40, 19)

#define OSCCTRL                DT_REG_ADDR(DT_PATH(soc, oscctrl_40001000))
#define OSCCTRL_STATUS         (OSCCTRL + 0x0C)
#define OSCCTRL_OSC48MDIV      (OSCCTRL + 0x15)
#define OSCCTRL_OSC48MSYNCBUSY (OSCCTRL + 0x18)
#define OSCCTRL_CAL48M         (OSCCTRL + 0x38)

#define OSCCTRL_CAL48M_MASK                  GENMASK(21, 0)
#define OSCCTRL_OSC48MSYNCBUSY_OSC48MDIV_BIT 2
#define OSCCTRL_STATUS_OSC48MRDY_BIT         4

#endif /* _SOC_MICROCHIP_PIC32CMMC00_H_ */

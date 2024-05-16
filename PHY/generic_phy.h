
/**
 * @file generic_phy.h
 * @brief Generic Phy Register Definations
 * @autho Disen-Shaw <Disen-Shaw@gmail.com>
 * @date 2024-05-16
 * @version lts 1.0
 */

#ifndef __GENERIC_PHY_H__
#define __GENERIC_PHY_H__

#include <stdint.h>

/**
 * @defgroup Generic_PHY_Address
 * @brief Generic Register Address 
 * @{
 */

/**
 * @brief Basic Control Register
 */
#define GEN_PHY_REGADDR_BCR				(0x00)

/**
 * @brief Basic Status Register
 */
#define GEN_PHY_REGADDR_BSR				(0x01)

/**
 * @brief PHY Identify Register 1
 */
#define GEN_PHY_REGADDR_IDENTIFY1		(0x02)

/**
 * @brief PHY Identify Register 2
 */
#define GEN_PHY_REGADDR_IDENTIFY2		(0x03)

/**
 * @brief Auto-Negotiation Advertisement Register
 */
#define GEN_PHY_REGADDR_ANAR			(0x04)

/**
 * @brief Auto-Negotiation Link Partner Ability Register
 */
#define GEN_PHY_REGADDR_ANLPAR			(0x05)

/**
 * @brief Auto-Negotiation Expansion Register
 */
#define GEN_PHY_REGADDR_ANER			(0x06)

/**
 * @brief Auto-Negotiation Next Page Transmit Register
 */
#define GEN_PHY_REGADDR_ANNPTR			(0x07)

/**
 * @brief Auto-Negotiation Next Page Receive Register
 */
#define GEN_PHY_REGADDR_ANNPRR			(0x08)

/**
 * @brief Master-Slave Control Register
 */
#define GEN_PHY_REGADDR_MSCR			(0x09)

/**
 * @brief Master-Slave Status Register
 */
#define GEN_PHY_REGADDR_MSSR			(0x0A)

/**
 * @brief PMD Auto-Negotiation Expanded Capability Register
 */
#define GEN_PHY_REGADDR_PMDANECR		(0x0B)

/**
 * @brief PMD Auto-Negotiation Next Page Receive Register
 */
#define GEN_PHY_REGADDR_PMDNPRR			(0x0C)

/**
 * @brief MMD Access Address Control Register
 */
#define GEN_PHY_REGADDR_MMDACR			(0x0D)

/**
 * @brief MMD Access Address Data Register
 */
#define GEN_PHY_REGADDR_AADR			(0x0E)

/**
 * @brief Extended Status Register
 */
#define GEN_PHY_REGADDR_ESR				(0x0F)

/**
 * @}
 */

#endif /* __GENERIC_PHY_H__ */

/*******************************************************************************
* File Name: mode_switch2.h  
* Version 2.20
*
* Description:
*  This file contains Pin function prototypes and register defines
*
* Note:
*
********************************************************************************
* Copyright 2008-2015, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
*******************************************************************************/

#if !defined(CY_PINS_mode_switch2_H) /* Pins mode_switch2_H */
#define CY_PINS_mode_switch2_H

#include "cytypes.h"
#include "cyfitter.h"
#include "cypins.h"
#include "mode_switch2_aliases.h"

/* APIs are not generated for P15[7:6] */
#if !(CY_PSOC5A &&\
	 mode_switch2__PORT == 15 && ((mode_switch2__MASK & 0xC0) != 0))


/***************************************
*        Function Prototypes             
***************************************/    

/**
* \addtogroup group_general
* @{
*/
void    mode_switch2_Write(uint8 value);
void    mode_switch2_SetDriveMode(uint8 mode);
uint8   mode_switch2_ReadDataReg(void);
uint8   mode_switch2_Read(void);
void    mode_switch2_SetInterruptMode(uint16 position, uint16 mode);
uint8   mode_switch2_ClearInterrupt(void);
/** @} general */

/***************************************
*           API Constants        
***************************************/
/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup driveMode Drive mode constants
     * \brief Constants to be passed as "mode" parameter in the mode_switch2_SetDriveMode() function.
     *  @{
     */
        #define mode_switch2_DM_ALG_HIZ         PIN_DM_ALG_HIZ
        #define mode_switch2_DM_DIG_HIZ         PIN_DM_DIG_HIZ
        #define mode_switch2_DM_RES_UP          PIN_DM_RES_UP
        #define mode_switch2_DM_RES_DWN         PIN_DM_RES_DWN
        #define mode_switch2_DM_OD_LO           PIN_DM_OD_LO
        #define mode_switch2_DM_OD_HI           PIN_DM_OD_HI
        #define mode_switch2_DM_STRONG          PIN_DM_STRONG
        #define mode_switch2_DM_RES_UPDWN       PIN_DM_RES_UPDWN
    /** @} driveMode */
/** @} group_constants */
    
/* Digital Port Constants */
#define mode_switch2_MASK               mode_switch2__MASK
#define mode_switch2_SHIFT              mode_switch2__SHIFT
#define mode_switch2_WIDTH              1u

/* Interrupt constants */
#if defined(mode_switch2__INTSTAT)
/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup intrMode Interrupt constants
     * \brief Constants to be passed as "mode" parameter in mode_switch2_SetInterruptMode() function.
     *  @{
     */
        #define mode_switch2_INTR_NONE      (uint16)(0x0000u)
        #define mode_switch2_INTR_RISING    (uint16)(0x0001u)
        #define mode_switch2_INTR_FALLING   (uint16)(0x0002u)
        #define mode_switch2_INTR_BOTH      (uint16)(0x0003u) 
    /** @} intrMode */
/** @} group_constants */

    #define mode_switch2_INTR_MASK      (0x01u) 
#endif /* (mode_switch2__INTSTAT) */


/***************************************
*             Registers        
***************************************/

/* Main Port Registers */
/* Pin State */
#define mode_switch2_PS                     (* (reg8 *) mode_switch2__PS)
/* Data Register */
#define mode_switch2_DR                     (* (reg8 *) mode_switch2__DR)
/* Port Number */
#define mode_switch2_PRT_NUM                (* (reg8 *) mode_switch2__PRT) 
/* Connect to Analog Globals */                                                  
#define mode_switch2_AG                     (* (reg8 *) mode_switch2__AG)                       
/* Analog MUX bux enable */
#define mode_switch2_AMUX                   (* (reg8 *) mode_switch2__AMUX) 
/* Bidirectional Enable */                                                        
#define mode_switch2_BIE                    (* (reg8 *) mode_switch2__BIE)
/* Bit-mask for Aliased Register Access */
#define mode_switch2_BIT_MASK               (* (reg8 *) mode_switch2__BIT_MASK)
/* Bypass Enable */
#define mode_switch2_BYP                    (* (reg8 *) mode_switch2__BYP)
/* Port wide control signals */                                                   
#define mode_switch2_CTL                    (* (reg8 *) mode_switch2__CTL)
/* Drive Modes */
#define mode_switch2_DM0                    (* (reg8 *) mode_switch2__DM0) 
#define mode_switch2_DM1                    (* (reg8 *) mode_switch2__DM1)
#define mode_switch2_DM2                    (* (reg8 *) mode_switch2__DM2) 
/* Input Buffer Disable Override */
#define mode_switch2_INP_DIS                (* (reg8 *) mode_switch2__INP_DIS)
/* LCD Common or Segment Drive */
#define mode_switch2_LCD_COM_SEG            (* (reg8 *) mode_switch2__LCD_COM_SEG)
/* Enable Segment LCD */
#define mode_switch2_LCD_EN                 (* (reg8 *) mode_switch2__LCD_EN)
/* Slew Rate Control */
#define mode_switch2_SLW                    (* (reg8 *) mode_switch2__SLW)

/* DSI Port Registers */
/* Global DSI Select Register */
#define mode_switch2_PRTDSI__CAPS_SEL       (* (reg8 *) mode_switch2__PRTDSI__CAPS_SEL) 
/* Double Sync Enable */
#define mode_switch2_PRTDSI__DBL_SYNC_IN    (* (reg8 *) mode_switch2__PRTDSI__DBL_SYNC_IN) 
/* Output Enable Select Drive Strength */
#define mode_switch2_PRTDSI__OE_SEL0        (* (reg8 *) mode_switch2__PRTDSI__OE_SEL0) 
#define mode_switch2_PRTDSI__OE_SEL1        (* (reg8 *) mode_switch2__PRTDSI__OE_SEL1) 
/* Port Pin Output Select Registers */
#define mode_switch2_PRTDSI__OUT_SEL0       (* (reg8 *) mode_switch2__PRTDSI__OUT_SEL0) 
#define mode_switch2_PRTDSI__OUT_SEL1       (* (reg8 *) mode_switch2__PRTDSI__OUT_SEL1) 
/* Sync Output Enable Registers */
#define mode_switch2_PRTDSI__SYNC_OUT       (* (reg8 *) mode_switch2__PRTDSI__SYNC_OUT) 

/* SIO registers */
#if defined(mode_switch2__SIO_CFG)
    #define mode_switch2_SIO_HYST_EN        (* (reg8 *) mode_switch2__SIO_HYST_EN)
    #define mode_switch2_SIO_REG_HIFREQ     (* (reg8 *) mode_switch2__SIO_REG_HIFREQ)
    #define mode_switch2_SIO_CFG            (* (reg8 *) mode_switch2__SIO_CFG)
    #define mode_switch2_SIO_DIFF           (* (reg8 *) mode_switch2__SIO_DIFF)
#endif /* (mode_switch2__SIO_CFG) */

/* Interrupt Registers */
#if defined(mode_switch2__INTSTAT)
    #define mode_switch2_INTSTAT            (* (reg8 *) mode_switch2__INTSTAT)
    #define mode_switch2_SNAP               (* (reg8 *) mode_switch2__SNAP)
    
	#define mode_switch2_0_INTTYPE_REG 		(* (reg8 *) mode_switch2__0__INTTYPE)
#endif /* (mode_switch2__INTSTAT) */

#endif /* CY_PSOC5A... */

#endif /*  CY_PINS_mode_switch2_H */


/* [] END OF FILE */

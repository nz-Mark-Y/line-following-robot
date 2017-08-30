/***************************************************************************
* File Name: USBUART_ep2_dma.c  
* Version 1.70
*
*  Description:
*   Provides an API for the DMAC component. The API includes functions
*   for the DMA controller, DMA channels and Transfer Descriptors.
*
*
*   Note:
*     This module requires the developer to finish or fill in the auto
*     generated funcions and setup the dma channel and TD's.
*
********************************************************************************
* Copyright 2008-2010, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
********************************************************************************/
#include <CYLIB.H>
#include <CYDMAC.H>
#include <USBUART_ep2_dma.H>



/****************************************************************************
* 
* The following defines are available in Cyfitter.h
* 
* 
* 
* USBUART_ep2__DRQ_CTL_REG
* 
* 
* USBUART_ep2__DRQ_NUMBER
* 
* Number of TD's used by this channel.
* USBUART_ep2__NUMBEROF_TDS
* 
* Priority of this channel.
* USBUART_ep2__PRIORITY
* 
* True if USBUART_ep2_TERMIN_SEL is used.
* USBUART_ep2__TERMIN_EN
* 
* TERMIN interrupt line to signal terminate.
* USBUART_ep2__TERMIN_SEL
* 
* 
* True if USBUART_ep2_TERMOUT0_SEL is used.
* USBUART_ep2__TERMOUT0_EN
* 
* 
* TERMOUT0 interrupt line to signal completion.
* USBUART_ep2__TERMOUT0_SEL
* 
* 
* True if USBUART_ep2_TERMOUT1_SEL is used.
* USBUART_ep2__TERMOUT1_EN
* 
* 
* TERMOUT1 interrupt line to signal completion.
* USBUART_ep2__TERMOUT1_SEL
* 
****************************************************************************/


/* Zero based index of USBUART_ep2 dma channel */
uint8 USBUART_ep2_DmaHandle = DMA_INVALID_CHANNEL;

/*********************************************************************
* Function Name: uint8 USBUART_ep2_DmaInitalize
**********************************************************************
* Summary:
*   Allocates and initialises a channel of the DMAC to be used by the
*   caller.
*
* Parameters:
*   BurstCount.
*       
*       
*   ReqestPerBurst.
*       
*       
*   UpperSrcAddress.
*       
*       
*   UpperDestAddress.
*       
*
* Return:
*   The channel that can be used by the caller for DMA activity.
*   DMA_INVALID_CHANNEL (0xFF) if there are no channels left. 
*
*
*******************************************************************/
uint8 USBUART_ep2_DmaInitialize(uint8 BurstCount, uint8 ReqestPerBurst, uint16 UpperSrcAddress, uint16 UpperDestAddress) 
{

    /* Allocate a DMA channel. */
    USBUART_ep2_DmaHandle = (uint8)USBUART_ep2__DRQ_NUMBER;

    /* Configure the channel. */
    (void)CyDmaChSetConfiguration(USBUART_ep2_DmaHandle,
                                  BurstCount,
                                  ReqestPerBurst,
                                  (uint8)USBUART_ep2__TERMOUT0_SEL,
                                  (uint8)USBUART_ep2__TERMOUT1_SEL,
                                  (uint8)USBUART_ep2__TERMIN_SEL);

    /* Set the extended address for the transfers */
    (void)CyDmaChSetExtendedAddress(USBUART_ep2_DmaHandle, UpperSrcAddress, UpperDestAddress);

    /* Set the priority for this channel */
    (void)CyDmaChPriority(USBUART_ep2_DmaHandle, (uint8)USBUART_ep2__PRIORITY);
    
    return USBUART_ep2_DmaHandle;
}

/*********************************************************************
* Function Name: void USBUART_ep2_DmaRelease
**********************************************************************
* Summary:
*   Frees the channel associated with USBUART_ep2.
*
*
* Parameters:
*   void.
*
*
*
* Return:
*   void.
*
*******************************************************************/
void USBUART_ep2_DmaRelease(void) 
{
    /* Disable the channel */
    (void)CyDmaChDisable(USBUART_ep2_DmaHandle);
}


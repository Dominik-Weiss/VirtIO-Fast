#include <VBox/vmm/pdmdev.h>
#include <VBox/version.h>
#include <iprt/assert.h>
#include <VBox/log.h>
#include <VBox/vmm/pdmpcidev.h>
#include "../VirtIO/Virtio.h"
#include <VBox/vmm/pgm.h>
#include <iprt/semaphore.h>

#ifdef IN_RING3
# include <iprt/mem.h>
# include <iprt/uuid.h>
#endif /* IN_RING3 */

#define VFAST_PCI_CLASS               0x0400 			//Multimedia Device
#define VFAST_PCI_DEVICE_ID			  25
#define VFAST_N_QUEUES                2					
#define VFAST_NAME_FMT                "DevVirtioFast%d"

#define VFAST_F_VQ_SINGLEACK		0x00000001 //Single Queue with ack available
#define VFAST_F_VQ_SINGLENOACK		0x00000002 //Single Queue with ack not available
#define VFAST_F_VQ_MULTI			0x00000020 //Multiple Queues available

#ifndef VFAST
#define VFAST

#define DEBUG_ENABLED
#ifdef DEBUG_ENABLED
	#define DEBUG
#else
	#define DEBUG for(;0;)
#endif


struct VFASTPCIConfig
{
    uint16_t uStatus;
};

/**
 * Device instance data
 */
typedef struct VFASTSTATE
{
	VPCISTATE 				VPCI;					//The state of the VirtIO PCI device
	
	PDMPCIDEV 				pciDev;					//PCI Device Structure
	
	//VIRTIO Queues
	R3PTRTYPE(PVQUEUE)      pFirstQueue;
	R3PTRTYPE(PVQUEUE)      pSecondQueue;
	
	RTSEMEVENT 				hEventMoreRxDescAvail; 	//Gets signalled when more RX descriptors become available
	
	PDMIDISPLAYPORT 		IPortDown;				//PDM Display Connector Interface Down
	
    bool volatile           fMaybeOutOfSpace;
	
	R0PTRTYPE(PPDMQUEUE)    pCanRxQueueR0;			//Rx wakeup signaller - R0
    RCPTRTYPE(PPDMQUEUE)    pCanRxQueueRC;			//Rx wakeup signaller - RC
	R3PTRTYPE(PPDMQUEUE)    pCanRxQueueR3;			//Rx wakeup signaller - R3
	
	struct VFASTPCIConfig 	config;					//PCI Config Data
	
	char* 					pSendBuffer;			//Buffer for received messages
	uint32_t 				lastBufSize;			//Size of the last received messages
	
	uint32_t 				features;				//supported features
}VFASTSTATE;
typedef VFASTSTATE *PVFASTSTATE;

/**
 *	Dumps the current state of the virtio queues
 *	@param	caller 		char* with the function name of the caller of this function
 *	@param	pThis		pointer to the PVFASTSTATE 
**/
void vfastDumpQueues(char* caller, PVFASTSTATE pThis)
{
	LogRel(("DevVirtioFast vfastDumpQueues: (called from %s)\n"
          "  uGuestFeatures = 0x%08x\n"
          "  uQueueSelector = 0x%04x\n"
          "  uStatus        = 0x%02x\n"
          "  uISR           = 0x%02x\n",
		  caller,
          pThis->VPCI.uGuestFeatures,
          pThis->VPCI.uQueueSelector,
          pThis->VPCI.uStatus,
          pThis->VPCI.uISR));

    for (unsigned i = 0; i < pThis->VPCI.nQueues; i++)
        LogRel((" %s queue:\n"
              "  VRing.uSize           = %u\n"
              "  VRing.addrDescriptors = %p\n"
              "  VRing.addrAvail       = %p\n"
              "  VRing.addrUsed        = %p\n"
              "  uNextAvailIndex       = %u\n"
              "  uNextUsedIndex        = %u\n"
              "  uPageNumber           = %x\n",
              pThis->VPCI.Queues[i].pcszName,
              pThis->VPCI.Queues[i].VRing.uSize,
              pThis->VPCI.Queues[i].VRing.addrDescriptors,
              pThis->VPCI.Queues[i].VRing.addrAvail,
              pThis->VPCI.Queues[i].VRing.addrUsed,
              pThis->VPCI.Queues[i].uNextAvailIndex,
              pThis->VPCI.Queues[i].uNextUsedIndex,
              pThis->VPCI.Queues[i].uPageNumber));

}

/*
==========================================
	Queue control functions
==========================================
*/

/**
 * Wakes up the RX thread.
 */
static void vfastWakeupReceive(PPDMDEVINS pDevIns)
{
	DEBUG LogRel(("DevVirtioFast: vfastWakeupReceive\n"));
    PVFASTSTATE pThis = PDMINS_2_DATA(pDevIns, PVFASTSTATE);
    if (pThis->fMaybeOutOfSpace
        &&  pThis->hEventMoreRxDescAvail != NIL_RTSEMEVENT)
    {
        DEBUG LogRel(("DevVirtioFast vfastWakeupReceive: Waking up Out-of-RX-space semaphore\n"));
        RTSemEventSignal(pThis->hEventMoreRxDescAvail);
    }
}

/**
 * @interface_method_impl{PDMIBASE,pfnQueryInterface}

 * Queries an interface to the driver.
 *
 * @returns Pointer to interface.
 * @returns NULL if the interface was not supported by the driver.
 * @param   pInterface          Pointer to this interface structure.
 * @param   pszIID              The interface ID, a UUID string.
 */
static DECLCALLBACK(void *) vfastQueryInterface(struct PDMIBASE *pInterface, const char *pszIID)
{
	DEBUG LogRel(("DevVirtioFast: vfastQueryInterface\n"));
    PVFASTSTATE pThis = RT_FROM_MEMBER(pInterface, VFASTSTATE, VPCI.IBase);
    Assert(&pThis->VPCI.IBase == pInterface);

    PDMIBASE_RETURN_INTERFACE(pszIID, PDMIDISPLAYPORT, &pThis->IPortDown);
    return vpciQueryInterface(pInterface, pszIID);
}


/**
 * Callback function for the virtio_fast_single_ack driver 
 * 
 * @param pvState	The device state structure
 * @param pQueue	The virtqueue from which the from got called
 */
static DECLCALLBACK(void) vfastSingleQueueACKCB(void *pvState, PVQUEUE pQueue)
{
	VQUEUEELEM elem;
    PVFASTSTATE pThis = (PVFASTSTATE)pvState;
	
	DEBUG LogRel(("DevVirtioFast vfastSingleQueueACKCB: News in the first single queue in ack mode.\n"));
    DEBUG vfastDumpQueues("vfastSingleQueueACKCB",pThis);
	
	//Receive all the elements present in the virtqueue
	while (vqueueGet(&pThis->VPCI, pQueue, &elem,true))
    {
		DEBUG LogRel(("DevVirtioFast vfastSingleQueueACKCB: Index %d, In %d, Out %d\n",elem.uIndex,elem.nIn,elem.nOut));
		
		//Process incoming messages
		if(elem.nOut == 1)
		{	
			pThis->lastBufSize = elem.aSegsOut[0].cb;

			//Prepare the message buffer
			pThis->pSendBuffer = (char*) malloc(pThis->lastBufSize);
			if(pThis->pSendBuffer == NULL)
			{
				LogRel(("DevVirtioFast vfastSingleQueueACKCB: Error allocating %d bytes for pSendBuffer",pThis->lastBufSize));
				continue;
			}
			
			PDMDevHlpPhysRead(pThis->VPCI.CTX_SUFF(pDevIns),
                              elem.aSegsOut[0].addr,
                              pThis->pSendBuffer, pThis->lastBufSize);

			DEBUG LogRel(("DevVirtioFast vfastSingleQueueACKCB: Data: addr %p , pv %d, cb %d\n", elem.aSegsOut[0].addr, elem.aSegsOut[0].pv,elem.aSegsOut[0].cb));
			DEBUG LogRel(("DevVirtioFast vfastSingleQueueACKCB: Read Data: msg %s\n", pThis->pSendBuffer));
			
			//Add the acknowledge message to the queue
			vqueuePut(&pThis->VPCI, pQueue, &elem, 0);
		}
		
		//Send outgoing messages
		if(elem.nIn == 1)
		{
			DEBUG LogRel(("DevVirtioFast vfastSingleQueueACKCB: send data\n"));

			PDMDevHlpPCIPhysWrite(pThis->VPCI.CTX_SUFF(pDevIns),
									elem.aSegsIn[0].addr,
									pThis->pSendBuffer, pThis->lastBufSize);
			elem.aSegsIn[0].cb = pThis->lastBufSize;

			//Message buffer is no longer needed -> free data
			free(pThis->pSendBuffer);
			
			DEBUG LogRel(("DevVirtioFast vfastSingleQueueACKCB: Data: addr %p , pv %d, cb %d\n", elem.aSegsIn[0].addr, elem.aSegsIn[0].pv,elem.aSegsIn[0].cb));

			vqueuePut(&pThis->VPCI, pQueue, &elem, pThis->lastBufSize);
		}
		vqueueSync(&pThis->VPCI, pQueue);
    }
}

/**
 * Callback function for the virtio_fast_single driver 
 * 
 * @param pvState	The device state structure
 * @param pQueue	The virtqueue from which the from got called
 */
static DECLCALLBACK(void) vfastSingleQueueCB(void *pvState, PVQUEUE pQueue)
{
	VQUEUEELEM elem;
    PVFASTSTATE pThis = (PVFASTSTATE)pvState;
	
	DEBUG LogRel(("DevVirtioFast vfastSingleQueueCB: News in the first single queue in no ack mode.\n"));
    DEBUG vfastDumpQueues("vfastSingleQueue",pThis);

	//Receive all the elements present in the virtqueue
	while (vqueueGet(&pThis->VPCI, pQueue, &elem,true))
    {
		DEBUG LogRel(("DevVirtioFast vfastSingleQueueCB: Index %d, In %d, Out %d\n",elem.uIndex,elem.nIn,elem.nOut));
		
		//Since there is no speci
		if(elem.nOut  != 1 && elem.nIn != 1)
		{
			LogRel(("DevVirtioFast vfastSingleQueueCB: Error incorrect amount of buffers: %d out-buffers %d in-buffers!\n",elem.nOut,elem.nIn));
			continue;
		}
		
		//Process incoming message
		pThis->lastBufSize = elem.aSegsOut[0].cb;
		pThis->pSendBuffer = (char*) malloc(pThis->lastBufSize);
		if(pThis->pSendBuffer == NULL)
		{
			LogRel(("DevVirtioFast vfastSingleQueueCB: Error allocating %d bytes for pSendBuffer",pThis->lastBufSize));
			continue;
		}
			
		PDMDevHlpPhysRead(pThis->VPCI.CTX_SUFF(pDevIns),
                          elem.aSegsOut[0].addr,
                          pThis->pSendBuffer, pThis->lastBufSize);

		DEBUG LogRel(("DevVirtioFast vfastSingleQueueCB: Data: addr %p , pv %d, cb %d\n", elem.aSegsOut[0].addr, elem.aSegsOut[0].pv,elem.aSegsOut[0].cb));
		DEBUG LogRel(("DevVirtioFast vfastSingleQueueCB: Read Data: msg %s\n", pThis->pSendBuffer));


		//Send message
		PDMDevHlpPCIPhysWrite(pThis->VPCI.CTX_SUFF(pDevIns),
							  elem.aSegsIn[0].addr,
							  pThis->pSendBuffer, pThis->lastBufSize);
		elem.aSegsIn[0].cb = pThis->lastBufSize;
		
		//Message buffer is no longer needed -> free data
		free(pThis->pSendBuffer);
		
		vqueuePut(&pThis->VPCI, pQueue, &elem, pThis->lastBufSize);
		vqueueSync(&pThis->VPCI, pQueue);
    }
}

/**
 * Callback function for the virtio_fast_multi driver to receive data
 * 
 * @param pvState	The device state structure
 * @param pQueue	The virtqueue from which the from got called
 */
static DECLCALLBACK(void) vfastMultiQueueACKCBOut(void *pvState, PVQUEUE pQueue)
{
	VQUEUEELEM elem;
    PVFASTSTATE pThis = (PVFASTSTATE)pvState;
	
	DEBUG LogRel(("DevVirtioFast vfastMultiQueueACKCBOut: News in the in queue in ack mode.\n"));
    DEBUG vfastDumpQueues("vfastMultiQueueACKCBOut",pThis);
	
	//Receive all the elements present in the virtqueue
	while (vqueueGet(&pThis->VPCI, pQueue, &elem,true))
    {
		DEBUG LogRel(("DevVirtioFast vfastMultiQueueACKCBOut: Index %d, In %d, Out %d\n",elem.uIndex,elem.nIn,elem.nOut));
		
		//Process incoming messages
		if(elem.nOut == 1)
		{	
			pThis->lastBufSize = elem.aSegsOut[0].cb;
			pThis->pSendBuffer = (char*) malloc(pThis->lastBufSize);
			if(pThis->pSendBuffer == NULL)
			{
				LogRel(("DevVirtioFast vfastMultiQueueACKCBOut: Error allocating %d bytes for pSendBuffer",pThis->lastBufSize));
				continue;
			}
			
			PDMDevHlpPhysRead(pThis->VPCI.CTX_SUFF(pDevIns),
                              elem.aSegsOut[0].addr,
                              pThis->pSendBuffer, pThis->lastBufSize);

			DEBUG LogRel(("DevVirtioFast vfastMultiQueueACKCBOut: Data: addr %p , pv %d, cb %d\n", elem.aSegsOut[0].addr, elem.aSegsOut[0].pv,elem.aSegsOut[0].cb));
			DEBUG LogRel(("DevVirtioFast vfastMultiQueueACKCBOut: Read Data: msg %s\n", pThis->pSendBuffer));
			
			//Add the acknowledge message to the queue
			vqueuePut(&pThis->VPCI, pQueue, &elem, 0);
		}
		vqueueSync(&pThis->VPCI, pQueue);
    }
}

/**
 * Callback function for the virtio_fast_multi driver to send data
 * 
 * @param pvState	The device state structure
 * @param pQueue	The virtqueue from which the from got called
 */
static DECLCALLBACK(void) vfastMultiQueueACKCBIn(void *pvState, PVQUEUE pQueue)
{
	VQUEUEELEM elem;
    PVFASTSTATE pThis = (PVFASTSTATE)pvState;
	
	DEBUG LogRel(("DevVirtioFast vfastMultiQueueACKCBIn: News in the out queue in ack mode.\n"));
    DEBUG vfastDumpQueues("vfastMultiQueueACKCBIn",pThis);
	
	DEBUG LogRel(("DevVirtioFast vfastMultiQueueACKCBIn: vqueueisEmpty: %d\n",vqueueIsEmpty(&pThis->VPCI,pQueue)));
	
	//Receive all the elements present in the virtqueue
	while (vqueueGet(&pThis->VPCI, pQueue, &elem,true))
    {
		DEBUG LogRel(("DevVirtioFast vfastMultiQueueACKCBIn: Index %d, In %d, Out %d\n",elem.uIndex,elem.nIn,elem.nOut));
		
		//Send outgoing messages
		if(elem.nIn == 1)
		{
			LogRel(("DevVirtioFast vfastMultiQueueACKCBIn: send data\n"));
			PDMDevHlpPCIPhysWrite(pThis->VPCI.CTX_SUFF(pDevIns),
									elem.aSegsIn[0].addr,
									pThis->pSendBuffer, pThis->lastBufSize);
			elem.aSegsIn[0].cb = pThis->lastBufSize;
			
			//Message buffer is no longer needed -> free data
			free(pThis->pSendBuffer);
			
			vqueuePut(&pThis->VPCI, pQueue, &elem, pThis->lastBufSize);
		}
		vqueueSync(&pThis->VPCI, pQueue);
    }
}

/**
 * Callback function of the first virtqueue to decide depending on the set features, which specific callback function to invoke
 * 
 * @param pvState	The device state structure
 * @param pQueue	The virtqueue from which the from got called
 */ 
static DECLCALLBACK(void) vfastGeneralCB(void *pvState, PVQUEUE pQueue)
{
	DEBUG LogRel(("DevVirtioFast: vfastGeneralCB\n"));
	PVFASTSTATE pThis = (PVFASTSTATE)pvState;

	if(pThis->features == VFAST_F_VQ_SINGLEACK)
	{
		vfastSingleQueueACKCB(pvState, pQueue);
	}
	else if(pThis->features == VFAST_F_VQ_SINGLENOACK)
	{
		vfastSingleQueueCB(pvState, pQueue);
	}
	else if(pThis->features == VFAST_F_VQ_MULTI)
	{
		vfastMultiQueueACKCBIn(pvState, pQueue);
	}
}

/*
==========================================
	IoCb callback functions
==========================================
*/

/**
 * VirtIO function to return the Hosts supported features
 */ 
static DECLCALLBACK(uint32_t) vfastIoCb_GetHostFeatures(void *pvState)
{
	DEBUG LogRel(("DevVirtioFast: vfastIoCb_GetHostFeatures\n"));
    RT_NOREF_PV(pvState);

    return VFAST_F_VQ_SINGLEACK | VFAST_F_VQ_SINGLENOACK | VFAST_F_VQ_MULTI;
}

/**
 * VirtIO function to return the hosts minimal features 
 * In this case it returns none, however this is not totally correct, at least one feature must be supported
 */ 
static DECLCALLBACK(uint32_t) vfastIoCb_GetHostMinimalFeatures(void *pvState)
{
	DEBUG LogRel(("DevVirtioFast: vfastIoCb_GetHostMinimalFeatures\n"));
    RT_NOREF_PV(pvState);
	
    return 0;
}

/**
 * VirtIO function to set the features of the host depending on the ones the driver supports
 */ 
static DECLCALLBACK(void) vfastIoCb_SetHostFeatures(void *pvState, uint32_t fFeatures)
{
    PVFASTSTATE pThis = (PVFASTSTATE)pvState;
	DEBUG LogRel(("DevVirtioFast: vfastIoCb_SetHostFeatures\n"));
	
	pThis->features = fFeatures;

	//Print supported features
	#ifdef DEBUG_ENABLED
		static struct
    	{
    	    uint32_t uMask;
    	    const char *pcszDesc;
    	} const s_aFeatures[] =
    	{
    	    { VFAST_F_VQ_SINGLEACK,		"single queue ack accepted" },
			{ VFAST_F_VQ_SINGLENOACK,	"single queue no ack accepted"	},
    	    { VFAST_F_VQ_MULTI,        	"multi queue accepted" },
    	};

    	for (unsigned i = 0; i < RT_ELEMENTS(s_aFeatures); ++i)
    	{
    	    if (s_aFeatures[i].uMask & fFeatures)
    	        DEBUG LogRel(("DevVirtioFast vfastIoCb_SetHostFeatures: %s\n", s_aFeatures[i].pcszDesc));
    	}
	#endif
}

/**
 * VirtIO function to retrieve the devices config
 */ 
static DECLCALLBACK(int) vfastIoCb_GetConfig(void *pvState, uint32_t offCfg, uint32_t cb, void *data)
{
    PVFASTSTATE pThis = (PVFASTSTATE)pvState;
	DEBUG LogRel(("DevVirtioFast: vfastIoCb_GetConfig\n"));
	DEBUG vfastDumpQueues("vfastIoCb_GetConfig",pThis);

    if (offCfg + cb > sizeof(struct VFASTPCIConfig))
    {
        LogRel(("DevVirtioFast vfastIoCb_GetConfig: Read beyond the config structure is attempted (offCfg=%#x cb=%x).\n", offCfg, cb));
        return VERR_IOM_IOPORT_UNUSED;
    }
    memcpy(data, (uint8_t *)&pThis->config + offCfg, cb);
    return VINF_SUCCESS;
}

/**
 * VirtIO function to set the devices config
 */ 
static DECLCALLBACK(int) vfastIoCb_SetConfig(void *pvState, uint32_t offCfg, uint32_t cb, void *data)
{
    PVFASTSTATE pThis = (PVFASTSTATE)pvState;
	
	DEBUG LogRel(("DevVirtioFast: vfastIoCb_SetConfig\n"));
	DEBUG vfastDumpQueues("vfastIoCb_SetConfig",pThis);
	
    if (offCfg + cb > sizeof(struct VFASTPCIConfig))
    {
        LogRel(("DevVirtioFast vfastIoCb_SetConfig: Write beyond the config structure is attempted (offCfg=%#x cb=%x).\n", offCfg, cb));
        if (offCfg < sizeof(struct VFASTPCIConfig))
            memcpy((uint8_t *)&pThis->config + offCfg, data,
                   sizeof(struct VFASTPCIConfig) - offCfg);
        return VINF_SUCCESS;
    }
    memcpy((uint8_t *)&pThis->config + offCfg, data, cb);
    return VINF_SUCCESS;
}

/**
 * VirtIO function for a hardware reset. Revert all registers to there initial values.
 */
static DECLCALLBACK(int) vfastIoCb_Reset(void *pvState)
{
    PVFASTSTATE pThis = (PVFASTSTATE)pvState;
	
	DEBUG LogRel(("DevVirtioFast: vfastIoCb_Reset\n"));
	DEBUG vfastDumpQueues("vfastIoCb_Reset",pThis);

    vpciReset(&pThis->VPCI);
	
	#ifndef IN_RING3
	    return VINF_IOM_R3_IOPORT_WRITE;
	#else
	    return VINF_SUCCESS;
	#endif
}

/**
 * VirtIO function which is called when the driver becomes ready.
 */
static DECLCALLBACK(void) vfastIoCb_Ready(void *pvState)
{
    PVFASTSTATE pThis = (PVFASTSTATE)pvState;
	DEBUG LogRel(("DevVirtioFast: vfastIoCb_Ready\n"));
	DEBUG vfastDumpQueues("vfastIoCb_Ready", pThis);

	#ifdef IN_RING3
	    vfastWakeupReceive(pThis->VPCI.CTX_SUFF(pDevIns));
	#else
	    PPDMQUEUEITEMCORE pItem = PDMQueueAlloc(pThis->CTX_SUFF(pCanRxQueue));
	    if (pItem)
	        PDMQueueInsert(pThis->CTX_SUFF(pCanRxQueue), pItem);
	#endif	
}

/**
 * VirtIO port callbacks.
 */
static const VPCIIOCALLBACKS g_IOCallbacks =
{
     vfastIoCb_GetHostFeatures,
     vfastIoCb_GetHostMinimalFeatures,
     vfastIoCb_SetHostFeatures,
     vfastIoCb_GetConfig,
     vfastIoCb_SetConfig,
     vfastIoCb_Reset,
     vfastIoCb_Ready,
};

/*
==========================================
	IOPort callback functions
==========================================
*/

/**
 * @callback_method_impl{FNIOMIOPORTIN}
 *
 * Port I/O Handler for IN operations.
 *
 * @returns VINF_SUCCESS or VINF_EM_*.
 * @returns VERR_IOM_IOPORT_UNUSED if the port is really unused and a ~0 value should be returned.
 *
 * @param   pDevIns     The device instance.
 * @param   pvUser      User argument.
 * @param   uPort       Port number used for the IN operation.
 * @param   pu32        Where to store the result.  This is always a 32-bit
 *                      variable regardless of what @a cb might say.
 * @param   cb          Number of bytes read.
 * @remarks Caller enters the device critical section.
 */
PDMBOTHCBDECL(int) vfastIOPortIn(PPDMDEVINS pDevIns, void *pvUser, RTIOPORT port, uint32_t *pu32, unsigned cb)
{
	DEBUG LogRel(("DevVirtioFast: vfastIOPortIn\n"));
    return vpciIOPortIn(pDevIns, pvUser, port, pu32, cb, &g_IOCallbacks);
}

/**
 * @callback_method_impl{FNIOMIOPORTOUT}
 * 
 * Port I/O Handler for OUT operations.
 *
 * @returns VINF_SUCCESS or VINF_EM_*.
 *
 * @param   pDevIns     The device instance.
 * @param   pvUser      User argument.
 * @param   uPort       Port number used for the OUT operation.
 * @param   u32         The value to output.
 * @param   cb          The value size in bytes.
 * @remarks Caller enters the device critical section.
 */
PDMBOTHCBDECL(int) vfastIOPortOut(PPDMDEVINS pDevIns, void *pvUser, RTIOPORT port, uint32_t u32, unsigned cb)
{
	DEBUG LogRel(("DevVirtioFast: vfastIOPortOut\n"));
    return vpciIOPortOut(pDevIns, pvUser, port, u32, cb, &g_IOCallbacks);
}

/**
 * @callback_method_impl{FNPCIIOREGIONMAP}
 * 
 * Callback function for mapping an PCI I/O region.
 *
 * @returns VBox status code.
 * @param   pDevIns         Pointer to the device instance the PCI device
 *                          belongs to.
 * @param   pPciDev         Pointer to the PCI device.
 * @param   iRegion         The region number.
 * @param   GCPhysAddress   Physical address of the region. If enmType is PCI_ADDRESS_SPACE_IO, this
 *                          is an I/O port, otherwise it's a physical address.
 *
 *                          NIL_RTGCPHYS indicates that a MMIO2 mapping is about to be unmapped and
 *                          that the device deregister access handlers for it and update its internal
 *                          state to reflect this.
 *
 * @param   cb              Size of the region in bytes.
 * @param   enmType         One of the PCI_ADDRESS_SPACE_* values.
 *
 * @remarks Called with the PDM lock held.  The device lock is NOT take because
 *          that is very likely be a lock order violation.
 */
static DECLCALLBACK(int) vfastMap(PPDMDEVINS pDevIns, PPDMPCIDEV pPciDev, uint32_t iRegion,
                                 RTGCPHYS GCPhysAddress, RTGCPHYS cb, PCIADDRESSSPACE enmType)
{
    RT_NOREF(pPciDev, iRegion);
	DEBUG LogRel(("DevVirtioFast: vfastMap\n"));
    PVFASTSTATE pThis = PDMINS_2_DATA(pDevIns, PVFASTSTATE);
    int rc;

    if (enmType != PCI_ADDRESS_SPACE_IO)
    {
        /* We should never get here */
        AssertMsgFailed(("DevVirtioFast vfastMap: Invalid PCI address space param in map callback"));
        return VERR_INTERNAL_ERROR;
    }

    pThis->VPCI.IOPortBase = (RTIOPORT)GCPhysAddress;
    rc = PDMDevHlpIOPortRegister(pDevIns, pThis->VPCI.IOPortBase,
                                 cb, 0, vfastIOPortOut, vfastIOPortIn,
                                 NULL, NULL, "VirtioFast");
    AssertRC(rc);
    return rc;
}

/**
 * @callback_method_impl{FNPDMQUEUEDEV, Handler for the wakeup signaller queue.}
 * 
 * Queue consumer callback for devices.
 *
 * @returns Success indicator.
 *          If false the item will not be removed and the flushing will stop.
 * @param   pDevIns     The device instance.
 * @param   pItem       The item to consume. Upon return this item will be freed.
 * @remarks The device critical section will NOT be entered before calling the
 *          callback.  No locks will be held, but for now it's safe to assume
 *          that only one EMT will do queue callbacks at any one time.
 */
static DECLCALLBACK(bool) vnetCanRxQueueConsumer(PPDMDEVINS pDevIns, PPDMQUEUEITEMCORE pItem)
{
	DEBUG LogRel(("DevVirtioFast: vnetCanRxQueueConsumer\n"));
    RT_NOREF(pItem);
    vfastWakeupReceive(pDevIns);
    return true;
}

/*
==========================================
	Device registration functions
==========================================
*/

static DECLCALLBACK(int) vfastConstruct(PPDMDEVINS pDevIns, int iInstance, PCFGMNODE pCfg)
{
	DEBUG LogRel(("DevVirtioFast: vfastConstruct\n"));

	//Check that the device instance structure is compatible
	PDMDEV_CHECK_VERSIONS_RETURN(pDevIns);
	
	//Init the instance data
	PVFASTSTATE pThis = PDMINS_2_DATA(pDevIns,PVFASTSTATE);
	pThis->hEventMoreRxDescAvail = NIL_RTSEMEVENT;
	//pThis->Data = 0;
	
	/* Do our own locking. */
    int rc = PDMDevHlpSetDeviceCritSect(pDevIns, PDMDevHlpCritSectGetNop(pDevIns));
    AssertRCReturn(rc, rc); 
	if(RT_FAILURE(rc))
	{
		LogRel(("DevVirtioFast: Error with custom locking\n"));
	}
	
	//Init the PCI part
	pThis->VPCI.IBase.pfnQueryInterface = vfastQueryInterface; 

	int ret = vpciConstruct (pDevIns,
	&pThis->VPCI, iInstance,
	VFAST_NAME_FMT, VFAST_PCI_DEVICE_ID,
	VFAST_PCI_CLASS,VFAST_N_QUEUES);

	if(RT_FAILURE(ret))
	{
		LogRel(("DevVirtioFast: Error with vpciConstruct, maybe there is no driver to attach\n"));
		//Set the number of maximum needed queues manually
		pThis->VPCI.nQueues = VFAST_N_QUEUES;
	}
	
	/* Map our ports to IO space. */
    rc = PDMDevHlpPCIIORegionRegister(pDevIns, 0,
                                      VPCI_CONFIG,
                                      PCI_ADDRESS_SPACE_IO, vfastMap);
	
    if (RT_FAILURE(rc))
        return rc;
	DEBUG LogRel(("DevVirtioFast: Ports mapped\n"));
	
	/* Create the RX notifier signaler. */
    rc = PDMDevHlpQueueCreate(pDevIns, sizeof(PDMQUEUEITEMCORE), 1, 0,
                              vnetCanRxQueueConsumer, true, "VNet-Rcv", &pThis->pCanRxQueueR3);
    if (RT_FAILURE(rc))
        return rc;
    pThis->pCanRxQueueR0 = PDMQueueR0Ptr(pThis->pCanRxQueueR3);
    pThis->pCanRxQueueRC = PDMQueueRCPtr(pThis->pCanRxQueueR3);

	DEBUG LogRel(("DevVirtioFast: Queue saved\n"));
	
	//Reset pci device
	vpciReset(&pThis->VPCI);
	
	DEBUG LogRel(("DevVirtioFast: Device reset\n"));
	
	pThis->VPCI.uStatus = 3;
	
	pThis->pFirstQueue  = vpciAddQueue(&pThis->VPCI, 256, vfastGeneralCB,  "First");
	pThis->pFirstQueue  = vpciAddQueue(&pThis->VPCI, 256, vfastMultiQueueACKCBOut,  "Second");

	
	DEBUG LogRel(("DevVirtioFast: Start sucessfull\n"));
	DEBUG vfastDumpQueues("vfastConstruct",pThis);
	
	//Use the instance number
	NOREF(iInstance);
	RT_NOREF(pCfg);
	
	return VINF_SUCCESS;
}

static DECLCALLBACK(int) vfastAttach(PPDMDEVINS pDevIns, unsigned iLUN, uint32_t fFlags)
{
	DEBUG LogRel(("DevVirtioFast: Attach\n"));
	RT_NOREF(pDevIns);
	RT_NOREF(iLUN);
	RT_NOREF(fFlags);
	return VINF_SUCCESS;
}

static DECLCALLBACK(void) vfastDetach(PPDMDEVINS pDevIns, unsigned iLUN, uint32_t fFlags)
{
	DEBUG LogRel(("DevVirtioFast: Detach\n"));
	RT_NOREF(pDevIns);
	RT_NOREF(iLUN);
	RT_NOREF(fFlags);
}

/**
 * The device registration structure.
 */
const PDMDEVREG g_DeviceVirtioFast =
{
    /* Structure version. PDM_DEVREG_VERSION defines the current version. */
    PDM_DEVREG_VERSION,
    /* Device name. */
    "DevVirtioFast",
    /* Name of guest context module (no path).
     * Only evalutated if PDM_DEVREG_FLAGS_RC is set. */
    "VBoxDDRC.rc",
    /* Name of ring-0 module (no path).
     * Only evalutated if PDM_DEVREG_FLAGS_RC is set. */
    "VBoxDDR0.r0",
    /* The description of the device. The UTF-8 string pointed to shall, like this structure,
     * remain unchanged from registration till VM destruction. */
    "Virtio Fast\n",

    /* Flags, combination of the PDM_DEVREG_FLAGS_* */
    PDM_DEVREG_FLAGS_DEFAULT_BITS,
	
    /* Device class, graphic device like vga */
    PDM_DEVREG_CLASS_GRAPHICS,
    /* Maximum number of instances (per VM). */
    1,
    /* Size of the instance data. */
    sizeof(VFASTSTATE),

    /* pfnConstruct */
    vfastConstruct,
    /* pfnDestruct */
	NULL,
    /* pfnRelocate */
	NULL,
    /* pfnMemSetup. */
    NULL,
    /* pfnPowerOn */
    NULL,
    /* pfnReset */
    NULL,
    /* pfnSuspend */
	NULL,
    /* pfnResume */
    NULL,
    /* pfnAttach */
    vfastAttach,
    /* pfnDetach */
    vfastDetach,
    /* pfnQueryInterface */
    NULL,
    /* pfnInitComplete */
    NULL,
    /* pfnPowerOff */
	NULL,
    /* pfnSoftReset */
    NULL,

    /* u32VersionEnd */
    PDM_DEVREG_VERSION
};

/**
 * Register devices provided by the plugin
 *
 * @returns VBox status code
 * @param pCallbacks Point to the callback table
 * @param u32Version VBox version number
 */
 
 extern "C" DECLEXPORT(int) VBoxDevicesRegister(PPDMDEVREGCB pCallbacks, uint32_t u32Version)
 {
	LogFlow(("DevVirtioFast::VBoxDevicesRegister: u32Version=%#x pCallbacks->u32Version=%#x\n",u32Version,pCallbacks->u32Version));
	DEBUG LogRel(("DevVirtioFast: Register\n"));
	AssertLogRelMsgReturn(u32Version >= VBOX_VERSION,("VirtualBox version %#x, expected %#x or higher\n",u32Version,VBOX_VERSION),VERR_VERSION_MISMATCH);
	AssertLogRelMsgReturn(pCallbacks->u32Version == PDM_DEVREG_CB_VERSION,("callback version %#x, expected %#x or higher\n",pCallbacks->u32Version,PDM_DEVREG_CB_VERSION),VERR_VERSION_MISMATCH);
	return pCallbacks->pfnRegister(pCallbacks,&g_DeviceVirtioFast);
 }
 
 #endif
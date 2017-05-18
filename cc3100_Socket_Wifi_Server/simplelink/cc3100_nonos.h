/*
 * nonos.h - CC31xx/CC32xx Host Driver Implementation
 *
 * Copyright (C) 2014 Texas Instruments Incorporated - http://www.ti.com/
 *
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
*/

#ifndef NONOS_H_
#define	NONOS_H_

#include "cc3100_simplelink.h"

namespace mbed_cc3100 {
	
/*****************************************************************************/
/* Macro declarations                                                        */
/*****************************************************************************/


#ifndef SL_PLATFORM_MULTI_THREADED

/* This function call the user defined function, if defined, from the sync wait loop  */
/* The use case of this function is to allow nonos system to call a user function to put the device into sleep */
/* The wake up should be activated after getting an interrupt from the device to Host */
/* The user function must return without blocking to prevent a delay on the event handling */
/*
#define _SlSyncWaitLoopCallback  UserSleepFunction
*/

const uint8_t NONOS_WAIT_FOREVER   			 =	0xFF;
const uint8_t NONOS_NO_WAIT        			 =	0x00;

const uint8_t NONOS_RET_OK                   =  (0);
const int8_t  NONOS_RET_ERR                  =  (0xFF);
const uint8_t OSI_OK                         =  NONOS_RET_OK;

const uint8_t NON_OS_SYNC_OBJ_CLEAR_VALUE	 =	0x11;
const uint8_t NON_OS_SYNC_OBJ_SIGNAL_VALUE	 =	0x22;
const uint8_t NON_OS_LOCK_OBJ_UNLOCK_VALUE	 =	0x33;
const uint8_t NON_OS_LOCK_OBJ_LOCK_VALUE	 =	0x44;

#define _SlTime_t               _SlNonOsTime_t

#define _SlSyncObj_t            _SlNonOsSemObj_t

#define _SlLockObj_t            _SlNonOsSemObj_t

const uint8_t SL_OS_WAIT_FOREVER   =   NONOS_WAIT_FOREVER;

const uint8_t SL_OS_RET_CODE_OK    =   NONOS_RET_OK;

const uint8_t SL_OS_NO_WAIT        =   NONOS_NO_WAIT;


    /*****************************************************************************/
    /* Function prototypes                                                       */
    /*****************************************************************************/

    /*!
    	\brief 	This function creates a sync object

    	The sync object is used for synchronization between different thread or ISR and
    	a thread.

    	\param	pSyncObj	-	pointer to the sync object control block

    	\return upon successful creation the function return 0
    			Otherwise, a negative value indicating the error code shall be returned
    	\note
    	\warning
    */
#define _SlNonOsSyncObjCreate(pSyncObj)			_SlNonOsSemSet(pSyncObj,NON_OS_SYNC_OBJ_CLEAR_VALUE)

    /*!
    	\brief 	This function deletes a sync object

    	\param	pSyncObj	-	pointer to the sync object control block

    	\return upon successful deletion the function should return 0
    			Otherwise, a negative value indicating the error code shall be returned
    	\note
    	\warning
    */
//#define _SlNonOsSyncObjDelete(pSyncObj)			_SlNonOsSemSet(pSyncObj,0)

    /*!
    	\brief 		This function generates a sync signal for the object.

    	All suspended threads waiting on this sync object are resumed

    	\param		pSyncObj	-	pointer to the sync object control block

    	\return 	upon successful signaling the function should return 0
    				Otherwise, a negative value indicating the error code shall be returned
    	\note		the function could be called from ISR context
    	\warning
    */
//#define _SlNonOsSyncObjSignal(pSyncObj)			_SlNonOsSemSet(pSyncObj,NON_OS_SYNC_OBJ_SIGNAL_VALUE)

    /*!
    	\brief 	This function waits for a sync signal of the specific sync object

    	\param	pSyncObj	-	pointer to the sync object control block
    	\param	Timeout		-	numeric value specifies the maximum number of mSec to
    							stay suspended while waiting for the sync signal
    							Currently, the simple link driver uses only two values:
    								- NONOS_WAIT_FOREVER
    								- NONOS_NO_WAIT

    	\return upon successful reception of the signal within the timeout window return 0
    			Otherwise, a negative value indicating the error code shall be returned
    	\note
    	\warning
    */
//#define _SlNonOsSyncObjWait(pSyncObj , Timeout)	_SlNonOsSemGet(pSyncObj,NON_OS_SYNC_OBJ_SIGNAL_VALUE,NON_OS_SYNC_OBJ_CLEAR_VALUE,Timeout)

    /*!
    	\brief 	This function clears a sync object

    	\param	pSyncObj	-	pointer to the sync object control block

    	\return upon successful clearing the function should return 0
    			Otherwise, a negative value indicating the error code shall be returned
    	\note
    	\warning
    */
//#define _SlNonOsSyncObjClear(pSyncObj)			_SlNonOsSemSet(pSyncObj,NON_OS_SYNC_OBJ_CLEAR_VALUE)

    /*!
    	\brief 	This function creates a locking object.

    	The locking object is used for protecting a shared resources between different
    	threads.

    	\param	pLockObj	-	pointer to the locking object control block

    	\return upon successful creation the function should return 0
    			Otherwise, a negative value indicating the error code shall be returned
    	\note
    	\warning
    */
#define _SlNonOsLockObjCreate(pLockObj)			_SlNonOsSemSet(pLockObj,NON_OS_LOCK_OBJ_UNLOCK_VALUE)

    /*!
    	\brief 	This function deletes a locking object.

    	\param	pLockObj	-	pointer to the locking object control block

    	\return upon successful deletion the function should return 0
    			Otherwise, a negative value indicating the error code shall be returned
    	\note
    	\warning
    */
//#define _SlNonOsLockObjDelete(pLockObj)			_SlNonOsSemSet(pLockObj,0)

    /*!
    	\brief 	This function locks a locking object.

    	All other threads that call this function before this thread calls
    	the _SlNonOsLockObjUnlock would be suspended

    	\param	pLockObj	-	pointer to the locking object control block
    	\param	Timeout		-	numeric value specifies the maximum number of mSec to
    							stay suspended while waiting for the locking object
    							Currently, the simple link driver uses only two values:
    								- NONOS_WAIT_FOREVER
    								- NONOS_NO_WAIT


    	\return upon successful reception of the locking object the function should return 0
    			Otherwise, a negative value indicating the error code shall be returned
    	\note
    	\warning
    */
//#define _SlNonOsLockObjLock(pLockObj , Timeout)	_SlNonOsSemGet(pLockObj,NON_OS_LOCK_OBJ_UNLOCK_VALUE,NON_OS_LOCK_OBJ_LOCK_VALUE,Timeout)

    /*!
    	\brief 	This function unlock a locking object.

    	\param	pLockObj	-	pointer to the locking object control block

    	\return upon successful unlocking the function should return 0
    			Otherwise, a negative value indicating the error code shall be returned
    	\note
    	\warning
    */
//#define _SlNonOsLockObjUnlock(pLockObj)			_SlNonOsSemSet(pLockObj,NON_OS_LOCK_OBJ_UNLOCK_VALUE)

/*****************************************************************************

    Overwrite SimpleLink driver OS adaptation functions


 *****************************************************************************/

#undef sl_SyncObjCreate
#define sl_SyncObjCreate(pSyncObj,pName)           _SlNonOsSemSet(pSyncObj,NON_OS_SYNC_OBJ_CLEAR_VALUE)

#undef sl_LockObjCreate
#define sl_LockObjCreate(pLockObj,pName)            _SlNonOsSemSet(pLockObj,NON_OS_LOCK_OBJ_UNLOCK_VALUE)
	
/*!
	\brief type definition for the return values of this adaptation layer
*/
typedef int8_t _SlNonOsRetVal_t;

/*!
	\brief type definition for a time value
*/
typedef uint8_t _SlNonOsTime_t;

/*!
	\brief 	type definition for a sync object container

	Sync object is object used to synchronize between two threads or thread and interrupt handler.
	One thread is waiting on the object and the other thread send a signal, which then
	release the waiting thread.
	The signal must be able to be sent from interrupt context.
	This object is generally implemented by binary semaphore or events.
*/
typedef uint8_t _SlNonOsSemObj_t;	
	
class cc3100_driver;

class cc3100_nonos
{

public:

    cc3100_nonos(cc3100_driver &driver);

    ~cc3100_nonos();

    /*!
    	\brief 	This function call the pEntry callback from a different context

    	\param	pEntry		-	pointer to the entry callback function

    	\param	pValue		- 	pointer to any type of memory structure that would be
    							passed to pEntry callback from the execution thread.

    	\param	flags		- 	execution flags - reserved for future usage

    	\return upon successful registration of the spawn the function return 0
    			(the function is not blocked till the end of the execution of the function
    			and could be returned before the execution is actually completed)
    			Otherwise, a negative value indicating the error code shall be returned
    	\note
    	\warning
    */
    _SlNonOsRetVal_t _SlNonOsSpawn(_SlSpawnEntryFunc_t pEntry , void* pValue , uint32_t flags);


    /*!
    	\brief 	This function must be called from the main loop in non-os paltforms

    	\param	None

    	\return 0 - No more activities
    			1 - Activity still in progress
    	\note
    	\warning
    */
    _SlNonOsRetVal_t _SlNonOsMainLoopTask(void);

    _SlNonOsRetVal_t _SlNonOsSemGet(_SlNonOsSemObj_t* pSyncObj, _SlNonOsSemObj_t WaitValue, _SlNonOsSemObj_t SetValue, _SlNonOsTime_t Timeout);
    _SlNonOsRetVal_t _SlNonOsSemSet(_SlNonOsSemObj_t* pSemObj , _SlNonOsSemObj_t Value);
//    _SlNonOsRetVal_t _SlNonOsSpawn(_SlSpawnEntryFunc_t pEntry , void* pValue , uint32_t flags);
//    _SlNonOsRetVal_t sl_SyncObjCreate(_SlNonOsSemObj_t* pSemObj, _SlNonOsSemObj_t NON_OS_SYNC_OBJ_CLEAR_VALUE);
    _SlNonOsRetVal_t sl_SyncObjDelete(_SlNonOsSemObj_t* pSemObj, _SlNonOsSemObj_t );
    _SlNonOsRetVal_t sl_SyncObjSignal(_SlNonOsSemObj_t* pSemObj, _SlNonOsSemObj_t NON_OS_SYNC_OBJ_SIGNAL_VALUE);
    _SlNonOsRetVal_t sl_SyncObjSignalFromIRQ(_SlNonOsSemObj_t* pSemObj, _SlNonOsSemObj_t NON_OS_SYNC_OBJ_SIGNAL_VALUE);
//    _SlNonOsRetVal_t sl_LockObjCreate(_SlNonOsSemObj_t* pSemObj, _SlNonOsSemObj_t NON_OS_LOCK_OBJ_UNLOCK_VALUE);
    _SlNonOsRetVal_t sl_LockObjDelete(_SlNonOsSemObj_t* pSemObj, _SlNonOsSemObj_t );
    _SlNonOsRetVal_t sl_LockObjUnlock(_SlNonOsSemObj_t* pSemObj, _SlNonOsSemObj_t NON_OS_LOCK_OBJ_UNLOCK_VALUE);
    _SlNonOsRetVal_t sl_SyncObjWait(_SlNonOsSemObj_t* pSyncObj, _SlNonOsSemObj_t NON_OS_SYNC_OBJ_SIGNAL_VALUE, _SlNonOsSemObj_t NON_OS_SYNC_OBJ_CLEAR_VALUE, _SlNonOsTime_t Timeout);
    _SlNonOsRetVal_t sl_LockObjLock(_SlNonOsSemObj_t* pSyncObj, _SlNonOsSemObj_t NON_OS_LOCK_OBJ_UNLOCK_VALUE, _SlNonOsSemObj_t NON_OS_LOCK_OBJ_LOCK_VALUE, _SlNonOsTime_t Timeout);
    

#if (defined(_SlSyncWaitLoopCallback))
    void _SlSyncWaitLoopCallback(void);
#endif

private:

    cc3100_driver &_driver;

};//class

}//namespace mbed_cc3100

#endif /* !SL_PLATFORM_MULTI_THREADED */



#endif


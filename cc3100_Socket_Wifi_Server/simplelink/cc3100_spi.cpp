/*
 * spi.cpp mbed
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

#include "cc3100_simplelink.h"
#include "cc3100_spi.h"


namespace mbed_cc3100 {

P_EVENT_HANDLER   pIraEventHandler = 0;
uint8_t           IntIsMasked;

cc3100_spi::cc3100_spi(PinName cc3100_irq, PinName cc3100_nHIB, PinName cc3100_cs, SPI cc3100_spi, cc3100_driver &driver)
    : _wlan_irq(cc3100_irq), _wlan_nHIB(cc3100_nHIB), _wlan_cs(cc3100_cs), _wlan_spi(cc3100_spi), _driver(driver)
{

    _wlan_spi.format(8,0);
    _wlan_spi.frequency(16000000);
    _wlan_irq.rise(this, &cc3100_spi::IntSpiGPIOHandler);      //_SlDrvRxIrqHandler is triggered after IntSpiGPIOHandler
    _wlan_nHIB = 0;
    _wlan_cs = 1;
    wait_ms(200);
    
    
}

cc3100_spi::~cc3100_spi()
{

}

int cc3100_spi::spi_Close(Fd_t fd)
{
    // Disable WLAN Interrupt ...
    cc3100_InterruptDisable();

    return NONOS_RET_OK;
}

void cc3100_spi::cc3100_InterruptEnable()
{
    __enable_irq();
}

void cc3100_spi::cc3100_InterruptDisable()
{
    __disable_irq();
}

void cc3100_spi::CC3100_disable()
{
    _wlan_nHIB = 0;
}

void cc3100_spi::CC3100_enable()
{
    
    _wlan_nHIB = 1;
}

Fd_t cc3100_spi::spi_Open(int8_t *ifName, uint32_t flags)
{

    // 50 ms delay
    wait_ms(50);

    // Enable WLAN interrupt
    cc3100_InterruptEnable();

    return NONOS_RET_OK;
}

int cc3100_spi::spi_Write(Fd_t fd, uint8_t *pBuff, int len)
{

    int len_to_return = len;

    _wlan_cs = 0;

    while(len) {
        _wlan_spi.write(*pBuff++);
        len--;
    }

    _wlan_cs = 1;

    return len_to_return;
}

int cc3100_spi::spi_Read(Fd_t fd, uint8_t *pBuff, int len)
{
    int i = 0;

    _wlan_cs = 0;

    for (i = 0; i < len; i++) {
        pBuff[i] = _wlan_spi.write(0xFF);
    }

    _wlan_cs = 1;
    
    return len;
}

void cc3100_spi::IntSpiGPIOHandler(void)
{
    
    if(_wlan_irq){
        _driver._SlDrvRxIrqHandler(0);
    }
}

/*!
    \brief register an interrupt handler for the host IRQ

    \param[in]      InterruptHdl    -    pointer to interrupt handler function

    \param[in]      pValue          -    pointer to a memory strcuture that is
                    passed to the interrupt handler.

    \return         upon successful registration, the function shall return 0.
                    Otherwise, -1 shall be returned

    \sa
    \note           If there is already registered interrupt handler, the
                    function should overwrite the old handler with the new one
    \warning
*/
int cc3100_spi::registerInterruptHandler(P_EVENT_HANDLER InterruptHdl , void* pValue)
{

    pIraEventHandler = InterruptHdl;
    return 0;
}

/*!
    \brief     Unmasks the Host IRQ

    \param[in]      none

    \return         none

    \warning
*/
void cc3100_spi::UnMaskIntHdlr()
{
    IntIsMasked = FALSE;
}

/*!
    \brief      Masks the Host IRQ

    \param[in]      none

    \return         none

    \warning
*/
void cc3100_spi::MaskIntHdlr()
{
    IntIsMasked = TRUE;
}

}//namespace mbed_cc3100






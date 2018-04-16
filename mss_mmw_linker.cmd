/*----------------------------------------------------------------------------*/
/* Linker Settings                                                            */
--retain="*(.intvecs)"

/*----------------------------------------------------------------------------*/
/* Section Configuration                                                      */
SECTIONS
{
    systemHeap : {} > DATA_RAM
    .hwaBufs: load = HWA_RAM, type = NOINIT

    /* L3_RAM has code that is overlaid with data, so data must be
       marked uninitialized. Application can initialize this section
       using _L3ram_* symbols defined below. Code should be written carefully as
       these are linker symbols (see for example http://e2e.ti.com/support/development_tools/compiler/f/343/t/92002 ):
        
        extern far uint8_t _L3ram_start; // the type here does not matter
        extern far uint8_t _L3ram_size;  // the type here does not matter

        memset((void *)_symval(&_L3ram_start), 0, (uint32_t) _symval(&_L3ram_size));
    */ 
    .l3ram: type=NOINIT, start(_L3ram_start), size(_L3ram_size), load=L3_RAM PAGE 1
   
    /* Overlay one-time/init-time (and non-critical in cycles) with L3 data,
       will be erased during data path processing. Note do not put any
       code that is required related to start/stop/reconfig processing */
    .overlay:
    {
        main.oer4f (.text:MmwDemo_initTask)
        libmailbox_xwr14xx.aer4f (.text:Mailbox_init)
        data_path.oer4f (.text:MmwDemo_edmaInit)
        libuart_xwr14xx.aer4f  (.text:UartSci_open)
        libmmwave_xwr14xx.aer4f (.text:MMWave_init)
        libmmwave_xwr14xx.aer4f (.text:MMWave_initLink)
        libcrc_xwr14xx.aer4f  (.text:CRC_initConfigParams)
        libcrc_xwr14xx.aer4f  (.text:CRC_open)
        libmmwavelink_xwr14xx.aer4f (.text:rlDevicePowerOn)
        
        
    } > L3_RAM PAGE 0
   
}
/*----------------------------------------------------------------------------*/


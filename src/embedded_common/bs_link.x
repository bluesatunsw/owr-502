MEMORY
{
    RAM (RW)    : ORIGIN = 0x20000000,  LENGTH = 80K
    CCM (RWX)   : ORIGIN = 0x10000000,  LENGTH = 32K
    EXT (RX)    : ORIGIN = 0x00000040,  LENGTH = 4M-64
    INT (RX)    : ORIGIN = 0x08008000,  LENGTH = 96K
}

__reset = 0x08007F00;

/* Prevent optimising out */
EXTERN(main);
ENTRY(__reset);

EXTERN(__EXCEPTIONS);
EXTERN(__INTERRUPTS);

EXTERN(DefaultHandler);
EXTERN(HardFaultTrampoline);

PROVIDE(NonMaskableInt = DefaultHandler);
PROVIDE(MemoryManagement = DefaultHandler);
PROVIDE(BusFault = DefaultHandler);
PROVIDE(UsageFault = DefaultHandler);
PROVIDE(SecureFault = DefaultHandler);
PROVIDE(SVCall = DefaultHandler);
PROVIDE(DebugMonitor = DefaultHandler);
PROVIDE(PendSV = DefaultHandler);
PROVIDE(SysTick = DefaultHandler);

PROVIDE(DefaultHandler = DefaultHandler_);
PROVIDE(HardFault = HardFault_);

SECTIONS
{
    .ram ORIGIN(RAM) :
    {
        *(.data .data.*);
        *(.ram .ram.*);

        . = ALIGN(4);
    } > RAM AT>EXT

    .bss :
    {
        *(.bss .bss.*);
        *(COMMON); /* Uninitialized C statics */
        *(.uninit .uninit.*);

        . = ALIGN(4);
    } > RAM

    .ccm ORIGIN(CCM) :
    {
        __vector_table = .;
        /* We don't store the initial stack pointer here */
        LONG(0x80808080);
        /* Magic */
        LONG(0x08007F00);

        KEEP(*(.vector_table.exceptions));
        KEEP(*(.vector_table.interrupts));

        *(.ccm .ccm.*);

        . = ALIGN(4);
    } > CCM AT>EXT

    .ext :
    {
        *(.text .text.*);
        *(.HardFaultTrampoline);
        *(.HardFault.*);
        *(.rodata .rodata.*);
        *(.ext .ext.*);

        . = ALIGN(4096);
    } > EXT

    .int ORIGIN(INT) :
    {
        *(.int .int.*);

        . = ALIGN(4096);
    } > INT

    /DISCARD/ :
    {
        /* Unused exception related info that only wastes space */
        *(.ARM.exidx);
        *(.ARM.exidx.*);
        *(.ARM.extab.*);
    }
}

INCLUDE device.x

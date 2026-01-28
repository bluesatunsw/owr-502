use stm32g4::stm32g474::{DBGMCU, DCB, DWT, ITM};

pub unsafe fn setup_itm(dcb: &mut DCB, dwt: &mut DWT, dbgmcu: &mut DBGMCU, itm: &mut ITM) {
    dcb.enable_trace();
    dwt.enable_cycle_counter();
    unsafe {
        dbgmcu
            .cr()
            .modify(|_, w| w.trace_ioen().set_bit().trace_mode().bits(0b00));
        itm.lar.write(0xC5ACCE55); // Unlock ITM registers
        itm.tcr.write(0x00010005); // AdvTBus ID = 1, Global ITM enable

        // Enable all
        itm.tpr.write(0b1111); // Enables STIM 0 to 3
        itm.ter[0].write(0xFFFF_FFFF); // Enables STIM[31:0]
    }
}

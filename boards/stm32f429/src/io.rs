use core::fmt::{Arguments, Write};




#[cfg(not(test))]
#[no_mangle]
#[lang = "panic_fmt"]
/// Panic handler
pub unsafe extern "C" fn panic_fmt(args: Arguments, file: &'static str, line: u32)  {


    
}
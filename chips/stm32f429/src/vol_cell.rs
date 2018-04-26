use core::ops::{BitOrAssign, BitAndAssign, BitOr, BitAnd, Shl, Shr, Not};

pub trait Zero {
    fn zero() -> Self; 
}

impl Zero for u8 { fn zero() -> Self { 0 } }
impl Zero for u16 { fn zero() -> Self { 0 } }
impl Zero for u32 { fn zero() -> Self { 0 } }
impl Zero for u64 { fn zero() -> Self { 0 } }
impl Zero for usize { fn zero() -> Self { 0 } }

#[derive(Copy, Clone)]
#[repr(C)]
pub struct VolatileCell<T> {
    value: T,
}


#[allow(dead_code)]
impl<T: BitAnd<T, Output = T>+BitOr<T, Output = T>+BitOrAssign<T>+BitAndAssign<T>+Shl<T, Output = T>+Shr<T, Output = T>+Not<Output = T>+Copy+Zero+PartialEq> VolatileCell<T> {    
    pub const fn new(value: T) -> Self {
        VolatileCell { value: value }
    }

    #[inline]
    pub fn get(&self) -> T {
        unsafe { ::core::ptr::read_volatile(&self.value) }
    }

    #[inline]
    pub fn set(&self, value: T) {
        unsafe { ::core::ptr::write_volatile(&self.value as *const T as *mut T, value) }
    }

    pub fn mask_set(&self, mask: T, shift: T, val: T) {
        let mut reg = self.get();
        reg = (reg  & !(mask << shift)) | (val << shift);
        self.set(reg);
    }

    pub fn mask_get(&self, mask: T, shift: T) -> T {
        (self.get() & !(mask << shift)) >> shift
    }

    pub fn check(&self, val: T) {
        let mut reg = self.get();
        reg |= val;
        self.set(reg);
    }

    pub fn uncheck(&self, val: T) {
        let mut reg = self.get();
        reg &= !val;
        self.set(reg);
    }

    pub fn test(&self, val: T) -> bool {
        self.get() & val != T::zero()
}
}
use core::mem::MaybeUninit;

pub(crate) struct RingBuffer<T, const SIZE: usize = 32> {
    inner: [MaybeUninit<T>; SIZE],
    read_index: usize,
    write_index: usize,
}

impl<T: Copy, const SIZE: usize> RingBuffer<T, SIZE> {
    pub(crate) const fn new() -> Self {
        Self {
            inner: [MaybeUninit::uninit(); SIZE],
            read_index: 0,
            write_index: 0,
        }
    }

    fn wrapping_inc_mut(n: &mut usize) {
        if *n == SIZE - 1 {
            *n = 0;
        } else {
            *n += 1;
        }
    }

    fn wrapping_dec(n: usize) -> usize {
        if n == 0 {
            SIZE - 1
        } else {
            n - 1
        }
    }

    pub(crate) fn pop(&mut self) -> Option<T> {
        if self.read_index == self.write_index {
            None
        } else {
            let res = self.inner[self.read_index];
            Self::wrapping_inc_mut(&mut self.read_index);
            // Safety: The read_index == write_index check above ensures we cannot pop an
            // uninitialised element
            unsafe { Some(res.assume_init()) }
        }
    }

    pub(crate) fn push(&mut self, item: T) -> bool {
        if self.write_index == Self::wrapping_dec(self.read_index) {
            false
        } else {
            self.inner[self.write_index] = MaybeUninit::new(item);
            Self::wrapping_inc_mut(&mut self.write_index);
            true
        }
    }

    pub(crate) fn clear(&mut self) {
        self.read_index = 0;
        self.write_index = 0;
    }

    pub(crate) fn is_empty(&self) -> bool {
        self.read_index == self.write_index
    }
}
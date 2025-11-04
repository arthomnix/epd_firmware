use core::hash::Hasher;
use siphasher::sip::SipHasher;
use tickv::{ErrorCode, TicKV};
use eepy_sys::header::{slot, slot_of_addr};
use eepy_sys::kv_store::{AppendKeyError, DeleteKeyError, KvStoreSyscall, PutKeyError, ReadKeyArgs, ReadKeyError, WriteKeyArgs};
use eepy_sys::SafeResult;
use crate::exception::StackFrame;
use crate::tickv::{with_tickv, EepyFlashController};

pub(super) fn handle_kv_store(stack_values: &mut StackFrame) {
    match KvStoreSyscall::from_repr(stack_values.r0) {
        Some(KvStoreSyscall::ReadKey) => handle_kv_get(stack_values),
        Some(KvStoreSyscall::AppendKey) => handle_kv_append(stack_values),
        Some(KvStoreSyscall::PutKey) => handle_kv_put(stack_values),
        Some(KvStoreSyscall::InvalidateKey) => handle_kv_invalidate(stack_values),
        Some(KvStoreSyscall::ZeroiseKey) => handle_kv_zeroise(stack_values),
        None => panic!("illegal syscall"),
    }
}

fn hash(stack_values: &mut StackFrame, key: &[u8]) -> u64 {
    let slot_number = slot_of_addr(stack_values.pc);
    let slot_header = unsafe { slot(slot_number) };
    let program_name = unsafe { core::slice::from_raw_parts((*slot_header).name_ptr, (*slot_header).name_len) };

    let mut hasher = SipHasher::new();
    hasher.write(program_name);
    hasher.write(key);
    hasher.finish()
}

fn append_key_gc(tickv: &TicKV<EepyFlashController, 4096>, key: u64, value: &[u8]) -> Result<(), ErrorCode> {
    let mut res = tickv.append_key(key, value).map(|_| ());
    if let Err(ErrorCode::RegionFull | ErrorCode::FlashFull) = res {
        res = tickv.garbage_collect().map(|_| ());
        if res.is_ok() {
            res = tickv.append_key(key, value).map(|_| ());
        }
    }
    res
}

fn handle_kv_get(stack_values: &mut StackFrame) {
    let args = stack_values.r1 as *const ReadKeyArgs;
    let key = unsafe { core::slice::from_raw_parts((*args).key_ptr, (*args).key_len) };
    let buf = unsafe { core::slice::from_raw_parts_mut((*args).buf_ptr, (*args).buf_len) };

    let hashed_key = hash(stack_values, key);
    let res = unsafe { with_tickv(|tickv| tickv.get_key(hashed_key, buf)) };

    let res_ptr = stack_values.r2 as *mut SafeResult<usize, ReadKeyError>;
    let res: SafeResult<usize, ReadKeyError> = res
        .map(|(_, read)| read)
        .map_err(|e| e.try_into().unwrap())
        .into();
    unsafe { res_ptr.write(res) };
}

fn handle_kv_append(stack_values: &mut StackFrame) {
    let args = stack_values.r1 as *const WriteKeyArgs;
    let key = unsafe { core::slice::from_raw_parts((*args).key_ptr, (*args).key_len) };
    let value = unsafe { core::slice::from_raw_parts((*args).value_ptr, (*args).value_len) };

    let hashed_key = hash(stack_values, key);
    let res = unsafe { with_tickv(|tickv| append_key_gc(tickv, hashed_key, value)) };

    let res_ptr = stack_values.r2 as *mut SafeResult<(), AppendKeyError>;
    let res: SafeResult<(), AppendKeyError> = res
        .map(|_| ())
        .map_err(|e| e.try_into().unwrap())
        .into();
    unsafe { res_ptr.write(res) };
}

fn handle_kv_put(stack_values: &mut StackFrame) {
    let args = stack_values.r1 as *const WriteKeyArgs;
    let key = unsafe { core::slice::from_raw_parts((*args).key_ptr, (*args).key_len) };
    let value = unsafe { core::slice::from_raw_parts((*args).value_ptr, (*args).value_len) };

    let hashed_key = hash(stack_values, key);
    let mut res = unsafe { with_tickv(|tickv| append_key_gc(tickv, hashed_key, value)) };

    // if there is already a value for this key, invalidate it and write the new value
    if let Err(ErrorCode::KeyAlreadyExists) = res {
        res = unsafe { with_tickv(|tickv| tickv.invalidate_key(hashed_key).map(|_| ())) };
        if res.is_ok() {
            res = unsafe { with_tickv(|tickv| append_key_gc(tickv, hashed_key, value)) };
        }
    }

    let res_ptr = stack_values.r2 as *mut SafeResult<(), PutKeyError>;
    let res: SafeResult<(), PutKeyError> = res
        .map(|_| ())
        .map_err(|e| e.try_into().unwrap())
        .into();
    unsafe { res_ptr.write(res) };
}

fn handle_kv_invalidate(stack_values: &mut StackFrame) {
    let key = unsafe { core::slice::from_raw_parts(stack_values.r1 as *const u8, stack_values.r2) };
    let hashed_key = hash(stack_values, key);
    let res = unsafe { with_tickv(|tickv| tickv.invalidate_key(hashed_key)) };

    let res_ptr = stack_values.r3 as *mut SafeResult<(), DeleteKeyError>;
    let res: SafeResult<(), DeleteKeyError> = res
        .map(|_| ())
        .map_err(|e| e.try_into().unwrap())
        .into();
    unsafe { res_ptr.write(res) };
}

fn handle_kv_zeroise(stack_values: &mut StackFrame) {
    let key = unsafe { core::slice::from_raw_parts(stack_values.r1 as *const u8, stack_values.r2) };
    let hashed_key = hash(stack_values, key);
    let res = unsafe { with_tickv(|tickv| tickv.zeroise_key(hashed_key)) };

    let res_ptr = stack_values.r3 as *mut SafeResult<(), DeleteKeyError>;
    let res: SafeResult<(), DeleteKeyError> = res
        .map(|_| ())
        .map_err(|e| e.try_into().unwrap())
        .into();
    unsafe { res_ptr.write(res) };
}
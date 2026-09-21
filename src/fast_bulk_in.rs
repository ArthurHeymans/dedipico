use core::sync::atomic::{Ordering, compiler_fence};

use embassy_rp::pac;
use embassy_usb::driver::EndpointError;

use crate::config::{BULK_BLOCK_SIZE, USB_MAX_PACKET_SIZE};

/// Double-buffered writer for DediProg's fixed bulk-IN endpoint (EP2 IN).
///
/// Embassy's RP2040 driver currently allocates one DPRAM buffer per endpoint
/// and does not expose the controller's double-buffer mode. This wrapper reads
/// the address Embassy allocated from the endpoint control register instead of
/// depending on allocation order, then owns EP2's buffer control while active.
pub struct FastBulkIn {
    buffer0_addr: usize,
    next_buf: usize,
    next_pid: bool,
}

impl FastBulkIn {
    const EP_INDEX: usize = 2;
    const EP_CONTROL_INDEX: usize = Self::EP_INDEX - 1;
    const RESERVE_EP_INDEX: usize = 15;
    const RESERVE_EP_CONTROL_INDEX: usize = Self::RESERVE_EP_INDEX - 1;
    const BUFFER_STRIDE: usize = USB_MAX_PACKET_SIZE as usize;

    pub fn new_ep2() -> Option<Self> {
        let dpram = pac::USB_DPRAM;
        let control = dpram.ep_in_control(Self::EP_CONTROL_INDEX).read();
        let buffer0_addr = usize::from(control.buffer_address());
        let reserve_addr = usize::from(
            dpram
                .ep_in_control(Self::RESERVE_EP_CONTROL_INDEX)
                .read()
                .buffer_address(),
        );
        if reserve_addr != buffer0_addr + Self::BUFFER_STRIDE {
            return None;
        }

        dpram.ep_in_control(Self::EP_CONTROL_INDEX).modify(|w| {
            w.set_interrupt_per_buff(true);
            w.set_interrupt_per_double_buff(false);
            w.set_double_buffered(true);
        });
        dpram.ep_in_buffer_control(Self::EP_INDEX).write(|w| {
            w.set_reset(true);
            w.set_pid(0, true);
            w.set_pid(1, false);
        });

        Some(Self {
            buffer0_addr,
            next_buf: 0,
            // Matches Embassy's first-packet behavior after endpoint enable: it
            // stores `pid = !current_pid`, where current_pid starts true.
            next_pid: false,
        })
    }

    async fn write_packet(&mut self, packet: &[u8]) -> Result<(), EndpointError> {
        if packet.len() > Self::BUFFER_STRIDE {
            return Err(EndpointError::BufferOverflow);
        }

        let dpram = pac::USB_DPRAM;
        let buf_index = self.next_buf;

        loop {
            if !dpram.ep_in_control(Self::EP_CONTROL_INDEX).read().enable() {
                return Err(EndpointError::Disabled);
            }
            if !dpram
                .ep_in_buffer_control(Self::EP_INDEX)
                .read()
                .available(buf_index)
            {
                break;
            }
            embassy_futures::yield_now().await;
        }

        compiler_fence(Ordering::SeqCst);
        // SAFETY: buffer0_addr is the enabled EP2 address allocated by Embassy.
        // Double-buffer mode reserves the adjacent max-packet-sized buffer.
        let dst = unsafe {
            core::slice::from_raw_parts_mut(
                (pac::USB_DPRAM.as_ptr() as *mut u8)
                    .add(self.buffer0_addr + buf_index * Self::BUFFER_STRIDE),
                packet.len(),
            )
        };
        dst.copy_from_slice(packet);
        compiler_fence(Ordering::SeqCst);

        if !dpram.ep_in_control(Self::EP_CONTROL_INDEX).read().enable() {
            return Err(EndpointError::Disabled);
        }

        dpram.ep_in_buffer_control(Self::EP_INDEX).modify(|w| {
            w.set_length(buf_index, packet.len() as u16);
            w.set_pid(buf_index, self.next_pid);
            w.set_full(buf_index, true);
        });
        cortex_m::asm::delay(12);
        dpram.ep_in_buffer_control(Self::EP_INDEX).modify(|w| {
            w.set_length(buf_index, packet.len() as u16);
            w.set_pid(buf_index, self.next_pid);
            w.set_full(buf_index, true);
            w.set_available(buf_index, true);
        });

        self.next_pid = !self.next_pid;
        self.next_buf ^= 1;
        Ok(())
    }

    pub async fn write_block(&mut self, data: &[u8; BULK_BLOCK_SIZE]) -> Result<(), EndpointError> {
        for chunk in data.chunks(USB_MAX_PACKET_SIZE as usize) {
            self.write_packet(chunk).await?;
        }
        Ok(())
    }
}

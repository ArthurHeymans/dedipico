#![no_std]

pub mod aux {
    pub const PACKET_LEN: usize = 64;
    pub const HEADER_LEN: usize = 3;
    pub const MAX_PAYLOAD_LEN: usize = PACKET_LEN - HEADER_LEN;

    pub const CMD_GPIO_GET_STATE: u8 = 0x01;
    pub const CMD_GPIO_SET_DIRECTION: u8 = 0x02;
    pub const CMD_GPIO_SET_OUTPUT: u8 = 0x03;
    pub const CMD_GPIO_PULSE_LOW: u8 = 0x04;
    pub const CMD_UART_SET_BAUD: u8 = 0x10;
    pub const CMD_UART_WRITE: u8 = 0x11;

    pub const EVT_RESPONSE: u8 = 0x80;
    pub const EVT_GPIO_STATE: u8 = 0x81;
    pub const EVT_UART_DATA: u8 = 0x90;

    pub const STATUS_OK: u8 = 0;
    pub const STATUS_INVALID: u8 = 1;
    pub const STATUS_BUSY: u8 = 2;

    pub const GPIO_RESET: u8 = 1 << 0;
    pub const GPIO_POWER: u8 = 1 << 1;
    pub const GPIO_POWER_STATE: u8 = 1 << 2;
    pub const GPIO_AUX: u8 = 1 << 3;
    pub const GPIO_COUNT: u8 = 4;

    pub const CAP_OPEN_DRAIN: u8 = 1 << 0;
    pub const CAP_PULSE: u8 = 1 << 1;

    pub fn payload(packet: &[u8]) -> Option<&[u8]> {
        let header = packet.get(..HEADER_LEN)?;
        let len = usize::from(header[2]);
        packet.get(HEADER_LEN..HEADER_LEN.checked_add(len)?)
    }

    pub fn encode<'a>(
        packet: &'a mut [u8; PACKET_LEN],
        kind: u8,
        request_id: u8,
        payload: &[u8],
    ) -> Option<&'a [u8]> {
        if payload.len() > MAX_PAYLOAD_LEN {
            return None;
        }

        packet[0] = kind;
        packet[1] = request_id;
        packet[2] = payload.len() as u8;
        packet[HEADER_LEN..HEADER_LEN + payload.len()].copy_from_slice(payload);
        Some(&packet[..HEADER_LEN + payload.len()])
    }
}

#[cfg(test)]
mod tests {
    use super::aux::*;

    #[test]
    fn packet_round_trip() {
        let mut packet = [0; PACKET_LEN];
        let encoded = encode(&mut packet, CMD_UART_WRITE, 7, b"hello").unwrap();

        assert_eq!(encoded[0], CMD_UART_WRITE);
        assert_eq!(encoded[1], 7);
        assert_eq!(payload(encoded), Some(b"hello".as_slice()));
    }

    #[test]
    fn rejects_truncated_payload() {
        assert_eq!(payload(&[CMD_UART_WRITE, 1, 4, 1, 2]), None);
    }

    #[test]
    fn rejects_oversized_payload() {
        let mut packet = [0; PACKET_LEN];
        assert!(encode(&mut packet, CMD_UART_WRITE, 1, &[0; MAX_PAYLOAD_LEN + 1]).is_none());
    }
}

use crate::{mem::Mem, cpu::CPU};

const RAM : u16 = 0x0000;
const RAM_END : u16 = 0x1FFF;
const PPU : u16 = 0x200;
const PPU_END : u16 = 0x3FFF;

pub struct Bus {
    cpu_vram: [u8; 2048],
    cycle : u8,
}

impl Mem for Bus {
    fn mem_read(&mut self, address : u16) -> u8 {
        match address {
            (RAM..=RAM_END) => {
                self.cpu_vram[(address & 0b0000_0111_1111_1111) as usize]
            },
            (PPU..=PPU_END) => {
                todo!("implement PPU")
            },
            _ => {
                println!("OOB MEM ACCESS @ {:x}", address);
                0
            },
        }
    }

    fn mem_write(&mut self, address : u16, data : u8) {
        match address {
            (RAM..=RAM_END) => {
                self.cpu_vram[(address & 0b0000_0111_1111_1111) as usize] = data
            },
            (PPU..=PPU_END) => {
                todo!("implement PPU")
            },
            _ => {
                println!("OOB MEM ACCESS @ {:x}", address);
            },
        }
    } 
}

impl Bus {
    
}

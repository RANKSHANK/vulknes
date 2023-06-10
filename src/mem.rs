
pub trait Mem {
    fn mem_read(&mut self, address : u16) -> u8;

    fn mem_write(&mut self, address : u16, data : u8);

    fn mem_read_u16(&mut self, address : u16) -> u16{
        (self.mem_read(address) as u16) | ((self.mem_read(address + 1) as u16) << 8)
    }
    
    fn mem_write_u16(&mut self, address : u16, data : u16) {
        self.mem_write(address, (data & 0xFF) as u8);
        self.mem_write(address + 1, (data >> 8) as u8);
    }
}

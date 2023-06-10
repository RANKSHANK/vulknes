use crate::mem::Mem;
use crate::bus::Bus;

const CARRY : u8 = 0b0000_0001;
const ZERO : u8 = 0b0000_0010;
const INTERRUPT_DISABLE : u8 = 0b0000_0100;
const DECIMAL_MODE : u8 = 0b0000_1000;
const BREAK : u8 = 0b0001_0000;
const BREAK2 : u8 = 0b0010_0000;
const OVERFLOW : u8 = 0b0100_0000;
const NEGATIVE : u8 = 0b1000_0000;
const STACK : u16 = 0x0100;
const STACK_RESET : u8 = 0xFD;

#[derive(Debug, Clone, Copy)]
pub enum AddressingMode {
    Absolute,
    AbsoluteX,
    AbsoluteY,
    Immediate,
    Implied,
    IndirectX,
    IndirectY,
    ZeroPage,
    ZeroPageX,
    ZeroPageY,
}

pub struct CPU {
    pub register_a : u8,
    pub register_x : u8,
    pub register_y : u8,
    pub stack_ptr : u8,
    pub status : u8,
    pub instruction_ptr : u16,
    instruction_set : [Option<Instruction>; 256],
    pub bus : Bus, 
}

#[derive(Clone, Copy)]
pub struct Instruction {
    pub extra_bytes : u8,
    pub cycles : u8,
    pub mode : AddressingMode,
    pub function : fn(&mut CPU, &AddressingMode),
}

impl Instruction {
    fn new(extra_bytes : u8, cycles : u8, mode : AddressingMode, function : fn(&mut CPU, &AddressingMode)) -> Self{
        Instruction{extra_bytes, cycles, mode, function}
    }
}

impl Mem for CPU {
    fn mem_read(&mut self, address : u16) -> u8 {
        self.bus.mem_read(address)
    }

    fn mem_write(&mut self, address : u16, data : u8){
        self.bus.mem_write(address, data);
    }
}

impl CPU {

    pub fn new(bus : Bus) -> Self {
        let mut cpu = CPU{
            register_a: 0,
            register_x: 0,
            register_y: 0,
            stack_ptr: STACK_RESET,
            status: 0,
            instruction_ptr: 0x8000,
            instruction_set: [None; 256],
            bus,
        };
        //adc
        cpu.instruction_set[0x69] = Some(Instruction::new(1, 2, AddressingMode::Immediate, CPU::adc));
        cpu.instruction_set[0x65] = Some(Instruction::new(1, 3, AddressingMode::ZeroPage, CPU::adc));
        cpu.instruction_set[0x75] = Some(Instruction::new(1, 4, AddressingMode::ZeroPageX, CPU::adc));
        cpu.instruction_set[0x6D] = Some(Instruction::new(2, 4, AddressingMode::Absolute, CPU::adc));
        cpu.instruction_set[0x7D] = Some(Instruction::new(2, 4, AddressingMode::AbsoluteX, CPU::adc));
        cpu.instruction_set[0x79] = Some(Instruction::new(2, 4, AddressingMode::AbsoluteY, CPU::adc));
        cpu.instruction_set[0x61] = Some(Instruction::new(1, 6, AddressingMode::IndirectX, CPU::adc));
        cpu.instruction_set[0x71] = Some(Instruction::new(1, 5, AddressingMode::IndirectY, CPU::adc));
        //and
        cpu.instruction_set[0x29] = Some(Instruction::new(1, 2, AddressingMode::Immediate, CPU::and));
        cpu.instruction_set[0x25] = Some(Instruction::new(1, 3, AddressingMode::ZeroPage, CPU::and));
        cpu.instruction_set[0x35] = Some(Instruction::new(1, 4, AddressingMode::ZeroPageX, CPU::and));
        cpu.instruction_set[0x2D] = Some(Instruction::new(2, 4, AddressingMode::Absolute, CPU::and));
        cpu.instruction_set[0x3D] = Some(Instruction::new(2, 4, AddressingMode::AbsoluteX, CPU::and));
        cpu.instruction_set[0x39] = Some(Instruction::new(2, 4, AddressingMode::AbsoluteY, CPU::and));
        cpu.instruction_set[0x21] = Some(Instruction::new(1, 6, AddressingMode::IndirectX, CPU::and));
        cpu.instruction_set[0x31] = Some(Instruction::new(1, 5, AddressingMode::IndirectY, CPU::and));
        //asl
        cpu.instruction_set[0x0A] = Some(Instruction::new(0, 2, AddressingMode::Implied, CPU::asl_accumulated));
        cpu.instruction_set[0x06] = Some(Instruction::new(1, 5, AddressingMode::ZeroPage, CPU::asl));
        cpu.instruction_set[0x16] = Some(Instruction::new(1, 6, AddressingMode::ZeroPageX, CPU::asl));
        cpu.instruction_set[0x0E] = Some(Instruction::new(2, 6, AddressingMode::Absolute, CPU::asl));
        cpu.instruction_set[0x1E] = Some(Instruction::new(2, 7, AddressingMode::AbsoluteX, CPU::asl));
        //bcc
        cpu.instruction_set[0x90] = Some(Instruction::new(1, 2, AddressingMode::Implied, CPU::bcc));
        //bcs
        cpu.instruction_set[0x80] = Some(Instruction::new(1, 2, AddressingMode::Implied, CPU::bcs));
        //beq
        cpu.instruction_set[0xF0] = Some(Instruction::new(1, 2, AddressingMode::Implied, CPU::beq));
        //bit
        cpu.instruction_set[0x24] = Some(Instruction::new(1, 3, AddressingMode::ZeroPage, CPU::bit));
        cpu.instruction_set[0x2C] = Some(Instruction::new(2, 4, AddressingMode::ZeroPage, CPU::bit));
        //bmi
        cpu.instruction_set[0x30] = Some(Instruction::new(1, 2, AddressingMode::Implied, CPU::bmi));
        //bne
        cpu.instruction_set[0xD0] = Some(Instruction::new(1, 2, AddressingMode::Implied, CPU::bne));
        //bpl
        cpu.instruction_set[0x10] = Some(Instruction::new(1, 2, AddressingMode::Implied, CPU::bpl)); 
        //bvc
        cpu.instruction_set[0x50] = Some(Instruction::new(1, 2, AddressingMode::Implied, CPU::bvc)); 
        //bvs
        cpu.instruction_set[0x70] = Some(Instruction::new(1, 2, AddressingMode::Implied, CPU::bpl));
        //clc
        cpu.instruction_set[0x18] = Some(Instruction::new(0, 2, AddressingMode::Implied, CPU::clc));
        //cld
        cpu.instruction_set[0xD8] = Some(Instruction::new(0, 2, AddressingMode::Implied, CPU::cld));
        //cli
        cpu.instruction_set[0x58] = Some(Instruction::new(0, 2, AddressingMode::Implied, CPU::cli));
        //clv
        cpu.instruction_set[0xB8] = Some(Instruction::new(0, 2, AddressingMode::Implied, CPU::clv));
        //cmp
        cpu.instruction_set[0xC9] = Some(Instruction::new(1, 2, AddressingMode::Immediate, CPU::cmp));
        cpu.instruction_set[0xC5] = Some(Instruction::new(1, 3, AddressingMode::ZeroPage, CPU::cmp));
        cpu.instruction_set[0xD5] = Some(Instruction::new(1, 4, AddressingMode::ZeroPageX, CPU::cmp));
        cpu.instruction_set[0xCD] = Some(Instruction::new(2, 4, AddressingMode::Absolute, CPU::cmp));
        cpu.instruction_set[0xDD] = Some(Instruction::new(2, 4, AddressingMode::AbsoluteX, CPU::cmp));
        cpu.instruction_set[0xD9] = Some(Instruction::new(2, 4, AddressingMode::AbsoluteY, CPU::cmp));
        cpu.instruction_set[0xC1] = Some(Instruction::new(1, 6, AddressingMode::IndirectX, CPU::cmp));
        cpu.instruction_set[0xD1] = Some(Instruction::new(1, 5, AddressingMode::IndirectY, CPU::cmp));
        //cpx
        cpu.instruction_set[0xE0] = Some(Instruction::new(1, 2, AddressingMode::Immediate, CPU::cpx));
        cpu.instruction_set[0xE4] = Some(Instruction::new(1, 3, AddressingMode::ZeroPage, CPU::cpx));
        cpu.instruction_set[0xEC] = Some(Instruction::new(2, 4, AddressingMode::Absolute, CPU::cpx));
        //cpy
        cpu.instruction_set[0xC0] = Some(Instruction::new(1, 2, AddressingMode::Immediate, CPU::cpy));
        cpu.instruction_set[0xC4] = Some(Instruction::new(1, 3, AddressingMode::ZeroPage, CPU::cpy));
        cpu.instruction_set[0xCC] = Some(Instruction::new(2, 4, AddressingMode::Absolute, CPU::cpy));
        //dec
        cpu.instruction_set[0xC6] = Some(Instruction::new(1, 5, AddressingMode::ZeroPage, CPU::dec));
        cpu.instruction_set[0xD6] = Some(Instruction::new(1, 6, AddressingMode::ZeroPageX, CPU::dec));
        cpu.instruction_set[0xCE] = Some(Instruction::new(2, 6, AddressingMode::Absolute, CPU::dec));
        cpu.instruction_set[0xDE] = Some(Instruction::new(2, 7, AddressingMode::AbsoluteX, CPU::dec));
        //dex
        cpu.instruction_set[0xCA] = Some(Instruction::new(0, 2, AddressingMode::Implied, CPU::dex));
        //dey
        cpu.instruction_set[0x88] = Some(Instruction::new(0, 2, AddressingMode::Implied, CPU::dey));
        //eor
        cpu.instruction_set[0x49] = Some(Instruction::new(1, 2, AddressingMode::Immediate, CPU::eor));
        cpu.instruction_set[0x45] = Some(Instruction::new(1, 3, AddressingMode::ZeroPage, CPU::eor));
        cpu.instruction_set[0x55] = Some(Instruction::new(1, 4, AddressingMode::ZeroPageX, CPU::eor));
        cpu.instruction_set[0x4D] = Some(Instruction::new(2, 4, AddressingMode::Absolute, CPU::eor));
        cpu.instruction_set[0x5D] = Some(Instruction::new(2, 4, AddressingMode::AbsoluteX, CPU::eor));
        cpu.instruction_set[0x59] = Some(Instruction::new(2, 4, AddressingMode::AbsoluteY, CPU::eor));
        cpu.instruction_set[0x41] = Some(Instruction::new(1, 6, AddressingMode::IndirectX, CPU::eor));
        cpu.instruction_set[0x51] = Some(Instruction::new(1, 5, AddressingMode::IndirectY, CPU::eor));
        // inc
        cpu.instruction_set[0xE6] = Some(Instruction::new(1, 5, AddressingMode::ZeroPage, CPU::inc));
        cpu.instruction_set[0xF6] = Some(Instruction::new(1, 6, AddressingMode::ZeroPageX, CPU::inc));
        cpu.instruction_set[0xEE] = Some(Instruction::new(2, 6, AddressingMode::Absolute, CPU::inc));
        cpu.instruction_set[0xFE] = Some(Instruction::new(2, 7, AddressingMode::AbsoluteX, CPU::inc));
        //inx
        cpu.instruction_set[0xE8] = Some(Instruction::new(0, 4, AddressingMode::Implied, CPU::inx));
        //iny
        cpu.instruction_set[0xC8] = Some(Instruction::new(0, 4, AddressingMode::Implied, CPU::iny));
        //jmp
        cpu.instruction_set[0x4C] = Some(Instruction::new(2, 3, AddressingMode::Implied, CPU::jmp_absolute));
        cpu.instruction_set[0x6C] = Some(Instruction::new(2, 5, AddressingMode::Implied, CPU::jmp_indirect));
        //jsr
        cpu.instruction_set[0x20] = Some(Instruction::new(2, 6, AddressingMode::Implied, CPU::jsr));
        //lda
        cpu.instruction_set[0xA9] = Some(Instruction::new(1, 2, AddressingMode::Immediate, CPU::lda));
        cpu.instruction_set[0xA5] = Some(Instruction::new(1, 3, AddressingMode::ZeroPage, CPU::lda));
        cpu.instruction_set[0xB5] = Some(Instruction::new(1, 4, AddressingMode::ZeroPageX, CPU::lda));
        cpu.instruction_set[0xAD] = Some(Instruction::new(2, 4, AddressingMode::Absolute, CPU::lda));
        cpu.instruction_set[0xBD] = Some(Instruction::new(2, 4, AddressingMode::AbsoluteX, CPU::lda));
        cpu.instruction_set[0xB9] = Some(Instruction::new(2, 4, AddressingMode::AbsoluteY, CPU::lda));
        cpu.instruction_set[0xA1] = Some(Instruction::new(1, 6, AddressingMode::IndirectX, CPU::lda));
        cpu.instruction_set[0xB1] = Some(Instruction::new(1, 5, AddressingMode::IndirectY, CPU::lda));
        //ldx
        cpu.instruction_set[0xA2] = Some(Instruction::new(1, 2, AddressingMode::Immediate, CPU::ldx));
        cpu.instruction_set[0xA6] = Some(Instruction::new(1, 3, AddressingMode::ZeroPage, CPU::ldx));
        cpu.instruction_set[0xB6] = Some(Instruction::new(1, 4, AddressingMode::ZeroPageY, CPU::ldx));
        cpu.instruction_set[0xAE] = Some(Instruction::new(2, 4, AddressingMode::Absolute, CPU::ldx));
        cpu.instruction_set[0xBE] = Some(Instruction::new(2, 4, AddressingMode::AbsoluteY, CPU::ldx));
        //ldy
        cpu.instruction_set[0xA0] = Some(Instruction::new(1, 2, AddressingMode::Immediate, CPU::ldy));
        cpu.instruction_set[0xA4] = Some(Instruction::new(1, 3, AddressingMode::ZeroPage, CPU::ldy));
        cpu.instruction_set[0xB4] = Some(Instruction::new(1, 4, AddressingMode::ZeroPageY, CPU::ldy));
        cpu.instruction_set[0xAC] = Some(Instruction::new(2, 4, AddressingMode::Absolute, CPU::ldy));
        cpu.instruction_set[0xBC] = Some(Instruction::new(2, 4, AddressingMode::AbsoluteY, CPU::ldy));
        //lsr
        cpu.instruction_set[0x4A] = Some(Instruction::new(0, 2, AddressingMode::Implied, CPU::lsr_accumulator));
        cpu.instruction_set[0x46] = Some(Instruction::new(1, 5, AddressingMode::ZeroPage, CPU::lsr));
        cpu.instruction_set[0x56] = Some(Instruction::new(1, 6, AddressingMode::ZeroPageX, CPU::lsr));
        cpu.instruction_set[0x4E] = Some(Instruction::new(2, 6, AddressingMode::Absolute, CPU::lsr));
        cpu.instruction_set[0x5E] = Some(Instruction::new(2, 7, AddressingMode::AbsoluteX, CPU::lsr));
        //nop
        cpu.instruction_set[0xEA] = Some(Instruction::new(0, 2, AddressingMode::Implied, CPU::nop));
        //ora
        cpu.instruction_set[0x09] = Some(Instruction::new(1, 2, AddressingMode::Immediate, CPU::ora));
        cpu.instruction_set[0x05] = Some(Instruction::new(1, 2, AddressingMode::ZeroPage, CPU::ora));
        cpu.instruction_set[0x15] = Some(Instruction::new(1, 2, AddressingMode::ZeroPageX, CPU::ora));
        cpu.instruction_set[0x0D] = Some(Instruction::new(3, 3, AddressingMode::Absolute, CPU::ora));
        cpu.instruction_set[0x1D] = Some(Instruction::new(3, 3, AddressingMode::AbsoluteX, CPU::ora));
        cpu.instruction_set[0x19] = Some(Instruction::new(3, 3, AddressingMode::AbsoluteY, CPU::ora));
        cpu.instruction_set[0x01] = Some(Instruction::new(1, 2, AddressingMode::IndirectX, CPU::ora));
        cpu.instruction_set[0x11] = Some(Instruction::new(1, 2, AddressingMode::IndirectY, CPU::ora));
        //pha
        cpu.instruction_set[0x48] = Some(Instruction::new(1, 3, AddressingMode::Implied, CPU::pha));
        //php
        cpu.instruction_set[0x08] = Some(Instruction::new(1, 3, AddressingMode::Implied, CPU::php));
        //pla
        cpu.instruction_set[0x68] = Some(Instruction::new(1, 4, AddressingMode::Implied, CPU::pla));
        //plp
        cpu.instruction_set[0x28] = Some(Instruction::new(1, 4, AddressingMode::Implied, CPU::plp));
        //rol
        cpu.instruction_set[0x2A] = Some(Instruction::new(0, 2, AddressingMode::Implied, CPU::rol_accumulator));
        cpu.instruction_set[0x26] = Some(Instruction::new(1, 5, AddressingMode::ZeroPage, CPU::rol));
        cpu.instruction_set[0x36] = Some(Instruction::new(1, 6, AddressingMode::ZeroPageX, CPU::rol));
        cpu.instruction_set[0x2E] = Some(Instruction::new(2, 6, AddressingMode::Absolute, CPU::rol));
        cpu.instruction_set[0x3E] = Some(Instruction::new(2, 7, AddressingMode::AbsoluteX, CPU::rol));
        //ror
        cpu.instruction_set[0x6A] = Some(Instruction::new(0, 2, AddressingMode::Implied, CPU::ror_accumulator));
        cpu.instruction_set[0x66] = Some(Instruction::new(1, 5, AddressingMode::ZeroPage, CPU::ror));
        cpu.instruction_set[0x76] = Some(Instruction::new(1, 6, AddressingMode::ZeroPageX, CPU::ror));
        cpu.instruction_set[0x6E] = Some(Instruction::new(2, 6, AddressingMode::Absolute, CPU::ror));
        cpu.instruction_set[0x7E] = Some(Instruction::new(2, 7, AddressingMode::AbsoluteX, CPU::ror));
        //rti
        cpu.instruction_set[0x40] = Some(Instruction::new(1, 6, AddressingMode::Implied, CPU::rti));
        //rts
        cpu.instruction_set[0x60] = Some(Instruction::new(1, 6, AddressingMode::Implied, CPU::rts));
        //sbc
        cpu.instruction_set[0xE9] = Some(Instruction::new(1, 2, AddressingMode::Immediate, CPU::sbc));
        cpu.instruction_set[0xE5] = Some(Instruction::new(1, 2, AddressingMode::ZeroPage, CPU::sbc));
        cpu.instruction_set[0xF5] = Some(Instruction::new(1, 2, AddressingMode::ZeroPageX, CPU::sbc));
        cpu.instruction_set[0xED] = Some(Instruction::new(2, 2, AddressingMode::Absolute, CPU::sbc));
        cpu.instruction_set[0xFD] = Some(Instruction::new(2, 2, AddressingMode::AbsoluteX, CPU::sbc));
        cpu.instruction_set[0xF9] = Some(Instruction::new(2, 2, AddressingMode::AbsoluteY, CPU::sbc));
        cpu.instruction_set[0xE1] = Some(Instruction::new(1, 2, AddressingMode::IndirectX, CPU::sbc));
        cpu.instruction_set[0xF1] = Some(Instruction::new(1, 2, AddressingMode::IndirectY, CPU::sbc));
        //sec
        cpu.instruction_set[0x38] = Some(Instruction::new(0, 2, AddressingMode::Implied, CPU::sec));
        //sed
        cpu.instruction_set[0xF8] = Some(Instruction::new(0, 2, AddressingMode::Implied, CPU::sed));
        //sei
        cpu.instruction_set[0xF8] = Some(Instruction::new(0, 2, AddressingMode::Implied, CPU::sei));
        //sta
        cpu.instruction_set[0x85] = Some(Instruction::new(1, 3, AddressingMode::ZeroPage, CPU::sta));
        cpu.instruction_set[0x95] = Some(Instruction::new(1, 4, AddressingMode::ZeroPageX, CPU::sta));
        cpu.instruction_set[0x8D] = Some(Instruction::new(2, 4, AddressingMode::Absolute, CPU::sta));
        cpu.instruction_set[0x9D] = Some(Instruction::new(2, 5, AddressingMode::AbsoluteX, CPU::sta));
        cpu.instruction_set[0x99] = Some(Instruction::new(2, 5, AddressingMode::AbsoluteY, CPU::sta));
        cpu.instruction_set[0x81] = Some(Instruction::new(1, 6, AddressingMode::IndirectX, CPU::sta));
        cpu.instruction_set[0x91] = Some(Instruction::new(1, 6, AddressingMode::IndirectY, CPU::sta));
        //stx
        cpu.instruction_set[0x86] = Some(Instruction::new(1, 3, AddressingMode::ZeroPage, CPU::stx));
        cpu.instruction_set[0x96] = Some(Instruction::new(1, 4, AddressingMode::ZeroPageY, CPU::stx));
        cpu.instruction_set[0x8E] = Some(Instruction::new(2, 4, AddressingMode::Absolute, CPU::stx));
        //sty
        cpu.instruction_set[0x84] = Some(Instruction::new(1, 3, AddressingMode::ZeroPage, CPU::sty));
        cpu.instruction_set[0x94] = Some(Instruction::new(1, 4, AddressingMode::ZeroPageX, CPU::sty));
        cpu.instruction_set[0x8C] = Some(Instruction::new(2, 4, AddressingMode::Absolute, CPU::sty));
        //tax
        cpu.instruction_set[0xAA] = Some(Instruction::new(0, 2, AddressingMode::Implied, CPU::tax));
        //tay
        cpu.instruction_set[0xA8] = Some(Instruction::new(0, 2, AddressingMode::Implied, CPU::tay));
        //tsx
        cpu.instruction_set[0xBA] = Some(Instruction::new(0, 2, AddressingMode::Implied, CPU::tsx));
        //txa
        cpu.instruction_set[0x8A] = Some(Instruction::new(0, 2, AddressingMode::Implied, CPU::txa));
        //txs
        cpu.instruction_set[0x9A] = Some(Instruction::new(0, 2, AddressingMode::Implied, CPU::txs));
        //tya
        cpu.instruction_set[0x98] = Some(Instruction::new(0, 2, AddressingMode::Implied, CPU::tya));

        cpu
    }

    pub fn reset(&mut self){
        self.register_x = 0;
        self.register_a = 0;
        self.status = 0;
        self.instruction_ptr = self.mem_read_u16(0xFFFC)
    }

    pub fn run<F>(&mut self, mut callback: F)
    where
        F: FnMut(&mut CPU),
    {

        loop {
            callback(self);
            let instruction : Option<Instruction> = self.instruction_set[self.mem_read(self.instruction_ptr) as usize];
            self.instruction_ptr += 1;
            match instruction {
                Some(inst) => {
                    (inst.function)(self, &inst.mode);
                    self.instruction_ptr += inst.extra_bytes as u16;
                },
                None => {
                    println!("Execution finished with code {:x}", self.mem_read(self.instruction_ptr));
                    return;
                },
            }
        }
    }

    fn stack_pop(&mut self) -> u8{
        if self.stack_ptr == 0xFF{
            self.stack_ptr = 0;
        } else {
            self.stack_ptr += 1;
        }
        self.mem_read((STACK as u16) + (self.stack_ptr as u16))
    }

    fn stack_pop_u16(&mut self) -> u16{
        self.stack_pop() as u16 | ((self.stack_pop() as u16) << 8)
    }

    fn stack_push(&mut self, data : u8){
        if self.stack_ptr == 0 {
            self.stack_ptr = 0xFF;
        } else {
            self.stack_ptr -= 1;
        }
        self.mem_write((STACK as u16) + self.stack_ptr as u16, data);
    }

    fn stack_push_u16(&mut self, data : u16){
        self.stack_push((data >> 8) as u8);
        self.stack_push((data & 0xFF) as u8)
    }

    fn update_zero_and_neg_flags(&mut self, flag : u8){
        if flag == 0 {
            self.status |= ZERO;
        } else {
            self.status &= !ZERO 
        };

        if flag & NEGATIVE != 0 {
            self.status |= NEGATIVE 
        } else {
            self.status &= !NEGATIVE 
        };
    }

    fn get_operand_addresss(&mut self, mode: &AddressingMode) -> u16 {
        match mode{
            AddressingMode::Absolute => self
                .mem_read_u16(self.instruction_ptr),
            AddressingMode::AbsoluteX => self
                .mem_read_u16(self.instruction_ptr)
                .wrapping_add(self.register_x as u16),
            AddressingMode::AbsoluteY => self
                .mem_read_u16(self.instruction_ptr)
                .wrapping_add(self.register_y as u16),
            AddressingMode::Immediate => self
                .instruction_ptr,
            AddressingMode::ZeroPage => self
                .mem_read(self.instruction_ptr) as u16,
            AddressingMode::ZeroPageX => self
                .mem_read(self.instruction_ptr)
                .wrapping_add(self.register_x) as u16,
            AddressingMode::ZeroPageY => self
                .mem_read(self.instruction_ptr)
                .wrapping_add(self.register_y) as u16,
            AddressingMode::IndirectX => {
                let ptr = self
                    .mem_read(self.instruction_ptr)
                    .wrapping_add(self.register_x);
                (self.mem_read(ptr as u16) as u16) | ((self.mem_read(ptr.wrapping_add(1) as u16) as u16) << 8)
            },
            AddressingMode::IndirectY => {
                let ptr = self
                    .mem_read(self.instruction_ptr);
                (self.mem_read(ptr as u16) as u16) | ((self.mem_read(ptr.wrapping_add(1) as u16) as u16) << 8).
                    wrapping_add(self.register_y as u16)
},
            AddressingMode::Implied => panic!("Attmepted to get operand address for AdressingMode::Implied"),
        }
    }

    fn accumulate(&mut self, data : u8) {
        let sum : u16 = (self.register_a as u16) + (data as u16) + if self.status & CARRY != 0 {
            1
        } else {
            0
        };

        if sum > 0xFF {
            self.status |= CARRY;
        } else {
            self.status &= !CARRY;
        }
        
        let sum : u8 = sum as u8;

        if (data ^ sum) & (sum ^ self.register_a) & 0x80 != 0 {
            self.status |= OVERFLOW;
        } else {
            self.status &= !OVERFLOW;
        }
        self.update_zero_and_neg_flags(self.register_a);
    }
    
    fn adc(&mut self, mode : &AddressingMode) {
        let address : u16 = self.get_operand_addresss(mode);
        let data : u8 = self.mem_read(address);
        self.accumulate(data);
    }

    fn and(&mut self, mode : &AddressingMode) {
        let address : u16 = self.get_operand_addresss(mode);
        self.register_a = self.mem_read(address) & self.register_a;
        self.update_zero_and_neg_flags(self.register_a);
    }

    fn asl(&mut self, mode : &AddressingMode) {
        let address = self.get_operand_addresss(mode);
        let mut data = self.mem_read(address);
        if data >> 7 == 1 {
            self.status |= CARRY;
        } else {
            self.status &= !CARRY;
        }
        data = data << 1;
        self.mem_write(address, data);
        self.update_zero_and_neg_flags(data);
    }
    
    fn asl_accumulated(&mut self, mode : &AddressingMode) {
        if self.register_a >> 7 == 1 {
            self.status |= CARRY;
        } else {
            self.status &= !CARRY;
        }
        self.register_a <<= 1;
    }

    fn branch(&mut self){
        let result = self
            .instruction_ptr
            .wrapping_add(1)
            .wrapping_add(self.mem_read(self.instruction_ptr) as u16);
        self.instruction_ptr = result; //TODO: Figure out if cycles need to be incremented here
    }

    fn bcc(&mut self, mode : &AddressingMode) {
        if self.status & CARRY == 0{
            self.branch();
        }
    }

    fn bcs(&mut self, mode : &AddressingMode) {
        if self.status & CARRY == CARRY {
            self.branch();
        }
    }

    fn beq(&mut self, mode : &AddressingMode){
        if self.status & ZERO == ZERO {
            self.branch();
        }
    }

    fn bit(&mut self, mode : &AddressingMode){
        let address : u16 = self.get_operand_addresss(mode);
        if self.register_a & self.mem_read(address) == 0{
           self.status |= ZERO; 
        } else {
            self.status &= !ZERO;
        }
    }

    fn bmi(&mut self, mode : &AddressingMode){
        if self.status & NEGATIVE == NEGATIVE {
            self.branch();
        }
    }

    fn bne(&mut self, mode : &AddressingMode){
        if self.status & ZERO == 0 {
            self.branch();
        }
    }

    fn bpl(&mut self, mode : &AddressingMode){
        if self.status & NEGATIVE == 0 {
            self.branch();
        }
    }
    
    fn bvc(&mut self, mode : &AddressingMode){
        if self.status & OVERFLOW == 0 {
            self.branch();
        }
    }

    fn bvs(&mut self, mode : &AddressingMode){
        if self.status & OVERFLOW == OVERFLOW {
            self.branch();
        }
    }

    fn clc(&mut self, mode : &AddressingMode){
        self.status &= !CARRY;
    }

    fn cld(&mut self, mode : &AddressingMode){
        self.status &= !DECIMAL_MODE;
    }

    fn cli(&mut self, mode : &AddressingMode){
        self.status &= !INTERRUPT_DISABLE;
    }

    fn clv(&mut self, mode : &AddressingMode){
        self.status &= !OVERFLOW;
    }

    fn cmp(&mut self, mode : &AddressingMode){
        let address : u16 = self.get_operand_addresss(mode);
        let data = self.mem_read(address);
        if data <= self.register_a {
            self.status |= CARRY;
        } else {
            self.status &= !CARRY;
        }

        self.update_zero_and_neg_flags(self.register_a.wrapping_sub(data));
    }
    
    fn cpx(&mut self, mode : &AddressingMode){
        let address : u16 = self.get_operand_addresss(mode);
        let data = self.mem_read(address);
        if data <= self.register_x {
            self.status |= CARRY;
        } else {
            self.status &= !CARRY;
        }

        self.update_zero_and_neg_flags(self.register_x.wrapping_sub(data))
    }

    fn cpy(&mut self, mode : &AddressingMode){
        let address : u16 = self.get_operand_addresss(mode);
        let data = self.mem_read(address);
        if data <= self.register_y {
            self.status |= CARRY;
        } else {
            self.status &= !CARRY;
        }

        self.update_zero_and_neg_flags(self.register_y.wrapping_sub(data));
    }
    
    fn dec(&mut self, mode : &AddressingMode){
        let address : u16 = self.get_operand_addresss(mode);
        let data : u8 = self.mem_read(address).wrapping_sub(1);
        self.mem_write(address, data);
        self.update_zero_and_neg_flags(data);
    }
    
    fn dex(&mut self, mode : &AddressingMode){
        self.register_x = self.register_x.wrapping_sub(1);
        self.update_zero_and_neg_flags(self.register_x);
    }

    fn dey(&mut self, mode : &AddressingMode){
        self.register_y = self.register_y.wrapping_sub(1);
        self.update_zero_and_neg_flags(self.register_y);
    }

    fn eor(&mut self, mode : &AddressingMode){
        let address : u16 = self.get_operand_addresss(mode);
        self.register_a ^= self.mem_read(address);
        self.update_zero_and_neg_flags(self.register_a);
    }

    fn inc(&mut self, mode : &AddressingMode){
        let address : u16 = self.get_operand_addresss(mode);
        let data : u8 = self.mem_read(address).wrapping_add(1);
        self.mem_write(address, data);
        self.update_zero_and_neg_flags(data);
    }
    
    fn inx(&mut self, mode: &AddressingMode){
        if self.register_x == 0xFF {
            self.register_x = 0x00
        } else {
            self.register_x += 1;
        };
        self.update_zero_and_neg_flags(self.register_x)
    }

    fn iny(&mut self, mode: &AddressingMode){
        if self.register_y == 0xFF {
            self.register_y = 0x00
        } else {
            self.register_y += 1;
        };
        self.update_zero_and_neg_flags(self.register_y)
    }

    fn jmp_absolute(&mut self, mode : &AddressingMode){
        let address : u16 = self.mem_read_u16(self.instruction_ptr);
        self.instruction_ptr = self.mem_read_u16(address);
    }

    fn jmp_indirect(&mut self, mode : &AddressingMode){
        let address : u16 = self.mem_read_u16(self.instruction_ptr);
        if address & 0x00FF == 0x00FF {
            self.instruction_ptr = (self.mem_read(address) as u16) | (self.mem_read(address & 0xFF00) as u16);
        } else {
            self.instruction_ptr = self.mem_read_u16(address);
        }
    }

    fn jsr(&mut self, mode: &AddressingMode){
        self.stack_push_u16(self.instruction_ptr + 2 - 1);
        self.instruction_ptr = self.mem_read_u16(self.instruction_ptr);
    }

    fn lda(&mut self, mode: &AddressingMode){
        let address : u16 = self.get_operand_addresss(mode);
        self.register_a = self.mem_read(address);
        self.update_zero_and_neg_flags(self.register_a);
    }

    fn ldx(&mut self, mode: &AddressingMode){
        let address : u16 = self.get_operand_addresss(mode);
        self.register_x = self.mem_read(address);
        self.update_zero_and_neg_flags(self.register_x);
    }

    fn ldy(&mut self, mode: &AddressingMode){
        let address : u16 = self.get_operand_addresss(mode);
        self.register_y = self.mem_read(address);
        self.update_zero_and_neg_flags(self.register_y);
    }

    fn lsr(&mut self, mode : &AddressingMode){
        let address : u16 = self.get_operand_addresss(mode);
        let mut data : u8 = self.mem_read(address);
        if data & 1 == 1 {
            self.status |= CARRY;
        } else {
            self.status &= !CARRY;
        }
        data >>= 1;
        self.mem_write(address, data);
        self.update_zero_and_neg_flags(data);
    }

    fn lsr_accumulator(&mut self, mode : &AddressingMode){
        if self.register_a & 1 == 1 {
            self.status |= CARRY;
        } else {
            self.status &= !CARRY;
        }
        self.register_a >>= 1;
        self.update_zero_and_neg_flags(self.register_a);
    }

    fn nop(&mut self, mode : &AddressingMode){}

    fn ora(&mut self, mode: &AddressingMode){
        let address : u16 = self.get_operand_addresss(mode);
        self.register_a |= self.mem_read(address);
        self.update_zero_and_neg_flags(self.register_a);
    }

    fn pha(&mut self, mode: &AddressingMode){
        self.stack_push(self.register_a);
    }
    
    fn php(&mut self, mode: &AddressingMode){
        self.stack_push(self.status);
        self.status |= BREAK;
        self.status |= BREAK2
    }
    
    fn pla(&mut self, mode: &AddressingMode){
        self.register_a = self.stack_pop();
        self.update_zero_and_neg_flags(self.register_a);
    }

    fn plp(&mut self, mode: &AddressingMode){
        self.status = self.stack_pop();
        self.status &= !BREAK;
        self.status |= BREAK2
    }

    fn rol(&mut self, mode: &AddressingMode){
        let address : u16 = self.get_operand_addresss(mode);
        let mut data : u8 = self.mem_read(address);
        let carry : bool = self.status & CARRY == CARRY;
        if data >> 7 == 1 {
            self.status |= CARRY;
        } else {
            self.status &= !CARRY;
        }
        data <<= 1;
        if carry {
            data |= 1;
        }
        self.mem_write(address, data);
        self.update_zero_and_neg_flags(data);
    }

    fn rol_accumulator(&mut self, mode: &AddressingMode){
        let carry : bool = self.status & CARRY == CARRY;
        if self.register_a >> 7 == 1 {
            self.status |= CARRY;
        } else {
            self.status &= !CARRY;
        }
        self.register_a <<= 1;
        if carry {
            self.register_a |= 1;
        }
        self.update_zero_and_neg_flags(self.register_a);
    }

    fn ror(&mut self, mode: &AddressingMode){
        let address : u16 = self.get_operand_addresss(mode);
        let mut data : u8 = self.mem_read(address);
        let carry : bool = self.status & CARRY == CARRY;
        if data & 1 == 1 {
            self.status |= CARRY;
        } else {
            self.status &= !CARRY;
        }
        data >>= 1;
        if carry {
            data |= 0b1000_0000;
        }
        self.mem_write(address, data);
        self.update_zero_and_neg_flags(data);
    }

    fn ror_accumulator(&mut self, mode: &AddressingMode){
        let carry : bool = self.status & CARRY == CARRY;
        if self.register_a & 1 == 1 {
            self.status |= CARRY;
        } else {
            self.status &= !CARRY;
        }
        self.register_a >>= 1;
        if carry {
            self.register_a |= 0b1000_0000;
        }
        self.update_zero_and_neg_flags(self.register_a);
    }

    fn rti(&mut self, mode: &AddressingMode){
        self.status = self.stack_pop();
        self.status &= !BREAK;
        self.status |= BREAK2;
        self.instruction_ptr = self.stack_pop_u16();
    }

    fn rts(&mut self, mode: &AddressingMode){
        self.instruction_ptr = self.stack_pop_u16() + 1;
    }

    fn sbc(&mut self, mode: &AddressingMode){
        let address : u16 = self.get_operand_addresss(mode);
        let data : i8 = (self.mem_read(address) as i8).wrapping_neg().wrapping_sub(1);
        self.accumulate(data as u8);
    }

    fn sec(&mut self, mode: &AddressingMode){
        self.status |= CARRY;
    }

    fn sed(&mut self, mode: &AddressingMode){
        self.status |= DECIMAL_MODE;
    }
    
    fn sei(&mut self, mode: &AddressingMode){
        self.status |= INTERRUPT_DISABLE;
    }

    fn sta(&mut self, mode: &AddressingMode){
        let address : u16 = self.get_operand_addresss(mode);
        self.mem_write(address, self.register_a);
    }

    fn stx(&mut self, mode: &AddressingMode){
        let address : u16 = self.get_operand_addresss(mode);
        self.mem_write(address, self.register_x);
    }

    fn sty(&mut self, mode: &AddressingMode){
        let address : u16 = self.get_operand_addresss(mode);
        self.mem_write(address, self.register_x);
    }

    fn tax(&mut self, mode: &AddressingMode){
        self.register_x = self.register_a;
        self.update_zero_and_neg_flags(self.register_a);
    }

    fn tay(&mut self, mode: &AddressingMode){
        self.register_y = self.register_a;
        self.update_zero_and_neg_flags(self.register_a);
    }

    fn tsx(&mut self, mode: &AddressingMode){
        self.register_x = self.stack_pop(); 
        self.update_zero_and_neg_flags(self.register_x);
    }
    
    fn txa(&mut self, mode: &AddressingMode){
        self.register_a = self.register_x; 
        self.update_zero_and_neg_flags(self.register_a);
    }
    
    fn txs(&mut self, mode: &AddressingMode){
        self.stack_push(self.register_x);
    }

    fn tya(&mut self, mode: &AddressingMode){
        self.register_a = self.register_y;
        self.update_zero_and_neg_flags(self.register_y);
    }
}

use crate::opcodes;
use crate::bus::Bus;
use std::{collections::HashMap, num::NonZero};
use crate::opcodes::OpCode;


pub struct CPU {
    pub register_a: u8,
    pub status: u8,
    pub program_counter: u16,
    pub register_x: u8,
    pub register_y: u8,
    pub stack_pointer: u8,
    pub bus: Bus,
}

#[derive(Debug, PartialEq)]
#[allow(non_camel_case_types)]
pub enum AddressingMode {
    Accumulator,
    Immediate,
    ZeroPage,
    ZeroPage_X,
    ZeroPage_Y,
    Absolute,
    Absolute_X,
    Absolute_Y,
    Indirect,
    Indirect_X,
    Indirect_Y,
    NoneAddressing,
    Implied,
    Relative,
}

const FLAG_CARRY: u8 = 1 << 0;
const FLAG_ZERO: u8 = 1 << 1;
const FLAG_INTERRUPT: u8 = 1 << 2;
const FLAG_DECIMAL: u8 = 1 << 3;
const FLAG_BREAK: u8 = 1 << 4;
const FLAG_BREAK2: u8 = 1 << 5;
const FLAG_OVERFLOW: u8 = 1 << 6;
const FLAG_NEGATIVE: u8 = 1 << 7;

const SIGN_BIT: u8 = 1 << 7;

impl CPU {
    pub fn new() -> Self{
        CPU {
            register_a: 0,
            status: FLAG_INTERRUPT | FLAG_BREAK2,
            program_counter: 0,
            register_x: 0,
            register_y: 0,
            stack_pointer: 0xFF,
            bus: Bus::new(),
            }
    }

    impl Mem for CPU {
        fn mem_read(&self, addr: u16) -> u8{
            self.bus.mem_read(addr)
        }

        fn mem_write(&mut self, addr: u16, data: u8){
            self.bus.mem_write(addr, data)
        }

        fn mem_read_u16(&self, pos: u16) -> u16{
            self.bus.mem_read_u16(pos)
        }

        fn mem_write_u16(&self, pos: u16, data: u16){
            self.bus.mem_write_u16(pos, data)
        }
    }


    pub fn mem_read(&self, addr: u16) -> u8 {
        self.memory[addr as usize]
    }

    pub fn mem_write(&mut self, addr: u16, data: u8){
        self.memory[addr as usize] = data;
    }

    pub fn mem_read_u16(&self, pos: u16) -> u16{
        let lo = self.mem_read(pos) as u16;
        let hi = self.mem_read(pos + 1) as u16;
        (hi << 8) | (lo as u16)
    }

    pub fn mem_write_u16(&mut self, pos: u16, data: u16){
        let hi = (data >> 8) as u8;
        let lo = (data & 0xff) as u8;
        self.mem_write(pos, lo);
        self.mem_write(pos + 1, hi);
    }

    pub fn reset(&mut self){
        self.register_a = 0;
        self.register_x = 0;
        self.register_y = 0;
        self.status = FLAG_INTERRUPT | FLAG_BREAK2;
        self.stack_pointer =  0xFD;
        //ToDo memory reset
        self.program_counter = self.mem_read_u16(0xFFFC);
        //self.mem_write_u16(0xFFFE, 0x0600);
    }

    pub fn load_and_run(&mut self, program: Vec<u8>){
        self.load(program);
        self.reset();
        self.run();
    }

    pub fn load(&mut self, game_code: Vec<u8>){
        self.memory[0x0600 .. (0x0600 + game_code.len())].copy_from_slice(&game_code[..]);
        self.mem_write_u16(0xFFFC, 0x0600);
    }


    fn get_operand_address(&self, mode: &AddressingMode) -> u16 {

        match mode {
            AddressingMode :: Implied => {
                panic!("AddressingMode :: Implied");
            }

            AddressingMode :: Accumulator => {
                panic!("AddressingMode :: Accumulator");
            }

            AddressingMode :: Relative => {
                let base = self.mem_read(self.program_counter);
                let np = (base as i8) as i32 + self.program_counter as i32;
                return np as u16;
            }

            AddressingMode :: Immediate => self.program_counter,
            
            AddressingMode :: ZeroPage => self.mem_read(self.program_counter) as u16,
            
            AddressingMode :: Absolute => self.mem_read_u16(self.program_counter),
            
            AddressingMode :: ZeroPage_X => {
                let pos = self.mem_read(self.program_counter);
                let addr = pos.wrapping_add(self.register_x) as u16;
                addr
            }

            AddressingMode :: ZeroPage_Y => {
                let pos = self.mem_read(self.program_counter);
                let addr = pos.wrapping_add(self.register_y) as u16;
                addr
            }

            AddressingMode :: Absolute_X => {
                let base = self.mem_read_u16(self.program_counter);
                let addr = base.wrapping_add(self.register_x as u16);
                addr
            }

            AddressingMode :: Absolute_Y => {
                let base = self.mem_read_u16(self.program_counter);
                let addr = base.wrapping_add(self.register_y as u16);
                addr
            }

            AddressingMode :: Indirect => {
                let base = self.mem_read_u16(self.program_counter);
                let addr = self.mem_read_u16(base as u16);
                addr
            }

            AddressingMode :: Indirect_X => {
                let base = self.mem_read(self.program_counter);
                let ptr: u8 = (base as u8).wrapping_add(self.register_x);
                let addr = self.mem_read_u16(ptr as u16);
                addr
            }
            
            AddressingMode :: Indirect_Y => {
                let base = self.mem_read(self.program_counter);
                let deref_base = self.mem_read_u16(base as u16);
                let deref = deref_base.wrapping_add(self.register_y as u16);
                deref
            }

            AddressingMode :: NoneAddressing => {
                panic!("mode {:?} is not supported", mode);
            }
            
        }
    }

    pub fn run(&mut self){
        self.run_with_callback(|_| {});
    }

    pub fn run_with_callback<F>(&mut self, mut callback:F)
    where F: FnMut(&mut CPU),
    {
        let ref opcodes: HashMap<u8, &'static opcodes::OpCode> = *opcodes::OPCODES_MAP;
        loop{
            
            let code = self.mem_read(self.program_counter);
            self.program_counter += 1;
            println!("code: {:04x}, PC: {:04X}, X: {:02X}",code, self.program_counter, self.register_x);
            let opcode = opcodes.get(&code).unwrap();
            callback(self);
            match code {
                0xa9 | 0xa5 | 0xb5 | 0xa1 | 0xb1 => {
                    self.lda(&opcode.mode);
                    self.program_counter += &opcode.len-1;
                }

                0xad | 0xbd | 0xb9 => {
                    self.lda(&opcode.mode);
                    self.program_counter += &opcode.len-1;
                }
                0xAA => {
                    self.tax(&opcode.mode);
                    self.program_counter += &opcode.len-1;
                }
                0xe8 => {
                    self.inx(&opcode.mode);
                    self.program_counter += &opcode.len-1;
                }

                0x00 => {
                    self.brk(&opcode.mode);
                    self.program_counter += &opcode.len-1;
                },

                /* CLD */ 0xd8 => {
                    self.cld(&opcode.mode);
                    self.program_counter += &opcode.len-1;
                }

                /* CLI */ 0x58 => {
                    self.cli(&opcode.mode);
                    self.program_counter += &opcode.len-1;
                }

                /* CLV */ 0xb8 => {
                    self.clv(&opcode.mode);
                    self.program_counter += &opcode.len-1;
                }

                /* CLC */ 0x18 => {
                    self.clc(&opcode.mode);
                    self.program_counter += &opcode.len-1;
                }

                /* SEC */ 0x38 => {
                    self.sec(&opcode.mode);
                    self.program_counter += &opcode.len-1;
                }

                /* SEI */ 0x78 => {
                    self.sei(&opcode.mode);
                    self.program_counter += &opcode.len-1;
                }
                
                /* SED */ 0xf8 => {
                    self.sed(&opcode.mode);
                    self.program_counter += &opcode.len-1;
                }

                /* PHA */ 0x48 => {
                    self.pha(&opcode.mode);
                    self.program_counter += &opcode.len-1;
                }

                /* PLA */
                0x68 => {
                    self.pla(&opcode.mode);
                    self.program_counter += &opcode.len-1;
                }

                /* PHP */
                0x08 => {
                    self.php(&opcode.mode);
                    self.program_counter += &opcode.len-1;
                }

                /* PLP */
                0x28 => {
                    self.plp(&opcode.mode);
                    self.program_counter += &opcode.len-1;
                }

                /* ADC */
                0x69 | 0x65 | 0x75 | 0x6d | 0x7d | 0x79 | 0x61 | 0x71 => {
                    self.adc(&opcode.mode);
                    self.program_counter += &opcode.len-1;
                }

                /* SBC */
                0xe9 | 0xe5 | 0xf5 | 0xed | 0xfd | 0xf9 | 0xe1 | 0xf1 => {
                    self.sbc(&opcode.mode);
                    self.program_counter += &opcode.len-1;
                }

                /* AND */
                0x29 | 0x25 | 0x35 | 0x2d | 0x3d | 0x39 | 0x21 | 0x31 => {
                    self.and(&opcode.mode);
                    self.program_counter += &opcode.len-1;
                }

                /* EOR */
                0x49 | 0x45 | 0x55 | 0x4d | 0x5d | 0x59 | 0x41 | 0x51 => {
                    self.eor(&opcode.mode);
                    self.program_counter += &opcode.len-1;
                }

                /* ORA */
                0x09 | 0x05 | 0x15 | 0x0d | 0x1d | 0x19 | 0x01 | 0x11 => {
                    self.ora(&opcode.mode);
                    self.program_counter += &opcode.len-1;
                }

                 /* LSR */ 0x4a => {
                    self.lsr(&opcode.mode);
                    self.program_counter += &opcode.len-1;
                }

                /* LSR */
                0x46 | 0x56 | 0x4e | 0x5e => {
                    self.lsr(&opcode.mode);
                    self.program_counter += &opcode.len-1;
                }

                /*ASL*/ 0x0a => {
                    self.asl(&opcode.mode);
                    self.program_counter += &opcode.len-1;
                }

                /* ASL */
                0x06 | 0x16 | 0x0e | 0x1e => {
                    self.asl(&opcode.mode);
                    self.program_counter += &opcode.len-1;
                }

                /*ROL*/ 0x2a => {
                    self.rol(&opcode.mode);
                    self.program_counter += &opcode.len-1;
                }

                /* ROL */
                0x26 | 0x36 | 0x2e | 0x3e => {
                    self.rol(&opcode.mode);
                    self.program_counter += &opcode.len-1;
                }

                /* ROR */ 0x6a => {
                    self.ror(&opcode.mode);
                    self.program_counter += &opcode.len-1;
                }

                /* ROR */
                0x66 | 0x76 | 0x6e | 0x7e => {
                    self.ror(&opcode.mode);
                    self.program_counter += &opcode.len-1;
                }

                /* INC */
                0xe6 | 0xf6 | 0xee | 0xfe => {
                    self.inc(&opcode.mode);
                    self.program_counter += &opcode.len-1;
                }

                /* INY */
                0xc8 => {
                    self.iny(&opcode.mode);
                    self.program_counter += &opcode.len-1;
                }

                /* DEC */
                0xc6 | 0xd6 | 0xce | 0xde => {
                    self.dec(&opcode.mode);
                    self.program_counter += &opcode.len-1;
                }

                /* DEX */
                0xca => {
                    self.dex(&opcode.mode);
                    self.program_counter += &opcode.len-1;
                }

                /* DEY */
                0x88 => {
                    self.dey(&opcode.mode);
                    self.program_counter += &opcode.len-1;
                }

                /* CMP */
                0xc9 | 0xc5 | 0xd5 | 0xcd | 0xdd | 0xd9 | 0xc1 | 0xd1 => {
                    self.cmp(&opcode.mode);
                    self.program_counter += &opcode.len-1;
                }

                /* CPY */
                0xc0 | 0xc4 | 0xcc => {
                    self.cpy(&opcode.mode);
                    self.program_counter += &opcode.len-1;
                }

                /* CPX */
                0xe0 | 0xe4 | 0xec => {
                    self.cpx(&opcode.mode);
                    self.program_counter += &opcode.len-1;
                }

                /* JMP */
                0x4c | 0x6c => {
                    self.jmp(&opcode.mode);
                    self.program_counter += &opcode.len-1;
                }

                /* JSR */
                0x20 => {
                    self.jsr(&opcode.mode);
                    self.program_counter += &opcode.len-1;
                }
                /* RTS */
                0x60 => {
                    self.rts(&opcode.mode);
                    self.program_counter += &opcode.len-1;
                }

                /* RTI */
                0x40 => {
                    self.rti(&opcode.mode);
                    self.program_counter += &opcode.len-1;
                }

                /* BNE */
                0xd0 => {
                    self.bne(&opcode.mode);
                    self.program_counter += &opcode.len-1;
                }

                /* BVS */
                0x70 => {
                  self.bvs(&opcode.mode);
                  self.program_counter += &opcode.len-1;
                }

                /* BVC */
                0x50 => {
                  self.bvc(&opcode.mode);
                  self.program_counter += &opcode.len-1;
                }

                /* BPL */
                0x10 => {
                    self.bpl(&opcode.mode);
                    self.program_counter += &opcode.len-1;
                }

                /* BMI */
                0x30 => {
                    self.bmi(&opcode.mode);
                    self.program_counter += &opcode.len-1;
                }

                /* BEQ */
                0xf0 => {
                    self.beq(&opcode.mode);
                    self.program_counter += &opcode.len-1;
                }

                /* BCS */
                0xb0 => {
                    self.bcs(&opcode.mode);
                    self.program_counter += &opcode.len-1;
                }

                /* BCC */
                0x90 => {
                    self.bcc(&opcode.mode);
                    self.program_counter += &opcode.len-1;
                }

                /* BIT */
                0x24 | 0x2c => {
                    self.bit(&opcode.mode);
                    self.program_counter += &opcode.len-1;
                }

                /* STA */
                0x85 | 0x95 | 0x81 | 0x91 => {
                    self.sta(&opcode.mode);
                    self.program_counter += &opcode.len-1;
                }

                0x8d | 0x9d | 0x99 => {
                    self.sta(&opcode.mode);
                    self.program_counter += &opcode.len-1;
                }

                /* STX */
                0x86 | 0x96 | 0x8e => {
                    self.stx(&opcode.mode);
                    self.program_counter += &opcode.len-1;
                }

                /* STY */
                0x84 | 0x94 | 0x8c => {
                    self.sty(&opcode.mode);
                    self.program_counter += &opcode.len-1;
                }

                /* LDX */
                0xa2 | 0xa6 | 0xb6 | 0xae | 0xbe => {
                    self.ldx(&opcode.mode);
                    self.program_counter += &opcode.len-1;
                }

                /* LDY */
                0xa0 | 0xa4 | 0xb4 | 0xac | 0xbc => {
                    self.ldy(&opcode.mode);
                    self.program_counter += &opcode.len-1;
                }

                /* NOP */
                0xea => {
                    self.nop();
                    self.program_counter += &opcode.len-1;
                }

                /* TAY */
                0xa8 => {
                    self.tay(&opcode.mode);
                    self.program_counter += &opcode.len-1;
                }

                /* TSX */
                0xba => {
                    self.tsx(&opcode.mode);
                    self.program_counter += &opcode.len-1;
                }

                /* TXA */
                0x8a => {
                    self.txa(&opcode.mode);
                    self.program_counter += &opcode.len-1;
                }

                /* TXS */
                0x9a => {
                    self.txs(&opcode.mode);
                    self.program_counter += &opcode.len-1;
                }

                /* TYA */
                0x98 => {
                    self.tya(&opcode.mode);
                    self.program_counter += &opcode.len-1;
                }

                _ => todo!(),

            }
            
        }
    }

    fn bvc(&mut self, mode: &AddressingMode){
        self._branch(mode, FLAG_OVERFLOW, false);
    }

    fn bvs(&mut self, mode: &AddressingMode){
        self._branch(mode, FLAG_CARRY, true);
    }

    fn adc(&mut self, mode: &AddressingMode){
        let addr = self.get_operand_address(mode);
        let value = self.mem_read(addr);

        let carry = self.status & FLAG_CARRY;
        let (rhs, carry_flag1) = value.overflowing_add(carry);
        let (n, carry_flag2) = self.register_a.overflowing_add(rhs);
        
        let overflow = (self.register_a & SIGN_BIT) == (value & SIGN_BIT) && (value & SIGN_BIT) != (n & SIGN_BIT);

        self.register_a = n;

        self.status = if carry_flag1 || carry_flag2 {
            self.status | FLAG_CARRY
        }else {
            self.status & !FLAG_CARRY
        };

        self.status = if overflow {
            self.status | FLAG_OVERFLOW
        }else {
            self.status & !FLAG_OVERFLOW
        };

        self.update_zero_and_negative_flags(self.register_a);
    }

    fn txs(&mut self, mode: &AddressingMode){
        self.stack_pointer = self.register_x;
    }

    fn tsx(&mut self, mode: &AddressingMode){
        self.register_x = self.stack_pointer;
        self.update_zero_and_negative_flags(self.register_x);
    }

    fn tya(&mut self, mode: &AddressingMode){
        self.register_a = self.register_y;
        self.update_zero_and_negative_flags(self.register_a);
    }

    fn tay(&mut self, mode: &AddressingMode){
        self.register_y = self.register_a;
        self.update_zero_and_negative_flags(self.register_y);
    }

    fn txa(&mut self, mode: &AddressingMode){
        self.register_a = self.register_x;
        self.update_zero_and_negative_flags(self.register_x);
    }

    fn tax(&mut self, mode: &AddressingMode){
        self.register_x = self.register_a;
        self.update_zero_and_negative_flags(self.register_a);
    }

    fn sty(&mut self, mode: &AddressingMode){
        let addr = self.get_operand_address(mode);
        self.mem_write(addr, self.register_y);
    }

    fn stx(&mut self, mode: &AddressingMode){
        let addr = self.get_operand_address(mode);
        self.mem_write(addr, self.register_x);
    }

    fn sta(&mut self, mode: &AddressingMode){
        let addr = self.get_operand_address(mode);
        self.mem_write(addr, self.register_a);
    }


    fn rti(&mut self, mode: &AddressingMode){
        self.status = self._pop() & !FLAG_BREAK | FLAG_BREAK2;
        self.program_counter = self._pop_u16();
    }
    fn plp(&mut self, mode: &AddressingMode){
        self.status = self._pop() & !FLAG_BREAK | FLAG_BREAK2;
    }
    
    fn php(&mut self, mode: &AddressingMode){
        self._push(self.status | FLAG_BREAK | FLAG_BREAK2);
    }

    fn pla(&mut self, mode: &AddressingMode){
        self.register_a = self._pop();
        self.update_zero_and_negative_flags(self.register_a);
    }

    fn pha(&mut self, mode: &AddressingMode){
        self._push(self.register_a);
    }

    fn nop(&mut self){
        //nothing to do
    }
    
    fn ldy(&mut self, mode: &AddressingMode){
        let addr = self.get_operand_address(mode);
        let value = self.mem_read(addr);

        self.register_y = value;
        self.update_zero_and_negative_flags(self.register_y);
    }

    fn ldx(&mut self, mode: &AddressingMode){
        let addr = self.get_operand_address(mode);
        let value = self.mem_read(addr);

        self.register_x = value;
        self.update_zero_and_negative_flags(self.register_x);
    }

    fn lda(&mut self, mode: &AddressingMode){
        let addr = self.get_operand_address(mode);
        let value = self.mem_read(addr);

        self.register_a = value;
        self.update_zero_and_negative_flags(self.register_a);
    }

    fn rts(&mut self, mode: &AddressingMode){
        let value = self._pop_u16() + 1;
        self.program_counter = value;
    }

    fn jsr(&mut self, mode: &AddressingMode){
        let addr = self.get_operand_address(mode);
        self._push_u16(self.program_counter + 2 -1);
        self.program_counter = addr;
        self.program_counter -= 2;
    }

    fn _push(&mut self, value: u8) {
        let addr = 0x0100 + self.stack_pointer as u16;
        self.mem_write(addr, value);
        self.stack_pointer = self.stack_pointer.wrapping_sub(1);
    }

    fn _pop(&mut self) -> u8{
        self.stack_pointer = self.stack_pointer.wrapping_add(1);
        let addr = 0x0100 + self.stack_pointer as u16;
        self.mem_read(addr)
        
    }

    fn _push_u16(&mut self, value: u16) {
        let addr = 0x0100 + self.stack_pointer.wrapping_sub(1) as u16;
        self.mem_write_u16(addr, value as u16);
        self.stack_pointer = self.stack_pointer.wrapping_sub(2);
    }

    fn _pop_u16(&mut self) -> u16{
        let addr = 0x0100 + self.stack_pointer.wrapping_add(1) as u16;
        let value = self.mem_read_u16(addr);
        self.stack_pointer = self.stack_pointer.wrapping_add(2);
        value
    }

    fn jmp(&mut self, mode: &AddressingMode){
        let addr = self.get_operand_address(mode);
        self.program_counter = addr;
        self.program_counter -= 2;
    }

    fn iny(&mut self, mode: &AddressingMode){
        self.register_y = self.register_y.wrapping_add(1);
        self.update_zero_and_negative_flags(self.register_y);
    }

    fn inx(&mut self, mode: &AddressingMode){
        self.register_x = self.register_x.wrapping_add(1);
        self.update_zero_and_negative_flags(self.register_x);
    }

    fn inc(&mut self, mode: &AddressingMode){
        let addr = self.get_operand_address(mode);
        let value = self.mem_read(addr);
        let value = value.wrapping_add(1);
        self.mem_write(addr, value);
        self.update_zero_and_negative_flags(value);
    }

    fn dey(&mut self, mode: &AddressingMode){
        let value = self.register_y.wrapping_sub(1);
        self.register_y = value;
        self.update_zero_and_negative_flags(self.register_y);
    }

    fn dex(&mut self, mode: &AddressingMode){
        let value = self.register_x.wrapping_sub(1);
        self.register_x = value;
        self.update_zero_and_negative_flags(self.register_x);
    }

    fn dec(&mut self, mode: &AddressingMode){
        let addr = self.get_operand_address(mode);
        let value = self.mem_read(addr);
        let value = value.wrapping_sub(1);
        self.mem_write(addr, value);
        self.update_zero_and_negative_flags(value);
    }

    fn cpy(&mut self, mode: &AddressingMode){
        self._cmp(self.register_y, mode);
    }

    fn cpx(&mut self, mode: &AddressingMode){
        self._cmp(self.register_x, mode);
    }

    fn cmp(&mut self, mode: &AddressingMode){
        self._cmp(self.register_a, mode);
    }

    fn _cmp(&mut self, target:u8, mode: &AddressingMode){
        let addr = self.get_operand_address(mode);
        let value = self.mem_read(addr);

        if target >= value {
            self.sec(&AddressingMode::Implied);
        }else{
            self.clc(&AddressingMode::Implied);
        }
        let value = target.wrapping_sub(value);
        self.update_zero_and_negative_flags(value);
    }

    fn clv(&mut self, mode: &AddressingMode){
        self.status = self.status & !FLAG_OVERFLOW;
    }

    fn sei(&mut self, mode: &AddressingMode){
        self.status = self.status | FLAG_INTERRUPT;
    }
    
    fn cli(&mut self, mode: &AddressingMode){
        self.status = self.status & !FLAG_INTERRUPT;
    }

    fn sed(&mut self, mode: &AddressingMode){
        self.status = self.status | FLAG_DECIMAL;
    }
    
    fn cld(&mut self, mode: &AddressingMode){
        self.status = self.status & !FLAG_DECIMAL;
    }

    fn clc(&mut self, mode: &AddressingMode){
        self.status = self.status & !FLAG_CARRY;
    }

    fn sec(&mut self, mode: &AddressingMode){
        self.status = self.status | FLAG_CARRY;
    }

    fn brk(&mut self, mode: &AddressingMode){
        if self.status & FLAG_BREAK != 0{
            return;
        }

        self._push_u16(self.program_counter);
        self._push(self.status);

        self.program_counter = self.mem_read_u16(0xFFFE);
        self.status = self.status | FLAG_BREAK;
    }

    fn _branch(&mut self, mode: &AddressingMode, flag:u8, nonzero:bool){
        let addr = self.get_operand_address(mode);
        if nonzero {
            if self.status & flag != 0{
                self.program_counter = addr
            }
        }else {
            if self.status & flag == 0 {
                self.program_counter = addr
            }
        }
    }

    fn bpl(&mut self, mode: &AddressingMode){
        self._branch(mode, FLAG_NEGATIVE, false);
    }

    fn bmi(&mut self, mode: &AddressingMode){
        self._branch(mode, FLAG_NEGATIVE, true);
    }

    fn bit(&mut self, mode: &AddressingMode){
        let addr = self.get_operand_address(mode);
        let value = self.mem_read(addr);

        let zero = self.register_a & value;
        if zero == 0 {
            self.status = self.status | FLAG_ZERO;
        }else{
            self.status = self.status & !FLAG_ZERO;
        }

        let n = value & FLAG_NEGATIVE;
        let v = value & FLAG_OVERFLOW;
        if n != 0 {
            self.status = self.status | FLAG_NEGATIVE;
        }else {
            self.status = self.status & !FLAG_NEGATIVE;
        }

        if v != 0 {
            self.status = self.status | FLAG_OVERFLOW;
        }else {
            self.status = self.status & !FLAG_OVERFLOW;
        }
    }

    fn beq(&mut self, mode: &AddressingMode){
        self._branch(mode, FLAG_ZERO, true);
    }

    fn bne(&mut self, mode: &AddressingMode){
        self._branch(mode, FLAG_ZERO, false);
    }

    fn bcc(&mut self, mode: &AddressingMode){
        self._branch(mode, FLAG_CARRY, false);
    }

    fn bcs(&mut self, mode: &AddressingMode){
        self._branch(mode, FLAG_CARRY, true);
    }
    

    fn ror(&mut self, mode: &AddressingMode){
        let (value, carry) = if mode == &AddressingMode::Accumulator {
            let carry = self.register_a & 0x01;
            self.register_a = self.register_a / 2;
            self.register_a = self.register_a | ((self.status & FLAG_CARRY) << 7);
            (self.register_a, carry)
        }else{
            let addr = self.get_operand_address(mode);
            let value = self.mem_read(addr);
            let carry = value & 0x01;
            let value = value / 2;
            let value = value | ((self.status & FLAG_CARRY) << 7);
            self.mem_write(addr, value);
            (value, carry)
        };
        self.status = if carry == 1 {
            self.status | FLAG_CARRY
        }else {
            self.status & !FLAG_CARRY
        };
        self.update_zero_and_negative_flags(value);
    }


    fn rol(&mut self, mode: &AddressingMode){
        let (value, carry) = if mode == &AddressingMode::Accumulator {
            let (value, carry) = self.register_a.overflowing_mul(2);
            self.register_a = value | (self.status & FLAG_CARRY);
            (self.register_a, carry)
        }else{
            let addr = self.get_operand_address(mode);
            let value = self.mem_read(addr);
            let (value, carry) = value.overflowing_mul(2);
            let value = value | (self.status & FLAG_CARRY);
            self.mem_write(addr, value);
            (value, carry)
        };
        self.status = if carry {
            self.status | FLAG_CARRY
        }else {
            self.status & !FLAG_CARRY
        };
        self.update_zero_and_negative_flags(value);
    }

    fn asl(&mut self, mode: &AddressingMode){
        let (value, carry) = if mode == &AddressingMode::Accumulator {
            let (value, carry) = self.register_a.overflowing_mul(2);
            self.register_a = value;
            (value, carry)
        }else{
            let addr = self.get_operand_address(mode);
            let value = self.mem_read(addr);
            let (value, carry) = value.overflowing_mul(2);
            self.mem_write(addr, value);
            (value, carry)
        };
        self.status = if carry {
            self.status | FLAG_CARRY
        }else {
            self.status & !FLAG_CARRY
        };
        self.update_zero_and_negative_flags(value);
    }

    fn lsr(&mut self, mode: &AddressingMode){
        let (value, carry) = if mode == &AddressingMode::Accumulator {
            let carry = self.register_a & 0x01;
            let value = self.register_a / 2;
            self.register_a = value;
            (value, carry)
        }else{
            let addr = self.get_operand_address(mode);
            let value = self.mem_read(addr);
            let carry = value & 0x01;
            let value = value / 2;
            self.mem_write(addr, value);
            (value, carry)
        };
        self.status = if carry == 1 {
            self.status | FLAG_CARRY
        }else {
            self.status & !FLAG_CARRY
        };
        self.update_zero_and_negative_flags(value);
    }

    fn and(&mut self, mode: &AddressingMode){
        let addr = self.get_operand_address(mode);
        let value = self.mem_read(addr);

        self.register_a = self.register_a & value;
        self.update_zero_and_negative_flags(self.register_a);
    }

    fn eor(&mut self, mode: &AddressingMode){
        let addr = self.get_operand_address(mode);
        let value = self.mem_read(addr);

        self.register_a = self.register_a ^ value;
        self.update_zero_and_negative_flags(self.register_a);
    }

    fn ora(&mut self, mode: &AddressingMode){
        let addr = self.get_operand_address(mode);
        let value = self.mem_read(addr);

        self.register_a = self.register_a | value;
        self.update_zero_and_negative_flags(self.register_a);
    }

    fn sbc(&mut self, mode: &AddressingMode){
        //A-M-(1-C)
        let addr = self.get_operand_address(mode);
        let value = self.mem_read(addr);
        
        let carry = self.status & FLAG_CARRY;
        let (v1, carry_flag1) = self.register_a.overflowing_sub(value);
        let (n, carry_flag2) = v1.overflowing_sub(1-carry);
        
        let overflow = (self.register_a & SIGN_BIT) != (value & SIGN_BIT) && (self.register_a & SIGN_BIT) != (n & SIGN_BIT);

        self.register_a = n;

        self.status = if !carry_flag1 && !carry_flag2 {
            self.status | FLAG_CARRY
        }else {
            self.status & !FLAG_CARRY
        };

        self.status = if overflow {
            self.status | FLAG_OVERFLOW
        }else {
            self.status & !FLAG_OVERFLOW
        };

        self.update_zero_and_negative_flags(self.register_a);
    }

    fn update_zero_and_negative_flags(&mut self, result: u8){

        self.status = if result == 0{
            self.status | FLAG_ZERO
        }else{
            self.status & !FLAG_ZERO
        };

        self.status = if result & SIGN_BIT != 0{
            self.status | FLAG_NEGATIVE
        }else{
            self.status & !FLAG_NEGATIVE
        };
    }


}
    //pub fn interpret(&mut self, program: Vec<u8>){
    

#[cfg(test)]
mod test {
    use super::*;

    fn run<F>(program: Vec<u8>, f:F) -> CPU
    where
        F: Fn(&mut CPU),
    {
        let mut cpu = CPU::new();
        cpu.load(program);
        cpu.reset();
        f(&mut cpu);
        cpu.run();
        cpu
    }

    fn assert_status(cpu: &CPU, flags: u8) {
        assert_eq!(cpu.status, flags);
    }

    #[test]
    fn test_0xa9_lda_immediate_load_data(){
        let mut cpu = CPU::new();
        cpu.load_and_run(vec![0xa9, 0x05, 0x00]);
        assert_eq!(cpu.register_a, 0x05);
        assert!(cpu.status & 0b0000_0010 == 0b00);
        assert!(cpu.status & 0b1000_0000 == 0);
    }
    #[test]
    fn test_0xa9_lda_zero_flag(){
        let mut cpu = CPU::new();
        cpu.load_and_run(vec![0xa9, 0x00, 0x00]);
        assert!(cpu.status & 0b0000_0010 == 0b10);
    }

    #[test]
    fn test_0xa9_lda_negative_flag(){
        let mut cpu = CPU::new();
        cpu.load_and_run(vec![0xa9, 0x80, 0x00]);
        assert!(cpu.status & 0b1000_0000 == 0b1000_0000);
    }
    #[test]
    fn test_0xxx_tax_move_a_to_x(){
        let mut cpu = CPU::new();
        let cpu = run(vec![0xaa, 0x00], |cpu|{
            cpu.register_a = 10;
        });
        assert_eq!(cpu.register_x, 10);
    }
    #[test]
    fn test_5_ops_working_together(){
        let mut cpu = CPU::new();
        cpu.load_and_run(vec![0xa9, 0xc0, 0xaa, 0xe8, 0x00]);

        assert_eq!(cpu.register_x, 0xc1);
    }

    #[test]
    fn test_inx_overflow(){
        let mut cpu = CPU::new();
        cpu.register_x = 0xff;
        cpu.load_and_run(vec![0xe8, 0xe8, 0x00]);

        assert_eq!(cpu.register_x, 1);
    }
    #[test]
    fn test_load_from_memory(){
        let mut cpu = CPU::new();
        cpu.mem_write(0x10, 0x55);

        cpu.load_and_run(vec![0xa5, 0x10, 0x00]);

        assert_eq!(cpu.register_a, 0x55);
    }

    #[test]
    fn test_lda_from_memory_zero_page_x(){
        let mut cpu = CPU::new();
        cpu.load(vec![0xb5, 0x10, 0x00]);
        cpu.reset();
        cpu.mem_write(0x11, 0x56);
        cpu.register_x = 0x01;
        cpu.run();
        assert_eq!(cpu.register_a, 0x56);
    }

    #[test]
    fn test_lda_from_memory_absolute(){
        let mut cpu = CPU::new();
        cpu.load(vec![0xad, 0x10, 0xaa, 0x00]);
        cpu.reset();
        cpu.mem_write(0xAA10, 0x56);
        cpu.register_x = 0x01;
        cpu.run();
        assert_eq!(cpu.register_a, 0x56);
    }

    #[test]
    fn test_lda_from_memory_absolute_x(){
        let mut cpu = CPU::new();
        cpu.load(vec![0xbd, 0x10, 0xaa, 0x00]);
        cpu.reset();
        cpu.mem_write(0xAA11, 0x57);
        cpu.register_x = 0x01;
        cpu.run();
        assert_eq!(cpu.register_a, 0x57);
    }

    #[test]
    fn test_lda_from_memory_absolute_y(){
        let mut cpu = CPU::new();
        cpu.load(vec![0xb9, 0x10, 0xaa, 0x00]);
        cpu.reset();
        cpu.mem_write(0xAA15, 0x58);
        cpu.register_y = 0x05;
        cpu.run();
        assert_eq!(cpu.register_a, 0x58);
    }

        #[test]
    fn test_lda_from_memory_indirect_x(){
        let mut cpu = CPU::new();
        cpu.load(vec![0xa1, 0x10]);
        cpu.reset();
        cpu.mem_write(0x10, 0x58);
        cpu.mem_write(0x11, 0x59);
        cpu.mem_write(0x5958, 0x5A);
        //cpu.register_y = 0x05;
        cpu.run();
        assert_eq!(cpu.register_a, 0x5A);
    }

    #[test]
    fn test_lda_from_memory_indirect_y(){
        let mut cpu = CPU::new();
        cpu.load(vec![0xb1, 0x10, 0x00]);
        cpu.reset();
        cpu.mem_write(0x10, 0x09);
        cpu.mem_write(0x11, 0xFF);
        cpu.mem_write(0xFF0C, 0x5B);
        cpu.register_y = 0x03;
        cpu.run();
        assert_eq!(cpu.register_a, 0x5B);
    }

        #[test]
    fn test_sta_from_memory(){
        let mut cpu = CPU::new();
        cpu.load(vec![0x85, 0x10, 0x00]);
        cpu.reset();
        cpu.register_a = 0xBA;
        cpu.run();
        assert_eq!(cpu.mem_read(0x10), 0xBA);
    }

    #[test]
    fn test_adc_no_carry(){
        let mut cpu = CPU::new();
        cpu.load(vec![0x69, 0x10, 0x00]);
        cpu.reset();
        cpu.register_a = 0x20;
        cpu.run();
        assert_eq!(cpu.register_a, 0x30);
        assert_eq!(cpu.status, 0x00);
    }

    #[test]
    fn test_adc_has_carry(){
        let mut cpu = CPU::new();
        cpu.load(vec![0x69, 0x10, 0x00]);
        cpu.reset();
        cpu.register_a = 0x20;
        cpu.status = 0x01;
        cpu.run();
        assert_eq!(cpu.register_a, 0x31);
        assert_eq!(cpu.status, 0x00);
    }

    #[test]
    fn test_adc_occur_carry(){
        let mut cpu = CPU::new();
        cpu.load(vec![0x69, 0x01, 0x00]);
        cpu.reset();
        cpu.register_a = 0xFF;
        cpu.run();
        assert_eq!(cpu.register_a, 0x00);
        assert_eq!(cpu.status, 0x03);
    }

    #[test]
    fn test_adc_no_overflow(){
        let mut cpu = CPU::new();
        cpu.load(vec![0x69, 0x7F, 0x00]);
        cpu.reset();
        cpu.register_a = 0x82;
        cpu.run();
        assert_eq!(cpu.register_a, 0x01);
        assert_eq!(cpu.status, 0x01);
    }

    #[test]
    fn test_adc_overflow_plus(){
        let mut cpu = CPU::new();
        cpu.load(vec![0x69, 0x10, 0x00]);
        cpu.reset();
        cpu.register_a = 0x7F;
        cpu.run();
        assert_eq!(cpu.register_a, 0x8F);
        assert_eq!(cpu.status, 0xC0);
    }

    #[test]
    fn test_adc_overflow_minus(){
        let mut cpu = CPU::new();
        cpu.load(vec![0x69, 0x81, 0x00]);
        cpu.reset();
        cpu.register_a = 0x81;
        cpu.run();
        assert_eq!(cpu.register_a, 0x02);
        assert_eq!(cpu.status, 0x41);
    }
    #[test]
    fn test_adc_overflow_plus_carry(){
        let mut cpu = CPU::new();
        cpu.load(vec![0x69, 0x6F, 0x00]);
        cpu.reset();
        cpu.register_a = 0x10;
        cpu.status = 0x01;
        cpu.run();
        assert_eq!(cpu.register_a, 0x80);
        assert_eq!(cpu.status, 0xC0);
    }
    #[test]
    fn test_adc_overflow_minus_carry(){
    let mut cpu = CPU::new();
        cpu.load(vec![0x69, 0x80, 0x00]);
        cpu.reset();
        cpu.register_a = 0x80;
        cpu.status = 0x01;
        cpu.run();
        assert_eq!(cpu.register_a, 0x01);
        assert_eq!(cpu.status, 0x41);
    }
    //SBC
    #[test]
    fn test_sbc_no_carry(){
        let mut cpu = CPU::new();
        cpu.load(vec![0xe9, 0x10, 0x00]);
        cpu.reset();
        cpu.register_a = 0x20;
        cpu.run();
        assert_eq!(cpu.register_a, 0x0F);
        assert_eq!(cpu.status, 0x01);
    }
    
    #[test]
    fn test_sbc_has_carry(){
        let mut cpu = CPU::new();
        cpu.load(vec![0xE9, 0x10, 0x00]);
        cpu.reset();
        cpu.register_a = 0x20;
        cpu.status = 0x01;
        cpu.run();
        assert_eq!(cpu.register_a, 0x31);
        assert_eq!(cpu.status, 0x00);
    }

    #[test]
    fn test_sbc_occur_carry(){
        let mut cpu = CPU::new();
        cpu.load(vec![0xe9, 0x02, 0x00]);
        cpu.reset();
        cpu.register_a = 0x01;
        cpu.run();
        assert_eq!(cpu.register_a, 0xFE);
        assert_eq!(cpu.status, 0x80);
    }

    #[test]
    fn test_sbc_no_overflow(){
        let mut cpu = CPU::new();
        cpu.load(vec![0xE9, 0x7F, 0x00]);
        cpu.reset();
        cpu.register_a = 0x7E;
        cpu.status = 0x01;
        println!("SBC start: A={:02X} status={:02X}", cpu.register_a, cpu.status);
        cpu.run();
        print!("{}", cpu.status);
        assert_eq!(cpu.register_a, 0xFF);
        assert_eq!(cpu.status, 0x80);
    }

    #[test]
    fn test_sbc_overflow(){
        let mut cpu = CPU::new();
        cpu.load(vec![0xE9, 0x81, 0x00]);
        cpu.reset();
        cpu.register_a = 0x7F;
        cpu.run();
        assert_eq!(cpu.register_a, 0xFD);
        assert_eq!(cpu.status, 0xC0);
    }
    #[test]
    fn test_sbc_overflow_with_carry(){
        let mut cpu = CPU::new();
        cpu.load(vec![0xE9, 0x81, 0x00]);
        cpu.reset();
        cpu.register_a = 0x7F;
        cpu.status = 0x01;
        cpu.run();
        assert_eq!(cpu.register_a, 0xFE);
        assert_eq!(cpu.status, 0xC0);
    }

    #[test]
    fn test_and(){
        let mut cpu = CPU::new();
        cpu.load(vec![0x29, 0x01, 0x00]);
        cpu.reset();
        cpu.register_a = 0x81;
        cpu.run();
        assert_eq!(cpu.register_a, 0x01);
    }

    #[test]
    fn test_eor(){
        let cpu = run(vec![0x49, 0x0c, 0x00], |cpu|{
            cpu.register_a = 0x0A;
        });
        assert_eq!(cpu.register_a, 0x06);
        assert_status(&cpu, 0);
    }

    #[test]
    fn test_asl_zero(){
        let cpu = run(vec![0x06, 0x01, 0x00], |cpu|{
            cpu.mem_write(0x0001, 0x03);
        });
        assert_eq!(cpu.mem_read(0x001), 0x06);
        assert_status(&cpu, 0);
    }

    #[test]
    fn test_asl_a(){
        let cpu = run(vec![0x0A, 0x00], |cpu|{
            cpu.register_a = 0x02;
        });
        assert_eq!(cpu.register_a, 0x04);
        assert_status(&cpu, 0);
    }

    #[test]
    fn test_asl_zero_overflow(){
        let cpu = run(vec![0x06, 0x01, 0x00], |cpu|{
            cpu.mem_write(0x0001, 0x81);
        });
        assert_eq!(cpu.mem_read(0x001), 0x02);
        assert_status(&cpu, FLAG_CARRY);
    }

    #[test]
    fn test_lsr_a_occur_carry(){
        let cpu = run(vec![0x4A, 0x00], |cpu|{
            cpu.register_a = 0x03;
        });
        assert_eq!(cpu.register_a, 0x01);
        assert_status(&cpu, FLAG_CARRY);
    }

    #[test]
    fn test_lsr_zero_carry(){
        let cpu = run(vec![0x46, 0x01, 0x00], |cpu|{
            cpu.mem_write(0x0001, 0x03);
        });
        assert_eq!(cpu.mem_read(0x001), 0x01);
        assert_status(&cpu, FLAG_CARRY);
    }

        #[test]
    fn test_lsr_zero_zero(){
        let cpu = run(vec![0x46, 0x01, 0x00], |cpu|{
            cpu.mem_write(0x0001, 0x01);
        });
        assert_eq!(cpu.mem_read(0x001), 0x00);
        assert_status(&cpu, FLAG_ZERO | FLAG_CARRY);
    }


    #[test]
    fn test_rol_zero(){
        let cpu = run(vec![0x26, 0x01, 0x00], |cpu|{
            cpu.mem_write(0x0001, 0x03);
        });
        assert_eq!(cpu.mem_read(0x001), 0x06);
        assert_status(&cpu, 0);
    }

    #[test]
    fn test_rol_a(){
        let cpu = run(vec![0x2A, 0x00], |cpu|{
            cpu.register_a = 0x02;
        });
        assert_eq!(cpu.register_a, 0x04);
        assert_status(&cpu, 0);
    }

    #[test]
    fn test_rol_a_carry(){
        let cpu = run(vec![0x2A, 0x00], |cpu|{
            cpu.register_a = 0x03;
            cpu.status = FLAG_CARRY;
        });
        assert_eq!(cpu.register_a, 0x03 * 2 + 1);
        assert_status(&cpu, 0);
    }

    #[test]
    fn test_rol_a_zero_carry(){
        let cpu = run(vec![0x2A, 0x00], |cpu|{
            cpu.register_a = 0x00;
            cpu.status = FLAG_CARRY;
        });
        assert_eq!(cpu.register_a, 0x01);
        assert_status(&cpu, 0);
    }

    #[test]
    fn test_rol_zeropage_zero_carry(){
        let cpu = run(vec![0x26, 0x01, 0x00], |cpu|{
            cpu.mem_write(0x0001, 0x00);
            cpu.status = FLAG_CARRY;
        });
        assert_eq!(cpu.mem_read(0x001), 0x01);
        assert_status(&cpu, 0);
    }

    #[test]
    fn test_ror_a(){
        let cpu = run(vec![0x6A, 0x00], |cpu|{
            cpu.register_a = 0x02;
        });
        assert_eq!(cpu.register_a, 0x01);
        assert_status(&cpu, 0);
    }

    #[test]
    fn test_ror_zero(){
        let cpu = run(vec![0x66, 0x01, 0x00], |cpu|{
            cpu.mem_write(0x0001, 0x02);
        });
        assert_eq!(cpu.mem_read(0x0001), 0x01);
        assert_status(&cpu, 0);
    }

    #[test]
    fn test_ror_a_carry(){
        let cpu = run(vec![0x6A, 0x00], |cpu|{
            cpu.register_a = 0x03;
            cpu.status = FLAG_CARRY;
        });
        assert_eq!(cpu.register_a, 0x81);
        assert_status(&cpu, FLAG_CARRY | FLAG_NEGATIVE);
    }

    #[test]
    fn test_ror_zero_carry(){
        let cpu = run(vec![0x66, 0x01, 0x00], |cpu|{
            cpu.mem_write(0x0001, 0x03);
            cpu.status = FLAG_CARRY;
        });
        assert_eq!(cpu.mem_read(0x001), 0x81);
        assert_status(&cpu, FLAG_CARRY | FLAG_NEGATIVE);
    }

    #[test]
    fn test_ror_a_zero(){
        let cpu = run(vec![0x6A, 0x00], |cpu|{
            cpu.register_a = 0x00;
            cpu.status = FLAG_CARRY;
        });
        assert_eq!(cpu.register_a, 0x80);
        assert_status(&cpu, FLAG_NEGATIVE);
    }

    #[test]
    fn test_ror_zero_zero(){
        let cpu = run(vec![0x66, 0x01, 0x00], |cpu|{
            cpu.mem_write(0x0001, 0x00);
            cpu.status = FLAG_CARRY;
        });
        assert_eq!(cpu.mem_read(0x001), 0x80);
        assert_status(&cpu, FLAG_NEGATIVE);
    }

    #[test]
    fn test_bcc(){
        let cpu = run(vec![0x90, 0x02, 0x00, 0x00, 0xE8, 0x00], |cpu|{
        });
        assert_eq!(cpu.register_x, 0x01);
        assert_status(&cpu, 0);
        assert_eq!(cpu.program_counter, 0x8005);
    }

    #[test]
    fn test_bcc_carry(){
        let cpu = run(vec![0x90, 0x02, 0x00, 0x00, 0xE8, 0x00], |cpu|{
            cpu.status = FLAG_CARRY;
        });
        assert_eq!(cpu.register_x, 0x00);
        assert_status(&cpu, FLAG_CARRY);
        assert_eq!(cpu.program_counter, 0x8003)
    }

    #[test]
    fn test_bcc_nagative(){
        let cpu = run(vec![0x90, 0xfc, 0x00], |cpu|{
            cpu.mem_write(0x7FFF, 0x00);
            cpu.mem_write(0x7FFE, 0xe8);
        });
        assert_eq!(cpu.register_x, 0x01);
        assert_status(&cpu, 0);
        assert_eq!(cpu.program_counter, 0x8000);
    }

    #[test]
    fn test_bcs(){
        let cpu = run(vec![0xB0, 0x02, 0x00, 0x00, 0xE8, 0x00], |cpu|{
        });
        assert_eq!(cpu.register_x, 0x00);
        assert_status(&cpu, 0);
        assert_eq!(cpu.program_counter, 0x8003);
    }

    #[test]
    fn test_bcs_carry(){
        let cpu = run(vec![0xB0, 0x02, 0x00, 0x00, 0xE8, 0x00], |cpu|{
            cpu.status = FLAG_CARRY;
        });
        assert_eq!(cpu.register_x, 0x01);
        assert_status(&cpu, FLAG_CARRY);
        assert_eq!(cpu.program_counter, 0x8006)
    }

    #[test]
    fn test_bcs_nagative(){
        let cpu = run(vec![0xB0, 0xfc, 0x00], |cpu|{
            cpu.mem_write(0x7FFF, 0x00);
            cpu.mem_write(0x7FFE, 0xe8);
            cpu.status = FLAG_CARRY;
        });
        assert_eq!(cpu.register_x, 0x01);
        assert_status(&cpu, FLAG_CARRY);
        assert_eq!(cpu.program_counter, 0x8000);
    }

    #[test]
    fn test_beq(){
        let cpu = run(vec![0xF0, 0x02, 0x00, 0x00, 0xE8, 0x00], |cpu|{
            cpu.status = FLAG_ZERO;
        });
        assert_eq!(cpu.register_x, 0x01);
        assert_status(&cpu, 0);
        assert_eq!(cpu.program_counter, 0x8006);
    }

        #[test]
    fn test_beq_zero(){
        let cpu = run(vec![0xF0, 0x02, 0x00, 0x00, 0xE8, 0x00], |cpu|{
        });
        assert_eq!(cpu.register_x, 0x00);
        assert_status(&cpu, 0);
        assert_eq!(cpu.program_counter, 0x8003)
    }

    #[test]
    fn test_bne(){
        let cpu = run(vec![0xD0, 0x02, 0x00, 0x00, 0xE8, 0x00], |cpu|{
            cpu.status = FLAG_ZERO;
        });
        assert_eq!(cpu.register_x, 0x00);
        assert_status(&cpu, FLAG_ZERO);
        assert_eq!(cpu.program_counter, 0x8003);
    }

    #[test]
    fn test_bne_zero(){
        let cpu = run(vec![0xD0, 0x02, 0x00, 0x00, 0xE8, 0x00], |cpu|{
        });
        assert_eq!(cpu.register_x, 0x01);
        assert_status(&cpu, 0);
        assert_eq!(cpu.program_counter, 0x8006)
    }

    #[test]
    fn test_bit(){
        let cpu = run(vec![0x24, 0x00, 0x00], |cpu|{
            cpu.register_a = 0x00;
            cpu.mem_write(0x0000, 0x00);
        });
        assert_status(&cpu, FLAG_ZERO);
    }

   #[test]
    fn test_bit_negative(){
        let cpu = run(vec![0x24, 0x00, 0x00], |cpu|{
            cpu.register_a = 0x00;
            cpu.mem_write(0x0000, 0x80);
        });
        assert_status(&cpu, FLAG_NEGATIVE | FLAG_ZERO);
    }

    #[test]
    fn test_bit_overflow(){
        let cpu = run(vec![0x24, 0x00, 0x00], |cpu|{
            cpu.register_a = 0x40;
            cpu.mem_write(0x0000, 0x40);
        });
        assert_status(&cpu, FLAG_OVERFLOW);
    }

    #[test]
    fn test_bmi_negative(){
        let cpu = run(vec![0x30, 0x02, 0x00, 0x00, 0xe8, 0x00], |cpu|{
            cpu.status = FLAG_NEGATIVE;
        });
        assert_eq!(cpu.register_x, 0x01);
        assert_status(&cpu, 0);
        assert_eq!(cpu.program_counter, 0x8006)
    }

    #[test]
    fn test_bmi(){
        let cpu = run(vec![0x30, 0x02, 0x00, 0x00, 0xe8, 0x00], |cpu|{
        });
        assert_eq!(cpu.register_x, 0x00);
        assert_status(&cpu, 0);
        assert_eq!(cpu.program_counter, 0x8003);
    }

        #[test]
    fn test_bpl(){
        let cpu = run(vec![0x10, 0x02, 0x00, 0x00, 0xe8, 0x00], |cpu|{
        });
        assert_eq!(cpu.register_x, 0x01);
        assert_status(&cpu, 0);
        assert_eq!(cpu.program_counter, 0x8006);
    }

   #[test]
    fn test_bpl_negative(){
        let cpu = run(vec![0x10, 0x02, 0x00, 0x00, 0xe8, 0x00], |cpu|{
            cpu.status = FLAG_NEGATIVE;
        });
        assert_eq!(cpu.register_x, 0x00);
        assert_status(&cpu, FLAG_NEGATIVE);
        assert_eq!(cpu.program_counter, 0x8003)
    }

    #[test]
    fn test_clc(){
        let cpu = run(vec![0x18,0x00], |cpu|{
            cpu.status = FLAG_CARRY | FLAG_NEGATIVE;
        });
        assert_status(&cpu, FLAG_NEGATIVE);
    }

    #[test]
    fn test_sec(){
        let cpu = run(vec![0x38,0x00], |cpu|{
            cpu.status = FLAG_NEGATIVE;
        });
        assert_status(&cpu, FLAG_CARRY | FLAG_NEGATIVE);
    }

        #[test]
    fn test_cld(){
        let cpu = run(vec![0xD8,0x00], |cpu|{
            cpu.status = FLAG_DECIMAL | FLAG_NEGATIVE;
        });
        assert_status(&cpu, FLAG_NEGATIVE);
    }

    #[test]
    fn test_sed(){
        let cpu = run(vec![0xF8,0x00], |cpu|{
            cpu.status = FLAG_NEGATIVE;
        });
        assert_status(&cpu, FLAG_DECIMAL | FLAG_NEGATIVE);
    }

    #[test]
    fn test_cli(){
        let cpu = run(vec![0x58,0x00], |cpu|{
            cpu.status = FLAG_INTERRUPT | FLAG_NEGATIVE;
        });
        assert_status(&cpu, FLAG_NEGATIVE);
    }

    #[test]
    fn test_sei(){
        let cpu = run(vec![0x78,0x00], |cpu|{
            cpu.status = FLAG_NEGATIVE;
        });
        assert_status(&cpu, FLAG_INTERRUPT | FLAG_NEGATIVE);
    }

    #[test]
    fn test_clv(){
        let cpu = run(vec![0xB8,0x00], |cpu|{
            cpu.status = FLAG_OVERFLOW | FLAG_NEGATIVE;
        });
        assert_status(&cpu, FLAG_NEGATIVE);
    }

    #[test]
    fn test_cmq_(){
        let cpu = run(vec![0xC9, 0x01, 0x00], |cpu|{
            cpu.register_a = 0x02;
        });
        assert_status(&cpu, FLAG_CARRY);
    }

    #[test]
        fn test_cmq_zero(){
        let cpu = run(vec![0xC9, 0x01, 0x00], |cpu|{
            cpu.register_a = 0x01;
        });
        assert_status(&cpu, FLAG_ZERO | FLAG_CARRY);
    }

        #[test]
        fn test_cmq_nega(){
        let cpu = run(vec![0xC9, 0x02, 0x00], |cpu|{
            cpu.register_a = 0x01;
        });
        assert_status(&cpu, FLAG_NEGATIVE);
    }

        #[test]
        fn test_cpx_nega(){
        let cpu = run(vec![0xE0, 0x02, 0x00], |cpu|{
            cpu.register_x = 0x01;
        });
        assert_status(&cpu, FLAG_NEGATIVE);
    }

    #[test]
    fn test_cpy_nega(){
        let cpu = run(vec![0xC0, 0x02, 0x00], |cpu|{
            cpu.register_x = 0x01;
        });
        assert_status(&cpu, FLAG_NEGATIVE);
    }

    #[test]
    fn test_dec(){
        let cpu = run(vec![0xC6, 0x01, 0x00], |cpu|{
            cpu.mem_write(0x0001, 0x05);
        });
        assert_eq!(cpu.mem_read(0x0001), 0x04);
        assert_status(&cpu, 0);
    }

    #[test]
    fn test_dec_overflow(){
        let cpu = run(vec![0xC6, 0x01, 0x00], |cpu|{
            cpu.mem_write(0x0001, 0x00);
        });
        assert_eq!(cpu.mem_read(0x0001), 0xFF);
        assert_status(&cpu, FLAG_NEGATIVE);
    }

        #[test]
    fn test_dex(){
        let cpu = run(vec![0xCA, 0x00], |cpu|{
            cpu.register_x = 0x05;
        });
        assert_eq!(cpu.register_x, 0x04);
        assert_status(&cpu, 0);
    }

    #[test]
    fn test_dex_overflow(){
        let cpu = run(vec![0xCA, 0x00], |cpu|{
            cpu.register_x = 0x00;
        });
        assert_eq!(cpu.register_x, 0xFF);
        assert_status(&cpu, FLAG_NEGATIVE);
    }

    #[test]
    fn test_dey(){
        let cpu = run(vec![0x88, 0x00], |cpu|{
            cpu.register_y = 0x05;
        });
        assert_eq!(cpu.register_y, 0x04);
        assert_status(&cpu, 0);
    }

    #[test]
    fn test_dey_overflow(){
        let cpu = run(vec![0x88, 0x00], |cpu|{
            cpu.register_y = 0x00;
        });
        assert_eq!(cpu.register_y, 0xFF);
        assert_status(&cpu, FLAG_NEGATIVE);
    }

    #[test]
    fn test_inc(){
        let cpu = run(vec![0xE6, 0x01, 0x00], |cpu|{
            cpu.mem_write(0x0001, 0x05);
        });
        assert_eq!(cpu.mem_read(0x0001), 0x06);
        assert_status(&cpu, 0);
    }

    #[test]
    fn test_inc_overflow(){
        let cpu = run(vec![0xE6, 0x01, 0x00], |cpu|{
            cpu.mem_write(0x0001, 0xFF);
        });
        assert_eq!(cpu.mem_read(0x0001), 0x00);
        assert_status(&cpu, FLAG_ZERO);
    }
    #[test]
    fn test_iny(){
        let cpu = run(vec![0xC8, 0x01, 0x00], |cpu|{
            cpu.register_y = 0x05;
        });
        assert_eq!(cpu.register_y, 0x06);
        assert_status(&cpu, 0);
    }

    #[test]
    fn test_iny_overflow(){
        let cpu = run(vec![0xC8, 0x01, 0x00], |cpu|{
            cpu.register_y = 0xFF;
        });
        assert_eq!(cpu.register_y, 0x00);
        assert_status(&cpu, FLAG_ZERO);
    }
    #[test]
    fn test_jmp(){
        let cpu = run(vec![0x4C, 0x30, 0x40, 0x00], |cpu|{
            cpu.mem_write(0x4030, 0xe8);
            cpu.mem_write(0x4031, 0x00);
        });
        assert_eq!(cpu.register_x, 0x01);
        assert_status(&cpu, 0);
        assert_eq!(cpu.program_counter, 0x4032)
    }

    #[test]
    fn test_jmp_indirect(){
        let cpu = run(vec![0x6C, 0x3, 0x40], |cpu|{
            cpu.mem_write(0x4031, 0x01);
            cpu.mem_write(0x4032, 0x02);

            cpu.mem_write(0x0201, 0xe8);
            cpu.mem_write(0x0202, 0x00);
        });
        assert_eq!(cpu.register_x, 0x01);
        assert_status(&cpu, 0);
        assert_eq!(cpu.program_counter, 0x0203)
    }
    
    #[test]
    fn test_jsr(){
        let cpu = run(vec![0x20, 0x30, 0x40, 0x00], |cpu|{
            cpu.mem_write(0x4030, 0xe8);
            cpu.mem_write(0x4031, 0x00);
        });
        assert_eq!(cpu.register_x, 0x01);
        assert_status(&cpu, 0);
        assert_eq!(cpu.program_counter, 0x4032);
        assert_eq!(cpu.stack_pointer, 0xFD);
        assert_eq!(cpu.mem_read_u16(0x01FE), 0x8003);
    }

    #[test]
    fn test_rts(){
        let cpu = run(vec![0x60, 0x00], |cpu|{
            cpu.mem_write(0x01FF, 0x05);
            cpu.mem_write(0x01FE, 0x06);
            cpu.mem_write(0x0506, 0xe8);
            cpu.mem_write(0x0507, 0x00);

            cpu.stack_pointer = 0xFD;
        });
        assert_eq!(cpu.register_x, 0x01);
        assert_status(&cpu, 0);
        assert_eq!(cpu.program_counter, 0x0508);
        assert_eq!(cpu.stack_pointer, 0xFF);
        assert_eq!(cpu.mem_read_u16(0x01FE), 0x0506);
    }

    #[test]
    fn test_jsr_rts(){
        let cpu = run(vec![0x20, 0x30, 0x40, 0x00], |cpu|{
            cpu.mem_write(0x4030, 0xe8);
            cpu.mem_write(0x4031, 0x60);
            cpu.mem_write(0x4032, 0x00);
        });
        assert_eq!(cpu.register_x, 0x01);
        assert_status(&cpu, 0);
        assert_eq!(cpu.program_counter, 0x8004);
        assert_eq!(cpu.stack_pointer, 0xFF);
        assert_eq!(cpu.mem_read_u16(0x01FE), 0x8003);
    }

    #[test]
    fn test_ldx(){
        let cpu = run(vec![0xA2, 0x05, 0x00], |cpu|{});
        assert_eq!(cpu.register_x, 0x05);
        assert_status(&cpu, 0);
    }

    #[test]
    fn test_ldy(){
        let cpu = run(vec![0xA0, 0x05, 0x00], |cpu|{});
        assert_eq!(cpu.register_y, 0x05);
        assert_status(&cpu, 0);
    }

    #[test]
    fn test_nop(){
        let cpu = run(vec![0xEA, 0x00], |cpu|{});
        assert_eq!(cpu.program_counter, 0x8002);
        assert_status(&cpu, 0);
    }

    #[test]
    fn test_pha(){
        let cpu = run(vec![0x48, 0x00], |cpu|{
            cpu.register_a = 0x01
        });
        assert_eq!(cpu.register_a, 0x01);
        assert_eq!(cpu.stack_pointer, 0xFE);
        assert_eq!(cpu.mem_read(0x01FF), 0x01);
        assert_status(&cpu, 0);
    }

        #[test]
    fn test_pla(){
        let cpu = run(vec![0x68, 0x00], |cpu|{
            cpu.mem_write(0x01FF, 0x01);
            cpu.stack_pointer = 0xFE;
        });
        assert_eq!(cpu.register_a, 0x01);
        assert_eq!(cpu.stack_pointer, 0xFF);
        assert_status(&cpu, 0);
    }

    #[test]
    fn test_pla_zero(){
        let cpu = run(vec![0x68, 0x00], |cpu|{
            cpu.mem_write(0x01FF, 0x00);
            cpu.stack_pointer = 0xFE;
        });
        assert_eq!(cpu.register_a, 0x00);
        assert_eq!(cpu.stack_pointer, 0xFF);
        assert_status(&cpu, FLAG_ZERO);
    }

    #[test]
    fn test_pha_pla(){
        let cpu = run(vec![0x48, 0xa9, 0x60, 0x68, 0x00], |cpu|{
            cpu.register_a = 0x80;
        });
        assert_eq!(cpu.register_a, 0x80);
        assert_eq!(cpu.stack_pointer, 0xFF);
        assert_status(&cpu, FLAG_NEGATIVE);
    }
    #[test]
    fn test_php(){
        let cpu = run(vec![0x08, 0x00], |cpu|{
            cpu.status = FLAG_NEGATIVE | FLAG_OVERFLOW;
        });
        assert_eq!(cpu.stack_pointer, 0xFE);
        assert_eq!(cpu.mem_read(0x01FF), cpu.status);
    }
        #[test]
    fn test_plp(){
        let cpu = run(vec![0x28, 0x00], |cpu|{
            cpu.mem_write(0x01FF, FLAG_CARRY | FLAG_INTERRUPT);
            cpu.stack_pointer = 0xFE;
        });
        assert_eq!(cpu.stack_pointer, 0xFF);
        assert_status(&cpu, FLAG_CARRY | FLAG_INTERRUPT);
    }

    #[test]
    fn test_php_plp(){
        let cpu = run(vec![0x08, 0xa9, 0xF0, 0x28, 0x00], |cpu|{
            cpu.register_a = 0xFF;
            cpu.status = FLAG_OVERFLOW | FLAG_BREAK;
        });
        assert_eq!(cpu.register_a, 0xF0);
        assert_eq!(cpu.stack_pointer, 0xFF);
        assert_status(&cpu, FLAG_OVERFLOW | FLAG_BREAK);
        assert_eq!(cpu.program_counter, 0x8005)
    }

        #[test]
    fn test_stx(){
        let cpu = run(vec![0x86, 0x10, 0x00], |cpu|{
            cpu.register_x = 0xBA;
        });
        assert_eq!(cpu.mem_read(0x10), 0xBA);
    }
    #[test]
    fn test_sty(){
        let cpu = run(vec![0x84, 0x10, 0x00], |cpu|{
            cpu.register_y = 0xBA;
        });
        assert_eq!(cpu.mem_read(0x10), 0xBA);
    }

    #[test]
    fn test_txa(){
        let cpu = run(vec![0x8A, 0x00], |cpu|{
            cpu.register_x = 10;
        });
        assert_eq!(cpu.register_a, 10);
    }

        #[test]
    fn test_tay(){
        let cpu = run(vec![0xA8, 0x00], |cpu|{
            cpu.register_a = 10;
        });
        assert_eq!(cpu.register_y, 10);
    }

    #[test]
    fn test_tya(){
        let cpu = run(vec![0x98, 0x00], |cpu|{
            cpu.register_y = 10;
        });
        assert_eq!(cpu.register_a, 10);
    }

    #[test]
    fn test_tsx(){
        let cpu = run(vec![0xBA, 0x00], |cpu|{});
        assert_eq!(cpu.register_x, 0xFF);
        assert_status(&cpu, FLAG_NEGATIVE);
    }

        #[test]
    fn test_tsx_sum(){
        let cpu = run(vec![0xBA, 0x00], |cpu|{
            cpu.stack_pointer = 0x75;
        });
        assert_eq!(cpu.register_x, 0x75);
        assert_status(&cpu, 0);
    }

    #[test]
    fn test_txs(){
        let cpu = run(vec![0x9A, 0x00], |cpu|{
            cpu.register_x = 0x80;
        });
        assert_eq!(cpu.stack_pointer, 0x80);
        assert_status(&cpu, 0);
    }
}


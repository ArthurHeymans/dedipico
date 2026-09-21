pub fn assemble_duplex(width: u8) -> pio::Program<32> {
    let side_set = pio::SideSet::new(false, 1, false);
    let mut assembler = pio::Assembler::<32>::new_with_side_set(side_set);
    let mut wrap_target = assembler.label();
    let mut wrap_source = assembler.label();

    assembler.bind(&mut wrap_target);
    // Five-cycle mode-0 bit cell tuned for 24 MHz. OUT drives MOSI during the
    // two-cycle low phase; two high-side NOPs delay MISO sampling until late in
    // the high phase, which avoids the 24 MHz bit-shift seen with earlier cells.
    assembler.out_with_delay_and_side_set(pio::OutDestination::PINS, width, 1, 0);
    assembler.nop_with_side_set(1);
    assembler.nop_with_side_set(1);
    assembler.r#in_with_side_set(pio::InSource::PINS, width, 1);
    assembler.bind(&mut wrap_source);

    assembler.assemble_with_wrap(wrap_source, wrap_target)
}

pub fn assemble_tx(width: u8) -> pio::Program<32> {
    let groups_per_byte = 8 / width;
    let side_set = pio::SideSet::new(false, 1, false);
    let mut assembler = pio::Assembler::<32>::new_with_side_set(side_set);
    let mut wrap_target = assembler.label();
    let mut wrap_source = assembler.label();
    let mut bitloop = assembler.label();

    assembler.bind(&mut wrap_target);
    assembler.pull_with_side_set(false, true, 0);
    assembler.set_with_side_set(pio::SetDestination::X, groups_per_byte - 1, 0);
    assembler.bind(&mut bitloop);
    assembler.out_with_delay_and_side_set(pio::OutDestination::PINS, width, 1, 0);
    assembler.jmp_with_delay_and_side_set(pio::JmpCondition::XDecNonZero, &mut bitloop, 2, 1);
    assembler.bind(&mut wrap_source);

    assembler.assemble_with_wrap(wrap_source, wrap_target)
}

pub fn assemble_rx(width: u8) -> pio::Program<32> {
    let side_set = pio::SideSet::new(false, 1, false);
    let mut assembler = pio::Assembler::<32>::new_with_side_set(side_set);
    let mut wrap_target = assembler.label();
    let mut wrap_source = assembler.label();

    assembler.bind(&mut wrap_target);
    // Read-only five-cycle bit cell matching the full-duplex sample phase. OUT
    // NULL consumes one lane-group from OSR so autopull supplies one byte worth
    // of clocks per TX FIFO word, while IN samples late in the SCK high phase.
    assembler.out_with_delay_and_side_set(pio::OutDestination::NULL, width, 1, 0);
    assembler.nop_with_side_set(1);
    assembler.nop_with_side_set(1);
    assembler.r#in_with_side_set(pio::InSource::PINS, width, 1);
    assembler.bind(&mut wrap_source);

    assembler.assemble_with_wrap(wrap_source, wrap_target)
}

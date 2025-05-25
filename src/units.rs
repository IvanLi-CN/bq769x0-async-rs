#![allow(non_upper_case_globals)]
#![allow(non_snake_case)]
#![allow(non_camel_case_types)]
#![allow(clippy::upper_case_acronyms)]

// 定义 ISQ! 宏，使用 i32 作为底层存储类型
ISQ!(
    uom::si,
    f32,
    (
        millimeter,
        kilogram,
        second,
        milliampere,
        kelvin,
        mole,
        candela
    )
);


#[cfg(test)]
mod tests {
    use super::{ElectricCurrent, ElectricPotential, ElectricalResistance};
    use approx::assert_relative_eq;
    use uom::si::{
        electric_current::milliampere,
        electric_potential::millivolt,
        electrical_resistance::{milliohm, ohm},
    };

    #[test]
    fn test_units() {
        let current = ElectricCurrent::new::<milliampere>(123.0);
        let potential = ElectricPotential::new::<millivolt>(4560.0);
        let resistance_20_ohm = ElectricalResistance::new::<ohm>(20.0);
        let resistance_10_milliohm = ElectricalResistance::new::<milliohm>(10.0);

        assert_relative_eq!(current.get::<milliampere>(), 123.0);
        assert_relative_eq!(potential.get::<millivolt>(), 4560.0);
        assert_relative_eq!(resistance_20_ohm.get::<ohm>(), 20.0);
        assert_relative_eq!(resistance_10_milliohm.get::<milliohm>(), 10.0);
    }
}
